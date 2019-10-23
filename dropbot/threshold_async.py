'''
.. versionadded:: 1.74.0
'''
from __future__ import division, print_function, unicode_literals
import collections
import datetime as dt
import functools as ft
import itertools as it
import threading
import types
import uuid

from asyncio_helpers import ensure_event_loop
from logging_helpers import _L
import dropbot as db
import dropbot.proxy
import pandas as pd
import trollius as asyncio

from .move import MoveTimeout, test_steady_state, window


@asyncio.coroutine
def actuate_channels(aproxy, channels, timeout=None, allow_disabled=True):
    '''
    Parameters
    ----------
    channels : list
        List of channel numbers to actuate.
    timeout : float, optional
        Timeout in seconds.  If ``None``, block until completed.
    allow_disabled : bool, optional
        If ``False``, verify actuated channels match specified channels
        _exactly_.  Otherwise, ensure that all actuated channels belong to the
        specified set of channels, _even if_ not _all_ specified channels are
        actuated.  This supports attempting to actuate channels that are
        disabled.

    Returns
    -------
    list
        List of actuated channels.  If :data:`allow_disabled` is ``True``, the
        returned list of channels may differ from the specified list of
        channels.


    .. versionchanged:: 2.2.0
        Use :meth:`dropbot.proxy_py2.ProxyMixin.set_state_of_channels()` with
        ``append=False`` to set channel states.  This reduces the number of
        required serial command transactions.
    '''

    # Add async and sync API for channel actuation to driver (maybe partially to firmware?):
    #
    #  1. Request actuation of specified channels
    #  2. Verify channels have been actuated
    channels_updated = threading.Event()

    def _on_channels_updated(*args, **message):
        channels_updated.actuated = message.get('actuated')
        channels_updated.set()

    @asyncio.coroutine
    def _actuate():
        # Enable `channels-updated` DropBot signal.
        state = yield asyncio.From(aproxy.state)
        yield asyncio.From(aproxy.update_state(event_mask=state.event_mask |
                                               db.proxy.EVENT_CHANNELS_UPDATED
                                               | db.proxy.EVENT_ENABLE))
        # Request to be notified when the set of actuated channels changes.
        signals = aproxy.__client__.signals
        signal = signals.signal('channels-updated')
        signal.connect(_on_channels_updated)
        # Request actuation of the specified channels.
        yield asyncio\
            .From(aproxy.set_state_of_channels(pd.Series(1, index=channels),
                                               append=False))
        if not channels_updated.wait(timeout):
            raise RuntimeError('Timed out waiting for actuation')
        elif not hasattr(channels_updated, 'actuated'):
            raise RuntimeError('Actuation was cancelled.')
        elif not allow_disabled and (set(channels_updated.actuated) !=
                                        set(channels)):
            raise RuntimeError('Actuated channels `%s` do not match '
                                'expected channels `%s`' %
                                (channels_updated.actuated, channels))
        elif set(channels_updated.actuated) - set(channels):
            # Disabled channels are allowed.
            raise RuntimeError('Actuated channels `%s` are not included in'
                                ' expected channels `%s`' %
                                (channels_updated.actuated, channels))
        raise asyncio.Return(channels_updated.actuated)

    with aproxy.transaction_lock:
        result = yield asyncio.From(_actuate())
        raise asyncio.Return(result)


@asyncio.coroutine
def co_target_capacitance(aproxy, channels, target_capacitance, count=3,
                          **kwargs):
    '''
    XXX Coroutine XXX

    Actuate specified channels and wait until target capacitance is reached.

    Parameters
    ----------
    channels : list-like
        Channels to actuate.
    target_capacitance : float
        Target capacitance value.
    count : optional, int
        Number of required consecutive readings above target capacitance.

    Returns
    -------
    dict
        ``capacitance-exceeded`` DropBot event message with the following
        additional fields:
         - ``start``: time channels were actuated (`datetime.datetime`).
         - ``end``: time target capacitance was reached (`datetime.datetime`).
         - ``actuated_channels``: actuated channels (`list`).
    '''
    actuation_uuid1 = uuid.uuid1()
    signals = aproxy.__client__.signals

    with aproxy.transaction_lock:
        capacitance_messages = []

        def _on_capacitance(*args, **message):
            message['actuation_uuid1'] = actuation_uuid1
            message['actuated_channels'] = actuated_channels
            capacitance_messages.append(message)

        def _on_done(*args, **message):
            message['end'] = dt.datetime.now()
            message['start'] = start
            message['actuated_channels'] = actuated_channels
            threshold_reached.result = message
            signals.signal('capacitance-exceeded').disconnect(_on_done)
            loop.call_soon_threadsafe(threshold_reached.set)

        threshold_reached = asyncio.Event()
        loop = ensure_event_loop()

        # Perform actuation and wait until actuation has been applied.
        _L().debug('actuate channels: %s', channels)
        actuated_channels = yield asyncio\
            .From(actuate_channels(aproxy, channels, **kwargs))
        _L().debug('channels actuated: %s', actuated_channels)

        # Connect to capacitance exceeded DropBot events, i.e., when specified
        # target capacitance has been exceeded.
        signals.signal('capacitance-exceeded').connect(_on_done)

        # Record timestamp where actuation has been verified as applied.
        start = dt.datetime.now()

        # Connect to `capacitance-updated` signal to record capacitance values
        # measured during actuation.
        signals.signal('capacitance-updated').connect(_on_capacitance)

        # Set `target_capacitance` to non-zero value to enable DropBot
        # `capacitance-exceeded` event once target capacitance is reached and
        # sustained for `target_count` consecutive readings.
        capacitance = yield asyncio\
            .From(aproxy.measure_capacitance())
        _L().debug('capacitance: %.2g', capacitance)
        _L().debug('wait for capacitance to reach: %.2g', target_capacitance)
        yield asyncio\
            .From(aproxy.update_state(target_capacitance=target_capacitance,
                                      target_count=count))

        yield asyncio.From(threshold_reached.wait())
        # Attach list of capacitance update messages recorded during
        # actuation to result.
        threshold_reached.result['capacitance_updates'] = \
            capacitance_messages
        raise asyncio.Return(threshold_reached.result)


def execute_actuation(self, chip_info_, specific_capacitance, channels,
                      duration_s=1.5, volume_threshold=None, **kwargs):
    '''
    XXX Coroutine XXX

    High-level wrapper around threshold event to:

     - Compute target capacitance based on electrode geometries, specific
       capacitance, and volume threshold.
     - Support case where no volume threshold is specified, i.e., simply delay
       for specified duration.

    Parameters
    ----------
    chip_info_ : dict
        Chip information, as returned by :func:`dropbot.chip.chip_info`.
    specific_capacitance : float
        Specific capacitance, i.e., capacitance per unit area.
    channels : `list`-like
        Numbers of channels to actuate or electrode IDs (e.g., `"electrode001",
        ...`).
    duration_s : float
        Time to wait for execution.

    Returns
    -------
    dict
        Actuation result with the fields:

        - ``start``: timestamp after channels actuated (datetime.datetime).
        - ``end``: timestamp after operation is completed (datetime.datetime).
        - ``actuated_area``: actuated electrode area (float).
        - ``actuated_channels``: actuated channel numbers (list).

        If :data:`volume_threshold` was specified, _at least_ the following
        ``capacitance-exceeded`` DropBot event message fields are also
        included:

        - ``new_value``: capacitance reached.
        - ``target``: target capacitance.
        - ``V_a``: actuation voltage.

    Raises
    ------
    RuntimeError
        If target capacitance was not reached after specified timeout.
    '''
    if isinstance(channels[0], types.IntType):
        # Channels were specified.
        electrodes = chip_info_['channel_electrodes'].loc[channels]
    else:
        # Assume electrode IDs (e.g., `"electrode001", ...`) were specified.
        electrodes = channels
        channels = chip_info_['electrode_channels'].loc[electrodes].astype(int)

    def _actuated_result_info(actuated_channels):
        actuated_electrodes = (chip_info_['channel_electrodes']
                               .loc[actuated_channels])
        return {'actuated_area': (chip_info_['electrode_shapes']['area']
                                  .loc[actuated_electrodes]).sum(),
                'actuated_electrodes': actuated_electrodes,
                'actuated_channels': actuated_channels}

    with self.transaction_lock:
        if not volume_threshold:
            # ## Case 1: no volume threshold specified.
            capacitance_messages = []
            result = {}

            def _on_capacitance_updated(self, message):
                message['actuated_channels'] = self.actuated_channels
                message['actuated_area'] = self.actuated_area
                capacitance_messages.append(message)

            #  1. Set control board state of channels according to requested
            #     actuation states; and
            #  2. Wait for channels to be actuated.
            actuated_channels = actuate_channels(self, channels,
                                                 timeout=duration_s)
            result['start'] = dt.datetime.now()
            #  3. Connect to `capacitance-updated` signal to record capacitance
            #     values measured during the step.
            (self.signals.signal('capacitance-updated')
                .connect(_on_capacitance_updated))
            #  4. Delay for specified duration.
            try:
                yield asyncio.From(asyncio.sleep(duration_s))
                result['end'] = dt.datetime.now()
                result.update(_actuated_result_info(actuated_channels))
            finally:
                (self.signals.signal('capacitance-updated')
                 .disconnect(_on_capacitance_updated))
        else:
            # ## Case 2: volume threshold specified.
            #
            # A volume threshold has been set for this step.

            # Calculate target capacitance based on actuated area.
            #
            # Note: `app_values['c_liquid']` represents a *specific
            # capacitance*, i.e., has units of $F/mm^2$.
            result = _actuated_result_info(channels)

            target_capacitance = (volume_threshold * result['actuated_area'] *
                                  specific_capacitance)

            # Wait for target capacitance to be reached in background thread,
            # timing out if the specified duration is exceeded.
            co_future = co_target_capacitance(self, channels,
                                              target_capacitance,
                                              allow_disabled=False,
                                              timeout=duration_s, **kwargs)
            try:
                dropbot_event = yield asyncio.From(asyncio
                                                   .wait_for(co_future,
                                                             duration_s))
                capacitance_messages = dropbot_event['capacitance_updates']
                result.update(dropbot_event)
            except asyncio.TimeoutError:
                raise RuntimeError('Timed out waiting for target capacitance.')

            # Add actuated area to capacitance update messages.
            for capacitance_i in capacitance_messages:
                capacitance_i['acuated_area'] = result['actuated_area']

        raise asyncio.Return(result)



@asyncio.coroutine
def wait_on_capacitance(aproxy, callback):
    '''Return once callback returns `True`.

    Parameters
    ----------
    callback
        Callback function accepting a list of ``capacitance-updated`` messages
        as only argument.

    Returns
    -------
    list
        List of DropBot ``capacitance-updated`` messages containing the
        following keys::

         - ``event``: ``"capacitance-updated"``
         - ``new_value``: capacitance value in Farads
         - ``time_us``: DropBot microsecond 32-bit counter
         - ``n_samples``: number of samples used for RMS measurement
         - ``V_a``: measured actuation voltage during capacitance reading
    '''
    move_done = asyncio.Event()
    loop = asyncio.get_event_loop()

    messages = []

    def _on_capacitance(*args, **message):
        # message.keys == ['event', 'new_value', 'time_us', 'n_samples', 'V_a']
        # Added by `co_target_capacitance()`: 'actuation_uuid1', 'actuated_channels'
        try:
            messages.append(message)
            if callback(messages):
                loop.call_soon_threadsafe(move_done.set)
        except Exception:
            _L().debug('capacitance event error.', exc_info=True)
            return

    aproxy.__client__.signals.signal('capacitance-updated').connect(_on_capacitance)
    yield asyncio.From(move_done.wait())
    raise asyncio.Return(messages)


@asyncio.coroutine
def move_liquid(aproxy, route, trail_length=1, wrapper=None,
                **kwargs):
    '''Move liquid along specified route (i.e., list of channels).

    Parameters
    ----------
    route : list[int]
        Ordered sequence of channels to move along.
    min_duration : float, optional
        Minimum time to apply each actuation.
    trail_length : int, optional
        Number of electrodes to actuate at the same time along route.

        .. versionadded:: 2.3.0
    wrapper : callable, optional
        Function to wrap around calls to `actuate()`.

        Useful, for example, to apply an actuation timeout using
        `asyncio.wait_for()`.

    Returns
    -------
    list[dict]
        List of dictionaries, each corresponding to a single channel actuation
        with the following keys::

         - ``channels``: actuated channels
         - ``messages``: ``capacitance-updated`` messages received during the
           actuation


    .. versionchanged:: 2.3.0
        Add `trail_length` keyword argument.
    '''
    if wrapper is None:
        def wrapper(task):
            return task

    messages_ = []

    try:
        for route_i in window(route, trail_length + 1):
            print('\r%-50s' % ('Wait for steady state: %s' % list(route_i)),
                  end='')
            messages = yield asyncio\
                .From(wrapper(actuate(aproxy, route_i,
                                      ft.partial(test_steady_state,
                                                 **kwargs))))
            messages_.append({'channels': tuple(route_i),
                              'messages': messages})
            head_channels_i = list(route_i[-trail_length:])
            print('\r%-50s' % ('Wait for steady state: %s' % head_channels_i),
                  end='')
            messages = yield asyncio\
                .From(wrapper(actuate(aproxy, head_channels_i,
                                      ft.partial(test_steady_state,
                                                 **kwargs))))
            messages_.append({'channels': tuple(head_channels_i),
                              'messages': messages})
    except (asyncio.CancelledError, asyncio.TimeoutError):
        raise MoveTimeout(route, route_i)

    raise asyncio.Return(messages_)


@asyncio.coroutine
def actuate(proxy, channels, callback):
    '''Actuate channels and wait for callback to return `True`.

    Parameters
    ----------
    channels : list
        List of channel numbers to actuate.
    callback
        Callback function accepting a list of ``capacitance-updated`` messages
        as only argument.
    '''
    # Actuate channels.
    yield asyncio.From(actuate_channels(proxy, channels))
    # Wait for callback.
    result = yield asyncio.From(wait_on_capacitance(proxy, callback))
    raise asyncio.Return(result)


# -----------------------------------------------------


class TransferTimeout(asyncio.TimeoutError):
    '''Exception occurred while performing move along route.

    Attributes
    ----------
    route : list[int]
        List of channels along route.
    route_i : tuple[int, int]
        Source and target channel of failed move.
    '''
    def __init__(self, channels, *args, **kwargs):
        self.channels = channels
        super(TransferTimeout, self).__init__(*args, **kwargs)


def test_steady_state_(messages, std_error=.02, min_duration=.3):
    '''Callback to check for capacitance steady state.

    Parameters can be set using `functools.partial()`.

    Parameters
    ----------
    messages : list
        List of DropBot ``capacitance-updated`` messages containing the
        following keys::

         - ``event``: ``"capacitance-updated"``
         - ``new_value``: capacitance value in Farads
         - ``time_us``: DropBot microsecond 32-bit counter
         - ``n_samples``: number of samples used for RMS measurement
         - ``V_a``: measured actuation voltage during capacitance reading
    std_error : float, optional
        Ratio of the standard deviation (from most recent 100 samples) to the
        median under which steady state is considered to be reached _(default:
        0.02, i.e., 2% of the median)_.
    min_duration : float, optional
        Minimum time (in seconds) since first capacitance update before
        considering steady state as reached (default: 0.3).
    threshold : float, optional
        Minimum median capacitance in Farads (from most recent 100 samples)
        before considering steady state as reached.
    '''
    df = pd.DataFrame(messages[-100:])
    df['time'] = df.time_us * 1e-6
    df['time'] -= df.time.iloc[0]
    df.set_index('time', inplace=True)
    if (df.index.values[-1] - df.index.values[0]) < min_duration:
        return False
    start = df.index.values[-1] - min_duration
    d = df.new_value.loc[start:].describe()
    result = (d['std'] / d['50%']) < std_error
    return result


@asyncio.coroutine
def transfer_liquid(aproxy, channels, wrapper=None, **kwargs):
    '''
    Transfer liquid from tail n-1 channels to head n-1 channels.

        xxxx... -> ...xxxx

    where ``x`` denotes liquid and ``.`` denotes an empty electrode.
    '''
    if wrapper is None:
        def wrapper(task):
            return task

    state = yield asyncio.From(aproxy.state)
    if state.capacitance_update_interval_ms == 0:
        raise ValueError('Capacitance update interval must be non-zero.')

    messages_ = []

    try:
        route_i = list(it.chain(*(c if isinstance(c, collections.Sequence)
                                  else [c] for c in channels)))
        print('\r%-50s' % ('Wait for steady state: %s' % list(route_i)),
              end='')
        messages = yield asyncio\
            .From(wrapper(db.threshold_async
                          .actuate(aproxy, route_i,
                                   ft.partial(test_steady_state_, **kwargs))))
        messages_.append({'channels': tuple(route_i),
                          'messages': messages})
        head_channels_i = list(channels[1:])
        print('\r%-50s' % ('Wait for steady state: %s' % head_channels_i),
              end='')
        route_i = list(it.chain(*(c if isinstance(c, collections.Sequence)
                                  else [c] for c in head_channels_i)))
        messages = yield asyncio\
            .From(wrapper(db.threshold_async
                          .actuate(aproxy, route_i,
                                  ft.partial(test_steady_state_,
                                             **kwargs))))
        messages_.append({'channels': tuple(route_i),
                          'messages': messages})
    except (asyncio.CancelledError, asyncio.TimeoutError):
        raise TransferTimeout(channels)

    raise asyncio.Return(messages_)
