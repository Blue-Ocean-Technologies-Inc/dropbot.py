'''
.. versionadded:: X.X.X
'''
from __future__ import division, print_function, unicode_literals
import datetime as dt
import threading
import types
import uuid

import base_node_rpc as bnr
import base_node_rpc.async
import dropbot as db
import dropbot.proxy
import pandas as pd
import trollius as asyncio


def actuate_channels(self, channels, timeout=None, allow_disabled=True):
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
    '''

    # Add async and sync API for channel actuation to driver (maybe partially to firmware?):
    #
    #  1. Request actuation of specified channels
    #  2. Verify channels have been actuated
    channels_updated = threading.Event()

    def _on_channels_updated(message):
        channels_updated.actuated = message.get('actuated')
        channels_updated.set()

    def _actuate():
        self.turn_off_all_channels()
        # Enable `channels-updated` DropBot signal.
        self.update_state(event_mask=self.state.event_mask |
                          db.proxy.EVENT_CHANNELS_UPDATED |
                          db.proxy.EVENT_ENABLE)
        # Request to be notified when the set of actuated channels changes.
        signal = self.signals.signal('channels-updated')
        signal.connect(_on_channels_updated)
        try:
            # Request actuation of the specified channels.
            self.state_of_channels = pd.Series(1, index=channels)
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
        finally:
            signal.disconnect(_on_channels_updated)
        return channels_updated.actuated

    with self.transaction_lock:
        return _actuate()


@asyncio.coroutine
def co_target_capacitance(self, channels, target_capacitance, count=3,
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
    with self.transaction_lock:
        capacitance_messages = []

        def _on_capacitance(message):
            message['actuation_uuid1'] = actuation_uuid1
            message['actuated_channels'] = actuated_channels
            capacitance_messages.append(message)

        def _on_done(message):
            message['end'] = dt.datetime.now()
            message['start'] = start
            message['actuated_channels'] = actuated_channels
            threshold_reached.result = message
            self.signals.signal('capacitance-exceeded').disconnect(_on_done)
            loop.call_soon_threadsafe(threshold_reached.set)

        threshold_reached = asyncio.Event()
        loop = bnr.async.ensure_event_loop()

        # Perform actuation and wait until actuation has been applied.
        actuated_channels = actuate_channels(self, channels, **kwargs)

        # Connect to capacitance exceeded DropBot events, i.e., when specified
        # target capacitance has been exceeded.
        self.signals.signal('capacitance-exceeded').connect(_on_done)

        # Record timestamp where actuation has been verified as applied.
        start = dt.datetime.now()

        # Connect to `capacitance-updated` signal to record capacitance values
        # measured during actuation.
        self.signals.signal('capacitance-updated').connect(_on_capacitance)

        # Set `target_capacitance` to non-zero value to enable DropBot
        # `capacitance-exceeded` event once target capacitance is reached and
        # sustained for `target_count` consecutive readings.
        self.update_state(target_capacitance=target_capacitance,
                          target_count=count)

        try:
            yield asyncio.From(threshold_reached.wait())
            # Attach list of capacitance update messages recorded during
            # actuation to result.
            threshold_reached.result['capacitance_updates'] = \
                capacitance_messages
            raise asyncio.Return(threshold_reached.result)
        finally:
            self.signals.signal('capacitance-updated')\
                .disconnect(_on_capacitance)
            self.signals.signal('capacitance-exceeded').disconnect(_on_done)


def execute_actuation(self, chip_info_, specific_capacitance, channels,
                      duration_s=1.5, volume_threshold=None, **kwargs):
    '''
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
    `OrderedDict` or `None`
        If a :data:`volume_threshold` was specified, return
        ``capacitance-exceeded`` DropBot event message with the fields:

            - ``start``: start timestamp.
            - ``end``: end timestamp.
            - ``timeout_s``: number of seconds to wait before retrying.
            - ``retries``: number of retries before success.
            - ``new_value``: capacitance reached.
            - ``target``: target capacitance.
            - ``V_a``: actuation voltage.

        Otherwise, return `None`.

    Raises
    ------
    RuntimeError
        If target capacitance was not reached after maximum number of retries.
    '''
    if len(channels) < 1:
        return None
    elif isinstance(channels[0], types.IntType):
        # Channels were specified.
        electrodes = chip_info_['channel_electrodes'].loc[channels]
    else:
        # Assume electrode IDs (e.g., `"electrode001", ...`) were specified.
        electrodes = channels
        channels = chip_info_['electrode_channels'].loc[electrodes].astype(int)

    area = chip_info_['electrode_shapes']['area'].loc[electrodes].sum()

    if volume_threshold is not None and volume_threshold > 0:
        target_capacitance_ = volume_threshold * specific_capacitance * area
        return target_capacitance(self, channels, target_capacitance_,
                                  timeout_s=duration_s, **kwargs)
    else:
        state_of_channels = np.zeros_like(self.state_of_channels, dtype=int)
        state_of_channels[channels] = 1
        self.state_of_channels = state_of_channels
        time.sleep(duration_s)
        return None
