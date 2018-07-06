'''
.. versionadded:: X.X.X
'''
from __future__ import division, print_function, unicode_literals
import datetime as dt
import threading
import time
import types

from logging_helpers import _L
import numpy as np
import si_prefix as si


def target_capacitance_event(self, channels, target_capacitance, count=3):
    '''
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
    threading.Event
        Event that is set once target capacitance has been reached.  The
        following attributes are added to the event as well:

         - ``cancel()``: method to cancel threshold request.
         - ``start``: time channels were actuated.
         - ``end``: time target capacitance was reached.
         - ``message``: the ``capacitance-exceeded`` DropBot event message.
    '''
    state_of_channels = np.zeros(self.number_of_channels, dtype=int)
    state_of_channels[channels] = 1
    self.state_of_channels = state_of_channels
    # XXX TODO Should probably wait until channels are reported as actuated
    # through `channels-updated` event.

    threshold_reached = threading.Event()

    # Add event to allow caller to cancel.
    threshold_reached.start = dt.datetime.now()

    def _on_done(message):
        threshold_reached.end = dt.datetime.now()
        threshold_reached.message = message
        # capacitance = message['new_value']
        self.signals.signal('capacitance-exceeded').disconnect(_on_done)
        threshold_reached.set()

    # Attach cancel method to event object.
    def _cancel(event):
        '''
        Disable DropBot ``capacitance-exceeded`` monitoring and disconnect
        event callback.
        '''
        self.update_state(target_capacitance=0)
        self.signals.signal('capacitance-exceeded').disconnect(_on_done)

    threshold_reached.cancel = _cancel

    # Connect to capacitance exceeded DropBot events, i.e., when specified
    # target capacitance has been exceeded.
    self.signals.signal('capacitance-exceeded').connect(_on_done, weak=False)
    # Set `target_capacitance` to non-zero value to enable DropBot
    # `capacitance-exceeded` event once target capacitance is reached and
    # sustained for `target_count` consecutive readings.
    self.update_state(target_capacitance=target_capacitance, target_count=count)

    return threshold_reached


def target_capacitance(self, channels, target_capacitance, timeout_s=1.5,
                       retries=10, **kwargs):
    '''
    Synchronous wrapper around :func:`target_capacitance_event`.

    If target capacitance not reached after specified timeout, retry with 98%
    of the target capacitance.

    Parameters
    ----------
    channels : list-like
        Channels to actuate.
    target_capacitance : float
        Target capacitance value.
    timeout_s : optional, float
        Timeout (in seconds) before retrying with reduced (98%) target
        capacitance.
    retries : optional, int
        Maximum number of retry attempts.

    Returns
    -------
    `OrderedDict`
        The ``capacitance-exceeded`` DropBot event message with the fields:

            - ``start``: start timestamp.
            - ``end``: end timestamp.
            - ``timeout_s``: number of seconds to wait before retrying.
            - ``retries``: number of retries before success.
            - ``new_value``: capacitance reached.
            - ``target``: target capacitance.
            - ``V_a``: actuation voltage.

    Raises
    ------
    RuntimeError
        If target capacitance was not reached after maximum number of retries.
    '''
    for i in range(retries):
        event = target_capacitance_event(self, channels, target_capacitance,
                                         **kwargs)
        if event.wait(timeout_s):
            target_capacitance = .98 * event.message['new_value']
            event.message['start'] = event.start
            event.message['end'] = event.end
            event.message['retries'] = i
            event.message['timeout_s'] = timeout_s
            return event.message
        else:
            target_capacitance *= .98
            _L().debug('Timed out on attempt %d.  Reduce target capacitance to'
                       ' %sF and retry.', i + 1,
                       si.si_format(target_capacitance, 2))
            event.cancel(None)
    else:
        raise RuntimeError('Timed out waiting for target capacitance')


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
