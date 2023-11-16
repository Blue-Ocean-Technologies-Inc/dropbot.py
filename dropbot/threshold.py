# coding: utf-8
import datetime as dt
import threading
import uuid

from dropbot.proxy import EVENT_ENABLE, EVENT_CHANNELS_UPDATED
import pandas as pd
import asyncio


def new_file_event_loop() -> asyncio.AbstractEventLoop:
    """
    Create an asyncio event loop compatible with file IO events, e.g., serial
    device events.

    Returns
    -------
    asyncio.ProactorEventLoop or asyncio.SelectorEventLoop
    """
    return asyncio.new_event_loop()


def ensure_event_loop() -> asyncio.AbstractEventLoop:
    """
    Ensure that an asyncio event loop has been bound to the local thread
    context.

    Returns
    -------
    asyncio.ProactorEventLoop or asyncio.SelectorEventLoop
    """
    try:
        loop = asyncio.get_event_loop()
    except RuntimeError as e:
        if 'There is no current event loop' in str(e):
            loop = new_file_event_loop()
            asyncio.set_event_loop(loop)
        else:
            raise
    return loop


def actuate_channels(proxy_, channels, timeout=None, allow_disabled=True):
    """
    Parameters
    ----------
    proxy_: dropbot.SerialProxy
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


    .. versionchanged:: 1.71.0
        Use :meth:`dropbot.proxy_py2.ProxyMixin.set_state_of_channels()` with
        ``append=False`` to set channel states.  This reduces the number of
        required serial command transactions.
    """

    # Add async and sync API for channel actuation to driver (maybe partially to firmware?):
    #
    #  1. Request actuation of specified channels
    #  2. Verify channels have been actuated
    channels_updated = threading.Event()

    def _on_channels_updated(message):
        channels_updated.actuated = message.get('actuated')
        channels_updated.set()

    def _actuate():
        # Enable `channels-updated` DropBot signal.
        proxy_.update_state(event_mask=proxy_.state.event_mask | EVENT_CHANNELS_UPDATED | EVENT_ENABLE)
        # Request to be notified when the set of actuated channels changes.
        signal = proxy_.signals.signal('channels-updated')
        signal.connect(_on_channels_updated)
        try:
            # Request actuation of the specified channels.
            proxy_.set_state_of_channels(pd.Series(1, index=channels), append=False)
            if not channels_updated.wait(timeout):
                raise RuntimeError('Timed out waiting for actuation')
            elif not hasattr(channels_updated, 'actuated'):
                raise RuntimeError('Actuation was cancelled.')
            elif not allow_disabled and (set(channels_updated.actuated) != set(channels)):
                raise RuntimeError('Actuated channels `%s` do not match '
                                   'expected channels `%s`' %
                                   (channels_updated.actuated, channels))
            elif set(channels_updated.actuated) - set(channels):
                # Disabled channels are allowed.
                raise RuntimeError(f'Actuated channels `{channels_updated.actuated}` are not included in'
                                   f' expected channels `{channels}`')
        finally:
            signal.disconnect(_on_channels_updated)
        return channels_updated.actuated

    with proxy_.transaction_lock:
        return _actuate()


async def co_target_capacitance(proxy_, channels, target_capacitance, count=3, **kwargs):
    """
    XXX Coroutine XXX

    Actuate specified channels and wait until target capacitance is reached.

    Parameters
    ----------
    proxy_: dropbot.SerialProxy
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
    """
    actuation_uuid1 = uuid.uuid1()
    with proxy_.transaction_lock:
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
            proxy_.signals.signal('capacitance-exceeded').disconnect(_on_done)
            loop.call_soon_threadsafe(threshold_reached.set)

        threshold_reached = asyncio.Event()
        loop = ensure_event_loop()

        # Perform actuation and wait until actuation has been applied.
        actuated_channels = actuate_channels(proxy_, channels, **kwargs)

        # Connect to capacitance exceeded DropBot events, i.e., when specified
        # target capacitance has been exceeded.
        proxy_.signals.signal('capacitance-exceeded').connect(_on_done)

        # Record timestamp where actuation has been verified as applied.
        start = dt.datetime.now()

        # Connect to `capacitance-updated` signal to record capacitance values
        # measured during actuation.
        proxy_.signals.signal('capacitance-updated').connect(_on_capacitance)

        # Set `target_capacitance` to non-zero value to enable DropBot
        # `capacitance-exceeded` event once target capacitance is reached and
        # sustained for `target_count` consecutive readings.
        proxy_.update_state(target_capacitance=target_capacitance, target_count=count)

        try:
            await threshold_reached.wait()
            # Attach a list of capacitance update messages recorded during actuation to result.
            threshold_reached.result['capacitance_updates'] = capacitance_messages
            return threshold_reached.result
        finally:
            proxy_.signals.signal('capacitance-updated').disconnect(_on_capacitance)
            proxy_.signals.signal('capacitance-exceeded').disconnect(_on_done)


async def execute_actuation(proxy_, chip_info_, specific_capacitance, channels,
                            duration_s=1.5, volume_threshold=None, **kwargs):
    """
    XXX Coroutine XXX

    High-level wrapper around threshold event to:

     - Compute target capacitance based on electrode geometries, specific
       capacitance, and volume threshold.
     - Support case where no volume threshold is specified, i.e., simply delay
       for specified duration.

    Parameters
    ----------
    proxy_: dropbot.SerialProxy
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
    """

    if isinstance(channels[0], int):
        # Channels were specified.
        electrodes = chip_info_['channel_electrodes'].loc[channels]
    else:
        # Assume electrode IDs (e.g., `"electrode001", ...`) were specified.
        electrodes = channels
        channels = chip_info_['electrode_channels'].loc[electrodes].astype(int)

    def _actuated_result_info(actuated_channels):
        actuated_electrodes = (chip_info_['channel_electrodes'].loc[actuated_channels])
        return {'actuated_area': (chip_info_['electrode_shapes']['area'].loc[actuated_electrodes]).sum(),
                'actuated_electrodes': actuated_electrodes,
                'actuated_channels': actuated_channels}

    with proxy_.transaction_lock:
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
            actuated_channels = actuate_channels(proxy_, channels, timeout=duration_s)
            result['start'] = dt.datetime.now()
            #  3. Connect to `capacitance-updated` signal to record capacitance
            #     values measured during the step.
            proxy_.signals.signal('capacitance-updated').connect(_on_capacitance_updated)
            #  4. Delay for specified duration.
            try:
                await asyncio.sleep(duration_s)
                result['end'] = dt.datetime.now()
                result.update(_actuated_result_info(actuated_channels))
            finally:
                proxy_.signals.signal('capacitance-updated').disconnect(_on_capacitance_updated)
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
            co_future = co_target_capacitance(proxy_, channels, target_capacitance,
                                              allow_disabled=False, timeout=duration_s, **kwargs)
            try:
                dropbot_event = await asyncio.wait_for(co_future, timeout=duration_s)
                capacitance_messages = dropbot_event['capacitance_updates']
                result.update(dropbot_event)
            except asyncio.TimeoutError:
                raise RuntimeError('Timed out waiting for target capacitance.')

            # Add actuated area to capacitance update messages.
            for capacitance_i in capacitance_messages:
                capacitance_i['acuated_area'] = result['actuated_area']
        return result


if __name__ == '__main__':
    import dropbot
    from pprint import pprint

    db = dropbot.SerialProxy()
    chip_info = {'channel_electrodes': pd.Series([1, 2, 3, 4, 5, 6, 7, 8]),
                 'electrode_channels': pd.Series([1, 2, 3, 4, 5, 6, 7, 8]),
                 'electrode_shapes': {'area': pd.Series([4, 4, 4, 4, 4, 4, 4, 4])}
                 }


    async def tester(*args, **kwargs):
        result = await execute_actuation(*args, **kwargs)
        pprint(result)


    loop = asyncio.get_event_loop()
    task = loop.create_task(tester(db, chip_info, 1e-10, [1, 2, 3]))
    loop.run_until_complete(task)
