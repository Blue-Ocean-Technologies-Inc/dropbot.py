'''
.. versionadded:: 1.67

.. versionchanged:: 1.68
    If 12V power is not detected, prompt to either a) ignore and connect
    anyway; or b) skip the DropBot.
'''
import time

import base_node_rpc as bnr

from pprint import pprint
from logging_helpers import _L
from dropbot.proxy import SerialProxy, NoPower, EVENT_CHANNELS_UPDATED, EVENT_SHORTS_DETECTED, EVENT_ENABLE, __version__
from dropbot.bin import upload

DROPBOT_SIGNAL_NAMES = ('halted', 'output_enabled',
                        'output_disabled', 'capacitance-updated',
                        'channels-updated', 'shorts-detected')

dropbot = None


async def monitor(signals_: dict):
    """
    Establish and maintain a DropBot connection.

    XXX Coroutine XXX

    If no DropBot is available or if the connection is lost, wait until a
    DropBot is detected on one of the available serial ports and (re)connect.

    DropBot signals are forwarded to the supplied :data:`signals` namespace,
    avoiding the need to manually connect signals after DropBot is
    (re)connected.

    DropBot connection is automatically closed when coroutine exits, e.g., when
    cancelled.

    Notes
    -----
    On Windows **MUST** be run using a `asyncio.ProactorEventLoop`.

    Parameters
    ----------
    signals_: dict of aiosignals
        Namespace for DropBot monitor signals.

    Sends
    -----
    connected
        When DropBot connection is established, with kwargs::
        - ``dropbot``: reference to DropBot proxy instance.
    disconnected
        When DropBot connection is lost.
    chip-inserted
        When DropBot detects a chip has been inserted.  Also sent upon
        connection to DropBot if a chip is present.
    chip-removed
        When DropBot detects a chip has been removed.  Also sent upon
        connection to DropBot if a chip is **not** present.

    Example
    -------

    >>> import blinker
    >>>
    >>> signals = blinker.Namespace()
    >>>
    >>> @asyncio.coroutine
    >>> def dump(*args, **kwargs):
    >>>     print('args=`%s`, kwargs=`%s`' % (args, kwargs))
    >>>
    >>> signals_.signal('chip-inserted').connect(dump, weak=False)
    >>> loop = asyncio.ProactorEventLoop()
    >>> asyncio.set_event_loop(loop)
    >>> task = loop.create_task(db.monitor.dropbot_monitor(signals_))
    >>> # Stop monitor after 15 seconds.
    >>> loop.call_later(15, task.cancel)
    >>> loop.run_until_complete(task)


    .. versionchanged:: 1.67.1
        Upon connection, send `'chip-inserted'` if chip is inserted or send
        `'chip-removed'` if no chip is inserted.

    .. versionchanged:: 1.68
        Send `'no-power'` signal if 12V power supply not connected.  Receivers
        may return `'ignore'` to attempt to connect anyway.
    """
    loop = asyncio.get_event_loop()
    global dropbot
    dropbot = None

    async def co_flash_firmware():
        if dropbot is not None:
            dropbot.terminate()
        upload.upload()
        time.sleep(.5)

    def flash_firmware(dropbot_):
        loop.create_task(co_flash_firmware())

    register_signal('flash-firmware',
                    lambda *args: loop.call_soon_threadsafe(flash_firmware, dropbot))

    # def reboot(dropbot_):
    #     if dropbot_ is not None:
    #         dropbot_._reboot()

    # register_signal('reboot',
    #                 lambda *args: loop.call_soon_threadsafe(reboot, dropbot))

    def reconnect(dropbot_):
        if dropbot_ is not None:
            dropbot_.terminate()

    register_signal('reconnect',
                    lambda *args: loop.call_soon_threadsafe(reconnect, dropbot))

    try:
        while True:
            # Multiple DropBot devices were found.
            # Get list of available devices.
            df_comports = await bnr.ser_async._available_devices(timeout=.1)

            if 'device_name' not in df_comports or not df_comports.shape[0]:
                await asyncio.sleep(.1)
                continue

            # Automatically select DropBot with highest version, with ties
            # going to the lowest port name (i.e., `COM1` before `COM2`).
            df_comports = df_comports.loc[df_comports.device_name == 'dropbot'].copy()
            df_comports.reset_index(inplace=True)

            df_comports.sort_values(['device_version', 'port'], ascending=[False, True], inplace=True)
            df_comports.set_index('port', inplace=True)
            if not len(df_comports):
                continue

            port = df_comports.index[0]

            async def _attempt_connect(**kwargs):
                ignore = kwargs.pop('ignore', [])
                try:
                    # Attempt to connect to automatically selected port.
                    dropbot_ = SerialProxy(port=port, ignore=ignore, **kwargs)
                    return dropbot_
                except NoPower as exception:
                    # No 12V power supply detected on DropBot.
                    _L().debug('No 12V power supply detected.')
                    response_future = asyncio.Future()
                    await signals_['no-power'].send('keep_alive', future=response_future)
                    response = response_future.result()

                    if response == 'ignore':
                        ignore.append(NoPower)
                    else:
                        raise exception

                except bnr.proxy.DeviceVersionMismatch as exception:
                    # Firmware version does not match driver version.
                    _L().debug(f"Driver version (`{__version__}`) does not match firmware "
                               f"version (`{exception.device_version}`)")

                    response_future = asyncio.Future()
                    await signals_['version-mismatch'].send('keep_alive', driver_version=__version__,
                                                            firmware_version=exception.device_version,
                                                            future=response_future)
                    response = response_future.result()

                    update = False

                    if response == 'ignore':
                        ignore.append(bnr.proxy.DeviceVersionMismatch)
                    elif response == 'update':
                        update = True
                    else:
                        raise

                    if update:
                        # Flash firmware and retry connection.
                        _L().info('Flash firmware and retry connection.')
                        await co_flash_firmware()

                dropbot_ = await _attempt_connect(ignore=ignore, **kwargs)
                return dropbot_

            try:
                dropbot = await _attempt_connect()
                pass
            except bnr.proxy.DeviceNotFound:
                raise 'Could not find device'
            except asyncio.CancelledError:
                raise
            except Exception:
                _L().debug('Error connecting to DropBot.', exc_info=True)
                await asyncio.sleep(.1)
                continue

            def co_connect(name):
                def _wrapped(sender, **message):
                    async def co_callback(message_):
                        signal = signals_.get(name)
                        if signal:
                            await signal.send('keep_alive', **message_)
                        # await asyncio.gather(*(listener[1] for listener in listeners))

                    return loop.call_soon_threadsafe(loop.create_task, co_callback(sender, **message))

                return _wrapped

            for name_j in DROPBOT_SIGNAL_NAMES:
                dropbot.signals.signal(name_j).connect(co_connect(name_j), weak=False)

            dropbot.signals.signal('output_enabled').connect(co_connect('chip-inserted'), weak=False)
            dropbot.signals.signal('output_disabled').connect(co_connect('chip-removed'), weak=False)

            await signals_['connected'].send('keep_alive', dropbot=dropbot)

            OUTPUT_ENABLE_PIN = 22
            # Chip may have been inserted before connecting, so `chip-inserted`
            # event may have been missed.
            # Explicitly check if chip is inserted by reading **active low**
            # `OUTPUT_ENABLE_PIN`.
            if dropbot.digital_read(OUTPUT_ENABLE_PIN):
                co_connect('chip-removed')({})
            else:
                co_connect('chip-inserted')({})

            disconnected = asyncio.Event()

            dropbot.signals.signal('disconnected').connect(lambda *args:
                                                           loop.call_soon_threadsafe(disconnected.set),
                                                           weak=False)

            await disconnected.wait()

            dropbot.terminate()

            responses = signals_['disconnected'].send('keep_alive')
            await asyncio.gather(*(r[1] for r in responses))
    finally:
        signals_['closed'].send('keep_alive')
        if dropbot is not None:
            dropbot.terminate()


if __name__ == '__main__':
    import logging
    import asyncio
    import aiosignal

    import functools as ft

    from debounce import DebounceAsync

    logging.basicConfig(level=logging.DEBUG)

    connected = asyncio.Event()


    async def on_connected(sender, **message):
        connected.dropbot = message['dropbot']
        _L().info(f'sender=`{sender}`')
        map(_L().info, str(connected.dropbot.properties).splitlines())
        connected.dropbot.update_state(capacitance_update_interval_ms=10,
                                       event_mask=EVENT_CHANNELS_UPDATED |
                                                  EVENT_SHORTS_DETECTED |
                                                  EVENT_ENABLE)
        connected.set()


    async def on_disconnected(*args, **kwargs):
        global dropbot
        dropbot = None
        _L().info(f'args=`{args}`, kwargs=`{kwargs}`')


    async def on_halted(*args, **kwargs):
        _L().info(f'args=`{args}`, kwargs=`{kwargs}`')


    def dump(name, *args, **kwargs):
        pprint(f'\r[{name}] args=`{args}`, kwargs=`{kwargs}`'),


    async def co_dump(*args, **kwargs):
        future = kwargs.get('future')
        if future:
            future.set_result(dump(*args, **kwargs))
        else:
            return dump(*args, **kwargs)


    async def on_version_mismatch(*args, **kwargs):
        _L().info(f'args=`{args}`, kwargs=`{kwargs}`')
        message = (f"Driver version `kwargs['driver_version']` does not match "
                   f"firmware `kwargs['firmware_version']` version.")
        while True:
            response = input(f"{message} [I]gnore/[u]pdate/[s]kip: ")
            if not response:
                # Default response is `ignore` and try to connect anyway.
                response = 'ignore'

            for action in ('ignore', 'update', 'skip'):
                if action.startswith(response.lower()):
                    response = action
                    break
            else:
                print(f'Invalid response: `{response}`')
                response = None

            if response is not None:
                break
        if response == 'skip':
            raise IOError(message)
        future = kwargs.get('future')
        if future:
            future.set_result(response)
        return response


    async def on_no_power(*args, **kwargs):
        while True:
            response = input('No 12V power supply detected. '
                             '[I]gnore/[s]kip: ')
            if not response:
                # Default response is `ignore` and try to connect anyway.
                response = 'ignore'

            for action in ('ignore', 'skip'):
                if action.startswith(response.lower()):
                    response = action
                    break
            else:
                print(f'Invalid response: `{response}`')
                response = None

            if response is not None:
                break
        future = kwargs.get('future')
        if future:
            future.set_result(response)
        return response


    debounced_dump = DebounceAsync(dump, 250, max_wait=500, leading=True)


    def on_closed(*args):
        global dropbot
        dropbot = None


    signal_register = {}


    def register_signal(signame, func):
        new_signal = aiosignal.Signal(signame)
        new_signal.append(func)
        new_signal.freeze()
        signal_register[signame] = new_signal


    register_signal('version-mismatch', on_version_mismatch)
    register_signal('no-power', on_no_power)
    register_signal('connected', on_connected)
    register_signal('disconnected', on_disconnected)

    for name_i in DROPBOT_SIGNAL_NAMES + ('chip-inserted', 'chip-removed'):
        if name_i in ('output_enabled', 'output_disabled'):
            continue
        elif name_i == 'capacitance-updated':
            task = ft.partial(debounced_dump, name_i)
            register_signal(name_i, task)
        else:
            task = ft.partial(co_dump, name_i)
            register_signal(name_i, task)

    register_signal('closed', on_closed)

    loop = asyncio.get_event_loop()
    task = loop.create_task(monitor(signal_register))
    loop.run_until_complete(task)
