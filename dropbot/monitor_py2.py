'''
.. versionadded:: 1.67

.. versionchanged:: 1.68
    If 12V power is not detected, prompt to either a) ignore and connect
    anyway; or b) skip the DropBot.
'''
from __future__ import division, print_function, unicode_literals
import platform
import time

from base_node_rpc.async import asyncio, _available_devices
from logging_helpers import _L
import base_node_rpc as bnr
import dropbot as db
import dropbot.proxy


DROPBOT_SIGNAL_NAMES = ('halted', 'output_enabled',
                        'output_disabled', 'capacitance-updated',
                        'channels-updated', 'shorts-detected')


@asyncio.coroutine
def monitor(signals):
    '''
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
    signals : blinker.Namespace
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
    >>> signals.signal('chip-inserted').connect(dump, weak=False)
    >>> loop = asyncio.ProactorEventLoop()
    >>> asyncio.set_event_loop(loop)
    >>> task = loop.create_task(db.monitor.dropbot_monitor(signals))
    >>> # Stop monitor after 15 seconds.
    >>> loop.call_later(15, task.cancel)
    >>> loop.run_until_complete(task)


    .. versionchanged:: 1.67.1
        Upon connection, send `'chip-inserted'` if chip is inserted or send
        `'chip-removed'` if no chip is inserted.

    .. versionchanged:: 1.68
        Send `'no-power'` signal if 12V power supply not connected.  Receivers
        may return `'ignore'` to attempt to connect anyway.
    '''
    loop = asyncio.get_event_loop()
    dropbot = None

    @asyncio.coroutine
    def co_flash_firmware():
        if dropbot is not None:
            dropbot.terminate()
        db.bin.upload.upload()
        time.sleep(.5)

    def flash_firmware(dropbot):
        loop.create_task(co_flash_firmware())

    signals.signal('flash-firmware') \
        .connect(lambda *args: loop.call_soon_threadsafe(flash_firmware,
                                                         dropbot), weak=False)

    def reboot(dropbot):
        if dropbot is not None:
            dropbot._reboot()

    signals.signal('reboot') \
        .connect(lambda *args: loop.call_soon_threadsafe(reboot, dropbot),
                 weak=False)

    def reconnect(dropbot):
        if dropbot is not None:
            dropbot.terminate()

    signals.signal('reconnect') \
        .connect(lambda *args: loop.call_soon_threadsafe(reconnect, dropbot),
                 weak=False)

    try:
        while True:
            # Multiple DropBot devices were found.
            # Get list of available devices.
            df_comports = yield asyncio.From(_available_devices(timeout=.1))

            if 'device_name' not in df_comports or not df_comports.shape[0]:
                yield asyncio.From(asyncio.sleep(.1))
                continue

            # Automatically select DropBot with highest version, with ties
            # going to the lowest port name (i.e., `COM1` before `COM2`).
            df_comports = df_comports.loc[df_comports.device_name ==
                                          'dropbot'].copy()
            df_comports.reset_index(inplace=True)

            df_comports.sort_values(['device_version', 'port'],
                                    ascending=[False, True], inplace=True)
            df_comports.set_index('port', inplace=True)
            port = df_comports.index[0]

            @asyncio.coroutine
            def _attempt_connect(**kwargs):
                ignore = kwargs.pop('ignore', [])
                try:
                    # Attempt to connect to automatically selected port.
                    dropbot = db.SerialProxy(port=port, ignore=ignore,
                                             **kwargs)
                    raise asyncio.Return(dropbot)
                except db.proxy.NoPower as exception:
                    # No 12V power supply detected on DropBot.
                    _L().debug('No 12V power supply detected.')
                    responses = signals.signal('no-power').send('keep_alive')

                    for t in asyncio.as_completed([loop.create_task(r[1])
                                                   for r in responses]):
                        response = yield asyncio.From(t)
                        if response == 'ignore':
                            ignore.append(db.proxy.NoPower)
                            break
                    else:
                        raise exception
                except bnr.proxy.DeviceVersionMismatch as exception:
                    # Firmware version does not match driver version.
                    _L().debug('Driver version (`%s`) does not match firmware '
                            'version (`%s`)', db.__version__,
                            exception.device_version)
                    responses = signals.signal('version-mismatch')\
                        .send('keep_alive', driver_version=db.__version__,
                            firmware_version=exception.device_version)

                    update = False

                    for t in asyncio.as_completed([loop.create_task(r[1])
                                                   for r in responses]):
                        response = yield asyncio.From(t)
                        if response == 'ignore':
                            ignore.append(bnr.proxy.DeviceVersionMismatch)
                            break
                        elif response == 'update':
                            update = True
                            break
                    else:
                        raise

                    if update:
                        # Flash firmware and retry connection.
                        _L().info('Flash firmware and retry connection.')
                        yield asyncio.From(co_flash_firmware())

                dropbot = yield asyncio.From(_attempt_connect(ignore=ignore,
                                                              **kwargs))
                raise asyncio.Return(dropbot)

            try:
                dropbot = yield asyncio.From(_attempt_connect())
            # except bnr.proxy.DeviceNotFound:
            except asyncio.CancelledError:
                raise
            except Exception:
                _L().debug('Error connecting to DropBot.', exc_info=True)
                yield asyncio.From(asyncio.sleep(.1))
                continue

            def co_connect(name):
                def _wrapped(sender, **message):
                    @asyncio.coroutine
                    def co_callback(message):
                        listeners = signals.signal(name).send('keep_alive',
                                                              **message)
                        yield asyncio.From(asyncio.gather(*(l[1] for l in listeners)))

                    return loop.call_soon_threadsafe(loop.create_task,
                                                     co_callback(sender,
                                                                 **message))
                return _wrapped

            for name_i in DROPBOT_SIGNAL_NAMES:
                dropbot.signals.signal(name_i).connect(co_connect(name_i),
                                                       weak=False)

            dropbot.signals.signal('output_enabled')\
                .connect(co_connect('chip-inserted'), weak=False)
            dropbot.signals.signal('output_disabled')\
                .connect(co_connect('chip-removed'), weak=False)

            responses = signals.signal('connected').send('keep_alive',
                                                         dropbot=dropbot)
            yield asyncio.From(asyncio.gather(*(r[1] for r in responses)))

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

            dropbot.serial_signals.signal('disconnected')\
                .connect(lambda *args:
                         loop.call_soon_threadsafe(disconnected.set),
                         weak=False)

            yield asyncio.From(disconnected.wait())

            dropbot.terminate()

            responses = signals.signal('disconnected').send('keep_alive')
            yield asyncio.From(asyncio.gather(*(r[1] for r in responses)))
    finally:
        signals.signal('closed').send('keep_alive')
        if dropbot is not None:
            dropbot.terminate()


if __name__ == '__main__':
    import logging
    import functools as ft

    import blinker
    import debounce.async
    import debounce

    logging.basicConfig(level=logging.DEBUG)

    connected = asyncio.Event()

    @asyncio.coroutine
    def on_connected(sender, **message):
        connected.dropbot = message['dropbot']
        _L().info('sender=`%s`', sender)
        map(_L().info, str(connected.dropbot.properties).splitlines())
        connected.dropbot.update_state(capacitance_update_interval_ms=10,
                                       event_mask=db.EVENT_CHANNELS_UPDATED |
                                       db.EVENT_SHORTS_DETECTED |
                                       db.EVENT_ENABLE)
        connected.set()

    @asyncio.coroutine
    def on_disconnected(*args, **kwargs):
        global dropbot
        dropbot = None
        _L().info('args=`%s`, kwargs=`%s`', args, kwargs)

    @asyncio.coroutine
    def on_halted(*args, **kwargs):
        _L().info('args=`%s`, kwargs=`%s`', args, kwargs)

    def dump(name, *args, **kwargs):
        print('\r[%s] args=`%s`, kwargs=`%s`%-20s' % (name, args, kwargs, '')),

    @asyncio.coroutine
    def co_dump(*args, **kwargs):
        raise asyncio.Return(dump(*args, **kwargs))

    @asyncio.coroutine
    def on_version_mismatch(*args, **kwargs):
        _L().info('args=`%s`, kwargs=`%s`', args, kwargs)
        message = ('Driver version `%(driver_version)s` does not match '
                   'firmware `%(firmware_version)s` version.' % kwargs)
        while True:
            response = raw_input(message + ' [I]gnore/[u]pdate/[s]kip: ')
            if not response:
                # Default response is `ignore` and try to connect anyway.
                response = 'ignore'

            for action in ('ignore', 'update', 'skip'):
                if action.startswith(response.lower()):
                    response = action
                    break
            else:
                print('Invalid response: `%s`' % response)
                response = None

            if response is not None:
                break
        if response == 'skip':
            raise IOError(message)
        raise asyncio.Return(response)

    @asyncio.coroutine
    def on_no_power(*args, **kwargs):
        while True:
            response = raw_input('No 12V power supply detected.  '
                                 '[I]gnore/[s]kip: ')
            if not response:
                # Default response is `ignore` and try to connect anyway.
                response = 'ignore'

            for action in ('ignore', 'skip'):
                if action.startswith(response.lower()):
                    response = action
                    break
            else:
                print('Invalid response: `%s`' % response)
                response = None

            if response is not None:
                break
        raise asyncio.Return(response)

    debounced_dump = debounce.async.Debounce(dump, 250, max_wait=500,
                                             leading=True)

    def on_closed(*args):
        global dropbot
        dropbot = None

    signals = blinker.Namespace()

    signals.signal('version-mismatch').connect(on_version_mismatch, weak=False)
    signals.signal('no-power').connect(on_no_power, weak=False)
    signals.signal('connected').connect(on_connected, weak=False)
    signals.signal('disconnected').connect(on_disconnected, weak=False)

    for name_i in DROPBOT_SIGNAL_NAMES + ('chip-inserted', 'chip-removed'):
        if name_i in ('output_enabled', 'output_disabled'):
            continue
        elif name_i == 'capacitance-updated':
            task = ft.partial(asyncio.coroutine(debounced_dump), name_i)
            signals.signal(name_i).connect(task, weak=False)
        else:
            signals.signal(name_i).connect(ft.partial(co_dump, name_i),
                                           weak=False)

    signals.signal('closed').connect(on_closed, weak=False)

    if platform.system() == 'Windows':
        loop = asyncio.ProactorEventLoop()
        asyncio.set_event_loop(loop)
    else:
        loop = asyncio.get_event_loop()
    task = loop.create_task(monitor(signals))
    loop.run_until_complete(task)
