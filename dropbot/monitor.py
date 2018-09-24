'''
.. versionadded:: X.X.X
'''
import time

from logging_helpers import _L
import base_node_rpc as bnr
import base_node_rpc.async
import dropbot as db
import trollius as asyncio


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
            df_comports = yield asyncio.From(bnr.async
                                             ._available_devices(timeout=.1))

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

            try:
                # Attempt to connect to automatically selected port.
                dropbot = db.SerialProxy(port=port)
            except bnr.proxy.DeviceVersionMismatch as exception:
                # Firmware version does not match driver version.
                _L().debug('Driver version (`%s`) does not match firmware '
                           'version (`%s`)', db.__version__,
                           exception.device_version)
                responses = signals.signal('version-mismatch')\
                    .send('keep_alive', driver_version=db.__version__,
                          firmware_version=exception.device_version)
                try:
                    results = yield asyncio.From(asyncio.gather(*(r[1]
                                                                  for r in
                                                                  responses)))
                    if results and results[0] == 'ignore':
                        dropbot = \
                            db.SerialProxy(port=port,
                                           ignore=[bnr.proxy
                                                   .DeviceVersionMismatch])
                    elif not results or results[0] == 'update':
                        # No signal receiver raised an exception.  Flash
                        # firmware and retry connection.
                        _L().info('Flash firmware and retry connection.')
                        yield asyncio.From(co_flash_firmware())
                        continue
                except asyncio.CancelledError:
                    raise
                except Exception as exception:
                    # # Signal receiver raised exception.  Do not connect.
                    _L().debug('No signal receiver raised an exception. Do not'
                               ' connect')
                    yield asyncio.From(asyncio.sleep(.1))
                    continue
            except bnr.proxy.DeviceNotFound:
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
