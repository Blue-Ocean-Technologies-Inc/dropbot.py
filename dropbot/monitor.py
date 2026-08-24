'''
.. versionadded:: 1.67

.. versionchanged:: 1.68
    If 12V power is not detected, prompt to either a) ignore and connect
    anyway; or b) skip the DropBot.
'''
import asyncio
import inspect

import base_node_rpc as bnr

from pprint import pprint
from logging_helpers import _L
from dropbot.proxy import SerialProxy, NoPower, EVENT_CHANNELS_UPDATED, EVENT_SHORTS_DETECTED, EVENT_ENABLE, __version__
from dropbot.bin import upload

DROPBOT_SIGNAL_NAMES = ('halted', 'output_enabled',
                        'output_disabled', 'capacitance-updated',
                        'channels-updated', 'shorts-detected')

dropbot = None


def _signal(signals_, name):
    """
    Look up the signal registered as :data:`name`.

    Supports both a `blinker.Namespace` (where ``signal()`` creates the signal
    on demand) and a plain `dict` of signals.

    Returns
    -------
    blinker.Signal or None
        ``None`` if :data:`signals_` is a plain mapping with no such key.

    .. versionadded:: 1.74.4
    """
    signal_factory = getattr(signals_, 'signal', None)
    if signal_factory is not None:
        return signal_factory(name)
    return signals_.get(name)


def _as_coroutine_function(func):
    """
    Wrap a plain function so that it may be used as a coroutine function.

    Used as the ``_sync_wrapper`` argument of `blinker.Signal.send_async()`.

    .. versionadded:: 1.74.4
    """

    async def _wrapped(*args, **kwargs):
        return func(*args, **kwargs)

    return _wrapped


async def _send(signals_, name, *args, **kwargs) -> list:
    """
    Send the :data:`name` signal, awaiting any coroutine receivers.

    Notes
    -----
    `blinker` >= 1.7 is strict about mixing receiver kinds: ``Signal.send()``
    raises ``RuntimeError`` if *any* receiver is a coroutine function, and
    ``Signal.send_async()`` raises ``RuntimeError`` if *any* receiver is a
    plain function unless a ``_sync_wrapper`` is supplied.  Since DropBot
    consumers register a mixture of both, this helper always uses
    ``send_async()`` with a ``_sync_wrapper``, which handles every
    combination (including a signal with no receivers at all).

    Older `blinker` releases have no ``send_async()``; those fall back to a
    plain ``send()`` with any returned coroutines awaited explicitly.

    Returns
    -------
    list
        ``(receiver, response)`` pairs.

    .. versionadded:: 1.74.4
    """
    signal = _signal(signals_, name)
    if signal is None:
        _L().debug(f'No `{name}` signal registered.')
        return []

    send_async = getattr(signal, 'send_async', None)
    if send_async is not None:
        return await send_async(*args, _sync_wrapper=_as_coroutine_function, **kwargs)

    # `blinker` < 1.7: `send()` returns un-awaited coroutines for coroutine
    # receivers.
    responses = []
    for receiver, response in signal.send(*args, **kwargs):
        if inspect.isawaitable(response):
            response = await response
        responses.append((receiver, response))
    return responses


async def monitor(signals_: dict, register_signal=None):
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
    signals_: blinker.Namespace
        Namespace for DropBot monitor signals.

        A plain `dict` mapping signal name to `blinker.Signal` is also
        accepted.  Receivers may be plain functions or coroutine functions;
        coroutine responses are awaited (see :func:`_send`).

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
    >>> async def dump(*args, **kwargs):
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

    .. versionchanged:: 1.74.4
        Use a single, consistent (`blinker`) signal API throughout.  A
        `'skip'` response to the `'no-power'`/`'version-mismatch'` prompts now
        actually skips the port instead of re-prompting forever, and a missing
        prompt receiver raises instead of retrying silently.
    """
    loop = asyncio.get_running_loop()
    global dropbot
    dropbot = None

    # Ports the user explicitly chose to skip, so that a `'skip'` response is
    # not immediately undone by the reconnect loop re-selecting the same port.
    skipped_ports = set()

    async def co_flash_firmware():
        if dropbot is not None:
            dropbot.terminate()
        # `upload()` shells out to PlatformIO and blocks for several seconds;
        # run it off the event loop so the loop is not stalled.
        await loop.run_in_executor(None, upload.upload)
        await asyncio.sleep(.5)

    def flash_firmware(dropbot_):
        loop.create_task(co_flash_firmware())

    if register_signal is not None:
        register_signal('flash-firmware',
                        lambda *args: loop.call_soon_threadsafe(flash_firmware, dropbot))

    def reconnect(dropbot_):
        if dropbot_ is not None:
            dropbot_.terminate()

    if register_signal is not None:
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
            df_comports = df_comports.reset_index()

            df_comports = df_comports.sort_values(['device_version', 'port'], ascending=[False, True])
            df_comports = df_comports.set_index('port')

            # Drop any ports the user asked to skip.
            if skipped_ports:
                df_comports = df_comports.loc[~df_comports.index.isin(skipped_ports)]

            if not len(df_comports):
                # Nothing (left) to connect to.  Sleep before retrying;
                # otherwise this becomes a busy-loop that pins a CPU core.
                await asyncio.sleep(.1)
                continue

            port = df_comports.index[0]

            async def _prompt(name, exception, **kwargs):
                """
                Ask receivers of the :data:`name` signal how to proceed.

                Receivers respond either by setting a result on the supplied
                ``future`` keyword argument or by returning the response
                directly.

                Raises
                ------
                Exception
                    Re-raises :data:`exception` if no receiver is registered
                    or if no receiver produced a response.  Retrying silently
                    would otherwise spin forever on the same failure.
                """
                signal = _signal(signals_, name)
                if signal is None or not signal.receivers:
                    _L().error(f'No `{name}` receiver is registered, so there '
                               f'is no way to determine how to proceed.  '
                               f'Re-raising.')
                    raise exception

                response_future = asyncio.Future()
                responses = await _send(signals_, name, 'keep_alive',
                                        future=response_future, **kwargs)

                if response_future.done():
                    return response_future.result()

                # Fall back to the first non-`None` value returned directly by
                # a receiver.
                for _, response in responses:
                    if response is not None:
                        return response

                _L().error(f'No receiver of the `{name}` signal provided a '
                           f'response, so there is no way to determine how to '
                           f'proceed.  Re-raising.')
                raise exception

            async def _attempt_connect(**kwargs):
                ignore = kwargs.pop('ignore', [])
                try:
                    # Attempt to connect to automatically selected port.
                    dropbot_ = SerialProxy(port=port, ignore=ignore, **kwargs)
                    return dropbot_
                except NoPower as exception:
                    # No 12V power supply detected on DropBot.
                    _L().debug('No 12V power supply detected.')
                    response = await _prompt('no-power', exception)

                    if response == 'ignore':
                        ignore.append(NoPower)
                    else:
                        # Do not re-select this port on the next iteration of
                        # the reconnect loop, otherwise the user would be
                        # prompted again immediately, forever.
                        skipped_ports.add(port)
                        raise exception

                except bnr.proxy.DeviceVersionMismatch as exception:
                    # Firmware version does not match driver version.
                    _L().debug(f"Driver version (`{__version__}`) does not match firmware "
                               f"version (`{exception.device_version}`)")

                    response = await _prompt('version-mismatch',
                                             exception,
                                             driver_version=__version__,
                                             firmware_version=exception.device_version)

                    update = False

                    if response == 'ignore':
                        ignore.append(bnr.proxy.DeviceVersionMismatch)
                    elif response == 'update':
                        update = True
                    else:
                        skipped_ports.add(port)
                        raise exception

                    if update:
                        # Flash firmware and retry connection.
                        _L().info('Flash firmware and retry connection.')
                        await co_flash_firmware()

                dropbot_ = await _attempt_connect(ignore=ignore, **kwargs)
                return dropbot_

            try:
                dropbot = await _attempt_connect()
            except bnr.proxy.DeviceNotFound:
                raise RuntimeError('Could not find device')
            except asyncio.CancelledError:
                raise
            except Exception:
                _L().debug('Error connecting to DropBot.', exc_info=True)
                await asyncio.sleep(.1)
                continue

            def co_connect(name):
                """
                Build a DropBot signal receiver that forwards the message to
                the `name` signal of the :data:`signals_` namespace.

                DropBot signals are emitted from the serial monitor thread, so
                forwarding is scheduled onto this coroutine's event loop.
                """

                def _wrapped(sender, **message):
                    async def co_callback():
                        await _send(signals_, name, 'keep_alive', **message)

                    return loop.call_soon_threadsafe(loop.create_task, co_callback())

                return _wrapped

            for name_j in DROPBOT_SIGNAL_NAMES:
                dropbot.signals.signal(name_j).connect(co_connect(name_j), weak=False)

            dropbot.signals.signal('output_enabled').connect(co_connect('chip-inserted'), weak=False)
            dropbot.signals.signal('output_disabled').connect(co_connect('chip-removed'), weak=False)

            await _send(signals_, 'connected', 'keep_alive', dropbot=dropbot)

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

            # NOTE `disconnected` is emitted on the serial monitor's
            # `serial_signals` namespace, **not** on `dropbot.signals` (which
            # carries device *events* decoded from stream packets).  Wiring it
            # to `dropbot.signals` meant the disconnect was never observed and
            # the monitor blocked here forever.
            dropbot.serial_signals.signal('disconnected').connect(
                lambda *args: loop.call_soon_threadsafe(disconnected.set),
                weak=False)

            await disconnected.wait()

            dropbot.terminate()

            await _send(signals_, 'disconnected', 'keep_alive')
    finally:
        # NOTE Terminate **before** notifying, so that the DropBot connection
        # is always released even if a `closed` receiver misbehaves (this
        # block also runs during cancellation).
        if dropbot is not None:
            dropbot.terminate()
        try:
            await _send(signals_, 'closed', 'keep_alive')
        except asyncio.CancelledError:
            # Already being cancelled; `closed` receivers that yield to the
            # loop cannot complete.  Nothing more to clean up.
            raise
        except Exception:
            _L().warning('Error sending `closed` signal.', exc_info=True)


if __name__ == '__main__':
    import logging
    import blinker

    import functools as ft

    from debounce import DebounceAsync

    logging.basicConfig(level=logging.DEBUG)

    connected = asyncio.Event()


    async def on_connected(sender, **message):
        connected.dropbot = message['dropbot']
        _L().info(f'sender=`{sender}`')
        for line in str(connected.dropbot.properties).splitlines():
            _L().info(line)
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
        message = (f"Driver version `{kwargs.get('driver_version')}` does not "
                   f"match firmware version `{kwargs.get('firmware_version')}`.")
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


    signal_register = blinker.Namespace()


    def register_signal(signame, func):
        signal_register.signal(signame).connect(func, weak=False)


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

    loop = asyncio.new_event_loop()
    task = loop.create_task(monitor(signal_register, register_signal=register_signal))
    loop.run_until_complete(task)
    loop.close()
