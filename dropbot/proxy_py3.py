# coding: utf-8
# # XXX TODO
#
#  1. Demux incoming packets based on packet type.
#      - [ ] Send `blinker` signal when a packet is received.
#  2. Requests should only complete if a **`DATA`** packet is available.
#
# For **(2)**, there could be an
# [`asyncio.Queue`](https://docs.python.org/3/library/asyncio-queue.html#asyncio.Queue)
# that the request could `await` on.  The `blinker` signal handler for
# **`DATA`** packets could push into that queue.
import logging
import platform
import sys
import time
import threading

import asyncio
import asyncserial
import base_node_rpc as bnr
import base_node_rpc.async
import serial
import serial_device as sd
import serial_device.threaded

from .node import Proxy


async def _async_serial_keepalive(parent, *args, **kwargs):
    port = None
    while not parent.stop_event.wait(.01):
        try:
            parent.connected_event.clear()
            with asyncserial.AsyncSerial(*args, **kwargs) as async_device:
                logging.info('connected to %s', async_device.ser.port)
                parent.connected_event.set()
                parent.device = async_device
                port = async_device.ser.port
                while async_device.ser.is_open:
                    try:
                        async_device.ser.in_waiting
                    except serial.SerialException:
                        break
                    else:
                        await asyncio.sleep(.01)
            logging.info('disconnected from %s', port)
        except serial.SerialException as e:
            pass
        parent.disconnected_event.set()
    logging.info('stopped monitoring %s', port)


@bnr.async.with_loop
def async_serial_monitor(parent, *args, **kwargs):
    return _async_serial_keepalive(parent, *args, **kwargs)


class AsyncSerialMonitor(threading.Thread):
    '''
    Thread connects to serial port and automatically tries to
    reconnect if disconnected.

    Can be used as a context manager to automatically release
    the serial port on exit.

    For example:

    >>> with BaseNodeSerialMonitor(port='COM8') as monitor:
    >>>     # Wait for serial device to connect.
    >>>     monitor.connected_event.wait()
    >>>     print(asyncio.run_coroutine_threadsafe(monitor.device.write('hello, world'), monitor.loop).result())

    Otherwise, the :meth:`stop` method must *explicitly* be called
    to release the serial connection before it can be connected to
    by other code.  For example:

    >>> monitor = BaseNodeSerialMonitor(port='COM8')
    >>> # Wait for serial device to connect.
    >>> monitor.connected_event.wait()
    >>> print(asyncio.run_coroutine_threadsafe(monitor.device.write('hello, world'), monitor.loop).result())
    >>> monitor.stop()

    Attributes
    ----------
    loop : asyncio event loop
        Event loop serial monitor is running under.
    device : asyncserial.AsyncSerial
        Reference to *active* serial device reference.

        Note that this reference *MAY* change if serial connection
        is interrupted and reconnected.
    connected_event : threading.Event
        Set when serial connection is established.
    disconnected_event : threading.Event
        Set when serial connection is lost.
    '''
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.connected_event = threading.Event()
        self.disconnected_event = threading.Event()
        self.stop_event = threading.Event()
        self.loop = None
        self.device = None
        super(AsyncSerialMonitor, self).__init__()
        self.daemon = True

    def run(self):
        if platform.system() == 'Windows':
            loop = asyncio.ProactorEventLoop()
            asyncio.set_event_loop(loop)
        else:
            loop = asyncio.new_event_loop()
        self.loop = loop
        self.kwargs['loop'] = loop
        async_serial_monitor(self, *self.args, **self.kwargs)

    def stop(self):
        self.stop_event.set()
        try:
            self.device.close()
        except:
            pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()


class BaseNodeSerialMonitor(AsyncSerialMonitor):
    def request(self, request, *args, **kwargs):
        return asyncio            .run_coroutine_threadsafe(self.arequest(request),
                                      loop=self.loop).result(*args, **kwargs)

    async def arequest(self, request):
        return await bnr._async_py36._request(request, device=self.device)


# In[4]:



class SerialProxy(Proxy):
    '''
    Example using :class:`BaseNodeSerialMonitor` for DropBot
    RPC communication.
    '''
    def __init__(self, settling_time_s=.05, **kwargs):
        if kwargs.get('port') is None:
            # Find DropBots
            df_devices = bnr.available_devices(timeout=.5)
            if not df_devices.shape[0]:
                raise IOError('No serial devices available for connection')
            df_dropbots = df_devices.loc[df_devices.device_name == 'dropbot']
            if not df_dropbots.shape[0]:
                raise IOError('No DropBot available for connection')
            kwargs['port'] = df_dropbots.index[0]
        self.monitor = BaseNodeSerialMonitor(*args, **kwargs)
        self.monitor.start()
        self.monitor.connected_event.wait()

    def _send_command(self, packet):
        return self.monitor.request(packet.tostring())

    def terminate(self):
        self.monitor.stop()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.terminate()

    def __del__(self):
        self.terminate()

