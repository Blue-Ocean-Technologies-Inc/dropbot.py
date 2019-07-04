# coding: utf-8
from logging_helpers import _L
import base_node_rpc as bnr
import base_node_rpc.async
import time

from .bin.upload import upload
from .node import Proxy
from .proxy_py2 import (ProxyMixin, EVENT_ACTUATED_CHANNEL_CAPACITANCES,
                        EVENT_CHANNELS_UPDATED, EVENT_SHORTS_DETECTED,
                        EVENT_DROPS_DETECTED, EVENT_ENABLE)


class SerialProxy(ProxyMixin, Proxy):
    '''
    Example using :class:`BaseNodeSerialMonitor` for DropBot
    RPC communication.
    '''
    def __init__(self, settling_time_s=.05, **kwargs):
        self.default_timeout = kwargs.pop('timeout', 5)
        port = kwargs.pop('port', None)
        if port is None:
            # Find DropBots
            df_devices = bnr.available_devices(timeout=settling_time_s)
            if not df_devices.shape[0]:
                raise IOError('No serial devices available for connection')
            df_dropbots = df_devices.loc[df_devices.device_name == 'dropbot']
            if not df_dropbots.shape[0]:
                raise IOError('No DropBot available for connection')
            port = df_dropbots.index[0]
        self.port = port
        self.monitor = None
        self.connect()
        super(SerialProxy, self).__init__(**kwargs)

    @property
    def signals(self):
        return self.monitor.signals

    def connect(self):
        self.terminate()
        monitor = bnr.async.BaseNodeSerialMonitor(port=self.port)
        monitor.start()
        monitor.connected_event.wait()
        self.monitor = monitor
        return self.monitor

    def _send_command(self, packet, timeout=None):
        if timeout is None:
            timeout = self.default_timeout
        _L().debug('using timeout %s', timeout)
        return self.monitor.request(packet.tostring(), timeout=timeout)

    def terminate(self):
        if self.monitor is not None:
            self.monitor.stop()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.terminate()

    def __del__(self):
        self.terminate()

    def flash_firmware(self):
        # currently, we're ignoring the hardware version, but eventually,
        # we will want to pass it to upload()
        self.terminate()
        try:
            upload()
        except Exception:
            _L().debug('error updating firmware')
        time.sleep(0.5)
        self.connect()


I2cProxy = None
