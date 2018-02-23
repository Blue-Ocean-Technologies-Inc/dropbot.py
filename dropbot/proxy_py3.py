# coding: utf-8
from logging_helpers import _L
import base_node_rpc as bnr
import base_node_rpc.async

from .node import Proxy


class SerialProxy(Proxy):
    '''
    Example using :class:`BaseNodeSerialMonitor` for DropBot
    RPC communication.
    '''
    def __init__(self, settling_time_s=.05, **kwargs):
        self.default_timeout = kwargs.pop('timeout', 5)
        if kwargs.get('port') is None:
            # Find DropBots
            df_devices = bnr.available_devices(timeout=settling_time_s)
            if not df_devices.shape[0]:
                raise IOError('No serial devices available for connection')
            df_dropbots = df_devices.loc[df_devices.device_name == 'dropbot']
            if not df_dropbots.shape[0]:
                raise IOError('No DropBot available for connection')
            kwargs['port'] = df_dropbots.index[0]
        self.monitor = bnr.async.BaseNodeSerialMonitor(**kwargs)
        self.monitor.start()
        self.monitor.connected_event.wait()

    def _send_command(self, packet, timeout=None):
        if timeout is None:
            timeout = self.default_timeout
        _L().debug('using timeout %s', timeout)
        return self.monitor.request(packet.tostring(), timeout=timeout)

    def terminate(self):
        self.monitor.stop()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.terminate()

    def __del__(self):
        self.terminate()
