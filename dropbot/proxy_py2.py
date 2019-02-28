from __future__ import absolute_import
import calendar
import datetime as dt
import logging
import math
import threading
import time
import uuid

from base_node_rpc.proxy import ConfigMixinBase, StateMixinBase
from path_helpers import path
from six.moves import range
from teensy_minimal_rpc.adc_sampler import AdcDmaMixin
import numpy as np
import pandas as pd
import serial
import six

from ._version import get_versions
from .bin.upload import upload

__version__ = get_versions()['version']
del get_versions

logger = logging.getLogger(__name__)

# Event mask flags
EVENT_ACTUATED_CHANNEL_CAPACITANCES = (1 << 31)
EVENT_CHANNELS_UPDATED              = (1 << 30)
EVENT_SHORTS_DETECTED               = (1 << 28)
EVENT_DROPS_DETECTED                = (1 << 27)
EVENT_ENABLE                        = (1 << 0)


class I2cAddressNotSet(Exception):
    pass


class NoPower(Exception):
    pass


class CommunicationError(Exception):
    pass


def _unpack_drops(packed_drops):
    '''
    Unpack raw drops array format:

        [drop 0 channel count][drop 0: channel 0, channel 1, ...][drop 1 channel count][drop 1: channel 0, channel 1, ...]

    into a list of `numpy.array` channels lists.


    Returns
    -------
    list<numpy.array>
        List of channels lists, each channel list corresponding to channels
        covered by a drop.
    '''
    drops = []
    i = 0
    while i < packed_drops.shape[0]:
        drop_j_size = packed_drops[i]
        start_j = i + 1
        end_j = start_j + drop_j_size
        drop_j = packed_drops[i + 1:end_j]
        drops.append(drop_j)
        i = end_j
    return drops


try:
    from .node import (Proxy as _Proxy, I2cProxy as _I2cProxy,
                       SerialProxy as _SerialProxy)
    from .config import Config
    from .state import State

    class ConfigMixin(ConfigMixinBase):
        @property
        def config_class(self):
            return Config

    class StateMixin(StateMixinBase):
        @property
        def state_class(self):
            return State

    class ProxyMixin(ConfigMixin, StateMixin, AdcDmaMixin):
        '''
        Mixin class to add convenience wrappers around methods of the generated
        `node.Proxy` class.
        '''
        host_package_name = str(path(__file__).parent.name.replace('_', '-'))

        def __init__(self, *args, **kwargs):
            '''
            .. versionchanged:: 1.27
                Add ``ignore`` parameter.

            .. versionchanged:: 1.34.1
                Add message to :class:`NoPower` exception.

            .. versionchanged:: 1.55
                Synchronize device millisecond counter to UTC time upon making
                a connection (and on any subsequent *reconnection*).


            Parameters
            ----------
            ignore : bool or list, optional
                List of non-critical exception types to ignore during
                initialization.

                This allows, for example:

                 - Testing connection to a DropBot control board that is not
                   connected to a power source.
                 - Connecting to a DropBot without a configured I2C address to
                   set an I2C address.

                If set to ``True``, all optional exception types to ignore
                during initialization.

                Default is to raise all exceptions encountered during
                initialization.
            '''
            self.transaction_lock = threading.RLock()
            self.__number_of_channels = 0
            try:
                # Get list of exception types to ignore.
                #
                # XXX This allows, for example:
                #  - Testing connection to a DropBot control board that is not
                #    connected to a power source.
                #  - Connecting to a DropBot without a configured I2C address
                #    to set an I2C address.
                super(ProxyMixin, self).__init__(*args, **kwargs)
                # XXX TODO Need to initialize DMA in embedded C++ code.
                # XXX Otherwise, initialization will not be performed on
                # device reset.
                self.init_dma()

                ignore = kwargs.pop('ignore', [])

                if isinstance(ignore, bool):
                    if ignore:
                        # If `ignore` is set to `True`, ignore all optional
                        # exceptions.
                        ignore = [NoPower, I2cAddressNotSet]
                    else:
                        ignore = []

                # Check that we have power
                if NoPower not in ignore and self.measure_voltage() < 5:
                    raise NoPower('Please check that 12V power supply is '
                                  'connected.')

                # Only initialize switching boards if the control board has
                # been assigned an i2c address.
                if self.config.i2c_address != 0:
                    self.initialize_switching_boards()
                elif I2cAddressNotSet not in ignore:
                    raise I2cAddressNotSet()

                # Synchronize device millisecond counter to UTC time upon
                # connection.
                self.signals.signal('connected').connect(lambda *args:
                                                         self.sync_time(),
                                                         weak=False)

                self.signals.signal('connected').send({'event': 'connected'})
            except Exception:
                logger.debug('Error connecting to device.', exc_info=True)
                self.terminate()
                raise

        def _initialize_switching_boards(self):
            return super(ProxyMixin, self).initialize_switching_boards()

        def initialize_switching_boards(self):
            '''
            .. versionadded:: X.X.X
                Wrap parent :meth:`initialize_switching_boards()` to cache
                number of available channels.
            '''
            self.__number_of_channels = self._initialize_switching_boards()
            return self.__number_of_channels

        def _connect(self, *args, **kwargs):
            '''
            .. versionadded:: 1.55
                Send ``connected`` event each time a connection has been
                established. Note that the first ``connected`` event is sent
                before any receivers have a chance to connect to the signal,
                but subsequent restored connection events after connecting to
                the ``connected`` signal will be received.
            '''
            super(ProxyMixin, self)._connect(*args, **kwargs)
            self.signals.signal('connected').send({'event': 'connected'})

        def sync_time(self):
            '''
            Synchronize device millisecond counter to UTC time.


            .. versionadded:: 1.55
            '''
            now = dt.datetime.utcnow()
            utc_timestamp = (calendar.timegm(now.utctimetuple()) +
                             now.microsecond * 1e-6)
            super(ProxyMixin, self).sync_time(utc_timestamp)

        @property
        def wall_time(self):
            '''
            Device UTC wall-clock time.


            .. versionadded:: 1.55
            '''
            wall_time = super(ProxyMixin, self).wall_time()
            return dt.datetime.utcfromtimestamp(wall_time)

        @property
        def signals(self):
            '''
            .. versionadded:: 1.43
            '''
            return self._packet_queue_manager.signals

        def i2c_eeprom_write(self, i2c_address, eeprom_address, data):
            '''
            Write data to specified address in I2C EEPROM chip.

            If the number of bytes exceeds the maximum I2C packet size,
            multiple requests will be made automatically.

            Parameters
            ----------
            i2c_address : int
                Address of ``CAT24Cxx`` EEPROM.

                See: http://www.onsemi.com/pub/Collateral/CAT24C01-D.PDF
            eeprom_address : int
                Address to write data to in EEPROM.
            data : list-like
                Bytes to write to :data:`eeprom_address`
            '''
            i2c_packet_size = 16
            for i in range(int(math.ceil(len(data) / float(i2c_packet_size)))):
                start = i * i2c_packet_size
                end = min((i + 1) * i2c_packet_size, len(data))
                self.i2c_write(i2c_address, [eeprom_address + start] +
                               (end - start) * [255])
                self.i2c_write(i2c_address, [eeprom_address + start] +
                               data[start:end])

        def i2c_eeprom_read(self, i2c_address, eeprom_address, length):
            '''
            Read specified number of bytes from I2C EEPROM chip starting at the
            address given.

            If the number of requested bytes exceeds the maximum I2C packet
            size, multiple requests will be made automatically.

            Parameters
            ----------
            i2c_address : int
                Address of ``CAT24Cxx`` EEPROM.

                See: http://www.onsemi.com/pub/Collateral/CAT24C01-D.PDF
            eeprom_address : int
                Address to write data to in EEPROM.
            length : int
                Number of bytes to read.
            '''
            i2c_packet_size = 16

            data = []
            for i in range(int(math.ceil(length / float(i2c_packet_size)))):
                start = i * i2c_packet_size
                end = min((i + 1) * i2c_packet_size, length)
                count = end - start
                self.i2c_write(i2c_address, eeprom_address + start)
                data_i = self.i2c_read(i2c_address, count)
                data.append(data_i)
            return np.concatenate(data)

        def __del__(self):
            try:
                # turn off the high voltage when we disconnect
                self.hv_output_enabled = False
                super(ProxyMixin, self).__del__()
            except Exception:
                # ignore any exceptions (e.g., if we can't communicate with the
                # board)
                pass

        def i2c_send_command(self, address, cmd, data):
            self.i2c_write(address, [cmd] + data)
            n = self.i2c_read(address, 1)
            return self.i2c_read(address, n)

        def measure_capacitance(self, n_samples=50, amplitude='filtered_mean'):
            '''
            Parameters
            ----------
            n_samples : int, optional
                Number of analog measurements to sample for the capacitance
                calculation.
            amplitude : str or list, optional
                The amplitude calculation method.  See `issue #25 <https://gitlab.com/sci-bots/dropbot.py/issues/25>`_
                for more information.  Each value must be one of
                ``filtered_mean`` or ``percentile_difference``.

            Returns
            -------
            float
                If a single amplitude calculation method is specified (i.e.,
                :data:`amplitude` is a string)
            pandas.Series
                If a list of amplitude calculation methods is specified.
                Series is indexed by the specified amplitude calculation
                methods.

            .. versionchanged:: 1.41
                Add :data:`amplitude` keyword argument.  See `issue #25 <https://gitlab.com/sci-bots/dropbot.py/issues/25>`_
                for more information.
            '''
            df_volts = pd.DataFrame({'volts':
                                     self.analog_reads_simple(11, n_samples) *
                                     3.3 / 2**16})

            singleton = False

            if isinstance(amplitude, six.string_types):
                amplitude = [amplitude]
                singleton = True

            values = []
            for method_i in amplitude:
                if method_i == 'filtered_mean':
                    v_gnd = np.mean(df_volts)
                    v_abs = np.abs(df_volts - v_gnd)
                    v_abs_mean = np.mean(v_abs)
                    filter_th = v_abs_mean * 1.5
                    amplitude_i = np.mean(v_abs[v_abs < filter_th])['volts']
                elif method_i == 'percentile_difference':
                    volts_description = df_volts['volts'].describe()
                    amplitude_i = .5 * (volts_description['75%'] -
                                        volts_description['25%'])
                else:
                    raise NameError('Unknown amplitude calculation method '
                                    '`%s`.' % method_i)
                values.append(amplitude_i)

            if not singleton:
                value = pd.Series(values, index=amplitude)
            else:
                value = values[0]
            return value / self.measure_voltage() * 0.15e-6

        def get_environment_state(self, i2c_address=0x27):
            '''
            Acquire temperature and humidity from `Honeywell HIH6000`_ series
            sensor.

            .. _Honeywell HIH6000: http://sensing.honeywell.com/index.php/ci_id/142171/la_id/1/document/1/re_id/0
            '''
            import pandas as pd

            # Trigger measurement.
            self.i2c_write(i2c_address, [])
            time.sleep(.01)

            while True:
                # Read 4 bytes from sensor.
                humidity_data, temperature_data = self.i2c_read(i2c_address,
                                                                4).view('>u2')
                status_code = (humidity_data >> 14) & 0x03
                if status_code == 0:
                    # Measurement completed successfully.
                    break
                elif status_code > 1:
                    raise IOError('Error reading from sensor.')
                # Measurement data is stale (i.e., measurement still in
                # progress).  Try again.
                time.sleep(.001)

            # See URL from docstring for source of equations.
            relative_humidity = float(humidity_data & 0x03FFF) / ((1 << 14) - 2)
            temperature_celsius = (float((temperature_data >> 2) & 0x3FFF) /
                                   ((1 << 14) - 2) * 165 - 40)

            return pd.Series([relative_humidity, temperature_celsius],
                             index=['relative_humidity',
                                    'temperature_celsius'])

        @property
        def frequency(self):
            return self.state['frequency']

        @frequency.setter
        def frequency(self, value):
            return self.update_state(frequency=value)

        @property
        def voltage_limit(self):
            '''
            .. versionadded:: 1.39
            '''
            return 1.5 / 2.0 * (2e6 / self.config['R7'] + 1)

        def measure_voltage(self):
            # divide by 2 to convert from peak-to-peak to rms
            return self.analog_read(1) / 2.0**16 * 3.3 * 2e6 / 20e3 / 2.0

        def measure_input_current(self, n=2000):
            i = self.analog_reads_simple(3, n) / 2.0**16 * 3.3 / 0.03
            results = dict(rms=np.mean(i), max=np.max(i))
            return results

        def measure_output_current(self, n=2000):
            current = (self.analog_reads_simple(2, n) / 2.0**16 * 3.3 /
                       (51e3 / 5.1e3 * 1))
            results = dict(rms=np.mean(current), max=np.max(current))
            return results

        def measure_input_voltage(self):
            # save the state of the the output voltage
            hv_output_enabled = self.hv_output_enabled
            voltage = self.voltage

            # set the voltage to the minimum and wait for it to settle
            self.voltage = self.min_waveform_voltage
            time.sleep(1)

            # disable the boost converter and wait for it to settle
            self.hv_output_enabled = False
            time.sleep(5)

            # take a measurement
            v = self.analog_reads_simple(1, 2000) / 2.0**16 * 3.3 * 2e6 / 20e3

            # restore the output voltage and let it settle
            self.hv_output_enabled = hv_output_enabled
            self.voltage = voltage
            time.sleep(1)
            return np.sqrt(np.mean(v**2))

        @property
        def voltage(self):
            return self.state['voltage']

        @voltage.setter
        def voltage(self, value):
            '''
            .. versionchanged:: 1.66
                Select and enable high voltage output if it is not currently
                enabled.
            '''
            with self.transaction_lock:
                original_state = self.state

                # Construct required state
                state = original_state.copy()
                state.hv_output_enabled = True
                state.hv_output_selected = True
                state.voltage = value

                if not (state == original_state).all():
                    # At least one state property must be modified.
                    # Only update modified state properties.
                    self.state = state[state != original_state]

        @property
        def hv_output_enabled(self):
            return self.state['hv_output_enabled']

        @hv_output_enabled.setter
        def hv_output_enabled(self, value):
            return self.update_state(hv_output_enabled=value)

        @property
        def hv_output_selected(self):
            return self.state['hv_output_selected']

        @hv_output_selected.setter
        def hv_output_selected(self, value):
            return self.update_state(hv_output_selected=value)

        def _disabled_channels_mask(self):
            '''
            .. versionadded:: 1.34

            Prepend underscore to the auto-generated disabled_channels_mask accessor
            '''
            return super(ProxyMixin, self).disabled_channels_mask()

        def _set_disabled_channels_mask(self, mask):
            '''
            .. versionadded:: 1.34

            Prepend underscore to the auto-generated disabled_channels_mask setter
            '''
            return super(ProxyMixin, self).set_disabled_channels_mask(mask)

        @property
        def disabled_channels_mask(self):
            '''
            .. versionadded:: 1.34

            Retrieve the mask bytes from the device and unpack them into an
            array with one entry per channel.  Return unpacked array.

            Notes
            -----

            State of mask for each channel is binary, 0 or 1.  On device,
            mask states are stored in bytes, where each byte corresponds to
            the mask state for eight channels.
            '''
            return np.unpackbits(super(ProxyMixin, self)
                                 .disabled_channels_mask()[::-1])[::-1]

        @disabled_channels_mask.setter
        def disabled_channels_mask(self, mask):
            self.set_disabled_channels_mask(mask)

        def set_disabled_channels_mask(self, mask):
            '''
            .. versionadded:: 1.34

            Pack array containing one entry per channel to bytes (8 channels
            per byte).  Set disabled channels mask on device using mask bytes.

            See also: `disabled_channels_mask` (get)
            '''
            if len(mask) != self.number_of_channels:
                raise ValueError('Error setting disabled channels mask.  Check '
                                 'size of mask matches channel count.')
            super(ProxyMixin, self).set_disabled_channels_mask(
                      np.packbits(mask.astype(int)[::-1])[::-1])

        def _state_of_channels(self):
            '''
            Prepend underscore to the auto-generated state_of_channels accessor
            '''
            return super(ProxyMixin, self).state_of_channels()

        def _set_state_of_channels(self, states):
            '''
            Prepend underscore to the auto-generated state_of_channels setter
            '''
            return super(ProxyMixin, self).set_state_of_channels(states)

        @property
        def state_of_channels(self):
            '''
            Retrieve the state bytes from the device and unpack them into an
            array with one entry per channel.

            Notes
            -----

            State of each channel is binary, 0 or 1.  On device, states are
            stored in bytes, where each byte corresponds to the state of eight
            channels.

            Returns
            -------
            `pandas.Series`
                Boolean channel states indexed by channel number.


            .. versionchanged:: 1.56
                Return channels as `pandas.Series` instance.
            '''
            return pd.Series(np.unpackbits(super(ProxyMixin, self)
                                           .state_of_channels()[::-1])[::-1])

        @state_of_channels.setter
        def state_of_channels(self, states):
            self.set_state_of_channels(states)

        def set_state_of_channels(self, states, append=True):
            '''
            Pack array containing one entry per channel to bytes (8 channels
            per byte).  Set state of channels on device using state bytes.

            See also: `state_of_channels` (get)

            Parameters
            ----------
            states : list or pandas.Series
                If :class:`list`, 0 or 1 for each channel (size must be equal
                to the total number of channels).

                If :class:`pandas.Series`, values 0 or 1 for each corresponding
                channel number listed in index.
            append : bool, optional
                If `True`, append states specified as a :class:`pandas.Series`.
                Otherwise, overwrite existing channel states.


            .. versionchanged:: X.X.X
                Add ``append`` keyword argument.
            '''
            N = self.number_of_channels
            if isinstance(states, pd.Series):
                if len(states) == N or not append:
                    channel_states = np.zeros(N, dtype=int)
                else:
                    channel_states = self.state_of_channels
                channel_states[states.index] = states
                states = channel_states
            else:
                states = np.asarray(states, dtype=int)

            state_bits = np.packbits(states.astype(int)[::-1])[::-1]
            for retry in range(3):
                if not super(ProxyMixin,
                             self).set_state_of_channels(state_bits):
                    raise ValueError('Error setting state of channels.  Check '
                                     'number of states matches channel count.')

                # Verify that the state we set matches the current state
                # (don't include disabled channels)
                current_state = self.state_of_channels
                try:
                    assert(np.all((states.astype(np.uint8) &
                                   ~self.disabled_channels_mask) ==
                           current_state))
                    return
                except Exception:
                    # if not, reset the switching boards and try again
                    self.reset_switching_boards()
                    continue

            # If we get to here, we were unable to verify the state
            raise CommunicationError('Error setting the state of channels')

        def reset_switching_boards(self):
            '''
            If pin A9 (D23) is jumpered to the reset pins of the switching
            boards, this method provides a software reset.
            '''
            self.pin_mode(23, 1)
            self.digital_write(23, 0)
            self.digital_write(23, 1)

            # Wait long enough for the boards to reset and become addressable
            # again on the i2c bus (seems to take ~2.5 s based on empirical
            # testing)
            time.sleep(5)
            self.initialize_switching_boards()

        @property
        def baud_rate(self):
            return self.config['baud_rate']

        @baud_rate.setter
        def baud_rate(self, baud_rate):
            return self.update_config(baud_rate=baud_rate)

        @property
        def id(self):
            return self.config['id']

        @id.setter
        def id(self, id):
            return self.set_id(id)

        @property
        def uuid(self):
            '''
            Returns
            -------
            uuid.UUID
                UUID constructed from the `Unique Identification Register`_
                (12.2.19 page 265).

                .. _Unique Identification Register: https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf
            '''
            return uuid.UUID(bytes=np.array(self._uuid(),
                                            dtype='uint8').tostring())

        def _number_of_channels(self):
            return super(ProxyMixin, self).number_of_channels()

        @property
        def number_of_channels(self):
            '''
            .. versionchanged:: X.X.X
                Return number of channels cached during most recent
                initialization of switching boards.
            '''
            return self.__number_of_channels

        def detect_shorts(self, delay_ms=5):
            return super(ProxyMixin, self).detect_shorts(delay_ms).tolist()

        def _hardware_version(self):
            return super(ProxyMixin, self).hardware_version()

        @property
        def hardware_version(self):
            return self._hardware_version().tostring()

        @property
        def min_waveform_frequency(self):
            return self.config['min_frequency']

        @property
        def max_waveform_frequency(self):
            return self.config['max_frequency']

        @property
        def max_waveform_voltage(self):
            return self.config['max_voltage']

        @property
        def min_waveform_voltage(self):
            return float(super(ProxyMixin, self).min_waveform_voltage())

        @property
        def neighbours(self):
            channel_neighbours = super(ProxyMixin, self).neighbours()
            N = channel_neighbours.shape[0] / 4
            index = pd.MultiIndex.from_arrays([np.repeat(range(N), 4),
                                               ['up', 'down', 'left', 'right']
                                               * N])
            channel_neighbours = pd.Series(channel_neighbours, index=index)
            channel_neighbours.loc[channel_neighbours == 255] = np.nan
            return channel_neighbours

        @neighbours.setter
        def neighbours(self, value):
            self.assign_neighbours(value.fillna(-1).astype('uint8').values)

        @property
        def drops(self):
            return _unpack_drops(super(ProxyMixin, self).drops())

        def get_drops(self, channels=None, capacitance_threshold=0):
            '''
            Parameters
            ----------
            channels : list-like, optional
                If ``None``, detect drops across **all** channels.  Otherwise,
                only detect drops over electrodes connected to the specified
                channels.
            capacitance_threshold : float, optional
                Minimum capacitance (in farads) to consider as liquid present
                on a channel electrode.

                If set to 0, a default of 3 pF is used (default).

            Returns
            -------
            list<numpy.array>
                List of channels where threshold capacitance was met, grouped
                by contiguous electrode regions (i.e., sets of electrodes that
                are connected by neighbours where capacitance threshold was
                also met).
            '''
            if channels is None:
                drops_raw = (super(ProxyMixin, self)
                             .get_all_drops(capacitance_threshold))
            else:
                drops_raw = (super(ProxyMixin, self)
                             .get_channels_drops(channels,
                                                 capacitance_threshold))
            return _unpack_drops(drops_raw)


    class Proxy(ProxyMixin, _Proxy):
        pass

    class I2cProxy(ProxyMixin, _I2cProxy):
        pass

    class SerialProxy(ProxyMixin, _SerialProxy):
        #: .. versionadded:: 1.40
        device_name = 'dropbot'
        #: .. versionadded:: 1.40
        device_version = __version__

        def __init__(self, settling_time_s=.05, **kwargs):
            '''
            Parameters
            ----------
            settling_time_s : float, optional
                If specified, wait :data:`settling_time_s` seconds after
                establishing serial connection before trying to execute test
                command.

                By default, :data:`settling_time_s` is set to 50 ms.
            **kwargs
                Extra keyword arguments to pass on to
                :class:`base_node_rpc.proxy.SerialProxyMixin`.

            .. versionchanged:: 1.40
                Delegate automatic port selection to
                :class:`base_node_rpc.proxy.SerialProxyMixin`.
            '''
            kwargs['settling_time_s'] = settling_time_s
            super(SerialProxy, self).__init__(**kwargs)

        @property
        def port(self):
            try:
                port = self.serial_thread.protocol.port
            except Exception:
                port = None
            return port

        def flash_firmware(self):
            # currently, we're ignoring the hardware version, but eventually,
            # we will want to pass it to upload()
            self.terminate()
            upload()
            time.sleep(0.5)
            self._connect()

        def _reboot(self):
            '''
            .. versionadded:: 1.67

            Reboot DropBot control board.

            .. note:: **Connection is lost.**
            '''
            # Reboot to put device in known state.
            try:
                # XXX Temporarily disable timeout to avoid waiting for a
                # response after reboot command has been sent.
                original_timeout_s = self._timeout_s
                self._timeout_s = 0
                super(SerialProxy, self).reboot()
            except (serial.SerialException, IOError):
                pass
            finally:
                # Restore original timeout duration.
                self._timeout_s = original_timeout_s
                self.terminate()

        def reboot(self):
            '''
            .. versionchanged:: 1.27.1
                Temporarily disable timeout to avoid waiting for a response
                after reboot command has been sent.
            '''
            # Reboot to put device in known state.
            self._reboot()

            # Wait for serial port to settle after reboot.
            time.sleep(.5)

            # Reestablish serial connection to device.
            self._connect()


except (ImportError, TypeError):
    Proxy = None
    I2cProxy = None
    SerialProxy = None
