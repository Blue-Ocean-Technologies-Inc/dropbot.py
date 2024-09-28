# coding: utf-8
import time
import math
import uuid
import pint
import serial
import logging
import threading

import datetime as dt
import numpy as np
import pandas as pd

import base_node_rpc as bnr

from typing import Optional, Union, List

# Load protobuf files before loading other rpc modules!
from .config import Config
from .state import State

from path_helpers import path
from nadamq.NadaMq import cPacket
from logging_helpers import _L
from teensy_minimal_rpc.adc_sampler import AdcDmaMixin
from base_node_rpc.proxy import ConfigMixinBase, StateMixinBase

from .core import dropbot_state, NOMINAL_ON_BOARD_CALIBRATION_CAPACITORS
from .node import Proxy
from .bin.upload import upload

from ._version import get_versions

__version__ = get_versions()['version']
del get_versions


# Unit conversion
ureg = pint.UnitRegistry()

# Event mask flags
EVENT_ACTUATED_CHANNEL_CAPACITANCES = (1 << 31)
EVENT_CHANNELS_UPDATED = (1 << 30)
EVENT_SHORTS_DETECTED = (1 << 28)
EVENT_DROPS_DETECTED = (1 << 27)
EVENT_ENABLE = (1 << 0)

I2cProxy = None


class I2cAddressNotSet(Exception):
    pass


class NoPower(Exception):
    pass


class CommunicationError(Exception):
    pass


class ConfigMixin(ConfigMixinBase):

    @property
    def config_class(self):
        return Config


class StateMixin(StateMixinBase):
    @property
    def state_class(self):
        return State


class ProxyMixin(ConfigMixin, StateMixin, AdcDmaMixin):
    """
    Mixin class to add convenience wrappers around methods of the generated
    `node.Proxy` class.
    """
    host_package_name = str(path(__file__).parent.name.replace('_', '-'))

    def __init__(self, *args, **kwargs):
        """
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

        Version log
        -----------
        . versionchanged:: 1.27
            Add ``ignore`` parameter.

        .. versionchanged:: 1.34.1
            Add message to :class:`NoPower` exception.

        .. versionchanged:: 1.55
            Synchronize device millisecond counter to UTC time upon making
            a connection (and on any subsequent *reconnection*).
        """
        self.transaction_lock = threading.RLock()
        self.__number_of_channels = 0

        try:
            # Get a list of exception types to ignore.
            #
            # XXX This allows, for example:
            #  - Testing connection to a DropBot control board that is not
            #    connected to a power source.
            #  - Connecting to a DropBot without a configured I2C address
            #    to set an I2C address.
            super().__init__(*args, **kwargs)
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
                raise NoPower('Please check that 12V power supply is connected.')

            # Only initialize switching boards if the control board has been assigned an i2c address.
            if self.config.i2c_address != 0:
                self.initialize_switching_boards()
            elif I2cAddressNotSet not in ignore:
                raise I2cAddressNotSet()

            # Synchronize device millisecond counter to UTC time upon connection.
            self.signals.signal('connected').connect(lambda *args: self.sync_time(), weak=False)

            self.signals.signal('connected').send({'event': 'connected'})
        except Exception:
            _L().debug('Error connecting to device.', exc_info=True)
            self.terminate()
            raise

    def _initialize_switching_boards(self) -> int:
        return super(ProxyMixin, self).initialize_switching_boards()

    def initialize_switching_boards(self) -> int:
        """
        Version log
        -----------
        .. versionadded:: 1.71.0
            Wrap parent :meth:`initialize_switching_boards()` to cache
            number of available channels.
        """
        self.__number_of_channels = self._initialize_switching_boards()
        return self.__number_of_channels

    def _connect(self, *args, **kwargs) -> None:
        """
        Version log
        -----------
        .. versionadded:: 1.55

            Send ``connected`` event each time a connection has been
            established. Note that the first ``connected`` event is sent
            before any receivers have a chance to connect to the signal,
            but subsequent restored connection events after connecting to
            the ``connected`` signal will be received.
        """
        super(ProxyMixin, self)._connect(*args, **kwargs)
        self.signals.signal('connected').send({'event': 'connected'})

    def sync_time(self) -> None:
        """
        Synchronize device millisecond counter to UTC time.

        Version log
        -----------
        .. versionadded:: 1.55
        """
        utc_timestamp = dt.datetime.now(dt.timezone.utc).timestamp()
        super(ProxyMixin, self).sync_time(utc_timestamp)

    @property
    def wall_time(self) -> dt.datetime:
        """
        Device UTC wall-clock time.

        Version log
        -----------
        .. versionadded:: 1.55
        """
        wall_time = super(ProxyMixin, self).wall_time()
        return dt.datetime.fromtimestamp(wall_time, dt.timezone.utc)

    @property
    def signals(self):
        """
        Version log
        -----------
        .. versionadded:: 1.43
        """
        return self._packet_queue_manager.signals

    def i2c_eeprom_write(self, i2c_address: int, eeprom_address: int, data: bytes) -> None:
        """
        Write data to specified address in I2C EEPROM chip.

        If the number of bytes exceeds the maximum I2C packet size,
        multiple requests will be made automatically.

        Parameters
        ----------
        i2c_address : int
            Address of ``CAT24Cxx`` EEPROM.

            See: https://www.onsemi.com/pub/Collateral/CAT24C01-D.PDF
        eeprom_address : int
            Address to write data to in EEPROM.
        data : list-like
            Bytes to write to :data:`eeprom_address`
        """
        i2c_packet_size = 16
        for i in range(int(math.ceil(len(data) / float(i2c_packet_size)))):
            start = i * i2c_packet_size
            end = min((i + 1) * i2c_packet_size, len(data))
            self.i2c_write(i2c_address, [eeprom_address + start] +
                           (end - start) * [255])
            self.i2c_write(i2c_address, [eeprom_address + start] +
                           data[start:end])

    def i2c_eeprom_read(self, i2c_address: int, eeprom_address: int, length: int) -> np.array:
        """
        Read a specified number of bytes from I2C EEPROM chip starting at the
        address given.

        If the number of requested bytes exceeds the maximum I2C packet
        size, multiple requests will be made automatically.

        Parameters
        ----------
        i2c_address : int
            Address of ``CAT24Cxx`` EEPROM.

            See: https://www.onsemi.com/pub/Collateral/CAT24C01-D.PDF
        eeprom_address : int
            Address to write data to in EEPROM.
        length : int
            Number of bytes to read.
        """
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

    def __del__(self) -> None:
        try:
            # turn off the high voltage when we disconnect
            self.hv_output_enabled = False
            super().__del__()
        except Exception:
            # ignore any exceptions (e.g., if we can't communicate with the board)
            _L().debug('Communication error', exc_info=True)
            pass

    def i2c_send_command(self, address: int, cmd: bytes, data: bytes) -> bytes:
        self.i2c_write(address, [cmd] + data)
        n = self.i2c_read(address, 1)
        return self.i2c_read(address, n)

    def on_board_capacitance(self) -> pd.Series:
        """
        Measure no actuation load and each on-board test capacitor.

        See on-board capacitors schematic [here][1].

        [1]: https://gitlab.com/sci-bots/dropbot-control-board.kicad/blob/77cd712f4fe4449aa735749f46212b20d290684e/pdf/on-board-calibration-On-board%20calibration.pdf

        Returns
        -------
        pd.Series
            Measured capacitance of each on-board test capacitor, indexed
            by nominal capacitance.

        Version log
        -----------
        .. versionadded:: 1.73.0
        """
        with dropbot_state(self):
            self.turn_off_all_channels()
            self.voltage = 50

            C_i = []

            for c_i in [-1, 0, 1, 2]:
                self.select_on_board_test_capacitor(c_i)
                C_i.append(self.capacitance(0))

            self.select_on_board_test_capacitor(-1)

            return pd.Series(C_i, index=NOMINAL_ON_BOARD_CALIBRATION_CAPACITORS)

    def channel_capacitances(self, channels) -> pd.Series:
        """
        Wrap C++ method to return :class:`pandas.Series`.

        Parameters
        ----------
        channels : list-like
            List of channels for which to measure capacitance.
        Returns
        -------
        pd.Series
            Capacitance of each channel indexed by channel ID.

        Version log
        -----------
        .. versionadded:: 1.73.0
        """
        return pd.Series(super().channel_capacitances(channels), index=channels)

    def reset_C16(self) -> Union[int, float]:
        """
        Reset ``C16`` to default.

        See [issue #42][i42] for more info.

        [i42]: https://gitlab.com/sci-bots/dropbot.py/issues/42

        Version log
        -----------
        .. versionadded:: 1.73.0
        """
        default_C16 = Config.DESCRIPTOR.fields_by_name['C16'].default_value
        self.update_config(C16=default_C16)
        logging.info(f"Reset `C16` as {(default_C16 * ureg.F).to('pF'):.1f}")
        return default_C16

    def calibrate_C16(self, n_samples: Optional[int] = 10, reset: Optional[bool] = True) -> Union[int, float]:
        """
        Calibrate ``C16`` using on-board test capacitors.

        Calibration procedure::

         1. Measure each on-board capacitor the specified number of times.
         2. Compute a correction factor for each on-board capacitor.
         3. Set ``C16`` to the average correction factor.

        See [issue #42][i42] for more info.

        [i42]: https://gitlab.com/sci-bots/dropbot.py/issues/42

        Parameters
        ----------
        n_samples : int, optional
            Number of measurements per on-board capacitor.
        reset : bool, optional
            If `True`, set ``C16`` to default before performing
            calibration.

        Returns
        -------
        float
            Calibrated ``C16`` value in farads.

        Version log
        -----------
        .. versionadded:: 1.73.0
        """
        if reset:
            self.reset_C16()

        init_C16 = self.config.C16
        nominal_C = NOMINAL_ON_BOARD_CALIBRATION_CAPACITORS[NOMINAL_ON_BOARD_CALIBRATION_CAPACITORS > 0]
        df_C = pd.DataFrame(self.on_board_capacitance() for i in range(n_samples))
        correction = df_C.median()[nominal_C] / nominal_C.values
        logging.debug(f"Correction factors relative to `C16` = {(init_C16 * ureg.F).to('pF'):.1f}:")
        map(logging.debug, str(correction).splitlines())
        C16_ = self.config.C16 / correction.mean()
        self.update_config(C16=C16_)
        logging.info(f"Calibrated `C16` as {(C16_ * ureg.F).to('pF'):1f}")
        return C16_

    def measure_capacitance(self, n_samples: Optional[int] = 50, amplitude: str = 'filtered_mean') -> float:
        """
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
        """
        df_volts = pd.DataFrame({'volts': self.analog_reads_simple(11, n_samples) * 3.3 / 2 ** 16})

        singleton = False

        if isinstance(amplitude, str):
            amplitude = [amplitude]
            singleton = True

        values = []
        for method_i in amplitude:
            if method_i == 'filtered_mean':
                v_gnd = np.mean(df_volts)
                v_abs = np.abs(df_volts - v_gnd)
                v_abs_mean = np.mean(v_abs)
                filter_th = v_abs_mean * 1.5
                amplitude_i = np.mean(v_abs[v_abs < filter_th])#['volts']
            elif method_i == 'percentile_difference':
                volts_description = df_volts['volts'].describe()
                amplitude_i = .5 * (volts_description['75%'] -
                                    volts_description['25%'])
            else:
                raise NameError(f'Unknown amplitude calculation method `{method_i}`.')
            values.append(amplitude_i)

        if not singleton:
            value = pd.Series(values, index=amplitude)
        else:
            value = values[0]
        return value / self.measure_voltage() * self.config.C16

    def get_environment_state(self, i2c_address: int = 0x27) -> pd.Series:
        """
        Acquire temperature and humidity from `Honeywell HIH6000`_ series
        sensor.

        .. _Honeywell HIH6000: http://sensing.honeywell.com/index.php/ci_id/142171/la_id/1/document/1/re_id/0
        """

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
    def frequency(self) -> Union[int, float]:
        return self.state['frequency']

    @frequency.setter
    def frequency(self, value: float) -> None:
        self.update_state(frequency=value)

    @property
    def voltage_limit(self) -> float:
        """
        Version log
        -----------
        .. versionadded:: 1.39
        """
        return 1.5 / 2.0 * (2e6 / self.config['R7'] + 1)

    def measure_voltage(self) -> float:
        # divide by 2 to convert from peak-to-peak to rms
        return self.analog_read(1) / 2.0 ** 16 * 3.3 * 2e6 / 20e3 / 2.0

    def measure_input_current(self, n: Optional[int] = 2000) -> dict:
        i = self.analog_reads_simple(3, n) / 2.0 ** 16 * 3.3 / 0.03
        results = dict(rms=np.mean(i), max=np.max(i))
        return results

    def measure_output_current(self, n: Optional[int] = 2000) -> dict:
        current = (self.analog_reads_simple(2, n) / 2.0 ** 16 * 3.3 /
                   (51e3 / 5.1e3 * 1))
        results = dict(rms=np.mean(current), max=np.max(current))
        return results

    def measure_input_voltage(self) -> float:
        # save the state of the output voltage
        hv_output_enabled = self.hv_output_enabled
        voltage = self.voltage

        # set the voltage to the minimum and wait for it to settle
        self.voltage = self.min_waveform_voltage
        time.sleep(1)

        # disable the boost converter and wait for it to settle
        self.hv_output_enabled = False
        time.sleep(5)

        # take a measurement
        v = self.analog_reads_simple(1, 2000) / 2.0 ** 16 * 3.3 * 2e6 / 20e3

        # restore the output voltage and let it settle
        self.hv_output_enabled = hv_output_enabled
        self.voltage = voltage
        time.sleep(1)
        return np.sqrt(np.mean(v ** 2))

    @property
    def voltage(self) -> float:
        return self.state['voltage']

    @voltage.setter
    def voltage(self, value) -> None:
        """
        Version log
        -----------
        .. versionchanged:: 1.66
            Select and enable high-voltage output if it is not currently
            enabled.
        """
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
    def hv_output_enabled(self) -> bool:
        return self.state['hv_output_enabled']

    @hv_output_enabled.setter
    def hv_output_enabled(self, value: bool) -> None:
        self.update_state(hv_output_enabled=value)

    @property
    def hv_output_selected(self) -> bool:
        return self.state['hv_output_selected']

    @hv_output_selected.setter
    def hv_output_selected(self, value: bool) -> None:
        self.update_state(hv_output_selected=value)

    def _disabled_channels_mask(self) -> np.array:
        """
        Version log
        -----------
        .. versionadded:: 1.34

        Prepend underscore to the auto-generated disabled_channels_mask accessor
        """
        return super(ProxyMixin, self).disabled_channels_mask()

    def _set_disabled_channels_mask(self, mask: np.array) -> np.array:
        """
        Version log
        -----------
        .. versionadded:: 1.34

        Prepend underscore to the auto-generated disabled_channels_mask setter
        """
        return super(ProxyMixin, self).set_disabled_channels_mask(mask)

    @property
    def disabled_channels_mask(self) -> np.array:
        """
        Version log
        -----------
        .. versionadded:: 1.34

        Retrieve the mask bytes from the device and unpack them into an
        array with one entry per channel.  Return unpacked array.

        Notes
        -----
        State of mask for each channel is binary, 0 or 1.  On device,
        mask states are stored in bytes, where each byte corresponds to
        the mask state for eight channels.
        """
        return np.unpackbits(super(ProxyMixin, self).disabled_channels_mask()[::-1])[::-1]

    @disabled_channels_mask.setter
    def disabled_channels_mask(self, mask: np.array) -> None:
        self.set_disabled_channels_mask(mask)

    def set_disabled_channels_mask(self, mask: np.array) -> None:
        """
        Version log
        -----------
        .. versionadded:: 1.34

        Pack array containing one entry per channel to bytes (8 channels
        per byte).  Set disabled channels mask on device using mask bytes.

        See also: `disabled_channels_mask` (get)
        """
        if len(mask) != self.number_of_channels:
            raise ValueError('Error setting disabled channels mask.  Check '
                             'size of mask matches channel count.')
        super(ProxyMixin, self).set_disabled_channels_mask(np.packbits(mask.astype(int)[::-1])[::-1])

    def _state_of_channels(self) -> np.array:
        """
        Prepend underscore to the auto-generated state_of_channels accessor
        """
        return super(ProxyMixin, self).state_of_channels()

    def _set_state_of_channels(self, states: np.array) -> None:
        """
        Prepend underscore to the auto-generated state_of_channels setter
        """
        return super(ProxyMixin, self).set_state_of_channels(states)

    @property
    def state_of_channels(self) -> pd.Series:
        """
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

        Version log
        -----------
        .. versionchanged:: 1.56
            Return channels as `pandas.Series` instance.
        """
        return pd.Series(np.unpackbits(super(ProxyMixin, self).state_of_channels()[::-1])[::-1])

    @state_of_channels.setter
    def state_of_channels(self, states: np.array) -> None:
        self.set_state_of_channels(states)

    def set_state_of_channels(self, states: np.array, append: Optional[bool] = True,
                              verify: Optional[bool] = True) -> None:
        """
        Pack array containing one entry per channel to bytes (8 channels per byte).
        Set state of channels on device using state bytes.

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
        verify : bool, optional
            If `True`, read channel states back from DropBot and verify
            against requested states.

        Raises
        ------
        CommunicationError
            If channel states read back from DropBot do not match the
            requested states.

        Version log
        -----------
        .. versionadded:: 1.73.2
        .. versionchanged:: 1.71.0
            Add ``append`` keyword argument.
        .. versionchanged:: 1.73.2
            Add ``verify`` keyword argument.  Remove call to deprecated
            :meth:`reset_switching_boards()`.
        """
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
        if not super(ProxyMixin, self).set_state_of_channels(state_bits):
            raise ValueError('Error setting state of channels.  Check number of states matches channel count.')

    def reset_switching_boards(self) -> DeprecationWarning:
        """
        Version log
        -----------
        .. deprecated:: 1.73.2
            Not supported by DropBot v3 hardware.
        """
        raise DeprecationWarning()

    @property
    def baud_rate(self) -> int:
        return self.config.baud_rate

    @baud_rate.setter
    def baud_rate(self, baud_rate: int) -> None:
        self.update_config(baud_rate=baud_rate)

    @property
    def id(self) -> str:
        return self.config.id

    @id.setter
    def id(self, id_: str) -> None:
        self.set_id(id_)

    @property
    def uuid(self) -> uuid:
        """
        Returns
        -------
        uuid.UUID
            UUID constructed from the `Unique Identification Register`_
            (12.2.19 page 265).

            .. _Unique Identification Register: https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf
        """
        return uuid.UUID(bytes=np.array(self._uuid(), dtype='uint8').tostring().decode('utf-8'))

    def _number_of_channels(self) -> np.array:
        return super(ProxyMixin, self).number_of_channels()

    @property
    def number_of_channels(self) -> int:
        """
        Version log
        -----------
        .. versionchanged:: 1.71.0
            Return number of channels cached during most recent
            initialization of switching boards.
        """
        return self.__number_of_channels

    def detect_shorts(self, delay_ms: Optional[int] = 5) -> List:
        return super(ProxyMixin, self).detect_shorts(delay_ms).tolist()

    def _hardware_version(self) -> np.array:
        return super(ProxyMixin, self).hardware_version()

    @property
    def hardware_version(self) -> str:
        return self._hardware_version().tostring().decode('utf-8')

    @property
    def min_waveform_frequency(self) -> float:
        return self.config['min_frequency']

    @property
    def max_waveform_frequency(self) -> float:
        return self.config['max_frequency']

    @property
    def max_waveform_voltage(self) -> float:
        return self.config['max_voltage']

    @property
    def min_waveform_voltage(self) -> float:
        return float(super(ProxyMixin, self).min_waveform_voltage())

    @property
    def neighbours(self) -> pd.Series:
        channel_neighbours = super(ProxyMixin, self).neighbours()
        N = channel_neighbours.shape[0] / 4
        index = pd.MultiIndex.from_arrays([np.repeat(range(N), 4),
                                           ['up', 'down', 'left', 'right']
                                           * N])
        channel_neighbours = pd.Series(channel_neighbours, index=index)
        channel_neighbours.loc[channel_neighbours == 255] = np.nan
        return channel_neighbours

    @neighbours.setter
    def neighbours(self, value: pd.Series) -> None:
        self.assign_neighbours(value.fillna(-1).astype('uint8').values)

    @property
    def drops(self) -> List[np.array]:
        return _unpack_drops(super(ProxyMixin, self).drops())

    def get_drops(self, channels: Optional[np.array] = None,
                  capacitance_threshold: Optional[float] = 0) -> List[np.array]:
        """
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
        """
        if channels is None:
            drops_raw = super(ProxyMixin, self).get_all_drops(capacitance_threshold)
        else:
            drops_raw = super(ProxyMixin, self).get_channels_drops(channels, capacitance_threshold)
        return _unpack_drops(drops_raw)


def _unpack_drops(packed_drops: np.array) -> List[np.array]:
    """
    Unpack raw drops array format:

        [drop 0 channel count][drop 0: channel 0, channel 1, ...][drop 1 channel count][drop 1: channel 0, channel 1, ...]

    into a list of `numpy.array` channels lists.


    Returns
    -------
    list<numpy.array>
        List of channels lists, each channel list corresponding to channels
        covered by a drop.
    """
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


class SerialProxy(ProxyMixin, Proxy):
    device_name = 'dropbot'
    device_version = __version__

    def __init__(self, settling_time_s: Optional[float] = .05, **kwargs):
        """
        Parameters
        ----------
        settling_time_s: float, optional
            If specified, wait :data:`settling_time_s` seconds after
            establishing serial connection before trying to execute test
            command.

            By default, :data:`settling_time_s` is set to 50 ms.
        **kwargs
            Extra keyword arguments to pass on to
            :class:`base_node_rpc.proxy.SerialProxyMixin`.

        Version log
        -----------
        . versionchanged:: 1.40
            Delegate automatic port selection to
            :class:`base_node_rpc.proxy.SerialProxyMixin`.
        """
        self.default_timeout = kwargs.pop('timeout', 5)
        self.monitor = None
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
        self.connect()
        super(SerialProxy, self).__init__(**kwargs)

    @property
    def signals(self):
        return self.monitor.signals

    def connect(self):
        self.terminate()
        monitor = bnr.ser_async.BaseNodeSerialMonitor(port=self.port)
        monitor.start()
        monitor.connected_event.wait()
        self.monitor = monitor
        return self.monitor

    def _send_command(self, packet: cPacket, timeout_s: Optional[float] = None, **kwargs):
        if timeout_s is None:
            timeout_s = self.default_timeout
        _L().debug(f'Using timeout {timeout_s}')
        return self.monitor.request(packet.tostring(), timeout=timeout_s)

    def terminate(self) -> None:
        if self.monitor is not None:
            self.monitor.stop()

    def __enter__(self) -> 'SerialProxy':
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.terminate()

    def __del__(self) -> None:
        self.terminate()

    def flash_firmware(self) -> None:
        # currently, we're ignoring the hardware version, but eventually,
        # we will want to pass it to upload()
        self.terminate()
        try:
            upload()
        except Exception:
            _L().debug('Error updating firmware', exc_info=True)
        time.sleep(0.5)
        self.connect()

    # TODO: Implement reboot
    # TODO: Currently reboot is not working because of async serial connection
    # def _reboot(self):
    #     """
    #     Version log
    #     -----------
    #     .. versionadded:: 1.67
    #
    #     Reboot DropBot control board.
    #
    #     .. note:: **Connection is lost.**
    #     """
    #     # Reboot to put the device in known state.
    #     try:
    #         # XXX Temporarily disable timeout to avoid waiting for a
    #         # response after reboot command has been sent.
    #         original_timeout_s = self._timeout_s
    #         self._timeout_s = 0
    #         super(SerialProxy, self).reboot()
    #     except (serial.SerialException, IOError):
    #         pass
    #     finally:
    #         # Restore original timeout duration.
    #         self._timeout_s = original_timeout_s
    #         self.terminate()

    
    # def reboot(self):
    #     """
    #     Version log
    #     -----------
    #     .. versionchanged:: 1.27.1
    #         Temporarily disable timeout to avoid waiting for a response
    #         after reboot command has been sent.
    #     """
    #     # Reboot to put the device in a known state.
    #     self._reboot()
    #
    #     # Wait for serial port to settle after reboot.
    #     time.sleep(.5)
    #
    #     # Reestablish serial connection to the device.
    #     self._connect()
