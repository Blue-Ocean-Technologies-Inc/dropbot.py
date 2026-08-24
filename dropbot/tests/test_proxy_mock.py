"""
Hardware-free unit tests for :mod:`dropbot.proxy`.

**No physical DropBot, no serial port and no firmware are required** — every
test in this module drives :class:`dropbot.proxy.ProxyMixin` through a stub
"node" class that stands in for the auto-generated
:class:`dropbot.node.Proxy` RPC layer.  The stubs record the RPC calls that
:class:`~dropbot.proxy.ProxyMixin` makes (``update_state``,
``assign_neighbours``, ``i2c_write``, ...) instead of sending NadaMQ packets,
so the tests exercise the *Python-side* convenience wrappers in isolation.

Contrast with the sibling modules (``test_actuation.py``, ``test_watchdog.py``,
``test_threads.py``, ``test_chip.py``) which do require a connected device.
"""
import logging
import threading

import numpy as np
import pandas as pd
import pytest

import base_node_rpc as bnr
import dropbot.proxy as dbp
from dropbot.proxy import ProxyMixin, _versions_match


# ---------------------------------------------------------------------------
# `neighbours` property round-trip
# ---------------------------------------------------------------------------
class _NeighbourNode:
    """Stub of the generated RPC layer for the ``neighbours`` commands."""

    def __init__(self):
        #: Raw packed array as the firmware would report it.
        self.raw = np.array([], dtype='uint8')
        #: Last array handed to the (stubbed) ``assign_neighbours`` command.
        self.assigned = None

    def neighbours(self):
        return self.raw

    def assign_neighbours(self, packed):
        self.assigned = np.asarray(packed)


class NeighbourProxy(ProxyMixin, _NeighbourNode):
    """``ProxyMixin`` bound to a stub node; never opens a serial port."""

    def __init__(self):
        _NeighbourNode.__init__(self)


#: Packed firmware representation: 3 channels x (up, down, left, right).
#: ``255`` is the firmware "no neighbour" sentinel.
PACKED_NEIGHBOURS = np.array([1, 2, 255, 3,
                              0, 255, 4, 5,
                              255, 255, 255, 0], dtype='uint8')


@pytest.fixture
def neighbour_proxy():
    proxy = NeighbourProxy()
    proxy.raw = PACKED_NEIGHBOURS.copy()
    return proxy


def test_neighbours_getter_maps_sentinel_to_nan(neighbour_proxy):
    """
    Guards the ``255`` -> ``NaN`` decode in the ``neighbours`` getter.

    The firmware reports ``255`` for "no neighbour"; leaking that raw value
    through would make ``255`` indistinguishable from channel 255 and would
    poison downstream arithmetic.
    """
    series = neighbour_proxy.neighbours

    assert isinstance(series, pd.Series)
    assert isinstance(series.index, pd.MultiIndex)
    # 3 channels x 4 directions.
    assert len(series) == 12
    assert list(series.index.get_level_values(1)[:4]) == ['up', 'down', 'left',
                                                          'right']
    expected_nan = np.asarray(PACKED_NEIGHBOURS == 255)
    assert np.array_equal(series.isna().values, expected_nan)
    # Non-sentinel entries survive untouched.
    kept = ~expected_nan
    assert np.array_equal(series.values[kept].astype('int64'),
                          PACKED_NEIGHBOURS[kept].astype('int64'))


def test_neighbours_setter_encodes_nan_as_sentinel(neighbour_proxy):
    """
    Guards the ``NaN`` -> ``255`` encode in the ``neighbours`` setter.

    The setter previously used ``fillna(-1).astype('uint8')``; ``pandas >= 2``
    refuses that cast ("cannot losslessly cast"), so the setter raised for
    virtually every input produced by
    :func:`dropbot.chip.get_channel_neighbours` (which nearly always leaves at
    least one ``NaN`` after its reindex).
    """
    neighbour_proxy.neighbours = neighbour_proxy.neighbours

    assert neighbour_proxy.assigned is not None
    assert neighbour_proxy.assigned.dtype == np.uint8
    assert np.array_equal(neighbour_proxy.assigned, PACKED_NEIGHBOURS)


def test_neighbours_setter_accepts_float_series_with_nan():
    """
    Guards the setter against the realistic ``float64``-with-``NaN`` input
    produced by :func:`dropbot.chip.get_channel_neighbours`.
    """
    index = pd.MultiIndex.from_product([[0, 1, 2],
                                        ['up', 'down', 'left', 'right']])
    value = pd.Series([1., 2., np.nan, 3.,
                       0., np.nan, 4., 5.,
                       np.nan, np.nan, np.nan, 0.],
                      index=index, dtype='float64')

    proxy = NeighbourProxy()
    proxy.neighbours = value

    assert proxy.assigned.dtype == np.uint8
    assert np.array_equal(proxy.assigned, PACKED_NEIGHBOURS)


# ---------------------------------------------------------------------------
# `measure_input_voltage()` high-voltage safety
# ---------------------------------------------------------------------------
class _HvNode:
    """Stub node exposing just what ``measure_input_voltage()`` touches."""

    #: Raised by `analog_reads_simple()` when set (exception-path tests).
    read_error = None

    def analog_reads_simple(self, pin, n):
        if self.read_error is not None:
            raise self.read_error
        return np.full(n, 1000.0)

    def min_waveform_voltage(self):
        return 1.0


class HvProxy(ProxyMixin, _HvNode):
    """
    ``ProxyMixin`` over an in-memory state ``Series``.

    Every state mutation is appended to :attr:`updates` so tests can assert
    the *order* in which the high-voltage output is restored.
    """

    def __init__(self, hv_output_enabled, hv_output_selected, voltage=100.):
        self.transaction_lock = threading.RLock()
        self._state = pd.Series({'voltage': float(voltage),
                                 'frequency': 10000.,
                                 'hv_output_enabled': bool(hv_output_enabled),
                                 'hv_output_selected': bool(hv_output_selected)})
        #: Ordered log of ``update_state()`` keyword sets.
        self.updates = []

    @property
    def state(self):
        return self._state.copy()

    @state.setter
    def state(self, value):
        self.update_state(**value.to_dict())

    def update_state(self, **kwargs):
        for key, value in kwargs.items():
            self._state[key] = value
        self.updates.append(set(kwargs))

    @property
    def min_waveform_voltage(self):
        return 1.

    def snapshot(self):
        return (bool(self._state['hv_output_enabled']),
                bool(self._state['hv_output_selected']),
                float(self._state['voltage']))


@pytest.fixture
def no_sleep(monkeypatch):
    """Collapse the ~7 s of settling delays in ``measure_input_voltage()``."""
    monkeypatch.setattr(dbp.time, 'sleep', lambda seconds: None)


@pytest.mark.parametrize('hv_enabled, hv_selected', [(False, False),
                                                     (False, True),
                                                     (True, False),
                                                     (True, True)])
def test_measure_input_voltage_restores_hv_state(no_sleep, hv_enabled,
                                                 hv_selected):
    """
    Guards the high-voltage restore *order* in ``measure_input_voltage()``.

    The ``voltage`` setter force-enables **and** force-selects the HV output,
    so restoring ``hv_output_enabled`` before ``voltage`` (the historical
    order) left the DropBot energised for callers that had HV switched off.
    The final state must match the state captured on entry.
    """
    proxy = HvProxy(hv_enabled, hv_selected, voltage=100.)

    result = proxy.measure_input_voltage()

    assert np.isfinite(result)
    assert proxy.snapshot() == (hv_enabled, hv_selected, 100.)
    # The very last write must be the `hv_output_enabled` restore, i.e. it
    # happens *after* the `voltage` restore re-energised the output.
    assert proxy.updates[-1] == {'hv_output_enabled'}
    assert proxy.updates[-2] == {'hv_output_selected'}


@pytest.mark.parametrize('hv_selected', [False, True])
def test_measure_input_voltage_leaves_hv_off_on_error(no_sleep, hv_selected):
    """
    Guards the fail-safe direction of the exception path.

    ``measure_input_voltage()`` de-energises the output *before* sampling.  If
    the sample RPC fails, the restore never runs -- the device must therefore
    be left with HV **off**, never energised.
    """
    proxy = HvProxy(False, hv_selected, voltage=100.)
    proxy.read_error = IOError('serial read failed')

    with pytest.raises(IOError):
        proxy.measure_input_voltage()

    assert not bool(proxy._state['hv_output_enabled'])
    # The last write to reach the device was the de-energising one.
    assert proxy.updates[-1] == {'hv_output_enabled'}


# ---------------------------------------------------------------------------
# `_versions_match()` truth table
# ---------------------------------------------------------------------------
@pytest.mark.parametrize('driver, device, expected, warns', [
    # Exactly-equal strings short-circuit to a match, with no warning.
    ('1.78.0', '1.78.0', True, False),
    ('0+unknown', '0+unknown', True, False),
    # Base-version comparison for plain releases.
    ('1.78.0', '1.77.0', False, False),
    ('1.78.0', '1.78.1', False, False),
    ('1.78.0', '2.0.0', False, False),
    # `versioneer` local segments (untagged working tree) -> tolerated.
    ('1.78.0+7.gdeadbee.dirty', '1.78.0', True, True),
    ('1.78.0', '1.78.0+7.gdeadbee', True, True),
    ('0+unknown', '1.78.0', True, True),
    # Development releases -> tolerated.
    ('1.78.0.dev3', '1.78.0', True, True),
    ('1.78.0', '1.78.0.dev3', True, True),
    # Unparseable versions -> tolerated.
    ('not-a-version', '1.78.0', True, True),
    ('1.78.0', 'not-a-version', True, True),
    (None, '1.78.0', True, True),
    ('1.78.0', None, True, True),
])
def test_versions_match_truth_table(caplog, driver, device, expected, warns):
    """
    Guards the tolerant driver/firmware version comparison.

    A strict string comparison made a locally regenerated development build
    unable to talk to the firmware it was just flashed from; a fully permissive
    comparison hid genuine release-vs-release mismatches.  Both failure modes
    are pinned here.
    """
    caplog.set_level(logging.WARNING)

    assert _versions_match(driver, device) is expected

    warnings_logged = [record for record in caplog.records
                       if record.levelno >= logging.WARNING]
    assert bool(warnings_logged) is warns


class _VersionNode:
    """Minimal stand-in for the attributes ``_check_device_version()`` reads."""

    def __init__(self, driver_version, device_version):
        self.device_version = driver_version
        self.properties = pd.Series({'software_version': device_version}
                                    if device_version is not None else {},
                                    dtype=object)


def test_check_device_version_raises_on_release_mismatch():
    """
    Guards the firmware version gate that ``SerialProxy`` bypasses.

    ``dropbot.proxy.SerialProxy`` drives the serial monitor directly and so
    never runs ``SerialProxyMixin._connect()``'s version check;
    ``_check_device_version()`` restores it.
    """
    node = _VersionNode('1.78.0', '1.77.0')

    with pytest.raises(bnr.proxy.DeviceVersionMismatch) as excinfo:
        ProxyMixin._check_device_version(node, [])

    assert excinfo.value.device_version == '1.77.0'


def test_check_device_version_honours_ignore_list(caplog):
    """
    Guards ``ignore=[DeviceVersionMismatch]`` staying meaningful for
    ``dropbot.proxy.SerialProxy``: a mismatch must warn instead of raising.
    """
    caplog.set_level(logging.WARNING)
    node = _VersionNode('1.78.0', '1.77.0')

    ProxyMixin._check_device_version(node,
                                     [bnr.proxy.DeviceVersionMismatch])

    assert any('does not match' in record.getMessage()
               for record in caplog.records)


@pytest.mark.parametrize('driver, device', [('1.78.0', '1.78.0'),
                                            ('1.78.0', None)])
def test_check_device_version_accepts(driver, device):
    """
    Guards the two non-raising paths: an exact match, and a device that
    reports no ``software_version`` at all (older firmware).
    """
    ProxyMixin._check_device_version(_VersionNode(driver, device), [])


def test_ignore_true_covers_device_version_mismatch():
    """
    Guards ``ignore=True`` expanding to include
    :class:`base_node_rpc.proxy.DeviceVersionMismatch` alongside
    :class:`~dropbot.proxy.NoPower` and
    :class:`~dropbot.proxy.I2cAddressNotSet`.
    """
    import inspect

    source = inspect.getsource(ProxyMixin.__init__)
    for name in ('NoPower', 'I2cAddressNotSet', 'DeviceVersionMismatch'):
        assert name in source


# ---------------------------------------------------------------------------
# `bytes` accepted by the I2C helpers
# ---------------------------------------------------------------------------
class _I2cNode:
    """Stub node recording ``i2c_write``/``i2c_read`` RPC traffic."""

    def __init__(self):
        self.writes = []
        self.reads = []

    def i2c_write(self, address, data):
        # The firmware RPC takes an array of bytes; normalize for comparison.
        self.writes.append((address, list(np.atleast_1d(data))))

    def i2c_read(self, address, count):
        return self.reads.pop(0)


class I2cProxy(ProxyMixin, _I2cNode):
    def __init__(self):
        _I2cNode.__init__(self)


@pytest.mark.parametrize('payload', [bytes([1, 2, 3]),
                                     bytearray([1, 2, 3]),
                                     [1, 2, 3],
                                     np.array([1, 2, 3], dtype='uint8')],
                         ids=['bytes', 'bytearray', 'list', 'ndarray'])
def test_i2c_eeprom_write_accepts_bytes_like(payload):
    """
    Guards the bytes/str boundary in ``i2c_eeprom_write()``.

    Slicing ``bytes`` yields ``bytes``, which cannot be concatenated to the
    ``[eeprom_address]`` list -- so the ``bytes`` input advertised by the type
    hint raised ``TypeError`` until the payload was normalized via ``list()``.
    """
    proxy = I2cProxy()

    proxy.i2c_eeprom_write(0x50, 0x10, payload)

    assert proxy.writes == [(0x50, [0x10, 1, 2, 3])]


def test_i2c_eeprom_write_chunks_at_16_bytes():
    """
    Guards the automatic 16-byte chunking (the maximum I2C packet size) and
    the per-chunk EEPROM address offset.
    """
    proxy = I2cProxy()
    payload = bytes(range(20))

    proxy.i2c_eeprom_write(0x50, 0x00, payload)

    assert len(proxy.writes) == 2
    assert proxy.writes[0] == (0x50, [0x00] + list(range(16)))
    assert proxy.writes[1] == (0x50, [16] + list(range(16, 20)))


def test_i2c_send_command_accepts_bytes():
    """
    Guards the same bytes/str boundary in ``i2c_send_command()``, whose
    ``data`` argument is likewise type-hinted as ``bytes``.
    """
    proxy = I2cProxy()
    proxy.reads = [np.array([2], dtype='uint8'),
                   np.array([9, 8], dtype='uint8')]

    result = proxy.i2c_send_command(0x50, 7, bytes([1, 2]))

    assert proxy.writes == [(0x50, [7, 1, 2])]
    assert np.array_equal(result, np.array([9, 8], dtype='uint8'))


def test_i2c_eeprom_read_concatenates_chunks():
    """
    Guards ``i2c_eeprom_read()`` splitting reads longer than the 16-byte I2C
    packet size and re-joining them into a single array.
    """
    proxy = I2cProxy()
    proxy.reads = [np.arange(16, dtype='uint8'),
                   np.arange(16, 20, dtype='uint8')]

    result = proxy.i2c_eeprom_read(0x50, 0x00, 20)

    assert np.array_equal(result, np.arange(20, dtype='uint8'))
    # One address write per chunk, offset by the chunk start.
    assert [write[1][0] for write in proxy.writes] == [0x00, 16]
