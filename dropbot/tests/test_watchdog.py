import logging
import time

import pytest
import serial


def _reset_proxy(proxy):
    # Reboot to put device in known state.
    try:
        proxy.reboot()
    except serial.SerialException:
        pass
    finally:
        proxy.terminate()

    # Wait for serial port to settle after reboot.
    time.sleep(.5)

    # Reestablish serial connection to device.
    proxy._connect()


@pytest.fixture(scope='module')
def proxy():
    import dropbot

    proxy_ = dropbot.SerialProxy()
    yield proxy_
    proxy_.terminate()


@pytest.mark.parametrize('retry_count', [1, 5])
def test_disable(proxy, retry_count):
    _reset_proxy(proxy)
    # XXX Executing `watchdog_disable` method results in undefined behaviour.
    # XXX Rebooting and retrying to disable the watchdog seems to help.
    # Empirically, disable test seems to pass on attempt 1-3.
    #
    # See [issue 4][i4].
    #
    # [i4]: https://gitlab.com/sci-bots/dropbot.py/issues/4
    WDOG_STCTRLH_WDOGEN = 0x01

    for i in xrange(retry_count):
        proxy.watchdog_disable()
        timer_output = proxy.watchdog_timer_output()
        if timer_output == 0:
            print 'Disabled on attempt', i + 1
            break
        else:
            _reset_proxy(proxy)
    else:
        assert timer_output == 0

    WDOG_STCTRLH = proxy.R_WDOG_STCTRLH()
    assert not (WDOG_STCTRLH & WDOG_STCTRLH_WDOGEN)


def test_time_out(proxy):
    start_reset_count = proxy.watchdog_reset_count()

    # Set watchdog time out to 2 second.
    proxy.watchdog_enable(0, 2000)

    # Disable watchdog refresh to trigger time out.
    proxy.watchdog_auto_refresh(False)

    # Wait for watchdog time out.
    time.sleep(2)

    # Verify that serial connection is lost due to time out.
    with pytest.raises(serial.SerialException):
        proxy.ram_free()

    # Explicitly tear-down proxy stream state.
    proxy.terminate()

    # Wait for serial port to settle after device reset due to watchdog time
    # out.
    time.sleep(1)

    # Reestablish serial connection to device.
    proxy._connect()

    # Verify that watchdog reset actually occurred.
    end_reset_count = proxy.watchdog_reset_count()
    proxy.watchdog_reset_count_clear(0xffff)
    assert end_reset_count > start_reset_count
