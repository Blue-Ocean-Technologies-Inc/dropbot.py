from concurrent.futures import ThreadPoolExecutor
import functools as ft
import pytest

from dropbot.threshold import actuate_channels
from test_threads import proxy_context


def test_actuate_channels_thread_safety():
    '''
    Execute actuate channels in multiple concurrent threads.
    '''
    with proxy_context(ignore=True) as proxy:
        channels = range(10)
        with ThreadPoolExecutor(max_workers=len(channels)) as executor:
            futures = [executor.submit(ft.partial(actuate_channels, proxy, [i], timeout=3)) for i in channels]

            for channel, future in zip(channels, futures):
                assert future.result() == [channel]


def test_actuate_channels_count():
    'Verify number of actuated channels against requested channels.'
    with proxy_context(ignore=True) as proxy:
        channel_count = proxy.number_of_channels
        assert channel_count == len(proxy.state_of_channels)
        proxy.set_state_of_channels(channel_count * [0])


def test_actuate_channels_disabled_channels():
    with proxy_context(ignore=True) as proxy:
        with proxy.transaction_lock:
            original_mask = proxy.disabled_channels_mask
            new_mask = original_mask.copy()
            channels = list(range(10))

            try:
                new_mask[:] = 0

                # Actuate without any channels disabled.
                actuated = actuate_channels(proxy, channels, allow_disabled=False)
                assert actuated == channels

                # Disable all even channels.
                new_mask = original_mask.copy()
                new_mask[::2] = 1
                proxy.disabled_channels_mask = new_mask

                # Test that an exception is raised if disabled channels are not allowed.
                with pytest.raises(RuntimeError):
                    actuate_channels(proxy, channels, allow_disabled=False)

                # Verify exception is not raised if disabled channels are allowed.
                actuated = actuate_channels(proxy, channels, allow_disabled=True)
                assert actuated == channels[1::2]
            finally:
                proxy.disabled_channels_mask = original_mask
