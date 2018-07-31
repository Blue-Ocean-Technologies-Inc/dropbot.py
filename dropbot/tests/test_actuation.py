from concurrent.futures import ThreadPoolExecutor
import functools as ft

from nose.tools import eq_, raises
from dropbot.threshold import actuate_channels

from test_threads import proxy_context


def test_actuate_channels_thread_safety():
    '''
    Execute actuate channels in multiple concurrent threads.
    '''
    with proxy_context(ignore=True) as proxy:
        channels = range(10)
        with ThreadPoolExecutor(max_workers=len(channels)) as executor:
            futures = [executor.submit(ft.partial(actuate_channels, proxy, [i],
                                                  timeout=3))
                       for i in channels]

            for channel, future in zip(channels, futures):
                eq_(future.result(), [channel])


def test_actuate_channels_disabled_channels():
    with proxy_context(ignore=True) as proxy:
        with proxy.transaction_lock:
            original_mask = proxy.disabled_channels_mask
            new_mask = original_mask.copy()
            channels = range(10)

            try:
                new_mask[:] = 0

                # Actuate without any channels disabled.
                actuated = actuate_channels(proxy, channels, allow_disabled=False)
                eq_(actuated, channels)

                # Disable all even channels.
                new_mask = original_mask.copy()
                new_mask[::2] = 1
                proxy.disabled_channels_mask = new_mask

                # Verify exception is raised if disabled channels are not allowed.
                raises(RuntimeError)(actuate_channels)(proxy, channels,
                                                       allow_disabled=False)

                # Verify exception is **not** raised if disabled channels **are**
                # allowed.
                actuated = actuate_channels(proxy, channels, allow_disabled=True)
                eq_(actuated, channels[1::2])
            finally:
                proxy.disabled_channels_mask = original_mask
