from __future__ import division, print_function, unicode_literals
from concurrent.futures import ThreadPoolExecutor
import threading
import time

import dropbot as db
import dropbot.threading_helpers
import nose.tools


@db.threading_helpers.cancellable
def _cancellable_test(duration_s):
    print('hello, world!')
    db.threading_helpers.sleep(duration_s)
    print('goodbye, world!')
    return 'done'


def test_cancellable_done():
    thread = threading.Thread(target=_cancellable_test, args=(1., ))
    thread.daemon = True
    thread.start()
    _cancellable_test.done.wait()
    nose.tools.eq_('done', _cancellable_test.done.result)


def test_cancellable_cancelled():
    def _test():
        try:
            _cancellable_test(1.)
        except db.threading_helpers.ThreadCancelled:
            pass
    thread = threading.Thread(target=_test)
    thread.daemon = True
    thread.start()
    time.sleep(.1)
    _cancellable_test.started.cancel()
    _cancellable_test.done.wait()
    nose.tools.eq_(_cancellable_test.done.cancelled, True)


def test_cancellable_executor_cancelled():
    with ThreadPoolExecutor() as executor:
        future = executor.submit(_cancellable_test, 1.)
        time.sleep(.1)
        _cancellable_test.started.cancel()
        nose.tools.raises(db.threading_helpers.ThreadCancelled)(future.result)()
