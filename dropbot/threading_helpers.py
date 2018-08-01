from __future__ import division, print_function, unicode_literals
from functools import partial
import ctypes
import threading

import base_node_rpc as bnr
import base_node_rpc.async
import trollius as asyncio

NULL = 0


class ThreadCancelled(Exception):
    pass


def sleep(duration_s):
    '''
    Workaround for https://bugs.python.org/issue1779233

    Instead of using a busy loop based on the number of instructions returned
    by `sys.getcheckinterval()`, use a `threading.Event.wait()` call, which
    seems to force a thread context switch.
    '''
    event = threading.Event()
    event.wait(duration_s)


def ctype_async_raise(target_tid, exception):
    '''
    Raise exception in target thread.

    See: https://gist.github.com/liuw/2407154
    '''
    # Throw exception in thread.
    ret = (ctypes.pythonapi
           .PyThreadState_SetAsyncExc(ctypes.c_long(target_tid),
                                      ctypes.py_object(exception)))
    # ref: http://docs.python.org/c-api/init.html#PyThreadState_SetAsyncExc
    if ret == 0:
        raise ValueError("Invalid thread ID")
    elif ret > 1:
        # Huh? Why would we notify more than one threads?
        # Because we punch a hole into C level interpreter.
        # So it is better to clean up the mess.
        ctypes.pythonapi.PyThreadState_SetAsyncExc(target_tid, NULL)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def cancellable(f):
    '''
    Decorator to add `started` event attribute.

    The `started` event attribute has a `cancel()` method which will raise a
    `ThreadCancelled` exception in the thread where the function was called.
    This is useful, for example, to interrupt and stop a running thread.

    Example
    -------

    >>> with ThreadPoolExecutor() as executor:
    ...     @db.threading_helpers.cancellable
    ...     def _cancellable_test(duration_s):
    ...         print('hello, world!')
    ...         db.threading_helpers.sleep(duration_s)
    ...         return 'done'
    ...
    ...     future = executor.submit(_cancellable_test, 2.5)
    ...     _cancellable_test.started.wait()
    ...     time.sleep(.1)
    ...     _cancellable_test.started.cancel()
    ...     nose.tools.raises(db.threading_helpers.ThreadCancelled)(future.result)()
    ...
    hello, world!
    '''
    started = threading.Event()
    done = threading.Event()

    def _wrapped(*args, **kwargs):
        done.clear()
        started.clear()

        done.error = None
        done.result = None
        done.cancelled = False

        started.ident = threading.current_thread().ident
        started.cancel = partial(ctype_async_raise, started.ident, SystemExit)
        started.set()
        try:
            done.result = f(*args, **kwargs)
            sleep(0.001)
        except SystemExit:
            done.cancelled = True
        except Exception as exception:
            done.error = exception
        finally:
            done.set()
        if done.cancelled:
            raise ThreadCancelled()
        elif done.error is not None:
            raise done.error
        return done.result

    _wrapped.started = started
    _wrapped.done = done
    return _wrapped


def co_cancellable(f):
    '''
    Decorator to add `started` event attribute and `cancel()` method.

    The `cancel()` method cancels the running coroutine and by raising a
    `CancelledError` exception.
    '''
    started = threading.Event()

    def _wrapped(*args, **kwargs):
        started.clear()
        started.loop = bnr.async.ensure_event_loop()
        started.set()
        return started.loop.run_until_complete(f(*args, **kwargs))

    def _cancel():
        loop = bnr.async.ensure_event_loop()
        tasks = [task for task in asyncio.Task.all_tasks(loop=loop)
                if task is not
                asyncio.tasks.Task.current_task(loop=loop)]
        list(map(lambda task: task.cancel(), tasks))

    def cancel():
        started.loop.call_soon_threadsafe(_cancel)

    _wrapped.started = started
    _wrapped.cancel = lambda: started.loop.call_soon_threadsafe(_cancel)
    return _wrapped
