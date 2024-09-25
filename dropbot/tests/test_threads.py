from contextlib import contextmanager
import threading
import time
import dropbot as db
import pandas as pd


@contextmanager
def proxy_context(*args, **kwargs):
    '''
    Context manager wrapper around DropBot proxy.
    '''
    for i in range(2):
        try:
            proxy = db.SerialProxy(*args, **kwargs)
        except ValueError:
            time.sleep(0.5)
            print('Error connecting. Retrying...')
            continue
        try:
            yield proxy
            break
        finally:
            proxy.terminate()
    else:
        raise IOError('Error connecting to DropBot.')


def test_threadsafe():
    '''
    Repeatedly execute DropBot commands concurrently in multiple threads.

    Run multiple threads which repeatedly execute DropBot commands of varying
    duration concurrently, for 10 seconds.  If an exception occurs in any
    thread, stop the test and raise a `RuntimeError` exception.

    Raises
    ------
    RuntimeError
        If an exception occurs in *any* thread.
    '''
    exceptions = []
    exception_occurred = threading.Event()

    with proxy_context(ignore=True) as proxy:
        stop_event = threading.Event()

        def _test_thread(thread_id):
            while not stop_event.wait(0.001):
                try:
                    if thread_id % 2 == 0:
                        proxy.ram_free()
                    else:
                        proxy.detect_shorts(10)
                except Exception as exception:
                    exceptions.append((thread_id, exception))
                    exception_occurred.set()
                    break

        threads = [threading.Thread(target=_test_thread, args=(i,)) for i in range(5)]

        for t in threads:
            t.daemon = True
            t.start()

        for _ in range(10):
            if exception_occurred.wait(1.0):
                break
        else:
            return
        stop_event.set()

    if exceptions:
        df_exceptions = pd.DataFrame(exceptions, columns=['thread_id', 'exception'])
        raise RuntimeError(f'The following exceptions occurred:\n{df_exceptions}')
