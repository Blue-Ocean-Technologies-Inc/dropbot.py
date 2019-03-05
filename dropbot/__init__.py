from __future__ import absolute_import
from collections import OrderedDict
import contextlib
import os.path
import warnings

from path_helpers import path
import pandas as pd

from ._version import get_versions
try:
    from .proxy import (Proxy, I2cProxy, SerialProxy,
                        EVENT_ACTUATED_CHANNEL_CAPACITANCES,
                        EVENT_CHANNELS_UPDATED, EVENT_SHORTS_DETECTED,
                        EVENT_DROPS_DETECTED, EVENT_ENABLE)
    from .config import Config
    from .state import State
except (ImportError, TypeError) as exception:
    warnings.warn(str(exception))

__version__ = get_versions()['version']
del get_versions


'''
.. versionadded:: 1.30
'''
NOMINAL_ON_BOARD_CALIBRATION_CAPACITORS = pd.Series([0, 10e-12, 100e-12,
                                                     470e-12],
                                                    name='Capacitance (F)')

# Resolve data directory path (with support for frozen Python apps).
DATA_DIR = path(os.environ.get('DROPBOT_DATA_DIR', path(__file__).parent
                               .joinpath('static'))).normpath()
if not DATA_DIR.isdir():
    # Add support for frozen apps, where data may be stored in a zip file.
    DATA_DIR = os.path.join(*[d for d in DATA_DIR.splitall()
                              if not d.endswith('.zip')])


def package_path():
    return path(__file__).parent


def get_sketch_directory():
    '''
    Return directory containing the Arduino sketch.
    '''
    return package_path().joinpath('..', 'src').realpath()


def get_lib_directory():
    return package_path().joinpath('..', 'lib').realpath()


def get_includes():
    '''
    Return directories containing the Arduino header files.

    Notes
    =====

    For example:

    .. sourcecode:: python

        import arduino_rpc
        ...
        print ' '.join(['-I%s' % i for i in arduino_rpc.get_includes()])
        ...

    '''
    import base_node_rpc

    return ([get_sketch_directory()] +
            list(get_lib_directory().walkdirs('src')) +
            base_node_rpc.get_includes())


def get_sources():
    '''
    Return Arduino source file paths.  This includes any supplementary source
    files that are not contained in Arduino libraries.
    '''
    import base_node_rpc

    return (get_sketch_directory().files('*.c*') +
            list(get_lib_directory().walkfiles('*.c*')) +
            base_node_rpc.get_sources())


def get_firmwares():
    '''
    Return compiled Arduino hex file paths.

    This function may be used to locate firmware binaries that are available
    for flashing to Arduino_ boards.

    .. _Arduino: http://arduino.cc
    '''
    return OrderedDict([(board_dir.name, [f.abspath() for f in
                                          board_dir.walkfiles('*.hex')])
                        for board_dir in
                        package_path().joinpath('firmware').dirs()])


@contextlib.contextmanager
def dropbot_state(proxy, **kwargs):
    '''
    Context manager to set/restore DropBot state.

    Automatically stop switching matrix upon entry and exit (if applicable).

    Parameters
    ----------
    proxy : dropbot.SerialProxy
        DropBot serial handle.
    **kwargs
        Keyword arguments are passed to `proxy.update_state()`.


    .. versionadded:: 1.72.0
    '''
    with proxy.transaction_lock:
        if hasattr(proxy, 'stop_switching_matrix'):
            proxy.stop_switching_matrix()
        original_state = proxy.state
        channel_states = proxy.state_of_channels
        if kwargs:
            with proxy.transaction_lock:
                proxy.update_state(**kwargs)

    try:
        yield proxy
    finally:
        # Restore original control board state.
        with proxy.transaction_lock:
            if hasattr(proxy, 'stop_switching_matrix'):
                proxy.stop_switching_matrix()
            state = proxy.state
            if not (state == original_state).all():
                proxy.state = \
                    original_state[state != original_state]
            proxy.state_of_channels = channel_states
