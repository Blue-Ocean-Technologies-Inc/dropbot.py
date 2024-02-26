# coding: utf-8
import warnings

from .core import (__version__, NOMINAL_ON_BOARD_CALIBRATION_CAPACITORS, package_path, get_sketch_directory,
                   get_lib_directory, get_includes, get_sources, get_firmwares, dropbot_state)

try:
    from .proxy import (Proxy, I2cProxy, SerialProxy, Config, State,
                        EVENT_ACTUATED_CHANNEL_CAPACITANCES,
                        EVENT_CHANNELS_UPDATED, EVENT_SHORTS_DETECTED,
                        EVENT_DROPS_DETECTED, EVENT_ENABLE)
except (ImportError, TypeError) as exception:
    warnings.warn(str(exception))
