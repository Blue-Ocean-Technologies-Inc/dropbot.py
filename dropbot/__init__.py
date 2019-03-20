import warnings

from .core import *
try:
    from .proxy import (Proxy, I2cProxy, SerialProxy,
                        EVENT_ACTUATED_CHANNEL_CAPACITANCES,
                        EVENT_CHANNELS_UPDATED, EVENT_SHORTS_DETECTED,
                        EVENT_DROPS_DETECTED, EVENT_ENABLE)
    from .config import Config
    from .state import State
except (ImportError, TypeError) as exception:
    warnings.warn(str(exception))
