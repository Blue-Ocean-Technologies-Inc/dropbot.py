from __future__ import absolute_import

import sys


if sys.version_info[0] < 3:
    from .proxy_py2 import *
else:
    from .proxy_py2 import ProxyMixin
    from .proxy_py3 import *

    _SerialProxy = SerialProxy
    del SerialProxy

    class SerialProxy(ProxyMixin, _SerialProxy):
        pass
