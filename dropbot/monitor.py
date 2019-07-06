from __future__ import absolute_import
import sys
if sys.version_info[0] < 3:
    from .monitor_py2 import asyncio, monitor
else:
    from .monitor_py3 import asyncio, monitor
