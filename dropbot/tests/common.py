from path_helpers import path
import os

# Resolve data directory path (with support for frozen Python apps).
DATA_DIR = path(os.environ.get('DROPBOT_DATA_DIR', path(__file__).parent.parent.joinpath('static'))).normpath()
if not DATA_DIR.isdir():
    # Add support for frozen apps, where data may be stored in a zip file.
    DATA_DIR = os.path.join(*[d for d in DATA_DIR.splitall() if not d.endswith('.zip')])