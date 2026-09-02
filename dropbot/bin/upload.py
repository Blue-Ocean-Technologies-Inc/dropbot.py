"""Upload the firmware bundled with the package to a DropBot board.

Both ``dropbot-upload`` (console script) and ``python -m dropbot.bin.upload``
go through :func:`upload`, so they accept the same options.
"""
import sys
from argparse import ArgumentParser

import platformio_helpers as pioh
from platformio_helpers.upload import upload_conda


def available_environments():
    """PlatformIO environments shipped with the package, e.g. ``['teensy31']``."""
    return sorted(dir_i.name for dir_i in
                  pioh.conda_bin_path().joinpath('dropbot').dirs())


def parse_args(args=None, environments=None):
    if args is None:
        args = sys.argv[1:]
    if environments is None:
        environments = available_environments()
    parser = ArgumentParser(description='Upload DropBot firmware to board.')
    parser.add_argument('-p', '--port', default=None,
                        help='Serial port of the board (default: auto-detect).')
    parser.add_argument('-b', '--hardware-version',
                        default=environments[-1] if environments else None,
                        choices=environments,
                        help='PlatformIO environment to flash (default: %(default)s).')
    return parser.parse_args(args)


def upload(args=None):
    """Entry point for ``dropbot-upload``; ``args`` defaults to ``sys.argv[1:]``."""
    parsed = parse_args(args)
    extra_args = [] if parsed.port is None else ['--upload-port', parsed.port]
    upload_conda('dropbot', env_name=parsed.hardware_version,
                 extra_args=extra_args)


if __name__ == '__main__':
    upload()
