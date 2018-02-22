import argparse
import sys

from dropbot import __version__ as DROPBOT_VERSION


def parse_args(args=None):
    if args is None:
        args = sys.argv[1:]
    parser = argparse.ArgumentParser()

    default_version = DROPBOT_VERSION
    parser.add_argument('-V', '--version', default=default_version)
    parser.add_argument('arg', nargs='*')

    return parser.parse_known_args(args=args)


if __name__ == '__main__':
    args, extra_args = parse_args()

    extra_args += [r'-DDEVICE_ID_RESPONSE=\"dropbot::{}\"'
                   .format(args.version), r'-DTWI_BUFFER_LENGTH=160',
                   r'-DPACKET_SIZE=160']

    print ' '.join(extra_args)
