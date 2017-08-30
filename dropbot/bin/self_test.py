import argparse
import logging
import sys

import json_tricks

from .. import SerialProxy
from ..hardware_test import ALL_TESTS, self_test

json_tricks.NumpyEncoder.SHOW_SCALAR_WARNING = False


def parse_args(args=None):
    if args is None:
        args = sys.argv[1:]

    parser = argparse.ArgumentParser(description='Execute DropBot self-tests.')

    parser.add_argument('test', nargs='*', choices=ALL_TESTS + ['all'],
                        help='Test(s) to run.  Default: %(default)s',
                        default='all')

    parsed_args = parser.parse_args(args)

    if 'all' in parsed_args.test:
        parsed_args.test = None

    return parsed_args


def main(argv=None):
    logging.basicConfig(format='%(message)s', level=logging.INFO)
    args = parse_args(args=argv)

    proxy = SerialProxy(ignore=True)
    results = self_test(proxy, tests=args.test)

    print json_tricks.dumps(results, indent=4)


if __name__ == '__main__':
    main()
