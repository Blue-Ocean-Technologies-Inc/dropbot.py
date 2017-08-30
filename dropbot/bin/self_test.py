import argparse
import logging
import sys

import json_tricks

from .. import SerialProxy
from ..hardware_test import ALL_TESTS
from ..self_test import (format_system_info_results, format_test_i2c_results,
                         format_test_voltage_results,
                         format_test_shorts_results,
                         format_test_on_board_feedback_calibration_results,
                         format_test_channels_results, self_test)

json_tricks.NumpyEncoder.SHOW_SCALAR_WARNING = False


def parse_args(args=None):
    '''
    .. versionadded:: 1.28
    '''
    if args is None:
        args = sys.argv[1:]

    parser = argparse.ArgumentParser(description='Execute DropBot self-tests.')

    parser.add_argument('test', nargs='*', choices=ALL_TESTS + ['all'],
                        help='Test(s) to run.  Default: %(default)s',
                        default='all')
    parser.add_argument('--json', help='Output in JSON format.',
                        action='store_true')

    parsed_args = parser.parse_args(args)

    if 'all' in parsed_args.test:
        parsed_args.test = None

    return parsed_args


def main(argv=None):
    '''
    .. versionadded:: 1.28
    '''
    logging.basicConfig(format='%(message)s', level=logging.INFO)
    args = parse_args(args=argv)

    proxy = SerialProxy(ignore=True)
    results = self_test(proxy, tests=args.test)

    if args.json:
        # XXX Dump using `json_tricks` rather than `json` to add support for
        # serializing `numpy` array and scalar types.
        print json_tricks.dumps(results, indent=4)
    else:
        for test_name_i in (args.test if args.test is not None else ALL_TESTS):
            format_func_i = eval('format_%s_results' % test_name_i)
            results_i = results[test_name_i]
            if results_i:
                print format_func_i(results_i)
                print 72 * '-'


if __name__ == '__main__':
    main()
