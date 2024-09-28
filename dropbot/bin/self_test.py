# coding: utf-8
import sys
import jinja2
import logging
import argparse
import json_tricks

import datetime as dt

from path_helpers import path

from .. import SerialProxy
from ..hardware_test import ALL_TESTS
# Import format functions used by `main`.
from ..self_test import (format_system_info_results, format_test_i2c_results,
                         format_test_voltage_results,
                         format_test_shorts_results,
                         format_test_on_board_feedback_calibration_results,
                         format_test_channels_results, generate_report,
                         self_test)

json_tricks.NumpyEncoder.SHOW_SCALAR_WARNING = False


def parse_args(args=None):
    """
    .. versionadded:: 1.28
    """
    if args is None:
        args = sys.argv[1:]

    parser = argparse.ArgumentParser(description='Execute DropBot self-tests.')

    subparsers = parser.add_subparsers(help='Commands', dest='command')

    parser_test = subparsers.add_parser('test')
    parser_test.add_argument('test', nargs='*', choices=ALL_TESTS + ['all'],
                             help='Test(s) to run.  Default: %(default)s', default='all')
    parser_test.add_argument('--json', help='Output in JSON format.', action='store_true')
    parser_test.add_argument('-o', '--output-path', help='Output path', required=False)
    parser_test.add_argument('-f', '--force', action='store_true',
                             help='Force overwrite of existing file.', required=False)

    parser_report = subparsers.add_parser('report')
    parser_report.add_argument('input_path', help='Input JSON results file')
    parser_report.add_argument('output_path', nargs='?',
                               help='Output path (filepath with `.docx` extension or directory '
                                    'for Markdown output with figure images)')
    parser_report.add_argument('-f', '--force', action='store_true',
                               help='Force overwrite of existing output path.', required=False)

    for subparser_i in (parser_test, parser_report):
        subparser_i.add_argument('--launch', action='store_true',
                                 help='Launch output path after creation.', required=False)

    parsed_args = parser.parse_args(args)

    if parsed_args.command == 'test' and 'all' in parsed_args.test:
        parsed_args.test = None

    if parsed_args.output_path:
        parsed_args.output_path = path(parsed_args.output_path)
        if parsed_args.output_path.exists() and not parsed_args.force:
            parser.error(f'Output path `{parsed_args.output_path}` exists.  Use `--force` to overwrite.')
        if parsed_args.output_path.ext.lower() == '.json':
            parsed_args.json = True
    elif parsed_args.launch:
        parser.error('Launch output only makes sense when output path is specified.')

    return parsed_args


def main(argv=None):
    """
    .. versionadded:: 1.28
    """
    logging.basicConfig(format='%(message)s', level=logging.INFO)
    args = parse_args(args=argv)

    def _render_output_path(output_path_):
        # Find the starting time of earliest test (or current date and time if no timestamp is available).
        min_timestamp = min([result_i['utc_timestamp']
                             for result_i in results.values()
                             if 'utc_timestamp' in result_i] +
                            [dt.datetime.now(dt.timezone.utc).isoformat()])
        # Get control board UUID from system info.
        uuid = (results.get('system_info', {}).get('control board', {})
                .get('uuid'))
        # Perform string substitution for timestamp and UUID in the output path.
        template = jinja2.Template(output_path_)
        return path(template.render(full_timestamp=min_timestamp,
                                    timestamp=min_timestamp
                                    .replace(':', '_').replace('-', '_').split('.')[0],
                                    uuid=uuid))

    if args.command == 'test':
        proxy = SerialProxy(ignore=True)
        results = self_test(proxy, tests=args.test)

        if args.json:
            # XXX Dump using `json_tricks` rather than `json` to add support for
            # serializing `numpy` array and scalar types.
            json_results = json_tricks.dumps(results, indent=4)
            if args.output_path:
                # Perform string substitution for timestamp and UUID in the output path.
                output_path = _render_output_path(args.output_path)
                with output_path.open('w') as output:
                    output.write(json_results)
                print(output_path)
            else:
                print(json_results)
            return
    elif args.command == 'report':
        # XXX Load using `json_tricks` rather than `json` to add support for
        # deserializing `numpy` array and scalar types.
        with open(args.input_path, 'r') as input_:
            results = json_tricks.loads(input_.read(), preserve_order=False)

    if args.output_path:
        # Perform string substitution for timestamp and UUID in the output path.
        output_path = _render_output_path(args.output_path)
        generate_report(results, output_path=output_path, force=args.force)
        if args.launch:
            # Launch the output path (either directory or Word document).
            output_path.launch()
    else:
        print(generate_report(results))


if __name__ == '__main__':
    main()
