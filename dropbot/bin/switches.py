# coding: utf-8
'''
.. versionadded:: 1.49
'''
from __future__ import (absolute_import, print_function, division,
                        unicode_literals)
from collections import OrderedDict
import argparse
import datetime as dt
import logging
import sys
import time

from tabulate import tabulate
import base_node_rpc as bnr
import colorama as C_
import pandas as pd
import path_helpers as ph
import platformio_helpers as pioh

from .. import SerialProxy
from ..hv_switching_boards import switching_boards, switching_boards_info


def parse_args(switching_boards_info, args=None):
    if args is None:
        args = sys.argv[1:]

    parser = argparse.ArgumentParser(description='DropBot switching boards '
                                     'utility')

    subparsers = parser.add_subparsers(help='Commands', dest='command')

    subparsers.add_parser('list', help='List available switching boards')

    parser_upload = subparsers.add_parser('upload', help='Upload firmware to '
                                          'one or more switching boards.')

    # Find HV switching board firmware (look in new and legacy PlatformIO
    # helpers binary directories).
    for bin_path_i in (pioh.conda_bin_path(), pioh.conda_bin_path_05()):
        default_firmware = bin_path_i.joinpath('hv-switching-board', 'v3_1',
                                               'firmware.hex')
        if default_firmware.isfile():
            parser_upload.add_argument('firmware_path', type=ph.path,
                                       help='Switching board firmware file '
                                       '(default=%(default)s)',
                                       default=default_firmware, nargs='?')
            break
    else:
        # No firmware found.
        parser_upload.add_argument('firmware_path', type=ph.path,
                                   help='Switching board firmware file.')

    boards_group = parser_upload.add_mutually_exclusive_group()
    boards_group.add_argument('-a', '--all', action='store_true', help='Upload'
                              ' to all.')
    boards_group.add_argument('-b', '--board', action='append', help='Address '
                              'of switching board to flash.',
                              choices=map(str, switching_boards_info.index))

    subparsers.add_parser('boot', help='Boot all switching boards currently '
                          'in bootloader mode.')

    subparsers.add_parser('boot_list', help='List available switching board '
                          'bootloaders.')

    parsed_args = parser.parse_args(args)

    if parsed_args.command == 'upload':
        if not parsed_args.firmware_path.realpath().isfile():
            parser_upload.error(C_.Fore.RED + 'could not read firmware: ' +
                                C_.Fore.WHITE +
                                parsed_args.firmware_path.realpath())
        elif not any([parsed_args.all, parsed_args.board]):
            parser_upload.error(C_.Fore.RED + 'switching board(s) must be '
                                'specified.' +
                                C_.Fore.MAGENTA + '\n\nList available '
                                'switching boards with the ' + C_.Fore.WHITE +
                                '`list`' + C_.Fore.MAGENTA + ' command.\n\n' +
                                C_.Fore.BLUE + 'Use ' + C_.Fore.WHITE + '`-a` '
                                + C_.Fore.BLUE + ' to upload to all '
                                'available switching boards or\nuse ' +
                                C_.Fore.WHITE + '`-b <i2c address>`' +
                                C_.Fore.BLUE + ' to choose a specific board '
                                '(may be specified multiple times).')
        if parsed_args.all:
            parsed_args.addresses = switching_boards_info.index.tolist()
        else:
            parsed_args.addresses = map(int, parsed_args.board)
        if not parsed_args.addresses:
            parser_upload.error(C_.Fore.RED + 'No switching board(s) '
                                'detected.')
    return parsed_args


def main(argv=None):
    C_.init(autoreset=True)

    try:
        proxy = SerialProxy(ignore=[bnr.proxy.DeviceVersionMismatch])
    except bnr.proxy.DeviceNotFound:
        print(C_.Fore.RED + 'No DropBot available.' +
              C_.Fore.BLUE + '\n\nPlease make sure DropBot USB cable is '
              'connected and DropBot is not being used by another '
              'application.', file=sys.stderr)
        sys.exit(-1)
    switching_boards_ = OrderedDict([(b.address, b)
                                     for b in switching_boards(proxy)])
    info = switching_boards_info(switching_boards_.values())

    args = parse_args(info, args=argv)
    print(args)

    list_boards = False
    if args.command == 'upload':
        bootloader = bnr.bootloader_driver.TwiBootloader(proxy)
        selected_boards = {k: switching_boards_[k] for k in args.addresses}
        for address, board in selected_boards.items():
            print(C_.Fore.MAGENTA + 'Board ' + C_.Fore.WHITE + str(address) +
                  C_.Fore.MAGENTA + ':' + C_.Fore.BLUE + ' reboot into '
                  'bootloader.')
            board.reboot_recovery()

        time.sleep(.2)

        try:
            print(C_.Fore.MAGENTA + 'Write firmware to: ' + C_.Fore.WHITE +
                  ', '.join(map(str, args.addresses)))
            start = dt.datetime.now()
            bootloader.write_firmware(args.firmware_path.realpath())
            end = dt.datetime.now()
            print(C_.Fore.LIGHTGREEN_EX + '\rDone ' + C_.Fore.BLUE +
                  '(duration: %s)' % (end - start) + 100 * ' ')
        finally:
            print(C_.Fore.MAGENTA + 'Board(s) ' + C_.Fore.WHITE +
                  ', '.join(map(str, args.addresses)) + C_.Fore.MAGENTA + ':' +
                  C_.Fore.BLUE + ' boot firmware.')
            bootloader.start_application()
            time.sleep(.2)
            list_boards = True
    elif args.command == 'boot':
        bootloader = bnr.bootloader_driver.TwiBootloader(proxy)
        print(C_.Fore.MAGENTA + 'Boot all switching boards currently in '
              'bootloader mode.')
        bootloader.start_application()
        time.sleep(.2)

        # Update list of switching boards.
        switching_boards_ = {b.address: b for b in switching_boards(proxy)}
        info = switching_boards_info(switching_boards_.values())
        list_boards = True
    elif args.command == 'boot_list':
        bootloader = bnr.bootloader_driver.TwiBootloader(proxy)

        # Boot any boards already in bootloader.
        bootloader.start_application()

        infos = []

        for board_i in switching_boards_.values():
            try:
                board_i.reboot_recovery()
                info_i = bootloader.read_chip_info()
                info_i['version'] = bootloader.read_bootloader_version()
                infos.append(info_i)
            finally:
                bootloader.start_application()
        df_boot_infos = pd.DataFrame(infos, index=switching_boards_.keys())
        df_boot_infos.index.name = 'address'
        print(C_.Fore.MAGENTA + 'Detected the following bootloaders:\n')
        print(tabulate(df_boot_infos, tablefmt="pipe", headers="keys"))

    if args.command == 'list' or list_boards:
        # Show available COM ports.

        # Increase pandas display width to show each row of the switching
        # boards table on a single line.
        pd.set_option('display.width', 200)
        print(C_.Fore.MAGENTA + 'Detected the following switching boards:\n')
        # Prepend `v` to versions to prevent coercion to a float.
        info.hardware_version = 'v' + info.hardware_version
        info.software_version = 'v' + info.software_version
        print(tabulate(info, tablefmt="pipe", headers="keys"))


if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    main()
