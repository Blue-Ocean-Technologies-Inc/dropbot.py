# coding: utf-8
from __future__ import (absolute_import, print_function, division,
                        unicode_literals)

import time

import hv_switching_board as hv
import logging
import pandas as pd


def write_firmware(bootloader, firmware_path):
    '''
    Parameters
    ----------
    bootloader : base_node_rpc.bootloader_driver.TwiBootloader
    firmware_path : str
        Path to firmware to write.
    '''
    bootloader.write_firmware(firmware_path)
    logging.info('Launch firmware')
    bootloader.start_application()
    # Wait for switching board firmware to boot.
    time.sleep(.2)


def switching_boards(proxy):
    '''
    Parameters
    ----------
    proxy : dropbot.SerialProxy

    Returns
    -------
    list
        List of switching board instances.
    '''
    return [hv.HVSwitchingBoard(proxy, address=address)
            for address in proxy.i2c_scan()
            if address in [32, 33, 34]]


def switching_boards_info(switching_boards):
    '''
    Parameters
    ----------
    proxy : dropbot.SerialProxy

    Returns
    -------
    pandas.DataFrame
        Table of switching board info, indexed by switching board I2C address.

        Columns include: ``uuid``, ``hardware_version``, and
        ``software_version``.
    '''
    switching_boards_info = pd.DataFrame(({k: getattr(switching_board_i, k)()
                                           .split(b'\0')[0]
                                           for k in ('hardware_version',
                                                     'software_version')}
                                          for switching_board_i in
                                          switching_boards),
                                         index=pd.Series([switching_board_i
                                                          .address
                                                          for switching_board_i
                                                          in switching_boards],
                                                         name='address'))
    switching_boards_info.insert(0, 'uuid', [switching_board_i.uuid
                                             for switching_board_i in
                                             switching_boards])
    return switching_boards_info
