# -*- coding: utf-8 -*-
from __future__ import absolute_import, print_function, unicode_literals
import itertools as it
import platform
import subprocess as sp
import sys
import threading
import time

from platformio_helpers.upload import upload_conda
import colorama as co
import conda_helpers as ch


def upload():
    '''
    .. versionchanged:: 1.29
        Add ``dropbot-upload`` entry point (i.e., make script callable as
        ``dropbot-upload`` from system command shell).

    .. versionchanged:: 0.48
        If upload fails, leave Teensy GUI open until either a) Teensy GUI
        window is closed; or b) Ctrl-C is pressed.
    '''
    if platform.system() == 'Windows':
        # Upload using Teensy GUI to allow auto-reboot on Windows.
        #
        # See [issue 19][i19] for more information.
        #
        # [i19]: https://gitlab.com/sci-bots/dropbot.py/issues/19

        # Launch Teensy graphical uploader program.
        teensy_tools_dir = (ch.conda_prefix() / 'share' / 'platformio' /
                            'packages' / 'tool-teensy')
        teensy_exe_path = teensy_tools_dir.joinpath('teensy.exe')
        p = sp.Popen([teensy_exe_path])

        def on_error(exception):
            # Loop calls to `teensy_reboot.exe` to keep retrying firmware
            # upload until Ctrl-C is pressed.
            #
            # XXX The `teensy_reboot.exe` process hangs if the Teensy Loader
            # GUI window is closed while the `teensy_reboot.exe` process is
            # waiting to detect the Teensy rebooting into programming mode.
            # Otherwise, we could also cancel the upload by detecting when the
            # GUI window is closed (i.e., `p.poll() is not None`).

            # Upload failed
            waiting_indicator = it.cycle(r'\|/-')

            reboot_done = threading.Event()
            status = threading.Event()
            wait_complete = threading.Event()

            def wait_for_reboot():
                '''
                Display status while waiting for Teensy to reboot into
                programming mode.
                '''
                # Update no faster than `stderr` flush interval (if set).
                update_interval = 2 * getattr(sys.stderr, 'flush_interval', .2)

                message = (co.Fore.MAGENTA + 'Waiting for Teensy program '
                           'button to be pushed...', co.Fore.BLUE + '(press '
                           'Ctrl-C to abort)')
                while not reboot_done.wait(update_interval):
                    print('\r' + co.Fore.WHITE + next(waiting_indicator),
                          *message, end='', file=sys.stderr)
                else:
                    print('\r' + co.Fore.MAGENTA + 'Teensy upload:',
                          status.message if status.is_set()
                          else co.Fore.RED + 'ERROR', 100 * ' ',
                          file=sys.stderr)
                wait_complete.set()

            # Launch background thread to display status while waiting for
            # programming button to be pressed.
            thread = threading.Thread(target=wait_for_reboot)
            thread.daemon = True
            thread.start()
            teensy_reboot_exe = teensy_tools_dir.joinpath('teensy_reboot.exe')

            reboot_process = None

            def _cancel():
                '''
                Clean up Teensy reboot process, set status to cancelled, and
                stop reboot attempt(s).
                '''
                if reboot_process is not None:
                    reboot_process.kill()
                status.message = co.Fore.YELLOW + 'CANCELLED'
                status.set()
                reboot_done.set()

            try:
                # Keep retrying firmware upload until either a) Teensy GUI
                # window is closed; or b) Ctrl-C is pressed.
                while not wait_complete.is_set() and p.poll() is None:
                    reboot_process = sp.Popen(teensy_reboot_exe,
                                              stdout=sp.PIPE, stderr=sp.PIPE)

                    while reboot_process.poll() is None:
                        if p.poll() is not None:
                            # Teensy GUI windows closed.  Cancel upload.
                            _cancel()
                            break
                        time.sleep(.5)

                    if status.is_set():
                        break

                    return_code = reboot_process.returncode

                    if return_code == 0:
                        status.message = co.Fore.LIGHTGREEN_EX + 'SUCCESS'
                        status.set()
                        break
            except KeyboardInterrupt:
                _cancel()
            finally:
                reboot_done.set()
                wait_complete.wait(5)
                print(co.Fore.RESET + co.Back.RESET + '', file=sys.stderr,
                      end='')

        # Trigger PlatformIO upload command, which will use graphical uploader
        # that is already running.
        # XXX Requires PlatformIO package, `platformio-tool-teensy==1.21.0`.
        # If upload fails, leave Teensy GUI open for 10 seconds to allow manual
        # reset into programming mode.
        upload_conda('dropbot', on_error=on_error)

        # Kill Teensy graphical uploader.
        p.kill()
    else:
        # Upload using default PlatformIO uploader.
        upload_conda('dropbot')


if __name__ == '__main__':
    upload()
