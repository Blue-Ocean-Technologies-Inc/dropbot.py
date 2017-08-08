import platform
import subprocess

from platformio_helpers.upload import upload_conda
import conda_helpers as ch


def upload():
    if platform.system() == 'Windows':
        # Upload using Teensy GUI to allow auto-reboot on Windows.
        #
        # See [issue 19][i19] for more information.
        #
        # [i19]: https://gitlab.com/sci-bots/dropbot.py/issues/19

        # Launch Teensy graphical uploader program.
        teensy_exe_path = (ch.conda_prefix() / 'share' / 'platformio' /
                           'packages' / 'tool-teensy' / 'teensy.exe')
        p = subprocess.Popen([teensy_exe_path])
        # Trigger PlatformIO upload command, which will use graphical uploader that
        # is already running.
        # XXX Requires PlatformIO package, `platformio-tool-teensy==1.21.0`.
        upload_conda('dropbot')
        # Kill Teensy graphical uploader.
        p.kill()
    else:
        # Upload using default PlatformIO uploader.
        upload_conda('dropbot')


if __name__ == '__main__':
    upload()
