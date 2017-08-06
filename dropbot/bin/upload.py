import subprocess

from platformio_helpers.upload import upload_conda, parse_args
import conda_helpers as ch


def upload():
    teensy_exe_path = (ch.conda_prefix() / 'share' / 'platformio' /
                       'packages' / 'tool-teensy' / 'teensy.exe')
    p = subprocess.Popen([teensy_exe_path])
    upload_conda('dropbot')
    p.kill()


if __name__ == '__main__':
    print upload()
