# coding: utf-8
import os
import re
import argparse
import subprocess

import platformio_helpers as pioh

from typing import Dict
from importlib import import_module

from path_helpers import path
from base_node_rpc.helpers import generate_all_code

DEFAULT_ARDUINO_BOARDS = []

PLATFORMIO_ENVS = ['teensy31']


def read_version() -> str:
    # Read the version without importing `dropbot` (whose generated modules do
    # not exist yet while this script runs).
    text = path(__file__).parent.joinpath('dropbot', '_version.py').read_text()
    return re.search(r'^__version__ = "([^"]+)"', text, re.M).group(1)


def get_properties(**kwargs) -> Dict:
    version = read_version()

    try:
        base_node_version = import_module('base_node_rpc').__version__
    except ImportError:
        base_node_version = None

    module_name = kwargs.get('module_name')
    package_name = kwargs.get('package_name')
    url = f'https://github.com/sci-bots/{package_name}'

    properties = dict(package_name=package_name,
                      display_name=package_name,
                      manufacturer='Sci-bots Inc.',
                      software_version=version,
                      base_node_software_version=base_node_version,
                      url=url
                      )

    meta = dict(short_description='Dropbot API',
                long_description='',
                author='Christian Fobel',
                author_email='christian@fobel.net',
                version=version,
                license='BSD-3',
                category='Communication',
                architectures='avr',
                )

    lib_properties = {**properties, **meta}

    options = dict(pointer_width=32,
                   rpc_module=import_module(module_name) if module_name else None,
                   PROPERTIES=properties,
                   LIB_PROPERTIES=lib_properties,
                   base_classes=['BaseNodeSerialHandler',
                                 'BaseNodeEeprom',
                                 'BaseNodeI2c',
                                 'BaseNodeI2cHandler<Handler>',
                                 'BaseNodeConfig<ConfigMessage, Address>',
                                 'BaseNodeState<StateMessage>'],
                   rpc_classes=[f'{module_name}::Node'],
                   DEFAULT_ARDUINO_BOARDS=DEFAULT_ARDUINO_BOARDS,
                   PLATFORMIO_ENVS=PLATFORMIO_ENVS,
                   )

    return {**kwargs, **options}


def transfer(**kwargs) -> None:
    # Copy Arduino library to Conda include directory
    source_dir = kwargs.get('source_dir')
    lib_name = kwargs.get('lib_name')
    # source_dir = path(source_dir).joinpath(module_name, 'Arduino', 'library', lib_name) # Use this for Arduino libs
    source_dir = path(source_dir).joinpath('lib', lib_name)
    install_dir = pioh.conda_arduino_include_path().joinpath(lib_name)
    if install_dir.exists():
        install_dir.rmtree()
    source_dir.copytree(install_dir)
    print(f"Copied tree from '{source_dir}' to '{install_dir}'")


def compile_protobufs(**kwargs) -> None:
    import nanopb_helpers as pbh

    package_name = kwargs.get('package_name')
    module_name = kwargs.get('module_name') or package_name.replace('-', '_')
    source_path = path(kwargs.get('source_dir')).joinpath(package_name, 'metadata.proto')
    # Scope the descriptor per package: every firmware package has a
    # `metadata.proto`, and unscoped names collide in the shared protobuf
    # descriptor pool when two packages are imported in one process.
    code = pbh.compile_pb(source_path, scope=module_name)
    output_path = source_path.with_suffix('.py')
    output_path.write_text(code['python'])
    print(f'Generated {output_path.name} > {output_path}')


def copy_compiled_firmware(**kwargs) -> None:
    # Copy compiled firmware to Conda bin directory
    source_dir = kwargs.get('source_dir')
    package_name = kwargs.get('package_name')
    src_dir = path(source_dir)
    pio_bin_dir = pioh.conda_bin_path().joinpath(package_name)

    pio_bin_dir.makedirs(exist_ok=True)

    src_dir.joinpath('platformio.ini').copy2(pio_bin_dir.joinpath('platformio.ini'))

    for pio_platform in PLATFORMIO_ENVS:
        hex_dir = pio_bin_dir.joinpath(pio_platform)
        hex_dir.makedirs(exist_ok=True)
        src = src_dir.joinpath('.pio', 'build', pio_platform, 'firmware.hex')
        dest = hex_dir.joinpath('firmware.hex')
        src.copy2(dest)
        print(f"Copied '{dest.name}' > '{dest}'")


def cli_parser():
    # Defaults come from the rattler-build environment (see recipe/recipe.yaml)
    # so the build script is the same on every platform.
    parser = argparse.ArgumentParser(description='Generate RPC code, build firmware and install both into a prefix.')
    parser.add_argument('source_dir', nargs='?', default=os.environ.get('SRC_DIR', path(__file__).parent))
    parser.add_argument('prefix', nargs='?', default=os.environ.get('PREFIX'))
    parser.add_argument('package_name', nargs='?', default='dropbot')
    parser.add_argument('--skip-firmware', action='store_true',
                        help='Only generate the RPC code and protobuf modules '
                             '(no Arduino library install, no PlatformIO build).')

    args = parser.parse_args()
    args_dict = vars(args)
    args_dict['module_name'] = args_dict['package_name'].replace('-', '_')
    args_dict['lib_name'] = ''.join(word.capitalize() for word in args_dict['package_name'].split('-'))
    execute(**args_dict)


def execute(**kwargs):
    properties = get_properties(**kwargs)

    top = '>' * 180
    print(top)

    generate_all_code(properties)
    compile_protobufs(**kwargs)
    if kwargs.get('skip_firmware'):
        # Python-package build: the generated modules are all that is needed.
        print('<' * len(top))
        return
    transfer(**kwargs)
    # PlatformIO finds the generated Arduino library (and the sci-bots ones
    # installed by the conda dependencies) through PLATFORMIO_LIB_EXTRA_DIRS.
    env = os.environ.copy()
    env['PLATFORMIO_LIB_EXTRA_DIRS'] = str(pioh.conda_arduino_include_path())
    if 'PLATFORMIO_CORE_DIR' not in env and os.name == 'nt' and env.get('HOMEDRIVE') and env.get('HOMEPATH'):
        # rattler-build points HOME/USERPROFILE at its (deep) work directory.
        # The ARM toolchain's include paths from there exceed MAX_PATH and the
        # MinGW-built GCC then cannot find its own C++ headers, so keep the
        # PlatformIO core dir in the real user profile (HOMEPATH is untouched).
        env['PLATFORMIO_CORE_DIR'] = os.path.join(env['HOMEDRIVE'] + env['HOMEPATH'], '.platformio')
    print(f"Setting PLATFORMIO_CORE_DIR={env.get('PLATFORMIO_CORE_DIR', '<default>')}", flush=True)
    print(f"Setting PLATFORMIO_LIB_EXTRA_DIRS={env['PLATFORMIO_LIB_EXTRA_DIRS']}", flush=True)
    # A firmware build failure must fail the package build.
    subprocess.run(['pio', 'run'], env=env, check=True)
    copy_compiled_firmware(**kwargs)

    print('<' * len(top))


if __name__ == '__main__':
    cli_parser()
