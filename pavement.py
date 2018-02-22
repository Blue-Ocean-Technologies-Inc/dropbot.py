from __future__ import absolute_import
from collections import OrderedDict
import sys
from importlib import import_module

from paver.easy import path, options
from paver.setuputils import install_distutils_tasks
import platformio_helpers as pioh
import platformio_helpers.develop
try:
    import base_node_rpc
    from base_node_rpc.pavement_base import *
except ImportError:
    import warnings

    warnings.warn('Could not import `base_node_rpc` (expected during '
                  'install).')

# Add current directory to Python path
sys.path.insert(0, '.')
import versioneer
install_distutils_tasks()

DEFAULT_ARDUINO_BOARDS = []  #['mega2560']
PROJECT_PREFIX = 'dropbot'
module_name = PROJECT_PREFIX
package_name = module_name.replace('_', '-')
rpc_module = import_module(PROJECT_PREFIX)
VERSION = versioneer.get_version()
URL='http://gitlab.com/sci-bots/%s.py.git' % package_name
PROPERTIES = OrderedDict([('base_node_software_version',
                           base_node_rpc.__version__),
                          ('package_name', package_name),
                          ('display_name', 'DropBot'),
                          ('manufacturer', 'Sci-Bots Inc.'),
                          ('software_version', VERSION),
                          ('url', URL)])
LIB_PROPERTIES = PROPERTIES.copy()
LIB_PROPERTIES.update(OrderedDict([('author', 'Christian Fobel'),
                                   ('author_email', 'christian@fobel.net'),
                                   ('short_description', 'Template project '
                                    'demonstrating use of Arduino base node '
                                    'RPC framework.'),
                                   ('version', VERSION),
                                   ('long_description', ''),
                                   ('category', 'Communication'),
                                   ('architectures', 'avr')]))

options(
    pointer_width=32,
    rpc_module=rpc_module,
    PROPERTIES=PROPERTIES,
    LIB_PROPERTIES=LIB_PROPERTIES,
    base_classes=['BaseNodeSerialHandler',
                  'BaseNodeEeprom',
                  'BaseNodeI2c',
                  'BaseNodeI2cHandler<Handler>',
                  'BaseNodeConfig<ConfigMessage, Address>',
                  'BaseNodeState<StateMessage>'],
    rpc_classes=['dropbot::Node'],
    DEFAULT_ARDUINO_BOARDS=DEFAULT_ARDUINO_BOARDS,
    setup=dict(name=package_name,
               version=VERSION,
               cmdclass=versioneer.get_cmdclass(),
               description=LIB_PROPERTIES['short_description'],
               author='Christian Fobel',
               author_email='christian@fobel.net',
               url=URL,
               license='BSD-3',
               install_requires=['teensy-minimal-rpc>=0.3.0'],
               include_package_data=True,
               packages=[str(PROJECT_PREFIX)]))


@task
def develop_link():
    import logging; logging.basicConfig(level=logging.INFO)
    pioh.develop.link(working_dir=path('.').realpath(),
                      package_name=package_name)


@task
def develop_unlink():
    import logging; logging.basicConfig(level=logging.INFO)
    pioh.develop.unlink(working_dir=path('.').realpath(),
                        package_name=package_name)


@task
@needs('generate_all_code')
@needs('compile_protobufs')
def build_firmware():
    sh('pio run')


@task
def upload():
    sh('pio run --target upload --target nobuild')


@task
def compile_protobufs():
    import nanopb_helpers as pbh

    code = pbh.compile_pb(path('.').joinpath('dropbot',
                          'metadata.proto').realpath())
    output_path = path('.').joinpath('dropbot',
                                     'metadata.py').realpath()
    with output_path.open('w') as output:
        output.write(code['python'])
