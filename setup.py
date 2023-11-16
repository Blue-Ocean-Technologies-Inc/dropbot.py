#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import versioneer

from setuptools import setup
from file_handler import get_properties

properties = get_properties(package_name='dropbot')['LIB_PROPERTIES']

setup(name=properties['package_name'],
      version=versioneer.get_version(),
      cmdclass=versioneer.get_cmdclass(),
      description=properties['short_description'],
      long_description='\n'.join([properties['short_description'],
                                  properties['long_description']]),
      author_email=properties['author_email'],
      author=properties['author'],
      url=properties['url'],
      # Install data listed in `MANIFEST.in`
      include_package_data=True,
      license='BSD-3',
      packages=[properties['package_name']])
