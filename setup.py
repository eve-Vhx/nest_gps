import os
from glob import glob
from setuptools import setup

package_name = 'connect_pkg'

setup(
    # Other parameters ...
    data_files=[
      (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
      (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ]
)