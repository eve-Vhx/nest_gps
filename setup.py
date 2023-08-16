from setuptools import setup
import os
from glob import glob

package_name = 'nest_gps'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mostafa',
    maintainer_email='mostafa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nest_gps_node=nest_gps.nest_gps_t:main',
            'nest_sync_node=nest_gps.nest_sync:main',
            'nest_sync_sim_node=nest_gps.nest_sync_sim:main',
        ],
    },
)
