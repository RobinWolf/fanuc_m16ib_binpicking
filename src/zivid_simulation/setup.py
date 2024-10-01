from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'zivid_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include all launch files (same as Cmakes InstallDirectory)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Include rviz config
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        # Include data source for test scenes
        (os.path.join('share', package_name, 'data'), glob('data/*.zdf')),

        # Include nodes executables
        (os.path.join('share', package_name, package_name), glob('zivid_simulation/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fanuc_m16ib',
    maintainer_email='fanuc_m16ib@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zivid_sim_node = zivid_simulation.zivid_sim_node:main',
        ],
    },
)
