from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'api_moveit_clients'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include all dependencie files (same as Cmakes InstallDirectory)
        (os.path.join('share', package_name, package_name), glob('manipulation_tasks/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='logilab',
    maintainer_email='logilab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
