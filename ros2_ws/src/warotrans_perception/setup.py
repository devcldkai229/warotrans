from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'warotrans_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/warotrans_perception/launch', glob('launch/*.launch.py')),
        ('share/warotrans_perception/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WaroTrans Team',
    maintainer_email='dev@warotrans.local',
    description='WaroTrans package: warotrans_perception',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add ROS 2 node entry points only when the node is implemented.
        ],
    },
)
