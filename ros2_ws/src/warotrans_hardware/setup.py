from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'warotrans_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/warotrans_hardware/launch', glob('launch/*.launch.py')),
        ('share/warotrans_hardware/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WaroTrans Team',
    maintainer_email='dev@warotrans.local',
    description='WaroTrans package: warotrans_hardware',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge_node = warotrans_hardware.esp32_bridge_node:main',
            'wheel_odom_node = warotrans_hardware.wheel_odom_node:main',
        ],
    },
)
