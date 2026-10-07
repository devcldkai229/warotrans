from setuptools import find_packages, setup
from glob import glob
import os

package_name = "warotrans_fleet"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/scripts", glob("scripts/*")),
    ],
    install_requires=["setuptools", "paho-mqtt"],
    zip_safe=True,
    maintainer="WaroTrans Team",
    maintainer_email="dev@warotrans.local",
    description="WaroTrans fleet Robot Gateway (ROS2 → MQTT)",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_gateway_node = warotrans_fleet.robot_gateway_node:main",
        ],
    },
)
