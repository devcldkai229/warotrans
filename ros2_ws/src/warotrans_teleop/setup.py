from setuptools import find_packages, setup
from glob import glob
import os

package_name = "warotrans_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="WaroTrans Team",
    maintainer_email="dev@warotrans.local",
    description="WaroTrans mobile web teleop for commissioning",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "web_teleop_node = warotrans_teleop.web_teleop_node:main",
        ],
    },
)
