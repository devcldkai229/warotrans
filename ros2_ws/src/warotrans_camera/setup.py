from glob import glob

from setuptools import find_packages, setup

package_name = "warotrans_camera"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="WaroTrans Team",
    maintainer_email="dev@warotrans.local",
    description="Robot camera MJPEG server and evidence recorder",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["camera_stream = warotrans_camera.camera_stream_node:main"]},
)
