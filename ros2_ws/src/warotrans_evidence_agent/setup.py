from glob import glob

from setuptools import find_packages, setup

package_name = "warotrans_evidence_agent"

setup(
    name=package_name,
    version="0.1.0",
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
    description="Evidence agent (recording, spool, upload)",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["evidence_agent = warotrans_evidence_agent.agent_node:main"]},
)
