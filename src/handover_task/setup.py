import os
from glob import glob

from setuptools import find_packages, setup

package_name = "handover_task"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ender",
    maintainer_email="ender@example.com",
    description="Single-node handover task policy package.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "base_policy = handover_task.base_policy:main",
        ]
    },
)
