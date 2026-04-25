import os
from glob import glob

from setuptools import find_packages, setup

package_name = "motion_state_estimator"

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
    description="Motion state estimator package skeleton.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grasp_pose_predictor_node = motion_state_estimator.grasp_pose_predictor_node:main",
        ]
    },
)
