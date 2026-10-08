import os
from glob import glob

from setuptools import find_packages, setup

package_name = "autonomous_explorer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Akhash Arjundayal",
    maintainer_email="116380148+itsakhash@users.noreply.github.com",
    description="Autonomous frontier-based exploration for a TurtleBot3 "
    "using SLAM Toolbox and Nav2 in Gazebo.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "frontier_explorer = autonomous_explorer.frontier_explorer:main",
            "twist_relay = autonomous_explorer.twist_relay:main",
        ],
    },
)
