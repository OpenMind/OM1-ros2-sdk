import os
from glob import glob

from setuptools import setup

package_name = "frontier_explorer"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="hello@openmind.org",
    description="Frontier explorer",
    license="BSD",
    entry_points={
        "console_scripts": [
            "explore = frontier_explorer.explore:main",
        ],
    },
)
