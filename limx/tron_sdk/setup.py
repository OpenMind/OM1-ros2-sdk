from glob import glob

from setuptools import find_packages, setup

package_name = "tron_sdk"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/tron_sdk"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="openmind",
    maintainer_email="hello@openmind.org",
    description="LIMX TRON robot SDK with sensor capabilities",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cmd_vel_to_tron = tron_sdk.tron_movement:main",
            "tron_odom = tron_sdk.tron_odom:main",
            "insta360_stream = tron_sdk.insta360_stream:main",
        ],
    },
)
