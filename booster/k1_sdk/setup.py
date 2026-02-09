from glob import glob

from setuptools import find_packages, setup

package_name = "k1_sdk"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="openmind",
    maintainer_email="hello@openmind.org",
    description="Booster K1 robot SDK with sensor capabilities",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cmd_vel_to_k1 = k1_sdk.k1_movement:main",
            "k1_odom = k1_sdk.k1_odom:main",
            "insta360_stream = k1_sdk.insta360_stream:main",
        ],
    },
)
