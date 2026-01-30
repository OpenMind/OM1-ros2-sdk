from setuptools import find_packages, setup

package_name = "unitree_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/unitree_common"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="openmind",
    maintainer_email="hello@openmind.org",
    description="Common shared modules for Unitree robots",
    license="MIT",
    entry_points={
        "console_scripts": [
            "d435_obstacle_dector = unitree_common.d435_obstacle_dector:main",
            "go2_lidar_localization = unitree_common.go2_lidar_localization:main",
            "local_traversability_node = unitree_common.local_traversability_node:main",
            "om_path = unitree_common.om_path:main",
        ],
    },
)
