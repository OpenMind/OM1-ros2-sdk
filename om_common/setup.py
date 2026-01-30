from setuptools import find_packages, setup

package_name = "om_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/om_common"]),
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
            "d435_obstacle_dector = om_common.d435_obstacle_dector:main",
            "d435_camera_stream = om_common.d435_camera_stream:main",
            "laser_scan_localization = om_common.laser_scan_localization:main",
            "local_traversability_node = om_common.local_traversability_node:main",
            "om_path = om_common.om_path:main",
        ],
    },
)
