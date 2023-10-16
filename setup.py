import os
from glob import glob
from setuptools import find_packages, setup

package_name = "lsm9ds0_ros"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
    ],
    install_requires=["setuptools", "adafruit-circuitpython-lsm9ds0"],
    zip_safe=True,
    maintainer="Damien LaRocque",
    maintainer_email="phicoltan@gmail.com",
    description="ROS 2 driver for the Adafruit LSM9DS0 IMU",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lsm9ds0_node = lsm9ds0_ros.lsm9ds0_node:main",
        ],
    },
)
