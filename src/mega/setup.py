import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mega"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ivan Rubtsov",
    maintainer_email="vanischesmall@gmail.com",
    description="MEGA Boat ROS 2 package",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_machine = mega.state_machine:main",
            "stereo_camera = mega.stereo_camera:main",
            "bottom_camera = mega.bottom_camera:main",
            "serial_comm   = mega.serial_comm:main",
        ],
    },
)
