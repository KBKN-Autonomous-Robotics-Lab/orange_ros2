import os
from glob import glob

from setuptools import setup

package_name = "orange_gnss"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer=["Saito"],
    maintainer_email=[
        "shun.saito.6t@stu.hosei.ac.jp",
    ],
    description="For GNSS",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "GPSodom_correction = orange_gnss.GPSodom_correction:main",
            "combination = orange_gnss.combination:main",
            "ekf_myself = orange_gnss.ekf_myself:main",
            "ekf_myself_noGPS = orange_gnss.ekf_myself_noGPS:main",
            "fix_to_GPSodom = orange_gnss.fix_to_GPSodom:main",
            "get_lonlat_ttyACM = orange_gnss.get_lonlat_ttyACM:main",
            "get_lonlat_ttyUSB = orange_gnss.get_lonlat_ttyUSB:main",
            "get_movingbase_quat_ttyUSB = orange_gnss.get_movingbase_quat_ttyUSB:main",
            "gnss_odom_movingbase_fix_publisher_ttyUSB = orange_gnss.gnss_odom_movingbase_fix_publisher_ttyUSB:main",
            "gnss_odom_publisher_ttyUSB = orange_gnss.gnss_odom_publisher_ttyUSB:main",
            "lonlat_to_odom = orange_gnss.lonlat_to_odom:main",
            "movingbase_yaw_to_quat = orange_gnss.movingbase_yaw_to_quat:main"
        ],
    },
)
