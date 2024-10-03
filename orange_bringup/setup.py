import os
from glob import glob

from setuptools import setup

package_name = "orange_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer=["Shibuya", "Kubota", "Mori"],
    maintainer_email=[
        "shunki.shibuya.5v@stu.hosei.ac.jp",
        "koki.kubota.8p@stu.hosei.ac.jp",
        "kimihiro.mori.4k@stu.hosei.ac.jp",
    ],
    description="This project is to use orange robot with ROS2",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_driver_node = orange_bringup.motor_driver_node:main",
            "fix_to_GPSodom = orange_bringup.fix_to_GPSodom:main",
            "movingbase_yaw_to_quat = orange_bringup.movingbase_yaw_to_quat:main",
            "combination = orange_bringup.combination:main",
            "ekf_myself = orange_bringup.ekf_myself:main",
            "get_lonlat = orange_bringup.get_lonlat:main",
            "GPSodom_correction = orange_bringup.GPSodom_correction:main",
            "lonlat_to_odom = orange_bringup.lonlat_to_odom:main",
            "ekf_myself_noGPS = orange_bringup.ekf_myself_noGPS:main"
        ],
    },
)
