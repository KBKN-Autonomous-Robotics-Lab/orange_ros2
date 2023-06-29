import os
from glob import glob

from setuptools import setup

package_name = "orange_teleop"

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
            "teleop_twist_keyboard = orange_teleop.teleop_twist_keyboard:main",
        ],
    },
)
