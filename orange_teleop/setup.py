import os
from glob import glob

from setuptools import find_packages, setup

package_name = "orange_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=[]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Mori",
    author_email="kimihiro.mori.4k@stu.hosei.ac.jp",
    maintainer="Shibuya",
    maintainer_email="shunki.shibuya.5v@stu.hosei.ac.jp",
    keywords=["ROS", "ROS2"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description=("This project is to use orange robot with ROS2"),
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_twist_keyboard = script.teleop_twist_keyboard:main"
        ],
    },
)
