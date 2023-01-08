import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'orange_teleop'

setup(
  name=package_name,
  version='2.1.5',
  packages=find_packages(exclude=[]),
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name), glob('launch/*launch.py'))
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  author='alpaca-zip',
  author_email="zip@todo.todo",
  maintainer='alpaca-zip',
  maintainer_email="zip@todo.todo",
  keywords=['ROS', 'ROS2'],
  classifiers=[
    'Intended Audience :: Developers',
    'License :: OSI Approved :: Apache Software License',
    'Programming Language :: Python',
    'Topic :: Software Development'
  ],
  description=(
    'This project is to use orange robot with ROS2'
  ),
  license='Apache License, Version 2.0',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      'teleop_twist_keyboard = script.teleop_twist_keyboard:main'
    ],
  },
)