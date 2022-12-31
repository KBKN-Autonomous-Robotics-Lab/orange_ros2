import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  world = LaunchConfiguration('world', default=[FindPackageShare('orange_gazebo'), '/worlds/orange_world.world'])

  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
      ),
      launch_arguments={'world': world}.items(),
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
      ),
    ),
  ])