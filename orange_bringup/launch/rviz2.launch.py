import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  config_file_name = 'orange_robot.rviz'
  config_path_ = os.path.join(get_package_share_directory('orange_description'), 'rviz2', config_file_name)
  config_path = LaunchConfiguration('config_filepath', default = config_path_)

  return LaunchDescription([
    DeclareLaunchArgument(
      'config_filepath',
      default_value = config_path_,
      description = 'Rviz2 config file name'
    ),

    # rviz2
    Node(
      package = 'rviz2',
      executable = 'rviz2',
      output = 'screen',
      arguments=['-d', config_path]
    )
  ])