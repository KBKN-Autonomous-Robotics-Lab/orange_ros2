import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  config_file_name = 'orange_robot.rviz'
  config_path = os.path.join(get_package_share_directory('orange_description'), 'rviz2', config_file_name)

  return LaunchDescription([

    # rviz2
    Node(
      package = 'rviz2',
      executable = 'rviz2',
      output = 'screen',
      arguments=['-d', config_path]
    )
  ])