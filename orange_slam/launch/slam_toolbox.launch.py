import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  slam_config_file_name = 'slam_toolbox_params.yaml'
  slam_config_path = os.path.join(get_package_share_directory('orange_slam'), 'config', slam_config_file_name)
  rviz_config_path = os.path.join(get_package_share_directory('orange_description'), 'rviz2', 'slam_toolbox.rviz')
  use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')
  
  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value = 'true',
      description = 'Use simulation (Gazebo) clock if true'
    ),

    # rviz2
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('orange_bringup'), 'launch', 'rviz2.launch.py')
      ),
      launch_arguments = {'config_filepath': rviz_config_path}.items()
    ),

    # slam_toolbox
    Node(
      package = 'slam_toolbox',
      executable = 'async_slam_toolbox_node',
      output = 'screen',
      parameters=[{'use_sim_time': use_sim_time}, slam_config_path]
    )
  ])