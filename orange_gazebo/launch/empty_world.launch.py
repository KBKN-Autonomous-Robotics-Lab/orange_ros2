import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  xacro_file_name = 'orange_robot.xacro'
  world_file_name = 'empty.world'
  xacro_path = os.path.join(get_package_share_directory('orange_description'), 'xacro', xacro_file_name)
  world_path = os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', world_file_name)
  use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')

  # Pose where we want to spawn the robot
  spawn_x = '0.0'
  spawn_y = '0.0'
  spawn_z = '0.0'
  spawn_yaw = '0.0'

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value = 'true',
      description = 'Use simulation (Gazebo) clock if true'
    ),

    # gzserver
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
      ),
      launch_arguments = {'world': world_path}.items()
    ),

    # gzclient
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
      )
    ),

    # robot_state_publisher
    Node(
      package = 'robot_state_publisher',
      executable = 'robot_state_publisher',
      output = 'screen',
      parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', xacro_path])}]
    ),

    # spawn_entity
    Node(
      package = 'gazebo_ros',
      executable = 'spawn_entity.py',
      respawn = True,
      output = 'screen',
      arguments = ['-entity', 'my_test_robot', '-topic', '/robot_description', '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw]
    )
  ])