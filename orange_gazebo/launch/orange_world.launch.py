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
  world_file_name = 'orange_world.world'
  cloud_in = '/velodyne_points'
  scan_out = '/velodyne_scan'
  odom_in = '/odom'
  imu_in = '/imu'
  fusion_odom_out = '/fusion/odom'
  xacro_path = os.path.join(get_package_share_directory('orange_description'), 'xacro', xacro_file_name)
  world_path = os.path.join(get_package_share_directory('orange_gazebo'), 'worlds', world_file_name)
  use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value = 'true',
      description = 'Use simulation (Gazebo) clock if true'
    ),

    # pointcloud_to_laserscan
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('orange_sensor_tools'), 'launch', 'pointcloud_to_laserscan.launch.py')
      ),
      launch_arguments = {
        'use_sim_time': use_sim_time,
        'cloud_in': cloud_in,
        'scan_out': scan_out
      }.items()
    ),

    # robot_localization
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('orange_sensor_tools'), 'launch', 'localization.launch.py')
      ),
      launch_arguments = {
        'use_sim_time': use_sim_time,
        'odom_in': odom_in,
        'imu_in': imu_in,
        'fusion_odom_out': fusion_odom_out
      }.items()
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
      parameters = [{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', xacro_path])}]
    ),

    # spawn_entity
    Node(
      package = 'gazebo_ros',
      executable = 'spawn_entity.py',
      output = 'screen',
      arguments = ['-entity', 'orange_robot', '-topic', '/robot_description']
    )
  ])