import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  config_file_name = 'localization.yaml'
  config_path_ = os.path.join(get_package_share_directory('orange_sensor_tools'), 'config', config_file_name)
  config_path = LaunchConfiguration('config_filepath', default = config_path_)
  odom_in = LaunchConfiguration('odom_in', default = '/odom')
  imu_in = LaunchConfiguration('imu_in', default = '/imu')
  fusion_odom_out = LaunchConfiguration('fusion_odom_out', default = '/fusion/odom')
  use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')
  
  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value = 'true',
      description = 'Use simulation (Gazebo) clock if true'
    ),

    DeclareLaunchArgument(
      'config_filepath',
      default_value = config_path,
      description = 'robot_localization config file path'
    ),

    DeclareLaunchArgument(
      'odom_in',
      default_value = '/odom',
      description = 'Input odometry'
    ),

    DeclareLaunchArgument(
      'imu_in',
      default_value = '/imu',
      description = 'Input IMU'
    ),

    DeclareLaunchArgument(
      'fusion_odom_out',
      default_value = '/fusion/odom',
      description = 'Output fusion odometry'
    ),

    # robot_localization
    # wheel odometry + imu
    Node(
      package = 'robot_localization',
      executable = 'ekf_node',
      output = 'screen',
      parameters=[{'use_sim_time': use_sim_time}, config_path],
      remappings={('odom0', odom_in), ('imu0', imu_in), ('/odometry/filtered', fusion_odom_out)}
    )
  ])