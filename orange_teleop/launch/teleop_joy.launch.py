import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  config_file_name = 'elecom.yaml'
  config_path_ = os.path.join(get_package_share_directory('orange_teleop'), 'config', config_file_name)
  joy_dev = LaunchConfiguration('joy_dev', default = '/dev/input/js0')
  config_path = LaunchConfiguration('config_filepath', default = config_path_)
  cmd_vel = LaunchConfiguration('cmd_vel', default = '/cmd_vel')

  return LaunchDescription([
    DeclareLaunchArgument(
      'joy_dev',
      default_value = '/dev/input/js0',
      description = 'Name of the joystick port'
    ),

    DeclareLaunchArgument(
      'config_filepath',
      default_value = config_path,
      description = 'Joystick config file path'
    ),

    DeclareLaunchArgument(
      'cmd_vel',
      default_value = '/cmd_vel',
      description = 'Topic name for cmd_vel'
    ),

    # joy node
    Node(
      package = 'joy',
      executable = 'joy_node',
      output = 'screen',
      parameters=[{'dev': joy_dev, 'deadzone': 0.3, 'autorepeat_rate': 20.0}]
    ),

    # teleop_twist_joy
    Node(
      package = 'teleop_twist_joy',
      executable = 'teleop_node',
      output = 'screen',
      parameters=[config_path],
      remappings={('/cmd_vel', cmd_vel)}
    )
  ])