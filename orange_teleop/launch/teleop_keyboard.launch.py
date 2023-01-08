from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  cmd_vel = LaunchConfiguration('cmd_vel', default = '/cmd_vel')

  return LaunchDescription([
    DeclareLaunchArgument(
      'cmd_vel',
      default_value = '/cmd_vel',
      description = 'Topic name for cmd_vel'
    ),

    # teleop_twist_keyboard
    Node(
      package = 'orange_teleop',
      executable = 'teleop_twist_keyboard',
      output = 'screen',
      prefix = 'xterm -e',
      remappings = [('cmd_vel', cmd_vel)]
    )
  ])