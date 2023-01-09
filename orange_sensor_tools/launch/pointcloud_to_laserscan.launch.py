from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  cloud_in = LaunchConfiguration('cloud_in', default = '/velodyne_points')
  scan_out = LaunchConfiguration('scan_out', default = '/velodyne_scan')
  use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value = 'true',
      description = 'Use simulation (Gazebo) clock if true'
    ),

    DeclareLaunchArgument(
      'cloud_in',
      default_value = '/velodyne_points',
      description = 'Input point cloud'
    ),

    DeclareLaunchArgument(
      'scan_out',
      default_value = '/velodyne_scan',
      description = 'Output laser scan'
    ),

    # pointcloud_to_laserscan
    Node(
      package = 'pointcloud_to_laserscan',
      executable = 'pointcloud_to_laserscan_node',
      output = 'screen',
      parameters=[{
        'target_frame': 'velodyne_link',
        'transform_tolerance': 0.01,
        'min_height': -0.8,
        'max_height': 0.3,
        'angle_min': -2.65,
        'angle_max': 2.65,
        'angle_increment': 0.0087,
        'scan_time': 0.3333,
        'range_min': 0.2,
        'range_max': 100.0,
        'use_inf': True,
        'inf_epsilon': 1.0,
        'use_sim_time': use_sim_time
      }],
      remappings={('cloud_in', cloud_in), ('scan', scan_out)}
    )
  ])