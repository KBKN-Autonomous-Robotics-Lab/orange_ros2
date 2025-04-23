from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/livox_points/obstacle'),
                        ('scan', '/livox_scan')],
            parameters=[{
                'target_frame': '',
                'transform_tolerance': 0.01,
                'min_height': -0.2,  # -20cm
                'max_height': 0.2,  # 70cm=bringup 20cm=simulation
                'angle_min': -3.1415,  # - M_PI
                'angle_max': 3.1415,  # M_PI
                'angle_increment': 0.0174,  # M_PI * 2 / 360.0 = 1 degree
                'scan_time': 0.1,  # 10Hz
                'range_min': 0.4,  # 20cm
                'range_max': 70.0,  # 70m
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
