from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    ws_dir_arg = DeclareLaunchArgument(
        'ws_dir',
        default_value='.',
        description='Working directory path'
    )

    workload_arg = DeclareLaunchArgument(
        'workload',
        description='Workload configuration'
    )

    return LaunchDescription([
        ws_dir_arg,
        workload_arg,
        Node(
            package='mp_eval',
            executable='mp_eval',
            name='mp_eval',
            parameters=[{
                'ws_dir': LaunchConfiguration('ws_dir'),
                'workload': LaunchConfiguration('workload')
            }],
            output='screen'
        ),
        Node(
            package='mp_eval',
            executable='metrics_collector',
            name='metrics_collector',
            parameters=[{
                'ws_dir': LaunchConfiguration('ws_dir'),
                'workload': LaunchConfiguration('workload')
            }],
            output='screen'
        )
    ])
