from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import launch.conditions

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

    disable_metrics_arg = DeclareLaunchArgument(
        'disable_metrics',
        default_value='false',
        description='Disable metrics collection if true'
    )

    timed_run_arg = DeclareLaunchArgument(
        'timed_run',
        default_value='0',
        description='Duration of timed run in seconds (0 for unlimited)'
    )

    # Create base launch description with common nodes
    ld = [
        ws_dir_arg,
        workload_arg,
        disable_metrics_arg,
        timed_run_arg,
        Node(
            package='mp_eval',
            executable='mp_eval',
            name='mp_eval',
            parameters=[{
                'ws_dir': LaunchConfiguration('ws_dir'),
                'workload': LaunchConfiguration('workload'),
                'timed_run': LaunchConfiguration('timed_run')
            }],
            output='screen'
        )
    ]

    # Conditionally add metrics collector node based on launch argument
    disable_metrics = LaunchConfiguration('disable_metrics')
    ld.append(Node(
        package='mp_eval',
        executable='metrics_collector',
        name='metrics_collector',
        parameters=[{
            'ws_dir': LaunchConfiguration('ws_dir'),
            'workload': LaunchConfiguration('workload')
        }],
        output='screen',
        condition=launch.conditions.UnlessCondition(disable_metrics)
    ))

    return LaunchDescription(ld)
