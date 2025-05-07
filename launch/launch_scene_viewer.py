from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import launch.conditions



def generate_launch_description():
    # Declare launch arguments
    workload_arg = DeclareLaunchArgument(
        'workload',
        description='Workload configuration'
    )

    # Create base launch description with common nodes
    ld = [
        workload_arg,
        Node(
            package='mp_eval',
            executable='scene_viewer',
            name='scene_viewer',
            parameters=[{
                'workload': LaunchConfiguration('workload'),
            }],
            output='screen'
        )
    ]

    return LaunchDescription(ld)
