import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_coverage')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'params', 'executor_params.yaml'),
            description='Optional parameter file for the coverage executor',
        ),
        Node(
            package='my_coverage',
            executable='coverage_executor',
            name='coverage_executor',
            output='screen',
            parameters=[params_file],
        ),
    ])
