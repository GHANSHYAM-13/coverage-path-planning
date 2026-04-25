from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    frame_id = LaunchConfiguration('frame_id')
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Optional map yaml path; if empty, reopen the last used map',
        ),
        Node(
            package='my_coverage',
            executable='gui_coverage',
            name='gui_coverage',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': frame_id,
            }],
        ),
        Node(
            package='my_coverage',
            executable='polygon_drawer',
            name='polygon_drawer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': frame_id,
                'map': map_yaml,
            }],
        ),
    ])
