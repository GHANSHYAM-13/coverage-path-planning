"""
Launch file for coverage using standard Nav2 bringup.

This launch file is for robots where nav2_bringup bringup_launch.py is already
working. Instead of using the separate my_coverage stack, this file includes
the standard nav2_bringup and adds only the coverage-specific components on top.

Usage:
    ros2 launch my_coverage coverage_bringup.launch.py \
        map:=/path/to/map.yaml \
        use_sim_time:=True
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Generate launch description for coverage with standard Nav2 bringup."""
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_dir = get_package_share_directory('my_coverage')

    # Launch arguments
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    container_name = LaunchConfiguration('container_name')
    coverage_container_name = LaunchConfiguration('coverage_container_name')

    # Coverage-specific lifecycle nodes
    coverage_lifecycle_nodes = [
        'coverage_server',
        'bt_navigator',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Launch Arguments
        DeclareLaunchArgument('map', description='Full path to the map yaml file'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'params', 'coverage_nav2_params.yaml'),
            description='Parameters file for coverage components',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument(
            'coverage_container_name',
            default_value='coverage_container',
            description='Name of the container for coverage components',
        ),

        # Include standard Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': '',
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': container_name,
                'log_level': 'info',
            }.items(),
        ),

        # Coverage-specific container with composable nodes
        Node(
            name=coverage_container_name,
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            remappings=remappings,
            output='screen',
        ),

        # Load coverage components
        LoadComposableNodes(
            target_container=coverage_container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package='backported_bt_navigator',
                    plugin='backported_bt_navigator::BtNavigator',
                    name='coverage_bt_navigator',
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                ComposableNode(
                    package='opennav_coverage',
                    plugin='opennav_coverage::CoverageServer',
                    name='coverage_server',
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='coverage_lifecycle_manager',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': coverage_lifecycle_nodes,
                    }],
                ),
            ],
        ),
    ])
