#!/usr/bin/env python3
"""
Unified launch for Gazebo + Nav2 localization/navigation + RViz.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package paths
    omni_gazebo_pkg = get_package_share_directory('omni_gazebo')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    omni_nav_pkg = get_package_share_directory('omni_navigation')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Autostart navigation stack'
    )
    
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(omni_nav_pkg, 'maps', 'my_map.yaml'),
        description='Map YAML file'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(omni_nav_pkg, 'config', 'nav2_params.yaml'),
        description='Nav2 parameters file'
    )

    use_gz_gui_arg = DeclareLaunchArgument(
        'use_gz_gui',
        default_value='false',
        description='Launch Gazebo GUI client. Disable to avoid gz GUI symbol issues.'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gz_gui = LaunchConfiguration('use_gz_gui')
    
    # Step 1: Gazebo (provides simulation clock)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(omni_gazebo_pkg, 'launch', 'omni_gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gz_gui': use_gz_gui,
        }.items()
    )
    
    # Step 2: Localization (AMCL + map_server)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )

    # Step 3: Navigation-only stack (planner/controller/bt/etc), no docking server
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(omni_nav_pkg, 'launch', 'navigation_no_docking.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )
    
    # Step 4: RViz (use workspace config to avoid nav2 default docking panel crash)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(omni_nav_pkg, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    delayed_nav_bringup = TimerAction(
        period=8.0,
        actions=[
            localization_launch,
            navigation_launch,
            rviz_node,
        ],
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        autostart_arg,
        map_yaml_arg,
        params_file_arg,
        use_gz_gui_arg,
        use_rviz_arg,
        # Launches (Gazebo first, then delayed Nav2 bringup)
        gazebo_launch,
        delayed_nav_bringup,
    ])
