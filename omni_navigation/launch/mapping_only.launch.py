#!/usr/bin/env python3
"""
Mapping-only launch: Gazebo + robot spawn + SLAM (+ optional RViz).
No AMCL/Nav2 servers are launched to avoid map corruption while mapping.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    omni_gazebo_pkg = get_package_share_directory('omni_gazebo')
    omni_nav_pkg = get_package_share_directory('omni_navigation')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )

    use_gz_gui_arg = DeclareLaunchArgument(
        'use_gz_gui',
        default_value='false',
        description='Launch Gazebo GUI client',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gz_gui = LaunchConfiguration('use_gz_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(omni_gazebo_pkg, 'launch', 'omni_gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gz_gui': use_gz_gui,
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(omni_gazebo_pkg, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(omni_nav_pkg, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # Delay SLAM and RViz until Gazebo and robot spawn are up.
    delayed_mapping_tools = TimerAction(
        period=9.0,
        actions=[slam_launch, rviz_node],
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_gz_gui_arg,
        use_rviz_arg,
        gazebo_launch,
        delayed_mapping_tools,
    ])
