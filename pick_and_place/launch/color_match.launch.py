#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    wait_sec = LaunchConfiguration('wait_sec')

    cube_spawner = Node(
        package='pick_and_place',
        executable='cube_spawner',
        name='cube_spawner',
        output='screen',
    )

    detect_small_cubes = Node(
        package='pick_and_place',
        executable='detect_small_cubes',
        name='detect_small_cubes',
        output='screen',
    )

    pick_middle_cube = Node(
        package='pick_and_place',
        executable='pick_middle_cube',
        name='pick_middle_cube',
        output='screen',
    )

    rotate_scan_color = Node(
        package='pick_and_place',
        executable='rotate_scan_color',
        name='rotate_scan_color',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('wait_sec', default_value='3.0'))
    ld.add_action(cube_spawner)
    ld.add_action(TimerAction(period=wait_sec, actions=[detect_small_cubes]))
    ld.add_action(TimerAction(period=PythonExpression([wait_sec, ' * 2']), actions=[pick_middle_cube]))
    ld.add_action(TimerAction(period=PythonExpression([wait_sec, ' * 3']), actions=[rotate_scan_color]))

    return ld
