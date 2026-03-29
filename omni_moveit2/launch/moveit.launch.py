#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    omni_moveit2_dir = get_package_share_directory('omni_moveit2')
    omni_description_dir = get_package_share_directory('omni_description')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                os.path.join(omni_description_dir, 'urdf', 'omni_robot.urdf.xacro'),
            ]),
            value_type=str,
        )
    }

    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            open(
                os.path.join(omni_moveit2_dir, 'config', 'omni_robot.srdf'),
                'r',
                encoding='utf-8',
            ).read(),
            value_type=str,
        )
    }

    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml(
            os.path.join(omni_moveit2_dir, 'config', 'kinematics.yaml')
        )
    }

    ompl_config = {
        'ompl': load_yaml(os.path.join(omni_moveit2_dir, 'config', 'moveit_planning.yaml'))
    }

    moveit_controllers = load_yaml(
        os.path.join(omni_moveit2_dir, 'config', 'moveit_controllers.yaml')
    )

    planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
    }

    trajectory_execution = {
        'trajectory_execution': {
            'allowed_execution_duration_scaling': 1.5,
            'allowed_goal_duration_margin': 0.75,
            'moveit_manage_controllers': False,
        }
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_config,
            ompl_config,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_base',
        output='screen',
        arguments=[
            '--x',
            '0',
            '--y',
            '0',
            '--z',
            '0',
            '--roll',
            '0',
            '--pitch',
            '0',
            '--yaw',
            '0',
            '--frame-id',
            'world',
            '--child-frame-id',
            'base_footprint',
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation (Gazebo) clock if true',
            ),
            move_group_node,
            static_tf_node,
        ]
    )
