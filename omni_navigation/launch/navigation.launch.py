# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.conditions import IfCondition

# def generate_launch_description():

#     # Create the launch configuration variables
#     namespace = LaunchConfiguration('namespace')
#     use_namespace = LaunchConfiguration('use_namespace')
#     slam = LaunchConfiguration('slam')
#     map_yaml_file = LaunchConfiguration('map')
#     use_localization = LaunchConfiguration('use_localization')
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     params_file = LaunchConfiguration('params_file')
#     autostart = LaunchConfiguration('autostart')
#     use_composition = LaunchConfiguration('use_composition')
#     use_respawn = LaunchConfiguration('use_respawn')
#     log_level = LaunchConfiguration('log_level')
#     # Create the launch configuration variables specific to RViz
#     use_rviz = LaunchConfiguration('use_rviz')
#     rviz_file = LaunchConfiguration('rviz_file')

#     # Declare the launch arguments
#     declare_namespace_cmd = DeclareLaunchArgument(
#         name='namespace',
#         default_value='',
#         description='Top-level namespace'
#     )

#     declare_use_namespace_cmd = DeclareLaunchArgument(
#         name='use_namespace',
#         default_value='false',
#         description='Whether to apply a namespace to the navigation stack'
#     )

#     declare_slam_cmd = DeclareLaunchArgument(
#         name='slam',
#         default_value='False',
#         description='Whether run a SLAM'
#     )

#     declare_map_yaml_cmd = DeclareLaunchArgument(
#         name='map',
#         default_value=os.path.join(get_package_share_directory('omni_navigation'), 'maps', '1311_v10.yaml'),
#         description='Full path to map file to load'
#     )

#     declare_use_localization_cmd = DeclareLaunchArgument(
#         name='use_localization',
#         default_value='True',
#         description='Whether to enable localization or not'
#     )

#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         name='use_sim_time',
#         default_value='true',
#         description='Use simulation (Gazebo) clock if true'
#     )

#     declare_params_file_cmd = DeclareLaunchArgument(
#         name='params_file',
#         default_value=os.path.join(get_package_share_directory('omni_navigation'), 'config', 'nav2_params.yaml'),
#         description='Full path to the ROS2 parameters file to use for all launched nodes'
#     )

#     declare_autostart_cmd = DeclareLaunchArgument(
#         name='autostart',
#         default_value='true',
#         description='Automatically startup the nav2 stack'
#     )

#     declare_use_composition_cmd = DeclareLaunchArgument(
#         name='use_composition',
#         default_value='True',
#         description='Whether to use composed bringup'
#     )

#     declare_use_respawn_cmd = DeclareLaunchArgument(
#         name='use_respawn',
#         default_value='False',
#         description='Whether to respawn if a node crashes. Applied when composition is disabled.'
#     )

#     declare_log_level_cmd = DeclareLaunchArgument(
#         name='log_level',
#         default_value='info',
#         description='log level'
#     )

#     declare_use_rviz_cmd = DeclareLaunchArgument(
#         name='use_rviz',
#         default_value='false',
#         description='Whether to start RVIZ'
#     )

#     declare_rviz_file_cmd = DeclareLaunchArgument(
#         name='rviz_file',
#         default_value=os.path.join(get_package_share_directory('omni_navigation'), 'rviz', 'navigation.rviz'),
#         description='Full path to the RVIZ config file to use'
#     )

#     # Declare the launch files
#     bringup_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             [os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
#         launch_arguments={'namespace': namespace,
#                           'use_namespace': use_namespace,
#                           'slam': slam,
#                           'map': map_yaml_file,
#                           'use_localization': use_localization,
#                           'use_sim_time': use_sim_time,
#                           'params_file': params_file,
#                           'autostart': autostart,
#                           'use_composition': use_composition,
#                           'use_respawn': use_respawn,
#                           'log_level': log_level}.items()
#     )

#     rviz_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             [os.path.join(get_package_share_directory('omni_navigation'), 'launch', 'rviz.launch.py')]),
#         condition=IfCondition(use_rviz),
#         launch_arguments={'namespace': namespace,
#                           'use_sim_time': use_sim_time,
#                           'rviz_file': rviz_file}.items()
# 	)

#     # Create the launch description and populate
#     ld = LaunchDescription()

#     # Declare the launch options
#     ld.add_action(declare_namespace_cmd)
#     ld.add_action(declare_use_namespace_cmd)
#     ld.add_action(declare_slam_cmd)
#     ld.add_action(declare_map_yaml_cmd)
#     ld.add_action(declare_use_localization_cmd)
#     ld.add_action(declare_use_sim_time_cmd)
#     ld.add_action(declare_params_file_cmd)
#     ld.add_action(declare_autostart_cmd)
#     ld.add_action(declare_use_composition_cmd)
#     ld.add_action(declare_use_respawn_cmd)
#     ld.add_action(declare_log_level_cmd)

#     ld.add_action(declare_use_rviz_cmd)
#     ld.add_action(declare_rviz_file_cmd)

#     # Add the actions to launch all of the navigation nodes
#     ld.add_action(bringup_cmd)
#     ld.add_action(rviz_cmd)

#     return ld

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='True',
        description='Run SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=os.path.join(
            get_package_share_directory('omni_navigation'), 'maps', 'my_map.yaml'),
        description='Full path to map yaml file when running localization'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=os.path.join(
            get_package_share_directory('omni_navigation'), 'config', 'nav2_params.yaml'),
        description='Nav2 params file'
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(bringup_cmd)

    return ld