import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():

    # -------------------------------
    # Launch configuration variables
    # -------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    Y_orient = LaunchConfiguration('Y_orient')
    use_controllers = LaunchConfiguration('use_controllers')

    # -------------------------------
    # Declare launch arguments
    # -------------------------------
    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_x_pos_arg = DeclareLaunchArgument(
        name='x_pos',
        default_value='0.0',
        description='Robot position (x)'
    )

    declare_y_pos_arg = DeclareLaunchArgument(
        name='y_pos',
        default_value='0.0',
        description='Robot position (y)'
    )

    declare_Y_orient_arg = DeclareLaunchArgument(
        name='Y_orient',
        default_value='0.0',
        description='Robot orientation (yaw)'
    )

    declare_use_controllers_arg = DeclareLaunchArgument(
        name='use_controllers',
        default_value='true',
        description='Start ros2_control controller spawners'
    )

    # -------------------------------
    # Include robot description launch
    # -------------------------------
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('omni_description'), 'launch', 'description.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': use_controllers,
        }.items()
    )


    robot_controllers_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('omni_control'), 'launch', 'controllers.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_controllers)
	)

    # -------------------------------
    # Spawn robot in Gazebo
    # -------------------------------
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-x', x_pos,
            '-y', y_pos,
            '-Y', Y_orient
        ]
    )

    # -------------------------------
    # Gazebo ROS bridge for robot
    # -------------------------------
    gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='robot_ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('omni_gazebo'), 'config', 'bridge.yaml')
        }]
    )

    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/omni_controller/cmd_vel')
        ]
    )

    # Camera node for puzzle detection
    camera_node = Node(
        package='pick_and_place',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # -------------------------------
    # Create launch description
    # -------------------------------
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_x_pos_arg)
    ld.add_action(declare_y_pos_arg)
    ld.add_action(declare_Y_orient_arg)
    ld.add_action(declare_use_controllers_arg)

    # Add included launches and nodes
    ld.add_action(robot_description_launch)
    ld.add_action(robot_controllers_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(gazebo_ros_bridge_cmd)
    ld.add_action(twist_stamper_node)
    ld.add_action(camera_node)

    return ld