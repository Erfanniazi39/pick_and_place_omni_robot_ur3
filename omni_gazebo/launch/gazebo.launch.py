import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Create the launch configuration variables
    world = LaunchConfiguration('world')
    use_gz_gui = LaunchConfiguration('use_gz_gui')

    # Declare the launch arguments
    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(get_package_share_directory('omni_gazebo'), 'worlds', 'empty_world.world'),
        description='Specify world file'
    )

    declare_use_gz_gui_arg = DeclareLaunchArgument(
        name='use_gz_gui',
        default_value='false',
        description='Launch Gazebo GUI client (gz -g). Disable on systems with GUI GLIBC conflicts.'
    )

    # Declare the launch files
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v4 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v4 '
        }.items(),
        condition=IfCondition(use_gz_gui)
    )

    # Declare the launch nodes
    gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('omni_gazebo'), 'config', 'gz_bridge.yaml')
        }]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_arg)
    ld.add_action(declare_use_gz_gui_arg)

    # Add the actions to launch all of the files and nodes
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(gazebo_ros_bridge_cmd)

    return ld