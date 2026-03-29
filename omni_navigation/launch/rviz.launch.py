import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    rviz_file = LaunchConfiguration('rviz_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description=('Top-level namespace')
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
		name='use_sim_time',
		default_value='true',
		description='Use simulation (Gazebo) clock if true'
	)

    declare_rviz_file_cmd = DeclareLaunchArgument(
        name='rviz_file',
        default_value=os.path.join(get_package_share_directory('omni_navigation'), 'rviz', 'navigation.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
		output='screen',
        namespace=namespace,
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/map', 'map'),
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')]
    )

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_file_cmd)

    # Add the actions to launch all of the files and nodes
    ld.add_action(rviz_cmd)

    # Add other nodes and processes we need
    ld.add_action(exit_event_handler)

    return ld
