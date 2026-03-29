import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

	# Create the launch configuration variables
	use_sim_time = LaunchConfiguration('use_sim_time')

	# Declare the launch arguments
	declare_use_sim_time_arg = DeclareLaunchArgument(
		name='use_sim_time',
		default_value='false',
		description='Use simulation (Gazebo) clock if true'
	)

	# Declare the launch files
	robot_description_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[os.path.join(get_package_share_directory('omni_description'), 'launch', 'description.launch.py')]),
		launch_arguments={'use_sim_time': use_sim_time}.items()
	)

	# Declare the launch nodes
	joint_state_publisher_cmd = Node(
		package='joint_state_publisher',
		executable='joint_state_publisher',
		name='joint_state_publisher',
		output='screen',
		parameters=[{'use_sim_time': use_sim_time}]
	)


	rviz_cmd = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', os.path.join(get_package_share_directory('omni_description'), 'rviz', 'display.rviz')],
		parameters=[{'use_sim_time': use_sim_time}]
	)

	

	# Create the launch description and populate
	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_use_sim_time_arg)

	# Add the actions to launch all of the files and nodes
	ld.add_action(robot_description_cmd)
	ld.add_action(joint_state_publisher_cmd)
	ld.add_action(rviz_cmd)

	return ld

	

	# Create the launch description and populate
	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_use_sim_time_arg)

	# Add the actions to launch all of the files and nodes
	ld.add_action(robot_description_cmd)
	ld.add_action(joint_state_publisher_gui_cmd)
	ld.add_action(rviz_cmd)

	return ld