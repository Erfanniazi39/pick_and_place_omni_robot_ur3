import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

	xacro_file = os.path.join(get_package_share_directory('omni_description'), 'urdf', 'omni_robot.urdf.xacro')

	# Create the launch configuration variables
	use_sim_time = LaunchConfiguration('use_sim_time')
	use_ros2_control = LaunchConfiguration('use_ros2_control')

	# Declare the launch arguments
	declare_use_sim_time_arg = DeclareLaunchArgument(
		name='use_sim_time',
		default_value='false',
		description='Use simulation (Gazebo) clock if true'
	)

	declare_use_ros2_control_arg = DeclareLaunchArgument(
		name='use_ros2_control',
		default_value='true',
		description='Enable ros2_control tags in robot description'
	)

	# Declare the launch nodes
	robot_state_publisher_cmd = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		parameters=[
			{'robot_description': ParameterValue(
				Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control]),
				value_type=str
			)},
			{'use_sim_time': use_sim_time},
			# Accept joint updates even if simulation time jumps backward after reset/relaunch.
			{'ignore_timestamp': True},
			{'publish_frequency': 50.0},
			# Match joint_state_broadcaster QoS to keep wheel joint transforms flowing reliably.
			{'qos_overrides': {
				'/joint_states': {
					'subscription': {
						'reliability': 'reliable',
						'durability': 'transient_local',
						'history': 'keep_last',
						'depth': 20,
					}
				}
			}},
		]
	)

	# Create the launch description and populate
	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_use_sim_time_arg)
	ld.add_action(declare_use_ros2_control_arg)

	# Add the actions to launch all of the nodes
	ld.add_action(robot_state_publisher_cmd)

	return ld