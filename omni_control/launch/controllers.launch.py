from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():

	# Create the launch configuration variables
	use_sim_time = LaunchConfiguration('use_sim_time')

	# Declare the launch arguments
	declare_use_sim_time_arg = DeclareLaunchArgument(
		name='use_sim_time',
		default_value='false',
		description='Use simulation (Gazebo) clock if true'
	)

	# Declare the launch nodes
	joint_state_broadcaster_spawner_cmd = Node(
		package="controller_manager",
		executable="spawner",
		name='joint_state_broadcaster',
		arguments=['joint_state_broadcaster',
			 	   '--controller-manager-timeout', '20'],
		parameters=[{'use_sim_time': use_sim_time}]
	)

	arm_controller_spawner_cmd = Node(
		package="controller_manager",
		executable="spawner",
		name='robot_controller',
		arguments=['omni_controller'],
		parameters=[{'use_sim_time': use_sim_time}]
	)

	ur3_controller_spawner_cmd = Node(
		package="controller_manager",
		executable="spawner",
		name='ur3_controller',
		arguments=['ur3_controller'],
		parameters=[{'use_sim_time': use_sim_time}]
	)

	gripper_controller_spawner_cmd = Node(
		package="controller_manager",
		executable="spawner",
		name='gripper_controller',
		arguments=['gripper_controller'],
		parameters=[{'use_sim_time': use_sim_time}]
	)

	delayed_arm_controller_spawner_cmd = RegisterEventHandler(
		event_handler=OnProcessExit(
			target_action=joint_state_broadcaster_spawner_cmd,
			on_exit=[arm_controller_spawner_cmd, ur3_controller_spawner_cmd, gripper_controller_spawner_cmd]
		)
	)

	odom_relay_cmd = Node(
		package='topic_tools',
		executable='relay',
		name='odom_relay',
		parameters=[{'use_sim_time': use_sim_time}],
		arguments=['/omni_controller/odom', '/odom']
	)

	# Create the launch description and populate
	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_use_sim_time_arg)

	# Add the actions to launch all of the files and nodes
	ld.add_action(joint_state_broadcaster_spawner_cmd)
	ld.add_action(delayed_arm_controller_spawner_cmd)
	ld.add_action(odom_relay_cmd)
	
	return ld