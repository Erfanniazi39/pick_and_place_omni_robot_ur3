import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import TimerAction

def generate_launch_description():

    # -------------------------------
    # Launch configuration variables
    # -------------------------------
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gz_gui = LaunchConfiguration('use_gz_gui')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    Y_orient = LaunchConfiguration('Y_orient')
    use_controllers = LaunchConfiguration('use_controllers')

    # -------------------------------
    # Declare the launch arguments
    # -------------------------------
    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(get_package_share_directory('omni_gazebo'), 'worlds', 'empty_world.world'),
        description='Specify world file'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_gz_gui_arg = DeclareLaunchArgument(
        name='use_gz_gui',
        default_value='false',
        description='Launch Gazebo GUI client (gz -g).'
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
    # Include other launch files
    # -------------------------------
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('omni_gazebo'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'use_gz_gui': use_gz_gui,
        }.items()
    )

    spawn_robot_cmd = TimerAction(
        period=5.0,  # seconds to wait for Gazebo
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('omni_gazebo'), 'launch', 'spawn.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'x_pos': x_pos,
                'y_pos': y_pos,
                'Y_orient': Y_orient,
                'use_controllers': use_controllers
            }.items()
        )]
    )

    # -------------------------------
    # Create the launch description
    # -------------------------------
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_world_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_use_gz_gui_arg)
    ld.add_action(declare_x_pos_arg)
    ld.add_action(declare_y_pos_arg)
    ld.add_action(declare_Y_orient_arg)
    ld.add_action(declare_use_controllers_arg)

    # Add the included launch files
    ld.add_action(gazebo_cmd)
    ld.add_action(spawn_robot_cmd)

    return ld