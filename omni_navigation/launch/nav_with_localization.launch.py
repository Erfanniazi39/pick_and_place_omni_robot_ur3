import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package paths
    nav_pkg = get_package_share_directory('nav2_bringup')
    this_pkg = get_package_share_directory('omni_navigation')
    
    # Declare arguments
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(this_pkg, 'maps', 'my_map.yaml'),
        description='Path to the map YAML file'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(this_pkg, 'config', 'nav2_params.yaml'),
        description='Path to the Nav2 parameters file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Autostart the navigation stack'
    )
    
    # Create launch configuration variables for substitution
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    # Include Nav2 localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )
    
    # Include Nav2 navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )

    return LaunchDescription([
        map_yaml_arg,
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        localization_launch,
        navigation_launch,
    ])
