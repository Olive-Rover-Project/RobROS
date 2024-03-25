from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Declare the path to your map and nav2 parameter files as launch arguments
    map_yaml_file = DeclareLaunchArgument(
        'map_yaml_file',
        default_value='/home/vboxuser/ros2_ws/src/olive_rover/config/map.yaml',
        description='Full path to the map yaml file to load')

    nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value='/home/vboxuser/ros2_ws/src/olive_rover/config/nav2_params.yaml',
        description='Full path to the nav2 parameters yaml file')

    # Launch the map server and remap the map topic to /static_map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[LaunchConfiguration('map_yaml_file')],
        remappings=[('/map', '/static_map')])

    # Set the map server to the configured state
    from launch.actions import ExecuteProcess

    # Assuming 'map_server' is the node name you want to configure and activate
    bringup_map_server = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
        output='screen'
    )

    activate_map_server = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
        output='screen'
    )


    # Include the Olive Rover simulation launch file
    olive_rover_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/home/vboxuser/ros2_ws/src/olive_rover/launch/launch_sim.launch.py']),
        launch_arguments={'use_sim_time': 'true'}.items())

    # Include the Nav2 bringup launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/home/vboxuser/ros2_ws/src/olive_rover/launch/navigation_launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': LaunchConfiguration('nav2_params_file')
        }.items())

    # Include the SLAM Toolbox launch file for online asynchronous mapping
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/home/vboxuser/ros2_ws/src/olive_rover/launch/online_async_launch.py']),
        launch_arguments={'use_sim_time': 'true'}.items())

    return LaunchDescription([
        map_yaml_file,
        nav2_params_file,
        map_server_node,
        set_map_server_configure,
        set_map_server_activate,
        olive_rover_sim_launch,
        nav2_bringup_launch,
        slam_toolbox_launch
    ])

