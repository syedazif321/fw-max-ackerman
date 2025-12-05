import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # --- 1. CONFIGURATION & PATHS ---

    pkg_ackerman_description = get_package_share_directory('ackerman_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Path to your XACRO file
    urdf_file_path = os.path.join(pkg_ackerman_description, 'urdf', 'fw-max.xacro')

    # Path to your controller configuration YAML
    controller_config_path = os.path.join(pkg_ackerman_description, 'config', 'fw_max_controllers.yaml')

    # Path to your custom Gazebo world
    custom_world_path = '/home/azif/projetcs/fw-max-ackerman/ackerman_description/world/no_roof_small_warehouse.world'

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_arg = LaunchConfiguration('world', default=custom_world_path)

    # --- 2. ROBOT DESCRIPTION & STATE PUBLISHER ---

    # Convert XACRO to URDF
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # --- 3. GAZEBO LAUNCH ---

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world_arg
        }.items(),
    )

    # --- 4. GAZEBO SPAWN ENTITY ---

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'fw_max',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # --- 5. CONTROLLER SPAWNERS ---

    # Controller manager (ros2_control)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config_path],
        output='screen',
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Wheel velocity controller
    wheel_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # Steering controller
    steering_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_position_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # --- 6. EVENT HANDLERS (Sequential Spawning) ---

    # Wait for robot to spawn → then load JSB
    load_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Wait for JSB → then load main controllers
    load_main_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[wheel_velocity_controller_spawner, steering_position_controller_spawner]
        )
    )

    # --- 7. RETURN LAUNCH DESCRIPTION ---

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=custom_world_path,
            description='Path to the Gazebo world file to load'
        ),

        # Main Components
        gazebo,
        robot_state_publisher_node,
        spawn_entity,

        # Controller Spawning
        load_jsb,
        load_main_controllers,
    ])
