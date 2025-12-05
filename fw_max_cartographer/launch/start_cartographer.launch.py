import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_cartographer_ros = get_package_share_directory('cartographer_ros')
    pkg_fw_max_cartographer = get_package_share_directory('fw_max_cartographer')

    # Path to the configuration file
    configuration_directory = os.path.join(pkg_fw_max_cartographer, 'config')
    configuration_basename = 'fw_max_3d.lua'

    return LaunchDescription([
        # Start Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename,
                '--ros-args', '--log-level', 'WARN'
            ],
        ),

        # Start Cartographer Occupancy Grid Node (to visualize the map in 2D)
        # Node(
        #     package='cartographer_ros',
        #     executable='occupancy_grid_node',
        #     name='occupancy_grid_node',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': True,
        #         'resolution': 0.05,
        #         'publish_frame_ids': ['map', 'odom']
        #     }],
        # ),

        # Publish the 3D Point Cloud topic to Cartographer's input
        Node(
            package='topic_tools',
            executable='relay',
            name='lidar_relay',
            output='screen',
            parameters=[{
                'input_topic': '/fw_max/points_raw',  # Your LiDAR topic
                'output_topic': '/points2'           # Cartographer's expected input topic
            }],
        ),
    ])