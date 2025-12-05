import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Get the path to your package's installed share directory
    pkg_share_dir = get_package_share_directory('citysim_ros2_pkg')

    # 2. Define the path to the world file
    world_file_name = 'simple_city.world'
    world_path = os.path.join(pkg_share_dir, 'worlds', world_file_name)

    # 3. Get the path to the gazebo_ros package
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # 4. **CRITICAL STEP: APPEND ASSET PATHS**
    # Get current GAZEBO_MODEL_PATH, default to empty string if not set
    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    
    # Get current GAZEBO_RESOURCE_PATH, default to empty string if not set
    current_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')

    # Append your package's asset directories to the existing paths
    # The colon (':') is the standard separator for environment paths
    os.environ["GAZEBO_MODEL_PATH"] = current_model_path + os.pathsep + os.path.join(pkg_share_dir, 'models')
    os.environ["GAZEBO_RESOURCE_PATH"] = current_resource_path + os.pathsep + os.path.join(pkg_share_dir, 'media')
    
    # Note: os.pathsep ensures the path separator is correct (usually ':')

    # 5. Launch Gazebo 11
    gazebo_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        # Pass the world path and implicitly let the GUI start (default behavior)
        launch_arguments={'world': world_path}.items()
    )

    return LaunchDescription([
        gazebo_server_launch
    ])