from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
import xacro
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

pkg_path = get_package_share_directory('scarabot1')
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    xacro_file_path = os.path.join(pkg_path, 'urdf', 'scarabot1.urdf.xacro')
    xacro_file = xacro.process_file(xacro_file_path)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': xacro_file.toxml(), 
                'use_sim_time': use_sim_time
            }
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time', 
                default_value='false', 
                description='Use sim time if true'
            ),
            robot_state_publisher_node
        ]
    )