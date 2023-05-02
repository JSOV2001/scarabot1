from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription

pkg_path = get_package_share_directory('scarabot1')

def generate_launch_description():
    rsp_file_path = os.path.join(pkg_path, "launch", "rsp.launch.py")
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rsp_file_path])
    )

    rviz_file_path = os.path.join(pkg_path, 'rviz', 'visualization.rviz')
    rviz_cmd = ExecuteProcess(
        cmd=['rviz2', '-d', str(rviz_file_path)]
    )

    jsp_gui_node = Node(
        package= "joint_state_publisher_gui",
        executable= "joint_state_publisher_gui"
    )

    return LaunchDescription(
        [
            rsp_launch,
            rviz_cmd,
            jsp_gui_node
        ]
    )
