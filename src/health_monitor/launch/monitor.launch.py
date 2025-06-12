import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('health_monitor'),
        'config',
        'health_monitor.yaml'
    )

    return LaunchDescription([
        Node(
            package='health_monitor',
            executable='monitor_node',
            name='health_monitor_node',
            parameters=[config]
        )
    ])