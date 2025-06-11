from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uwb_location',
            executable='uwb_location_node',
            name='uwb_location_node',
            output='screen'
        )
    ]) 