#!/usr/bin/env python3
"""
Launch file for the Forward ASR node.

This launch file starts the forward_asr node which subscribes to ASR commands
and forwards them to appropriate topics based on command mapping.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the Forward ASR node."""
    
    return LaunchDescription([
        Node(
            package='vita_tools',
            executable='forward_asr',
            name='forward_asr_node',
            output='screen',
            parameters=[],
            remappings=[],
        )
    ])
