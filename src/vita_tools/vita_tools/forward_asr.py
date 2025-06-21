#!/usr/bin/env python3
"""
Forward ASR Node for VITA Tools Package.

This node subscribes to ASR (Automatic Speech Recognition) commands and forwards
them to appropriate topics based on predefined mapping rules.

Input Topic:
    - asr_command (std_msgs/String): Incoming ASR commands

Output Topics:
    - /test/command (std_msgs/String): Movement commands
    - /test/follow_control (std_msgs/String): Follow control commands

Author: VITA Tools Package
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ForwardASRNode(Node):
    """
    ROS2 node that forwards ASR commands to appropriate topics based on command mapping.
    """

    def __init__(self):
        super().__init__('forward_asr_node')

        # Define command mappings
        self.command_mappings = {
            # Movement commands for /test/command topic
            'move_to_master': 'Come Here',
            'move_to_master_front': 'Come to my front',
            'move_to_master_behind': 'Come to my behind/back',
            'move_to_master_left': 'Come to my left',
            'move_to_master_right': 'Come to my right',
        }

        self.follow_commands = {
            # Follow control commands for /test/follow_control topic
            'follow_start': 'follow_start',
            'follow_stop': 'follow_stop',
        }

        # Create subscriber for ASR commands
        self.asr_subscriber = self.create_subscription(
            String,
            'asr_command',
            self.asr_callback,
            10
        )

        # Create publishers for output topics
        self.command_publisher = self.create_publisher(
            String,
            '/test/command',
            10
        )

        self.follow_control_publisher = self.create_publisher(
            String,
            '/test/follow_control',
            10
        )

        self.get_logger().info('Forward ASR Node initialized')
        self.get_logger().info('Subscribed to: asr_command')
        self.get_logger().info('Publishing to: /test/command, /test/follow_control')
        self.get_logger().info(f'Command mappings loaded: {len(self.command_mappings)} movement commands, {len(self.follow_commands)} follow commands')

    def asr_callback(self, msg):
        """
        Callback function for processing incoming ASR commands.

        Args:
            msg (std_msgs/String): The incoming ASR command message
        """
        command = msg.data.strip()

        self.get_logger().info(f'Received ASR command: "{command}"')

        # Check if command is a movement command
        if command in self.command_mappings:
            mapped_command = self.command_mappings[command]
            output_msg = String()
            output_msg.data = mapped_command

            self.command_publisher.publish(output_msg)
            self.get_logger().info(f'Published to /test/command: "{mapped_command}"')

        # Check if command is a follow control command
        elif command in self.follow_commands:
            mapped_command = self.follow_commands[command]
            output_msg = String()
            output_msg.data = mapped_command

            self.follow_control_publisher.publish(output_msg)
            self.get_logger().info(f'Published to /test/follow_control: "{mapped_command}"')

        else:
            # Log unrecognized commands for debugging
            self.get_logger().warn(f'Unrecognized ASR command: "{command}"')
            self.get_logger().info(f'Available movement commands: {list(self.command_mappings.keys())}')
            self.get_logger().info(f'Available follow commands: {list(self.follow_commands.keys())}')


def main(args=None):
    """
    Main function to run the Forward ASR node.

    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)

    try:
        node = ForwardASRNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in Forward ASR node: {e}")
    finally:
        # Clean shutdown
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()