#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys


class TextPublisher(Node):
    def __init__(self):
        super().__init__('text_publisher')
        
        # Create publisher for /test/command topic
        self.publisher = self.create_publisher(String, '/test/command', 10)
        
        # Flag to control the input thread
        self.running = True
        
        self.get_logger().info('Text Publisher node started')
        self.get_logger().info('Publishing to topic: /test/command')
        self.get_logger().info('Type your messages and press Enter to publish them')
        self.get_logger().info('Type "exit" or "quit" to stop the node')
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
    
    def input_loop(self):
        """Loop to get user input and publish messages"""
        try:
            while self.running and rclpy.ok():
                try:
                    # Get input from user
                    user_input = input("Enter message to publish: ")
                    
                    # Check for exit commands
                    if user_input.lower() in ['exit', 'quit']:
                        self.get_logger().info('Exit command received. Shutting down...')
                        self.running = False
                        rclpy.shutdown()
                        break
                    
                    # Create and publish message
                    msg = String()
                    msg.data = user_input
                    self.publisher.publish(msg)
                    
                    self.get_logger().info(f'Published: "{user_input}"')
                    
                except EOFError:
                    # Handle Ctrl+D
                    self.get_logger().info('EOF received. Shutting down...')
                    self.running = False
                    rclpy.shutdown()
                    break
                except KeyboardInterrupt:
                    # Handle Ctrl+C
                    self.get_logger().info('Keyboard interrupt received. Shutting down...')
                    self.running = False
                    rclpy.shutdown()
                    break
                    
        except Exception as e:
            self.get_logger().error(f'Error in input loop: {e}')
            self.running = False
            rclpy.shutdown()
    
    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        text_publisher = TextPublisher()
        
        # Spin the node
        rclpy.spin(text_publisher)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        if 'text_publisher' in locals():
            text_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()