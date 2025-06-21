#!/usr/bin/env python3
"""
Unit tests for the Forward ASR node.

This test module verifies the functionality of the ForwardASRNode class,
including command mapping and message publishing.
"""

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading

from vita_tools.forward_asr import ForwardASRNode


class TestForwardASRNode(unittest.TestCase):
    """Test cases for the Forward ASR node."""
    
    @classmethod
    def setUpClass(cls):
        """Set up the test environment."""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Clean up the test environment."""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up each test case."""
        self.node = ForwardASRNode()
        self.received_command_msgs = []
        self.received_follow_msgs = []
        
        # Create test subscribers to capture published messages
        self.command_subscriber = self.node.create_subscription(
            String,
            '/test/command',
            self.command_callback,
            10
        )
        
        self.follow_subscriber = self.node.create_subscription(
            String,
            '/test/follow_control',
            self.follow_callback,
            10
        )
        
        # Create publisher to send test ASR commands
        self.asr_publisher = self.node.create_publisher(
            String,
            'asr_command',
            10
        )
        
        # Give some time for publishers/subscribers to connect
        time.sleep(0.1)
    
    def tearDown(self):
        """Clean up after each test case."""
        self.node.destroy_node()
    
    def command_callback(self, msg):
        """Callback to capture messages on /test/command topic."""
        self.received_command_msgs.append(msg.data)
    
    def follow_callback(self, msg):
        """Callback to capture messages on /test/follow_control topic."""
        self.received_follow_msgs.append(msg.data)
    
    def publish_asr_command(self, command):
        """Helper method to publish ASR command and wait for processing."""
        msg = String()
        msg.data = command
        self.asr_publisher.publish(msg)
        
        # Spin briefly to allow message processing
        rclpy.spin_once(self.node, timeout_sec=0.1)
        time.sleep(0.05)  # Small delay to ensure message is processed
    
    def test_movement_commands(self):
        """Test movement command mappings."""
        test_cases = [
            ('move_to_master', 'Come Here'),
            ('move_to_master_front', 'Come to my front'),
            ('move_to_master_behind', 'Come to my behind/back'),
            ('move_to_master_left', 'Come to my left'),
            ('move_to_master_right', 'Come to my right'),
        ]
        
        for input_cmd, expected_output in test_cases:
            with self.subTest(input_cmd=input_cmd):
                self.received_command_msgs.clear()
                self.publish_asr_command(input_cmd)
                
                self.assertEqual(len(self.received_command_msgs), 1)
                self.assertEqual(self.received_command_msgs[0], expected_output)
    
    def test_follow_commands(self):
        """Test follow control command mappings."""
        test_cases = [
            ('follow_start', 'follow_start'),
            ('follow_stop', 'follow_stop'),
        ]
        
        for input_cmd, expected_output in test_cases:
            with self.subTest(input_cmd=input_cmd):
                self.received_follow_msgs.clear()
                self.publish_asr_command(input_cmd)
                
                self.assertEqual(len(self.received_follow_msgs), 1)
                self.assertEqual(self.received_follow_msgs[0], expected_output)
    
    def test_unrecognized_command(self):
        """Test handling of unrecognized commands."""
        self.received_command_msgs.clear()
        self.received_follow_msgs.clear()
        
        self.publish_asr_command('unknown_command')
        
        # Should not publish to either topic
        self.assertEqual(len(self.received_command_msgs), 0)
        self.assertEqual(len(self.received_follow_msgs), 0)
    
    def test_command_with_whitespace(self):
        """Test handling of commands with leading/trailing whitespace."""
        self.received_command_msgs.clear()
        
        self.publish_asr_command('  move_to_master  ')
        
        self.assertEqual(len(self.received_command_msgs), 1)
        self.assertEqual(self.received_command_msgs[0], 'Come Here')


if __name__ == '__main__':
    unittest.main()
