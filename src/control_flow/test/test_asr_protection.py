#!/usr/bin/env python3
"""
Test script for ASR Protection feature in TaskLogicNode.

This script tests the ASR command buffering and deduplication functionality.
"""

import unittest
import time
import threading
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the control_flow package to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Mock ROS2 dependencies
class MockNode:
    def __init__(self, name):
        self.name = name
        self.logger = Mock()
        self.parameters = {}
        
    def declare_parameter(self, name, default_value):
        self.parameters[name] = default_value
        
    def get_parameter(self, name):
        mock_param = Mock()
        mock_param.get_parameter_value.return_value.bool_value = self.parameters.get(name, False)
        mock_param.get_parameter_value.return_value.double_value = self.parameters.get(name, 0.0)
        mock_param.get_parameter_value.return_value.string_value = self.parameters.get(name, "")
        return mock_param
        
    def get_logger(self):
        return self.logger
        
    def create_subscription(self, *args, **kwargs):
        return Mock()
        
    def create_timer(self, *args, **kwargs):
        return Mock()
        
    def get_clock(self):
        mock_clock = Mock()
        mock_clock.now.return_value.to_msg.return_value = Mock()
        return mock_clock

class MockString:
    def __init__(self, data=""):
        self.data = data

# Mock all ROS2 imports
sys.modules['rclpy'] = Mock()
sys.modules['rclpy.node'] = Mock()
sys.modules['rclpy.callback_groups'] = Mock()
sys.modules['rclpy.executors'] = Mock()
sys.modules['std_msgs.msg'] = Mock()
sys.modules['uwb_location.msg'] = Mock()
sys.modules['nav_msgs.msg'] = Mock()
sys.modules['sensor_msgs.msg'] = Mock()
sys.modules['vita_agent.processors.sensor_processor'] = Mock()
sys.modules['vita_agent.processors.asr_processor'] = Mock()

# Mock the imports
with patch.multiple('sys.modules', 
                   rclpy=Mock(),
                   **{'rclpy.node': Mock(), 
                      'rclpy.callback_groups': Mock(),
                      'rclpy.executors': Mock(),
                      'std_msgs.msg': Mock(),
                      'uwb_location.msg': Mock(),
                      'nav_msgs.msg': Mock(),
                      'sensor_msgs.msg': Mock(),
                      'vita_agent.processors.sensor_processor': Mock(),
                      'vita_agent.processors.asr_processor': Mock()}):
    
    # Import the module under test
    from control_flow.control_node import TaskLogicNode


class TestASRProtection(unittest.TestCase):
    """Test cases for ASR Protection feature"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock all the dependencies
        with patch('control_flow.control_node.SensorProcessor'), \
             patch('control_flow.control_node.ASRProcessor') as mock_asr, \
             patch('control_flow.control_node.ActionExecutor') as mock_executor:
            
            # Create a mock node that inherits from our MockNode
            with patch('control_flow.control_node.Node', MockNode):
                self.node = TaskLogicNode()
                
            # Set up ASR processor mock
            self.node.asr_processor = mock_asr.return_value
            self.node.asr_processor.map_command.return_value = "mapped_command"
            
            # Set up action executor mock
            self.node.action_executor = mock_executor.return_value
            self.node.action_executor.execute_action.return_value = True
            
            # Enable ASR protection for testing
            self.node.asr_protection_enabled = True
            
    def test_asr_protection_disabled_immediate_processing(self):
        """Test that commands are processed immediately when ASR protection is disabled"""
        self.node.asr_protection_enabled = False
        
        # Create mock ASR message
        asr_msg = MockString("test_command")
        
        # Call the callback
        self.node.asr_callback(asr_msg)
        
        # Verify immediate processing
        self.node.asr_processor.map_command.assert_called_once_with("test_command")
        self.node.action_executor.execute_action.assert_called_once_with("mapped_command")
        
    def test_asr_protection_enabled_buffering(self):
        """Test that commands are buffered when ASR protection is enabled"""
        # Create mock ASR messages
        asr_msg1 = MockString("command1")
        asr_msg2 = MockString("command2")
        asr_msg3 = MockString("command1")  # Duplicate
        
        # Call the callbacks
        self.node.asr_callback(asr_msg1)
        self.node.asr_callback(asr_msg2)
        self.node.asr_callback(asr_msg3)  # Should be deduplicated
        
        # Check buffer contents
        self.assertEqual(len(self.node.asr_command_buffer), 2)
        self.assertIn("command1", self.node.asr_command_buffer)
        self.assertIn("command2", self.node.asr_command_buffer)
        
        # Commands should not be processed immediately
        self.node.asr_processor.map_command.assert_not_called()
        self.node.action_executor.execute_action.assert_not_called()
        
    def test_asr_protection_buffer_execution(self):
        """Test that buffered commands are executed after the timer expires"""
        # Create mock ASR messages
        asr_msg1 = MockString("command1")
        asr_msg2 = MockString("command2")
        
        # Call the callbacks
        self.node.asr_callback(asr_msg1)
        self.node.asr_callback(asr_msg2)
        
        # Manually trigger buffer execution
        self.node._execute_buffered_asr_commands()
        
        # Verify commands were processed
        self.assertEqual(self.node.asr_processor.map_command.call_count, 2)
        self.assertEqual(self.node.action_executor.execute_action.call_count, 2)
        
        # Verify buffer is cleared
        self.assertEqual(len(self.node.asr_command_buffer), 0)
        
    def test_asr_protection_status_methods(self):
        """Test ASR protection status and control methods"""
        # Test initial state
        self.assertTrue(self.node.is_asr_protection_enabled())
        
        # Test disabling
        self.node.set_asr_protection_enabled(False)
        self.assertFalse(self.node.is_asr_protection_enabled())
        
        # Test enabling
        self.node.set_asr_protection_enabled(True)
        self.assertTrue(self.node.is_asr_protection_enabled())
        
        # Test status method
        status = self.node.get_asr_protection_status()
        self.assertIn('asr_protection_enabled', status)
        self.assertIn('buffering_window', status)
        self.assertIn('inactivity_threshold', status)
        self.assertIn('buffer_active', status)
        self.assertIn('buffered_commands_count', status)


if __name__ == '__main__':
    unittest.main()
