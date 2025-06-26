#!/usr/bin/env python3
"""
Test script for the multi_control feature in ActionExecutor.

This script creates a minimal test environment to verify that:
1. When multi_control is disabled, following commands are sent once
2. When multi_control is enabled, following commands are sent 10 times with 0.1s intervals
3. Runtime parameter updates work correctly
"""

import time
import threading
from unittest.mock import Mock, MagicMock
import sys
import os

# Add the control_flow package to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'control_flow'))

from control_flow.action_executor import ActionExecutor
from std_msgs.msg import String


class MockNode:
    """Mock ROS2 node for testing"""
    
    def __init__(self):
        self.logger = Mock()
        self.clock = Mock()
        self.clock.now.return_value.to_msg.return_value = Mock()
        
    def create_publisher(self, msg_type, topic, qos):
        """Create a mock publisher that tracks published messages"""
        publisher = Mock()
        publisher.published_messages = []
        
        def publish_side_effect(msg):
            publisher.published_messages.append({
                'timestamp': time.time(),
                'data': msg.data if hasattr(msg, 'data') else str(msg)
            })
            
        publisher.publish.side_effect = publish_side_effect
        return publisher
        
    def create_subscription(self, msg_type, topic, callback, qos):
        return Mock()
        
    def get_logger(self):
        return self.logger


def test_single_command_mode():
    """Test that single command mode works correctly (multi_control disabled)"""
    print("\n=== Testing Single Command Mode (multi_control=False) ===")
    
    # Create mock node and action executor with multi_control disabled
    mock_node = MockNode()
    executor = ActionExecutor(mock_node, multi_control_enabled=False)
    
    # Test publishing a following command
    result = executor._publish_following_command('follow_start')
    
    # Verify results
    assert result == True, "Command should be published successfully"
    
    # Check that exactly one message was published
    published_messages = executor.following_publisher.published_messages
    assert len(published_messages) == 1, f"Expected 1 message, got {len(published_messages)}"
    
    # Verify the message content
    assert published_messages[0]['data'] == 'start following', f"Expected 'start following', got '{published_messages[0]['data']}'"
    
    print("✓ Single command mode test passed")
    return True


def test_multi_command_mode():
    """Test that multi command mode works correctly (multi_control enabled)"""
    print("\n=== Testing Multi Command Mode (multi_control=True) ===")
    
    # Create mock node and action executor with multi_control enabled
    mock_node = MockNode()
    executor = ActionExecutor(mock_node, multi_control_enabled=True)
    
    # Test publishing a following command
    result = executor._publish_following_command('follow_start')
    
    # Verify initial result
    assert result == True, "Command should be published successfully"
    
    # Wait for all timers to complete (1.0 seconds + small buffer)
    print("Waiting for all 10 commands to be published...")
    time.sleep(1.2)
    
    # Check that exactly 10 messages were published
    published_messages = executor.following_publisher.published_messages
    assert len(published_messages) == 10, f"Expected 10 messages, got {len(published_messages)}"
    
    # Verify all messages have the correct content
    for i, msg in enumerate(published_messages):
        assert msg['data'] == 'start following', f"Message {i+1}: Expected 'start following', got '{msg['data']}'"
    
    # Verify timing intervals (approximately 0.1s apart)
    for i in range(1, len(published_messages)):
        time_diff = published_messages[i]['timestamp'] - published_messages[i-1]['timestamp']
        # Allow some tolerance for timing (0.08s to 0.12s)
        assert 0.08 <= time_diff <= 0.12, f"Message {i+1}: Expected ~0.1s interval, got {time_diff:.3f}s"
    
    print("✓ Multi command mode test passed")
    return True


def test_runtime_parameter_update():
    """Test that runtime parameter updates work correctly"""
    print("\n=== Testing Runtime Parameter Updates ===")
    
    # Create mock node and action executor starting with multi_control disabled
    mock_node = MockNode()
    executor = ActionExecutor(mock_node, multi_control_enabled=False)
    
    # Verify initial state
    assert executor.get_multi_control_enabled() == False, "Initial state should be disabled"
    
    # Test enabling multi_control at runtime
    executor.set_multi_control_enabled(True)
    assert executor.get_multi_control_enabled() == True, "Should be enabled after update"
    
    # Test disabling multi_control at runtime
    executor.set_multi_control_enabled(False)
    assert executor.get_multi_control_enabled() == False, "Should be disabled after update"
    
    # Test invalid parameter type
    executor.set_multi_control_enabled("invalid")
    assert executor.get_multi_control_enabled() == False, "Should remain disabled with invalid input"
    
    print("✓ Runtime parameter update test passed")
    return True


def test_timer_cancellation():
    """Test that active timers are cancelled when multi_control is disabled"""
    print("\n=== Testing Timer Cancellation ===")
    
    # Create mock node and action executor with multi_control enabled
    mock_node = MockNode()
    executor = ActionExecutor(mock_node, multi_control_enabled=True)
    
    # Start a multi-control command
    executor._publish_following_command('follow_start')
    
    # Verify timers are active
    assert len(executor.active_multi_control_timers) == 9, "Should have 9 active timers"
    
    # Disable multi_control (should cancel timers)
    executor.set_multi_control_enabled(False)
    
    # Verify timers are cancelled
    assert len(executor.active_multi_control_timers) == 0, "All timers should be cancelled"
    
    print("✓ Timer cancellation test passed")
    return True


def main():
    """Run all tests"""
    print("Starting multi_control feature tests...")
    
    tests = [
        test_single_command_mode,
        test_multi_command_mode,
        test_runtime_parameter_update,
        test_timer_cancellation
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            if test():
                passed += 1
            else:
                failed += 1
                print(f"✗ {test.__name__} failed")
        except Exception as e:
            failed += 1
            print(f"✗ {test.__name__} failed with exception: {e}")
    
    print(f"\n=== Test Results ===")
    print(f"Passed: {passed}")
    print(f"Failed: {failed}")
    print(f"Total: {passed + failed}")
    
    if failed == 0:
        print("🎉 All tests passed!")
        return 0
    else:
        print("❌ Some tests failed!")
        return 1


if __name__ == "__main__":
    exit(main())
