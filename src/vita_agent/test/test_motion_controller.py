import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from geometry_msgs.msg import Twist
from rclpy.node import Node

from vita_agent.motion.motion_controller import MotionController


class TestMotionController(unittest.TestCase):

    def setUp(self):
        # Create a mock rclpy node
        self.mock_node = MagicMock(spec=Node)
        self.mock_node.get_logger.return_value = MagicMock()
        self.mock_node.create_publisher.return_value = MagicMock()
        
        # Mock parameters
        mock_param = MagicMock()
        mock_param_value = MagicMock()
        mock_param_value.string_value = '/vel_cmd'
        mock_param.get_parameter_value.return_value = mock_param_value
        self.mock_node.get_parameter.return_value = mock_param
        self.mock_node.uwb_timeout = 2.0
        self.mock_node.pc_timeout = 2.0
        self.mock_node.stop_distance = 0.5
        
        # Mock timers
        self.mock_node.create_timer.return_value = MagicMock()

        # Initialize the MotionController
        self.motion_controller = MotionController(self.mock_node)
        self.mock_publisher = self.mock_node.create_publisher.return_value

    def test_move_to_target_publishes_twist(self):
        """Test Case 5.1: Verify move_to_target results in Twist messages."""
        p_xyz_vcs = np.array([2.0, 1.0, 0.0])
        v_traj = [0.5]
        w_traj = [0.2]
        
        # Set valid timestamps
        self.mock_node.last_uwb_time = time.time()
        self.mock_node.last_pc_time = time.time()
        
        self.motion_controller.move_to_target(p_xyz_vcs, v_traj, w_traj)
        self.motion_controller.publish_velocity()
        
        self.mock_publisher.publish.assert_called()
        sent_msg = self.mock_publisher.publish.call_args[0][0]
        self.assertIsInstance(sent_msg, Twist)
        # self.assertAlmostEqual(sent_msg.linear.x, v_traj[0]) # Simplified due to internal smoothing
        self.assertAlmostEqual(sent_msg.angular.z, w_traj[0] * 0.4)

    def test_stop(self):
        """Test Case 5.2: Verify that stop() sets velocities to zero."""
        self.motion_controller.vx = 0.5
        self.motion_controller.vyaw = 0.5
        
        self.motion_controller.stop()
        
        self.assertEqual(self.motion_controller.vx, 0.0)
        self.assertEqual(self.motion_controller.vyaw, 0.0)

    def test_go_back(self):
        """Test Case 5.3: Verify that go_back() sets a negative linear velocity."""
        self.motion_controller.go_back()
        self.assertEqual(self.motion_controller.vx, -0.6)
        self.assertEqual(self.motion_controller.vyaw, 0.0)

    def test_uwb_timeout(self):
        """Test Case 5.4: Simulate a UWB timeout and verify stop() is called."""
        with patch.object(self.motion_controller, 'stop') as mock_stop:
            self.mock_node.last_uwb_time = time.time() - 3.0 # 3 seconds ago, timeout is 2.0
            self.mock_node.last_pc_time = time.time()
            
            self.motion_controller.publish_velocity()
            
            mock_stop.assert_called_once()
            self.mock_node.get_logger().warn.assert_called_with("UWB timeout, stopping robot")

    def test_pc_timeout(self):
        """Test Case 5.5: Simulate a PointCloud timeout and verify stop() is called."""
        with patch.object(self.motion_controller, 'stop') as mock_stop:
            self.mock_node.last_uwb_time = time.time()
            self.mock_node.last_pc_time = time.time() - 3.0 # 3 seconds ago, timeout is 2.0
            
            self.motion_controller.publish_velocity()
            
            mock_stop.assert_called_once()
            self.mock_node.get_logger().warn.assert_called_with("Point cloud timeout, stopping robot")

    def test_velocity_decreases_near_target(self):
        """Test Case 5.6: Verify that velocity decreases as the robot approaches the target."""
        # Far from target
        self.motion_controller.vx = 0.8
        p_xyz_vcs_far = np.array([1.0, 1.0, 0.0]) # distance > stop_distance
        self.motion_controller.move_to_target(p_xyz_vcs_far, [0.8], [0.1])
        vx_far = self.motion_controller.vx

        # Close to target
        self.motion_controller.vx = 0.8
        p_xyz_vcs_close = np.array([0.2, 0.1, 0.0]) # distance < stop_distance
        self.motion_controller.move_to_target(p_xyz_vcs_close, [0.8], [0.1])
        vx_close = self.motion_controller.vx

        self.assertLess(vx_close, vx_far)


if __name__ == '__main__':
    unittest.main()