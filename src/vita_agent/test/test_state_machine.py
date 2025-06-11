import unittest
from unittest.mock import MagicMock, patch, AsyncMock

import numpy as np
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

from vita_agent.zero_shot_vlm_planner import TaskLogicNode

class TestStateMachine(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.node = TaskLogicNode()
        
        # Mock external dependencies
        self.node.sensor_processor = MagicMock()
        self.node.vlm_client = MagicMock()
        self.node.path_planner = MagicMock()
        self.node.motion_controller = MagicMock()
        
        # Mock publishers
        self.node.motion_controller.vel_pub = MagicMock()
        
    def tearDown(self):
        self.node.destroy_node()

    def test_idle_to_waiting_for_vlm(self):
        """Test Case 6.1: IDLE -> WAITING_FOR_VLM"""
        self.node.state = self.node.State.IDLE
        
        # Simulate receiving a text command
        test_command = "Come Here"
        msg = String(data=test_command)
        self.node.text_callback(msg)
        
        self.assertEqual(self.node.state, self.node.State.WAITING_FOR_VLM)
        self.assertEqual(self.node.text_data, test_command)

    def test_waiting_for_vlm_to_executing_action(self):
        """Test Case 6.2: WAITING_FOR_VLM -> EXECUTING_ACTION"""
        self.node.state = self.node.State.WAITING_FOR_VLM
        self.node.text_data = "Come Here"
        self.node.rgb = "some_image_data" # Simulate image availability
        
        # Mock UWB data to avoid early return
        self.node.uwb_target = np.array([1.0, 1.0, 0.0, 1.0])
        
        # Mock VLM client to return a successful response
        mock_vlm_response = [1.0, 2.0]
        self.node.vlm_client.get_action = AsyncMock(return_value=mock_vlm_response)
        
        # Simulate receiving a point cloud message to trigger the VLM call
        self.node.pc_callback(PointCloud2())
        
        self.assertEqual(self.node.state, self.node.State.EXECUTING_ACTION)
        self.assertEqual(self.node.action_target_position, mock_vlm_response)

    def test_waiting_for_vlm_to_idle_on_vlm_failure(self):
        """Test Case 6.3: WAITING_FOR_VLM -> IDLE"""
        self.node.state = self.node.State.WAITING_FOR_VLM
        self.node.text_data = "Come Here"
        self.node.rgb = "some_image_data"
        
        # Mock UWB data to avoid early return
        self.node.uwb_target = np.array([1.0, 1.0, 0.0, 1.0])
        
        # Mock VLM client to return a failure (None)
        self.node.vlm_client.get_action = AsyncMock(return_value=None)
        
        # Simulate receiving a point cloud message
        self.node.pc_callback(PointCloud2())
        
        self.assertEqual(self.node.state, self.node.State.IDLE)

    def test_executing_action_to_idle_on_task_completion(self):
        """Test Case 6.4: EXECUTING_ACTION -> IDLE"""
        self.node.state = self.node.State.EXECUTING_ACTION
        self.node.action_target_position = [0.1, 0.1]  # Very close to robot (distance < stop_distance)
        
        # Mock UWB data to avoid early return
        self.node.uwb_target = np.array([1.0, 1.0, 0.0, 1.0])  # Non-zero UWB target
        
        # Mock sensor processor transformation matrix as identity (no transformation)
        self.node.sensor_processor.Tr_init2ego = np.eye(4)
        
        # Simulate being close to the target for multiple cycles
        with patch.object(self.node.path_planner, 'plan_trajectory', return_value=([], [], False)):
            for _ in range(3):
                self.node.pc_callback(PointCloud2())
        
        self.assertEqual(self.node.state, self.node.State.IDLE)
        self.assertIsNone(self.node.action_target_position)
        self.assertIsNone(self.node.text_data)
        self.node.motion_controller.stop.assert_called_once()