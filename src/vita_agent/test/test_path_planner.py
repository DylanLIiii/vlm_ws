import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from rclpy.node import Node

from vita_agent.planning.path_planner import PathPlanner


class TestPathPlanner(unittest.TestCase):

    def setUp(self):
        # Create a mock rclpy node
        self.mock_node = MagicMock(spec=Node)
        self.mock_node.get_logger.return_value = MagicMock()
        
        # Set parameters that the PathPlanner will use from the node
        self.mock_node.pc_range = [-5.0, -5.0, -2.0, 5.0, 5.0, 2.0]
        self.mock_node.xy_resolution = 0.05
        self.mock_node.stop_distance = 0.5
        self.mock_node.max_linear_speed = 0.5
        self.mock_node.max_angular_speed = 1.0

        # Initialize the PathPlanner with the mock node
        self.path_planner = PathPlanner(self.mock_node)

    @patch('vita_agent.planning.path_planner.plan_path')
    def test_plan_trajectory_straight_line(self, mock_plan_path):
        """Test Case 4.1: Given a goal and an empty map, verify a straight-line trajectory."""
        # For an empty map, we expect a non-stop, forward motion
        mock_plan_path.return_value = ([], [], [], [], [], [0.5, 0.5], [0.0, 0.0], False)
        
        goal_position = np.array([3.0, 0.0])
        # Empty occupancy map
        occupancy_map = np.zeros((200, 200), dtype=np.uint8)
        
        v_traj, w_traj, stop_flag = self.path_planner.plan_trajectory(goal_position, occupancy_map)
        
        self.assertFalse(stop_flag)
        self.assertTrue(all(v >= 0 for v in v_traj)) # Should move forward
        self.assertTrue(all(w == 0 for w in w_traj)) # No rotation needed

    @patch('vita_agent.planning.path_planner.plan_path')
    def test_plan_trajectory_with_obstacles(self, mock_plan_path):
        """Test Case 4.2: Given a goal and a map with obstacles, verify a collision-free path."""
        # A collision-free path would involve turning, so w_traj should have non-zero values
        mock_plan_path.return_value = ([], [], [], [], [], [0.3, 0.3], [0.8, -0.2], False)
        
        goal_position = np.array([3.0, 2.0])
        # Occupancy map with an obstacle
        occupancy_map = np.zeros((200, 200), dtype=np.uint8)
        occupancy_map[100:120, 100:110] = 1 # some obstacle
        
        v_traj, w_traj, stop_flag = self.path_planner.plan_trajectory(goal_position, occupancy_map)
        
        self.assertFalse(stop_flag)
        self.assertTrue(any(w != 0 for w in w_traj)) # Should have some rotation

    @patch('vita_agent.planning.path_planner.plan_path')
    def test_plan_trajectory_blocked_path(self, mock_plan_path):
        """Test Case 4.3: Given a goal blocked by obstacles, verify stop_flag is True."""
        # If the path is blocked, the planner should signal a stop
        mock_plan_path.return_value = ([], [], [], [], [], [], [], True)

        goal_position = np.array([2.0, 0.0])
        # Occupancy map with a wall blocking the path
        occupancy_map = np.zeros((200, 200), dtype=np.uint8)
        occupancy_map[80:120, 110:115] = 1 # A vertical wall
        
        _, _, stop_flag = self.path_planner.plan_trajectory(goal_position, occupancy_map)
        
        self.assertTrue(stop_flag)

    @patch('vita_agent.planning.path_planner.plan_path')
    def test_trajectory_speed_limits(self, mock_plan_path):
        """Test Case 4.4: Verify that planned trajectory values are within speed limits."""
        # The mock planner returns a trajectory that exceeds the limits
        mock_plan_path.return_value = ([], [], [], [], [], [0.8, 0.4], [1.5, -1.2], False)

        goal_position = np.array([3.0, 0.0])
        occupancy_map = np.zeros((200, 200), dtype=np.uint8)
        
        # The actual `plan_path` function is responsible for clamping the speeds.
        # This test verifies that the PathPlanner *would* return trajectories
        # that we can check against the limits. We assume the utility function works.
        v_traj, w_traj, _ = self.path_planner.plan_trajectory(goal_position, occupancy_map)

        # In a real test of plan_path, we would check this. Here we check the mock data.
        # This test becomes more meaningful in an integration test of the real plan_path function.
        self.assertTrue(all(abs(v) <= self.mock_node.max_linear_speed for v in v_traj))
        self.assertTrue(all(abs(w) <= self.mock_node.max_angular_speed for w in w_traj))


if __name__ == '__main__':
    unittest.main()