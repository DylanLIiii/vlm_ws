import numpy as np
from rclpy.node import Node

from vita_agent.utils.path_planner_utils import (
    PathFinderController,
    generate_local_yaw_rate,
    plan_path,
)


class PathPlanner:
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        # Path planner parameters
        kp_rho = 0.5
        kp_alpha = 3.0
        rot_steps = 5
        rot_step_size = np.pi / (2 * rot_steps)
        self.dt = 0.1
        self.max_plan_steps = 10  # 18
        self.occ_dilate_kernel_size = 3

        self.global_planner = PathFinderController(kp_rho, kp_alpha)
        self.local_yaw_rate_list = generate_local_yaw_rate(rot_steps, rot_step_size)

    def plan_trajectory(self, goal_position, occupancy_map):
        # Check if straight-line planning is enabled
        if hasattr(self.node, 'use_straight_line_planning') and self.node.use_straight_line_planning:
            self.logger.info("Using straight-line path planning (obstacle avoidance disabled)")
            return self.plan_straight_line_trajectory(goal_position)
        
        # Original obstacle-aware path planning
        self.logger.debug("Using obstacle-aware path planning")
        x_s, y_s, theta_s = 0.0, 0.0, 0.0
        x_g, y_g = goal_position[0], goal_position[1]
        x_path_global, y_path_global = [], []
        _, _, _, _, _, v_traj, w_traj, stop_flag = plan_path(
            x_s, y_s, theta_s, x_g, y_g, x_path_global, y_path_global, occupancy_map,
            self.local_yaw_rate_list, self.occ_dilate_kernel_size,
            self.node.pc_range, self.node.xy_resolution, self.node.stop_distance,
            self.global_planner, self.node.max_linear_speed, self.node.max_angular_speed,
            self.dt, self.max_plan_steps
        )
        return v_traj, w_traj, stop_flag

    def plan_straight_line_trajectory(self, goal_position):
        """
        Plan a straight-line trajectory to the goal without obstacle avoidance.

        Args:
            goal_position: Target position [x, y, z, w] in robot frame

        Returns:
            v_traj: List of linear velocities
            w_traj: List of angular velocities
            stop_flag: Always False for straight-line planning
        """
        x_goal, y_goal = goal_position[0], goal_position[1]

        # Calculate distance and angle to target
        distance = np.hypot(x_goal, y_goal)
        target_angle = np.arctan2(y_goal, x_goal)

        # Check if we're close enough to stop
        if distance < self.node.stop_distance:
            return [0.0], [0.0], False

        # Generate simple trajectory using the global planner controller
        rho, v, w = self.global_planner.calc_control_command(x_goal, y_goal, 0.0)

        # Apply speed limits
        v = np.clip(v, -self.node.max_linear_speed, self.node.max_linear_speed)
        w = np.clip(w, -self.node.max_angular_speed, self.node.max_angular_speed)

        # Generate a short trajectory (similar to the original planner)
        trajectory_length = min(5, int(distance / (v * self.dt)) + 1) if v > 0 else 5
        v_traj = [v] * trajectory_length
        w_traj = [w] * trajectory_length

        self.logger.info(f"Straight-line planning: distance={distance:.2f}, v={v:.2f}, w={w:.2f}")

        return v_traj, w_traj, False