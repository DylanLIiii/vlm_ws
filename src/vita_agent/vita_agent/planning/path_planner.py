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

    def plan_trajectory(self, goal_position, occupancy_map, uwb_target_position=None):
        # path planning
        x_s, y_s, theta_s = 0.0, 0.0, 0.0
        x_g, y_g = goal_position[0], goal_position[1]
        x_path_global, y_path_global = [], []
        _, _, _, _, _, v_traj, w_traj, stop_flag = plan_path(
            x_s, y_s, theta_s, x_g, y_g, x_path_global, y_path_global, occupancy_map,
            self.local_yaw_rate_list, self.occ_dilate_kernel_size,
            self.node.pc_range, self.node.xy_resolution, self.node.stop_distance,
            self.global_planner, self.node.max_linear_speed, self.node.max_angular_speed,
            self.dt, self.max_plan_steps, uwb_target_position
        )
        return v_traj, w_traj, stop_flag