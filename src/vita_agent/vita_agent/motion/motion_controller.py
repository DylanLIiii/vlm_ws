import time
import numpy as np
import rclpy.node
from geometry_msgs.msg import Twist


class MotionController:
    def __init__(self, node: rclpy.node.Node):
        self.node = node
        self.logger = node.get_logger()
        # Velocity command
        self.vel_cmd_publisher = self.node.create_publisher(
            Twist, self.node.get_parameter('topic_vel_cmd').get_parameter_value().string_value, 1
        )

        # Set a timer for publishing velocity command msg
        self.vel_cmd_timer = self.node.create_timer(0.1, self.publish_velocity)

        self.vx = 0.0  # [0.0] * self.queue_size
        self.vyaw = 0.0  # [0.0] * self.queue_size
    
    def publish_velocity(self):
        if self.node.last_uwb_time is None or self.node.last_pc_time is None:
            self.logger.warn(f"Timeout, UWB: {self.node.last_uwb_time}, Lidar: {self.node.last_pc_time}. Halting velocity publication.")
            return

        if time.time() - self.node.last_uwb_time > self.node.uwb_timeout:
            self.logger.warn("UWB timeout, stopping robot")
            self.stop()

        if time.time() - self.node.last_pc_time > self.node.pc_timeout:
            self.logger.warn("Point cloud timeout, stopping robot")
            self.stop()

        if self.vx > 0.8:
            self.vx = 0.8

        if abs(self.vyaw) > 0.75:
            self.vyaw = np.sign(self.vyaw) * 0.75

        msg = Twist()
        msg.linear.x = self.vx
        msg.angular.z = self.vyaw * 0.4

        self.vel_cmd_publisher.publish(msg)
        self.logger.info(f'Publishing velocity: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

    def go_back(self):
        self.logger.info("Collision detected, moving back.")
        self.vx = -0.6
        self.vyaw = 0.0

    def stop(self):
        """完全停止"""
        self.vx = 0.0
        self.vyaw = 0.0

    def move_to_target(self, p_xyz_vcs, v_traj, w_traj):
        distance = np.hypot(p_xyz_vcs[0], p_xyz_vcs[1])

        if distance < self.node.stop_distance:
            # self.stop()
            # return

            if self.vx > 0.3:
                self.vx = 0.5 * self.vx
                self.vyaw = 0.0
            else:
                self.vx = 0.0
                self.vyaw = 0.0

        else:
            # self.vx = v_traj[0]  # [-1]
            self.vx = 0.1 * v_traj[0] + 0.9 * self.vx


        self.vyaw = w_traj[0]  # [-1]