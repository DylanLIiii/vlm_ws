import numpy as np
import rclpy.node
from geometry_msgs.msg import Twist


class MotionController:
    def __init__(self, node: rclpy.node.Node, callback_group=None):
        self.node = node
        self.logger = node.get_logger()
        # Velocity command
        self.vel_cmd_publisher = self.node.create_publisher(
            Twist, self.node.get_parameter('topic_vel_cmd').get_parameter_value().string_value, 1
        )

        # Set a timer for publishing velocity command msg
        self.vel_cmd_timer = self.node.create_timer(0.5, self.publish_velocity, callback_group=callback_group)

        self.vx = 0.0  # [0.0] * self.queue_size
        self.vyaw = 0.0  # [0.0] * self.queue_size
    
    def publish_velocity(self):
        """Publish velocity commands with safety limits"""
        # Check if the main node has determined it's safe to move
        if hasattr(self.node, 'check_safety_conditions'):
            if not self.node.check_safety_conditions():
                # Safety conditions not met - stop the robot
                self.stop()
                return
        
        # Apply safety limits to prevent excessive speeds
        if self.vx > self.node.max_linear_speed:
            self.vx = self.node.max_linear_speed

        if abs(self.vyaw) > self.node.max_angular_speed:
            self.vyaw = np.sign(self.vyaw) * self.node.max_angular_speed

        msg = Twist()
        msg.linear.x = self.vx
        msg.angular.z = self.vyaw * 0.4  # Apply angular velocity scaling factor

        self.vel_cmd_publisher.publish(msg)
        
        # Log velocity commands periodically (every 50 messages to avoid spam)
        if not hasattr(self, '_publish_count'):
            self._publish_count = 0
        self._publish_count += 1
        
        if self._publish_count % 10 == 1 or (abs(self.vx) > 0.1 or abs(self.vyaw) > 0.1):
            self.logger.info(f'Publishing velocity: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}')

    def go_back(self):
        self.logger.info("Collision detected, moving back.")
        self.vx = -0.6
        self.vyaw = 0.0

    def stop(self):
        """Stop the robot completely"""
        self.vx = 0.0
        self.vyaw = 0.0
        self.logger.info("Robot stopped")

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
