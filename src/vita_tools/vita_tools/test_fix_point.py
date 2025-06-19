#!/usr/bin/env python3
"""
Test node for fixed point navigation using odometry feedback.
This node subscribes to odometry messages and publishes velocity commands 
to navigate to a fixed target point at x=3.0, y=0.0.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import time
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from scipy.spatial.transform import Rotation as R

# Import from vita_agent package
from vita_agent.processors.sensor_processor import SensorProcessor
from vita_agent.planning.path_planner import PathPlanner
from vita_agent.motion.motion_controller import MotionController


class FixPointTestNode(Node):
    """Test node for navigating to a fixed target point using odometry feedback"""
    
    def __init__(self):
        super().__init__("fix_point_test_node")
        
        # Occupancy grid map parameters
        self.declare_parameter('vlm_server_url', 'http://192.168.31.21:8000/process_spatial_data/')
        self.declare_parameter('topic_lidar', '/lidar_points')
        self.declare_parameter('topic_uwb', '/uwb/data')
        self.declare_parameter('topic_vel_cmd', '/vel_cmd')
        self.declare_parameter('topic_text_command', '/test/command')
        self.declare_parameter('topic_video', '/image_right_raw/h264_half')
        self.declare_parameter('topic_odom', '/rt/odom')
        self.declare_parameter('pc_range', [-5.0, -5.0, -0.1, 5.0, 5.0, 1.2])
        self.declare_parameter('xy_resolution', 0.1)
        self.declare_parameter('stop_distance', 1.2)
        self.declare_parameter('uwb_timeout', 30.0)
        self.declare_parameter('pc_timeout', 30.0)
        self.declare_parameter('odom_timeout', 5.0)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 0.7)

        # Control parameters
        self.declare_parameter('target_x', 5.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('position_tolerance', 0.1)
        
        # Initialize sensor processor
        self.sensor_processor = SensorProcessor(self)
        
        # Initialize path planner and motion controller
        self.path_planner = PathPlanner(self)
        self.motion_controller = MotionController(self)
        
        # Create callback group for sensor processing
        self.sensor_group = ReentrantCallbackGroup()
        
        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('topic_odom').get_parameter_value().string_value,
            self.odom_callback,
            1,
            callback_group=self.sensor_group
        )
        
        # Point cloud subscriber for occupancy mapping
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('topic_lidar').get_parameter_value().string_value,
            self.point_cloud_callback,
            1,
            callback_group=self.sensor_group
        )
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz control loop
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Target point
        self.target_x = self.get_parameter('target_x').get_parameter_value().double_value
        self.target_y = self.get_parameter('target_y').get_parameter_value().double_value
        self.target_point = np.array([self.target_x, self.target_y, 0, 1])
        
        # Control parameters
        self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self.stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        
        # Store pc_range and xy_resolution as instance variables for path planner
        self.pc_range = self.get_parameter('pc_range').get_parameter_value().double_array_value
        self.xy_resolution = self.get_parameter('xy_resolution').get_parameter_value().double_value
        
        # State tracking
        self.target_reached = False
        self.start_time = time.time()
        self.last_control_time = None
        self.latest_occupancy_map = None
        self.latest_point_cloud_msg = None
        
        self.get_logger().info("Fix Point Test Node initialized")
        self.get_logger().info(f"Target point: ({self.target_x:.2f}, {self.target_y:.2f})")
        self.get_logger().info(f"Listening on odometry topic: {self.get_parameter('topic_odom').get_parameter_value().string_value}")
        self.get_logger().info(f"Publishing velocity commands on: {self.get_parameter('topic_vel_cmd').get_parameter_value().string_value}")
        self.get_logger().info(f"Position tolerance: {self.position_tolerance:.3f}m")
        self.get_logger().info(f"Stop distance: {self.stop_distance:.3f}m")

    def odom_callback(self, odom_msg):
        """Process odometry message through SensorProcessor"""
        self.sensor_processor.process_odom(odom_msg)
        
    def point_cloud_callback(self, point_cloud_msg):
        """Process point cloud message and store for path planning"""
        self.latest_point_cloud_msg = point_cloud_msg
        # Process point cloud through sensor processor to get occupancy map
        self.latest_occupancy_map = self.sensor_processor.process_point_cloud(point_cloud_msg)
        
    def control_loop(self):
        """Main control loop using path planner and motion controller"""
        # Check if we have valid odometry data
        odom_status = self.sensor_processor.get_odom_status()
        if not odom_status['fresh'] or not odom_status['transforms_ready']:
            # Stop the robot if no fresh odometry
            self.motion_controller.stop()
            if self.last_control_time is None or (time.time() - self.last_control_time) > 5.0:
                self.get_logger().warn("No fresh odometry data - stopping robot")
                self.last_control_time = time.time()
            return
            
        # Check if we have a fresh occupancy map from point cloud
        if self.latest_occupancy_map is None:
            self.motion_controller.stop()
            if self.last_control_time is None or (time.time() - self.last_control_time) > 5.0:
                self.get_logger().warn("No occupancy map available - stopping robot")
                self.last_control_time = time.time()
            return
            
        # Get current position from sensor processor
        if self.sensor_processor.xyz is None:
            self.motion_controller.stop()
            return
            
        current_pos = self.sensor_processor.xyz[:2]  # [x, y] in initial frame
        
        # Calculate distance to target
        target_pos = self.target_point[:2]  # [x, y]
        position_error = target_pos - current_pos
        distance_to_target = np.linalg.norm(position_error)
        
        # Check if target is reached
        if distance_to_target < self.position_tolerance:
            if not self.target_reached:
                self.get_logger().info(f"Target reached! Distance: {distance_to_target:.3f}m")
                self.target_reached = True
            self.motion_controller.stop()
            return
            
        # Transform target to ego frame for path planning
        target_ego = self.sensor_processor.Tr_init2ego @ self.target_point
        
        # Use path planner to generate trajectory
        v_traj, w_traj, stop_flag = self.path_planner.plan_trajectory(
            target_ego, self.latest_occupancy_map
        )
        
        if stop_flag:
            self.get_logger().warn("Path blocked - executing go back maneuver")
            self.motion_controller.go_back()
        else:
            # Use motion controller to execute the planned trajectory
            self.motion_controller.move_to_target(target_ego, v_traj, w_traj)
        
        # Log control info periodically
        if time.time() - self.start_time > 2.0 and int(time.time() * 2) % 10 == 0:  # Every 5 seconds
            self.get_logger().info(
                f"Control: pos=({current_pos[0]:.2f}, {current_pos[1]:.2f}), "
                f"dist={distance_to_target:.3f}m, stop_flag={stop_flag}, "
                f"v_traj={v_traj[0] if len(v_traj) > 0 else 0:.2f}, "
                f"w_traj={w_traj[0] if len(w_traj) > 0 else 0:.2f}"
            )
            
    def publish_status(self):
        """Publish periodic status information"""
        runtime = time.time() - self.start_time
        odom_status = self.sensor_processor.get_odom_status()
        
        if odom_status['transforms_ready'] and self.sensor_processor.xyz is not None:
            current_pos = self.sensor_processor.xyz[:2]
            distance_to_target = np.linalg.norm(self.target_point[:2] - current_pos)
            
            status_msg = (
                f"Runtime: {runtime:.1f}s, "
                f"Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f}), "
                f"Distance to target: {distance_to_target:.3f}m, "
                f"Target reached: {self.target_reached}, "
                f"Has occupancy map: {self.latest_occupancy_map is not None}"
            )
        else:
            status_msg = f"Runtime: {runtime:.1f}s, Waiting for sensor data..."
            
        self.get_logger().info(status_msg)
        
        # Log sensor status
        if not odom_status['fresh']:
            if odom_status['time_since_last'] is not None:
                self.get_logger().warn(f"Odometry data stale: {odom_status['time_since_last']:.1f}s since last message")
            else:
                self.get_logger().warn("No odometry data received yet")


def main(args=None):
    """Main function to run the fix point test node"""
    rclpy.init(args=args)
    
    try:
        node = FixPointTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in fix point test node: {e}")
    finally:
        # Clean shutdown
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()