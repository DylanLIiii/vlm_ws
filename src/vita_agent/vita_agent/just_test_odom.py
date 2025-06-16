#!/usr/bin/env python3
"""
Test file for odometry handling in VITA Agent.
This test verifies proper odometry processing, timeout handling, and transform calculations.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from std_msgs.msg import Header

from vita_agent.processors.sensor_processor import SensorProcessor


class OdometryTestNode(Node):
    """Test node for odometry handling functionality"""
    
    def __init__(self):
        super().__init__("odometry_test_node")
        
        # Declare parameters similar to the main planner
        self.declare_parameter('pc_range', [-5.0, -5.0, -0.1, 5.0, 5.0, 1.2])
        self.declare_parameter('xy_resolution', 0.1)
        self.declare_parameter('odom_timeout', 5.0)
        self.declare_parameter('topic_odom', '/rt/odom')
        self.declare_parameter('publish_test_data', False)  # Enable/disable synthetic data
        
        # Initialize sensor processor
        self.sensor_processor = SensorProcessor(self)
        
        # Create callback group for sensor processing
        self.sensor_group = ReentrantCallbackGroup()
        
        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('topic_odom').get_parameter_value().string_value,
            self.odom_callback,
            10,
            callback_group=self.sensor_group
        )
        
        # Timer for periodic testing and monitoring
        self.test_timer = self.create_timer(2.0, self.periodic_test)
        
        # Test publisher for synthetic odometry data (optional)
        self.publish_test_data = self.get_parameter('publish_test_data').get_parameter_value().bool_value
        
        if self.publish_test_data:
            self.test_odom_pub = self.create_publisher(
                Odometry,
                '/test/odom',
                10
            )
            
            # Timer for publishing test data
            self.test_pub_timer = self.create_timer(0.1, self.publish_test_odom)
            self.get_logger().info("Publishing synthetic test data on: /test/odom")
        else:
            self.get_logger().info("Synthetic data publishing disabled - listening only")
        
        # Test state
        self.test_count = 0
        self.start_time = time.time()
        
        self.get_logger().info("Odometry test node initialized")
        self.get_logger().info(f"Listening on topic: {self.get_parameter('topic_odom').get_parameter_value().string_value}")
        if self.publish_test_data:
            self.get_logger().info(f"Publishing test data on: /test/odom")
        self.get_logger().info(f"Odometry timeout set to: {self.get_parameter('odom_timeout').get_parameter_value().double_value}s")

    def odom_callback(self, odom_msg):
        """Main odometry callback - processes odometry through SensorProcessor"""
        self.sensor_processor.process_odom(odom_msg)
        
    def publish_test_odom(self):
        """Publish synthetic odometry data for testing"""
        # Create test odometry message
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Generate circular motion for testing
        t = (time.time() - self.start_time) * 0.5  # Slow motion
        radius = 2.0
        
        # Position
        odom_msg.pose.pose.position.x = radius * np.cos(t)
        odom_msg.pose.pose.position.y = radius * np.sin(t)
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (facing direction of motion)
        yaw = t + np.pi/2  # Tangent to circle
        quat = self.euler_to_quaternion(0, 0, yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Velocity
        odom_msg.twist.twist.linear.x = -radius * 0.5 * np.sin(t)
        odom_msg.twist.twist.linear.y = radius * 0.5 * np.cos(t)
        odom_msg.twist.twist.angular.z = 0.5
        
        # Publish test data
        self.test_odom_pub.publish(odom_msg)
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]
        
    def periodic_test(self):
        """Periodic test function to check odometry processing status"""
        self.test_count += 1
        
        # Get odometry status from sensor processor
        odom_status = self.sensor_processor.get_odom_status()
        
        self.get_logger().info("=== ODOMETRY TEST STATUS ===")
        self.get_logger().info(f"Test cycle: {self.test_count}")
        self.get_logger().info(f"Odometry messages received: {odom_status['message_count']}")
        self.get_logger().info(f"Data fresh: {odom_status['fresh']}")
        self.get_logger().info(f"Transforms ready: {odom_status['transforms_ready']}")
        
        if odom_status['time_since_last'] is not None:
            self.get_logger().info(f"Time since last odom: {odom_status['time_since_last']:.2f}s")
        else:
            self.get_logger().warn("No odometry data received yet!")
            
        # Test transform matrices if available
        if self.sensor_processor.Tr_init2ego is not None:
            self.get_logger().info("Transform matrices available:")
            self.get_logger().info(f"Current position (init frame): {self.sensor_processor.xyz[:3] if self.sensor_processor.xyz is not None else 'None'}")
            
            # Test transform properties
            self.test_transform_properties()
        else:
            self.get_logger().warn("Transform matrices not yet initialized")
            
        # Test timeout handling
        self.test_timeout_handling()
        
        self.get_logger().info("=============================\n")
        
    def test_transform_properties(self):
        """Test mathematical properties of transformation matrices"""
        try:
            # Test that Tr_init2ego and Tr_ego2init are inverses
            if self.sensor_processor.Tr_init2ego is not None and self.sensor_processor.Tr_ego2init is not None:
                product = self.sensor_processor.Tr_init2ego @ self.sensor_processor.Tr_ego2init
                identity_error = np.linalg.norm(product - np.eye(4))
                
                if identity_error < 1e-10:
                    self.get_logger().info("✓ Transform matrices are proper inverses")
                else:
                    self.get_logger().warn(f"⚠ Transform matrices not perfect inverses (error: {identity_error:.2e})")
                    
            # Test that transformation matrices are orthogonal
            if self.sensor_processor.Tr_ego2init is not None:
                rotation_part = self.sensor_processor.Tr_ego2init[:3, :3]
                det = np.linalg.det(rotation_part)
                orthogonal_error = np.linalg.norm(rotation_part @ rotation_part.T - np.eye(3))
                
                if abs(det - 1.0) < 1e-10 and orthogonal_error < 1e-10:
                    self.get_logger().info("✓ Rotation matrices are orthogonal")
                else:
                    self.get_logger().warn(f"⚠ Rotation matrix issues (det: {det:.6f}, orthogonal_error: {orthogonal_error:.2e})")
                    
        except Exception as e:
            self.get_logger().error(f"Error in transform property test: {e}")
            
    def test_timeout_handling(self):
        """Test timeout handling functionality"""
        is_fresh = self.sensor_processor.is_odom_data_fresh()
        odom_status = self.sensor_processor.get_odom_status()
        
        if odom_status['time_since_last'] is not None:
            timeout_threshold = self.get_parameter('odom_timeout').get_parameter_value().double_value
            
            if odom_status['time_since_last'] > timeout_threshold:
                self.get_logger().warn(f"⚠ Odometry timeout detected! ({odom_status['time_since_last']:.2f}s > {timeout_threshold}s)")
                if is_fresh:
                    self.get_logger().error("✗ Timeout detection inconsistency!")
                else:
                    self.get_logger().info("✓ Timeout correctly detected")
            else:
                if is_fresh:
                    self.get_logger().info("✓ Odometry data fresh and detected as fresh")
                else:
                    self.get_logger().error("✗ Data should be fresh but detected as stale!")
                    
    def run_comprehensive_test(self):
        """Run comprehensive tests that simulate main planner usage"""
        self.get_logger().info("Starting comprehensive odometry test...")
        
        # Test 1: Safety condition check (simulating main planner behavior)
        safety_ok = self.check_safety_conditions()
        self.get_logger().info(f"Safety conditions check: {'PASS' if safety_ok else 'FAIL'}")
        
        # Test 2: Transform readiness check (simulating main planner behavior)
        transforms_ready = self.sensor_processor.Tr_init2ego is not None
        self.get_logger().info(f"Transforms ready check: {'PASS' if transforms_ready else 'FAIL'}")
        
        # Test 3: Position tracking simulation
        if transforms_ready:
            self.test_position_tracking()
        
    def check_safety_conditions(self):
        """Simulate safety condition checking from main planner"""
        odom_status = self.sensor_processor.get_odom_status()
        
        if not self.sensor_processor.is_odom_data_fresh():
            if odom_status['time_since_last'] is None:
                self.get_logger().warn("Safety check: No odometry data received")
            else:
                self.get_logger().warn(f"Safety check: Odometry timeout ({odom_status['time_since_last']:.1f}s)")
            return False
            
        return True
        
    def test_position_tracking(self):
        """Test position tracking functionality"""
        if self.sensor_processor.xyz is not None:
            current_pos = self.sensor_processor.xyz[:3]
            self.get_logger().info(f"Current position in init frame: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
            
            # Simulate target tracking like in main planner
            test_target = np.array([1.0, 1.0, 0.0, 1.0])  # Target in init frame
            target_ego = self.sensor_processor.Tr_init2ego @ test_target
            distance = np.hypot(target_ego[0], target_ego[1])
            
            self.get_logger().info(f"Test target in ego frame: [{target_ego[0]:.3f}, {target_ego[1]:.3f}]")
            self.get_logger().info(f"Distance to test target: {distance:.3f}m")
        else:
            self.get_logger().warn("Position tracking test: No position data available")


def main(args=None):
    """Main function to run the odometry test"""
    rclpy.init(args=args)
    
    try:
        node = OdometryTestNode()
        
        # Run initial comprehensive test
        node.get_logger().info("Running initial comprehensive test...")
        node.run_comprehensive_test()
        
        # Start spinning
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("Starting odometry test... Press Ctrl+C to stop")
        executor.spin()
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed with error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()