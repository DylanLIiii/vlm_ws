#!/usr/bin/env python3
"""
Launch script to test odometry with REAL data only (no synthetic data)
Use this when you have actual odometry data coming from your robot
"""

import rclpy
import sys
import os
sys.path.append('/root/vlm_ws/src/vita_agent')

from vita_agent.just_test_odom import OdometryTestNode

def main():
    rclpy.init()
    
    # Create node with synthetic data disabled
    node = OdometryTestNode()
    
    # Override the parameter to disable synthetic data
    node.set_parameters([
        rclpy.parameter.Parameter('publish_test_data', rclpy.Parameter.Type.BOOL, False),
        rclpy.parameter.Parameter('topic_odom', rclpy.Parameter.Type.STRING, '/rt/odom')
    ])
    
    try:
        print("=== REAL ODOMETRY TEST ===")
        print("Listening for real odometry data on /rt/odom")
        print("Make sure your robot/simulator is publishing odometry data!")
        print("Press Ctrl+C to stop\n")
        
        # Run initial test
        node.run_comprehensive_test()
        
        # Start spinning
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
        
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
