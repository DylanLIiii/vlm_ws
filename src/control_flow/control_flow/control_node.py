import time
import json
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from uwb_location.msg import UWB
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Joy

from vita_agent.processors.sensor_processor import SensorProcessor
from vita_agent.processors.asr_processor import ASRProcessor
from .action_executor import ActionExecutor




# Manages the overall task logic using a state machine.
# It integrates sensor processing and health monitoring.
#
# State Machine:
# - IDLE: The default state, waiting for a command.
class TaskLogicNode(Node):
    class State(Enum):
        IDLE = auto()

    def __init__(self):
        super().__init__("task_logic_node")
        # Health monitoring parameters
        self.declare_parameter('topic_lidar', '/lidar_points')
        self.declare_parameter('topic_uwb', '/uwb/data')
        self.declare_parameter('topic_text_command', '/test/command')
        self.declare_parameter('topic_asr_command', '/asr_command')
        self.declare_parameter('topic_odom', '/rt/odom')
        self.declare_parameter('uwb_timeout', 30.0)
        self.declare_parameter('pc_timeout', 30.0)
        self.declare_parameter('odom_timeout', 5.0)
        # Parameters required by SensorProcessor
        self.declare_parameter('pc_range', [-5.0, -5.0, -0.1, 5.0, 5.0, 1.2])
        self.declare_parameter('xy_resolution', 0.1)
        # Task completion parameters
        self.declare_parameter('movement_distance_threshold', 1.5)

        self.uwb_timeout = self.get_parameter('uwb_timeout').get_parameter_value().double_value
        self.pc_timeout = self.get_parameter('pc_timeout').get_parameter_value().double_value
        self.odom_timeout = self.get_parameter('odom_timeout').get_parameter_value().double_value

        self.sensor_processor = SensorProcessor(self)
        self.asr_processor = ASRProcessor(self)

        # Get distance threshold parameter and pass to ActionExecutor
        distance_threshold = self.get_parameter('movement_distance_threshold').get_parameter_value().double_value
        self.action_executor = ActionExecutor(self)
        self.action_executor.set_distance_threshold(distance_threshold)

        self.odom_callback_group = ReentrantCallbackGroup()

        self.point_cloud_sub = self.create_subscription(
            PointCloud2, self.get_parameter('topic_lidar').get_parameter_value().string_value, self.pc_callback, 1, callback_group=self.odom_callback_group
        )
        self.get_logger().info("Point cloud subscriber node started")

        # UWB
        self.uwb_sub = self.create_subscription(
             UWB, self.get_parameter('topic_uwb').get_parameter_value().string_value, self.uwb_callback, 1, callback_group=self.odom_callback_group
        )
        self.get_logger().info("UWB subscriber node started")

        self.last_pc_time = None

        # Text command subscriber for basic task logic
        self.text_sub = self.create_subscription(
            String, self.get_parameter('topic_text_command').get_parameter_value().string_value, self.text_callback, 1, callback_group=self.odom_callback_group
        )
        self.get_logger().info("Text command subscriber node started")

        # ASR command subscriber
        self.asr_sub = self.create_subscription(
            String, self.get_parameter('topic_asr_command').get_parameter_value().string_value, self.asr_callback, 1, callback_group=self.odom_callback_group
        )
        self.get_logger().info("ASR command subscriber node started")

        self.state = self.State.IDLE
        self.text_data = None
        self.latest_odom_msg = None

        # Odom subscriber - store latest odom data without immediate processing
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('topic_odom').get_parameter_value().string_value, self.odom_callback, 1, callback_group=self.odom_callback_group
        )
        self.get_logger().info("Odom subscriber node started")
        
        # Add timer for monitoring sensor health including odometry
        self.sensor_health_timer = self.create_timer(
            10.0, self.check_sensor_health, #callback_group=self.timer_group
        )


    def destroy_node(self):
        """Cleanup method"""
        super().destroy_node()

    def uwb_callback(self, uwb_msg):
        """Store UWB message in sensor processor for later processing"""
        # TODO: we may need to change the api of the uwb to use sensor_processor.
        self.sensor_processor.process_uwb(uwb_msg)

    def text_callback(self, text_msg):
        """Callback for text commands from /test/command topic"""
        if text_msg and text_msg.data:
            self.text_data = text_msg.data
            self.get_logger().info(f"Text command received: '{self.text_data}'")

    def asr_callback(self, asr_msg):
        """Callback for ASR commands - maps ASR commands to control commands and executes actions"""
        if asr_msg and asr_msg.data:
            asr_command = asr_msg.data.strip()
            self.get_logger().info(f"ASR command received: '{asr_command}'")

            # Check health conditions before processing ASR commands
            if not self.check_safety_conditions():
                self.get_logger().warn(f"Health check failed - ignoring ASR command: '{asr_command}'")
                return

            # Use ASR processor to map the command
            mapped_command = self.asr_processor.map_command(asr_command)

            if mapped_command:
                # Store the mapped command as text data for task logic
                self.text_data = mapped_command
                self.get_logger().info(f"ASR command mapped and stored: '{mapped_command}'")

                # Execute the action using ActionExecutor
                action_success = self.action_executor.execute_action(mapped_command)
                if action_success:
                    self.get_logger().info(f"Action executed successfully for command: '{mapped_command}'")
                else:
                    self.get_logger().warn(f"Action execution failed for command: '{mapped_command}'")
            else:
                self.get_logger().warn(f"ASR command could not be mapped: '{asr_command}'")

    def pc_callback(self, point_cloud_msg):
        # This callback processes point cloud data for health monitoring
        self.last_pc_time = time.time()

        # Process point cloud through sensor processor for health monitoring
        self.sensor_processor.process_point_cloud(point_cloud_msg)

    def check_sensor_health(self):
        """Periodic health check for all sensors including odometry"""
        odom_status = self.sensor_processor.get_odom_status()
        
        # Log odometry status
        if not odom_status['fresh']:
            if odom_status['time_since_last'] is None:
                self.get_logger().warn("No odometry data received yet!")
            else:
                self.get_logger().warn(f"Odometry data stale! Last received {odom_status['time_since_last']:.1f}s ago (timeout: {self.sensor_processor.odom_timeout}s)")
        else:
            if odom_status['message_count'] % 100 == 0:  # Log periodically when healthy
                self.get_logger().info(f"Odometry healthy: {odom_status['message_count']} messages, last {odom_status['time_since_last']:.1f}s ago")
        
        # Check UWB status
        uwb_status = self.sensor_processor.get_uwb_status()
        if not uwb_status['fresh']:
            if uwb_status['time_since_last'] is None:
                self.get_logger().warn("No UWB data received yet!")
            else:
                self.get_logger().warn(f"UWB data stale! Last received {uwb_status['time_since_last']:.1f}s ago")
        
        # Check point cloud
        current_time = time.time()
        if self.last_pc_time is not None:
            pc_age = current_time - self.last_pc_time
            if pc_age > self.pc_timeout:
                self.get_logger().warn(f"Point cloud data stale! Last received {pc_age:.1f}s ago")
        else:
            self.get_logger().warn("No point cloud data received yet!")

    def check_safety_conditions(self):
        """
        Centralized safety check for all critical sensors
        Returns True if it's safe to continue navigation, False otherwise
        """
        # Check UWB timeout using sensor processor
        if not self.sensor_processor.is_uwb_data_fresh():
            uwb_status = self.sensor_processor.get_uwb_status()
            if uwb_status['time_since_last'] is None:
                self.get_logger().warn("No UWB data - cannot navigate safely")
            else:
                self.get_logger().warn(f"UWB timeout ({uwb_status['time_since_last']:.1f}s) - stopping robot")
            return False
            
        # Check point cloud timeout
        current_time = time.time()
        if self.last_pc_time is None:
            self.get_logger().warn("No point cloud data - cannot navigate safely") 
            return False
        elif current_time - self.last_pc_time > self.pc_timeout:
            self.get_logger().warn(f"Point cloud timeout ({current_time - self.last_pc_time:.1f}s) - stopping robot")
            return False
            
        # Check odometry timeout (warn but don't stop)
        if not self.sensor_processor.is_odom_data_fresh():
            odom_status = self.sensor_processor.get_odom_status()
            if odom_status['time_since_last'] is None:
                self.get_logger().warn("No odometry data - navigation may be unreliable")
            else:
                self.get_logger().warn(f"Odometry timeout ({odom_status['time_since_last']:.1f}s) - navigation may be unreliable")
            # Don't return False for odometry timeout - continue with degraded navigation
            
        return True

    def odom_callback(self, odom_msg):
        """Store odometry message and update timing for health monitoring"""
        self.latest_odom_msg = odom_msg
        # Update timing information without full processing (high frequency callback)
        # print("get odom ")
        self.sensor_processor.last_odom_time = time.time()
        self.sensor_processor.odom_message_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TaskLogicNode()

    # rclpy.spin(node)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()