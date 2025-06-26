import time
import json
from enum import Enum, auto
import threading

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
# It integrates sensor processing and health monitoring with configurable health check toggle.
#
# Health Check Toggle:
# - When enabled (default): Full sensor monitoring, health validation, and safety checks
# - When disabled: Bypasses all health checks, allows all ASR commands regardless of sensor status
#
# State Machine:
# - IDLE: The default state, waiting for a command.
class TaskLogicNode(Node):
    class State(Enum):
        IDLE = auto()

    def __init__(self):
        super().__init__("task_logic_node")
        # Health check toggle parameter - default True for backward compatibility
        self.declare_parameter('health_check_enabled', False)

        # ASR Protection parameter - default False for backward compatibility
        self.declare_parameter('asr_protection_enabled', False)

        # Multi-control parameter - default False for backward compatibility
        self.declare_parameter('multi_control', False)

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
        self.declare_parameter('movement_distance_threshold', 1.0)

        # Get health check toggle state
        self.health_check_enabled = self.get_parameter('health_check_enabled').get_parameter_value().bool_value

        # Get ASR protection toggle state
        self.asr_protection_enabled = self.get_parameter('asr_protection_enabled').get_parameter_value().bool_value

        # Get multi-control toggle state
        self.multi_control_enabled = self.get_parameter('multi_control').get_parameter_value().bool_value

        self.uwb_timeout = self.get_parameter('uwb_timeout').get_parameter_value().double_value
        self.pc_timeout = self.get_parameter('pc_timeout').get_parameter_value().double_value
        self.odom_timeout = self.get_parameter('odom_timeout').get_parameter_value().double_value

        self.sensor_processor = SensorProcessor(self)
        self.asr_processor = ASRProcessor(self)

        # Get distance threshold parameter and pass to ActionExecutor
        distance_threshold = self.get_parameter('movement_distance_threshold').get_parameter_value().double_value
        self.action_executor = ActionExecutor(self, multi_control_enabled=self.multi_control_enabled)
        self.action_executor.set_distance_threshold(distance_threshold)

        # ASR Protection buffering variables
        self.asr_command_buffer = []  # List to maintain exact order and allow non-consecutive duplicates
        self.asr_buffer_timer = None
        self.asr_buffer_lock = threading.Lock()
        self.last_asr_activity_time = None
        self.asr_buffering_window = 1.0  # 1 second buffering window
        self.asr_inactivity_threshold = 1.0  # 1 second inactivity threshold

        self.odom_callback_group = ReentrantCallbackGroup()

        # Initialize health monitoring variables
        self.last_pc_time = None
        self.point_cloud_sub = None
        self.uwb_sub = None
        self.odom_sub = None
        self.sensor_health_timer = None

        self.state = self.State.IDLE
        self.text_data = None
        self.latest_odom_msg = None

        # Conditionally create health monitoring subscriptions
        if self.health_check_enabled:
            self.point_cloud_sub = self.create_subscription(
                PointCloud2, self.get_parameter('topic_lidar').get_parameter_value().string_value, self.pc_callback, 1, callback_group=self.odom_callback_group
            )
            self.get_logger().info("Point cloud subscriber node started (health check enabled)")

            # UWB
            self.uwb_sub = self.create_subscription(
                 UWB, self.get_parameter('topic_uwb').get_parameter_value().string_value, self.uwb_callback, 1, callback_group=self.odom_callback_group
            )
            self.get_logger().info("UWB subscriber node started (health check enabled)")
            # Odom subscriber - store latest odom data without immediate processing
            self.odom_sub = self.create_subscription(
                Odometry, self.get_parameter('topic_odom').get_parameter_value().string_value, self.odom_callback, 1, callback_group=self.odom_callback_group
            )
            self.get_logger().info("Odom subscriber node started (health check enabled)")
            # Add timer for monitoring sensor health including odometry
            self.sensor_health_timer = self.create_timer(
                10.0, self.check_sensor_health, #callback_group=self.timer_group
            )
            self.get_logger().info("Health monitoring timer started")
        else:
            self.get_logger().info("Health check disabled - skipping sensor/odom monitoring subscriptions")

        self.get_logger().info("Text command subscriber node started")

        # ASR command subscriber
        self.asr_sub = self.create_subscription(
            String, self.get_parameter('topic_asr_command').get_parameter_value().string_value, self.asr_callback, 1, callback_group=self.odom_callback_group
        )
        self.get_logger().info("ASR command subscriber node started")

        # Log ASR protection status
        if self.asr_protection_enabled:
            self.get_logger().info(f"ASR Protection enabled - buffering window: {self.asr_buffering_window}s, inactivity threshold: {self.asr_inactivity_threshold}s")
        else:
            self.get_logger().info("ASR Protection disabled - commands will be processed immediately")

        # Log multi-control status
        if self.multi_control_enabled:
            self.get_logger().info("Multi-control enabled - following commands will be sent 10 times with 0.1s intervals")
        else:
            self.get_logger().info("Multi-control disabled - following commands will be sent once")



    def destroy_node(self):
        """Cleanup method with health check aware cleanup"""
        # Clean up ASR buffer timer if it exists
        if hasattr(self, 'asr_buffer_timer') and self.asr_buffer_timer is not None:
            self.asr_buffer_timer.cancel()
            self.get_logger().info("ASR buffer timer cancelled during cleanup")

        if self.health_check_enabled:
            self.get_logger().info("Destroying node with health check cleanup")
        else:
            self.get_logger().info("Destroying node (health check was disabled)")
        super().destroy_node()

    def uwb_callback(self, uwb_msg):
        """Store UWB message in sensor processor for later processing"""
        # Only process if health check is enabled
        if self.health_check_enabled:
            # TODO: we may need to change the api of the uwb to use sensor_processor.
            self.sensor_processor.process_uwb(uwb_msg)

    def asr_callback(self, asr_msg):
        """Callback for ASR commands - handles buffering and deduplication if ASR protection is enabled"""
        if asr_msg and asr_msg.data:
            asr_command = asr_msg.data.strip()
            self.get_logger().info(f"ASR command received: '{asr_command}'")

            # Check health conditions before processing ASR commands (only if health check is enabled)
            if self.health_check_enabled:
                if not self.check_safety_conditions():
                    self.get_logger().warn(f"Health check failed - ignoring ASR command: '{asr_command}'")
                    return
            else:
                self.get_logger().debug(f"Health check disabled - processing ASR command without safety validation: '{asr_command}'")

            # Process command based on ASR protection setting
            if self.asr_protection_enabled:
                self._handle_asr_command_with_protection(asr_command)
            else:
                self._process_asr_command_immediately(asr_command)

    def _handle_asr_command_with_protection(self, asr_command):
        """Handle ASR command with buffering and consecutive duplicate removal"""
        with self.asr_buffer_lock:
            current_time = time.time()

            # Check if this is the first command after inactivity
            if (self.last_asr_activity_time is None or
                current_time - self.last_asr_activity_time > self.asr_inactivity_threshold):

                # Start buffering window
                self.get_logger().info(f"Starting ASR buffering window - first command after inactivity: '{asr_command}'")
                self._start_asr_buffering_window()

            # Update last activity time
            self.last_asr_activity_time = current_time

            # Add command to buffer only if it's not the same as the last command (consecutive duplicate removal)
            if not self.asr_command_buffer or self.asr_command_buffer[-1] != asr_command:
                self.asr_command_buffer.append(asr_command)
                self.get_logger().info(f"Added ASR command to buffer: '{asr_command}' (buffer size: {len(self.asr_command_buffer)})")
            else:
                self.get_logger().info(f"Consecutive duplicate ASR command ignored: '{asr_command}'")

    def _start_asr_buffering_window(self):
        """Start the ASR buffering window timer"""
        # Cancel any existing timer
        if self.asr_buffer_timer is not None:
            self.asr_buffer_timer.cancel()

        # Start new timer for buffering window
        self.asr_buffer_timer = threading.Timer(self.asr_buffering_window, self._execute_buffered_asr_commands)
        self.asr_buffer_timer.start()
        self.get_logger().info(f"ASR buffering window started - will execute commands in {self.asr_buffering_window}s")

    def _execute_buffered_asr_commands(self):
        """Execute all buffered ASR commands sequentially"""
        with self.asr_buffer_lock:
            if not self.asr_command_buffer:
                self.get_logger().info("ASR buffering window ended - no commands to execute")
                return

            commands_to_execute = self.asr_command_buffer.copy()
            self.get_logger().info(f"ASR buffering window ended - executing {len(commands_to_execute)} commands: {commands_to_execute}")

            # Clear the buffer
            self.asr_command_buffer.clear()
            self.asr_buffer_timer = None

        # Execute commands sequentially (outside the lock to avoid blocking)
        for command in commands_to_execute:
            self.get_logger().info(f"Executing buffered ASR command: '{command}'")
            self._process_asr_command_immediately(command)

    def _process_asr_command_immediately(self, asr_command):
        """Process ASR command immediately without buffering"""
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
        if self.health_check_enabled:
            self.last_pc_time = time.time()
            # Process point cloud through sensor processor for health monitoring
            self.sensor_processor.process_point_cloud(point_cloud_msg)

    def check_sensor_health(self):
        """Periodic health check for all sensors including odometry"""
        # Only perform health checks if enabled
        if not self.health_check_enabled:
            self.get_logger().debug("Health check disabled - skipping sensor health monitoring")
            return

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
        # If health check is disabled, always return True (allow all commands)
        if not self.health_check_enabled:
            self.get_logger().debug("Health check disabled - bypassing safety conditions")
            return True

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
        if self.health_check_enabled:
            self.latest_odom_msg = odom_msg
            # Update timing information without full processing (high frequency callback)
            # print("get odom ")
            self.sensor_processor.last_odom_time = time.time()
            self.sensor_processor.odom_message_count += 1

    def is_health_check_enabled(self):
        """
        Check if health check is currently enabled.

        Returns:
            bool: True if health check is enabled, False otherwise
        """
        return self.health_check_enabled

    def get_health_check_status(self):
        """
        Get comprehensive health check status information.

        Returns:
            dict: Dictionary containing health check status and related information
        """
        status = {
            'health_check_enabled': self.health_check_enabled,
            'sensor_subscriptions_active': {
                'point_cloud': self.point_cloud_sub is not None,
                'uwb': self.uwb_sub is not None,
                'odometry': self.odom_sub is not None
            },
            'health_timer_active': self.sensor_health_timer is not None
        }

        if self.health_check_enabled:
            # Add sensor status information when health check is enabled
            status['sensor_status'] = {
                'uwb_fresh': self.sensor_processor.is_uwb_data_fresh() if hasattr(self.sensor_processor, 'is_uwb_data_fresh') else None,
                'odom_fresh': self.sensor_processor.is_odom_data_fresh() if hasattr(self.sensor_processor, 'is_odom_data_fresh') else None,
                'last_pc_time': self.last_pc_time
            }

        return status

    def set_health_check_enabled(self, enabled):
        """
        Enable or disable health check at runtime.
        Note: This only affects the toggle state, not subscription management.
        For full subscription management, node restart is recommended.

        Args:
            enabled (bool): True to enable health check, False to disable
        """
        old_state = self.health_check_enabled
        self.health_check_enabled = enabled

        if old_state != enabled:
            if enabled:
                self.get_logger().info("Health check enabled at runtime")
            else:
                self.get_logger().info("Health check disabled at runtime - ASR commands will bypass safety validation")
        else:
            self.get_logger().debug(f"Health check state unchanged: {enabled}")

    def is_asr_protection_enabled(self):
        """
        Check if ASR protection is currently enabled.

        Returns:
            bool: True if ASR protection is enabled, False otherwise
        """
        return self.asr_protection_enabled

    def set_asr_protection_enabled(self, enabled):
        """
        Enable or disable ASR protection at runtime.

        Args:
            enabled (bool): True to enable ASR protection, False to disable
        """
        old_state = self.asr_protection_enabled
        self.asr_protection_enabled = enabled

        if old_state != enabled:
            if enabled:
                self.get_logger().info("ASR Protection enabled at runtime - commands will be buffered and deduplicated")
            else:
                self.get_logger().info("ASR Protection disabled at runtime - commands will be processed immediately")
                # Cancel any active buffering timer when disabling
                with self.asr_buffer_lock:
                    if self.asr_buffer_timer is not None:
                        self.asr_buffer_timer.cancel()
                        self.asr_buffer_timer = None
                        self.get_logger().info("Active ASR buffering timer cancelled")
                    # Clear any buffered commands
                    if self.asr_command_buffer:
                        cleared_count = len(self.asr_command_buffer)
                        self.asr_command_buffer.clear()
                        self.get_logger().info(f"Cleared {cleared_count} buffered ASR commands")
        else:
            self.get_logger().debug(f"ASR protection state unchanged: {enabled}")

    def get_asr_protection_status(self):
        """
        Get comprehensive ASR protection status information.

        Returns:
            dict: Dictionary containing ASR protection status and related information
        """
        with self.asr_buffer_lock:
            status = {
                'asr_protection_enabled': self.asr_protection_enabled,
                'buffering_window': self.asr_buffering_window,
                'inactivity_threshold': self.asr_inactivity_threshold,
                'buffer_active': self.asr_buffer_timer is not None,
                'buffered_commands_count': len(self.asr_command_buffer),
                'buffered_commands': self.asr_command_buffer.copy() if self.asr_command_buffer else [],
                'last_activity_time': self.last_asr_activity_time
            }
        return status

    def is_multi_control_enabled(self):
        """
        Check if multi-control is currently enabled.

        Returns:
            bool: True if multi-control is enabled, False otherwise
        """
        return self.multi_control_enabled

    def set_multi_control_enabled(self, enabled):
        """
        Enable or disable multi-control at runtime.

        Args:
            enabled (bool): True to enable multi-control, False to disable
        """
        old_state = self.multi_control_enabled
        self.multi_control_enabled = enabled

        # Update the ActionExecutor's setting
        self.action_executor.set_multi_control_enabled(enabled)

        if old_state != enabled:
            if enabled:
                self.get_logger().info("Multi-control enabled at runtime - following commands will be sent 10 times with 0.1s intervals")
            else:
                self.get_logger().info("Multi-control disabled at runtime - following commands will be sent once")
        else:
            self.get_logger().debug(f"Multi-control state unchanged: {enabled}")

    def get_multi_control_status(self):
        """
        Get comprehensive multi-control status information.

        Returns:
            dict: Dictionary containing multi-control status and related information
        """
        status = {
            'multi_control_enabled': self.multi_control_enabled,
            'action_executor_multi_control': self.action_executor.get_multi_control_enabled(),
            'repeat_count': 10,
            'interval_seconds': 0.1,
            'total_duration_seconds': 1.0
        }
        return status

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