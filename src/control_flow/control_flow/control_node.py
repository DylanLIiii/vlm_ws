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


class ActionExecutor:
    """
    Handles the execution of mapped commands by publishing appropriate ROS2 messages.

    This class takes mapped commands from the ASRProcessor and executes them by publishing
    Joy messages for robot control and String messages for following commands.
    """

    def __init__(self, node):
        """
        Initialize the ActionExecutor.

        Args:
            node: ROS2 node instance for publishing and logging
        """
        self.node = node
        self.logger = node.get_logger()

        # Create publishers for different command types
        self.joy_publisher = self.node.create_publisher(Joy, '/joy', 10)
        self.following_publisher = self.node.create_publisher(String, '/following/task_type', 10)

        # Define command mappings for Joy messages
        self.joy_commands = {
            'stand_up': {
                'buttons': [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                'axes': [0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            },
            'stand_down': {
                'buttons': [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                'axes': [0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            },
            'shake_hand': {
                'buttons': [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                'axes': [0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            },
            'enter_rl_mode': {
                'buttons': [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                'axes': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            }
        }

        # Define command mappings for following commands
        self.following_commands = {
            'follow_start': 'start following',
            'follow_stop': 'stop following'
        }

        # Task status monitoring setup
        self.task_status_subscriber = self.node.create_subscription(
            String, '/task_status', self.task_status_callback, 10
        )

        # Movement command tracking
        self.movement_commands = {
            'Come Here', 'Come to my front', 'Come to my behind/back',
            'Come to my left', 'Come to my right'
        }
        self.active_movement_command = None
        self.movement_command_start_time = None

        # Task completion parameters
        self.distance_threshold = 1.5  # meters - configurable threshold for task completion
        self.last_task_status = None

        self.logger.info("ActionExecutor initialized")
        self.logger.info(f"Joy commands available: {list(self.joy_commands.keys())}")
        self.logger.info(f"Following commands available: {list(self.following_commands.keys())}")
        self.logger.info(f"Movement commands monitored: {list(self.movement_commands)}")
        self.logger.info(f"Task status monitoring enabled on /task_status topic")

    def execute_action(self, mapped_command):
        """
        Execute an action based on the mapped command.

        Args:
            mapped_command (str): The mapped command from ASRProcessor

        Returns:
            bool: True if action was executed successfully, False otherwise
        """
        if not isinstance(mapped_command, str):
            self.logger.error(f"Invalid mapped command type: {type(mapped_command)}")
            return False

        # Check if it's a Joy command
        if mapped_command in self.joy_commands:
            return self._publish_joy_command(mapped_command)

        # Check if it's a following command
        elif mapped_command in self.following_commands:
            return self._publish_following_command(mapped_command)

        else:
            # For movement commands like "Come Here", "Come to my front", etc.
            # These are handled by the existing task logic, so we just log and return True
            if mapped_command in self.movement_commands:
                self._start_movement_command_tracking(mapped_command)
                self.logger.info(f"Movement command '{mapped_command}' will be handled by task logic - tracking started")
            else:
                self.logger.info(f"Command '{mapped_command}' will be handled by task logic")
            return True

    def _publish_joy_command(self, command):
        """
        Publish a Joy message for robot control commands.

        Args:
            command (str): The command to execute

        Returns:
            bool: True if published successfully, False otherwise
        """
        try:
            if command not in self.joy_commands:
                self.logger.error(f"Unknown Joy command: {command}")
                return False

            # Create Joy message
            joy_msg = Joy()
            joy_msg.header.stamp = self.node.get_clock().now().to_msg()
            joy_msg.header.frame_id = "joy"

            # Set buttons and axes from mapping
            joy_msg.buttons = self.joy_commands[command]['buttons']
            joy_msg.axes = self.joy_commands[command]['axes']

            # Publish the message
            self.joy_publisher.publish(joy_msg)
            self.logger.info(f"Published Joy command for '{command}': buttons={joy_msg.buttons}, axes={joy_msg.axes}")
            return True

        except Exception as e:
            self.logger.error(f"Error publishing Joy command '{command}': {e}")
            return False

    def _publish_following_command(self, command):
        """
        Publish a String message for following commands.

        Args:
            command (str): The command to execute

        Returns:
            bool: True if published successfully, False otherwise
        """
        try:
            if command not in self.following_commands:
                self.logger.error(f"Unknown following command: {command}")
                return False

            # Create String message
            string_msg = String()
            string_msg.data = self.following_commands[command]

            # Publish the message
            self.following_publisher.publish(string_msg)
            self.logger.info(f"Published following command for '{command}': '{string_msg.data}'")
            return True

        except Exception as e:
            self.logger.error(f"Error publishing following command '{command}': {e}")
            return False

    def enter_rl_mode(self):
        """
        Special method to enter RL mode by publishing the appropriate Joy command.

        Returns:
            bool: True if published successfully, False otherwise
        """
        return self._publish_joy_command('enter_rl_mode')

    def _start_movement_command_tracking(self, command):
        """
        Start tracking a movement command for completion monitoring.

        Args:
            command (str): The movement command to track
        """
        self.active_movement_command = command
        self.movement_command_start_time = time.time()
        self.logger.info(f"Started tracking movement command: '{command}'")

    def _stop_movement_command_tracking(self):
        """
        Stop tracking the current movement command.
        """
        if self.active_movement_command:
            duration = time.time() - self.movement_command_start_time if self.movement_command_start_time else 0
            self.logger.info(f"Stopped tracking movement command: '{self.active_movement_command}' (duration: {duration:.1f}s)")
            self.active_movement_command = None
            self.movement_command_start_time = None

    def task_status_callback(self, msg):
        """
        Callback for task status updates from /task_status topic.

        Args:
            msg (String): JSON-formatted string containing task status information
        """
        try:
            # Parse JSON from the message
            task_status = json.loads(msg.data)
            self.last_task_status = task_status

            # Check if we're currently tracking a movement command
            if self.active_movement_command:
                self._check_movement_task_completion(task_status)

        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse task status JSON: {e}")
        except Exception as e:
            self.logger.error(f"Error processing task status: {e}")

    def _check_movement_task_completion(self, task_status):
        """
        Check if the current movement task is complete based on task status.

        Args:
            task_status (dict): Parsed task status information
        """
        try:
            # Extract required fields from task status
            current_distance = task_status.get('current_distance_to_target')
            target_reached = task_status.get('target_reached', False)
            timestamp = task_status.get('timestamp')

            # Log task status for debugging
            if current_distance is not None:
                self.logger.debug(f"Task status - Distance: {current_distance:.2f}m, Target reached: {target_reached}")

            # Check completion conditions
            task_complete = False
            completion_reason = ""

            if target_reached:
                task_complete = True
                completion_reason = "target_reached flag is True"
            elif current_distance is not None and current_distance <= self.distance_threshold:
                task_complete = True
                completion_reason = f"distance ({current_distance:.2f}m) below threshold ({self.distance_threshold}m)"

            if task_complete:
                self.logger.info(f"Movement task '{self.active_movement_command}' completed: {completion_reason}")
                self._handle_movement_task_completion()

        except Exception as e:
            self.logger.error(f"Error checking movement task completion: {e}")

    def _handle_movement_task_completion(self):
        """
        Handle the completion of a movement task by automatically stopping following.
        """
        try:
            # Stop tracking the movement command
            completed_command = self.active_movement_command
            self._stop_movement_command_tracking()

            # Automatically publish stop following command
            stop_success = self._publish_following_command('follow_stop')

            if stop_success:
                self.logger.info(f"Automatically stopped following after completing movement task: '{completed_command}'")
            else:
                self.logger.warn(f"Failed to automatically stop following after completing movement task: '{completed_command}'")

        except Exception as e:
            self.logger.error(f"Error handling movement task completion: {e}")

    def get_task_status(self):
        """
        Get the latest task status information.

        Returns:
            dict: Latest task status or None if no status received
        """
        return self.last_task_status

    def is_movement_command_active(self):
        """
        Check if a movement command is currently being tracked.

        Returns:
            bool: True if a movement command is active, False otherwise
        """
        return self.active_movement_command is not None

    def get_active_movement_command(self):
        """
        Get the currently active movement command.

        Returns:
            str: Active movement command or None if no command is active
        """
        return self.active_movement_command

    def set_distance_threshold(self, threshold):
        """
        Set the distance threshold for task completion detection.

        Args:
            threshold (float): Distance threshold in meters
        """
        if threshold > 0:
            self.distance_threshold = threshold
            self.logger.info(f"Distance threshold updated to {threshold}m")
        else:
            self.logger.error(f"Invalid distance threshold: {threshold}. Must be positive.")


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