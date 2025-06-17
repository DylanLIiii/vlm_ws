import json
import time
from enum import Enum, auto
import cv2
import numpy as np
import urllib.parse
import base64
import httpx
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import String, UInt8MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.spatial.transform import Rotation as R
from foxglove_msgs.msg import CompressedVideo
from uwb_location.msg import UWB

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from vita_agent.utils.lidar_utils import extract_lidar, lidar_simple_grid_map


from vita_agent.utils.path_planner_utils import (
    PathFinderController,
    check_collision_free,
    sample_path,
    generate_local_yaw_rate,
    plan_path,
)

from vita_agent.utils.h264_utils import H264DecoderContext


from vita_agent.clients.vlm_client import VlmClient
from vita_agent.processors.sensor_processor import SensorProcessor
from vita_agent.planning.path_planner import PathPlanner
from vita_agent.motion.motion_controller import MotionController




# Manages the overall task logic using a state machine.
# It integrates sensor processing, VLM interaction, path planning, and motion control.
#
# State Machine:
# - IDLE: The default state, waiting for a command.
# - WAITING_FOR_VLM: A text command has been received. The node is waiting for
#   the necessary sensor data (image) to call the VLM.
# - EXECUTING_ACTION: The VLM has provided a target. The robot is navigating
#   towards the target while avoiding obstacles.
class TaskLogicNode(Node):
    class State(Enum):
        IDLE = auto()
        WAITING_FOR_VLM = auto()
        EXECUTING_ACTION = auto()

    def __init__(self):
        super().__init__("task_logic_node")
        # Occupancy grid map parametersq
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
 
        self.vlm_server_url = self.get_parameter('vlm_server_url').get_parameter_value().string_value
        self.pc_range = self.get_parameter('pc_range').get_parameter_value().double_array_value
        self.xy_resolution = self.get_parameter('xy_resolution').get_parameter_value().double_value
        self.stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value
        self.uwb_timeout = self.get_parameter('uwb_timeout').get_parameter_value().double_value
        self.pc_timeout = self.get_parameter('pc_timeout').get_parameter_value().double_value
        self.odom_timeout = self.get_parameter('odom_timeout').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        self.sensor_processor = SensorProcessor(self)

        self.odom_callback_group = ReentrantCallbackGroup()

        self.occ_map_dimension = int((self.pc_range[3] - self.pc_range[0]) / self.xy_resolution)
        pc_callback_group = MutuallyExclusiveCallbackGroup()
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
 
        # VLM
        self.vlm_client = VlmClient(self)
        self.vlm_reponse = None
 
        self.path_planner = PathPlanner(self)

        # Control
        self.stop_flag = False
 
        self.motion_controller = MotionController(self, 
        #callback_group=self.motion_control_group
        )
 
        # ASR subscriber
        self.text_sub = self.create_subscription(
            String, self.get_parameter('topic_text_command').get_parameter_value().string_value, self.text_callback, 1, callback_group=self.odom_callback_group
        )
        self.get_logger().info("Text command subscriber node started")
 
        self.state = self.State.IDLE
        self.text_data = None
        self.arrive_vlm_count = 0
        self.action_target_position = None
        self.vlm_request_in_progress = False
 
        # H.264 decoder initialization
        self.bridge = CvBridge()
        self.h264_decoder = H264DecoderContext(debug_first_frames=False, logger=self.get_logger(), verbose=False)
        self.frame_count = 0
 
        # CompressedVideo subscriber
        self.video_sub = self.create_subscription(
            CompressedVideo,
            self.get_parameter('topic_video').get_parameter_value().string_value,
            self.compressed_video_callback,
            10,
            # callback_group=self.sensor_group
        )
        self.get_logger().info("CompressedVideo subscriber node started")        
        self.rgb = None
        self.depth = None
        self.latest_odom_msg = None

        self.valid_atomic_actions = {
            "Come Here": False,
        }
 
        # Odom subscriber - store latest odom data without immediate processing
        self.latest_odom_msg = None
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('topic_odom').get_parameter_value().string_value, self.odom_callback, 1, callback_group=self.odom_callback_group
        )
        self.get_logger().info("Odom subscriber node started")
        
        # Add timer for monitoring sensor health including odometry
        self.sensor_health_timer = self.create_timer(
            10.0, self.check_sensor_health, #callback_group=self.timer_group
        )


    def destroy_node(self):
        """Cleanup method to properly close H.264 decoder context"""
        try:
            if hasattr(self, 'h264_decoder') and self.h264_decoder is not None:
                self.h264_decoder.reset()
                self.get_logger().info("H.264 decoder context reset")
        except Exception as e:
            self.get_logger().warn(f"Error resetting H.264 decoder context: {e}")
        
        super().destroy_node()




    def uwb_callback(self, uwb_msg):
        """Store UWB message in sensor processor for later processing"""
        # TODO: we may need to change the api of the uwb to use sensor_processor.
        self.sensor_processor.process_uwb(uwb_msg)

    def text_callback(self, text_msg):
        """Callback for text commands from /test/command topic"""
        print("Now I am getting text msg")
        if text_msg and text_msg.data:
            if self.state == self.State.IDLE:
                self.text_data = text_msg.data
                self.state = self.State.WAITING_FOR_VLM
                self.vlm_request_in_progress = False  # Reset flag for new command
                self.get_logger().info(f"Text command received: '{self.text_data}'. Transitioning to WAITING_FOR_VLM.")
            else:
                self.get_logger().warn(f"Ignoring text command '{text_msg.data}' while in state {self.state.name}")

    def decode_h264_frame(self, compressed_video_msg):
        """
        Decode H.264 frame using the H264DecoderContext
        
        Args:
            compressed_video_msg: CompressedVideo message containing H.264 data
            
        Returns:
            Decoded OpenCV image or None if decoding failed
        """
        # Use the H264DecoderContext to decode the frame
        cv_image = self.h264_decoder.decode_frame(compressed_video_msg)
        
        if cv_image is not None:
            if self.frame_count <= 5:  # Log success for first few frames
                self.get_logger().info(f"Successfully decoded H.264 frame {self.frame_count}")
            return cv_image
        
    def compressed_video_callback(self, msg: CompressedVideo):
        """
        Compressed video callback with fallback decoding
        """
        self.frame_count += 1
        
        # Disable debug logging after first few frames
        if self.frame_count == 5:
            self.h264_decoder.debug_first_frames = False
            self.get_logger().info("Disabling debug logging for subsequent frames")
        
        try:
            # First try H.264 decoding
            cv_image = self.decode_h264_frame(msg)
            
            if cv_image is None:
                if self.frame_count <= 5:  # Only log for first few frames
                    self.get_logger().warn(f'Frame {self.frame_count}: All decoding methods failed')
                return
            
            # Apply fisheye correction if available
            if hasattr(self, 'corrector_func') and self.corrector_func is not None:
                cv_image = self.corrector_func(cv_image)
                if cv_image is None:
                    self.get_logger().warn('Fisheye correction resulted in None.')
                    return
            
            # Convert from BGR to RGB for consistency with previous implementation
            self.rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Note: depth is no longer available from CompressedVideo
            self.depth = None
            
        except Exception as e:
            self.get_logger().error(f'Error in compressed video callback: {str(e)}', exc_info=True)


    def pc_callback(self, point_cloud_msg):
        # This is the main callback that drives the state machine based on Lidar data.
        # It processes the point cloud to create a local occupancy map and then
        # executes the logic for the current state.
        self.last_pc_time = time.time()

        if self.state == self.State.IDLE:
            return

        # Get UWB target when needed for navigation
        uwb_target = self.sensor_processor.get_uwb_target_position()
        if uwb_target is None or uwb_target.sum() == 0:
            self.get_logger().info("Waiting for valid UWB data...")
            return
            
        # Process latest odometry data when needed for navigation
        if self.latest_odom_msg is not None:
            self.sensor_processor.process_odom(self.latest_odom_msg)
            
        # Check if transforms are ready
        if self.sensor_processor.Tr_init2ego is None:
            self.get_logger().warn("Odometry transforms not yet initialized - waiting for odometry data")
            return

        # For navigation states, check safety conditions
        if self.state in [self.State.WAITING_FOR_VLM, self.State.EXECUTING_ACTION]:
            if not self.check_safety_conditions():
                self.get_logger().warn("Safety conditions not met - stopping navigation")
                if self.state == self.State.EXECUTING_ACTION:
                    self.motion_controller.stop()
                return

        

        if self.state == self.State.WAITING_FOR_VLM:
            self.get_logger().info("State: WAITING_FOR_VLM. Checking for prerequisites...")
            if self.text_data is None:
                self.get_logger().info("Waiting for text command...")
                return
            if self.rgb is None:
                self.get_logger().info("Waiting for image...")
                return
            
            # Check if VLM request is already in progress
            if self.vlm_request_in_progress:
                self.get_logger().info("VLM request already in progress, waiting for response...")
                return
            
            # Mark that we're making a VLM request
            self.vlm_request_in_progress = True
            self.get_logger().info("Prerequisites met. Calling VLM client...")
            vlm_res = asyncio.run(self.vlm_client.get_action(self.rgb, self.text_data, uwb_target))

            if vlm_res is None:
                self.get_logger().error("VLM processing failed. Returning to IDLE.")
                self.state = self.State.IDLE
                self.vlm_request_in_progress = False  # Reset flag on failure
                return
            
            self.get_logger().info(f"VLM response received: {vlm_res}")
            
            class FakeAction:
                action = "Come Here"
                parameters = vlm_res
            
            fake_response = {"actions": [FakeAction]}
            atomic_actions = fake_response['actions']

            action_found = False
            for atomic_action in atomic_actions:
                if atomic_action.action == "ComeToObject" or atomic_action.action == "Come Here":
                    self.action_target_position = atomic_action.parameters
                    self.state = self.State.EXECUTING_ACTION
                    self.vlm_request_in_progress = False  # Reset flag on success
                    action_found = True
                    self.get_logger().info(f"Action 'Come Here' found. Target: {self.action_target_position}. Transitioning to EXECUTING_ACTION.")
                    break
            
            if not action_found:
                self.get_logger().warn("Unsupported action from VLM. Returning to IDLE.")
                self.state = self.State.IDLE
                self.vlm_request_in_progress = False  # Reset flag on unsupported action

        elif self.state == self.State.EXECUTING_ACTION:
            self.get_logger().info("State: EXECUTING_ACTION.")
            if self.action_target_position is None:
                self.get_logger().error("In EXECUTING_ACTION state but no target position. Returning to IDLE.")
                self.state = self.State.IDLE
                return

            tracking_position = [self.action_target_position[0], self.action_target_position[1], 0, 1]
            tracking_position = self.sensor_processor.Tr_init2ego @ tracking_position
            
            self.get_logger().info(f"Current tracking position: {tracking_position[:2]}")

            distance_to_target = np.hypot(tracking_position[0], tracking_position[1])
            if distance_to_target < self.stop_distance:
                self.arrive_vlm_count += 1
                self.get_logger().info(f"Near target. Arrival count: {self.arrive_vlm_count}")
            else:
                self.arrive_vlm_count = 0

            if self.arrive_vlm_count >= 3:
                self.get_logger().info("Target reached. Task complete. Returning to IDLE.")
                self.state = self.State.IDLE
                self.action_target_position = None
                self.arrive_vlm_count = 0
                self.text_data = None
                self.vlm_request_in_progress = False  # Reset flag when task completes
                self.motion_controller.stop()
                return
            occupancy_map_local = self.sensor_processor.process_point_cloud(point_cloud_msg)
            v_traj, w_traj, stop_flag = self.path_planner.plan_trajectory(tracking_position, occupancy_map_local)
            self.stop_flag = stop_flag

            if self.stop_flag:
                self.motion_controller.go_back()
            else:
                self.motion_controller.move_to_target(tracking_position, v_traj, w_traj)

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
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

