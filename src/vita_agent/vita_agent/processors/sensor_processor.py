import numpy as np
import time
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from vita_agent.utils.lidar_utils import extract_lidar, lidar_simple_grid_map


class SensorProcessor:
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        self.pc_range = node.get_parameter('pc_range').get_parameter_value().double_array_value
        self.xy_resolution = node.get_parameter('xy_resolution').get_parameter_value().double_value

        self.Tr_world2init = None
        self.Tr_ego2init = None
        self.Tr_init2ego = None
        self.xyz = None
        
        # Add odometry timeout tracking
        self.last_odom_time = None
        self.odom_timeout = node.get_parameter('odom_timeout').get_parameter_value().double_value
        self.odom_message_count = 0
        
        # Add UWB data storage
        self.latest_uwb_msg = None
        self.last_uwb_time = None
        self.uwb_timeout = node.get_parameter('uwb_timeout').get_parameter_value().double_value

    def process_odom(self, odom_msg):
        """Process odometry message with timeout tracking and validation"""
        current_time = time.time()
        self.last_odom_time = current_time
        self.odom_message_count += 1
        
        # Log odometry reception status periodically
        if self.odom_message_count % 50 == 1:  # Log every 50th message to avoid spam
            self.logger.info(f"Odometry callback active - message count: {self.odom_message_count}")
        
        try:
            xyz = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0, 1])
            
            # Log detailed position info for first few messages or periodically
            if self.odom_message_count <= 5 or self.odom_message_count % 100 == 0:
                self.logger.info(f"Odom xyz: {xyz[:3]} (message #{self.odom_message_count})")
            
            odom_orientation = np.array([
                odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w
            ])
            
            # These lines convert the quaternion to a rotation matrix
            # and then create a transformation matrix from the ego frame to the world frame.
            if len(odom_orientation) != 4:
                self.logger.error("Invalid odom orientation length, expected 4 elements.")
                return
                
            # Validate quaternion (should have unit length)
            quat_norm = np.linalg.norm(odom_orientation)
            if abs(quat_norm - 1.0) > 0.1:  # Allow some tolerance
                self.logger.warn(f"Quaternion not normalized: norm = {quat_norm}")
            
            rotation_matrix = R.from_quat(odom_orientation).as_matrix()
            Tr_ego2world = np.eye(4)
            Tr_ego2world[:3, :3] = rotation_matrix
            Tr_ego2world[:3, 3] = xyz[:3]

            if self.Tr_world2init is None:
                self.Tr_world2init = np.linalg.inv(Tr_ego2world)
                self.logger.info("Initial odometry transform established")
                
            self.Tr_ego2init = self.Tr_world2init @ Tr_ego2world
            self.Tr_init2ego = np.linalg.inv(self.Tr_ego2init)
            self.xyz = self.Tr_world2init @ xyz
            
        except Exception as e:
            self.logger.error(f"Error processing odometry: {e}")

    def is_odom_data_fresh(self):
        """Check if odometry data is fresh (within timeout period)"""
        if self.last_odom_time is None:
            return False
        return (time.time() - self.last_odom_time) < self.odom_timeout

    def get_odom_status(self):
        """Get comprehensive odometry status information"""
        current_time = time.time()
        if self.last_odom_time is None:
            return {
                'fresh': False,
                'message_count': self.odom_message_count,
                'time_since_last': None,
                'transforms_ready': False
            }
        
        time_since_last = current_time - self.last_odom_time
        return {
            'fresh': time_since_last < self.odom_timeout,
            'message_count': self.odom_message_count,
            'time_since_last': time_since_last,
            'transforms_ready': self.Tr_init2ego is not None
        }

    def process_point_cloud(self, point_cloud_msg):
        point_cloud = extract_lidar(point_cloud_msg, self.pc_range)
        occupancy_map_local = lidar_simple_grid_map(point_cloud, self.pc_range, self.xy_resolution)
        occupancy_map_local = np.flip(occupancy_map_local, axis=0)
        occupancy_map_local = np.rot90(occupancy_map_local, k=3)
        return occupancy_map_local

    def process_uwb(self, uwb_msg):
        """Store the latest UWB message for later processing"""
        self.latest_uwb_msg = uwb_msg
        self.last_uwb_time = time.time()

    def get_uwb_target_position(self):
        """
        Calculate UWB target position when needed for path planning
        Returns the UWB target as a numpy array [x, y, z, 1] or None if no valid data
        """
        if self.latest_uwb_msg is None:
            self.logger.warn("No UWB data available for target calculation")
            return None
            
        if not self.is_uwb_data_fresh():
            self.logger.warn("UWB data is stale - not using for target calculation")
            return None
            
        # Extract UWB state and calculate target position
        distance = self.latest_uwb_msg.distance
        angle = np.deg2rad(-self.latest_uwb_msg.angle - 120)
        uwb_target = np.array([distance * np.cos(angle), distance * np.sin(angle), 0, 1])
        
        if self.logger.get_effective_level() <= 10:  # DEBUG level
            self.logger.debug(f"UWB target calculated: distance={distance:.2f}, angle={np.rad2deg(angle):.1f}°, position={uwb_target[:2]}")
            
        return uwb_target

    def is_uwb_data_fresh(self):
        """Check if UWB data is fresh (within timeout period)"""
        if self.last_uwb_time is None:
            return False
        return (time.time() - self.last_uwb_time) < self.uwb_timeout

    def get_uwb_status(self):
        """Get comprehensive UWB status information"""
        current_time = time.time()
        if self.last_uwb_time is None:
            return {
                'fresh': False,
                'time_since_last': None,
                'has_data': False
            }
        
        time_since_last = current_time - self.last_uwb_time
        return {
            'fresh': time_since_last < self.uwb_timeout,
            'time_since_last': time_since_last,
            'has_data': self.latest_uwb_msg is not None
        }