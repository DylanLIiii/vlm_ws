import numpy as np
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

    def process_odom(self, odom_msg):
        xyz = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0, 1])
        self.logger.info(f"--------------------------- Odom xyz: {xyz}")
        odom_orientation = np.array([
            odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w
        ])
        # These lines convert the quaternion to a rotation matrix
        # and then create a transformation matrix from the ego frame to the world frame.
        if len(odom_orientation) != 4:
            self.logger.error("Invalid odom orientation length, expected 4 elements.")
            return
        rotation_matrix = R.from_quat(odom_orientation).as_matrix()
        Tr_ego2world = np.eye(4)
        Tr_ego2world[:3, :3] = rotation_matrix
        Tr_ego2world[:3, 3] = xyz[:3]

        if self.Tr_world2init is None:
            self.Tr_world2init = np.linalg.inv(Tr_ego2world)
        self.Tr_ego2init = self.Tr_world2init @ Tr_ego2world
        self.Tr_init2ego = np.linalg.inv(self.Tr_ego2init)
        self.xyz = self.Tr_world2init @ xyz

    def process_point_cloud(self, point_cloud_msg):
        point_cloud = extract_lidar(point_cloud_msg, self.pc_range)
        occupancy_map_local = lidar_simple_grid_map(point_cloud, self.pc_range, self.xy_resolution)
        occupancy_map_local = np.flip(occupancy_map_local, axis=0)
        occupancy_map_local = np.rot90(occupancy_map_local, k=3)
        return occupancy_map_local