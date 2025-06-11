import numpy as np
from scipy.spatial.transform import Rotation
import sensor_msgs_py.point_cloud2 as pc2

def extract_lidar(point_cloud_msg, pc_range):
    """提取point cloud数据"""
    point_list = []

    for point in pc2.read_points(point_cloud_msg, skip_nans=True):
        # point 是一个包含 (x, y, z, ...) 的元组
        point_list.append([point[0], point[1], point[2], point[3]])  # 只取 x, y, z

    point_cloud = np.array(point_list)

    # Transform the point cloud to the lidar frame
    xyz = np.array([0, 0, 0, 1])  # lidar state
    rpy = np.array([0, 0, 180])
    rotation_matrix = Rotation.from_euler('xyz', rpy, degrees=True).as_matrix()

    # # Create transformation matrix
    Tr = np.eye(4)  # Start with 4x4 identity matrix
    Tr[:3, :3] = rotation_matrix  # Set rotation part
    Tr[:3, 3] = xyz[:3]  # Set translation part

    point_cloud = (np.linalg.inv(Tr) @ point_cloud.T).T

    ego_point_mask = (point_cloud[:, 0] < 0.1) & (point_cloud[:, 0] > -1) & (point_cloud[:, 1] > -0.2) & (point_cloud[:, 1] < 0.2)
    point_cloud = point_cloud[~ego_point_mask]


    filter_mask = (point_cloud[:, 0] > pc_range[0]) & (point_cloud[:, 0] < pc_range[3]) & \
                (point_cloud[:, 1] > pc_range[1]) & (point_cloud[:, 1] < pc_range[4]) & \
                (point_cloud[:, 2] > pc_range[2]) & (point_cloud[:, 2] < pc_range[5])
    point_cloud = point_cloud[filter_mask]
    return point_cloud


def lidar_simple_grid_map(raw_point_cloud, pc_range, xy_resolution):
    occ_map_dimension = int((pc_range[3] - pc_range[0]) / xy_resolution)
    # initialize grid
    grid = np.zeros((occ_map_dimension, occ_map_dimension))

    # to grid indices
    x_indices = ((raw_point_cloud[:, 0] - pc_range[0]) / xy_resolution).astype(int)
    y_indices = ((raw_point_cloud[:, 1] - pc_range[1]) / xy_resolution).astype(int)

    # Clip indices to valid range
    x_indices = np.clip(x_indices, 0, occ_map_dimension - 1)
    y_indices = np.clip(y_indices, 0, occ_map_dimension - 1)

    grid[x_indices, y_indices] = 1
    return grid
