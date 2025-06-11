import unittest
from unittest.mock import Mock, patch
import numpy as np

# Mock ROS2 message types for testing in a non-ROS environment
class MockOdometry:
    def __init__(self, x, y, z, qx, qy, qz, qw):
        self.pose = Mock()
        self.pose.pose = Mock()
        self.pose.pose.position = Mock()
        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = z
        self.pose.pose.orientation = Mock()
        self.pose.pose.orientation.x = qx
        self.pose.pose.orientation.y = qy
        self.pose.pose.orientation.z = qz
        self.pose.pose.orientation.w = qw

class MockPointCloud2:
    pass

class MockCompressedVideo:
    pass

class MockUWB:
    pass

from vita_agent.processors.sensor_processor import SensorProcessor

class TestSensorProcessor(unittest.TestCase):

    def setUp(self):
        self.mock_node = Mock()
        
        # Mock parameters
        mock_pc_range_param = Mock()
        mock_pc_range_param.get_parameter_value.return_value.double_array_value = [-51.2, -51.2, -5.0, 51.2, 51.2, 3.0]
        
        mock_xy_res_param = Mock()
        mock_xy_res_param.get_parameter_value.return_value.double_value = 0.2
        
        self.mock_node.get_parameter.side_effect = lambda name: {
            'pc_range': mock_pc_range_param,
            'xy_resolution': mock_xy_res_param
        }.get(name)
        
        self.mock_node.get_logger.return_value = Mock()
        
        self.sensor_processor = SensorProcessor(self.mock_node)

    def test_process_odom(self):
        """Test Case 2.3: Verify odometry processing and transform updates."""
        # First odom message (identity pose at origin)
        odom_msg1 = MockOdometry(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        self.sensor_processor.process_odom(odom_msg1)
        
        # After the first message, the ego-to-init transform should be the identity matrix
        np.testing.assert_array_almost_equal(self.sensor_processor.Tr_ego2init, np.eye(4))
        
        # Second odom message (translation)
        odom_msg2 = MockOdometry(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        self.sensor_processor.process_odom(odom_msg2)
        
        expected_Tr_ego2init = np.array([
            [1.0, 0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        np.testing.assert_array_almost_equal(self.sensor_processor.Tr_ego2init, expected_Tr_ego2init)

    @patch('vita_agent.processors.sensor_processor.lidar_simple_grid_map')
    @patch('vita_agent.processors.sensor_processor.extract_lidar')
    def test_process_point_cloud(self, mock_extract_lidar, mock_lidar_simple_grid_map):
        """Test Cases 2.1 & 2.2: Verify point cloud processing and filtering."""
        mock_point_cloud_msg = MockPointCloud2()
        mock_extracted_points = np.random.rand(100, 3)
        mock_grid_map = np.ones((10, 10))
        
        mock_extract_lidar.return_value = mock_extracted_points
        mock_lidar_simple_grid_map.return_value = mock_grid_map
        
        result_map = self.sensor_processor.process_point_cloud(mock_point_cloud_msg)
        
        # Test Case 2.2: Verify that filtering utility is called with correct range
        mock_extract_lidar.assert_called_once_with(mock_point_cloud_msg, self.sensor_processor.pc_range)
        
        # Test Case 2.1: Verify grid map generation
        mock_lidar_simple_grid_map.assert_called_once_with(
            mock_extracted_points, 
            self.sensor_processor.pc_range, 
            self.sensor_processor.xy_resolution
        )
        
        # Verify the map transformations
        expected_map = np.rot90(np.flip(mock_grid_map, axis=0), k=3)
        np.testing.assert_array_equal(result_map, expected_map)

    @unittest.skip("Video processing (Test Case 2.4) is not implemented in SensorProcessor.")
    def test_process_video(self):
        pass

    @unittest.skip("UWB processing (Test Case 2.5) is not implemented in SensorProcessor.")
    def test_process_uwb(self):
        pass

if __name__ == '__main__':
    unittest.main()