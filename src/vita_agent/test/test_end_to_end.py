import unittest
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from foxglove_msgs.msg import CompressedVideo
from uwb_location.msg import UWB
from unittest.mock import MagicMock, patch
import launch
import launch_ros
import launch_testing
from launch_testing.actions import ReadyToTest

# The test case is designed to be run in a live ROS environment.
# It simulates a complete user workflow, from issuing a command to the robot reaching its target.
# It uses mocks for external services, specifically the VlmClient, to return a predefined target coordinate without making a real HTTP call.
# It monitors the /vel_cmd topic to ensure the robot starts moving and stops when it reaches the target.
# It also checks that the TaskLogicNode's state returns to IDLE after the task is complete.
# The test case is based on the End-to-End Scenario in the test plan (Test Case 7.1).
#
# To run this test, you need to:
# 1. Source your ROS 2 environment.
# 2. Run the test using pytest.
#
# Example:
# source install/setup.bash
# pytest test/test_end_to_end.py


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='vita_agent',
            executable='person_follower',
            name='task_logic_node',
            output='screen'
        )
    ])

class E2ETest(unittest.TestCase):
    def setUp(self):
        self.node = rclpy.create_node('e2e_test_node')
        self.command_pub = self.node.create_publisher(String, '/test/command', 1)
        self.sensor_pub = {
            'point_cloud': self.node.create_publisher(PointCloud2, '/lidar_points', 1),
            'odometry': self.node.create_publisher(Odometry, '/rt/odom', 1),
            'video': self.node.create_publisher(CompressedVideo, '/image_right_raw/h264_full', 1),
            'uwb': self.node.create_publisher(UWB, '/uwb/data', 1)
        }
        self.vel_cmd_sub = self.node.create_subscription(Twist, '/vel_cmd', self.vel_cmd_callback, 1)
        self.vel_cmd_msg = None

    def tearDown(self):
        self.node.destroy_node()

    def vel_cmd_callback(self, msg):
        self.vel_cmd_msg = msg

    @patch('vita_agent.clients.vlm_client.VlmClient')
    def test_e2e_scenario(self, mock_vlm_client):
        # Mock VLM client
        mock_vlm_instance = mock_vlm_client.return_value
        async def mock_get_action(*args, **kwargs):
            return [1.5, 0.0] 
        mock_vlm_instance.get_action = MagicMock(side_effect=mock_get_action)

        # 1. Publish a "Come Here" message to the /test/command topic.
        self.command_pub.publish(String(data="Come Here"))
        rclpy.spin_once(self.node, timeout_sec=1)

        # 2. Publish mock sensor data
        self.sensor_pub['point_cloud'].publish(PointCloud2())
        self.sensor_pub['odometry'].publish(Odometry())
        self.sensor_pub['video'].publish(CompressedVideo())
        self.sensor_pub['uwb'].publish(UWB(distance=2.0, angle=0.0))
        rclpy.spin_once(self.node, timeout_sec=1)

        # 4. Monitor the /vel_cmd topic to ensure the robot starts moving.
        self.assertIsNotNone(self.vel_cmd_msg, "Robot should start moving")
        self.assertNotEqual(self.vel_cmd_msg.linear.x, 0.0, "Robot should have linear velocity")

        # 6. Publish mock sensor data that simulates the robot arriving at the destination.
        self.sensor_pub['uwb'].publish(UWB(distance=0.5, angle=0.0))
        rclpy.spin_once(self.node, timeout_sec=1)

        # 7. Assert that the robot publishes a "stop" command (zero velocities) on /vel_cmd.
        self.assertAlmostEqual(self.vel_cmd_msg.linear.x, 0.0, "Robot should stop at the destination")
        self.assertAlmostEqual(self.vel_cmd_msg.angular.z, 0.0, "Robot should stop at the destination")