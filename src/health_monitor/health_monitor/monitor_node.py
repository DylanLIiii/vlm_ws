# Copyright 2024 Heng Li
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Joy
from std_msgs.msg import String
from uwb_location.msg import UWB
from foxglove_msgs.msg import CompressedVideo
from nav_msgs.msg import Odometry
# Attempt to import LowState and LowCmd, will fail if lowlevel_msg is not found
# but the structure is what the user requested.
# from lowlevel_msg.msg import LowState, LowCmd

# A mapping from topic name to message type.
# In a real-world scenario, this could be discovered dynamically
# or configured more robustly.
TOPIC_TYPE_MAP = {
    "/lidar_points": PointCloud2,
    "/uwb/data": UWB,
    "/image_right_raw/h264_half": CompressedVideo,
    "/rt/odom": Odometry,
    "/joy": Joy,
}

class HealthMonitorNode(Node):
    """Monitors topics for message activity and reports timeouts."""

    def __init__(self):
        """Initialize the HealthMonitorNode."""
        super().__init__('health_monitor_node')

        # New: topic configs with baseline_hz and tolerable_fluctuation_pct
        self.declare_parameter('topics_to_monitor', [
            {'name': '/scan', 'baseline_hz': 10.0, 'tolerable_fluctuation_pct': 20.0},
            {'name': '/image_raw', 'baseline_hz': 30.0, 'tolerable_fluctuation_pct': 10.0}
        ])
        self.declare_parameter('topic_timeout_s', 10.0)

        # Parse topic configs
        self.topic_configs = self.get_parameter('topics_to_monitor').get_parameter_value().string_array_value
        if not self.topic_configs:
            # fallback for old config style
            self.topic_configs = [
                {'name': t, 'baseline_hz': 1.0, 'tolerable_fluctuation_pct': 100.0} for t in self.get_parameter('topics_to_monitor').get_parameter_value().string_array_value
            ]
        else:
            import json
            self.topic_configs = [json.loads(cfg) if isinstance(cfg, str) else cfg for cfg in self.topic_configs]

        self.topic_timeout_s = self.get_parameter('topic_timeout_s').get_parameter_value().double_value

        self.topics_to_monitor = [cfg['name'] for cfg in self.topic_configs]
        self.last_message_times = {topic: self.get_clock().now() for topic in self.topics_to_monitor}
        # New: store message timestamps for Hz calculation
        self.message_timestamps = {topic: [] for topic in self.topics_to_monitor}
        self.topic_config_map = {cfg['name']: cfg for cfg in self.topic_configs}
        self._create_subscriptions()

        self.timer = self.create_timer(1.0, self.check_topics)
        self.get_logger().info('Health monitor node started.')

    def _create_subscriptions(self):
        """Create subscriptions for the topics to monitor."""
        for topic_name in self.topics_to_monitor:
            msg_type = TOPIC_TYPE_MAP.get(topic_name)
            if msg_type:
                self.create_subscription(
                    msg_type,
                    topic_name,
                    lambda msg, topic=topic_name: self.topic_callback(msg, topic),
                    QoSProfile(
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST,
                        depth=1
                    )
                )
                self.get_logger().info(f'Subscribed to {topic_name} with type {msg_type.__name__}')
            else:
                self.get_logger().error(f'Could not find message type for topic: {topic_name}')

    def topic_callback(self, msg, topic_name):
        """Update the last message time and record timestamp for Hz calculation."""
        now = self.get_clock().now()
        self.last_message_times[topic_name] = now
        # Record timestamp for Hz calculation (keep last 100)
        ts_list = self.message_timestamps[topic_name]
        ts_list.append(now)
        if len(ts_list) > 100:
            ts_list.pop(0)
        self.get_logger().debug(f'Received message on {topic_name}')

    def check_topics(self):
        """Check for topics that have not received messages recently and check Hz."""
        current_time = self.get_clock().now()
        for topic_name, last_time in self.last_message_times.items():
            duration_since_last_message = current_time - last_time
            if duration_since_last_message.nanoseconds / 1e9 > self.topic_timeout_s:
                self.get_logger().error(
                    f"Topic '{topic_name}' has timed out. "
                    f"No messages received for {duration_since_last_message.nanoseconds / 1e9:.2f} seconds."
                )
            # Check Hz
            ts_list = self.message_timestamps[topic_name]
            if len(ts_list) >= 2:
                # Calculate Hz over last N messages (use last 10s window if possible)
                window_sec = 10.0
                cutoff = current_time.nanoseconds - int(window_sec * 1e9)
                recent_ts = [t for t in ts_list if t.nanoseconds >= cutoff]
                if len(recent_ts) >= 2:
                    dt = (recent_ts[-1].nanoseconds - recent_ts[0].nanoseconds) / 1e9
                    hz = (len(recent_ts) - 1) / dt if dt > 0 else 0.0
                else:
                    hz = 0.0
                cfg = self.topic_config_map[topic_name]
                baseline = cfg.get('baseline_hz', 1.0)
                tol_pct = cfg.get('tolerable_fluctuation_pct', 100.0)
                min_hz = baseline * (1 - tol_pct / 100.0)
                max_hz = baseline * (1 + tol_pct / 100.0)
                if hz < min_hz or hz > max_hz:
                    self.get_logger().error(
                        f"Topic '{topic_name}' Hz out of range: {hz:.2f} Hz (baseline: {baseline} Hz, allowed: {min_hz:.2f}-{max_hz:.2f} Hz)"
                    )

def main(args=None):
    """Main entry point for the health monitor node."""
    rclpy.init(args=args)
    node = HealthMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()