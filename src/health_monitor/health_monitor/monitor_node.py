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
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String

# A mapping from topic name to message type.
# In a real-world scenario, this could be discovered dynamically
# or configured more robustly.
TOPIC_TYPE_MAP = {
    '/scan': LaserScan,
    '/image_raw': Image,
    '/chatter': String,
}

class HealthMonitorNode(Node):
    """Monitors topics for message activity and reports timeouts."""

    def __init__(self):
        """Initialize the HealthMonitorNode."""
        super().__init__('health_monitor_node')

        self.declare_parameter('topics_to_monitor', ['/scan', '/image_raw'])
        self.declare_parameter('topic_timeout_s', 5.0)

        self.topics_to_monitor = self.get_parameter('topics_to_monitor').get_parameter_value().string_array_value
        self.topic_timeout_s = self.get_parameter('topic_timeout_s').get_parameter_value().double_value

        self.last_message_times = {topic: self.get_clock().now() for topic in self.topics_to_monitor}
        
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
        """Update the last message time for a given topic."""
        self.last_message_times[topic_name] = self.get_clock().now()
        self.get_logger().debug(f'Received message on {topic_name}')

    def check_topics(self):
        """Check for topics that have not received messages recently."""
        current_time = self.get_clock().now()
        for topic_name, last_time in self.last_message_times.items():
            duration_since_last_message = current_time - last_time
            if duration_since_last_message.nanoseconds / 1e9 > self.topic_timeout_s:
                self.get_logger().error(
                    f"Topic '{topic_name}' has timed out. "
                    f"No messages received for {duration_since_last_message.nanoseconds / 1e9:.2f} seconds."
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