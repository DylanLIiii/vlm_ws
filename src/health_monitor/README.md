# Health Monitor Package

## 1. Overview

This package provides a simple and lightweight ROS2 node (`health_monitor_node`) to monitor the status of critical topics in the workspace. It is designed to be self-starting with minimal performance overhead.

The node subscribes to a configurable list of topics and checks whether messages are being received within a specified timeout. If a topic stops publishing, the node will log an error, indicating a potential sensor or node failure.

## 2. Configuration

The topics to monitor and the timeout period are configured in the [`config/health_monitor.yaml`](config/health_monitor.yaml:1) file.

*   `topic_timeout_s`: The time in seconds to wait for a message before logging an error.
*   `topics_to_monitor`: A list of topic names to subscribe to and monitor.

You can edit this file to add, remove, or change the topics according to your system's needs.

## 3. Build and Run

To build the package, run the following command from the root of your workspace (`~/vlm_ws`):

```bash
colcon build --packages-select health_monitor --symlink-install
```

After building, source the workspace:

```bash
source install/local_setup.bash
```

To run the health monitor, use the provided launch file:

```bash
ros2 launch health_monitor monitor.launch.py
```

## 4. Integration (常态化)

To make the health monitor run continuously as part of your main system, you can include its launch file in your top-level launch file. Add the following `IncludeLaunchDescription` action:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# Inside your main launch description
health_monitor_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('health_monitor'),
            'launch/monitor.launch.py'
        )
    )
)
# Then add 'health_monitor_launch' to your list of actions.