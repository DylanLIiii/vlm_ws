# Forward ASR Node

The `forward_asr` node is part of the VITA Tools package and provides ASR (Automatic Speech Recognition) command forwarding functionality.

## Overview

This node subscribes to ASR commands and forwards them to appropriate topics based on predefined mapping rules. It acts as a bridge between speech recognition output and robot control commands.

## Topics

### Input Topic
- **`asr_command`** (std_msgs/String): Incoming ASR commands

### Output Topics
- **`/test/command`** (std_msgs/String): Movement commands
- **`/test/follow_control`** (std_msgs/String): Follow control commands

## Command Mappings

### Movement Commands (published to `/test/command`)
| Input Command | Output Message |
|---------------|----------------|
| `move_to_master` | `"Come Here"` |
| `move_to_master_front` | `"Come to my front"` |
| `move_to_master_behind` | `"Come to my behind/back"` |
| `move_to_master_left` | `"Come to my left"` |
| `move_to_master_right` | `"Come to my right"` |

### Follow Control Commands (published to `/test/follow_control`)
| Input Command | Output Message |
|---------------|----------------|
| `follow_start` | `"follow_start"` |
| `follow_stop` | `"follow_stop"` |

## Usage

### Building the Package
```bash
cd /path/to/your/workspace
colcon build --packages-select vita_tools
source install/setup.bash
```

### Running the Node

#### Method 1: Using the executable directly
```bash
ros2 run vita_tools forward_asr
```

#### Method 2: Using the launch file
```bash
ros2 launch vita_tools forward_asr.launch.py
```

### Testing the Node

#### Manual Testing
You can test the node by publishing messages to the `asr_command` topic:

```bash
# Test movement command
ros2 topic pub /asr_command std_msgs/String "data: 'move_to_master'"

# Test follow command
ros2 topic pub /asr_command std_msgs/String "data: 'follow_start'"
```

Monitor the output topics:
```bash
# Monitor movement commands
ros2 topic echo /test/command

# Monitor follow control commands
ros2 topic echo /test/follow_control
```

#### Unit Testing
Run the unit tests to verify functionality:
```bash
cd /path/to/your/workspace
python3 -m pytest src/vita_tools/test/test_forward_asr.py -v
```

## Features

- **Command Mapping**: Translates ASR commands to human-readable robot commands
- **Error Handling**: Logs unrecognized commands for debugging
- **Whitespace Handling**: Automatically strips leading/trailing whitespace from commands
- **Comprehensive Logging**: Provides detailed logging for debugging and monitoring
- **ROS2 Best Practices**: Follows ROS2 coding conventions and node structure

## Error Handling

- Unrecognized commands are logged as warnings
- Available commands are listed in the log when an unrecognized command is received
- The node continues running even when receiving invalid commands

## Dependencies

- `rclpy`: ROS2 Python client library
- `std_msgs`: Standard ROS2 message types

## Node Information

- **Package**: vita_tools
- **Node Name**: forward_asr_node
- **Executable**: forward_asr
- **Language**: Python 3
