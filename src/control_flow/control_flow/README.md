# TaskLogicNode Control Flow Documentation

## Overview

The `TaskLogicNode` class (`control_node.py`) manages task execution in a ROS2-based robotic system through a state machine architecture. It integrates sensor processing, health monitoring, and command execution with configurable safety checks.

## Task Categories

The system handles three main categories of tasks:

1. **Text Commands** - Direct text-based commands via `/test/command` topic
2. **ASR Commands** - Speech recognition commands via `/asr_command` topic that get mapped to control commands
3. **Health Monitoring Tasks** - Continuous sensor health validation and safety checks

## Task Properties

### Command Properties
- **Text Data**: Raw command strings stored in `self.text_data`
- **Mapped Commands**: ASR commands processed through `ASRProcessor.map_command()`
- **Execution Status**: Tracked by `ActionExecutor` with success/failure reporting

### Health Check Properties
- **Configurable**: Controlled by `health_check_enabled` parameter (default: True)
- **Timeout-based**: UWB (30s), Point Cloud (30s), Odometry (5s) timeouts
- **Non-blocking**: Odometry timeout warns but doesn't stop execution

## Task Control

### Command Processing Flow
1. **ASR Commands**: `asr_callback()` → `ASRProcessor.map_command()` → `ActionExecutor.execute_action()`
2. **Text Commands**: `text_callback()` → Store in `self.text_data`
3. **Safety Validation**: `check_safety_conditions()` validates sensor health before execution

### Health Check Control
- **Enabled Mode**: Full sensor monitoring with safety validation
- **Disabled Mode**: Bypasses all safety checks, allows all commands
- **Runtime Toggle**: `set_health_check_enabled()` for dynamic control

## Status Management

### Task Status Tracking
- **Action Execution**: `ActionExecutor.execute_action()` returns success/failure
- **Command Storage**: Latest commands stored in `self.text_data`
- **State Machine**: Currently uses single `IDLE` state

### Health Status Monitoring
- **Sensor Health Timer**: 10-second periodic health checks via `check_sensor_health()`
- **Real-time Validation**: `check_safety_conditions()` for immediate safety assessment
- **Status Reporting**: `get_health_check_status()` provides comprehensive health information

## Status Retrieval

### Health Check Status
```python
# Check if health monitoring is enabled
is_enabled = node.is_health_check_enabled()

# Get comprehensive health status
status = node.get_health_check_status()
# Returns: health_check_enabled, sensor_subscriptions_active, health_timer_active, sensor_status
```

### Sensor Status
- **UWB Status**: `sensor_processor.get_uwb_status()` - freshness and timing
- **Odometry Status**: `sensor_processor.get_odom_status()` - message count and timing  
- **Point Cloud**: Tracked via `last_pc_time` timestamp

### Safety Conditions
The `check_safety_conditions()` method provides centralized safety validation, returning `True` when safe to navigate or `False` when sensors indicate unsafe conditions.

## Key Methods

### Core Control Methods
- `asr_callback(asr_msg)` - Process ASR commands with health validation
- `text_callback(text_msg)` - Handle direct text commands
- `check_safety_conditions()` - Centralized safety validation

### Health Monitoring Methods
- `check_sensor_health()` - Periodic sensor health assessment
- `is_health_check_enabled()` - Query health check status
- `get_health_check_status()` - Comprehensive health information
- `set_health_check_enabled(enabled)` - Runtime health check toggle

### Sensor Callbacks
- `uwb_callback(uwb_msg)` - UWB sensor data processing
- `pc_callback(point_cloud_msg)` - Point cloud data processing  
- `odom_callback(odom_msg)` - Odometry data processing

## Configuration Parameters

- `health_check_enabled` (bool, default: True) - Enable/disable health monitoring
- `uwb_timeout` (double, default: 30.0) - UWB data timeout in seconds
- `pc_timeout` (double, default: 30.0) - Point cloud timeout in seconds
- `odom_timeout` (double, default: 5.0) - Odometry timeout in seconds
- `movement_distance_threshold` (double, default: 1.0) - Movement completion threshold
- `topic_*` parameters for configuring topic names

## Usage Example

```python
# Initialize node with health check enabled (default)
node = TaskLogicNode()

# Check health status
if node.is_health_check_enabled():
    status = node.get_health_check_status()
    print(f"Health check active: {status['health_check_enabled']}")

# Disable health check at runtime for testing
node.set_health_check_enabled(False)
```

## Safety Features

- **Sensor Timeout Detection**: Monitors UWB, point cloud, and odometry data freshness
- **Conditional Command Processing**: ASR commands blocked when sensors are unhealthy
- **Graceful Degradation**: Odometry timeout warns but doesn't stop navigation
- **Bypass Mode**: Health check can be disabled for testing/development


