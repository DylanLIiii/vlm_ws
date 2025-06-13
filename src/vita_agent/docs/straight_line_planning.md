# Straight-Line Path Planning Feature Configuration

## Overview

The vita_agent now supports a straight-line path planning feature that bypasses obstacle avoidance for direct navigation to targets. This is useful for:

- Simple environments with known obstacle-free paths
- Testing and debugging without complex path planning
- Emergency scenarios where direct movement is required
- Environments where the robot's sensors or path planning might be unreliable

## Configuration

The feature is controlled by a single ROS parameter:

### Parameter: `use_straight_line_planning`
- **Type**: `bool`
- **Default**: `false`
- **Description**: Enable/disable straight-line path planning instead of obstacle avoidance

## Usage Examples

### 1. Enable via ROS parameter at launch time

```bash
ros2 run vita_agent zero_shot_vlm_planner --ros-args -p use_straight_line_planning:=true
```

### 2. Enable via parameter file

Create a parameter file `straight_line_config.yaml`:

```yaml
zero_shot_vlm_planner:
  ros__parameters:
    use_straight_line_planning: true
    max_linear_speed: 0.8
    max_angular_speed: 0.5
    stop_distance: 1.0
```

Then launch with:

```bash
ros2 run vita_agent zero_shot_vlm_planner --ros-args --params-file straight_line_config.yaml
```

### 3. Enable at runtime

```bash
# Enable straight-line planning
ros2 param set /task_logic_node use_straight_line_planning true

# Disable straight-line planning (return to obstacle avoidance)
ros2 param set /task_logic_node use_straight_line_planning false
```

## Behavior Differences

### With Straight-Line Planning (use_straight_line_planning: true)
- Robot moves directly toward target using simple P-controller
- No obstacle detection or avoidance
- Faster response and simpler trajectory
- Point cloud data is still processed but not used for path planning
- Speed limits and safety distance checks still apply

### With Obstacle Avoidance (use_straight_line_planning: false) - Default
- Full obstacle avoidance using point cloud data
- Complex trajectory generation with multiple path candidates
- Safer but potentially slower navigation
- All original functionality preserved

## Safety Considerations

⚠️ **Important Safety Notes:**

1. **Use straight-line planning only in safe, obstacle-free environments**
2. **The robot will NOT avoid obstacles when this feature is enabled**
3. **Maintain visual supervision when using straight-line mode**
4. **Test in controlled environments before deployment**
5. **Speed limits and stop distance still apply for basic safety**

## Technical Details

### Implementation
- New method `plan_straight_line_trajectory()` in `PathPlanner` class
- Uses existing `PathFinderController` for consistent velocity control
- Respects all speed and distance parameters
- Non-intrusive: original code unchanged
- Zero performance impact when disabled

### Code Flow
1. `TaskLogicNode.pc_callback()` calls `path_planner.plan_trajectory()`
2. `PathPlanner.plan_trajectory()` checks `use_straight_line_planning` parameter
3. If enabled: calls `plan_straight_line_trajectory()` (new)
4. If disabled: calls original `plan_path()` function (existing)
5. Returns same interface: `(v_traj, w_traj, stop_flag)`

## Testing

Run the test script to verify functionality:

```bash
cd /home/heng.li/repo/vlm_ws/src/vita_agent
python3 test_straight_line_planning.py
```

## Troubleshooting

### Issue: Parameter not recognized
**Solution**: Ensure the parameter is declared in `TaskLogicNode.__init__()`

### Issue: Robot still avoiding obstacles
**Solution**: Verify parameter is set correctly:
```bash
ros2 param get /task_logic_node use_straight_line_planning
```

### Issue: Robot moving too fast/slow
**Solution**: Adjust speed parameters:
```bash
ros2 param set /task_logic_node max_linear_speed 0.5
ros2 param set /task_logic_node max_angular_speed 0.3
```

## Migration Path

This feature is fully backwards compatible:
- Default behavior unchanged
- Existing launch files continue to work
- No code changes required for current deployments
- Can be enabled/disabled without restart (parameter change only)
