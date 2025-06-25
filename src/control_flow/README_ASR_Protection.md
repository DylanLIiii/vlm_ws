# ASR Protection Feature

## Overview

The ASR (Automatic Speech Recognition) Protection feature is a command buffering and deduplication system implemented in the `TaskLogicNode` to handle rapid or duplicate ASR commands more intelligently. This feature helps prevent command flooding and ensures that only unique commands are executed in a controlled manner.

## Features

### 1. **Buffering Window**
- When the first ASR command is received after a period of inactivity, a 1-second buffering window is started
- All subsequent ASR commands received within this window are collected in a buffer
- Commands are executed sequentially only after the buffering window expires

### 2. **Dynamic Deduplication**
- Duplicate ASR commands are automatically removed from the buffer using a set-like behavior
- Only unique commands are retained, preserving the order of first occurrence
- Deduplication happens in real-time as commands are received

### 3. **Sequential Execution**
- After the buffering window expires, all unique commands are executed in the order they were originally received
- Each command goes through the normal ASR processing pipeline (mapping and action execution)

### 4. **Inactivity Detection**
- The system tracks the last ASR activity time
- If no ASR commands are received for 1 second, the next command will trigger a new buffering window
- This ensures that commands separated by significant time gaps are processed in separate batches

### 5. **Runtime Configuration**
- The feature can be enabled/disabled via the `asr_protection_enabled` ROS parameter
- When disabled, ASR commands are processed immediately as they were before
- The feature can also be toggled at runtime using the provided API methods

## Configuration

### ROS Parameters

```yaml
# Enable/disable ASR Protection (default: false for backward compatibility)
asr_protection_enabled: true
```

### Timing Parameters

The following timing parameters are currently hardcoded but can be made configurable if needed:

- **Buffering Window**: 1.0 seconds
- **Inactivity Threshold**: 1.0 seconds

## API Methods

### Status and Control Methods

```python
# Check if ASR protection is enabled
is_enabled = node.is_asr_protection_enabled()

# Enable/disable ASR protection at runtime
node.set_asr_protection_enabled(True)  # Enable
node.set_asr_protection_enabled(False)  # Disable

# Get comprehensive status information
status = node.get_asr_protection_status()
```

### Status Information

The `get_asr_protection_status()` method returns a dictionary with the following information:

```python
{
    'asr_protection_enabled': bool,      # Whether protection is enabled
    'buffering_window': float,           # Buffering window duration (seconds)
    'inactivity_threshold': float,       # Inactivity threshold (seconds)
    'buffer_active': bool,               # Whether buffering is currently active
    'buffered_commands_count': int,      # Number of commands in buffer
    'buffered_commands': list,           # List of buffered command strings
    'last_activity_time': float          # Timestamp of last ASR activity
}
```

## Usage Examples

### Example 1: Rapid Commands with Duplicates

**Input sequence (within 1 second):**
```
move_to_master
follow_start
move_to_master  # Duplicate
action_stand
follow_start    # Duplicate
```

**With ASR Protection ENABLED:**
- All commands are buffered for 1 second
- Duplicates are removed
- Final execution: `move_to_master`, `follow_start`, `action_stand`

**With ASR Protection DISABLED:**
- Each command is processed immediately
- All 5 commands are executed, including duplicates

### Example 2: Commands Separated by Inactivity

```
move_to_master
follow_start
[1.5 second gap - inactivity]
action_sit
follow_stop
```

**Result:**
- First batch: `move_to_master`, `follow_start` (executed after 1s)
- Second batch: `action_sit`, `follow_stop` (executed after another 1s)

## Implementation Details

### Thread Safety
- All buffer operations are protected by a threading lock (`asr_buffer_lock`)
- The buffering timer runs in a separate thread
- Command execution happens in the main thread context

### Data Structures
- Uses `collections.OrderedDict` to maintain insertion order while providing set-like deduplication
- Commands are stored with their reception timestamps for potential future use

### Integration with Existing Systems
- The feature is fully backward compatible when disabled
- Integrates seamlessly with existing health check and action execution systems
- Does not interfere with other node operations

## Logging

The feature provides comprehensive logging for debugging and monitoring:

```
[INFO] ASR Protection enabled - buffering window: 1.0s, inactivity threshold: 1.0s
[INFO] Starting ASR buffering window - first command after inactivity: 'move_to_master'
[INFO] Added unique ASR command to buffer: 'follow_start' (buffer size: 2)
[INFO] Duplicate ASR command ignored: 'move_to_master' (already in buffer)
[INFO] ASR buffering window ended - executing 2 unique commands: ['move_to_master', 'follow_start']
[INFO] Executing buffered ASR command: 'move_to_master'
```

## Testing

### Unit Tests
Run the unit tests to verify the feature functionality:

```bash
cd src/control_flow/test
python3 test_asr_protection.py
```

### Demonstration Script
Run the demonstration script to see the feature in action:

```bash
cd src/control_flow/examples
python3 asr_protection_demo.py
```

## Benefits

1. **Reduces Command Flooding**: Prevents rapid duplicate commands from overwhelming the system
2. **Improves User Experience**: Eliminates unintended duplicate actions from speech recognition errors
3. **Maintains Responsiveness**: 1-second delay is minimal while providing effective protection
4. **Preserves Intent**: Sequential execution ensures user commands are processed in the intended order
5. **Backward Compatible**: Can be disabled for systems that don't need this protection

## Future Enhancements

Potential improvements that could be added:

1. **Configurable Timing**: Make buffering window and inactivity threshold configurable via ROS parameters
2. **Command Prioritization**: Allow certain critical commands to bypass buffering
3. **Adaptive Timing**: Adjust buffering window based on command frequency patterns
4. **Statistics**: Track and report buffering effectiveness metrics
5. **Command Grouping**: Group related commands for more intelligent batching
