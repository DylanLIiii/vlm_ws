#!/usr/bin/env python3
"""
Enhanced Trigger System Demo Script

This script demonstrates the enhanced ASR trigger system with task status monitoring including:
- ASR command mapping via ASRProcessor
- Action execution via ActionExecutor
- Task status monitoring for movement commands
- Automatic following stop when movement tasks complete
- Joy message publishing for robot control
- String message publishing for following commands

Usage:
    python3 enhanced_trigger_system_demo.py
"""

import rclpy
import json
from control_node import TaskLogicNode
from std_msgs.msg import String


def main():
    """Main demo function"""
    print("Enhanced ASR Trigger System Demo")
    print("=" * 70)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create control node with enhanced trigger system
        node = TaskLogicNode()
        
        print(f"\nEnhanced Trigger System Components Initialized:")
        print(f"  - ASRProcessor: {len(node.asr_processor.get_available_commands())} commands")
        print(f"  - ActionExecutor: {len(node.action_executor.joy_commands)} Joy commands, {len(node.action_executor.following_commands)} following commands")
        print(f"  - Task Status Monitoring: Enabled on /task_status topic")
        print(f"  - Movement Commands Tracked: {list(node.action_executor.movement_commands)}")
        print(f"  - Distance Threshold: {node.action_executor.distance_threshold}m")
        
        # Demonstrate complete workflow with task status monitoring
        print(f"\nDemonstrating Complete Enhanced Workflow:")
        print("-" * 70)
        
        # Step 1: Execute a movement command
        print(f"\n[STEP 1] Executing Movement Command")
        asr_msg = String()
        asr_msg.data = 'move_to_master_left'
        print(f"  ASR Input: '{asr_msg.data}'")
        node.asr_callback(asr_msg)
        print(f"  Movement Tracking Active: {node.action_executor.is_movement_command_active()}")
        print(f"  Active Command: '{node.action_executor.get_active_movement_command()}'")
        print(f"  Text Data: '{node.text_data}'")
        
        # Step 2: Simulate task status updates during navigation
        print(f"\n[STEP 2] Simulating Task Status Updates During Navigation")
        
        # Status 1: Robot is navigating, far from target
        task_status_1 = {
            'current_distance_to_target': 4.2,
            'target_reached': False,
            'timestamp': '2024-01-01T12:00:00',
            'system_state': {'status': 'navigating', 'speed': 0.8}
        }
        status_msg_1 = String()
        status_msg_1.data = json.dumps(task_status_1)
        print(f"  Status Update 1: distance=4.2m, target_reached=False")
        node.action_executor.task_status_callback(status_msg_1)
        print(f"    Still tracking: {node.action_executor.is_movement_command_active()}")
        
        # Status 2: Robot is approaching, getting closer
        task_status_2 = {
            'current_distance_to_target': 2.1,
            'target_reached': False,
            'timestamp': '2024-01-01T12:00:30',
            'system_state': {'status': 'approaching', 'speed': 0.5}
        }
        status_msg_2 = String()
        status_msg_2.data = json.dumps(task_status_2)
        print(f"  Status Update 2: distance=2.1m, target_reached=False")
        node.action_executor.task_status_callback(status_msg_2)
        print(f"    Still tracking: {node.action_executor.is_movement_command_active()}")
        
        # Status 3: Robot reaches target (distance below threshold)
        task_status_3 = {
            'current_distance_to_target': 1.2,
            'target_reached': False,
            'timestamp': '2024-01-01T12:01:00',
            'system_state': {'status': 'arrived', 'speed': 0.0}
        }
        status_msg_3 = String()
        status_msg_3.data = json.dumps(task_status_3)
        print(f"  Status Update 3: distance=1.2m, target_reached=False")
        print(f"    (Distance below threshold of {node.action_executor.distance_threshold}m)")
        node.action_executor.task_status_callback(status_msg_3)
        print(f"    Task completed - tracking stopped: {not node.action_executor.is_movement_command_active()}")
        print(f"    Automatic follow stop command sent!")
        
        # Step 3: Demonstrate target_reached flag scenario
        print(f"\n[STEP 3] Demonstrating target_reached Flag Scenario")
        
        # Start another movement command
        asr_msg.data = 'move_to_master_right'
        print(f"  Starting new movement: '{asr_msg.data}'")
        node.asr_callback(asr_msg)
        print(f"  New tracking active: {node.action_executor.is_movement_command_active()}")
        
        # Complete via target_reached flag (even if distance is above threshold)
        task_status_4 = {
            'current_distance_to_target': 2.8,  # Above threshold
            'target_reached': True,             # But explicitly reached
            'timestamp': '2024-01-01T12:02:00',
            'system_state': {'status': 'target_reached', 'precision_mode': True}
        }
        status_msg_4 = String()
        status_msg_4.data = json.dumps(task_status_4)
        print(f"  Status Update: distance=2.8m, target_reached=True")
        print(f"    (Distance above threshold but target_reached=True)")
        node.action_executor.task_status_callback(status_msg_4)
        print(f"    Task completed via flag - tracking stopped: {not node.action_executor.is_movement_command_active()}")
        
        # Step 4: Demonstrate non-movement commands (no tracking)
        print(f"\n[STEP 4] Demonstrating Non-Movement Commands")
        
        non_movement_commands = ['stand_up', 'follow_start', 'shake_hand']
        for cmd in non_movement_commands:
            asr_msg.data = cmd
            print(f"  Executing: '{cmd}'")
            node.asr_callback(asr_msg)
            print(f"    Movement tracking: {node.action_executor.is_movement_command_active()}")
        
        # Step 5: Show system status and capabilities
        print(f"\n[STEP 5] System Status and Capabilities")
        print("-" * 70)
        print(f"  Latest Task Status: {node.action_executor.get_task_status()}")
        print(f"  Current Text Data: '{node.text_data}'")
        print(f"  Distance Threshold: {node.action_executor.distance_threshold}m")
        print(f"  Movement Commands: {list(node.action_executor.movement_commands)}")
        print(f"  Joy Commands: {list(node.action_executor.joy_commands.keys())}")
        print(f"  Following Commands: {list(node.action_executor.following_commands.keys())}")
        
        # Step 6: Test configuration
        print(f"\n[STEP 6] Testing Configuration")
        print(f"  Original threshold: {node.action_executor.distance_threshold}m")
        node.action_executor.set_distance_threshold(2.0)
        print(f"  Updated threshold: {node.action_executor.distance_threshold}m")
        
        print(f"\nDemo completed successfully!")
        print("The enhanced trigger system is ready to:")
        print("  ✓ Process ASR commands and execute appropriate actions")
        print("  ✓ Monitor task status for movement commands")
        print("  ✓ Automatically stop following when movement tasks complete")
        print("  ✓ Handle both distance-based and flag-based task completion")
        print("  ✓ Provide comprehensive error handling and logging")
        
    except Exception as e:
        print(f"Error during demo: {e}")
    finally:
        # Cleanup
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
