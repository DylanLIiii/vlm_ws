#!/usr/bin/env python3
"""
Trigger System Demo Script

This script demonstrates the complete ASR trigger system including:
- ASR command mapping via ASRProcessor
- Action execution via ActionExecutor
- Joy message publishing for robot control
- String message publishing for following commands

Usage:
    python3 trigger_system_demo.py
"""

import rclpy
from control_node import TaskLogicNode
from std_msgs.msg import String


def main():
    """Main demo function"""
    print("ASR Trigger System Demo")
    print("=" * 60)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create control node with integrated trigger system
        node = TaskLogicNode()
        
        print(f"\nTrigger System Components Initialized:")
        print(f"  - ASRProcessor: {len(node.asr_processor.get_available_commands())} commands")
        print(f"  - ActionExecutor: {len(node.action_executor.joy_commands)} Joy commands, {len(node.action_executor.following_commands)} following commands")
        
        # Demonstrate all ASR command types
        demo_commands = [
            # Robot control commands (Joy messages)
            ('stand_up', 'Robot Control'),
            ('stand_down', 'Robot Control'),
            ('shake_hand', 'Robot Control'),
            
            # Following commands (String messages)
            ('follow_start', 'Following Control'),
            ('follow_stop', 'Following Control'),
            
            # Movement commands (Task Logic)
            ('move_to_master', 'Movement'),
            ('move_to_master_front', 'Movement'),
            ('move_to_master_left', 'Movement'),
            
            # Invalid command
            ('invalid_command', 'Invalid')
        ]
        
        print(f"\nDemonstrating Complete ASR Trigger Workflow:")
        print("-" * 60)
        
        for asr_command, category in demo_commands:
            print(f"\n[{category}] Testing ASR command: '{asr_command}'")
            
            # Create ASR message
            asr_msg = String()
            asr_msg.data = asr_command
            
            # Process through complete trigger system
            print(f"  Input: ASR command '{asr_command}'")
            
            # Show mapping
            mapped = node.asr_processor.map_command(asr_command)
            if mapped:
                print(f"  Mapped: '{asr_command}' -> '{mapped}'")
            else:
                print(f"  Mapping: FAILED (unrecognized command)")
                continue
            
            # Execute via callback (simulates real ASR input)
            old_text_data = node.text_data
            node.asr_callback(asr_msg)
            
            # Show results
            print(f"  Text Data: {old_text_data} -> {node.text_data}")
            
            # Show what type of action was executed
            if mapped in node.action_executor.joy_commands:
                joy_config = node.action_executor.joy_commands[mapped]
                print(f"  Action: Published Joy message to /joy")
                print(f"    Buttons: {joy_config['buttons']}")
                print(f"    Axes: {joy_config['axes']}")
            elif mapped in node.action_executor.following_commands:
                follow_msg = node.action_executor.following_commands[mapped]
                print(f"  Action: Published String message to /following/task_type")
                print(f"    Data: '{follow_msg}'")
            else:
                print(f"  Action: Stored for task logic processing")
        
        # Demonstrate special RL mode functionality
        print(f"\n[Special] Testing RL Mode Activation:")
        print("-" * 60)
        result = node.action_executor.enter_rl_mode()
        print(f"  RL Mode Activation: {'SUCCESS' if result else 'FAILED'}")
        if result:
            rl_config = node.action_executor.joy_commands['enter_rl_mode']
            print(f"  Published Joy message to /joy:")
            print(f"    Buttons: {rl_config['buttons']}")
            print(f"    Axes: {rl_config['axes']}")
        
        # Show system status
        print(f"\nSystem Status Summary:")
        print("-" * 60)
        print(f"  Current text_data: '{node.text_data}'")
        print(f"  ASR Processor Status: Ready")
        print(f"  Action Executor Status: Ready")
        print(f"  Publishers Active: /joy, /following/task_type")
        
        print(f"\nDemo completed successfully!")
        print("The trigger system is ready to process real ASR commands.")
        
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
