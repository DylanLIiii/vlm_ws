#!/usr/bin/env python3
"""
Demonstration script for the ASR Processor functionality.
This script shows how to use the ASR processor to handle ASR commands and results.
"""

import sys
import os
from unittest.mock import Mock

# Add the vita_agent module to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vita_agent.processors.asr_processor import ASRProcessor


def demonstrate_asr_processor():
    """Demonstrate the ASR processor functionality."""
    
    print("=" * 60)
    print("ASR Processor Demonstration")
    print("=" * 60)
    
    # Create a mock node for the demonstration
    mock_node = Mock()
    mock_logger = Mock()
    mock_node.get_logger.return_value = mock_logger
    
    # Create ASR processor instance
    asr_processor = ASRProcessor(mock_node)
    
    print("\n1. Available ASR Commands:")
    print("-" * 30)
    commands = asr_processor.get_available_commands()
    for command, vlm_prompt in commands.items():
        print(f"  {command:<25} -> {vlm_prompt}")
    
    print(f"\nTotal commands: {asr_processor.get_command_count()}")
    
    print("\n2. Testing ASR Command Recognition:")
    print("-" * 40)
    test_inputs = [
        "move_to_master",
        "move_to_master_right", 
        "MOVE_TO_MASTER_LEFT",
        "  move_to_master_front  ",
        "invalid_command",
        "move_to_somewhere",
        ""
    ]
    
    for test_input in test_inputs:
        is_command = asr_processor.is_asr_command(test_input)
        print(f"  '{test_input}' -> {'ASR Command' if is_command else 'Not ASR Command'}")
    
    print("\n3. Testing ASR Command Processing:")
    print("-" * 42)
    valid_commands = [
        "move_to_master",
        "move_to_master_front",
        "move_to_master_behind", 
        "move_to_master_left",
        "move_to_master_right"
    ]
    
    for command in valid_commands:
        vlm_prompt = asr_processor.process_asr_command(command)
        print(f"  '{command}' -> '{vlm_prompt}'")
    
    print("\n4. Testing Invalid ASR Commands:")
    print("-" * 38)
    invalid_commands = ["invalid_command", "move_to_somewhere", ""]
    
    for command in invalid_commands:
        vlm_prompt = asr_processor.process_asr_command(command)
        print(f"  '{command}' -> {vlm_prompt if vlm_prompt else 'None (invalid)'}")
    
    print("\n5. Testing ASR Result Processing (Placeholder):")
    print("-" * 52)
    asr_results = [
        "Hello, how are you?",
        "Please move forward",
        "  I need help  ",
        ""
    ]
    
    for asr_result in asr_results:
        processed = asr_processor.process_asr_result(asr_result)
        print(f"  '{asr_result}' -> '{processed}'")
    
    print("\n6. Testing Dynamic Command Management:")
    print("-" * 44)
    
    # Add a new command
    print("Adding new command: 'move_to_master_down' -> 'come here below me'")
    success = asr_processor.add_asr_command('move_to_master_down', 'come here below me')
    print(f"  Add success: {success}")
    print(f"  New command count: {asr_processor.get_command_count()}")
    
    # Test the new command
    vlm_prompt = asr_processor.process_asr_command('move_to_master_down')
    print(f"  Testing new command: 'move_to_master_down' -> '{vlm_prompt}'")
    
    # Try to add duplicate command
    print("\nTrying to add duplicate command: 'move_to_master' -> 'duplicate'")
    success = asr_processor.add_asr_command('move_to_master', 'duplicate')
    print(f"  Add success: {success} (should be False)")
    
    # Remove a command
    print("\nRemoving command: 'move_to_master_down'")
    success = asr_processor.remove_asr_command('move_to_master_down')
    print(f"  Remove success: {success}")
    print(f"  Final command count: {asr_processor.get_command_count()}")
    
    print("\n7. Integration Example - Simulating Text Callback:")
    print("-" * 56)
    
    def simulate_text_callback(input_text):
        """Simulate how the text callback would use the ASR processor."""
        print(f"\nReceived input: '{input_text}'")
        
        if asr_processor.is_asr_command(input_text):
            vlm_prompt = asr_processor.process_asr_command(input_text)
            if vlm_prompt:
                print(f"  -> Identified as ASR command")
                print(f"  -> VLM prompt: '{vlm_prompt}'")
                print(f"  -> Would transition to WAITING_FOR_VLM state")
            else:
                print(f"  -> Failed to process ASR command")
        else:
            processed_text = asr_processor.process_asr_result(input_text)
            print(f"  -> Identified as ASR result")
            print(f"  -> Processed text: '{processed_text}'")
            print(f"  -> Would transition to WAITING_FOR_VLM state")
    
    # Simulate various inputs
    simulation_inputs = [
        "move_to_master_right",
        "Please come to my location",
        "MOVE_TO_MASTER_FRONT",
        "Can you help me with something?",
        "invalid_command"
    ]
    
    for sim_input in simulation_inputs:
        simulate_text_callback(sim_input)
    
    print("\n" + "=" * 60)
    print("ASR Processor Demonstration Complete!")
    print("=" * 60)


if __name__ == "__main__":
    demonstrate_asr_processor()
