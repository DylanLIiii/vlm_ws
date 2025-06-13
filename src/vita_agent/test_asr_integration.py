#!/usr/bin/env python3
"""
Test script to demonstrate ASR integration with the VLM planner.
This script shows how ASR commands are processed and converted to VLM prompts.
"""

import sys
import os
from unittest.mock import Mock

# Add the vita_agent module to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vita_agent.processors.asr_processor import ASRProcessor


def test_asr_integration():
    """Test the ASR integration functionality."""
    
    print("=" * 70)
    print("ASR Integration Test - VLM Planner Integration")
    print("=" * 70)
    
    # Create a mock node for testing
    mock_node = Mock()
    mock_logger = Mock()
    mock_node.get_logger.return_value = mock_logger
    
    # Create ASR processor instance
    asr_processor = ASRProcessor(mock_node)
    
    print("\n1. Available ASR Commands and their VLM Prompts:")
    print("-" * 52)
    commands = asr_processor.get_available_commands()
    for command, vlm_prompt in commands.items():
        print(f"  ASR Command: {command:<25} -> VLM Prompt: '{vlm_prompt}'")
    
    print(f"\nTotal ASR commands: {asr_processor.get_command_count()}")
    
    print("\n2. Simulating Text Callback Processing:")
    print("-" * 44)
    
    def simulate_text_callback_logic(input_text):
        """Simulate the logic from the updated text_callback method."""
        print(f"\nReceived input: '{input_text}'")
        
        # Check if the input is an ASR command
        if asr_processor.is_asr_command(input_text):
            # Process ASR command and get VLM prompt
            vlm_prompt = asr_processor.process_asr_command(input_text)
            if vlm_prompt:
                print(f"  ✓ Identified as ASR command")
                print(f"  ✓ VLM prompt: '{vlm_prompt}'")
                print(f"  ✓ Would set text_data = '{vlm_prompt}'")
                print(f"  ✓ Would transition to WAITING_FOR_VLM state")
                return vlm_prompt
            else:
                print(f"  ✗ Failed to process ASR command")
                return None
        else:
            # Process as general ASR result
            processed_text = asr_processor.process_asr_result(input_text)
            print(f"  ✓ Identified as ASR result")
            print(f"  ✓ Processed text: '{processed_text}'")
            print(f"  ✓ Would set text_data = '{processed_text}'")
            print(f"  ✓ Would transition to WAITING_FOR_VLM state")
            return processed_text
    
    # Test various inputs to simulate real scenarios
    test_inputs = [
        # ASR Commands (should be converted to VLM prompts)
        "move_to_master",
        "move_to_master_front", 
        "move_to_master_behind",
        "MOVE_TO_MASTER_LEFT",  # Test case insensitive
        "  move_to_master_right  ",  # Test with whitespace
        
        # General ASR results (should be processed as-is)
        "come to me please",
        "go to the kitchen",
        "help me with this task",
        "navigate to the door",
        
        # Invalid/Unknown commands
        "move_to_somewhere_unknown",
        "invalid_command"
    ]
    
    for input_text in test_inputs:
        simulate_text_callback_logic(input_text)
    
    print("\n3. State Machine Integration Summary:")
    print("-" * 43)
    print("The updated text_callback in TaskLogicNode now:")
    print("  • Checks if input is a predefined ASR command")
    print("  • Converts ASR commands to appropriate VLM prompts")
    print("  • Processes general speech as ASR results")
    print("  • Always transitions to WAITING_FOR_VLM state when text is received")
    print("  • Provides better logging for debugging")
    
    print("\n4. Expected VLM Behavior:")
    print("-" * 30)
    print("When VLM receives these prompts, it should:")
    print("  • 'come here' -> Navigate to user's current location")
    print("  • 'come here to my front' -> Navigate to front of user")
    print("  • 'come here to my behind' -> Navigate behind user")
    print("  • 'come here to my left' -> Navigate to user's left side")
    print("  • 'come here to my right' -> Navigate to user's right side")
    
    print("\n" + "=" * 70)
    print("ASR Integration Test Complete!")
    print("The VLM planner is now ready to handle ASR commands!")
    print("=" * 70)


if __name__ == "__main__":
    test_asr_integration()
