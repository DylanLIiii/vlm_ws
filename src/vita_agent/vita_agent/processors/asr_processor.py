#!/usr/bin/env python3

from typing import Dict, Optional
from rclpy.node import Node


class ASRProcessor:
    """
    Processor for handling ASR (Automatic Speech Recognition) commands and results.
    
    This processor handles two types of input:
    1. ASR Commands: A finite set of predefined command strings that map to specific actions
    2. ASR Results: General speech recognition results (placeholder for future implementation)
    """
    
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        
        # Define the mapping from ASR commands to VLM prompts
        self.asr_command_map: Dict[str, str] = {
            'move_to_master': 'come here',
            'move_to_master_front': 'come here to my front',
            'move_to_master_behind': 'come here to my behind', 
            'move_to_master_left': 'come here to my left',
            'move_to_master_right': 'come here to my right'
        }
        
        self.logger.info(f"ASRProcessor initialized with {len(self.asr_command_map)} ASR commands")
        self.logger.info(f"Available ASR commands: {list(self.asr_command_map.keys())}")
    
    def is_asr_command(self, input_text: str) -> bool:
        """
        Check if the input text is a recognized ASR command.
        
        Args:
            input_text: The input text to check
            
        Returns:
            True if the input is an ASR command, False otherwise
        """
        if not input_text:
            return False
            
        # Strip whitespace and convert to lowercase for case-insensitive matching
        normalized_input = input_text.strip().lower()
        
        # Check if the normalized input matches any ASR command (case-insensitive)
        for command in self.asr_command_map.keys():
            if normalized_input == command.lower():
                return True
                
        return False
    
    def process_asr_command(self, asr_command: str) -> Optional[str]:
        """
        Process an ASR command and return the corresponding VLM prompt.
        
        Args:
            asr_command: The ASR command string
            
        Returns:
            The corresponding VLM prompt string, or None if command is not recognized
        """
        if not asr_command:
            self.logger.error("Empty ASR command provided")
            return None
            
        # Normalize the command for case-insensitive matching
        normalized_command = asr_command.strip().lower()
        
        # Find the matching command (case-insensitive)
        for command, vlm_prompt in self.asr_command_map.items():
            if normalized_command == command.lower():
                self.logger.info(f"ASR command '{asr_command}' mapped to VLM prompt: '{vlm_prompt}'")
                return vlm_prompt
        
        self.logger.warning(f"Unrecognized ASR command: '{asr_command}'")
        self.logger.info(f"Available commands: {list(self.asr_command_map.keys())}")
        return None
    
    def process_asr_result(self, asr_result: str) -> str:
        """
        Process a general ASR result (speech recognition output).
        
        This is a placeholder method for future implementation where general
        speech recognition results will be processed and potentially transformed
        before being sent to the VLM.
        
        Args:
            asr_result: The ASR result string
            
        Returns:
            The processed ASR result (currently returns input as-is)
        """
        if not asr_result:
            self.logger.warning("Empty ASR result provided")
            return ""
            
        # Placeholder implementation - currently just returns the input
        # In the future, this could involve:
        # - Text normalization
        # - Intent recognition
        # - Context-aware processing
        # - Language understanding
        
        processed_result = asr_result.strip()
        self.logger.info(f"ASR result processed: '{asr_result}' -> '{processed_result}'")
        
        return processed_result
    
    def add_asr_command(self, command: str, vlm_prompt: str) -> bool:
        """
        Add a new ASR command to the command map.
        
        Args:
            command: The ASR command string
            vlm_prompt: The corresponding VLM prompt
            
        Returns:
            True if command was added successfully, False if command already exists
        """
        if not command or not vlm_prompt:
            self.logger.error("Command and VLM prompt cannot be empty")
            return False
            
        command = command.strip()
        vlm_prompt = vlm_prompt.strip()
        
        if command.lower() in [cmd.lower() for cmd in self.asr_command_map.keys()]:
            self.logger.warning(f"ASR command '{command}' already exists")
            return False
            
        self.asr_command_map[command] = vlm_prompt
        self.logger.info(f"Added new ASR command: '{command}' -> '{vlm_prompt}'")
        return True
    
    def remove_asr_command(self, command: str) -> bool:
        """
        Remove an ASR command from the command map.
        
        Args:
            command: The ASR command string to remove
            
        Returns:
            True if command was removed successfully, False if command doesn't exist
        """
        if not command:
            self.logger.error("Command cannot be empty")
            return False
            
        # Find and remove the command (case-insensitive)
        command_to_remove = None
        for existing_command in self.asr_command_map.keys():
            if command.strip().lower() == existing_command.lower():
                command_to_remove = existing_command
                break
                
        if command_to_remove:
            removed_prompt = self.asr_command_map.pop(command_to_remove)
            self.logger.info(f"Removed ASR command: '{command_to_remove}' (was mapped to '{removed_prompt}')")
            return True
        else:
            self.logger.warning(f"ASR command '{command}' not found")
            return False
    
    def get_available_commands(self) -> Dict[str, str]:
        """
        Get a copy of all available ASR commands and their corresponding VLM prompts.
        
        Returns:
            Dictionary mapping ASR commands to VLM prompts
        """
        return self.asr_command_map.copy()
    
    def get_command_count(self) -> int:
        """
        Get the number of available ASR commands.
        
        Returns:
            Number of ASR commands in the command map
        """
        return len(self.asr_command_map)