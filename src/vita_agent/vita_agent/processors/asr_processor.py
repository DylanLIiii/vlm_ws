"""
ASR Processor for VITA Agent Package.

This processor handles command mapping between ASR output and control commands.
It provides a utility class for converting ASR commands to standardized control strings
without handling ROS topic publishing/subscribing.

Author: VITA Agent Package
"""

from typing import Optional, Dict, List
from rclpy.node import Node


class ASRProcessor:
    """
    Processor class that handles command mapping between ASR output and control commands.

    This class provides command mapping functionality without handling ROS communication.
    It can be used by other nodes to convert ASR commands to standardized control strings.
    """

    def __init__(self, node: Node):
        """
        Initialize the ASR processor.

        Args:
            node (Node): ROS2 node instance for logging
        """
        self.node = node
        self.logger = node.get_logger()

        # Define command mappings as specified in requirements
        self.command_mappings = {
            'move_to_master': 'Come Here',
            'move_to_master_front': 'Come to my front',
            'move_to_master_behind': 'Come to my behind/back',
            'move_to_master_left': 'Come to my left',
            'move_to_master_right': 'Come to my right',
            'follow_start': 'follow_start',
            'follow_stop': 'follow_stop',
            'stand_up': 'stand_up',
            'stand_down': 'stand_down',
            'shake_hand': 'shake_hand'
        }

        self.logger.info(f"ASRProcessor initialized with {len(self.command_mappings)} command mappings")
        self.logger.debug(f"Available commands: {list(self.command_mappings.keys())}")

    def map_command(self, asr_command: str) -> Optional[str]:
        """
        Map an ASR command to a standardized control string.

        Args:
            asr_command (str): The raw ASR command to map

        Returns:
            Optional[str]: The mapped control command, or None if command is not recognized
        """
        if not isinstance(asr_command, str):
            self.logger.error(f"Invalid input type for ASR command: {type(asr_command)}")
            return None

        # Clean the input command
        cleaned_command = asr_command.strip()

        if not cleaned_command:
            self.logger.warn("Empty ASR command received")
            return None

        # Check if command exists in mappings
        if cleaned_command in self.command_mappings:
            mapped_command = self.command_mappings[cleaned_command]
            self.logger.info(f"ASR command mapped: '{cleaned_command}' -> '{mapped_command}'")
            return mapped_command
        else:
            self.logger.warn(f"Unrecognized ASR command: '{cleaned_command}'")
            self.logger.debug(f"Available commands: {list(self.command_mappings.keys())}")
            return None

    def get_available_commands(self) -> List[str]:
        """
        Get a list of all available ASR commands.

        Returns:
            List[str]: List of available ASR command keys
        """
        return list(self.command_mappings.keys())

    def get_command_mappings(self) -> Dict[str, str]:
        """
        Get the complete command mappings dictionary.

        Returns:
            Dict[str, str]: Dictionary of ASR command to control command mappings
        """
        return self.command_mappings.copy()

    def is_valid_command(self, asr_command: str) -> bool:
        """
        Check if an ASR command is valid (exists in mappings).

        Args:
            asr_command (str): The ASR command to validate

        Returns:
            bool: True if command is valid, False otherwise
        """
        if not isinstance(asr_command, str):
            return False
        return asr_command.strip() in self.command_mappings

    def get_command_categories(self) -> Dict[str, List[str]]:
        """
        Get commands organized by categories for easier management.

        Returns:
            Dict[str, List[str]]: Dictionary with command categories
        """
        categories = {
            'movement': [
                'move_to_master',
                'move_to_master_front',
                'move_to_master_behind',
                'move_to_master_left',
                'move_to_master_right'
            ],
            'follow': [
                'follow_start',
                'follow_stop'
            ],
            'posture': [
                'stand_up',
                'stand_down'
            ],
            'interaction': [
                'shake_hand'
            ]
        }
        return categories