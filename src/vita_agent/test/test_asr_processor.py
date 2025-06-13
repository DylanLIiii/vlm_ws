#!/usr/bin/env python3

import unittest
from unittest.mock import Mock, patch
import sys
import os

# Add the vita_agent module to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from vita_agent.processors.asr_processor import ASRProcessor


class TestASRProcessor(unittest.TestCase):
    """Test cases for ASRProcessor class."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create mock node
        self.mock_node = Mock()
        self.mock_logger = Mock()
        self.mock_node.get_logger.return_value = self.mock_logger
        
        # Create ASR processor instance
        self.asr_processor = ASRProcessor(self.mock_node)
    
    def test_initialization(self):
        """Test ASRProcessor initialization."""
        # Check that the processor was initialized with expected commands
        expected_commands = {
            'move_to_master': 'come here',
            'move_to_master_front': 'come here to my front',
            'move_to_master_behind': 'come here to my behind',
            'move_to_master_left': 'come here to my left',
            'move_to_master_right': 'come here to my right'
        }
        
        self.assertEqual(self.asr_processor.get_available_commands(), expected_commands)
        self.assertEqual(self.asr_processor.get_command_count(), 5)
        
        # Check that logger was called during initialization
        self.mock_logger.info.assert_called()
    
    def test_is_asr_command_valid_commands(self):
        """Test is_asr_command with valid commands."""
        # Test exact matches
        self.assertTrue(self.asr_processor.is_asr_command('move_to_master'))
        self.assertTrue(self.asr_processor.is_asr_command('move_to_master_front'))
        self.assertTrue(self.asr_processor.is_asr_command('move_to_master_behind'))
        self.assertTrue(self.asr_processor.is_asr_command('move_to_master_left'))
        self.assertTrue(self.asr_processor.is_asr_command('move_to_master_right'))
        
        # Test case insensitive matching
        self.assertTrue(self.asr_processor.is_asr_command('MOVE_TO_MASTER'))
        self.assertTrue(self.asr_processor.is_asr_command('Move_To_Master_Front'))
        
        # Test with whitespace
        self.assertTrue(self.asr_processor.is_asr_command('  move_to_master  '))
    
    def test_is_asr_command_invalid_commands(self):
        """Test is_asr_command with invalid commands."""
        # Test non-existent commands
        self.assertFalse(self.asr_processor.is_asr_command('invalid_command'))
        self.assertFalse(self.asr_processor.is_asr_command('move_to_somewhere'))
        
        # Test empty/None input
        self.assertFalse(self.asr_processor.is_asr_command(''))
        self.assertFalse(self.asr_processor.is_asr_command(None))
        
        # Test partial matches (should not match)
        self.assertFalse(self.asr_processor.is_asr_command('move_to'))
        self.assertFalse(self.asr_processor.is_asr_command('master'))
    
    def test_process_asr_command_valid_commands(self):
        """Test process_asr_command with valid commands."""
        # Test exact command processing
        result = self.asr_processor.process_asr_command('move_to_master')
        self.assertEqual(result, 'come here')
        
        result = self.asr_processor.process_asr_command('move_to_master_front')
        self.assertEqual(result, 'come here to my front')
        
        result = self.asr_processor.process_asr_command('move_to_master_behind')
        self.assertEqual(result, 'come here to my behind')
        
        result = self.asr_processor.process_asr_command('move_to_master_left')
        self.assertEqual(result, 'come here to my left')
        
        result = self.asr_processor.process_asr_command('move_to_master_right')
        self.assertEqual(result, 'come here to my right')
        
        # Test case insensitive processing
        result = self.asr_processor.process_asr_command('MOVE_TO_MASTER')
        self.assertEqual(result, 'come here')
        
        # Test with whitespace
        result = self.asr_processor.process_asr_command('  move_to_master_left  ')
        self.assertEqual(result, 'come here to my left')
    
    def test_process_asr_command_invalid_commands(self):
        """Test process_asr_command with invalid commands."""
        # Test non-existent commands
        result = self.asr_processor.process_asr_command('invalid_command')
        self.assertIsNone(result)
        
        # Test empty/None input
        result = self.asr_processor.process_asr_command('')
        self.assertIsNone(result)
        
        result = self.asr_processor.process_asr_command(None)
        self.assertIsNone(result)
    
    def test_process_asr_result(self):
        """Test process_asr_result method."""
        # Test normal ASR result processing
        result = self.asr_processor.process_asr_result('hello world')
        self.assertEqual(result, 'hello world')
        
        # Test with whitespace
        result = self.asr_processor.process_asr_result('  hello world  ')
        self.assertEqual(result, 'hello world')
        
        # Test empty input
        result = self.asr_processor.process_asr_result('')
        self.assertEqual(result, '')
        
        result = self.asr_processor.process_asr_result(None)
        self.assertEqual(result, '')
    
    def test_add_asr_command(self):
        """Test adding new ASR commands."""
        # Test adding a new command
        success = self.asr_processor.add_asr_command('move_to_master_up', 'come here above me')
        self.assertTrue(success)
        self.assertEqual(self.asr_processor.get_command_count(), 6)
        
        # Test the new command works
        result = self.asr_processor.process_asr_command('move_to_master_up')
        self.assertEqual(result, 'come here above me')
        
        # Test adding duplicate command
        success = self.asr_processor.add_asr_command('move_to_master', 'duplicate')
        self.assertFalse(success)  # Should fail due to duplicate
        self.assertEqual(self.asr_processor.get_command_count(), 6)  # Count shouldn't change
        
        # Test adding empty command
        success = self.asr_processor.add_asr_command('', 'empty command')
        self.assertFalse(success)
        
        success = self.asr_processor.add_asr_command('valid_command', '')
        self.assertFalse(success)
    
    def test_remove_asr_command(self):
        """Test removing ASR commands."""
        # Test removing existing command
        initial_count = self.asr_processor.get_command_count()
        success = self.asr_processor.remove_asr_command('move_to_master')
        self.assertTrue(success)
        self.assertEqual(self.asr_processor.get_command_count(), initial_count - 1)
        
        # Test that removed command no longer works
        self.assertFalse(self.asr_processor.is_asr_command('move_to_master'))
        result = self.asr_processor.process_asr_command('move_to_master')
        self.assertIsNone(result)
        
        # Test removing non-existent command
        success = self.asr_processor.remove_asr_command('non_existent_command')
        self.assertFalse(success)
        
        # Test removing empty command
        success = self.asr_processor.remove_asr_command('')
        self.assertFalse(success)
        
        # Test case insensitive removal
        success = self.asr_processor.remove_asr_command('MOVE_TO_MASTER_FRONT')
        self.assertTrue(success)
    
    def test_get_available_commands(self):
        """Test getting available commands."""
        commands = self.asr_processor.get_available_commands()
        
        # Should return a copy, not the original
        self.assertIsNot(commands, self.asr_processor.asr_command_map)
        
        # Should have the expected commands
        expected_commands = {
            'move_to_master': 'come here',
            'move_to_master_front': 'come here to my front',
            'move_to_master_behind': 'come here to my behind',
            'move_to_master_left': 'come here to my left',
            'move_to_master_right': 'come here to my right'
        }
        self.assertEqual(commands, expected_commands)


if __name__ == '__main__':
    unittest.main()
