import unittest
from unittest.mock import Mock, patch, MagicMock
import threading
import time
from vita_agent.text_publisher import TextPublisher

class TestTextPublisher(unittest.TestCase):
    
    def setUp(self):
        # We need to patch before creating the TextPublisher instance
        # since the constructor starts the input thread
        self.mock_rclpy_ok_patcher = patch('rclpy.ok')
        self.mock_rclpy_ok = self.mock_rclpy_ok_patcher.start()
        self.mock_rclpy_ok.return_value = True
        
        self.mock_threading_patcher = patch('threading.Thread')
        self.mock_threading = self.mock_threading_patcher.start()
        self.mock_thread = Mock()
        self.mock_threading.return_value = self.mock_thread
    
    def tearDown(self):
        self.mock_rclpy_ok_patcher.stop()
        self.mock_threading_patcher.stop()
    
    @patch('vita_agent.text_publisher.Node.__init__')
    def test_initialization(self, mock_node_init):
        mock_node_init.return_value = None
        
        with patch.object(TextPublisher, 'create_publisher') as mock_create_pub, \
             patch.object(TextPublisher, 'get_logger') as mock_logger:
            
            mock_publisher = Mock()
            mock_create_pub.return_value = mock_publisher
            mock_logger.return_value = Mock()
            
            text_publisher = TextPublisher()
            
            # Verify node initialization
            mock_node_init.assert_called_once_with('text_publisher')
            
            # Verify publisher creation - check the actual String message type
            from std_msgs.msg import String
            mock_create_pub.assert_called_once_with(String, '/test/command', 10)
            
            # Verify thread setup
            self.mock_threading.assert_called_once()
            self.mock_thread.start.assert_called_once()
            self.assertTrue(text_publisher.running)

    @patch('vita_agent.text_publisher.Node.__init__')
    def test_input_loop_publish_message(self, mock_node_init):
        mock_node_init.return_value = None
        
        with patch.object(TextPublisher, 'create_publisher') as mock_create_pub, \
             patch.object(TextPublisher, 'get_logger') as mock_logger, \
             patch('builtins.input', side_effect=['Test message', 'quit']) as mock_input:
            
            mock_publisher = Mock()
            mock_create_pub.return_value = mock_publisher
            mock_logger_instance = Mock()
            mock_logger.return_value = mock_logger_instance
            
            # Don't start the thread automatically
            with patch('threading.Thread') as mock_thread_class:
                text_publisher = TextPublisher()
                text_publisher.publisher = mock_publisher
                
                # Manually call input_loop to test it
                with patch('rclpy.shutdown') as mock_shutdown:
                    text_publisher.input_loop()
                    
                    # Verify message was published
                    self.assertEqual(mock_publisher.publish.call_count, 1)
                    published_msg = mock_publisher.publish.call_args[0][0]
                    self.assertEqual(published_msg.data, 'Test message')
                    
                    # Verify shutdown was called when 'quit' was entered
                    mock_shutdown.assert_called_once()
                    self.assertFalse(text_publisher.running)

    @patch('vita_agent.text_publisher.Node.__init__')
    def test_input_loop_exit_command(self, mock_node_init):
        mock_node_init.return_value = None
        
        with patch.object(TextPublisher, 'create_publisher') as mock_create_pub, \
             patch.object(TextPublisher, 'get_logger') as mock_logger, \
             patch('builtins.input', side_effect=['exit']) as mock_input:
            
            mock_publisher = Mock()
            mock_create_pub.return_value = mock_publisher
            mock_logger_instance = Mock()
            mock_logger.return_value = mock_logger_instance
            
            # Don't start the thread automatically
            with patch('threading.Thread') as mock_thread_class:
                text_publisher = TextPublisher()
                
                # Manually call input_loop to test it
                with patch('rclpy.shutdown') as mock_shutdown:
                    text_publisher.input_loop()
                    
                    # Verify no message was published
                    mock_publisher.publish.assert_not_called()
                    
                    # Verify shutdown was called when 'exit' was entered
                    mock_shutdown.assert_called_once()
                    self.assertFalse(text_publisher.running)

    @patch('vita_agent.text_publisher.Node.__init__')
    def test_input_loop_eof_error(self, mock_node_init):
        mock_node_init.return_value = None
        
        with patch.object(TextPublisher, 'create_publisher') as mock_create_pub, \
             patch.object(TextPublisher, 'get_logger') as mock_logger, \
             patch('builtins.input', side_effect=EOFError()) as mock_input:
            
            mock_publisher = Mock()
            mock_create_pub.return_value = mock_publisher
            mock_logger_instance = Mock()
            mock_logger.return_value = mock_logger_instance
            
            # Don't start the thread automatically
            with patch('threading.Thread') as mock_thread_class:
                text_publisher = TextPublisher()
                
                # Manually call input_loop to test it
                with patch('rclpy.shutdown') as mock_shutdown:
                    text_publisher.input_loop()
                    
                    # Verify shutdown was called on EOFError
                    mock_shutdown.assert_called_once()
                    self.assertFalse(text_publisher.running)

    @patch('vita_agent.text_publisher.Node.__init__')
    def test_input_loop_keyboard_interrupt(self, mock_node_init):
        mock_node_init.return_value = None
        
        with patch.object(TextPublisher, 'create_publisher') as mock_create_pub, \
             patch.object(TextPublisher, 'get_logger') as mock_logger, \
             patch('builtins.input', side_effect=KeyboardInterrupt()) as mock_input:
            
            mock_publisher = Mock()
            mock_create_pub.return_value = mock_publisher
            mock_logger_instance = Mock()
            mock_logger.return_value = mock_logger_instance
            
            # Don't start the thread automatically
            with patch('threading.Thread') as mock_thread_class:
                text_publisher = TextPublisher()
                
                # Manually call input_loop to test it
                with patch('rclpy.shutdown') as mock_shutdown:
                    text_publisher.input_loop()
                    
                    # Verify shutdown was called on KeyboardInterrupt
                    mock_shutdown.assert_called_once()
                    self.assertFalse(text_publisher.running)

    @patch('vita_agent.text_publisher.Node.__init__')
    def test_destroy_node(self, mock_node_init):
        mock_node_init.return_value = None
        
        with patch.object(TextPublisher, 'create_publisher') as mock_create_pub, \
             patch.object(TextPublisher, 'get_logger') as mock_logger:
            
            mock_publisher = Mock()
            mock_create_pub.return_value = mock_publisher
            mock_logger_instance = Mock()
            mock_logger.return_value = mock_logger_instance
            
            # Don't start the thread automatically
            with patch('threading.Thread') as mock_thread_class:
                text_publisher = TextPublisher()
                
                # Test destroy_node
                with patch('vita_agent.text_publisher.Node.destroy_node') as mock_super_destroy:
                    text_publisher.destroy_node()
                    
                    # Verify running flag is set to False
                    self.assertFalse(text_publisher.running)
                    # Verify parent destroy_node is called
                    mock_super_destroy.assert_called_once()

if __name__ == '__main__':
    unittest.main()