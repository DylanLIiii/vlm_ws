import unittest
from unittest.mock import MagicMock, patch, AsyncMock

import cv2
import httpx
import numpy as np
import pytest
from rclpy.node import Node

from vita_agent.clients.vlm_client import VlmClient


class TestVlmClient(unittest.TestCase):

    @pytest.fixture(autouse=True)
    def _(self, capsys):
        self.capsys = capsys

    def setUp(self):
        # Create a mock rclpy node
        self.mock_node = MagicMock(spec=Node)
        self.mock_node.get_logger.return_value = MagicMock()
        mock_param = MagicMock()
        mock_param.get_parameter_value.return_value.string_value = "http://fake-vlm-server.com/api"
        self.mock_node.get_parameter.return_value = mock_param

        # Initialize the VlmClient with the mock node
        self.vlm_client = VlmClient(self.mock_node)

    @pytest.mark.asyncio
    async def test_get_action_success(self):
        """Test Case 3.1: Mock a successful VLM server response."""
        mock_response = MagicMock(spec=httpx.Response)
        mock_response.status_code = 200
        mock_response.json.return_value = {"predicted_bev_coordinates": [1.0, 2.0]}
        mock_response.text = '{"predicted_bev_coordinates": [1.0, 2.0]}'

        with patch('httpx.AsyncClient') as mock_async_client:
            mock_async_client.return_value.__aenter__.return_value.post = AsyncMock(return_value=mock_response)
            
            image = np.zeros((100, 100, 3), dtype=np.uint8)
            command_text = "go there"
            uwb_target = np.array([0.5, -1.5, 0])
            
            result = await self.vlm_client.get_action(image, command_text, uwb_target)

            self.assertEqual(result, (2.0, -1.0))
            self.mock_node.get_logger().info.assert_called_with('Server response: 200 {"predicted_bev_coordinates": [1.0, 2.0]}')

    @pytest.mark.asyncio
    async def test_get_action_timeout(self):
        """Test Case 3.2: Mock a VLM server timeout."""
        with patch('httpx.AsyncClient') as mock_async_client:
            mock_async_client.return_value.__aenter__.return_value.post = AsyncMock(side_effect=httpx.TimeoutException("Timeout"))

            image = np.zeros((100, 100, 3), dtype=np.uint8)
            command_text = "go there"
            uwb_target = np.array([0.5, -1.5, 0])

            result = await self.vlm_client.get_action(image, command_text, uwb_target)

            self.assertIsNone(result)
            self.mock_node.get_logger().error.assert_called_with('Failed to post to VLM server: Timeout')

    @pytest.mark.asyncio
    async def test_get_action_server_error(self):
        """Test Case 3.3: Mock a VLM server error (e.g., 500 status code)."""
        mock_response = MagicMock(spec=httpx.Response)
        mock_response.status_code = 500
        mock_response.text = 'Internal Server Error'

        with patch('httpx.AsyncClient') as mock_async_client:
            # The JSON parsing will fail, triggering the generic exception
            mock_async_client.return_value.__aenter__.return_value.post = AsyncMock(return_value=mock_response)
            
            image = np.zeros((100, 100, 3), dtype=np.uint8)
            command_text = "go there"
            uwb_target = np.array([0.5, -1.5, 0])
            
            result = await self.vlm_client.get_action(image, command_text, uwb_target)
            
            self.assertIsNone(result)
            # The specific error message depends on what json.loads throws on 'Internal Server Error'
            self.mock_node.get_logger().error.assert_called()


    @patch('vita_agent.clients.vlm_client.convert_image_to_base64', return_value=None)
    @pytest.mark.asyncio
    async def test_get_action_invalid_image(self, mock_convert):
        """Test Case 3.4: Provide an invalid image and verify graceful failure."""
        with patch('httpx.AsyncClient') as mock_async_client:
            # We don't even need to mock the post call, as it should handle the empty base64 string
            mock_post = AsyncMock()
            mock_async_client.return_value.__aenter__.return_value.post = mock_post
            
            # An invalid image that causes convert_image_to_base64 to return None
            invalid_image = "this is not an image" 
            command_text = "go there"
            uwb_target = np.array([0.5, -1.5, 0])

            await self.vlm_client.get_action(invalid_image, command_text, uwb_target)
            
            # Check that the payload sent to the server has an empty image_base64 string
            sent_payload = mock_post.call_args[1]['json']
            self.assertEqual(sent_payload['image_base64'], "")

if __name__ == '__main__':
    unittest.main()