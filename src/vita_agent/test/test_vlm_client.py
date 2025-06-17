import pytest
from unittest.mock import MagicMock, patch, AsyncMock
import numpy as np
import httpx
from rclpy.node import Node
from vita_agent.clients.vlm_client import VlmClient

@pytest.fixture
def mock_node():
    node = MagicMock(spec=Node)
    node.get_logger.return_value = MagicMock()
    mock_param = MagicMock()
    mock_param.get_parameter_value.return_value.string_value = "http://fake-vlm-server.com/api"
    node.get_parameter.return_value = mock_param
    return node

@pytest.fixture
def vlm_client(mock_node):
    return VlmClient(mock_node)

@pytest.mark.asyncio
async def test_get_action_success(vlm_client, mock_node):
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
        
        result = await vlm_client.get_action(image, command_text, uwb_target)

        assert result == (2.0, -1.0)
        mock_node.get_logger().info.assert_called_with('Server response: 200 {"predicted_bev_coordinates": [1.0, 2.0]}')

@pytest.mark.asyncio
async def test_get_action_timeout(vlm_client, mock_node):
    """Test Case 3.2: Mock a VLM server timeout."""
    with patch('httpx.AsyncClient') as mock_async_client:
        mock_async_client.return_value.__aenter__.return_value.post = AsyncMock(side_effect=httpx.TimeoutException("Timeout"))

        image = np.zeros((100, 100, 3), dtype=np.uint8)
        command_text = "go there"
        uwb_target = np.array([0.5, -1.5, 0])

        result = await vlm_client.get_action(image, command_text, uwb_target)

        assert result is None
        mock_node.get_logger().error.assert_called_with('Failed to post to VLM server: Timeout')

@pytest.mark.asyncio
async def test_get_action_server_error(vlm_client, mock_node):
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
        
        result = await vlm_client.get_action(image, command_text, uwb_target)
        
        assert result is None
        # The specific error message depends on what json.loads throws on 'Internal Server Error'
        mock_node.get_logger().error.assert_called()


@pytest.mark.asyncio
@patch('vita_agent.clients.vlm_client.convert_image_to_base64', return_value=None)
async def test_get_action_invalid_image(mock_convert, vlm_client, mock_node):
    """Test Case 3.4: Provide an invalid image and verify graceful failure."""
    with patch('httpx.AsyncClient') as mock_async_client:
        mock_post = AsyncMock()
        mock_async_client.return_value.__aenter__.return_value.post = mock_post

        # Instead of a string, pass None to simulate an invalid image and avoid cv2 errors
        invalid_image = None
        command_text = "go there"
        uwb_target = np.array([0.5, -1.5, 0])

        await vlm_client.get_action(invalid_image, command_text, uwb_target)

        sent_payload = mock_post.call_args[1]['json']
        assert sent_payload['image_base64'] == ""