"""
Module for calling Vision Language Models using litellm.

This module provides functionality to make API calls to Qwen vision-language models
using the litellm library. It handles image processing, request formatting, and error handling.
"""

import base64
import logging
import os
from io import BytesIO
from typing import Any, Dict, Optional, Union

import litellm
from PIL import Image


class BaseVLMClient:
    """
    Client for making calls to Qwen VLM models using litellm.

    This class provides methods to prepare prompts, process images, and make API calls
    to Qwen vision-language models through litellm.
    """

    def __init__(
        self,
        model_name: Optional[str] = None,
        api_key: Optional[str] = None,
        api_base: Optional[str] = None,
        max_tokens: int = 2048,
        temperature: float = 0.7,
        timeout: int = 60,
    ):
        """
        Initialize the QwenVLMClient.

        Args:
            model_name: Name of the Qwen VLM model to use. If None, will use the
                        QWEN_VLM_MODEL environment variable or default to "qwen-vl-plus".
            api_key: API key for model access. If None, will use the QWEN_API_KEY
                    environment variable.
            api_base: API base URL. If None, will use the QWEN_API_BASE
                     environment variable.
            max_tokens: Maximum number of tokens to generate in the response.
            temperature: Sampling temperature for generation (0.0 to 1.0).
            timeout: Timeout for API requests in seconds.
        """
        self.model_name = model_name or os.getenv("QWEN_VLM_MODEL", "qwen-vl-plus")
        self.api_key = api_key or os.getenv("QWEN_API_KEY")
        self.api_base = api_base or os.getenv("QWEN_API_BASE")
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.timeout = timeout
        self.logger = logging.getLogger(__name__)

        if not self.api_key:
            self.logger.warning(
                "No API key provided. Either set it during initialization or "
                "using the QWEN_API_KEY environment variable."
            )

    def _encode_image(self, image_path_or_data: Union[str, bytes, Image.Image]) -> str:
        """
        Encode an image to base64 for API request.

        Args:
            image_path_or_data: Either a path to an image file, bytes containing the image data,
                               or a PIL Image object.

        Returns:
            Base64 encoded string of the image.

        Raises:
            ValueError: If the image cannot be processed or encoded.
            FileNotFoundError: If the image path does not exist.
        """
        try:
            # Handle different input types
            if isinstance(image_path_or_data, str):
                # It's a file path
                if not os.path.exists(image_path_or_data):
                    raise FileNotFoundError(
                        f"Image file not found: {image_path_or_data}"
                    )
                with open(image_path_or_data, "rb") as img_file:
                    return base64.b64encode(img_file.read()).decode("utf-8")

            elif isinstance(image_path_or_data, bytes):
                # It's raw bytes
                return base64.b64encode(image_path_or_data).decode("utf-8")

            elif isinstance(image_path_or_data, Image.Image):
                # It's a PIL Image
                buffered = BytesIO()
                # If image has RGBA mode, convert to RGB for better compatibility
                if image_path_or_data.mode == "RGBA":
                    image_path_or_data = image_path_or_data.convert("RGB")
                image_path_or_data.save(buffered, format="JPEG")
                return base64.b64encode(buffered.getvalue()).decode("utf-8")

            else:
                raise ValueError(
                    "Unsupported image type. Must be path, bytes, or PIL Image."
                )

        except Exception as e:
            self.logger.error(f"Error encoding image: {str(e)}")
            raise

    def _format_message_content(self, user_prompt: str, base64_image: str) -> list:
        """
        Format the message content for the API request.

        Args:
            user_prompt: The user's query or instruction.
            base64_image: Base64 encoded image string.

        Returns:
            List of content items for the API request.
        """
        return [
            {"type": "text", "text": user_prompt},
            {
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
            },
        ]

    def call_vlm(
        self,
        system_prompt: str,
        user_prompt: str,
        image_path_or_data: Union[str, bytes, Image.Image],
        additional_params: Optional[Dict[str, Any]] = None,
        stream: bool = False,
    ) -> str:
        """
        Call the Qwen VLM model with text and image inputs.

        Args:
            system_prompt: System-level instructions for the VLM.
            user_prompt: The user's query or instruction.
            image_path_or_data: Path to an image file, bytes containing the image data,
                               or a PIL Image object.
            additional_params: Optional dictionary of additional parameters to pass to litellm.
            stream: Whether to use streaming response mode.
        Returns:
            The text response from the VLM.
        Raises:
            Exception: If there's an error with the API call.
        """
        try:
            # Encode the image to base64
            base64_image = self._encode_image(image_path_or_data)
            # Format the message content
            message_content = self._format_message_content(user_prompt, base64_image)
            # Prepare the messages
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": message_content},
            ]
            # Prepare API parameters
            params = {
                "model": self.model_name,
                "messages": messages,
                "max_tokens": self.max_tokens,
                "temperature": self.temperature,
                "timeout": self.timeout,
                "stream": stream,
                "num_retries": 5,
            }
            # Add API credentials if available
            if self.api_key:
                params["api_key"] = self.api_key
            if self.api_base:
                params["api_base"] = self.api_base
            # Add any additional parameters
            if additional_params:
                params.update(additional_params)
            self.logger.info(
                f"Calling {self.model_name} with litellm (stream={stream})"
            )
            response = litellm.completion(**params)
            # --- Response Handling ---
            text_response = response.choices[0].message.content  # type: ignore
            return text_response  # type: ignore

        except Exception as e:
            self.logger.error(f"Error calling VLM: {str(e)}")
            raise


def call_vlm(
    client: BaseVLMClient,
    system_prompt: str,
    user_prompt: str,
    image_path_or_data: Union[str, bytes, Image.Image],
    additional_params: Optional[Dict[str, Any]] = None,
) -> str:
    """
    Call a Qwen VLM model with text and image inputs (functional interface).

    Args:
        system_prompt: System-level instructions for the VLM.
        user_prompt: The user's query or instruction.
        image_path_or_data: Path to an image file, bytes containing the image data,
                           or a PIL Image object.
        model_name: Name of the Qwen VLM model to use. If None, will use the
                   QWEN_VLM_MODEL environment variable or default to "qwen-vl-plus".
        api_key: API key for model access. If None, will use the QWEN_API_KEY
                environment variable.
        api_base: API base URL. If None, will use the QWEN_API_BASE
                 environment variable.
        max_tokens: Maximum number of tokens to generate in the response.
        temperature: Sampling temperature for generation (0.0 to.0).
        additional_params: Optional dictionary of additional parameters to pass to litellm.

    Returns:
        The text response from the VLM.

    Raises:
        Exception: If there's an error with the API call.
    """
    return client.call_vlm(
        system_prompt=system_prompt,
        user_prompt=user_prompt,
        image_path_or_data=image_path_or_data,
        additional_params=additional_params,
    )


# openrouter api key: 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
clients_registry = {
    ## qwen series model
    "qwen-vl-max": BaseVLMClient(
        model_name="openai/qwen-vl-max",
        api_key="sk-416a4cfb6c8d421a941ba5a94a575d6e",
        api_base="https://dashscope.aliyuncs.com/compatible-mode/v1",
    ),
    # "qwen-vl-plus": BaseVLMClient(
    #     model_name="openai/qwen-vl-plus",
    #     api_key="sk-416a4cfb6c8d421a941ba5a94a575d6e",
    #     api_base="https://dashscope.aliyuncs.com/compatible-mode/v1"
    # ),
    ## Gemini
    "gemini-2.5-pro-preview": BaseVLMClient(
        model_name="openrouter/google/gemini-2.5-pro-preview",
        api_key="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",
        api_base="https://openrouter.ai/api/v1",
    ),
    ## Open AI Model
    "gpt-4o": BaseVLMClient(
        model_name="openrouter/openai/gpt-4o",
        api_key="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",
        api_base="https://openrouter.ai/api/v1",
    ),
    ## Claude Model
    "claude": BaseVLMClient(
        model_name="openrouter/anthropic/claude-3.7-sonnet",
        api_key="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",
        api_base="https://openrouter.ai/api/v1",
    ),
    # # Open Source
    # internvl
    # "internvl": BaseVLMClient(
    #     model_name="openrouter/opengvlab/internvl3-14b:free",
    #     api_key="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",
    #     api_base="https://openrouter.ai/api/v1"
    # ),
    "qwen-2.5-vl-free": BaseVLMClient(
        model_name="openrouter/qwen/qwen2.5-vl-72b-instruct:free",
        api_key="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",
        api_base="https://openrouter.ai/api/v1",
    ),
    "qwen-2.5-vl": BaseVLMClient(
        model_name="openai/qwen2.5-vl-72b-instruct",
        api_key="sk-416a4cfb6c8d421a941ba5a94a575d6e",
        api_base="https://dashscope.aliyuncs.com/compatible-mode/v1",
    ),
    # "llama4": BaseVLMClient(
    #     model_name="openrouter/meta-llama/llama-4-maverick:free",
    #     api_key="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",
    #     api_base="https://openrouter.ai/api/v1"
    # ),
    # "mistral": BaseVLMClient(
    #     model_name="openrouter/mistralai/mistral-small-3.1-24b-instruct:free",
    #     api_key="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",
    #     api_base="https://openrouter.ai/api/v1"
    # ),
    # "kimi": BaseVLMClient(
    #     model_name="openrouter/moonshotai/kimi-vl-a3b-thinking:free",
    #     api_key="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",
    #     api_base="https://openrouter.ai/api/v1"
    # )
    # Dummy client for testing
    "dummy_client": BaseVLMClient(
        model_name="openai/qwen-vl-plus",
        api_key="sk-416a4cfb6c8d421a941ba5a94a575d6e",
        api_base="https://dashscope.aliyuncs.com/compatible-mode/v1",
    ),
}
