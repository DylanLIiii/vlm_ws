import cv2
import httpx
from rclpy.node import Node

from vita_agent.utils.image_utils import convert_image_to_base64


class VlmClient:
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        self.vlm_server_url = node.get_parameter('vlm_server_url').get_parameter_value().string_value

    async def get_action(self, image, command_text, uwb_target):
        vlm_client_name = 'qwen-2.5-vl'
        task_mode = 'free'  # or free
        if task_mode == 'free':
            prompt_name = 'cot_few_shot_go_here_orientation_prompt'
        elif task_mode == 'fix':
            prompt_name = 'cot_go_here_strict_orientation_prompt'
            
        person_x, person_y = uwb_target[0], uwb_target[1]

        if image is not None:
            bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            image_base64 = convert_image_to_base64(bgr_image)
            if image_base64 is None:
                image_base64 = ""
        else:
            image_base64 = ""

        payload = {
            "vlm_client_name": vlm_client_name,
            "task_mode": task_mode,
            "prompt_name": prompt_name,
            "image_base64": image_base64,
            "command_text": command_text,
            "person_x": person_x,
            "person_y": person_y,
        }

        try:
            # async with httpx.AsyncClient() as client:
            #     response = await client.post(self.vlm_server_url, json=payload, timeout=60.0)
            # self.logger.info(f'Server response: {response.status_code} {response.text}')
            # data = response.json()
            # self.node.vlm_reponse = data
            # vlm_x, vlm_y = data.get('predicted_bev_coordinates', [0, 0])
            # self.node.predicted_cooridates = vlm_y, -vlm_x
            return person_y, - person_x
        except httpx.RequestError as e:
            self.logger.error(f'Failed to post to VLM server: {e}')
        except Exception as e:
            self.logger.error(f'An unexpected error occurred: {e}')
        return None
