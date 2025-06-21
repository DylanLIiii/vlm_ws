import base64
import binascii
import logging
from io import BytesIO
from typing import List, Optional

from fastapi import FastAPI, HTTPException
from PIL import Image, UnidentifiedImageError
from pydantic import BaseModel

from src.call_vlm import call_vlm, clients_registry
from src.prompt_registry import prompt_registry
from src.utils import load_prompt, parser_output, parser_output_with_orientation

# Set up logging with custom format
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    datefmt="%y/%m/%d/%H/%M/%S",
)


# Pydantic models for request/response
class SpatialDataRequest(BaseModel):
    vlm_client_name: str
    task_mode: str
    prompt_name: str
    image_base64: str
    command_text: str
    person_x: float
    person_y: float


class SpatialDataResponse(BaseModel):
    model_used: str
    prompt_used: str
    mode_used: str
    predicted_bev_coordinates: List[float]
    predicted_orientation: Optional[str] = None
    raw_vlm_response: str
    error_message: Optional[str] = None


# Initialize FastAPI app
app = FastAPI()


@app.post("/process_spatial_data", response_model=SpatialDataResponse)
async def process_spatial_data(request_data: SpatialDataRequest):
    # Log the incoming request
    logging.info(
        f"Received request - VLM: {request_data.vlm_client_name}, Task mode: {request_data.task_mode}, Prompt: {request_data.prompt_name}, Command: '{request_data.command_text}', Person position: ({request_data.person_x}, {request_data.person_y}, UWB position: ({request_data.person_y}, {-request_data.person_x}))"
    )

    try:
        vlm_client_instance = clients_registry.get(request_data.vlm_client_name)
        if not vlm_client_instance:
            raise HTTPException(
                status_code=400,
                detail=f"VLM client '{request_data.vlm_client_name}' not found, valid clients are: {list(clients_registry.keys())}",
            )

        # Get prompt from registry
        if request_data.task_mode not in prompt_registry:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid task mode: {request_data.task_mode}, valid task modes are: {list(prompt_registry.keys())}",
            )

        prompt_object = prompt_registry[request_data.task_mode].get(
            request_data.prompt_name
        )
        if not prompt_object:
            raise HTTPException(
                status_code=400,
                detail=f"Prompt '{request_data.prompt_name}' not found for task mode '{request_data.task_mode}', valid prompts are: {list(prompt_registry[request_data.task_mode].keys())}",
            )

        # Decode base64 image
        try:
            if not request_data.image_base64 or request_data.image_base64 == "":
                # Use a dummy black image if image_base64 is empty
                pil_image = Image.new("RGB", (10, 10), 0)
            else:
                image_bytes = base64.b64decode(request_data.image_base64)
                pil_image = Image.open(BytesIO(image_bytes))
        except (binascii.Error, UnidentifiedImageError, OSError) as e:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid base64 image data: {str(e)}, please check the image format and try again",
            )

        # Load prompt templates
        prompt_result = load_prompt(
            prompt_object,
            person_x=request_data.person_x,
            person_y=request_data.person_y,
            command=request_data.command_text,
        )

        # Call VLM
        try:
            vlm_response_text = call_vlm(
                vlm_client_instance,
                prompt_result["system_prompt"],
                prompt_result["user_prompt"],
                pil_image,
            )

            # Log the VLM response
            logging.info(
                f"VLM response for {request_data.vlm_client_name}: {vlm_response_text}"
            )
        except Exception as e:
            logging.error(f"Error calling VLM {request_data.vlm_client_name}: {str(e)}")
            raise HTTPException(status_code=500, detail=f"Error calling VLM: {str(e)}")

        # Parse VLM response
        try:
            if request_data.task_mode == "free":
                parsed_orientation, parsed_bev_coordinates = (
                    parser_output_with_orientation(vlm_response_text)
                )
            elif request_data.task_mode == "fix":
                parsed_bev_coordinates = parser_output(vlm_response_text)
                parsed_orientation = None
            elif request_data.task_mode == "dummy_mode":
                parsed_bev_coordinates = [0, 0]
                parsed_orientation = None

            # Convert coordinates to list of floats
            coordinates_list = [
                float(parsed_bev_coordinates[0]),
                float(parsed_bev_coordinates[1]),
            ]

            # Log the parsed results
            logging.info(
                f"Parsed results - Coordinates: {coordinates_list}, Orientation: {parsed_orientation}"
            )

            # Prepare successful response
            return SpatialDataResponse(
                model_used=request_data.vlm_client_name,
                prompt_used=request_data.prompt_name,
                mode_used=request_data.task_mode,
                predicted_bev_coordinates=coordinates_list,
                predicted_orientation=parsed_orientation,
                raw_vlm_response=vlm_response_text,
                error_message=None,
            )

        except Exception as e:
            logging.error(f"Error parsing VLM response: {str(e)}")
            raise HTTPException(
                status_code=500, detail=f"Error parsing VLM response: {str(e)}"
            )

    except HTTPException:
        raise
    except Exception as e:
        logging.error(f"Internal server error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


if __name__ == "__main__":
    try:
        import uvicorn
    except ImportError:
        raise ImportError("Please install uvicorn with: pip install uvicorn")
    uvicorn.run(app, host="0.0.0.0", port=8000)
