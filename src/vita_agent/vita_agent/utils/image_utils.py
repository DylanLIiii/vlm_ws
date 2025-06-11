"""
Image processing utilities for the VLM planner.
"""
import cv2
import base64


def convert_image_to_base64(image):
    """
    Convert OpenCV image to base64 encoded string.
    
    Args:
        image: OpenCV image (BGR format)
        
    Returns:
        str: Base64 encoded image string, or None if conversion fails
    """
    if image is None:
        return None
    
    # Encode image as JPEG
    success, encoded_img = cv2.imencode('.jpg', image)
    if not success:
        return None
    
    # Convert to base64 string
    base64_str = base64.b64encode(encoded_img).decode('utf-8')
    return base64_str
