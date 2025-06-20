import fnmatch
import json
import os
import re
import shutil
import zipfile
from typing import Dict, Optional, Tuple

import numpy as np
import tqdm
from deep_translator import GoogleTranslator
from PIL import Image

from src.prompts import Prompt


def extract_average_uwb(video_dir):
    metadata_path = os.path.join(video_dir, "metadata.json")
    try:
        with open(metadata_path) as f:
            metadata = json.load(f)

        # Get the command
        command = metadata["tags"]["command"]
        uwbs_x = []
        uwbs_y = []
        distances = []

        for uwb in metadata["/uwb/data"][:5]:
            angle = uwb["data"]["angle"]
            distance = uwb["data"]["distance"]
            x, y = uwb2coordinates(angle, distance)
            uwbs_x.append(x)
            uwbs_y.append(y)
            distances.append(distance)

        average_uwb_position = (float(np.mean(uwbs_x)), float(np.mean(uwbs_y)))
        average_distance = float(np.mean(distances))

        # Save to JSON
        with open(os.path.join(video_dir, "average_uwb_position.json"), "w") as f:
            json.dump(
                {
                    "command": command,
                    "average_uwb_position": average_uwb_position,
                    "average_distance": average_distance,
                },
                f,
                indent=2,
                ensure_ascii=False,  # <-- Add this line
            )

    except (FileNotFoundError, KeyError, json.JSONDecodeError) as e:
        print(f"Error processing metadata: {e}")
        return None


def iterate_folder(folder_path, existing_keys=None):
    if existing_keys is None:
        existing_keys = set()
    # Get all directories and sort them
    dirs = [
        d
        for d in os.listdir(folder_path)
        if os.path.isdir(os.path.join(folder_path, d))
        and os.path.join(folder_path, d) not in existing_keys
    ]
    dirs.sort()  # Sort directories alphabetically

    for folder in dirs:
        yield os.path.join(folder_path, folder)


def load_prompt(prompt_object: Prompt, **kwargs) -> Dict[str, str]:
    """
    Load and format prompts from prompt.toml file.

    Args:
        prompt_type: Type of prompt to load ('go2object_prompt' or 'gohere_prompt')
        file_path: Path to the prompt.toml file
        **kwargs: Values to fill into the prompt templates

    Returns:
        Dictionary containing the formatted system_prompt and user_prompt
    """

    result = {}

    # Handle system prompt
    result["system_prompt"] = prompt_object.system_prompt

    # Handle user prompt
    result["user_prompt"] = prompt_object.user_prompt

    # Format system prompt if it has placeholders and kwargs are provided
    result["user_prompt"] = result["user_prompt"].format(**kwargs)

    return result


def parser_output_with_orientation(vlm_response):
    # there are 8 directions, including "back", "front", "left", "right", "upper left", "upper right", "lower left", "lower right".
    coordinates = parser_output(vlm_response)
    orientation = parse_person_orientation(vlm_response)
    return orientation, coordinates


def parser_output(vlm_response: str) -> Tuple[float, float]:
    """
    Parse the output from a VLM to extract BEV coordinates (x, y).
    Expects a response containing a coordinate tuple like '(1.5, 3.2)'.

    Args:
        vlm_response: String response from the VLM

    Returns:
        Tuple of (x, y) coordinates as floats

    Raises:
        ValueError: If coordinates cannot be parsed from the response
    """
    # Look for patterns like (x, y) or [x, y] in the response
    coordinate_pattern = r"[\(\[]?\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*[\)\]]?"
    matches = list(re.finditer(coordinate_pattern, vlm_response))

    if matches:
        # Get the last match
        match = matches[-1]
        try:
            x = float(match.group(1))
            y = float(match.group(2))
            return (x, y)
        except (ValueError, IndexError):
            raise ValueError(
                f"Found coordinate-like pattern but couldn't convert to floats: {match.group(0)}"
            )
    else:
        # If no explicit pattern found, try to find any two floating point numbers
        numbers = re.findall(r"(-?\d+\.?\d*)", vlm_response)
        if len(numbers) >= 2:
            try:
                # Take the last two numbers found
                x = float(numbers[-2])
                y = float(numbers[-1])
                return (x, y)
            except ValueError:
                pass

        raise ValueError("Could not parse coordinates from VLM response")


def parser_command(command: str):
    """
    Parse the command to get the object between "到" and "去".
    Uses deep-translator library to translate from Chinese to English.

    Args:
        command (str): Chinese command string

    Returns:
        str: English translation of the object (or None if no match)
    """
    # Extract text between "到" and "去" (supports optional whitespace)
    match = re.search(
        r"到\s*(.+?)\s*那去", command
    )  # Updated regex for whitespace tolerance
    if not match:
        return None

    object_chinese = match.group(1).strip()

    # Translate using deep-translator
    translator = GoogleTranslator(source="zh-CN", target="en")
    translation = translator.translate(object_chinese)
    return translation


def translate_command(command: str):
    """
    Translate the command to English.
    """
    translator = GoogleTranslator(source="zh-CN", target="en")
    translation = translator.translate(command)
    return translation


def recover_depth_map(image_array):
    # TODO: need reparse the depth map.
    if image_array.dtype == np.uint8:
        # Assuming the 8-bit values need to be scaled to the 16-bit range.
        # This is a common way to represent 16-bit data as 8-bit for visualization,
        # but the inverse operation doesn't restore lost precision.
        depth_approx_16bit = image_array.astype(np.uint16) * 256 / 1000
        return depth_approx_16bit
    return image_array


def prepare_go_here_command():
    return [
        "到我左边来",
        "到我前面来",
        "到我右边来",
        "到我后面来",
    ]


def command_to_prompt(command: str):
    """
    Extract the object from a command like '到 xxxx 去'.
    Returns the text between '到' and '去'.
    If the pattern does not match, returns an empty string.
    """
    # TODO: make sure it only return a object is necessary.
    match = re.search(r"到\s*(.+?)\s*去", command)
    if match:
        return match.group(1).strip()
    return ""


def uwb2coordinates(angle, distance):
    angle = np.deg2rad(-angle)
    x = distance * np.cos(angle)
    y = distance * np.sin(angle)
    return (
        -y,
        x,
    )  # currently, x is the horizon, y is the verical, and position x is the left.


def prepare_single_data(video_dir: str, parse_uwb: bool = True):
    # TODO: I think we can extract the average uwb position from the metadata.json firstly.
    # 1. get first jpeg as Image.Image
    jpeg_files = [
        f
        for f in os.listdir(video_dir)
        if os.path.isfile(os.path.join(video_dir, f)) and f.lower().endswith(".jpeg")
    ]
    if not jpeg_files:
        raise FileNotFoundError(f"No JPEG files found in {video_dir}")
    first_jpeg = sorted(jpeg_files)[0]
    jpeg_path = os.path.join(video_dir, first_jpeg)
    try:
        image = Image.open(jpeg_path)
    except Exception as e:
        raise ValueError(f"Failed to open image {jpeg_path}") from e

    # 2. get depth map
    frame_number = first_jpeg.split(".")[0]  # Extract frame number from jpeg filename
    depth_path = os.path.join(video_dir, f"{frame_number}_depth.npy")
    depth_map = np.load(depth_path)
    depth_map = recover_depth_map(depth_map)
    depth_map = np.transpose(depth_map)
    # 3. get metadata
    metadata_path = os.path.join(video_dir, "average_uwb_position.json")
    metadata = json.load(open(metadata_path))
    # 3.1 get the command
    command = metadata["command"]
    if parse_uwb:
        average_uwb_position = metadata["average_uwb_position"]
        average_distance = metadata["average_distance"]
    else:
        average_uwb_position = (0.0, 0.0)
        average_distance = 0.0
    return (
        image,
        depth_map,
        command,
        (round(average_uwb_position[0], 3), round(average_uwb_position[1], 3)),
        round(average_distance, 3),
    )


def gather_metadata(data_dir: str):
    # scan from 20250419 to 20250427 folder
    # read the json file as a tag
    # for each json, let the file name as the key. Related zip is the video folder.
    # 20250421/recording_20250421_105224_0.json 20250421/recording_20250421_105224_0.zip
    # json structure: {"tags": {"command": "跟我走", "status": "有效"}}
    # stats the various command .

    # 1. scan the data_dir
    mapping_dict = {}
    target_folders = []
    for folder in os.listdir(data_dir):
        if not os.path.isdir(os.path.join(data_dir, folder)):
            continue
        target_folders.append(folder)

    # 2. scan the folder to search read each json
    for folder in target_folders:
        json_files = []
        for file in os.listdir(os.path.join(data_dir, folder)):
            if file.endswith(".json"):
                json_files.append(file)
        # read each json file
        for json_file in json_files:
            with open(os.path.join(data_dir, folder, json_file), "r") as f:
                meta_data = json.load(f)
                command = meta_data["tags"]["command"]
                video_name = json_file.split(".")[0]
                mapping_dict[video_name] = command

    # 3. stats the various command
    command_stats = {}
    for command in mapping_dict.values():
        if command not in command_stats:
            command_stats[command] = 0
        command_stats[command] += 1
    print(command_stats)
    return command_stats, mapping_dict


def filter_out_command(mapping_dict, command_name: str):
    """
    Filter out items where the command matches the pattern (supports * wildcard).
    Example: command_name="come to *" will exclude all commands starting with "come to ".
    """
    return {
        k: v for k, v in mapping_dict.items() if not fnmatch.fnmatch(v, command_name)
    }


def filter_in_command(mapping_dict, command_name: str):
    """
    Filter in items where the command matches the pattern (supports * wildcard).
    Example: command_name="come to *" will include all commands starting with "come to ".
    """
    return {k: v for k, v in mapping_dict.items() if fnmatch.fnmatch(v, command_name)}


def get_first_five_images_and_depths(data_dir, mapping_dict, output_dir):
    """
    For each video in mapping_dict, extract/copy the first five .jpeg and .npy depth files into output_dir/video_name/.
    Also copies the metadata json file.
    Returns a dict: {video_name: {'jpeg': [list of output paths], 'depth': [list of output paths], 'metadata': metadata_path}}
    """
    result = {}
    os.makedirs(output_dir, exist_ok=True)
    for video_name in tqdm.tqdm(mapping_dict, desc="Processing videos"):
        # Find the folder or zip
        folder = video_name.split("_")[
            1
        ]  # e.g., 20250421 from recording_20250421_105224_0
        base_path = os.path.join(data_dir, folder)
        zip_path = os.path.join(base_path, f"{video_name}.zip")
        folder_path = os.path.join(base_path, video_name)
        video_output_dir = os.path.join(output_dir, video_name)
        os.makedirs(video_output_dir, exist_ok=True)
        if os.path.exists(zip_path):
            # Handle zip
            with zipfile.ZipFile(zip_path, "r") as zf:
                files = zf.namelist()
                jpegs = sorted([f for f in files if f.endswith(".jpeg")])[:5]
                depths = sorted([f for f in files if f.endswith("_depth.npy")])[:5]
                metadata_file = sorted([f for f in files if f.endswith(".json")])[0]
                jpeg_paths = []
                depth_paths = []
                for f in tqdm.tqdm(
                    jpegs, desc=f"Extracting jpegs for {video_name}", leave=False
                ):
                    out_path = os.path.join(video_output_dir, os.path.basename(f))
                    zf.extract(f, video_output_dir)
                    # Move to root of video_output_dir if needed
                    extracted_path = os.path.join(video_output_dir, f)
                    if extracted_path != out_path:
                        os.rename(extracted_path, out_path)
                        # Remove any intermediate dirs
                        parent_dir = os.path.dirname(extracted_path)
                        if parent_dir != video_output_dir and os.path.isdir(parent_dir):
                            try:
                                os.rmdir(parent_dir)
                            except OSError:
                                pass
                    jpeg_paths.append(out_path)
                for f in tqdm.tqdm(
                    depths, desc=f"Extracting depths for {video_name}", leave=False
                ):
                    out_path = os.path.join(video_output_dir, os.path.basename(f))
                    zf.extract(f, video_output_dir)
                    extracted_path = os.path.join(video_output_dir, f)
                    if extracted_path != out_path:
                        os.rename(extracted_path, out_path)
                        parent_dir = os.path.dirname(extracted_path)
                        if parent_dir != video_output_dir and os.path.isdir(parent_dir):
                            try:
                                os.rmdir(parent_dir)
                            except OSError:
                                pass
                    depth_paths.append(out_path)
                # Extract metadata file
                metadata_out_path = os.path.join(
                    video_output_dir, os.path.basename(metadata_file)
                )
                zf.extract(metadata_file, video_output_dir)
                extracted_metadata_path = os.path.join(video_output_dir, metadata_file)
                if extracted_metadata_path != metadata_out_path:
                    os.rename(extracted_metadata_path, metadata_out_path)
                    parent_dir = os.path.dirname(extracted_metadata_path)
                    if parent_dir != video_output_dir and os.path.isdir(parent_dir):
                        try:
                            os.rmdir(parent_dir)
                        except OSError:
                            pass
        elif os.path.exists(folder_path):
            # Handle folder
            files = os.listdir(folder_path)
            jpegs = sorted([f for f in files if f.endswith(".jpeg")])[:5]
            depths = sorted([f for f in files if f.endswith("_depth.npy")])[:5]
            metadata_file = sorted([f for f in files if f.endswith(".json")])[0]
            jpeg_paths = []
            depth_paths = []
            for f in tqdm.tqdm(
                jpegs, desc=f"Copying jpegs for {video_name}", leave=False
            ):
                src = os.path.join(folder_path, f)
                dst = os.path.join(video_output_dir, f)
                shutil.copy(src, dst)
                jpeg_paths.append(dst)
            for f in tqdm.tqdm(
                depths, desc=f"Copying depths for {video_name}", leave=False
            ):
                src = os.path.join(folder_path, f)
                dst = os.path.join(video_output_dir, f)
                shutil.copy(src, dst)
                depth_paths.append(dst)
            # Copy metadata file
            metadata_src = os.path.join(folder_path, metadata_file)
            metadata_dst = os.path.join(video_output_dir, metadata_file)
            shutil.copy(metadata_src, metadata_dst)
        else:
            continue  # Not found
        result[video_name] = {
            "jpeg": jpeg_paths,
            "depth": depth_paths,
            "metadata": os.path.join(video_output_dir, os.path.basename(metadata_file)),
        }
    return result


def parse_person_orientation(text: str) -> Optional[str]:
    """
    Extracts the value of 'person orientation' from a string containing that key.
    Returns the orientation string, or None if not found.
    """
    # Regex to match: "person orientation": "some value"
    match = re.search(r'"person orientation"\s*:\s*"([^"]+)"', text)
    if match:
        return match.group(1)
    return None


if __name__ == "__main__":
    # Example usage of the prompt loading and parsing functions
    try:
        # Example 1: Load go2object prompt
        # object_prompt = load_prompt(
        #     "go2object_prompt", object_name="chair", depth="2.5"
        # )
        # print("Go2Object System Prompt:", object_prompt["system_prompt"][:100] + "...")
        # print("Go2Object User Prompt:", object_prompt["user_prompt"])

        # Example 2: Load gohere prompt
        # here_prompt = load_prompt("gohere_prompt", person_x=320, person_y=240)
        # print("\nGoHere System Prompt:", here_prompt["system_prompt"][:100] + "...")
        # print("GoHere User Prompt:", here_prompt["user_prompt"])

        # Example 3: Parse VLM output
        sample_response = (
            "After analyzing the image, I estimate the coordinates to be (1.2, 3.4)"
        )
        coordinates = parser_output(sample_response)
        print("\nParsed coordinates:", coordinates)

    except Exception as e:
        print(f"Error in example: {str(e)}")

    # Original data gathering code
    _, map_dict = gather_metadata(
        "/home/heng.li/minio-mount/vita-algo-data/vita-go2edu-nx"
    )
    result = get_first_five_images_and_depths(
        data_dir="/home/heng.li/minio-mount/vita-algo-data/vita-go2edu-nx",
        mapping_dict=map_dict,
        output_dir="sample_data/output",
    )
