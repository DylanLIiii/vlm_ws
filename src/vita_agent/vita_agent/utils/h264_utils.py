import av 
import numpy as np 

def get_logger():
    """Placeholder for logger, replace with actual logging implementation"""
    import logging
    return logging.getLogger(__name__)


def find_nal_units(data):
    """Find NAL unit start codes in H.264 data"""
    nal_units = []
    i = 0
    while i < len(data) - 3:
        # Look for 3-byte start code (0x000001) or 4-byte start code (0x00000001)
        if data[i:i+3] == b'\x00\x00\x01':
            nal_units.append(i)
            i += 3
        elif data[i:i+4] == b'\x00\x00\x00\x01':
            nal_units.append(i)
            i += 4
        else:
            i += 1
    return nal_units

def add_nal_delimiters(data):
    """Add NAL unit delimiters if missing"""
    if isinstance(data, np.ndarray):
        data_bytes = data.tobytes()
    elif hasattr(data, 'tobytes'):
        data_bytes = np.array(data, dtype=np.uint8).tobytes()
    else:
        data_bytes = bytes(data)
    
    # Check if data already has NAL start codes
    if b'\x00\x00\x01' in data_bytes or b'\x00\x00\x00\x01' in data_bytes:
        return data_bytes
    
    # If no start codes found, prepend one
    return b'\x00\x00\x00\x01' + data_bytes

# Now the decode method is working
def decode_h264_frame(compressed_video_msg, h264_codec_context=None, debug_first_frames=False, frame_count=0, logger=None, verbose=False):
    """
    Decode H.264 compressed video message with enhanced error handling
    
    Args:
        compressed_video_msg: Video message containing H.264 data
        h264_codec_context: PyAV codec context (will be created if None)
        debug_first_frames: Whether to enable debug logging for first frames
        frame_count: Current frame count for debugging
        logger: Logger instance (will use get_logger() if None)
    
    Returns:
        tuple: (decoded_cv_image, h264_codec_context) or (None, h264_codec_context)
    """
    if logger is None:
        logger = get_logger()
        
    try:
        if h264_codec_context is None:
            logger.info("Initializing PyAV H.264 decoder context.")
            h264_codec_context = av.CodecContext.create('h264', 'r')
            # Set decoder options for better compatibility
            h264_codec_context.options = {
                'flags2': '+export_mvs',
                'err_detect': 'ignore_err'  # Try to continue despite errors
            }

        video_data = compressed_video_msg.data
        
        if not video_data or len(video_data) == 0:
            logger.warn('Received empty video data')
            return None, h264_codec_context

        # Debug info for first few frames
        if debug_first_frames and frame_count < 5:
            logger.info(f"Frame {frame_count}: Data length: {len(video_data)}")
            if len(video_data) >= 10:
                # Show first 10 bytes in hex
                hex_data = ' '.join(f'{b:02x}' for b in video_data[:10])
                logger.info(f"First 10 bytes: {hex_data}")

        # Try to add NAL delimiters if needed
        data_bytes = add_nal_delimiters(video_data)

        if debug_first_frames and frame_count < 5:
            nal_positions = find_nal_units(data_bytes)
            logger.info(f"Found {len(nal_positions)} NAL units at positions: {nal_positions[:5]}")

        # Try different approaches for parsing
        packets = []
        
        # Method 1: Direct parsing
        try:
            packets = h264_codec_context.parse(data_bytes)
        except Exception as e:
            logger.warn(f"Direct parsing failed: {e}")
            
            # Method 2: Try with packet creation
            try:
                packet = av.Packet(data_bytes)
                packets = [packet]
            except Exception as e2:
                logger.warn(f"Packet creation also failed: {e2}")
                return None, h264_codec_context

        if not packets:
            if debug_first_frames:
                logger.warn('No packets generated from H.264 data')
            return None, h264_codec_context

        # Decode packets
        decoded_frames = []
        for packet in packets:
            try:
                if hasattr(packet, 'is_corrupt') and packet.is_corrupt:
                    continue
                
                frames = h264_codec_context.decode(packet)
                decoded_frames.extend(frames)
                
            except av.error.InvalidDataError as e:
                if debug_first_frames:
                    logger.warn(f"InvalidDataError during decode: {e}")
                continue
            except Exception as e:
                if debug_first_frames:
                    logger.warn(f"Other decode error: {e}")
                continue

        if not decoded_frames:
            if debug_first_frames:
                logger.warn('No frames decoded from packets')
            return None, h264_codec_context

        # Convert first frame to OpenCV format
        av_frame = decoded_frames[0]
        cv_image = av_frame.to_ndarray(format='bgr24')
        

        if verbose:
            logger.info(f"Successfully decoded frame {frame_count}: shape {cv_image.shape}")
        return cv_image, h264_codec_context

    except Exception as e:
        if debug_first_frames:
            logger.error(f'Unexpected error in decode_h264_frame: {e}', exc_info=True)
        return None, h264_codec_context

def decode_h264_frame_simple(compressed_video_msg, logger=None):
    """
    Simple H.264 decoder that creates a new codec context each time.
    Use this for one-off decoding or when you don't need to maintain state.
    
    Args:
        compressed_video_msg: Video message containing H.264 data
        logger: Logger instance (will use get_logger() if None)
    
    Returns:
        Decoded OpenCV image or None if decoding failed
    """
    decoded_image, _ = decode_h264_frame(compressed_video_msg, None, False, 0, logger)
    return decoded_image


class H264DecoderContext:
    """
    Helper class to maintain H.264 decoder state for efficient sequential decoding.
    This is useful when decoding multiple frames from the same stream.
    """
    
    def __init__(self, debug_first_frames=False, logger=None, verbose=False):
        self.h264_codec_context = None
        self.debug_first_frames = debug_first_frames
        self.frame_count = 0
        self.logger = logger or get_logger()
        self.verbose = verbose
    
    def decode_frame(self, compressed_video_msg):
        """
        Decode a single frame using the maintained codec context.
        
        Args:
            compressed_video_msg: Video message containing H.264 data
            
        Returns:
            Decoded OpenCV image or None if decoding failed
        """
        decoded_image, self.h264_codec_context = decode_h264_frame(
            compressed_video_msg, 
            self.h264_codec_context, 
            self.debug_first_frames, 
            self.frame_count, 
            self.logger,
            self.verbose
        )
        self.frame_count += 1
        return decoded_image
    
    def reset(self):
        """Reset the decoder context and frame count."""
        self.h264_codec_context = None
        self.frame_count = 0