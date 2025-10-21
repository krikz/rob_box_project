"""
Frame Renderer

Loads PNG images and converts them to ROS2 Image messages.
"""

from PIL import Image
import numpy as np
from sensor_msgs.msg import Image as ImageMsg
from typing import Tuple


class FrameRenderer:
    """Renders animation frames to ROS2 messages"""

    def __init__(self):
        """Initialize renderer"""
        self.image_cache = {}

    def load_image(self, image_path: str) -> np.ndarray:
        """
        Load PNG image from file

        Args:
            image_path: Path to PNG file

        Returns:
            Numpy array with RGB data (H, W, 3)

        Raises:
            FileNotFoundError: If image file not found
            ValueError: If image format is invalid
        """
        # Check cache
        if image_path in self.image_cache:
            return self.image_cache[image_path]

        # Load image
        try:
            img = Image.open(image_path)
        except FileNotFoundError:
            raise FileNotFoundError(f'Image not found: {image_path}')
        except Exception as e:
            raise ValueError(f'Failed to load image {image_path}: {e}')

        # Convert to RGB
        if img.mode != 'RGB':
            img = img.convert('RGB')

        # Convert to numpy array
        img_array = np.array(img, dtype=np.uint8)

        # Validate shape
        if len(img_array.shape) != 3 or img_array.shape[2] != 3:
            raise ValueError(f'Invalid image shape: {img_array.shape} (expected H×W×3)')

        # Cache and return
        self.image_cache[image_path] = img_array
        return img_array

    def image_to_ros_msg(
        self,
        image_array: np.ndarray,
        logical_group: str,
        encoding: str = 'rgb8'
    ) -> ImageMsg:
        """
        Convert numpy array to ROS2 Image message

        Args:
            image_array: Numpy array (H, W, 3)
            logical_group: LED panel logical group name (used as frame_id)
            encoding: Image encoding (default: 'rgb8')

        Returns:
            sensor_msgs/Image message
        """
        msg = ImageMsg()

        # Set header
        msg.header.frame_id = logical_group

        # Set image properties
        msg.height = image_array.shape[0]
        msg.width = image_array.shape[1]
        msg.encoding = encoding
        msg.step = image_array.shape[1] * 3  # Width × 3 bytes per pixel
        msg.is_bigendian = 0

        # Set image data
        msg.data = image_array.tobytes()

        return msg

    def render_frame(self, image_path: str, logical_group: str) -> ImageMsg:
        """
        Load image and convert to ROS2 message

        Args:
            image_path: Path to PNG file
            logical_group: LED panel logical group name

        Returns:
            sensor_msgs/Image message ready to publish
        """
        img_array = self.load_image(image_path)
        return self.image_to_ros_msg(img_array, logical_group)

    def validate_image_size(
        self,
        image_path: str,
        expected_width: int,
        expected_height: int
    ) -> Tuple[bool, str]:
        """
        Validate image size matches expected dimensions

        Args:
            image_path: Path to image
            expected_width: Expected width in pixels
            expected_height: Expected height in pixels

        Returns:
            Tuple of (is_valid, error_message)
        """
        try:
            img_array = self.load_image(image_path)
            height, width = img_array.shape[0], img_array.shape[1]

            if width != expected_width or height != expected_height:
                return False, f'Image size {width}×{height} doesn\'t match expected {expected_width}×{expected_height}'

            return True, ''

        except Exception as e:
            return False, str(e)

    def clear_cache(self):
        """Clear image cache to free memory"""
        self.image_cache.clear()

    def get_cache_size(self) -> int:
        """Get number of cached images"""
        return len(self.image_cache)


# Panel size definitions
PANEL_SIZES = {
    'wheel_front_left': (8, 8),
    'wheel_front_right': (8, 8),
    'wheel_rear_left': (8, 8),
    'wheel_rear_right': (8, 8),
    'wheels_front': (16, 8),
    'wheels_rear': (16, 8),
    'wheels_all': (16, 16),
    'main_display': (25, 5),  # Width × Height (landscape)
}


def get_panel_size(logical_group: str) -> Tuple[int, int]:
    """
    Get expected size for a logical group

    Args:
        logical_group: LED panel logical group name

    Returns:
        Tuple of (width, height)

    Raises:
        ValueError: If logical group is unknown
    """
    if logical_group not in PANEL_SIZES:
        raise ValueError(f'Unknown logical group: {logical_group}')

    return PANEL_SIZES[logical_group]
