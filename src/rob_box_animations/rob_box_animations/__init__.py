"""
rob_box_animations - LED Matrix Animation System

This package provides animation playback for LED matrices on rob_box robot.
"""

__version__ = "1.0.0"
__author__ = "krikz"
__license__ = "MIT"

from .animation_loader import AnimationLoader
from .animation_player import AnimationPlayer
from .frame_renderer import FrameRenderer

__all__ = [
    'AnimationLoader',
    'AnimationPlayer',
    'FrameRenderer',
]
