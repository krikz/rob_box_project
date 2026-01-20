"""
Core business logic modules for rob_box_voice.

This package contains pure Python modules with no ROS dependencies,
making them easy to test and understand.
"""

from .dialogue_manager import DialogueManager, DialogueState

__all__ = ['DialogueManager', 'DialogueState']
