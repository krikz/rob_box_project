# Copyright 2026 krikz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
