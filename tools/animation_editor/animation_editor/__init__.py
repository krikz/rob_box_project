"""
LED Animation Editor для rob_box

Полнофункциональный редактор анимаций для LED матриц робота.
Поддерживает редактирование пикселей, timeline, импорт/экспорт.
"""

__version__ = "1.0.0"
__author__ = "rob_box team"

from .app import AnimationEditorApp
from .models import Animation, Frame, Panel
from .canvas import RobotCanvas
from .timeline import Timeline
from .palette import ColorPalette

__all__ = [
    'AnimationEditorApp',
    'Animation',
    'Frame',
    'Panel',
    'RobotCanvas',
    'Timeline',
    'ColorPalette',
]
