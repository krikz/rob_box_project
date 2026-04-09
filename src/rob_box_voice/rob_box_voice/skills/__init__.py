"""
skills/ — Agent-as-tool skill sub-agents for РОББОКС voice assistant.

Each skill wraps a specialised Agent (with its own LLM loop and domain prompt)
and exposes it as a single FunctionTool via Agent.as_tool(), so the Compositor
agent can delegate domain-specific subtasks.

Skill ↔ domain mapping:
  MusicSkill      — execute_music_code, stop_music, set_vibe_preset, get_music_state, search_samples
  NavigationSkill — navigate_to_waypoint, move_direction
  MemorySkill     — memory_save, memory_search, memory_context
  StatusSkill     — get_robot_status, get_battery_level, get_current_time, set_volume, set_pitch
"""

from .memory_skill import MemorySkill
from .music_skill import MusicSkill
from .navigation_skill import NavigationSkill
from .status_skill import StatusSkill
from .faq_skill import FAQSkill

__all__ = [
    "MusicSkill",
    "NavigationSkill",
    "MemorySkill",
    "StatusSkill",
    "FAQSkill",
]
