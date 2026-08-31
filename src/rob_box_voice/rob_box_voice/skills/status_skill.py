"""
status_skill.py — StatusSkill: sub-agent for robot status queries and voice adjustments.

Tools exposed to the sub-agent:
  get_robot_status  — full system status
  get_battery_level — battery percentage
  get_current_time  — current date and time
  set_volume        — adjust TTS volume
  set_pitch         — adjust TTS pitch
"""

from agents import function_tool

from .base_skill import BaseSkill


class StatusSkill(BaseSkill):
    """Sub-agent that handles status queries and voice parameter adjustments."""

    def _make_tools(self) -> list:
        async def _call(name, params, timeout=10.0):
            return await self._call(name, params, timeout)

        @function_tool
        async def get_robot_status() -> str:
            """Get full robot status: battery, sensors, navigation state, uptime."""
            return await _call("get_robot_status", {})

        @function_tool
        async def get_battery_level() -> str:
            """Get battery charge level as a percentage."""
            return await _call("get_battery_level", {})

        @function_tool
        async def get_current_time() -> str:
            """Get current local date and time."""
            return await _call("get_current_time", {})

        @function_tool
        async def set_volume(action: str) -> str:
            """Adjust TTS voice volume.

            Args:
                action: One of 'louder', 'quieter', 'max', 'normal'.
            """
            return await _call("set_volume", {"action": action})

        @function_tool
        async def set_pitch(action: str) -> str:
            """Adjust TTS voice pitch.

            Args:
                action: One of 'higher', 'lower', 'normal'. 'higher' raises the
                    pitch by +0.2 (max 2.0), 'lower' lowers by -0.2 (min 0.5),
                    'normal' resets to 1.0.
            """
            return await _call("set_pitch", {"action": action})

        return [get_robot_status, get_battery_level, get_current_time, set_volume, set_pitch]
