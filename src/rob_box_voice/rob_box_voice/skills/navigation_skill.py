"""
navigation_skill.py — NavigationSkill: sub-agent for robot movement and navigation.

Tools exposed to the sub-agent:
  navigate_to_waypoint  — travel to a named location
  move_direction        — move a short distance in a direction
"""

import asyncio

from agents import function_tool

from .base_skill import BaseSkill


class NavigationSkill(BaseSkill):
    """Sub-agent that handles all navigation and movement requests.

    Uses navigate_to_waypoint and move_direction MCP tools.  The sub-agent
    waits for movement to complete before returning.
    """

    def _make_tools(self) -> list:
        async def _call(name, params, timeout=10.0):
            return await self._call(name, params, timeout)

        @function_tool
        async def navigate_to_waypoint(waypoint: str) -> str:
            """Navigate the robot to a named waypoint.
            Blocks until arrival or failure.  Returns status string.

            Args:
                waypoint: Named location (e.g. 'kitchen', 'entrance', 'charging_dock').
            """
            return await _call("navigate_to_waypoint", {"waypoint": waypoint}, timeout=130.0)

        @function_tool
        async def move_direction(direction: str, distance: float = 0.5) -> str:
            """Move the robot a short distance.

            Args:
                direction: One of 'вперёд', 'назад', 'налево', 'направо'.
                distance:  Distance in metres (default 0.5 m).
            """
            return await _call(
                "move_direction", {"direction": direction, "distance": distance}, timeout=70.0
            )

        return [navigate_to_waypoint, move_direction]
