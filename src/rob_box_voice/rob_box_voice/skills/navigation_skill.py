"""
navigation_skill.py — NavigationSkill: sub-agent for robot movement, navigation and waypoint management.

Tools exposed to the sub-agent:
  navigate_to_waypoint   — travel to a named location (from DB)
  navigate_to_coordinates — travel to arbitrary (x, y, theta)
  move_direction          — move a short distance in a direction
  list_waypoints          — get all saved waypoints
  save_waypoint           — save current position as named waypoint
  delete_waypoint         — remove a named waypoint
  clear_waypoints         — remove all waypoints on the current map
  get_current_pose        — get robot's current position on the map
"""

from agents import function_tool

from .base_skill import BaseSkill


class NavigationSkill(BaseSkill):
    """Sub-agent that handles all navigation, movement and waypoint management.

    Uses navigate_to_waypoint, move_direction, save_waypoint, delete_waypoint,
    clear_waypoints, list_waypoints, get_current_pose and navigate_to_coordinates
    MCP tools.  The sub-agent waits for movement to complete before returning.
    """

    def _make_tools(self) -> list:
        async def _call(name, params, timeout=10.0):
            return await self._call(name, params, timeout)

        @function_tool
        async def navigate_to_waypoint(waypoint: str) -> str:
            """Navigate the robot to a named waypoint from the database.
            Blocks until arrival or failure.  Returns status string.
            Call list_waypoints first to see available waypoints.

            Args:
                waypoint: Named location (e.g. 'кухня', 'зал', 'спальня').
            """
            return await _call("navigate_to_waypoint", {"waypoint": waypoint}, timeout=130.0)

        @function_tool
        async def navigate_to_coordinates(x: float, y: float, theta: float = 0.0) -> str:
            """Navigate the robot to arbitrary coordinates in the map frame.
            Use after get_current_pose to return to a saved position.

            Args:
                x: X coordinate in metres.
                y: Y coordinate in metres.
                theta: Orientation in radians (default 0.0).
            """
            return await _call(
                "navigate_to_coordinates", {"x": x, "y": y, "theta": theta}, timeout=130.0
            )

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

        @function_tool
        async def list_waypoints() -> str:
            """Get a list of all saved waypoints on the current map.
            Returns waypoint names and coordinates.
            """
            return await _call("list_waypoints", {})

        @function_tool
        async def save_waypoint(name: str) -> str:
            """Save the robot's current position as a named waypoint.
            ALWAYS use when user names a place/location: 'это кухня', 'тут база', 'здесь зал', 'это его база', 'запомни это место'. NOT memory_save!

            Args:
                name: Human-friendly name for the waypoint (e.g. 'кухня', 'зал').
            """
            return await _call("save_waypoint", {"name": name})

        @function_tool
        async def delete_waypoint(name: str) -> str:
            """Delete a saved waypoint by name.
            Use when user says 'удали зал', 'забудь кухню'.

            Args:
                name: Name of the waypoint to delete.
            """
            return await _call("delete_waypoint", {"name": name})

        @function_tool
        async def clear_waypoints() -> str:
            """Delete ALL saved waypoints on the current map.
            Use when user says 'очисти все точки', 'удали все точки'.
            """
            return await _call("clear_waypoints", {})

        @function_tool
        async def get_current_pose() -> str:
            """Get the robot's current position (x, y, theta) on the map.
            Use before a 'go-speak-return' mission to remember the return position,
            or when user asks 'где ты?'.
            """
            return await _call("get_current_pose", {})

        @function_tool
        async def start_mapping(map_name: str = "") -> str:
            """Start new mapping session. Creates a backup of the current map and resets RTABMap.
            Use when user says 'начни карту', 'новая карта', 'начинаем маппинг', 'исследуй территорию'.

            Args:
                map_name: Optional name for the new map (e.g. 'квартира', 'офис').
            """
            params = {"map_name": map_name} if map_name else {}
            return await _call("start_mapping", params, timeout=15.0)

        @function_tool
        async def continue_mapping() -> str:
            """Continue an existing mapping session (resume RTABMap mapping mode).
            Use when user says 'продолжи маппинг', 'продолжай исследовать'.
            """
            return await _call("continue_mapping", {}, timeout=10.0)

        @function_tool
        async def finish_mapping() -> str:
            """Finish mapping and switch to localization mode.
            Use when user says 'закончи карту', 'хватит маппить', 'завершить исследование'.
            """
            return await _call("finish_mapping", {}, timeout=10.0)

        @function_tool
        async def speak_text(text: str, animation: str = "talking") -> str:
            """Make the robot say something out loud via TTS.
            Use for go-speak-return missions: navigate first, then speak_text at destination.
            Допустимые animation: idle talking wakeup sleep happy sad angry surprised thinking victory error excited confused neutral

            Args:
                text: Text to say (max 150 chars recommended).
                animation: LED animation to play while speaking (default 'talking').
            """
            return await _call("speak_text", {"text": text, "animation": animation})

        return [
            navigate_to_waypoint,
            navigate_to_coordinates,
            move_direction,
            list_waypoints,
            save_waypoint,
            delete_waypoint,
            clear_waypoints,
            get_current_pose,
            start_mapping,
            continue_mapping,
            finish_mapping,
            speak_text,
        ]

