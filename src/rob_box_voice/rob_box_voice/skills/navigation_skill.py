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

import asyncio
import json
import logging
import re

from agents import Agent, Runner, function_tool
from agents.model_settings import ModelSettings

from .base_skill import BaseSkill

logger = logging.getLogger(__name__)


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

    def as_tool(self, tool_name: str, tool_description: str):
        """Override: adds anti-hallucination verification for save_waypoint tasks.

        Strategy: for save-intent tasks, compare list_waypoints count before/after
        sub-agent run. If count is unchanged despite the agent claiming success,
        the LLM hallucinated — extract the waypoint name from the task and call
        save_waypoint directly via the adapter.
        """
        agent = self.build_agent()
        max_turns = self._agent_max_turns
        skill_name = self._name
        adapter = self._adapter

        # Keywords that indicate a save-waypoint intent
        _SAVE_KWS = [
            "сохрани", "запомни", "отметь", "назови", "save", "mark",
            "это кухня", "это зал", "это спальня", "это база", "это коридор",
            "тут база", "здесь", "это место", "это точка",
        ]

        # Try to extract the canonical waypoint name from the task text
        _NAME_RE = re.compile(
            r"(?:как|as|:)\s*[\"']?([\w\-а-яА-ЯёЁ]+)[\"']?",
            re.IGNORECASE,
        )
        # Fallback: last capitalized / cyrillic word cluster
        _LAST_WORD_RE = re.compile(r"[\"']?([\w\-а-яА-ЯёЁ]{2,})[\"']?\s*$", re.IGNORECASE)

        def _has_save_intent(task: str) -> bool:
            t = task.lower()
            return any(kw in t for kw in _SAVE_KWS)

        def _extract_name(task: str) -> str | None:
            m = _NAME_RE.search(task)
            if m:
                return m.group(1).strip()
            m = _LAST_WORD_RE.search(task)
            if m:
                n = m.group(1).strip()
                if n.lower() not in ("как", "as", "место", "точку", "точка", "здесь", "тут"):
                    return n
            return None

        def _count_waypoints(raw: str) -> int:
            """Heuristic: count waypoint entries in list_waypoints output."""
            raw = raw.strip()
            if not raw or "нет" in raw.lower() or "пусто" in raw.lower() or raw == "[]":
                return 0
            # Count lines/entries separated by newlines or commas
            if "\n" in raw:
                return len([ln for ln in raw.splitlines() if ln.strip()])
            return len([x for x in raw.split(",") if x.strip()])

        def _direct_call(tool: str, params: dict, timeout: float = 10.0) -> str:
            """Call MCP tool directly via adapter (sync, for executor)."""
            result = adapter.execute_tool_call_sync(tool, params, timeout=timeout)
            if isinstance(result, dict):
                return result.get("result", json.dumps(result, ensure_ascii=False))
            return str(result)

        async def _run_skill(task: str) -> str:
            from openai import APIConnectionError

            loop = asyncio.get_running_loop()
            is_save = _has_save_intent(task)

            # ── Baseline snapshot before sub-agent runs ──────────────────────
            count_before = 0
            if is_save:
                try:
                    wp_before = await loop.run_in_executor(
                        None, lambda: _direct_call("list_waypoints", {}, timeout=5.0)
                    )
                    count_before = _count_waypoints(wp_before)
                except Exception as e:
                    logger.warning(f"[NavigationSkill] list_waypoints (before) failed: {e}")

            # ── Run sub-agent ─────────────────────────────────────────────────
            last_exc = None
            output = ""
            for attempt in range(3):
                try:
                    result = await Runner.run(agent, input=task, max_turns=max_turns)
                    output = result.final_output or ""
                    break
                except APIConnectionError as exc:
                    last_exc = exc
                    if attempt < 2:
                        delay = 2.0 * (2 ** attempt)
                        logger.warning(
                            f"⚠️ {skill_name} APIConnectionError (attempt {attempt + 1}/3), "
                            f"retrying in {delay:.0f}s: {exc}"
                        )
                        await asyncio.sleep(delay)
                    else:
                        return f"Ошибка связи с LLM: {exc}"

            if not is_save:
                return output

            # ── Post-run verification ─────────────────────────────────────────
            try:
                wp_after = await loop.run_in_executor(
                    None, lambda: _direct_call("list_waypoints", {}, timeout=5.0)
                )
                count_after = _count_waypoints(wp_after)
            except Exception as e:
                logger.warning(f"[NavigationSkill] list_waypoints (after) failed: {e}")
                return output

            if count_after > count_before:
                # Sub-agent actually saved something — all good
                return output

            # ── Hallucination detected ────────────────────────────────────────
            name = _extract_name(task)
            logger.warning(
                f"⚠️ NavigationSkill hallucination detected: count_before={count_before} "
                f"count_after={count_after}, extracted name={name!r}. "
                f"Calling save_waypoint directly."
            )
            if not name:
                return (
                    f"⚠️ Не смог определить название точки из задания. "
                    f"Пожалуйста, уточни название (например: 'запомни как база')."
                )

            rescue = await loop.run_in_executor(
                None, lambda: _direct_call("save_waypoint", {"name": name}, timeout=10.0)
            )
            logger.info(f"💾 Direct save_waypoint('{name}'): {rescue}")

            if "недоступен" in rescue or "неизвестна" in rescue or "error" in rescue.lower():
                return (
                    f"⚠️ Не удалось сохранить точку '{name}': {rescue}\n"
                    f"Убедись что rtabmap запущен и TF map→base_link доступен."
                )

            return f"✅ Точка '{name}' сохранена: {rescue}"

        async def _nav_skill_tool(task: str) -> str:
            return await _run_skill(task)

        return function_tool(_nav_skill_tool, name_override=tool_name, description_override=tool_description)

