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
        """Override: anti-hallucination verification for save_waypoint AND navigate_to_waypoint.

        Strategy: run sub-agent, then check result.new_items to see which tools
        were actually called (via OpenAI Agents SDK ToolCallItem).
        - Save intent without save_waypoint call → call save_waypoint directly.
        - Navigate intent without navigate_to_waypoint/navigate_to_coordinates call
          → extract waypoint name and call navigate_to_waypoint directly.
        """
        agent = self.build_agent()
        max_turns = self._agent_max_turns
        skill_name = self._name
        adapter = self._adapter

        # ── Intent keyword tables ────────────────────────────────────────────
        _SAVE_KWS = [
            "сохрани", "запомни", "отметь", "назови", "добавь", "поставь",
            "пометь", "запиши как", "save", "mark", "add",
            "это кухня", "это зал", "это спальня", "это база", "это коридор",
            "это прихожая", "это туалет", "тут база", "здесь", "это место",
            "это точка", "добавь точку",
        ]
        _NAVIGATE_KWS = [
            "едь", "поехай", "поезжай", "езжай", "отвези", "навигируй",
            "доедь", "едем на", "едет на", "navigate to", "go to",
        ]

        # ── Name extraction ──────────────────────────────────────────────────
        # "добавь точку база", "сохрани как кухня", "это зал", "едь на базу"
        _NAME_RE = re.compile(
            r"(?:точку?\s+|waypoint\s+|как\s+|as\s+|на\s+|:\s*)[\"']?([\w\-а-яА-ЯёЁ]+)[\"']?",
            re.IGNORECASE,
        )
        _LAST_WORD_RE = re.compile(r"[\"']?([\w\-а-яА-ЯёЁ]{1,})[\"']?\s*$", re.IGNORECASE)
        _NOISE = {"как", "as", "место", "точку", "точка", "здесь", "тут", "это",
                  "на", "к", "в", "and", "и"}

        def _has_save_intent(task: str) -> bool:
            t = task.lower()
            return any(kw in t for kw in _SAVE_KWS)

        def _has_navigate_intent(task: str) -> bool:
            t = task.lower()
            return any(kw in t for kw in _NAVIGATE_KWS)

        def _extract_name(task: str) -> str | None:
            for m in _NAME_RE.finditer(task):
                n = m.group(1).strip()
                if n.lower() not in _NOISE and len(n) >= 1:
                    return n
            m = _LAST_WORD_RE.search(task)
            if m:
                n = m.group(1).strip()
                if n.lower() not in _NOISE and len(n) >= 1:
                    return n
            return None

        def _direct_call(tool: str, params: dict, timeout: float = 10.0) -> str:
            """Call MCP tool directly via adapter (sync, for executor)."""
            res = adapter.execute_tool_call_sync(tool, params, timeout=timeout)
            if isinstance(res, dict):
                return res.get("result", json.dumps(res, ensure_ascii=False))
            return str(res)

        def _tools_called(run_result) -> set[str]:
            """Return set of tool names actually invoked by the sub-agent."""
            called = set()
            try:
                from agents.items import ToolCallItem
                for item in run_result.new_items:
                    if isinstance(item, ToolCallItem):
                        called.add(item.raw_item.name)
            except Exception:
                pass
            return called

        async def _run_skill(task: str) -> str:
            from openai import APIConnectionError

            loop = asyncio.get_running_loop()
            is_save = _has_save_intent(task)
            is_nav = _has_navigate_intent(task)

            # ── Run sub-agent ─────────────────────────────────────────────────
            run_result = None
            output = ""
            for attempt in range(3):
                try:
                    run_result = await Runner.run(agent, input=task, max_turns=max_turns)
                    output = run_result.final_output or ""
                    break
                except APIConnectionError as exc:
                    if attempt < 2:
                        delay = 2.0 * (2 ** attempt)
                        logger.warning(
                            f"⚠️ {skill_name} APIConnectionError (attempt {attempt + 1}/3), "
                            f"retrying in {delay:.0f}s: {exc}"
                        )
                        await asyncio.sleep(delay)
                    else:
                        return f"Ошибка связи с LLM: {exc}"

            if run_result is None:
                return output

            actually_called = _tools_called(run_result)
            logger.debug(f"[NavigationSkill] tools actually called: {actually_called}")

            # ── Verify save_waypoint ──────────────────────────────────────────
            if is_save and "save_waypoint" not in actually_called:
                name = _extract_name(task)
                logger.warning(
                    f"⚠️ NavigationSkill hallucination (save): sub-agent skipped "
                    f"save_waypoint. Extracted name={name!r}. Direct call via adapter."
                )
                if not name:
                    return "⚠️ Не смог определить название точки. Повтори: 'добавь точку база'."
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

            # ── Verify navigate_to_waypoint ───────────────────────────────────
            nav_tools = {"navigate_to_waypoint", "navigate_to_coordinates", "move_direction"}
            if is_nav and not (actually_called & nav_tools):
                name = _extract_name(task)
                logger.warning(
                    f"⚠️ NavigationSkill hallucination (navigate): sub-agent skipped "
                    f"navigation tools. Extracted name={name!r}. Direct call via adapter."
                )
                if not name:
                    return "⚠️ Не смог определить точку назначения. Скажи: 'едь на база'."
                rescue = await loop.run_in_executor(
                    None, lambda: _direct_call("navigate_to_waypoint", {"waypoint": name}, timeout=130.0)
                )
                logger.info(f"🧭 Direct navigate_to_waypoint('{name}'): {rescue}")
                return str(rescue)

            return output

        async def _nav_skill_tool(task: str) -> str:
            return await _run_skill(task)

        return function_tool(_nav_skill_tool, name_override=tool_name, description_override=tool_description)

