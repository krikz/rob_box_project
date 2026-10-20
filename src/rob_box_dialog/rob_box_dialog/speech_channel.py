"""Канал речи: два транспорта одной ``Command`` для бейк-оффа (ADR-0150 §4.4, решение В1; карточка #3686).

``FieldChannel``  — речь в поле ``speech`` структурного ответа (tool_call ``command`` или JSON).
``ToolChannel``   — речь аргументом тула ``speak_text``; ``content`` игнорируется ВСЕГДА (правило кода).

Победителя выберет PR-1c, проигравший транспорт удаляется в PR-7 (останется один класс канала).

Каналы не знают ни ROS, ни адаптеров: строят ``ModeRequest`` (нейтральный запрос), принимают ``ModeReply``
и превращают его в текст JSON-команды, который разбирает ОДИН парсер (``command.parse``, PR-1), передаваемый снаружи.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, TypeVar

from rob_box_dialog.degrade import CommandMode

T = TypeVar("T")

COMMAND_TOOL = "command"
SPEAK_TOOL = "speak_text"

_JSON_INSTRUCTION = "Отвечай ровно одним JSON-объектом команды без пояснений вокруг."


@dataclass(frozen=True)
class ModeRequest:
    """Нейтральный запрос к адаптеру; соответствие полям ``LLMSettings`` делает вызывающий."""

    mode: CommandMode
    tools: tuple[Mapping[str, Any], ...] = ()
    tool_choice: str | None = None  # "required" | "auto" | None
    response_format: Mapping[str, Any] | None = None
    instruction: str = ""  # добавка к системному промпту (одинакова по смыслу во всех режимах)


@dataclass(frozen=True)
class ModeReply:
    """Нейтральный ответ адаптера: текст и вызовы тулов как ``(имя, аргументы)``."""

    content: str = ""
    tool_calls: tuple[tuple[str, Mapping[str, Any]], ...] = field(default_factory=tuple)


def _function_tool(name: str, description: str, parameters: Mapping[str, Any]) -> Mapping[str, Any]:
    return {"type": "function", "function": {"name": name, "description": description, "parameters": dict(parameters)}}


class FieldChannel:
    """Вариант A: речь — поле структурного ответа."""

    def __init__(self, schema: Mapping[str, Any]) -> None:
        self._schema = dict(schema)

    def build_request(self, mode: CommandMode) -> ModeRequest:
        if mode == "structured_output":
            fmt = {"type": "json_schema", "json_schema": {"name": COMMAND_TOOL, "schema": self._schema}}
            return ModeRequest(mode, response_format=fmt)
        if mode == "tool_choice":
            tool = _function_tool(COMMAND_TOOL, "Выдать команду роботу", self._schema)
            return ModeRequest(mode, tools=(tool,), tool_choice="required")
        if mode == "json_mode":
            return ModeRequest(mode, response_format={"type": "json_object"}, instruction=_JSON_INSTRUCTION)
        return ModeRequest(mode, instruction=_JSON_INSTRUCTION)

    def to_text(self, reply: ModeReply, mode: CommandMode) -> str:
        """Текст команды для парсера: аргументы тула ``command`` или ``content``."""
        if mode == "tool_choice":
            for name, args in reply.tool_calls:
                if name == COMMAND_TOOL:
                    return json.dumps(dict(args), ensure_ascii=False)
            return ""
        return reply.content

    def parse(self, reply: ModeReply, mode: CommandMode, parse: Callable[[str], T]) -> T:
        return parse(self.to_text(reply, mode))


class ToolChannel:
    """Вариант B: речь — аргумент тула ``speak_text``; ``content`` не читается никогда."""

    def __init__(self, action_tools: tuple[Mapping[str, Any], ...] = ()) -> None:
        self._action_tools = tuple(action_tools)

    def build_request(self, mode: CommandMode) -> ModeRequest:
        speak = _function_tool(
            SPEAK_TOOL,
            "Сказать фразу человеку",
            {"type": "object", "properties": {"text": {"type": "string", "maxLength": 280}}, "required": ["text"]},
        )
        # Принуждение к тулу только там, где адаптер его честно держит (ADR-0143); иначе "auto".
        choice = "required" if mode in ("structured_output", "tool_choice") else "auto"
        return ModeRequest(mode, tools=(speak, *self._action_tools), tool_choice=choice)

    def to_text(self, reply: ModeReply, mode: CommandMode) -> str:
        speech: str | None = None
        actions: list[dict[str, Any]] = []
        for name, args in reply.tool_calls:
            if name == SPEAK_TOOL:
                speech = speech if speech is not None else str(args.get("text", ""))
            else:
                actions.append({"tool": name, "args": dict(args)})
        payload: dict[str, Any] = {"actions": actions}
        if speech:
            payload["speech"] = speech
        return json.dumps(payload, ensure_ascii=False)

    def parse(self, reply: ModeReply, mode: CommandMode, parse: Callable[[str], T]) -> T:
        return parse(self.to_text(reply, mode))
