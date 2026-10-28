"""Канал речи — поле ``speech`` структурного ответа (ADR-0150 §4.4; решение О4 21.10.2026, бейк-офф PR-1c).

``FieldChannel`` строит запрос в режиме адаптера (``tool_call`` ``command`` / JSON-схема / JSON в тексте) и превращает
ответ в текст JSON-команды, который разбирает ОДИН парсер (``command.parse``). Проигравший транспорт — речь тулом
``speak_text`` (``ToolChannel``) — удалён в PR-7: ``content`` вне канала не озвучивается никогда.

Канал не знает ни ROS, ни адаптеров: строит ``ModeRequest`` (нейтральный запрос), принимает ``ModeReply``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, TypeVar

from rob_box_dialog.degrade import CommandMode

T = TypeVar("T")

COMMAND_TOOL = "command"

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
    """Речь — поле ``speech`` структурного ответа."""

    def __init__(self, schema: Mapping[str, Any]) -> None:
        self._schema = dict(schema)

    def build_request(self, mode: CommandMode) -> ModeRequest:
        if mode == "structured_output":
            fmt = {"type": "json_schema", "json_schema": {"name": COMMAND_TOOL, "schema": self._schema}}
            return ModeRequest(mode, response_format=fmt)
        if mode in ("tool_choice", "tool_offered"):
            tool = _function_tool(COMMAND_TOOL, "Выдать команду роботу", self._schema)
            return ModeRequest(mode, tools=(tool,), tool_choice="required" if mode == "tool_choice" else None)
        if mode == "json_mode":
            return ModeRequest(mode, response_format={"type": "json_object"}, instruction=_JSON_INSTRUCTION)
        return ModeRequest(mode, instruction=_JSON_INSTRUCTION)

    def to_text(self, reply: ModeReply, mode: CommandMode) -> str:
        """Текст команды для парсера: аргументы тула ``command`` или ``content``. Без принуждения (``tool_offered``)
        модель может ответить JSON-ом в тексте — его разбирает тот же парсер."""
        if mode in ("tool_choice", "tool_offered"):
            for name, args in reply.tool_calls:
                if name == COMMAND_TOOL:
                    return json.dumps(dict(args), ensure_ascii=False)
            return reply.content if mode == "tool_offered" else ""
        return reply.content

    def parse(self, reply: ModeReply, mode: CommandMode, parse: Callable[[str], T]) -> T:
        return parse(self.to_text(reply, mode))
