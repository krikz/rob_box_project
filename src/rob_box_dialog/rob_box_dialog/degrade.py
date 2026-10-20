"""Деградация провайдеров: режим команд и порядок (ADR-0150 §4.5, §8; карточка #3686).

Чистый Python, без ROS и без импорта адаптеров: пакет не знает ни одного провайдера, возможности приходят
утиным объектом (``Capabilities`` — подмножество ``rob_box_llm.provider.ProviderCapabilities``).
Режим команд — ветка кода по честно объявленным возможностям, а не правило промпта (ADR-0148).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Literal, Protocol, Sequence

CommandMode = Literal["structured_output", "tool_choice", "json_mode", "json_in_text"]

# Порядок §4.5: от самого строгого к самому слабому. Единственная таблица знания о режимах.
MODE_ORDER: tuple[CommandMode, ...] = ("structured_output", "tool_choice", "json_mode", "json_in_text")

ProviderKind = Literal["llm", "stt", "tts"]
ProviderHealth = Literal["ok", "degraded", "quota", "auth", "down"]

# Состояния, при которых провайдер ещё можно пробовать (в порядке предпочтения).
_USABLE_HEALTH: tuple[ProviderHealth, ...] = ("ok", "degraded")


class Capabilities(Protocol):
    """То, что degrade читает у адаптера (``ProviderCapabilities`` подходит как есть)."""

    tools: bool
    tool_choice: bool
    json_mode: bool
    structured_output: bool


@dataclass(frozen=True)
class ProviderState:
    """Состояние провайдера для latched ``/voice/providers/state`` (ADR §8)."""

    name: str
    kind: ProviderKind
    health: ProviderHealth = "ok"
    since: float = 0.0
    last_error_code: str | None = None


def select_mode(caps: Capabilities) -> CommandMode:
    """Выбрать форму ответа по возможностям адаптера (ADR §4.5)."""
    if caps.structured_output:
        return "structured_output"
    if caps.tool_choice and caps.tools:
        return "tool_choice"
    if caps.json_mode:
        return "json_mode"
    return "json_in_text"


def provider_order(states: Iterable[ProviderState], priority: Sequence[str]) -> tuple[str, ...]:
    """Порядок LLM-провайдеров: приоритет из конфига, здоровые впереди, мёртвые (quota/auth/down) отброшены.

    Провайдер без записи в ``states`` считается здоровым (состояние ещё не опубликовано).
    """
    health = {s.name: s.health for s in states if s.kind == "llm"}
    ranked: list[tuple[int, int, str]] = []
    for index, name in enumerate(priority):
        state = health.get(name, "ok")
        if state in _USABLE_HEALTH:
            ranked.append((_USABLE_HEALTH.index(state), index, name))
    ranked.sort()
    return tuple(name for _, _, name in ranked)
