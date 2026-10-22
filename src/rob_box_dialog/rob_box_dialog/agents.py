"""Спецификации агентов на общем движке (ADR-0150 §9): Личность, оператор (ТАРС), телеграм.

Один движок — три спецификации: какая закрытая грамматика Tier-1 доступна, какой срез каталога тулов, какой промпт
и куда по умолчанию звучат фразы. Значения — стартовые; уточняют PR-6 (грамматика), PR-12a (телеграм), PR-13 (ТАРС).
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Callable, Mapping, Optional, Tuple

from rob_box_core.tool_catalog import ToolCatalogEntry, llm_visible_tools, operator_visible_tools
from rob_box_core.utterance import Sink
from rob_box_dialog import knowledge as kn

#: Группы закрытой грамматики Tier-1 (§4.2); разбор — ``grammar.py`` (PR-6).
GRAMMAR_GROUPS: Tuple[str, ...] = ("stop", "silence", "volume", "voice", "new_session", "dj", "yes_no")
#: Вид команды Tier-1 (``grammar.match``, ``knowledge.TIER1_KINDS``) → группа грамматики агента.
TIER1_GROUPS: Mapping[str, str] = MappingProxyType({
    "stop_all": "stop", "silence": "silence", "unsilence": "silence", "volume": "volume", "set_voice": "voice",
    "new_session": "new_session", "answer": "yes_no",
})

_VIEWS: Mapping[str, Callable[[], Tuple[ToolCatalogEntry, ...]]] = MappingProxyType({
    "llm": llm_visible_tools,
    "operator": operator_visible_tools,
})


@dataclass(frozen=True)
class AgentSpec:
    """Спецификация агента: ``grammar`` — группы Tier-1, ``tools_view`` — срез каталога (``llm``/``operator``),
    ``prompt`` — имя системного промпта, ``sink`` — куда звучат фразы; ``None`` — текстовый канал агента."""

    name: str
    grammar: Tuple[str, ...]
    tools_view: str
    prompt: str
    sink: Optional[Sink]


#: Тишина «замолчи/помолчи» — только у Личности (решение О2): оператор и телеграм ею не блокируются.
AGENT_SPECS: Mapping[str, AgentSpec] = MappingProxyType({
    "personality": AgentSpec("personality", GRAMMAR_GROUPS, "llm", "personality", Sink.SPEAKERS),
    "operator": AgentSpec("operator", ("stop", "volume", "yes_no"), "operator", "operator", Sink.HEADSET),
    "telegram": AgentSpec("telegram", ("stop", "new_session"), "llm", "telegram", None),
})


def agent_spec(name: str) -> AgentSpec:
    """Спецификация агента по имени (``knowledge.AGENTS``); чужое имя — ``KeyError``."""
    return AGENT_SPECS[name]


def allows_tier1(spec: AgentSpec, kind: str) -> bool:
    """Доступна ли агенту команда Tier-1 этого вида: «замолчи» в чате — не тишина Личности (О2), а слова для LLM."""
    return TIER1_GROUPS.get(kind) in spec.grammar


def tools_for(spec: AgentSpec) -> Tuple[str, ...]:
    """Тулы среза агента, у которых есть класс действия и этот класс агенту доступен (``say`` — не Личности)."""
    return tuple(
        t.name for t in _VIEWS[spec.tools_view]()
        if t.name in kn.TOOL_CLASSES and spec.name in kn.ACTION_CLASSES[kn.TOOL_CLASSES[t.name]].agents
    )
