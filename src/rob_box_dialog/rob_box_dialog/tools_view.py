"""Срез тулов хода: LLM видит ≤ 12 схем по снимку, словам человека и агенту (ADR-0150 §4.2, §2.1 п.5; #3696).

Функция, а не правило в промпте и не догадка LLM (подгрузка скиллов моделью удалена): всегда —
``knowledge.TOOLS_ALWAYS`` (речь, память, время, поиск, анимация); группы ``knowledge.TOOL_GROUPS`` — по словам
реплики («спой», «голос», «поезжай»…) и по сценам снимка (кадр ``DJ`` и играющая музыка — музыкальные тулы); ни одна
группа не совпала — группы прошлого хода (продолжение разговора), иначе ``TOOL_GROUPS_DEFAULT``. В SILENCED ничего звучащего: срез пуст (LLM в тишине не зовётся). Провайдер без тулов —
пусто. Голос (``set_voice``/``set_tts_provider``) виден только при явной голосовой просьбе (#3744).
"""

from __future__ import annotations

from typing import Any, List, Optional, Sequence, Tuple

from rob_box_core.tool_catalog import ToolCatalogEntry, get_tool
from rob_box_dialog import knowledge as kn
from rob_box_dialog.agents import agent_spec, tools_for
from rob_box_dialog.session import SessionSnapshot
from rob_box_dialog.words import has_word


def groups_for(snapshot: SessionSnapshot, text: str, carry: Sequence[str] = ()) -> Tuple[str, ...]:
    """Группы среза: сцены снимка и прямо названные (порядок ``TOOL_GROUPS``), затем названные только общим глаголом;
    ни одной — ``carry`` (группы прошлого хода: «я же просил про стрижей» — продолжение), иначе по умолчанию."""
    scene = {"music"} if snapshot.music.dj or snapshot.music.state == "playing" else set()
    strong = {name for name, g in kn.TOOL_GROUPS.items() if has_word(text or "", g.stems)} | scene
    weak = {name for name, g in kn.TOOL_GROUPS.items() if g.weak and has_word(text or "", g.weak)} - strong
    if not strong | weak:
        strong = {g for g in carry if g in kn.TOOL_GROUPS} or set(kn.TOOL_GROUPS_DEFAULT)
    return tuple(n for n in kn.TOOL_GROUPS if n in strong) + tuple(n for n in kn.TOOL_GROUPS if n in weak)


def available_tools(snapshot: SessionSnapshot, caps: Optional[Any] = None, agent: str = "personality",
                    text: str = "", carry: Sequence[str] = ()) -> Tuple[ToolCatalogEntry, ...]:
    """Тулы хода (≤ ``TOOLS_VISIBLE_MAX``) из среза агента: всегда-видимые, затем группы по приоритету.

    ``caps`` — возможности провайдера (``tools=False`` — тулов нет вовсе); ``None`` — не известны, тулы есть.
    ``carry`` — группы прошлого хода (см. :func:`groups_for`).
    """
    if snapshot.top("Silence") is not None or (caps is not None and not getattr(caps, "tools", True)):
        return ()
    allowed = set(tools_for(agent_spec(agent))) | set(kn.SPEECH_CHANNEL_TOOLS)
    names: List[str] = []
    for name in kn.TOOLS_ALWAYS + tuple(t for g in groups_for(snapshot, text, carry) for t in kn.TOOL_GROUPS[g].tools):
        if name in allowed and name not in names and len(names) < kn.TOOLS_VISIBLE_MAX:
            names.append(name)
    return tuple(get_tool(name) for name in names)


__all__ = ["available_tools", "groups_for"]
