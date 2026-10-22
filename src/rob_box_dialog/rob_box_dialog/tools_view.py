"""Срез тулов хода: LLM видит ≤ 12 схем по снимку, словам человека и агенту (ADR-0150 §4.2, §2.1 п.5; #3696).

Функция, а не правило в промпте и не догадка LLM (подгрузка скиллов моделью удалена): всегда —
``knowledge.TOOLS_ALWAYS`` (речь, память, время, поиск, анимация); группы ``knowledge.TOOL_GROUPS`` — по словам
реплики («спой», «голос», «поезжай»…) и по сценам снимка (кадр ``DJ`` и играющая музыка — музыкальные тулы); ни одна
группа не совпала — группы прошлого хода (продолжение разговора), иначе ``TOOL_GROUPS_DEFAULT``. В SILENCED
ничего звучащего: срез пуст (LLM в тишине не зовётся). Провайдер без тулов — пусто. Голос (``set_voice`` /
``set_tts_provider``) виден только при явной голосовой просьбе в этой же реплике (#3744).
"""

from __future__ import annotations

from typing import Any, List, Optional, Sequence, Tuple

from rob_box_core.tool_catalog import ToolCatalogEntry, get_tool
from rob_box_dialog import knowledge as kn
from rob_box_dialog.agents import agent_spec, tools_for
from rob_box_dialog.session import SessionSnapshot
from rob_box_dialog import phrases
from rob_box_dialog.words import has_word


def groups_for(snapshot: SessionSnapshot, text: str, carry: Sequence[str] = ()) -> Tuple[str, ...]:
    """Группы среза: сцены снимка и прямо названные (порядок ``TOOL_GROUPS``), затем названные только общим глаголом;
    ни одной — ``carry`` (группы прошлого хода: «я же просил про стрижей» — продолжение; кроме тех, что только по явной
    просьбе, ``TOOL_GROUPS_EXPLICIT``), иначе по умолчанию."""
    scene = {"music"} if snapshot.top("DJ") is not None or snapshot.music.state == "playing" else set()
    strong = _named(text, "stems") | scene
    weak = _named(text, "weak") - strong
    if not strong | weak:
        strong = {g for g in carry if g in kn.TOOL_GROUPS and g not in kn.TOOL_GROUPS_EXPLICIT}
        strong = strong or set(kn.TOOL_GROUPS_DEFAULT)
    return _ordered(strong) + _ordered(weak)


def requested_groups(text: str) -> Tuple[str, ...]:
    """Группы, названные словами человека в ЭТОЙ реплике: без сцен снимка, переноса и умолчания."""
    return _ordered(_named(text, "stems") | _named(text, "weak"))


def command_groups(text: str, called: Sequence[str]) -> Tuple[str, ...]:
    """Исполненная просьба человека: группы, названные его словами, в которых LLM вызвала тул-действие (класс
    ``TOOL_CLASSES`` не ``query``). Пусто — ход не команда.

    Решает запрос человека, тулы LLM — только подтверждение исполнения: чтение (статус, время, память, поиск,
    восприятие) и действие, о котором человек не просил (анимация к болтовне), командой ход не делают; слово группы
    в болтовне («узнал по голосу») без исполнения — тоже (В17, ADR-0148; живой робот 21.10).
    """
    actions = {t for t in called if kn.TOOL_CLASSES.get(t, "query") != "query"}
    return tuple(g for g in requested_groups(text) if actions & set(kn.TOOL_GROUPS[g].tools))


def motion_named(text: str) -> bool:
    """Назвал ли человек движение базы глаголом («поезжай», «двигайся», «повернись»). Восприятие («посмотри налево»)
    и голые направления движение не открывают; «повернись глазами» — взгляд, а не поворот базы (#3779)."""
    if has_word(text or "", kn.MOTION_VERB_STEMS):
        return True
    return has_word(text or "", kn.MOTION_TURN_STEMS) and not has_word(text or "", kn.MOTION_EYES_STEMS)


def motion_refusal(tool: str, text: str) -> Optional[str]:
    """Честная фраза отказа, если тул движения вызван в ходе, где человек движение не называл; иначе ``None``.

    Страховка исполнителя поверх среза: что бы ни написала LLM, база без глагола движения не поедет. Пустая реплика
    (вызов не по голосу человека) отказа не даёт — судить не по чему."""
    if tool not in kn.MOTION_TOOLS or not (text or "").strip() or motion_named(text):
        return None
    return phrases.text("motion.rejected.not_asked")


def _named(text: str, field: str) -> set:
    return {name for name, group in kn.TOOL_GROUPS.items() if has_word(text or "", getattr(group, field))}


def _ordered(names: set) -> Tuple[str, ...]:
    return tuple(name for name in kn.TOOL_GROUPS if name in names)


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
    drive = motion_named(text)
    for name in kn.TOOLS_ALWAYS + tuple(t for g in groups_for(snapshot, text, carry) for t in kn.TOOL_GROUPS[g].tools):
        if name in kn.MOTION_TOOLS and not drive:
            continue
        if name in allowed and name not in names and len(names) < kn.TOOLS_VISIBLE_MAX:
            names.append(name)
    return tuple(get_tool(name) for name in names)


__all__ = ["available_tools", "command_groups", "groups_for", "motion_named", "motion_refusal", "requested_groups"]
