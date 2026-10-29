"""Варианты «кто понимает фразу» для сухого стенда (карточка #4163): код по словам или модель.

Только стенд: боевой код (``src/``) не меняется. Движок хода зовёт ``match`` (Tier-1) и ``available_tools`` (срез тулов)
как имена модуля :mod:`rob_box_dialog.turn`; стенд на время одной реплики (:func:`installed`) подставляет туда обёртки.
Какие слова остаются у кода, решает таблица :data:`VARIANTS`, а не текст промпта.

* ``A`` - как на роботе сейчас: Tier-1 целиком, срез тулов по словам фразы (``tools_view.groups_for``) и снимку.
* ``B`` - понимает модель. Tier-1 только безопасность (:data:`SAFETY_KINDS`). Срез тулов - ТОЛЬКО по снимку состояния:
  тишина (``Silence``) - пусто, иначе все тулы, которые срез A вообще когда-либо показывает (``TOOLS_ALWAYS`` + группы
  ``TOOL_GROUPS``), без ограничения ``TOOLS_VISIBLE_MAX`` и без движения базы. Слова фразы срез не смотрят. Какой тул
  звать, решает модель той же командой JSON.
* ``Bn`` - узкое прочтение B: настоящий ``available_tools`` с пустым текстом (``TOOLS_ALWAYS`` + группа по умолчанию +
  музыка, если играет; ≤ ``TOOLS_VISIBLE_MAX``). Модель не видит тулов вне этого среза. Нужен, если Шифу читает «срез
  только по состоянию» буквально; в замерах этапа 1 не участвует.
* ``C`` - понимает классификатор (Jev): Tier-1 как в B; группу тулов выбирает ``jev_router`` перед ходом. Уверенность
  не ниже порога - срез = ``TOOLS_ALWAYS`` + выбранная группа + сцена снимка (играет музыка), ≤ ``TOOLS_VISIBLE_MAX``;
  ниже порога или сбой Jev - срез B.

Движение базы (``knowledge.MOTION_TOOLS``) в B, Bn и C скрыто: его гард - глагол человека (``tools_view.motion_named``) и
отказ исполнителя (``tools_view.motion_refusal``); оба остаются словесными во всех вариантах, это решение безопасности,
а не понимания.
"""

from __future__ import annotations

import contextlib
from typing import Any, Callable, Iterator, List, Optional, Sequence, Tuple

from rob_box_core.tool_catalog import ToolCatalogEntry, get_tool
from rob_box_dialog import knowledge as kn
from rob_box_dialog import tools_view
from rob_box_dialog import turn as turn_mod
from rob_box_dialog.agents import agent_spec, tools_for
from rob_box_dialog.grammar import match as real_match

A, B, BN, C = "A", "B", "Bn", "C"
VARIANTS: Tuple[str, ...] = (A, B, BN, C)
#: Виды Tier-1 (``knowledge.TIER1_KINDS``), которые остаются у кода в B, Bn и C: то, что нельзя доверить пониманию
#: модели. Остальные виды (громкость, смена голоса, шаг темпа и тона, «говори нормально», очередь показа лица, «повтори»,
#: «да/нет» на вопрос робота) в этих вариантах решает модель.
#:   stop_all, stop_speech - «стоп», «хватит»: остановка звука и движения, «хватит читать» (гаснет речь, музыка нет);
#:   silence, unsilence    - тишина и «Робот, говори»: без второго нет выхода из тишины, без первого нет «замолчи»;
#:   new_session           - «забудь», «начнём сначала»: сброс сессии, чистит память о недавнем.
SAFETY_KINDS = frozenset({"stop_all", "stop_speech", "silence", "unsilence", "new_session"})

#: ``available_tools(snapshot, caps, agent, text, carry, asked)`` - подпись среза, как в ``tools_view``.
Slicer = Callable[..., Tuple[ToolCatalogEntry, ...]]
Matcher = Callable[..., Any]
#: Выбор группы на ход (вариант C): ``(группа | "none", принято ли по порогу)``; ``None`` - выбора нет.
RouteOf = Callable[[], Optional[Tuple[str, bool]]]
#: Выбор «группа тулов не нужна» в вопросе классификатору.
NO_GROUP = "none"


def check(variant: str) -> str:
    if variant not in VARIANTS:
        raise ValueError(f"вариант {variant!r}: известны {', '.join(VARIANTS)}")
    return variant


# ── Tier-1 ────────────────────────────────────────────────────────────────────────────────────────────────────────


def safety_match(real: Matcher = real_match) -> Matcher:
    """``grammar.match``, у которого остались только виды безопасности (:data:`SAFETY_KINDS`)."""

    def match(text: str, snapshot: Any, voices: Sequence[Any] = ()) -> Any:
        command = real(text, snapshot, voices)
        return command if command is not None and command.kind in SAFETY_KINDS else None

    return match


# ── Срез тулов ────────────────────────────────────────────────────────────────────────────────────────────────────


def _closed(snapshot: Any, caps: Any) -> bool:
    """Срез пуст по состоянию: тишина (LLM в ней не зовётся) или провайдер без тулов - то же, что в ``available_tools``."""
    return snapshot.top("Silence") is not None or (caps is not None and not getattr(caps, "tools", True))


def _allowed(agent: str) -> set:
    return set(tools_for(agent_spec(agent))) | set(kn.SPEECH_CHANNEL_TOOLS)


def _names(candidates: Sequence[str], agent: str, cap: Optional[int]) -> Tuple[ToolCatalogEntry, ...]:
    """Тулы по порядку кандидатов: доступные агенту, без движения базы, без повторов, не больше ``cap``."""
    allowed, names = _allowed(agent), []
    for name in candidates:
        if name in kn.MOTION_TOOLS or name not in allowed or name in names or (cap is not None and len(names) >= cap):
            continue
        names.append(name)
    return tuple(get_tool(name) for name in names)


def full_slice() -> Slicer:
    """Срез B: все тулы, которые срез A показывает в каком-то ходе, - по состоянию, без слов и без ограничения числа."""

    def available(snapshot: Any, caps: Any = None, agent: str = kn.AGENTS[0], text: str = "",
                  carry: Sequence[str] = (), asked: Sequence[str] = ()) -> Tuple[ToolCatalogEntry, ...]:
        if _closed(snapshot, caps):
            return ()
        grouped = tuple(t for group in kn.TOOL_GROUPS.values() for t in group.tools)
        return _names(kn.TOOLS_ALWAYS + kn.ALOUD_TOOLS + grouped, agent, None)

    return available


def narrow_slice(real: Slicer = tools_view.available_tools) -> Slicer:
    """Срез Bn: настоящий ``available_tools`` с пустым текстом - слова фразы не доходят до него."""

    def available(snapshot: Any, caps: Any = None, agent: str = kn.AGENTS[0], text: str = "",
                  carry: Sequence[str] = (), asked: Sequence[str] = ()) -> Tuple[ToolCatalogEntry, ...]:
        return real(snapshot, caps, agent, "", carry, asked)

    return available


def routed_slice(route_of: RouteOf, wide: Slicer, narrow: Slicer) -> Slicer:
    """Срез C: выбор классификатора, принятый по порогу, - ``TOOLS_ALWAYS`` + группа + сцена снимка; иначе срез B."""

    def available(snapshot: Any, caps: Any = None, agent: str = kn.AGENTS[0], text: str = "",
                  carry: Sequence[str] = (), asked: Sequence[str] = ()) -> Tuple[ToolCatalogEntry, ...]:
        if _closed(snapshot, caps):
            return ()
        route = route_of()
        if route is None or not route[1]:
            return wide(snapshot, caps, agent, "", carry, asked)
        group = route[0]
        if group == NO_GROUP or group not in kn.TOOL_GROUPS:
            return narrow(snapshot, caps, agent, "", carry, asked)
        scene = kn.TOOL_GROUPS["music"].tools if snapshot.music_playing and group != "music" else ()
        chosen = kn.TOOL_GROUPS[group].tools
        return _names(kn.TOOLS_ALWAYS + kn.ALOUD_TOOLS + chosen + scene, agent, kn.TOOLS_VISIBLE_MAX)

    return available


# ── Подмена на время реплики ──────────────────────────────────────────────────────────────────────────────────────


def _recording(inner: Slicer, offered: List[Tuple[str, ...]]) -> Slicer:
    """Срез как есть, но выданные имена тулов складываются в ``offered`` (для отчёта: что видела модель)."""

    def available(*args: Any, **kwargs: Any) -> Tuple[ToolCatalogEntry, ...]:
        tools = inner(*args, **kwargs)
        offered.append(tuple(t.name for t in tools))
        return tools

    return available


@contextlib.contextmanager
def installed(variant: str, offered: List[Tuple[str, ...]], route_of: RouteOf = lambda: None) -> Iterator[None]:
    """Подставить в движок хода ``match`` и ``available_tools`` варианта на время реплики; выдачи среза - в ``offered``.

    Подмена затрагивает модуль целиком, поэтому две реплики разных вариантов одновременно идти не могут: вход при
    уже стоящей подмене - ``RuntimeError`` (стенд ходит по одной реплике, замер - вариант за вариантом)."""
    check(variant)
    if getattr(turn_mod, "_understanding_variant", None) is not None:
        raise RuntimeError(f"подмена варианта {turn_mod._understanding_variant!r} уже стоит")  # noqa: SLF001
    saved = turn_mod.match, turn_mod.available_tools
    real_slice = saved[1]
    if variant == A:
        slicer, matcher = real_slice, saved[0]
    elif variant == B:
        slicer, matcher = full_slice(), safety_match()
    elif variant == BN:
        slicer, matcher = narrow_slice(real_slice), safety_match()
    else:
        slicer, matcher = routed_slice(route_of, full_slice(), narrow_slice(real_slice)), safety_match()
    turn_mod.match, turn_mod.available_tools = matcher, _recording(slicer, offered)
    turn_mod._understanding_variant = variant  # noqa: SLF001
    try:
        yield
    finally:
        turn_mod.match, turn_mod.available_tools = saved
        turn_mod._understanding_variant = None  # noqa: SLF001


__all__ = ["A", "B", "BN", "C", "NO_GROUP", "SAFETY_KINDS", "VARIANTS", "check", "full_slice", "installed",
           "narrow_slice", "routed_slice", "safety_match"]
