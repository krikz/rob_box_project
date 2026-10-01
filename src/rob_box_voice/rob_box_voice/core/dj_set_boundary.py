"""ADR-0129 (issue #3000) — граница DJ-сета: окно разговора и штамп ``<dj_state>``.

Смена сета — новый сет, смена персоны или темы внутри сета, конец сета —
это новая «вечеринка». Ходы прошлых сетов в окне разговора перебивали
новую: живой прогон 01.10 10:47 — сет «ВосьмиБитный монмтр / денди»
закончился, реплика «Ты Диджй Ускоглазый … вечеринка любителей Азидтской
музыки» пошла в LLM (опечатка «Диджй» мимо роутера медиакоманд), и модель
вызвала ``set_dj_mode(theme='вечеринка любителей денди',
persona='ВосьмиБитный монмтр')`` — тему и персону прошлого сета из окна.

Два слоя, оба вне классов ноды/контроллера/ядра (ADR-0145):

1. :class:`DJSetBoundary` замечает смену ``(enabled, persona, theme)`` и на
   ближайшем ходе (:func:`settle_dj_set_boundary`, в asyncio-цикле, до
   сборки истории) вычищает из окна обмены прошлых сетов — ходы, где
   вызван ``set_dj_mode``. Пока сет идёт, последний такой обмен (тот, что
   включил текущий сет) остаётся. Отложено до хода, а не сделано в
   колбэке топика: колбэк живёт в потоке ROS, ход LLM — в asyncio-цикле,
   и ход роутера записывается в окно позже топика (гонка).
2. :func:`dj_state_lines` — блок ``<dj_state>`` в ``<system_context>``
   каждого хода: текущий сет (тема, персона) или «сет не идёт», и правило
   «прошлые сеты завершены, их тему и персону не бери».
"""

from __future__ import annotations

import logging
from typing import Any, Callable, Iterable, List, Optional, Tuple

#: Тул, которым включается/меняется сет. Обмен (реплика + ответ), где он
#: вызван, — граница сета в окне разговора.
SET_DJ_MODE_TOOL = "set_dj_mode"

_OFF_KEY: Tuple[bool, str, str] = (False, "", "")


def set_key(state: Any) -> Tuple[bool, str, str]:
    """``(enabled, persona, theme)`` — то, что делает сет «этим» сетом.

    План, счётчик переходов, лимиты — не граница: DJ_AUTO повторяет
    ``set_dj_mode`` с теми же темой и персоной на каждом переходе.
    """
    if state is None:
        return _OFF_KEY
    return (
        bool(getattr(state, "enabled", False)),
        str(getattr(state, "persona", "") or ""),
        str(getattr(state, "theme", "") or ""),
    )


class DJSetBoundary:
    """Замечает смену сета; окно чистит ближайший ход (см. модуль)."""

    def __init__(self) -> None:
        self._seen: Tuple[bool, str, str] = _OFF_KEY
        self._pending = False
        self._had_set = False

    @property
    def had_set(self) -> bool:
        """Был ли хоть один сет — без него ``<dj_state>`` не нужен."""
        return self._had_set

    @property
    def pending(self) -> bool:
        return self._pending

    def observe(self, state: Any) -> bool:
        """Сверить состояние DJ с последним виденным. ``True`` — граница."""
        key = set_key(state)
        self._had_set = self._had_set or key[0]
        if key == self._seen:
            return False
        self._seen = key
        self._pending = True
        return True

    def take(self, state: Any) -> bool:
        """Забрать отложенную границу (сначала сверив состояние)."""
        self.observe(state)
        pending, self._pending = self._pending, False
        return pending


def apply_dj_mode_message(
    boundary: Optional[DJSetBoundary],
    dj: Any,
    payload: str,
    *,
    raw_utterance: str = "",
) -> None:
    """``/voice/dj_mode`` → контроллер, со сверкой границы до и после.

    «До» ловит выключение мимо топика (``reset_silently`` по стоп-команде,
    #2897): эхо ``enabled=false`` контроллер уже не меняет.
    """
    if boundary is not None:
        boundary.observe(dj.state)
    dj.handle_message(payload, raw_utterance=raw_utterance)
    if boundary is not None:
        boundary.observe(dj.state)


def _turn_tools(turn: Any) -> List[str]:
    metadata = getattr(turn, "metadata", None) or {}
    raw = metadata.get("tools_called") if hasattr(metadata, "get") else None
    if not isinstance(raw, (list, tuple)):
        return []
    return [name for name in raw if isinstance(name, str)]


def _exchanges(turns: Iterable[Any]) -> List[List[Any]]:
    """Окно → обмены: user-ход и все ответы до следующего user-хода."""
    groups: List[List[Any]] = []
    for turn in turns:
        if getattr(turn, "role", None) == "user" or not groups:
            groups.append([])
        groups[-1].append(turn)
    return groups


def _is_set_exchange(group: List[Any]) -> bool:
    return any(SET_DJ_MODE_TOOL in _turn_tools(turn) for turn in group)


def keep_current_set_turns(dj_on: bool) -> Callable[[List[Any]], List[Any]]:
    """Фильтр окна для ``AgentCore.clear_history(keep=...)``.

    Выбрасывает обмены с ``set_dj_mode``; при идущем сете последний из
    них — включивший текущий сет — остаётся. Остальные ходы (вопросы не
    про DJ, «останови музыку») не трогаются.
    """

    def keep(turns: List[Any]) -> List[Any]:
        groups = _exchanges(turns)
        set_indexes = [i for i, group in enumerate(groups) if _is_set_exchange(group)]
        current = set_indexes[-1] if (dj_on and set_indexes) else None
        drop = set(set_indexes) - {current}
        return [turn for i, group in enumerate(groups) if i not in drop for turn in group]

    return keep


def settle_dj_set_boundary(
    boundary: Optional[DJSetBoundary],
    dj: Any,
    core: Any,
    logger: Optional[logging.Logger] = None,
) -> bool:
    """На ходе после смены сета вычистить окно от прошлых сетов.

    Зовётся в asyncio-цикле перед сборкой истории хода. ``True`` — окно
    почищено. Ядро без ``clear_history(keep=...)`` (стабы тестов) — окно
    не трогаем, граница снята: штамп ``<dj_state>`` всё равно уйдёт.
    """
    if boundary is None or dj is None or not boundary.take(dj.state):
        return False
    clear = getattr(core, "clear_history", None)
    if not callable(clear):
        return False
    try:
        clear(keep=keep_current_set_turns(set_key(dj.state)[0]))
    except Exception as exc:  # noqa: BLE001 — ход не должен падать
        if logger is not None:
            logger.warning(f"⚠️ [ADR-0129] окно не почищено: {type(exc).__name__}: {exc}")
        return False
    if logger is not None:
        logger.info(
            f"🎧 [ADR-0129] смена DJ-сета {set_key(dj.state)!r}: "
            "обмены прошлых сетов убраны из окна разговора"
        )
    return True


_DJ_ON_RULE = (
    "Идёт DJ-сет: тема «{theme}», диджей «{persona}». Это единственный "
    "текущий сет, прошлые сеты завершены. Если в истории диалога ты был "
    "другим диджеем или играл другую тему — это прошлые сеты: не говори от "
    "их лица и не бери их тему."
)

_DJ_OFF_RULE = (
    "DJ-сет сейчас не идёт; все сеты в истории диалога завершены. Если юзер "
    "включает новый сет — тему и персону для set_dj_mode бери ТОЛЬКО из его "
    "текущей реплики; не названы — не передавай их и не бери из прошлых сетов."
)


def dj_state_lines(boundary: Optional[DJSetBoundary], dj: Any) -> List[str]:
    """Блок ``<dj_state>`` для ``<system_context>``; ``[]`` — сетов не было.

    Штамп собирается на каждом ходе заново — свежее состояние ближе к
    реплике, чем любые ходы прошлых сетов в окне.
    """
    if dj is None:
        return []
    state = getattr(dj, "state", None)
    enabled, persona, theme = set_key(state)
    if boundary is not None:
        boundary.observe(state)
    if not enabled and (boundary is None or not boundary.had_set):
        return []
    if not enabled:
        return [
            '  <dj_state enabled="no">',
            f"    <rule>{_DJ_OFF_RULE}</rule>",
            "  </dj_state>",
        ]
    persona = persona or str(getattr(dj, "_persona_default", "") or "")
    theme = theme or "без темы"
    return [
        '  <dj_state enabled="yes">',
        f"    <persona>{persona}</persona>",
        f"    <theme>{theme}</theme>",
        f"    <rule>{_DJ_ON_RULE.format(theme=theme, persona=persona)}</rule>",
        "  </dj_state>",
    ]


__all__ = [
    "DJSetBoundary",
    "SET_DJ_MODE_TOOL",
    "apply_dj_mode_message",
    "dj_state_lines",
    "keep_current_set_turns",
    "set_key",
    "settle_dj_set_boundary",
]
