"""Деградация провайдеров: режим команд и порядок (ADR-0150 §4.5, §8; карточка #3686).

Чистый Python, без ROS и без импорта адаптеров: пакет не знает ни одного провайдера, возможности приходят
утиным объектом (``Capabilities`` — подмножество ``rob_box_llm.provider.ProviderCapabilities``).
Режим команд — ветка кода по честно объявленным возможностям, а не правило промпта (ADR-0148).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Literal, Mapping, Optional, Protocol, Sequence, Tuple, Union

from . import phrases
from .knowledge import PROVIDER_HEALTH, PROVIDER_KINDS, PROVIDER_STALE_S, PROVIDER_USABLE_HEALTH

CommandMode = Literal["structured_output", "tool_choice", "json_mode", "json_in_text"]

# Порядок §4.5: от самого строгого к самому слабому. Единственная таблица знания о режимах.
MODE_ORDER: tuple[CommandMode, ...] = ("structured_output", "tool_choice", "json_mode", "json_in_text")

ProviderKind = Literal["llm", "stt", "tts", "music_gen"]
ProviderHealth = Literal["ok", "degraded", "quota", "network", "timeout", "auth", "down", "unknown"]

# Состояния, при которых провайдера ещё можно пробовать (в порядке предпочтения): таблица знания - knowledge.
_USABLE_HEALTH: Tuple[str, ...] = PROVIDER_USABLE_HEALTH
#: Не чаще одной фразы деградации на (провайдер, причину) за это время (ADR §8).
ANNOUNCE_MIN_INTERVAL_S = 600.0
#: Состояние -> причина в ключе ``provider.exhausted.{причина}``; ``down`` - непонятная ошибка (``error``).
_EXHAUSTED_REASON: Dict[str, str] = {
    "quota": "quota", "network": "network", "timeout": "timeout", "auth": "auth", "down": "error",
}


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


def provider_order(states: Iterable[ProviderState], priority: Sequence[str], kind: str = "llm") -> tuple[str, ...]:
    """Порядок провайдеров вида ``kind`` (по умолчанию LLM): приоритет из конфига, здоровые впереди, мёртвые (quota/auth/down) отброшены.

    Провайдер без записи в ``states`` считается здоровым (состояние ещё не опубликовано).
    """
    health = {s.name: s.health for s in states if s.kind == kind}
    ranked: list[tuple[int, int, str]] = []
    for index, name in enumerate(priority):
        state = health.get(name, "ok")
        if state in _USABLE_HEALTH:
            ranked.append((_USABLE_HEALTH.index(state), index, name))
    ranked.sort()
    return tuple(name for _, _, name in ranked)


class ProviderBoard:
    """Снимок всех писателей ``/voice/providers/state``: ``(kind, name)`` -> последнее состояние и время получения.

    Каждый писатель переиздаёт только свой вид раз в 2 с (поздний подписчик по transient_local получает не всех
    писателей, проверено на rmw_zenoh), поэтому корректность - от переиздания: запись без обновлений дольше
    ``PROVIDER_STALE_S`` отдаётся как ``unknown`` (фраза о ней не звучит, ``provider_order`` её не отбрасывает).
    """

    def __init__(self, stale_s: float = PROVIDER_STALE_S) -> None:
        self._stale_s = stale_s
        self._seen: Dict[Tuple[str, str], Tuple[ProviderState, float]] = {}

    def update(self, states: Iterable[ProviderState], now: float) -> None:
        for st in states:
            self._seen[(st.kind, st.name)] = (st, now)

    def states(self, now: float, kinds: Optional[Iterable[str]] = None) -> List[ProviderState]:
        wanted = None if kinds is None else set(kinds)
        out: List[ProviderState] = []
        for (kind, _), (st, at) in self._seen.items():
            if wanted is not None and kind not in wanted:
                continue
            fresh = now - at <= self._stale_s
            out.append(st if fresh else ProviderState(st.name, st.kind, "unknown", st.since, st.last_error_code))
        return out


def phrase_on_change(prev: Optional[ProviderState], cur: ProviderState) -> Optional[str]:
    """Ключ фразы о смене состояния или ``None``: молчим, пока ничего не изменилось и при возврате в ``ok``.

    Фраза - только на переход в состояние, из которого провайдер пропускают (quota/network/timeout/auth/down);
    ``degraded`` (лимит частоты) и выздоровление не озвучиваются: человеку нечего делать.
    """
    reason = _EXHAUSTED_REASON.get(cur.health)
    if reason is None or (prev is not None and prev.health == cur.health):
        return None
    return f"provider.exhausted.{reason}"


Priority = Union[Sequence[str], Mapping[str, Sequence[str]]]


def _priority_of(priority: Priority, kind: str) -> Sequence[str]:
    """Порядок провайдеров вида ``kind``: словарь вид -> порядок (как публикуют писатели) или один общий список."""
    if isinstance(priority, Mapping):
        return priority.get(kind, ())
    return priority


def _was_active(cur: ProviderState, prev: Optional[ProviderState], states: Sequence[ProviderState],
                priority: Sequence[str]) -> bool:
    """``cur`` был тем провайдером своего вида, которым робот отвечал человеку, ДО отказа.

    Активный - первый пригодный в порядке вида при состоянии, каким оно было до перехода. Сбой запасного при живом
    основном человеку ничего не меняет: фразы нет (ADR §8 «одна фраза на смену, которую слышит человек», #3755).
    """
    before = [prev or ProviderState(cur.name, cur.kind, "ok") if s is cur else s for s in states]
    order = provider_order(before, priority, cur.kind)
    return bool(order) and order[0] == cur.name


class ProviderAnnouncer:
    """Одна фраза на смену состояния АКТИВНОГО провайдера вида, не чаще ``ANNOUNCE_MIN_INTERVAL_S`` на (провайдер, причину).

    Состояние хранит само: вызывающий отдаёт текущий снимок ``observe(states, priority, now)`` и получает готовые
    тексты (ключ и слоты - код, текст - ``phrases``). Что провайдер вылечился, помнится: следующий отказ после
    паузы озвучится снова. ``priority`` - порядок провайдеров по видам (``{вид: [имена]}``): запасной берётся из
    порядка того же вида. Отказ неактивного (запасного) провайдера не озвучивается - его видно на доске и в логе.
    """

    def __init__(self, min_interval_s: float = ANNOUNCE_MIN_INTERVAL_S) -> None:
        self._min_interval_s = min_interval_s
        self._seen: Dict[Tuple[str, str], ProviderState] = {}
        self._spoken_at: Dict[Tuple[str, str], float] = {}

    def observe(self, states: Sequence[ProviderState], priority: Priority, now: float) -> List[str]:
        """Тексты фраз о переходах с прошлого вызова (обычно пусто)."""
        out: List[str] = []
        for cur in states:
            key = (cur.kind, cur.name)
            prev = self._seen.get(key)
            self._seen[key] = cur
            phrase_key = phrase_on_change(prev, cur)
            if phrase_key is None or not _was_active(cur, prev, states, _priority_of(priority, cur.kind)):
                continue
            # Тариф один на LLM и TTS (MiniMax): одна причина у одного имени - одна фраза, а не по фразе на вид.
            spoken_key = (cur.name, phrase_key)
            last = self._spoken_at.get(spoken_key)
            if last is not None and now - last < self._min_interval_s:
                continue
            self._spoken_at[spoken_key] = now
            out.append(_compose(phrase_key, cur, states, _priority_of(priority, cur.kind)))
        return out


def _compose(phrase_key: str, cur: ProviderState, states: Sequence[ProviderState], priority: Sequence[str]) -> str:
    """Фраза об отказе + «что делаю дальше»: вид назван, запасной того же вида или честное «запасного нет»."""
    alt = next(iter(provider_order(states, [p for p in priority if p != cur.name], cur.kind)), None)
    if alt:
        switched = phrases.first_key(f"provider.switched.{cur.kind}", "provider.switched")
        nxt = phrases.text(switched, provider=alt)
    else:
        nxt = phrases.text("provider.no_fallback")
    subject = phrases.text(f"provider.subject.{cur.kind}", provider=cur.name)
    return phrases.text(phrase_key, subject=subject, next=nxt)


assert set(_EXHAUSTED_REASON) <= set(PROVIDER_HEALTH) and "music_gen" in PROVIDER_KINDS
