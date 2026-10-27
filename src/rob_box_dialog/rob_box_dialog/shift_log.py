"""Журнал смены: кто приходил, что звучало, что сделал робот — структурой со временем (#4076, ADR-0148).

Решение товарища Шифу 27.10: «можно просто вести лог событий и туда смотреть». Марафон 27.10, n1001 «подведи итог
ночи: кто к нам приходил, что мы слушали»: источника фактов смены не было (память о собеседнике и урезанное окно
диалога), и модель утверждала отрицательное — «никто, кроме тебя, не подходил, музыку мы не слушали». Теперь факты
смены ведёт код, записи пишут владельцы событий:

* приход (:data:`VISIT`) — голос из снимка сессии (``SessionChanged`` ``set_speaker``; имя — только уверенное,
  ``speaker_adapter.speaker_block``: гипотеза и спор голоса — «незнакомый», A30) и встреча лицом (``meeting``). Тот же
  человек в пределах ``knowledge.SHIFT_VISIT_GAP_S`` — продолжение прихода (``until`` — когда видели последний раз);
* музыка (:data:`TRACK`) — снимок владельца плеера (``/voice/music/state``): название, источник, как кончилось — по
  правилу ``music_state.MusicStateMemory`` (``last_ended``: доиграл / остановлен / сменён), второго правила нет;
* действия и выступления (:data:`ACTION`, :data:`PERFORM`) — исходы владельцев из журнала ходов
  (``TurnLog(on_outcome=...)``): тул и статус; поздний итог и конец выступления обновляют запись на месте. Чтение
  (класс ``query``), обещание ``pending`` и тулы ``knowledge.SHIFT_SKIP_TOOLS`` не пишутся;
* тишина (:data:`SILENCE`) — кадр ``Silence`` сессии.

Журнал — в памяти процесса: кольцо ``SHIFT_LOG_MAX_EVENTS`` записей за ``SHIFT_LOG_WINDOW_S``. Новая сессия разговора
(«забудь всё», ``Session.reset``) его не чистит — это журнал смены, а не разговора. Рестарт процесса журнал обнуляет,
и раздел говорит об этом сам: ``covers_min`` — сколько минут журнал покрывает, фраза-итог — «за последние N мин».

Раздел снимка ``shift_log`` (:meth:`ShiftLog.section`) идёт модели, когда человек спрашивает о смене
(:func:`shift_asked`): записи и фраза-итог ``summary``, которую строит код из записей (:func:`summary`, тексты —
``phrases/ru.yaml``). Пустой журнал — фраза кода «в журнале смены ничего не записано», а не догадка модели.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from typing import Any, Callable, Deque, Dict, List, Mapping, Optional, Sequence

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.music_state import UNNAMED_TRACK
from rob_box_dialog.speaker_adapter import speaker_block
from rob_box_dialog.turn_log import PERFORM_TOOL
from rob_box_dialog.words import has_form, has_word

#: Раздел снимка хода (``knowledge.CONTEXT_SECTIONS``).
SECTION = "shift_log"
#: Виды записей журнала.
VISIT, TRACK, ACTION, PERFORM, SILENCE = "visit", "track", "action", "perform", "silence"
#: Каналы прихода.
VOICE, FACE = "voice", "face"
#: ``ShiftEvent.status`` записи без конца: трек ещё звучит, тишина ещё идёт.
ONGOING = "ongoing"
#: ``ShiftEvent.status`` конца тишины.
ENDED = "ended"
_QUERY = "query"


@dataclass(frozen=True)
class ShiftEvent:
    """Запись журнала. ``at`` — начало (приход, старт трека, исход действия), ``until`` — последнее обновление (видели
    последний раз, трек кончился); ``key`` — по нему владелец обновляет запись на месте."""

    kind: str
    at: float
    until: float
    key: str
    who: str = ""  # VISIT: имя (только уверенное); "" — незнакомый
    what: str = ""  # TRACK: название; ACTION: тул; PERFORM: вид выступления
    status: str = ""  # ACTION/PERFORM: исход; TRACK/SILENCE: ONGOING или как кончилось
    detail: str = ""  # VISIT: каналы через запятую; TRACK: источник; ACTION: причина отказа


def shift_asked(text: str) -> bool:
    """Спрашивает ли человек о смене («подведи итог ночи», «кто приходил», «что мы слушали»): основы
    ``knowledge.SHIFT_ASK_STEMS`` или целые слова ``SHIFT_ASK_FORMS``. Таблица одна — ``knowledge``."""
    return has_word(text or "", kn.SHIFT_ASK_STEMS) or has_form(text or "", kn.SHIFT_ASK_FORMS)


class ShiftLog:
    """Журнал смены одного процесса: пишут владельцы событий, читает раздел снимка хода."""

    def __init__(self, clock: Callable[[], float] = time.time, window_s: float = kn.SHIFT_LOG_WINDOW_S,
                 max_events: int = kn.SHIFT_LOG_MAX_EVENTS) -> None:
        self._clock = clock
        self._window_s = float(window_s)
        self._events: Deque[ShiftEvent] = deque(maxlen=int(max_events))
        self._lock = threading.RLock()  # пишут колбэки ROS (свои потоки) и ход (поток asyncio)
        self.started_at = float(clock())
        self._track: Optional[str] = None  # трек, который звучит по последнему снимку плеера
        self._ended: Any = None  # последний учтённый ``MusicStateMemory.last_ended``
        self._silenced = False  # кадр ``Silence`` в последнем снимке сессии
        self._silences = 0  # сколько раз включали тишину (ключ записи)

    # -- запись: владельцы событий ------------------------------------------------------------------------------

    def observe(self, change: Any) -> None:
        """Подписчик сессии (``Session.subscribe``): голос собеседника — приход, кадр ``Silence`` — тишина."""
        snap = change.snapshot
        with self._lock:
            if change.change == "set_speaker" and snap.speaker is not None:
                self.seen(speaker_block(snap.speaker)["name"], VOICE)
            self._silence(snap.top("Silence") is not None)

    def seen(self, name: Optional[str], via: str = VOICE) -> None:
        """Человек замечен каналом ``via`` (голос, лицо): имя — только уверенное, ``None`` — незнакомый. Тот же
        человек после паузы меньше ``SHIFT_VISIT_GAP_S`` — продолжение прихода, иначе — новый приход."""
        who = (name or "").strip()
        key = who.lower() or f"?{via}"
        with self._lock:
            now = self._clock()
            slot = self._find(VISIT, key)
            old = self._events[slot] if slot is not None else None
            if old is not None and now - old.until <= kn.SHIFT_VISIT_GAP_S:
                self._events[slot] = replace(old, until=now, detail=_with_channel(old.detail, via))
            else:
                self._events.append(ShiftEvent(VISIT, now, now, key, who=who, detail=via))

    def music(self, snapshot: Any, ended: Any = None) -> None:
        """Снимок владельца плеера (``MusicPlayerState``) после ``MusicStateMemory.observe_state``: ``ended`` — её
        ``last_ended`` (как и когда замолчал прошлый трек). Новый звучащий ``track_id`` — запись трека."""
        with self._lock:
            at = float(getattr(snapshot, "ts", None) or self._clock())
            if ended is not None and ended is not self._ended:
                self._ended = ended
                self._close(TRACK, str(ended.track_id or ""), float(ended.at), str(ended.how))
            track = snapshot.track_id if snapshot.is_playing(at) else None
            if track and track != self._track:
                info = getattr(snapshot, "dj_info", None) or {}
                self._events.append(ShiftEvent(TRACK, at, at, track, what=_title(info), status=ONGOING,
                                               detail=_source(info)))
            self._track = track

    def outcome(self, ref: str, cls: str, content: Mapping[str, Any]) -> None:
        """Исход действия или выступления из журнала ходов (``TurnLog`` ``on_outcome``): ``ref`` — id вызова
        (``<ход>/<n>``) или выступления хода, ``content`` — запись окна (тул, статус, причина, данные). Повтор ``ref``
        (поздний итог, конец выступления) обновляет запись на месте."""
        tool, status = str(content.get("tool") or ""), str(content.get("status") or "")
        if cls == _QUERY or tool in kn.SHIFT_SKIP_TOOLS or status in kn.SHIFT_SKIP_STATUSES:
            return
        kind = PERFORM if tool == PERFORM_TOOL else ACTION
        what = str((content.get("data") or {}).get("kind") or "") if kind == PERFORM else tool
        with self._lock:
            now = self._clock()
            fresh = ShiftEvent(kind, now, now, ref, what=what, status=status, detail=str(content.get("reason") or ""))
            slot = self._find(kind, ref)
            if slot is None:
                self._events.append(fresh)
            else:
                old = self._events[slot]
                self._events[slot] = replace(fresh, at=old.at, what=what or old.what)

    def _silence(self, on: bool) -> None:
        """Кадр ``Silence`` появился — запись тишины; снят (сам, словом, по TTL) — её конец."""
        if on == self._silenced:
            return
        self._silenced = on
        now = self._clock()
        if on:
            self._silences += 1
            self._events.append(ShiftEvent(SILENCE, now, now, _silence_key(self._silences), status=ONGOING))
        else:
            self._close(SILENCE, _silence_key(self._silences), now, ENDED)

    def _find(self, kind: str, key: str) -> Optional[int]:
        """Индекс последней записи вида ``kind`` с ключом ``key``; выпала из кольца или не было — ``None``."""
        for i in range(len(self._events) - 1, -1, -1):
            if self._events[i].kind == kind and self._events[i].key == key:
                return i
        return None

    def _close(self, kind: str, key: str, until: float, status: str) -> None:
        slot = self._find(kind, key)
        if slot is not None:
            self._events[slot] = replace(self._events[slot], until=until, status=status)

    # -- чтение ---------------------------------------------------------------------------------------------------

    def events(self, now: Optional[float] = None) -> List[ShiftEvent]:
        """Записи окна ``SHIFT_LOG_WINDOW_S`` (по последнему обновлению) в порядке появления."""
        now = self._clock() if now is None else float(now)
        with self._lock:
            return [e for e in self._events if now - e.until <= self._window_s]

    def view(self, now: Optional[float] = None) -> Dict[str, Any]:
        """Раздел ``shift_log``: фраза-итог кода (:func:`summary`), сколько минут журнал покрывает, записи по видам
        (новые, не больше ``SHIFT_SECTION_MAX_ITEMS`` каждого); вид без записей пропущен."""
        now = self._clock() if now is None else float(now)
        events = self.events(now)
        covers = min(max(0.0, now - self.started_at), self._window_s)
        data: Dict[str, Any] = {"summary": summary(events, covers), "covers_min": _minutes(covers)}
        for name in kn.SHIFT_PART_PHRASES:
            value = _VIEW[name](events, now)
            if value:
                data[name] = value
        return data

    def section(self, text: str, now: Optional[float] = None) -> Dict[str, Any]:
        """Разделы снимка хода: ``{shift_log: ...}``, когда человек спрашивает о смене (:func:`shift_asked`), иначе
        пусто — журнал не занимает бюджет снимка в каждом ходе."""
        return {SECTION: self.view(now)} if shift_asked(text) else {}


def summary(events: Sequence[ShiftEvent], covers_s: float) -> str:
    """Фраза-итог из записей журнала (не из памяти диалога): части ``knowledge.SHIFT_PART_PHRASES`` по видам, где есть
    записи. Записей нет — честное «в журнале смены за последние N мин ничего не записано»."""
    span = span_words(covers_s)
    parts = []
    for name, key in kn.SHIFT_PART_PHRASES.items():
        slots = _SLOTS[name](events)
        if slots:
            parts.append(phrases.text(key, **slots))
    if not parts:
        return phrases.text(kn.SHIFT_EMPTY_PHRASE, span=span)
    return phrases.text(kn.SHIFT_SUMMARY_PHRASE, span=span, events="; ".join(parts))


def span_words(seconds: float) -> str:
    """Длительность словами для фразы: «25 мин», «3 ч», «3 ч 20 мин» (не меньше минуты)."""
    hours, minutes = divmod(max(1, _minutes(seconds)), 60)
    if not hours:
        return f"{minutes} мин"
    return f"{hours} ч {minutes} мин" if minutes else f"{hours} ч"


# -- части раздела: записи окна → данные ---------------------------------------------------------------------------

def _people(events: Sequence[ShiftEvent], now: float) -> List[Dict[str, Any]]:
    """Узнанные люди: имя, сколько приходов, когда впервые и в последний раз (минут назад), каналы."""
    rows: Dict[str, Dict[str, Any]] = {}
    for e in events:
        if e.kind != VISIT or not e.who:
            continue
        row = rows.setdefault(e.key, {"name": e.who, "visits": 0, "first": e.at, "last": e.until, "via": set()})
        row["visits"] += 1
        row["first"], row["last"] = min(row["first"], e.at), max(row["last"], e.until)
        row["via"].update(e.detail.split(","))
    recent = sorted(rows.values(), key=lambda r: r["last"], reverse=True)[:kn.SHIFT_SECTION_MAX_ITEMS]
    return [{"name": r["name"], "visits": r["visits"], "first_min_ago": _minutes(now - r["first"]),
             "last_min_ago": _minutes(now - r["last"]), "via": ",".join(sorted(r["via"]))} for r in recent]


def _unknown(events: Sequence[ShiftEvent], _now: float) -> Dict[str, int]:
    """Приходы незнакомых по каналу: ``{"voice": 3, "face": 1}``."""
    counts: Dict[str, int] = {}
    for e in events:
        if e.kind == VISIT and not e.who:
            counts[e.detail] = counts.get(e.detail, 0) + 1
    return counts


def _music(events: Sequence[ShiftEvent], now: float) -> List[Dict[str, Any]]:
    """Треки: название, источник, минут назад начался, сколько звучал и как кончился (``ongoing`` — звучит)."""
    rows = []
    for e in [e for e in events if e.kind == TRACK][-kn.SHIFT_SECTION_MAX_ITEMS:]:
        row: Dict[str, Any] = {"track": e.what, "min_ago": _minutes(now - e.at), "ended": e.status}
        if e.detail:
            row["source"] = e.detail
        if e.status != ONGOING:
            row["played_min"] = _minutes(e.until - e.at)
        rows.append(row)
    return rows


def _performances(events: Sequence[ShiftEvent], now: float) -> List[Dict[str, Any]]:
    """Выступления: вид, исход (``started`` — ещё идёт), минут назад."""
    performs = [e for e in events if e.kind == PERFORM][-kn.SHIFT_SECTION_MAX_ITEMS:]
    return [{"kind": e.what, "status": e.status, "min_ago": _minutes(now - e.at)} for e in performs]


def _actions(events: Sequence[ShiftEvent], _now: float) -> Dict[str, Dict[str, int]]:
    """Исходы действий по тулам: ``{"request_music": {"done": 3, "rejected": 1}}``."""
    counts: Dict[str, Dict[str, int]] = {}
    for e in events:
        if e.kind == ACTION:
            by_status = counts.setdefault(e.what, {})
            by_status[e.status] = by_status.get(e.status, 0) + 1
    return counts


def _silence_rows(events: Sequence[ShiftEvent], now: float) -> List[Dict[str, Any]]:
    """Тишина: минут назад началась и сколько длилась (идущая — до сейчас, ``ongoing``)."""
    rows = []
    for e in [e for e in events if e.kind == SILENCE][-kn.SHIFT_SECTION_MAX_ITEMS:]:
        end = now if e.status == ONGOING else e.until
        rows.append({"min_ago": _minutes(now - e.at), "for_min": _minutes(end - e.at), "ended": e.status})
    return rows


_VIEW: Mapping[str, Callable[[Sequence[ShiftEvent], float], Any]] = {
    "people": _people, "unknown": _unknown, "music": _music, "performances": _performances, "actions": _actions,
    "silence": _silence_rows,
}


# -- части фразы-итога: записи окна → слоты фразы ------------------------------------------------------------------

def _people_slots(events: Sequence[ShiftEvent]) -> Optional[Dict[str, Any]]:
    names = list(dict.fromkeys(e.who for e in events if e.kind == VISIT and e.who))
    return {"names": ", ".join(names)} if names else None


def _unknown_slots(events: Sequence[ShiftEvent]) -> Optional[Dict[str, Any]]:
    count = sum(1 for e in events if e.kind == VISIT and not e.who)
    return {"count": count} if count else None


def _music_slots(events: Sequence[ShiftEvent]) -> Optional[Dict[str, Any]]:
    tracks = [e for e in events if e.kind == TRACK]
    return {"count": len(tracks), "last": tracks[-1].what} if tracks else None


def _performance_slots(events: Sequence[ShiftEvent]) -> Optional[Dict[str, Any]]:
    kinds = list(dict.fromkeys(e.what for e in events if e.kind == PERFORM and e.what))
    return {"kinds": ", ".join(kinds)} if kinds else None


def _action_slots(events: Sequence[ShiftEvent]) -> Optional[Dict[str, Any]]:
    acts = [e for e in events if e.kind == ACTION]
    if not acts:
        return None
    failed = sum(1 for e in acts if e.status in kn.SHIFT_FAILED_STATUSES)
    return {"done": sum(1 for e in acts if e.status == "done"), "failed": failed}


def _silence_slots(events: Sequence[ShiftEvent]) -> Optional[Dict[str, Any]]:
    count = sum(1 for e in events if e.kind == SILENCE)
    return {"count": count} if count else None


_SLOTS: Mapping[str, Callable[[Sequence[ShiftEvent]], Optional[Dict[str, Any]]]] = {
    "people": _people_slots, "unknown": _unknown_slots, "music": _music_slots, "performances": _performance_slots,
    "actions": _action_slots, "silence": _silence_slots,
}


def _with_channel(channels: str, via: str) -> str:
    have = [c for c in channels.split(",") if c]
    return ",".join(have if via in have else have + [via])


def _title(info: Mapping[str, Any]) -> str:
    """Название трека из снимка плеера: имя трека, иначе заголовок деки, иначе «без названия»."""
    return str(info.get("name") or info.get("title") or "") or UNNAMED_TRACK


def _source(info: Mapping[str, Any]) -> str:
    """Источник трека из снимка плеера (синтез, генерация, библиотека, кэш) или движок."""
    return str(info.get("source") or info.get("engine") or "")


def _silence_key(n: int) -> str:
    return f"silence/{n}"


def _minutes(seconds: float) -> int:
    return max(0, int(round(float(seconds) / 60.0)))


__all__ = ["ACTION", "ENDED", "FACE", "ONGOING", "PERFORM", "SECTION", "SILENCE", "ShiftEvent", "ShiftLog", "TRACK",
           "VISIT", "VOICE", "shift_asked", "span_words", "summary"]
