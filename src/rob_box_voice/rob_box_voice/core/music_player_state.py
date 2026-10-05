"""Контракт топика ``/voice/music/state`` — состояние плеера музыки (ADR-0141).

Issue #3133. Владелец состояния «играет / какой трек / DJ» — плеер
(``mcp_server`` + ``MusicManager``). Он публикует снимок сам: после каждого
музыкального тула, на каждом тике watchdog'а и в момент, когда конечный
трек (``repeat=False``) доиграл форму. Все читатели (``audio_node``,
``dialogue_node``) берут «играет» ТОЛЬКО отсюда, а не выводят его из имён
вызванных тулов.

Формат — JSON-строка в ``std_msgs/String``::

    {
      "state": "playing" | "idle",
      "track_id": str | null,          # id трека, который звучит сейчас
      "form_ends_at": float | null,    # epoch конца прохода формы (любой repeat)
      "stops_at": float | null,        # epoch остановки конечного трека
      "dj": {"enabled": bool, ...},    # DJ-режим; владелец
                                       # плеера кладёт сюда и сет (ADR-0149 §2.3)
      "finished_track_id": str | null, # idle потому, что этот трек доиграл сам
      "ts": float                      # epoch публикации
    }

QoS: ``RELIABLE`` + ``TRANSIENT_LOCAL`` + ``KEEP_LAST 1`` — последний снимок
получает и тот, кто подписался позже (перезапуск ноды). Подписчику нужен
тот же ``TRANSIENT_LOCAL``: с ``VOLATILE``-подпиской истории не будет.

``track_id`` — непрозрачная строка: сравнивать только на равенство, формат
не разбирать (ADR-0142 сменит его на токен ``<set_id>:<track_no>:…``).
Событие «трек доиграл сам» в этом контракте — ``finished_track_id`` в
снимке ``idle``. Поток событий ``/voice/music/event`` (ADR-0142 §3.1,
ADR-0149 §2.3) пишет только владелец плеера v2: JSON
``{"event", "track_id", "ts", ...поля события}``, см.
:func:`build_music_event_payload`.

Модуль без ROS-импортов: его используют и публикатор (``rob_box_mcp_tools``),
и подписчики (``rob_box_voice``), а юнит-тесты гоняют его без rclpy.
"""

from __future__ import annotations

import json
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

#: Имя топика — одно на публикатора и всех подписчиков.
MUSIC_STATE_TOPIC = "/voice/music/state"

#: События плеера v2 (ADR-0149 §2.3): RELIABLE, KEEP_LAST 10, не latched.
MUSIC_EVENT_TOPIC = "/voice/music/event"
MUSIC_EVENTS = frozenset({"queued", "started", "nearly_finished", "finished", "rejected", "idle"})

STATE_PLAYING = "playing"
STATE_IDLE = "idle"

#: Сколько секунд после ``stops_at`` подписчик ещё верит «playing».
#: Плеер сам переводит сессию в idle через ~0.3 с после ``stops_at``;
#: запас нужен на доставку. Если idle так и не пришёл (плеер умер/завис),
#: подписчик перестаёт считать музыку играющей сам — трек физически
#: остановлен Renardo (``Clock.future(end, Clock.clear)``).
STOPS_AT_GRACE_S = 1.0


def _epoch_or_none(value: Any) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _str_or_none(value: Any) -> Optional[str]:
    return value if isinstance(value, str) and value else None


def _dj_payload(value: Any) -> dict:
    """``dj`` снимка: bool (v1) → ``{"enabled": bool}``; объект (v2) — как есть + ``enabled``."""
    if isinstance(value, dict):
        return {**value, "enabled": value.get("enabled") is True}
    return {"enabled": bool(value)}


def _dj_enabled(value: Any) -> bool:
    """``dj`` — объект ``{"enabled": bool, ...}``; голый bool тоже принимаем."""
    if isinstance(value, dict):
        return value.get("enabled") is True
    return value is True


@dataclass(frozen=True)
class MusicPlayerState:
    """Разобранный снимок ``/voice/music/state``."""

    state: str
    track_id: Optional[str] = None
    form_ends_at: Optional[float] = None
    stops_at: Optional[float] = None
    dj: bool = False  # dj.enabled
    finished_track_id: Optional[str] = None
    ts: Optional[float] = None
    #: Поля ``dj`` целиком (v2: ``set_id``, ``track_no``, ``theme``, ``bpm``, ``title``…) — для ``<music_state>``.
    dj_info: Mapping[str, Any] = field(default_factory=dict, compare=False, hash=False)

    def is_playing(self, now: Optional[float] = None) -> bool:
        """Звучит ли музыка по мнению плеера (с защитой по ``stops_at``).

        ``stops_at`` в прошлом дальше :data:`STOPS_AT_GRACE_S` — конечный
        трек уже замолчал, даже если свежий ``idle`` не дошёл.
        """
        if self.state != STATE_PLAYING:
            return False
        if self.stops_at is None:
            return True
        current = time.time() if now is None else float(now)
        return current <= self.stops_at + STOPS_AT_GRACE_S


def build_music_state_payload(
    *,
    playing: bool,
    track_id: Optional[str] = None,
    form_ends_at: Optional[float] = None,
    stops_at: Optional[float] = None,
    dj: Any = False,
    finished_track_id: Optional[str] = None,
    ts: Optional[float] = None,
) -> str:
    """Собрать JSON для ``/voice/music/state`` (сторона плеера)."""
    return json.dumps(
        {
            "state": STATE_PLAYING if playing else STATE_IDLE,
            "track_id": _str_or_none(track_id),
            "form_ends_at": _epoch_or_none(form_ends_at),
            "stops_at": _epoch_or_none(stops_at),
            "dj": _dj_payload(dj),
            # «доиграл сам» имеет смысл только в idle.
            "finished_track_id": None if playing else _str_or_none(finished_track_id),
            "ts": time.time() if ts is None else float(ts),
        },
        ensure_ascii=False,
    )


def build_music_event_payload(event: str, track_id: Optional[str], ts: Optional[float] = None,
                              **fields: Any) -> str:
    """Собрать JSON для ``/voice/music/event``; неизвестное имя события — ``ValueError``."""
    if event not in MUSIC_EVENTS:
        raise ValueError(f"неизвестное событие плеера: {event!r}")
    payload = {"event": event, "track_id": _str_or_none(track_id),
               "ts": time.time() if ts is None else float(ts)}
    payload.update(fields)
    return json.dumps(payload, ensure_ascii=False)


def parse_music_state(data: Optional[str]) -> Optional[MusicPlayerState]:
    """Разобрать ``msg.data`` топика; ``None`` — мусор, не трогать состояние.

    Понимает и старый плоский формат ``"playing"`` / ``"idle"`` (до
    issue #3133): сервер и подписчики могут обновиться не одновременно.
    """
    text = (data or "").strip()
    if not text:
        return None
    if text in (STATE_PLAYING, STATE_IDLE):
        return MusicPlayerState(state=text)
    try:
        payload = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(payload, dict):
        return None
    state = payload.get("state")
    if state not in (STATE_PLAYING, STATE_IDLE):
        return None
    return MusicPlayerState(
        state=state,
        track_id=_str_or_none(payload.get("track_id")),
        form_ends_at=_epoch_or_none(payload.get("form_ends_at")),
        stops_at=_epoch_or_none(payload.get("stops_at")),
        dj=_dj_enabled(payload.get("dj")),
        finished_track_id=_str_or_none(payload.get("finished_track_id")),
        ts=_epoch_or_none(payload.get("ts")),
        dj_info=_dj_payload(payload.get("dj")),
    )


@dataclass(frozen=True)
class MusicEvent:
    """Разобранное событие ``/voice/music/event`` (ADR-0149 §2.3)."""

    event: str
    track_id: Optional[str]
    ts: Optional[float] = None
    fields: Mapping[str, Any] = field(default_factory=dict, compare=False, hash=False)


#: События, которые решают судьбу запуска трека: звук встал или отказ (I5, I25).
DECISIVE_EVENTS = frozenset({"started", "rejected"})


def parse_music_event(data: Optional[str]) -> Optional[MusicEvent]:
    """Разобрать ``msg.data`` события; ``None`` — мусор или неизвестное событие."""
    try:
        payload = json.loads(data or "")
    except (TypeError, ValueError):
        return None
    if not isinstance(payload, dict) or payload.get("event") not in MUSIC_EVENTS:
        return None
    rest = {k: v for k, v in payload.items() if k not in ("event", "track_id", "ts")}
    return MusicEvent(payload["event"], _str_or_none(payload.get("track_id")),
                      _epoch_or_none(payload.get("ts")), rest)


class MusicEventLog:
    """Последнее решающее событие (``started``/``rejected``) по ``track_id``; ждать его из другого потока.

    Фраза об успехе запуска строится только по ``started`` (ADR-0149 §5.1, A14). Событие может
    прийти раньше, чем ответ тула с этим ``track_id`` — поэтому лог помнит последние ``keep`` треков.
    """

    def __init__(self, keep: int = 32) -> None:
        self._keep = keep
        self._cond = threading.Condition()
        self._by_track: "OrderedDict[str, MusicEvent]" = OrderedDict()

    def observe(self, event: Optional[MusicEvent]) -> None:
        if event is None or event.event not in DECISIVE_EVENTS or not event.track_id:
            return
        with self._cond:
            self._by_track.pop(event.track_id, None)
            self._by_track[event.track_id] = event
            while len(self._by_track) > self._keep:
                self._by_track.popitem(last=False)
            self._cond.notify_all()

    def observe_json(self, data: Optional[str]) -> None:
        self.observe(parse_music_event(data))

    def observe_msg(self, msg: Any) -> None:
        """Колбэк подписки ``std_msgs/String``."""
        self.observe_json(getattr(msg, "data", None))

    def wait(self, track_id: Optional[str], timeout: float) -> Optional[MusicEvent]:
        """``started``/``rejected`` трека или ``None`` — за ``timeout`` секунд ничего не пришло."""
        if not track_id:
            return None
        with self._cond:
            self._cond.wait_for(lambda: track_id in self._by_track, timeout=max(0.0, timeout))
            return self._by_track.get(track_id)
