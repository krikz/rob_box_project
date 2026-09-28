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
      "dj": {"enabled": bool},         # DJ-режим (MusicManager); объект —
                                       # ADR-0142 добавит persona/theme/plan
      "finished_track_id": str | null, # idle потому, что этот трек доиграл сам
      "ts": float                      # epoch публикации
    }

QoS: ``RELIABLE`` + ``TRANSIENT_LOCAL`` + ``KEEP_LAST 1`` — последний снимок
получает и тот, кто подписался позже (перезапуск ноды). Подписчику нужен
тот же ``TRANSIENT_LOCAL``: с ``VOLATILE``-подпиской истории не будет.

``track_id`` — непрозрачная строка: сравнивать только на равенство, формат
не разбирать (ADR-0142 сменит его на токен ``<set_id>:<track_no>:…``).
Событие «трек доиграл сам» в этом контракте — ``finished_track_id`` в
снимке ``idle``; отдельный поток событий ``/voice/music/event``
(``started`` / ``nearly_finished`` / ``finished``) вводит ADR-0142.

Модуль без ROS-импортов: его используют и публикатор (``rob_box_mcp_tools``),
и подписчики (``rob_box_voice``), а юнит-тесты гоняют его без rclpy.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, Optional

#: Имя топика — одно на публикатора и всех подписчиков.
MUSIC_STATE_TOPIC = "/voice/music/state"

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
    dj: bool = False,
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
            "dj": {"enabled": bool(dj)},
            # «доиграл сам» имеет смысл только в idle.
            "finished_track_id": None if playing else _str_or_none(finished_track_id),
            "ts": time.time() if ts is None else float(ts),
        },
        ensure_ascii=False,
    )


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
    )
