"""``<music_state>`` для LLM — из снимка плеера (issue #3161, ADR-0141).

До #3161 тег собирался эвристикой ``beat``: «играет», если взведён
cleanup-флаг или живы TTS-батчи. Живой прогон 28.09 22:32: через 10 с
после «выключи музыку» модель прочла в теге устаревшее «играет» и сказала
«Сейчас играет клубный трек». Теперь источник один — то, что публикует
плеер:

* ``/voice/music/state`` (:class:`MusicPlayerState`) — играет/нет,
  ``track_id``, ``stops_at`` (конечный трек), DJ, ``finished_track_id``;
* ``/voice/music/form`` — имя играющей темы (``track``): в снимке только
  непрозрачный ``track_id``, имя плеер кладёт в соседний топик в том же
  ``publish_music_state()``.

Чтобы на «что ты сейчас включал?» модель могла честно ответить «минуту
назад играл X», :class:`MusicStateMemory` помнит последний трек, который
перестал звучать: как (доиграл / остановлен / сменён другим) и когда.
Это не эвристика — переход «играл → не играет» виден в самих снимках.

Модуль без ROS-импортов: юнит-тесты гоняют его без rclpy.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from rob_box_voice.core.music_player_state import MusicPlayerState

#: Сколько секунд после конца трека тег ещё показывает «недавно играл».
#: Дольше получаса «что ты включал?» — уже история разговора, не состояние.
RECENT_END_WINDOW_S = 1800.0

#: Как трек перестал звучать.
ENDED_FINISHED = "finished"  # доиграл форму сам (``finished_track_id``)
ENDED_STOPPED = "stopped"    # остановлен (стоп, watchdog, idle-TTL)
ENDED_REPLACED = "replaced"  # вместо него запущен другой трек

UNNAMED_TRACK = "без названия"


def _xml_attr(value: str) -> str:
    """Экранировать значение для XML-атрибута в двойных кавычках."""
    return (
        value.replace("&", "&amp;")
        .replace('"', "&quot;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


@dataclass(frozen=True)
class EndedTrack:
    """Трек, который перестал звучать."""

    track_id: Optional[str]
    name: Optional[str]
    how: str
    at: float


class MusicStateMemory:
    """Последний снимок плеера + память о последнем замолчавшем треке.

    Кормится из двух колбэков ноды: :meth:`observe_state` — на каждый
    ``/voice/music/state``, :meth:`observe_track_name` — на каждый
    ``/voice/music/form``. Плеер публикует их парой (сначала state, потом
    form), поэтому имя нового трека приходит ПОСЛЕ снимка с его
    ``track_id`` — смена трека обнуляет имя, form его заполняет.
    """

    def __init__(self) -> None:
        self._snapshot: Optional[MusicPlayerState] = None
        self._playing = False
        self._track_id: Optional[str] = None
        self._name: Optional[str] = None
        self._last_ended: Optional[EndedTrack] = None

    @property
    def snapshot(self) -> Optional[MusicPlayerState]:
        return self._snapshot

    @property
    def last_ended(self) -> Optional[EndedTrack]:
        return self._last_ended

    @property
    def track_name(self) -> Optional[str]:
        """Имя играющего сейчас трека (``None`` — тишина или имени нет)."""
        return self._name if self._playing else None

    def observe_state(
        self, snapshot: MusicPlayerState, now: Optional[float] = None
    ) -> None:
        """Принять снимок ``/voice/music/state``."""
        at = snapshot.ts if snapshot.ts is not None else _now(now)
        playing = snapshot.is_playing(at)
        track_changed = playing and snapshot.track_id != self._track_id
        if self._playing and (not playing or track_changed):
            self._last_ended = EndedTrack(
                track_id=self._track_id,
                name=self._name,
                how=self._ended_how(snapshot, playing),
                at=at,
            )
        if not playing or track_changed:
            self._name = None
        self._snapshot = snapshot
        self._playing = playing
        self._track_id = snapshot.track_id if playing else None

    def observe_track_name(self, name: Optional[str]) -> None:
        """Принять имя темы из ``/voice/music/form`` (только пока играет)."""
        if self._playing and isinstance(name, str) and name:
            self._name = name

    def _ended_how(self, snapshot: MusicPlayerState, playing: bool) -> str:
        if playing:
            return ENDED_REPLACED
        finished = snapshot.finished_track_id
        if finished is not None and finished == self._track_id:
            return ENDED_FINISHED
        return ENDED_STOPPED

    def render(
        self,
        *,
        generated_title: Optional[str] = None,
        now: Optional[float] = None,
    ) -> str:
        """Строка ``<music_state …/>`` для ``<system_context>``.

        ``generated_title`` — mp3 из AI-библиотеки (``gen_play_from_library``)
        играет в ``sound_node`` мимо ``MusicManager`` и в снимок плеера не
        попадает; его состояние публикует сам плеер mp3
        (``/voice/generated_music/state``), нода передаёт название, если
        там ``playing``.
        """
        current = _now(now)
        attrs = self._playback_attrs(current)
        ai = f"playing: {generated_title}" if generated_title else "idle"
        attrs.append(("ai", ai))
        attrs.extend(self._last_ended_attrs(current))
        body = " ".join(f'{key}="{_xml_attr(value)}"' for key, value in attrs)
        return f"  <music_state {body} />"

    def _playback_attrs(self, now: float) -> list:
        snap = self._snapshot
        if snap is None:
            # Плеер ещё ничего не сообщил — честно «не знаю».
            return [("playing", "unknown"), ("dj", "unknown")]
        dj = "on" if snap.dj else "off"
        if not snap.is_playing(now):
            return [("playing", "no"), ("dj", dj)]
        attrs = [("playing", "yes"), ("track", self._name or UNNAMED_TRACK)]
        if snap.stops_at is None:
            attrs.append(("repeat", "loop"))
        else:
            attrs.append(("repeat", "once"))
            left = max(0, round(snap.stops_at - now))
            attrs.append(("ends_in_s", str(left)))
        attrs.append(("dj", dj))
        return attrs

    def _effective_last_ended(self, now: float) -> Optional[EndedTrack]:
        """Последний замолчавший трек, включая «доиграл, а idle не дошёл».

        Конечный трек, у которого ``stops_at`` уже прошёл (с запасом
        :data:`STOPS_AT_GRACE_S`), по :meth:`MusicPlayerState.is_playing`
        не играет, даже если свежий ``idle`` потерялся, — значит, он и
        есть последний замолчавший.
        """
        snap = self._snapshot
        if (
            self._playing
            and snap is not None
            and snap.stops_at is not None
            and not snap.is_playing(now)
        ):
            return EndedTrack(
                track_id=self._track_id,
                name=self._name,
                how=ENDED_FINISHED,
                at=snap.stops_at,
            )
        return self._last_ended

    def _last_ended_attrs(self, now: float) -> list:
        ended = self._effective_last_ended(now)
        if ended is None or now - ended.at > RECENT_END_WINDOW_S:
            return []
        return [
            ("last_track", ended.name or UNNAMED_TRACK),
            ("last_ended", ended.how),
            ("last_ended_ago_s", str(max(0, round(now - ended.at)))),
        ]


def _now(now: Optional[float]) -> float:
    return time.time() if now is None else float(now)


__all__ = [
    "ENDED_FINISHED",
    "ENDED_REPLACED",
    "ENDED_STOPPED",
    "EndedTrack",
    "MusicStateMemory",
    "RECENT_END_WINDOW_S",
]
