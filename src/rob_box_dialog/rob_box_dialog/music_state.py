"""Раздел ``music_state`` снимка хода для LLM — из снимка плеера (ADR-0141; ADR-0150 §7.3).

Раньше тег собирался эвристикой ``beat``: «играет», если взведён
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

Раздел — данные (словарь), текст из него строит ``context.build_context``; снимок плеера — любой объект с полями
``MusicPlayerState`` (``rob_box_voice.core.music_player_state``): пакет чистый и rob_box_voice не импортирует.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Tuple

#: Снимок ``/voice/music/state`` (``MusicPlayerState``): ``ts``, ``track_id``, ``stops_at``, ``finished_track_id``,
#: ``dj``, ``dj_info``, ``is_playing(now)``.
MusicPlayerState = Any

#: Сколько секунд после конца трека раздел ещё показывает «недавно играл».
#: Дольше получаса «что ты включал?» — уже история разговора, не состояние.
RECENT_END_WINDOW_S = 1800.0

#: Как трек перестал звучать.
ENDED_FINISHED = "finished"  # доиграл форму сам (``finished_track_id``)
ENDED_STOPPED = "stopped"    # остановлен (стоп, watchdog, idle-TTL)
ENDED_REPLACED = "replaced"  # вместо него запущен другой трек

UNNAMED_TRACK = "без названия"


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

    def section(self, *, now: Optional[float] = None) -> Dict[str, str]:
        """Раздел ``music_state`` снимка хода: снимок владельца плеера — единственный источник (трек библиотеки
        генераций тоже на деке, ADR-0158 PR-2)."""
        current = _now(now)
        return dict(self._playback_attrs(current) + self._last_ended_attrs(current))

    def _playback_attrs(self, now: float) -> List[Tuple[str, str]]:
        snap = self._snapshot
        if snap is None:
            # Плеер ещё ничего не сообщил — честно «не знаю».
            return [("playing", "unknown"), ("dj", "unknown")]
        dj = "on" if snap.dj else "off"
        if not snap.is_playing(now):
            return [("playing", "no"), ("dj", dj)]
        title = self._name or _track_title(snap.dj_info)
        attrs = [("playing", "yes"), ("track", title)]
        stops_at = snap.stops_at
        started, length = snap.dj_info.get("started_at"), snap.dj_info.get("length_s")
        if stops_at is None and isinstance(started, (int, float)) and isinstance(length, (int, float)):
            stops_at = float(started) + float(length)  # пьеса v2: один проход формы
        if stops_at is None:
            attrs.append(("repeat", "loop"))
        else:
            attrs.append(("repeat", "once"))
            left = max(0, round(stops_at - now))
            attrs.append(("ends_in_s", str(left)))
        attrs.append(("dj", dj))
        return attrs + _set_attrs(snap)

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

    def _last_ended_attrs(self, now: float) -> List[Tuple[str, str]]:
        ended = self._effective_last_ended(now)
        if ended is None or now - ended.at > RECENT_END_WINDOW_S:
            return []
        return [
            ("last_track", ended.name or UNNAMED_TRACK),
            ("last_ended", ended.how),
            ("last_ended_ago_s", str(max(0, round(now - ended.at)))),
        ]


#: Поля сета v2 из снимка (``dj`` владельца плеера, ADR-0149 §2.3) → ключи раздела: имя в разделе ← ключ снимка.
#: Факты играющего трека (``engine.dj_lines``): мелодия, следующие по плану, части темы без мелодий в библиотеке.
SET_ATTRS = (("set_theme", "theme"), ("set_track_no", "track_no"), ("set_tracks", "tracks"), ("melody", "melody"),
             ("played", "played"), ("next_melodies", "next_melodies"), ("not_found", "not_found"), ("bpm", "bpm"),
             ("persona", "persona"), ("source", "source"))


#: Ключи раздела, что описывают ИГРАЮЩЕЕ (``MusicStateMemory.section`` при ``playing=yes``): после стопа их нет.
_PLAYING_ONLY = ("track", "repeat", "ends_in_s", *(attr for attr, _ in SET_ATTRS))


def after_stop(section: Optional[Mapping[str, str]]) -> Dict[str, str]:
    """Раздел ``music_state`` после того, как музыку выключил стоп-тул ЭТОГО хода (``_Turn.music_stopped``, исход
    ``done`` владельца плеера): снимок плеера ``/voice/music/state`` приходит позже стопа (в n709 остаток фразы уходил
    модели через 30 мс после стопа), и раздел с прежним ``playing=yes`` вернул бы её к «музыка играет». Играющее
    убирается, ``playing=no``, играющий трек - в ``last_track`` (остановлен, только что). Раздел уже «не играет» - как
    есть."""
    data = dict(section or {})
    if data.get("playing") != "yes":
        return data
    track = data.get("track")
    out = {k: v for k, v in data.items() if k not in _PLAYING_ONLY and k != "playing"}
    out = {"playing": "no", **out}
    return {**out, "last_track": track, "last_ended": ENDED_STOPPED, "last_ended_ago_s": "0"} if track else out


def _track_title(info: Mapping[str, Any]) -> str:
    """Как названа играющая дорожка в разделе (``track``): короткое описание из фактов, которое строит владелец плеера
    (``brief``: «название», стиль, темп - без номера и id); снимок без него (сервер старее подписчика) - ``title``."""
    return str(info.get("brief") or info.get("title") or "") or UNNAMED_TRACK


def _set_attrs(snap: MusicPlayerState) -> List[Tuple[str, str]]:
    """Сет v2 — из latched-снимка, не из догадки (ADR-0149 §5.2); у v1 этих полей в снимке нет."""
    info = snap.dj_info
    return [(attr, _attr_value(info[key])) for attr, key in SET_ATTRS if info.get(key) not in (None, "", [])]


def _attr_value(value: object) -> str:
    """Список фактов (мелодии; сыгранные — «трек: мелодия») — через запятую, пустые пропущены; остальное — строкой."""
    if not isinstance(value, (list, tuple)):
        return str(value)
    items = (f"{v[0]}: {v[1]}" if isinstance(v, (list, tuple)) and len(v) == 2 else str(v) for v in value)
    return ", ".join(i for i in items if i)


def _now(now: Optional[float]) -> float:
    return time.time() if now is None else float(now)


__all__ = [
    "after_stop",
    "ENDED_FINISHED",
    "ENDED_REPLACED",
    "ENDED_STOPPED",
    "EndedTrack",
    "MusicStateMemory",
    "RECENT_END_WINDOW_S",
]
