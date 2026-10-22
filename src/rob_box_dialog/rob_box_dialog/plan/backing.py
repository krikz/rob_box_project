"""Подложка выступления (ADR-0150 §5.6.3, решение О10): попытки, срок ``started``, сетка тактов, продление, снятие.

Подложку играет владелец плеера (``PlayerOwner``, ADR-0149) по ``request_music(intent=backing)``; здесь — только
решение выступления о ней по событиям плеера: ``required`` (BML) — не ``started`` за ``PERFORM_BACKING_START_S`` или
``rejected`` → вторая попытка другим настроением → фраза по причине ``perform.no_backing.{reason}``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Optional, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.plan.sync import BeatGrid


@dataclass(frozen=True)
class Backing:
    """Эффект ``request_music(intent=backing)``: настроение и сколько секунд подложка должна звучать от сейчас;
    ``extend`` — продлить играющую подложку, а не ставить новую."""

    mood: str
    min_s: float
    extend: bool = False


@dataclass(frozen=True)
class MusicStop:
    """Эффект: снять подложку — ``bar`` (outro на ближайшем такте) или ``now`` (сразу)."""

    boundary: str
    reason: str


def grid_of(fields: Mapping[str, Any]) -> Optional[BeatGrid]:
    """Сетка тактов из ``started{bpm, beat_at, bar_beats, start_beat}``; чего-то нет — ``None`` (чтение без тактов)."""
    try:
        return BeatGrid(float(fields["bpm"]), float(fields["beat_at"]), float(fields["bar_beats"]),
                        float(fields.get("start_beat") or 0.0))
    except (KeyError, TypeError, ValueError):
        return None


class BackingTrack:
    """Подложка одного выступления. Состояния: ``none`` (не просили) → ``requested`` → ``playing`` | ``failed``;
    ``playing`` → ``ended`` (кончилась или сняли снаружи) | ``stopped`` (сняли мы)."""

    def __init__(self, kind: str) -> None:
        self.moods: Tuple[str, ...] = tuple(kn.PERFORM_BACKING_MOODS.get(kind, ()))
        self.state = "none"
        self.attempt = 0
        self.deadline: Optional[float] = None
        self.track_id = ""
        self.grid: Optional[BeatGrid] = None
        self.reason = ""
        self.late_until = 0.0

    @property
    def wanted(self) -> bool:
        return bool(self.moods)

    @property
    def mood(self) -> str:
        return self.moods[max(0, self.attempt - 1)] if self.moods else ""

    @property
    def sounding(self) -> bool:
        """Играет или может вот-вот встать — снимать при конце выступления."""
        return self.state in ("requested", "playing")

    def request(self, now: float, min_s: float) -> Backing:
        """Следующая попытка: настроение по порядку, срок ``started`` — ``PERFORM_BACKING_START_S``."""
        self.attempt += 1
        self.state, self.deadline = "requested", now + kn.PERFORM_BACKING_START_S
        return Backing(self.mood, round(min_s, 1))

    def extension(self, now: float, fields: Mapping[str, Any], speech_end: float, min_s: float) -> Optional[Backing]:
        """``nearly_finished``: речь (с outro) кончится позже подложки — продлить её проходом формы."""
        end_beat = fields.get("form_end_beat")
        music_end = self.grid.beat_time(float(end_beat)) if self.grid is not None and end_beat is not None else now
        if speech_end + kn.PERFORM_OUTRO_S <= music_end:
            return None
        return Backing(self.mood, round(min_s, 1), extend=True)

    def started(self, track_id: str, fields: Mapping[str, Any]) -> None:
        self.state, self.track_id, self.deadline = "playing", track_id, None
        self.grid = grid_of(fields)

    def ended(self) -> None:
        self.state, self.grid = "ended", None

    def stopped(self, now: float) -> None:
        self._give_up("stopped", now)

    def fail(self, reason: str, now: float) -> str:
        """Отказ без новой попытки: причина фразы (``quota``/``network``/``timeout`` или ``error``)."""
        self._give_up("failed", now)
        self.reason = reason if reason in kn.PERFORM_NO_BACKING_REASONS else "error"
        return self.reason

    def _give_up(self, state: str, now: float) -> None:
        """Подложку больше не ждём. Если она так и не встала — её поздний ``started`` ещё ``PERFORM_BACKING_START_S``
        считается нашим (снять, а не оставить играть без выступления)."""
        self.late_until = now + kn.PERFORM_BACKING_START_S if self.state == "requested" else 0.0
        self.state, self.deadline = state, None

    def can_retry(self) -> bool:
        return self.attempt < len(self.moods)

    def timed_out(self, now: float) -> bool:
        return self.state == "requested" and self.deadline is not None and now >= self.deadline

    def verdict(self, kind: str, track_id: Optional[str], now: float) -> str:
        """Что событие плеера значит для выступления: ``ignore``; ``late`` (наша подложка встала, когда её уже не
        ждут — снять); ``started``/``rejected`` (исход запроса); ``nearly`` (скоро конец формы); ``ended`` (кончилась
        или деку заняла другая музыка — тактов нашей подложки больше нет)."""
        if self.state == "playing" and kind == "started" and track_id and track_id != self.track_id:
            return "ended"
        if not self.mine(track_id, now):
            return "ignore"
        if self.state in ("stopped", "failed"):
            return "late" if kind == "started" else "ignore"
        if self.state == "requested":
            return kind if kind in ("started", "rejected") else "ignore"
        if self.state != "playing":
            return "ignore"
        return {"nearly_finished": "nearly", "finished": "ended", "idle": "ended"}.get(kind, "ignore")

    def mine(self, track_id: Optional[str], now: float) -> bool:
        """Событие о нашей подложке: трек тот же; до ``started`` id трека не знаем — наше любое, пока просим или
        недолго после отказа (поздний ``started``)."""
        if self.track_id:
            return track_id == self.track_id
        return self.state == "requested" or now <= self.late_until


__all__ = ["Backing", "BackingTrack", "MusicStop", "grid_of"]
