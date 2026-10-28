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
    #: Человек согласился сменить играющую музыку на бит («да» на ``perform.ask_replace_music``, О9 (б)): сначала
    #: снять её, потом ставить бит.
    replace: bool = False


@dataclass(frozen=True)
class MusicStop:
    """Эффект: снять НАШУ подложку — ``bar`` (outro на ближайшем такте) или ``now`` (сразу).

    ``track_id`` обязателен (ADR-0148, хвост ревью PR #4111): снимается именно этот трек, а на деке уже чужая музыка
    (человек заказал трек, пока бит просили или после его отказа) - тул ничего не трогает. Подложку, чей запрос ещё в
    пути (id не знаем), вслепую не снимают: id приходит ответом тула, и поздний ``started`` (или ``rejected`` с id
    ещё не вставшего трека) снимается по нему (:meth:`BackingTrack.verdict` → ``late``)."""

    boundary: str
    reason: str
    track_id: str


def grid_of(fields: Mapping[str, Any]) -> Optional[BeatGrid]:
    """Сетка тактов из ``started{bpm, beat_at, bar_beats, start_beat}``; чего-то нет — ``None`` (чтение без тактов)."""
    try:
        return BeatGrid(float(fields["bpm"]), float(fields["beat_at"]), float(fields["bar_beats"]),
                        float(fields.get("start_beat") or 0.0))
    except (KeyError, TypeError, ValueError):
        return None


class BackingTrack:
    """Подложка одного выступления. Состояния: ``none`` (не просили) → [``asking`` — играет чужая музыка, спросили
    человека] → ``requested`` → ``playing`` | ``failed``; ``playing`` → ``ended`` (кончилась или сняли снаружи) |
    ``stopped`` (сняли мы) | ``kept`` (выступление сняли, а бит человек оставил — «а музыку оставь», #3994: дальше он
    просто играющая музыка, выступление его не снимает и не продлевает).

    Своя подложка — только та, что заказало само выступление (``Performance.backing`` при старте, «да» на смену
    музыки, «добавь бит» командой выступления), и узнаётся она по исходу своего запроса (``track_id``). Музыку, которую
    человек заказал тулом музыки (до выступления или во время него), выступление не присваивает и концом не снимает
    (#4085)."""

    def __init__(self, kind: str) -> None:
        self.moods: Tuple[str, ...] = tuple(kn.PERFORM_BACKING_MOODS.get(kind, ()))
        self.state = "none"
        self.attempt = 0
        self.deadline: Optional[float] = None
        self.track_id = ""
        self.grid: Optional[BeatGrid] = None
        self.reason = ""
        #: Запросов подложки (``request``), исход которых (ответ тула) ещё не пришёл. Пока он >0, исход - наш, даже
        #: если выступление уже сдалось (срок/«хватит»): поздний ``started`` снимается по id, а не вслепую.
        self.awaiting = 0

    @property
    def wanted(self) -> bool:
        return bool(self.moods)

    @property
    def mood(self) -> str:
        return self.moods[max(0, self.attempt - 1)] if self.moods else ""

    @property
    def unresolved(self) -> bool:
        """Исход подложки ещё неизвестен (ждём ``started`` или ответа человека) — выступление не начинается."""
        return self.state in ("requested", "asking")

    @property
    def sounding(self) -> bool:
        """Играет или может вот-вот встать — снимать при конце выступления."""
        return self.state in ("requested", "playing")

    def request(self, now: float, min_s: float, replace: bool = False) -> Backing:
        """Следующая попытка: настроение по порядку, срок ``started`` — ``PERFORM_BACKING_START_S``."""
        self.attempt += 1
        self.awaiting += 1
        self.state, self.deadline = "requested", now + kn.PERFORM_BACKING_START_S
        return Backing(self.mood, round(min_s, 1), replace=replace)

    def ask(self, now: float) -> None:
        """Играет чужая музыка: бит — только если человек согласится; ответ ждём ``PERFORM_ASK_WAIT_S``."""
        self.state, self.deadline = "asking", now + kn.PERFORM_ASK_WAIT_S

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
        self._give_up("stopped")

    def kept(self) -> bool:
        """«А музыку оставь»: играющий бит больше не наш (``True``); не играл — оставлять нечего (``False``)."""
        if self.state != "playing":
            return False
        self.state, self.deadline = "kept", None
        return True

    def fail(self, reason: str, now: float) -> str:
        """Отказ без новой попытки: причина фразы (``quota``/``network``/``timeout`` или ``error``)."""
        self._give_up("failed")
        self.reason = reason if reason in kn.PERFORM_NO_BACKING_REASONS else "error"
        return self.reason

    def _give_up(self, state: str) -> None:
        """Подложку больше не ждём. Если запрос ещё в пути (``awaiting``), его поздний исход остаётся нашим: ``started``
        снимается по id, а не оставляется играть без выступления."""
        self.state, self.deadline = state, None

    def can_retry(self) -> bool:
        return self.attempt < len(self.moods)

    def timed_out(self, now: float) -> bool:
        return self.unresolved and self.deadline is not None and now >= self.deadline

    def verdict(self, kind: str, track_id: Optional[str], now: float, own: bool = False) -> str:
        """Что событие значит для выступления: ``ignore``; ``late`` (исход нашего запроса пришёл, когда выступление
        уже сдалось: ``started`` - трек встал, ``rejected`` с id - трек поставлен, но не встал и может встать
        позже; снять по id); ``started``/``rejected`` (исход запроса); ``nearly`` (скоро конец формы); ``ended``
        (кончилась или деку заняла другая музыка — тактов нашей подложки больше нет). ``own`` — исход НАШЕГО запроса
        (ответ ``request_music(intent=backing)``), а не событие плеера о любой музыке (:meth:`mine`)."""
        if self.state == "playing" and kind == "started" and track_id and track_id != self.track_id:
            return "ended"
        mine = self.mine(track_id, own)
        if own and self.awaiting:
            self.awaiting -= 1  # ответ тула - исход одного запроса; следующий ответ принадлежит следующему
        if not mine:
            return "ignore"
        if self.state in ("stopped", "failed"):
            return "late" if kind == "started" or (kind == "rejected" and track_id) else "ignore"
        if self.state == "requested":
            return kind if kind in ("started", "rejected") else "ignore"
        if self.state != "playing":
            return "ignore"
        return {"nearly_finished": "nearly", "finished": "ended", "idle": "ended"}.get(kind, "ignore")

    def mine(self, track_id: Optional[str], own: bool = False) -> bool:
        """Событие о нашей подложке (владение — по id трека, #4085): id знаем — трек тот же; до ``started`` id не
        знаем — наше только исход нашего же запроса (``own``), пока он не пришёл (``awaiting``) — и во время ожидания,
        и после того, как выступление сдалось (поздний исход снимается по id). Событие плеера о чужом треке (человек
        заказал музыку, пока бит просили или после отказа) — не наше: конец выступления его не снимает."""
        if self.track_id:
            return track_id == self.track_id
        return own and self.awaiting > 0


__all__ = ["Backing", "BackingTrack", "MusicStop", "grid_of"]
