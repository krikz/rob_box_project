"""DJ-сет v2: очередь, предгенерация N+1, переход по ``nearly_finished`` (ADR-0149 §4.1–§4.4, PR-5).

``SetSession`` решает, *что* играет следующим и *когда* переходить; звук, события и снимок —
у ``PlayerOwner`` (единственный писатель). Время — только доли клока Renardo от реального
старта трека (колбэк ``started``): ``form_end_beat = start_beat + form_beats``; стенные часы
сессия не читает.

* На ``started(N)`` в фоне сразу компонуется и рендерится N+1 на другой деке → ``queued``.
* ``nearly_finished(N)`` за ``lead_beats`` до конца формы: N+1 в очереди — **блэнд** (PR-8, §3.12):
  N+1 встаёт за ``model.blend_bars(N, N+1)`` тактов до границы формы N (фаза 0 на такте), оба
  трека звучат вместе, бочка и бас меняются на такте свопа (это свойство форм, а не движка),
  дека N снимается на границе и свободна. Формы не сводятся (``blend_bars = 0``) — стык ровно на
  границе формы (PR-5). Не готов (не успел, отказ рендера, чужой темп) — **текущий трек играет ещё
  проход формы** («продлить, а не замолчать», I1), проверка повторяется через проход.
* Темп один на сет (I7, §4.4): его ставит первый трек (``Clock.update_tempo_now``), трек с
  другим темпом до деки не доходит — ``rejected{tempo_mismatch}`` и продление.

Источник следующего трека — шов ``TrackSource``: ``(track_no, deck) -> Track``. По умолчанию
:func:`compose_source` — ``arrange.compose`` по плану сета (``set_plan.seeded_plan``, PR-3b: темп,
дуга энергии, тоника — там, своей копии плана здесь нет); план от LLM (PR-10) — тот же шов.
"""

from __future__ import annotations

import logging
import threading
from typing import Any, Callable, Dict, List, Mapping, Optional

from rob_box_music.arrange.compose import compose
from rob_box_music.model import BEATS_PER_BAR, Track, blend_bars
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import SetPlan

_LOG = logging.getLogger(__name__)

#: ``nearly_finished`` за ``(phrase_bars + 1) × 4`` доли до конца формы (ADR-0149 §4.3): 36 долей
#: ≈ 16 с при 132 BPM — запас на предгенерацию повторно, если первая не удалась.
PHRASE_BARS = 8
NEARLY_LEAD_BEATS = float((PHRASE_BARS + 1) * 4)

#: Источник следующего трека: ``(track_no, deck) -> Track``.
TrackSource = Callable[[int, str], Track]


def compose_source(plan: SetPlan, melodies: Optional[Mapping[str, str]] = None) -> TrackSource:
    """Треки сета из ``arrange.compose`` по плану; хук только что сыгранного трека — последним в выборе."""
    recent: List[str] = []

    def next_track(track_no: int, deck: str) -> Track:
        track = compose(plan, track_no, melodies=melodies, recent_hooks=tuple(recent), deck=deck)
        if track.hook is not None and track.hook.source:
            recent.insert(0, track.hook.source)
        return track

    return next_track


def _other_deck(deck: str) -> str:
    return "B" if deck == "A" else "A"


def _start_thread(fn: Callable[[], None]) -> None:
    threading.Thread(target=fn, name="rbx-set-pregen", daemon=True).start()


class SetSession:
    """Один DJ-сет поверх ``PlayerOwner``.

    Args:
        owner: :class:`~rob_box_mcp_tools.engine.player_owner.PlayerOwner` (или фейк с тем же API).
        source: следующий трек сета, см. :data:`TrackSource`.
        set_id: идентификатор сета (префикс ``track_id``).
        bpm: темп сета — единственный (I7).
        dj: поля снимка ``dj`` (персона, тема…); ``set_id``/``track_no``/``bpm`` дописываются.
        submit: где компоновать N+1 (по умолчанию — фоновый поток: клок не ждёт рендера).
        lead_beats: за сколько долей до конца формы ``nearly_finished``.
    """

    def __init__(self, owner: Any, source: TrackSource, *, set_id: str, bpm: int,
                 dj: Optional[Mapping[str, Any]] = None, submit: Callable[[Callable[[], None]], None] = _start_thread,
                 lead_beats: float = NEARLY_LEAD_BEATS, logger: Any = None) -> None:
        self._owner = owner
        self._source = source
        self.set_id = set_id
        self.bpm = int(bpm)
        self._dj = dict(dj or {})
        self._submit = submit
        self._lead = float(lead_beats)
        self._log = logger or _LOG
        self._lock = threading.Lock()
        self._active = False
        self._tracks: Dict[str, int] = {}  # track_id → номер в сете
        self._models: Dict[str, Track] = {}  # track_id → модель (играющий и в очереди: длина блэнда)
        self._next_id: Optional[str] = None  # что последним поставлено в очередь
        self._current: Optional[Dict[str, Any]] = None  # {track_id, no, deck, form_beats}
        self._preparing = False

    def dj_fields(self, track_no: int) -> Dict[str, Any]:
        title = f"{self._dj.get('theme') or 'диджей-сет'} · трек {track_no}"  # имя для <music_state> (ADR-0149 §2.3)
        return {**self._dj, "enabled": True, "set_id": self.set_id, "track_no": track_no, "bpm": self.bpm,
                "title": title}

    def start(self) -> Dict[str, Any]:
        """Трек 1 на деке A; звук — по ``started``. Отказ — ``{ok: False, reason}`` (громко, I25)."""
        program = self._prepare(1, "A")
        if program is None:
            return {"ok": False, "reason": "track_rejected", "set_id": self.set_id}
        with self._lock:
            self._active = True
        self._owner.on_started = self._on_started
        result = self._owner.play(program, dj=self.dj_fields(1))
        if not result.get("ok"):
            self._deactivate()
        return {**result, "set_id": self.set_id, "bpm": self.bpm}

    def stop(self, reason: str = "user_stop") -> Dict[str, Any]:
        """Сет закрыт: очередь пуста, плеер ``idle``; запланированное на клоке устаревает."""
        self._deactivate()
        self._log.info(f"🎧 [set v2] {self.set_id} stop reason={reason}")
        return {**self._owner.stop(reason), "set_id": self.set_id}

    @property
    def active(self) -> bool:
        return self._active

    def _deactivate(self) -> None:
        with self._lock:
            self._active = False
        if self._owner.on_started == self._on_started:
            self._owner.on_started = None

    def _prepare(self, track_no: int, deck: str) -> Optional[Any]:
        """Компоновка → рендер → тот же темп. ``None`` — артефакт отвергнут (событие ``rejected``)."""
        try:
            track = self._source(track_no, deck)
            program = render(track, deck)
        except Exception as exc:  # noqa: BLE001 — отказ громкий; музыка (если есть) играет дальше
            self._owner.reject(f"{self.set_id}:{track_no:02d}:{deck}", "compose_error", f"{type(exc).__name__}: {exc}")
            return None
        if program.bpm != self.bpm:
            self._owner.reject(program.track_id, "tempo_mismatch", f"bpm {program.bpm} ≠ темп сета {self.bpm}")
            return None
        with self._lock:
            self._tracks[program.track_id] = track_no
            self._models[program.track_id] = track
        return program

    def _on_started(self, snap: Mapping[str, Any]) -> None:
        """Поток клока: трек N встал — считать конец формы от его доли и готовить N+1."""
        track_id = snap.get("track_id")
        with self._lock:
            no = self._tracks.get(track_id)
            if not self._active or no is None:
                return
            self._current = {"track_id": track_id, "no": no, "deck": snap.get("deck"),
                             "form_beats": float(snap["form_beats"])}
            self._models = {k: v for k, v in self._models.items() if k == track_id}
        form_end = float(snap["start_beat"]) + float(snap["form_beats"])
        self._log.info(f"🎧 [set v2] {self.set_id} трек {no} started track_id={track_id} form_end_beat={form_end}")
        self._owner.watch(track_id, form_end, self._lead, self._on_nearly_finished)
        self._pregenerate_soon()

    def _pregenerate_soon(self) -> None:
        with self._lock:
            if self._preparing or not self._active:
                return
            self._preparing = True
        self._submit(self._pregenerate)

    def _pregenerate(self) -> None:
        """Фон: N+1 на другой деке → ``queued`` с токеном N."""
        with self._lock:
            current = dict(self._current or {})
        try:
            program = self._prepare(current["no"] + 1, _other_deck(current["deck"])) if current else None
        finally:
            with self._lock:
                self._preparing = False
        with self._lock:
            still = self._active and (self._current or {}).get("track_id") == current.get("track_id")
        if program is not None and still:
            with self._lock:
                self._next_id = program.track_id
            self._owner.enqueue(program, current["track_id"])

    def _on_nearly_finished(self, track_id: str, form_end: float) -> None:
        """Поток клока: N+1 готов — блэнд до ``form_end`` (или стык на ней); нет — ещё проход формы N (I1)."""
        with self._lock:
            current = dict(self._current or {})
            if not self._active or current.get("track_id") != track_id:
                return
        overlap = self._blend_beats(track_id)
        advanced = self._owner.advance(form_end - overlap, dj=self.dj_fields(current["no"] + 1),
                                       leave_at=form_end if overlap else None)
        if not advanced:
            self._log.warning(f"⚠️ [set v2] {self.set_id} трек {current['no'] + 1} не готов — "
                              f"продлеваю трек {current['no']} ещё на проход формы")
            self._pregenerate_soon()
        elif overlap:
            self._log.info(f"🎧 [set v2] {self.set_id} блэнд трек {current['no']}→{current['no'] + 1}: "
                           f"{overlap / BEATS_PER_BAR:g} тактов с доли {form_end - overlap}")
        # Страховка и при стыке: не встанет N+1 (exec упал) — N играет, проверка через проход.
        self._owner.watch(track_id, form_end + current["form_beats"], self._lead, self._on_nearly_finished)

    def _blend_beats(self, track_id: str) -> float:
        """Доли блэнда играющего трека с тем, что в очереди; 0 — стык встык (формы не сводятся)."""
        with self._lock:
            leaving, incoming = self._models.get(track_id), self._models.get(self._next_id or "")
        if leaving is None or incoming is None:
            return 0.0
        beats = float(blend_bars(leaving, incoming) * BEATS_PER_BAR)
        if not beats or beats > self._lead - BEATS_PER_BAR:  # вход должен успеть встать на такт после события
            self._log.warning(f"⚠️ [set v2] {self.set_id} блэнда нет ({leaving.track_id}→{incoming.track_id}: "
                              f"{beats:g} долей при lead {self._lead:g}) — стык на границе формы")
            return 0.0
        return beats


__all__ = ["NEARLY_LEAD_BEATS", "PHRASE_BARS", "SetSession", "TrackSource", "compose_source"]
