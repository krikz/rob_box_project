"""Владелец плеера v2 — единственный писатель состояния музыки (ADR-0149 §2.3, §4.3; PR-4).

``PlayerOwner`` решает «что играет» только по событиям адаптера Renardo, а не по именам
вызванных тулов (К1). Он один публикует latched ``/voice/music/state`` (контракт ADR-0141,
``rob_box_voice.core.music_player_state``) и ``/voice/music/event``:

* ``started`` — из потока клока на фактической доле старта плееров, со снимком фазы
  (``phase_in_form`` должна быть 0, I9);
* ``rejected`` — громко (I25): ресурс не прошёл проверку до exec (несуществующий синт →
  ``rejected``, а не тишина, I15), exec упал, или scsynth прислал ``/fail`` «SynthDef not
  found» на трек, который сейчас стартует/играет.

Каждый ``/fail`` логируется с ``track_id`` (I15). Публикация — через переданные функции:
модуль без ROS, тесты гоняют его на фейковом адаптере.

PR-5: очередь из одного подготовленного трека (``queued``, токен ``expected_previous`` — I4,
чужой артефакт выбрасывается с ``artifact_stale``), ``nearly_finished`` по доле клока и стык
следующего трека на границе формы (:meth:`advance`). Что и когда ставить в очередь, решает
``SetSession``. PR-8: блэнд двух дек — входящий встаёт до границы формы, уходящая дека снимается
на границе и освобождается (лог ``deck … free``); ``finished`` конечного трека — с финалом сета.
PR-11: конечный трек (``play(..., once=True)``, classic-песня) — один проход формы, на её конце дека
снимается, событие ``finished`` и снимок ``idle`` с ``finished_track_id`` (как v1 ``repeat=False``).
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Dict, Mapping, Optional, Tuple

from rob_box_voice.core.music_player_state import build_music_event_payload, build_music_state_payload

_LOG = logging.getLogger(__name__)


def _missing_synth_fail(detail: str) -> bool:
    """``/fail`` от ``/s_new`` про несуществующий SynthDef — нота не прозвучала."""
    return detail.startswith("/s_new") and "not found" in detail


class PlayerOwner:
    """Одна дека: старт трека, события ``started``/``rejected``, latched-снимок.

    Args:
        adapter: ``check(program)``, ``start(program, on_started)``, ``stop()`` —
            :class:`~rob_box_mcp_tools.engine.renardo_adapter.RenardoAdapter` или фейк.
        publish_state: публикация JSON снимка ``/voice/music/state``.
        publish_event: публикация JSON события ``/voice/music/event``.
        clock: стенные часы (epoch) — только для ``ts``/``form_ends_at`` наблюдателям.
    """

    def __init__(self, adapter: Any, publish_state: Callable[[str], None],
                 publish_event: Callable[[str], None], *, logger: Any = None,
                 clock: Callable[[], float] = time.time) -> None:
        self._adapter = adapter
        self._publish_state = publish_state
        self._publish_event = publish_event
        self._log = logger or _LOG
        self._clock = clock
        self._lock = threading.Lock()
        self._pending: Optional[str] = None
        self._current: Optional[Dict[str, Any]] = None
        self._dj: Dict[str, Any] = {"enabled": False}
        self._rejected: set = set()
        self._queued: Optional[Tuple[Any, str]] = None  # (программа N+1, expected_previous)
        self._pending_dj: Optional[Dict[str, Any]] = None
        self._once: Optional[str] = None  # track_id, который играет один проход формы
        #: Слушатель ``started`` (``SetSession``): зовётся из потока клока после публикации.
        self.on_started: Optional[Callable[[Dict[str, Any]], None]] = None

    def play(self, program: Any, dj: Optional[Mapping[str, Any]] = None, *, once: bool = False) -> Dict[str, Any]:
        """Проверить ресурсы и запустить программу на деке. Звук — по событию ``started``.

        ``once`` — один проход формы, потом ``finished`` (иначе форма повторяется до стопа или следующего трека).
        """
        problem = self._adapter.check(program)
        if problem is not None:
            return self._reject(program.track_id, *problem)
        with self._lock:  # до start: колбэк клока может прийти раньше, чем start вернётся
            self._pending = program.track_id
            self._dj = {**(dj or {}), "enabled": bool((dj or {}).get("enabled"))}
            self._queued, self._pending_dj = None, None
            self._once = program.track_id if once else None
        try:
            self._adapter.start(program, self.track_started)
        except Exception as exc:  # noqa: BLE001 — отказ громкий, а не тишина
            return self._reject(program.track_id, "exec_error", f"{type(exc).__name__}: {exc}")
        self._log.info(f"🎵 [music v2] exec track_id={program.track_id} deck={program.deck} "
                       f"bpm={program.bpm} — ждём started")
        return {"ok": True, "track_id": program.track_id}

    def track_started(self, snap: Dict[str, Any]) -> None:
        """Колбэк адаптера из потока клока: трек реально встал на долю ``start_beat``."""
        track_id = snap.get("track_id")
        with self._lock:
            if track_id != self._pending:
                stale = True
            else:
                stale = False
                self._pending = None
                self._current = dict(snap, started_at=self._clock())
                if self._pending_dj is not None:
                    self._dj, self._pending_dj = self._pending_dj, None
        if stale:
            self._log.warning(f"⚠️ [music v2] started track_id={track_id} устарел — заменён до старта")
            return
        fields = {k: snap.get(k) for k in ("clock_beat", "start_beat", "phase_in_form", "late_beats",
                                           "bpm", "deck", "form_beats", "players_aligned", "latency_s")}
        log = self._log.info if snap.get("phase_in_form") == 0 and snap.get("players_aligned") else self._log.warning
        log(f"🎵 [music v2] started track_id={track_id} " + " ".join(f"{k}={v}" for k, v in fields.items()))
        self._publish_event(build_music_event_payload("started", track_id, ts=self._clock(), **fields))
        self.publish_state()
        listener = self.on_started
        if listener is not None:
            listener(dict(snap))
        if track_id == self._once:
            self._adapter.at(float(snap["start_beat"]) + float(snap["form_beats"]), lambda: self._finish(track_id))

    def _finish(self, track_id: str) -> None:
        """Поток клока: конечный трек доиграл форму — дека свободна, ``finished`` и ``idle``."""
        with self._lock:
            if (self._current or {}).get("track_id") != track_id:
                return
            self._current, self._once, self._dj = None, None, {"enabled": False}
        self._adapter.stop()
        self._log.info(f"🎵 [music v2] finished track_id={track_id}")
        self._publish_event(build_music_event_payload("finished", track_id, ts=self._clock()))
        self.publish_state(finished_track_id=track_id)

    def enqueue(self, program: Any, expected_previous: str) -> None:
        """Трек N+1 отрендерен и проверен — в очередь (одно место), событие ``queued``."""
        with self._lock:
            self._queued = (program, expected_previous)
        self._log.info(f"🎵 [music v2] queued track_id={program.track_id} expected_previous={expected_previous}")
        self._publish_event(build_music_event_payload(
            "queued", program.track_id, ts=self._clock(), expected_previous=expected_previous))

    def advance(self, at_beat: float, dj: Optional[Mapping[str, Any]] = None, *,
                leave_at: Optional[float] = None) -> bool:
        """Поставить трек из очереди на долю ``at_beat``. ``False`` — ставить нечего.

        ``leave_at`` — блэнд: играющий трек звучит до этой доли (граница его формы), потом его
        дека свободна; без него — стык встык на ``at_beat``.

        Исполняется только артефакт, у которого ``expected_previous`` = играющий трек (I4);
        иначе ``WARNING artifact_stale`` и очередь пуста. Ресурс не прошёл проверку → ``rejected``.
        """
        with self._lock:
            prepared, self._queued = self._queued, None
            current = (self._current or {}).get("track_id")
            leaving_deck = (self._current or {}).get("deck")
        if prepared is None:
            return False
        program, expected = prepared
        if expected != current:
            self._log.warning(f"⚠️ [music v2] artifact_stale track_id={program.track_id} "
                              f"expected_previous={expected} играет={current}")
            return False
        problem = self._adapter.check(program)
        if problem is not None:
            self._reject(program.track_id, *problem)
            return False
        with self._lock:
            self._pending = program.track_id
            self._pending_dj = {**(dj or self._dj), "enabled": bool((dj or self._dj).get("enabled"))}
        self._adapter.cue(program, at_beat, self.track_started,
                          lambda reason, detail: self._reject(program.track_id, reason, detail), leave_at=leave_at,
                          on_left=lambda: self._log.info(f"🎵 [music v2] deck {leaving_deck} free track_id={current}"))
        self._log.info(f"🎵 [music v2] cue track_id={program.track_id} deck={program.deck} at_beat={at_beat} "
                       f"leave_at={leave_at}")
        return True

    def watch(self, track_id: str, form_end_beat: float, lead_beats: float,
              on_nearly_finished: Callable[[str, float], None]) -> None:
        """``nearly_finished`` за ``lead_beats`` до ``form_end_beat``, если трек ещё играет."""
        def _nearly() -> None:
            with self._lock:
                playing = (self._current or {}).get("track_id")
            if playing != track_id:
                return
            self._publish_event(build_music_event_payload(
                "nearly_finished", track_id, ts=self._clock(), form_end_beat=form_end_beat, lead_beats=lead_beats))
            on_nearly_finished(track_id, form_end_beat)

        self._adapter.at(form_end_beat - lead_beats, _nearly)

    def reject(self, track_id: str, reason: str, detail: str) -> Dict[str, Any]:
        """Громкий отказ артефакта, который до деки не дошёл (компоновка, рендер, чужой темп)."""
        return self._reject(track_id, reason, detail)

    def on_server_fail(self, detail: str) -> None:
        """``/fail`` от scsynth → ``track_id`` (I15); «SynthDef not found» → ``rejected``."""
        with self._lock:
            track_id = self._pending or (self._current or {}).get("track_id")
            first = track_id is not None and track_id not in self._rejected
        self._log.warning(f"🔴 [music v2] /fail track_id={track_id}: {detail}")
        if first and _missing_synth_fail(detail):
            self._reject(track_id, "server_fail", detail)

    def stop(self, reason: str = "user_stop") -> Dict[str, Any]:
        """Снять деку и опубликовать ``idle``."""
        self._adapter.stop()
        with self._lock:
            last = (self._current or {}).get("track_id") or self._pending
            self._current = None
            self._pending = None
            self._dj, self._once = {"enabled": False}, None
            self._queued, self._pending_dj = None, None
        self._log.info(f"🎵 [music v2] stop track_id={last} reason={reason}")
        self.publish_state()
        return {"ok": True, "track_id": last}

    def publish_state(self, finished_track_id: Optional[str] = None) -> None:
        """Опубликовать latched-снимок (после старта, стопа и при подключении к ROS)."""
        now = self._clock()
        with self._lock:
            current = dict(self._current) if self._current else None
            dj = dict(self._dj)
        form_ends_at = None
        if current:
            pass_s = current["form_beats"] * 60.0 / current["bpm"]
            passes = int(max(0.0, now - current["started_at"]) // pass_s) + 1
            form_ends_at = current["started_at"] + passes * pass_s
        self._publish_state(build_music_state_payload(
            playing=current is not None, track_id=(current or {}).get("track_id"),
            form_ends_at=form_ends_at, dj=dj, finished_track_id=finished_track_id, ts=now))

    def _reject(self, track_id: str, reason: str, detail: str) -> Dict[str, Any]:
        with self._lock:
            self._rejected.add(track_id)
            if self._pending == track_id:
                self._pending = None
        self._log.warning(f"⛔ [music v2] rejected track_id={track_id} reason={reason}: {detail}")
        self._publish_event(build_music_event_payload(
            "rejected", track_id, ts=self._clock(), reason=reason, detail=detail))
        return {"ok": False, "track_id": track_id, "reason": reason, "detail": detail}


__all__ = ["PlayerOwner"]
