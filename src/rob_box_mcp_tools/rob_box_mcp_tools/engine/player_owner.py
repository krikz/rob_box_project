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

На этом шаге одна дека и бесконечный повтор формы: очередь, ``nearly_finished``,
``finished`` — PR-5, блэнд двух дек — PR-8.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Dict, Mapping, Optional

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

    def play(self, program: Any, dj: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
        """Проверить ресурсы и запустить программу на деке. Звук — по событию ``started``."""
        problem = self._adapter.check(program)
        if problem is not None:
            return self._reject(program.track_id, *problem)
        with self._lock:  # до start: колбэк клока может прийти раньше, чем start вернётся
            self._pending = program.track_id
            self._dj = {**(dj or {}), "enabled": bool((dj or {}).get("enabled"))}
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
        if stale:
            self._log.warning(f"⚠️ [music v2] started track_id={track_id} устарел — заменён до старта")
            return
        fields = {k: snap.get(k) for k in ("clock_beat", "start_beat", "phase_in_form", "late_beats",
                                           "bpm", "deck", "form_beats", "players_aligned", "latency_s")}
        log = self._log.info if snap.get("phase_in_form") == 0 and snap.get("players_aligned") else self._log.warning
        log(f"🎵 [music v2] started track_id={track_id} " + " ".join(f"{k}={v}" for k, v in fields.items()))
        self._publish_event(build_music_event_payload("started", track_id, ts=self._clock(), **fields))
        self.publish_state()

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
            self._dj = {"enabled": False}
        self._log.info(f"🎵 [music v2] stop track_id={last} reason={reason}")
        self.publish_state()
        return {"ok": True, "track_id": last}

    def publish_state(self) -> None:
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
            form_ends_at=form_ends_at, dj=dj, ts=now))

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
