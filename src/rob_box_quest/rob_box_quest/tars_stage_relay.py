"""ROS → WS реле стадий ТАРС-в-шлем (issue #3253 Ш2, ADR-0078 §3.6).

Путь реплики ТАРС в шлем и откуда берётся каждая стадия ``tars_state``:

1. ``accepted`` — ``/avatar/stt/result`` (stt_node, wake «ТАРС» принят);
   шлёт ``QuestNode._on_avatar_stt_result``.
2. ``thinking`` — ``/avatar/tars/stage`` от supervisor'а: ход LLM начался
   (``AvatarSupervisor._on_avatar_command`` перед ``_run_agent_sync``).
   Оттуда же ``idle``, если ход кончился без реплики в шлем.
3. Ответ LLM → ``/avatar/tts/request`` (sink=headset, ``speech_id``) →
   :meth:`TarsStageRelay.on_request`.
4. ``speaking`` — первый ``/avatar/tts/audio``, доставленный в шлем
   (:meth:`on_chunk`).
5. ``idle`` + ``operator_tts_done`` — после ``/voice/tts/finished`` с тем же
   ``speech_id`` И оценки конца звука в шлеме (:meth:`tick`); при ошибке
   синтеза — ``idle`` + ``operator_tts_error``; на барж-ине (PTT) — сразу.

Логика стадий — в :class:`~rob_box_quest.core.tars_stages.TarsStageTracker`
(без часов и сети); здесь только разбор ROS-сообщений и отправка в WS.
Отдельный класс, а не методы ``QuestNode``: класс-бюджет ADR-0145.

Формат WS-события — тот, что парсит клиент
(``webxr_client/src/ui/tars_state_indicator.ts:parseTarsStateEvent``):
``{type: "tars_state", stage, request_id, ts_ms, reason?}``.
"""

from __future__ import annotations

import json
import time
from typing import Any, Callable, Optional

from .core.tars_stages import Event, TarsStageTracker

# Стадии, которые реле пропускает от supervisor'а в WS.
SUPERVISOR_TARS_STAGES: frozenset[str] = frozenset({"thinking", "idle"})


def _json_dict(msg: Any) -> Optional[dict]:
    """String-сообщение → dict, или None (битый JSON / не объект)."""
    try:
        payload = json.loads(getattr(msg, "data", "") or "")
    except (json.JSONDecodeError, TypeError):
        return None
    return payload if isinstance(payload, dict) else None


class TarsStageRelay:
    """Факты ROS → трекер стадий → WS (``tars_state`` / ``operator_tts_*``)."""

    def __init__(
        self,
        ws_server: Callable[[], Any],
        logger: Any,
        *,
        clock: Callable[[], float] = time.monotonic,
        tracker: Optional[TarsStageTracker] = None,
    ) -> None:
        # ws_server — геттер: в QuestNode сервер создаётся позже подписок.
        self._ws = ws_server
        self._log = logger
        self._clock = clock
        self.tracker = tracker or TarsStageTracker()

    # ── входы ──────────────────────────────────────────────────────────

    def on_supervisor_stage(self, msg: Any) -> None:
        """ROS ``/avatar/tars/stage`` → WS ``tars_state`` (thinking | idle)."""
        payload = _json_dict(msg)
        if payload is None or payload.get("stage") not in SUPERVISOR_TARS_STAGES:
            return
        self.broadcast(
            str(payload["stage"]),
            str(payload.get("request_id", "") or ""),
            str(payload.get("reason", "") or ""),
        )

    def on_tts_finished(self, msg: Any) -> None:
        """ROS ``/voice/tts/finished`` → конец синтеза реплики (по speech_id).

        Топик общий с голосом личности: чужие speech_id трекер отбросит.
        ``queued=True`` — чанк лишь поставлен в очередь tts_node. Не-JSON
        (``silero_warming:...``) пропускаем.
        """
        payload = _json_dict(msg)
        if payload is None or payload.get("queued"):
            return
        self.emit(self.tracker.on_finished(
            str(payload.get("speech_id", "") or ""),
            bool(payload.get("success", False)),
            str(payload.get("error", "") or ""),
            self._clock(),
        ))

    def on_request(self, request_id: str, speech_id: Any) -> None:
        """Реплика ТАРС зарегистрирована на шлем (``/avatar/tts/request``)."""
        self.emit(self.tracker.on_request(request_id, speech_id, self._clock()))

    def on_chunk(self, delivered: Any, request_id: str, n_bytes: int, sample_rate: int) -> None:
        """Чанк ушёл в шлем. ``delivered`` не True (ws закрыт) — звука нет."""
        if delivered is not True:
            return
        self.emit(self.tracker.on_chunk(request_id, n_bytes, sample_rate, self._clock()))

    def on_barge_in(self) -> None:
        """PTT оператора: клиент уже оборвал звук — закрываем стадию."""
        self.emit(self.tracker.on_cancel("barge_in", self._clock()))

    def tick(self) -> None:
        """Таймер QuestNode: отложенный idle после конца звука в шлеме."""
        self.emit(self.tracker.tick(self._clock()))

    # ── выходы ─────────────────────────────────────────────────────────

    def emit(self, events: list[Event]) -> None:
        """События трекера → WS: tars_state / operator_tts_done / _error."""
        for event in events:
            request_id = str(event.get("request_id", "") or "")
            reason = str(event.get("reason", "") or "")
            kind = event.get("kind")
            if kind == "stage":
                self.broadcast(str(event.get("stage")), request_id, reason)
            elif kind == "done":
                self._ws().deliver_operator_tts_done(request_id)
            elif kind == "error":
                self._ws().deliver_operator_tts_error(request_id, reason)

    def broadcast(self, stage: str, request_id: str, reason: str = "") -> None:
        """WS ``tars_state`` всем сессиям + INFO-лог стадии (raw для e2e)."""
        event: dict[str, Any] = {
            "type": "tars_state",
            "stage": stage,
            "request_id": request_id,
            "ts_ms": int(time.time() * 1000),
        }
        if reason:
            event["reason"] = reason
        self._log.info(
            f"🎧 [#3253] tars_state {stage} request_id={request_id[:24]}"
            + (f" reason={reason}" if reason else "")
        )
        try:
            self._ws().broadcast_json_event(event)
        except Exception as e:  # noqa: BLE001
            self._log.debug(f"tars_state {stage} broadcast failed: {e}")
