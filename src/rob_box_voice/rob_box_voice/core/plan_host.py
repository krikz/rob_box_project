"""Планировщик ресурсов в ноде v1 (ADR-0150 §5.6, PR-7b #3699): сборка исполнителя тулов и корм снимка владельцев.

Функции модуля, а не методы ``DialogueNode`` (бюджет класса ADR-0145 не растёт): нода зовёт их из своих колбэков.
Снимок (:class:`rob_box_dialog.plan.resources.ResourceState`) кормят только события владельцев: ``SpeechEvent``
(tts_node), ``SoundEvent`` (sound_node), снимок ``/voice/music/state`` (плеер). Движение в v1 снимком не ведётся
(Nav2-результат нода не слушает): «стой» исполняет Tier-1 грамматика, «направо» во время песни — MERGE по таблице.

Выступление (PR-7c, :mod:`.perform_host`): его реплики видит только его хост (их ``SpeechEvent`` в снимок речи не
идут — пред-синтез без звука голос не занимает); колонку ``perform`` снимка ведут события блока
(``block_start``/``prediction``/``block_end``).
"""

from __future__ import annotations

import time
from typing import Any, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Busy
from rob_box_dialog.plan.plan_events import PlanEvent
from rob_box_dialog.plan.runner import PlanRunner
from rob_box_dialog_msgs.msg import PlanEvent as PlanEventMsg
from rob_box_dialog_msgs.msg import SoundEvent, SpeechEvent

from . import perform_ask
from .music_player_state import parse_music_event
from .perform_host import TICK_S, PerformHost
from .plan_tools import PlanToolExecutor

PLAN_EVENT_TOPIC = "/dialog/plan_event"
_SPEECH_KINDS = {getattr(SpeechEvent, n): n[5:].lower() for n in dir(SpeechEvent) if n.startswith("KIND_")}
_SOUND_KINDS = {getattr(SoundEvent, n): n[5:].lower() for n in dir(SoundEvent) if n.startswith("KIND_")}


def make_tool_executor(node: Any, provider_adapter: Any, on_tool: Any) -> PlanToolExecutor:
    """Исполнитель тулов личности под планом и хост выступления; ``PlanEvent`` — в ``/dialog/plan_event`` и логом."""
    pub = node.create_publisher(PlanEventMsg, PLAN_EVENT_TOPIC, 10)

    def publish(event: PlanEvent) -> None:
        msg = PlanEventMsg()
        msg.block_id, msg.turn_id, msg.epoch, msg.agent = event.block_id, event.turn_id, event.epoch, event.agent
        msg.kind, msg.segment_no, msg.after_block = event.code, event.segment_no, event.after_block
        msg.eta_ms, msg.synced, msg.reason = event.eta_ms, event.synced, event.reason
        msg.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    planner = PlanRunner(publish=publish, log=node.get_logger().info)
    executor = PlanToolExecutor(provider_adapter, planner, on_tool=on_tool)

    def perform_event(event: PlanEvent) -> None:
        """Событие выступления: колонка ``perform`` снимка (APPEND «а потом …» ждёт конца выступления) и публикация."""
        active = event.kind != "block_end"
        remaining = event.eta_ms / 1000.0 if event.kind == "prediction" else None
        if event.kind in ("block_start", "prediction", "block_end"):
            planner.state.on_block("perform", event.block_id, active, remaining)
            planner.changed()
        planner.emit(event)

    executor.performer = PerformHost(node, executor.execute_owned, lambda: getattr(node, "_loop", None),
                                     perform_event, kn.SPEECH_OUTPUT_LATENCY_MS, clock=lambda: _now(node),
                                     playing=lambda: _foreign_music(node),
                                     on_ask=lambda title: perform_ask.open_window(node, title),
                                     tts_provider=lambda: _tts_provider(node))
    node.create_timer(TICK_S, executor.performer.tick)
    return executor


def _foreign_music(node: Any) -> str:
    """Что играет сейчас (трек, сет) — название для вопроса «выключить и зачитать?» (О9′); тишина — ``""``."""
    player = getattr(node, "_music_player_state", None)
    if player is None or not (player.is_playing() or player.dj):
        return ""
    return str((player.dj_info or {}).get("title") or "музыка")


def _tts_provider(node: Any) -> str:
    """Провайдер TTS сейчас: фактический из ``/voice/tts/provider_state`` (после фолбека), иначе номинальный
    параметр ``tts_provider``; неизвестен — ``""`` (выступление тогда без пауз MiniMax, с причиной в логе).
    ``prefetch`` — не провайдер: так tts_node метит звучание готового пред-синтеза, кто синтезировал — не сказано."""
    actual = getattr(node, "_actual_tts_provider", None)
    if actual and actual != "prefetch":
        return str(actual)
    try:
        return str(node.get_parameter("tts_provider").value or "")
    except Exception:  # noqa: BLE001 — параметр не объявлен (тестовая нода): провайдер неизвестен
        return ""


def _now(node: Any) -> float:
    """Время выступления — ROS-часы ноды (стенные, как ``SpeechRequest.at`` и ``beat_at`` плеера)."""
    return node.get_clock().now().nanoseconds / 1e9


def _planner(node: Any) -> Any:
    return getattr(getattr(node, "_tool_executor", None), "planner", None)


def performer(node: Any) -> Optional[PerformHost]:
    """Хост выступления ноды (``None`` — исполнителя тулов нет)."""
    return getattr(getattr(node, "_tool_executor", None), "performer", None)


def begin_turn(node: Any, user_input: str) -> None:
    """Tier-1 плана по словам человека этого хода («а потом …»)."""
    planner = _planner(node)
    if planner is not None:
        planner.begin_turn(user_input)


def on_speech(node: Any, msg: SpeechEvent) -> None:
    kind = _SPEECH_KINDS.get(int(msg.kind), "")
    host = performer(node)
    if host is not None and host.owns(msg.speech_id):
        host.on_speech(kind, msg.speech_id, int(msg.duration_ms), int(msg.spoken_chars))
        return
    planner = _planner(node)
    if planner is None:
        return
    planner.state.on_speech(
        kind, msg.speech_id, turn_id=msg.turn_id, epoch=int(msg.epoch),
        text_len=int(msg.text_len), duration_ms=int(msg.duration_ms), spoken_chars=int(msg.spoken_chars),
        seconds=float(msg.seconds))
    planner.changed()


def on_sound(node: Any, msg: SoundEvent) -> None:
    planner = _planner(node)
    if planner is None:
        return
    planner.state.on_sound(_SOUND_KINDS.get(int(msg.kind), ""), msg.request_id)
    planner.changed()


def on_music(node: Any, snapshot: Any) -> None:
    """Снимок плеера: играет ли трек и сколько осталось (``stops_at`` конечного трека, по стенным часам)."""
    planner = _planner(node)
    if planner is None or snapshot is None:
        return
    now = time.time()
    stops_at = getattr(snapshot, "stops_at", None)
    planner.state.on_music(bool(snapshot.is_playing(now)), str(getattr(snapshot, "track_id", "") or ""),
                           remaining_s=None if stops_at is None else max(0.0, float(stops_at) - now))
    planner.changed()


def on_music_event(node: Any, msg: Any) -> None:
    """``/voice/music/event``: ``nearly_finished``/``finished``/``idle`` подложки — выступлению."""
    host = performer(node)
    event = parse_music_event(getattr(msg, "data", None))
    if host is not None and event is not None:
        host.on_music_event(event.event, event.track_id, event.fields)


def perform_busy(node: Any) -> Busy:
    """Что занято для валидатора команды: идущее выступление и его ACTIVE-сегмент (``edit_pending``)."""
    host = performer(node)
    return Busy(active_segment=None if host is None else host.active_segment)


__all__ = ["PLAN_EVENT_TOPIC", "begin_turn", "make_tool_executor", "on_music", "on_music_event", "on_sound",
           "on_speech", "perform_busy", "performer"]
