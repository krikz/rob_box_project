"""Планировщик ресурсов в ноде v1 (ADR-0150 §5.6, PR-7b #3699): сборка исполнителя тулов и корм снимка владельцев.

Функции модуля, а не методы ``DialogueNode`` (бюджет класса ADR-0145 не растёт): нода зовёт их из своих колбэков.
Снимок (:class:`rob_box_dialog.plan.resources.ResourceState`) кормят только события владельцев: ``SpeechEvent``
(tts_node), ``SoundEvent`` (sound_node), снимок ``/voice/music/state`` (плеер). Движение в v1 снимком не ведётся
(Nav2-результат нода не слушает): «стой» исполняет Tier-1 грамматика, «направо» во время песни — MERGE по таблице.
"""

from __future__ import annotations

import time
from typing import Any

from rob_box_dialog.plan.plan_events import PlanEvent
from rob_box_dialog.plan.runner import PlanRunner
from rob_box_dialog_msgs.msg import PlanEvent as PlanEventMsg
from rob_box_dialog_msgs.msg import SoundEvent, SpeechEvent

from .plan_tools import PlanToolExecutor

PLAN_EVENT_TOPIC = "/dialog/plan_event"
_SPEECH_KINDS = {getattr(SpeechEvent, n): n[5:].lower() for n in dir(SpeechEvent) if n.startswith("KIND_")}
_SOUND_KINDS = {getattr(SoundEvent, n): n[5:].lower() for n in dir(SoundEvent) if n.startswith("KIND_")}


def make_tool_executor(node: Any, provider_adapter: Any, on_tool: Any) -> PlanToolExecutor:
    """Исполнитель тулов личности под планом; ``PlanEvent`` — в ``/dialog/plan_event`` и строкой лога."""
    pub = node.create_publisher(PlanEventMsg, PLAN_EVENT_TOPIC, 10)

    def publish(event: PlanEvent) -> None:
        msg = PlanEventMsg()
        msg.block_id, msg.turn_id, msg.epoch, msg.agent = event.block_id, event.turn_id, event.epoch, event.agent
        msg.kind, msg.segment_no, msg.after_block = event.code, event.segment_no, event.after_block
        msg.eta_ms, msg.synced, msg.reason = event.eta_ms, event.synced, event.reason
        msg.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    return PlanToolExecutor(provider_adapter, PlanRunner(publish=publish, log=node.get_logger().info), on_tool=on_tool)


def _planner(node: Any) -> Any:
    return getattr(getattr(node, "_tool_executor", None), "planner", None)


def begin_turn(node: Any, user_input: str) -> None:
    """Tier-1 плана по словам человека этого хода («а потом …»)."""
    planner = _planner(node)
    if planner is not None:
        planner.begin_turn(user_input)


def on_speech(node: Any, msg: SpeechEvent) -> None:
    planner = _planner(node)
    if planner is None:
        return
    planner.state.on_speech(
        _SPEECH_KINDS.get(int(msg.kind), ""), msg.speech_id, turn_id=msg.turn_id, epoch=int(msg.epoch),
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


__all__ = ["PLAN_EVENT_TOPIC", "begin_turn", "make_tool_executor", "on_music", "on_sound", "on_speech"]
