"""voice_state - состояние голоса робота для читателей (шлем Quest, арбитр, кольцо) из контрактов диалога v2.

Раньше его отдавал один топик ``/voice/dialogue/state`` (String ``IDLE|LISTENING|DIALOGUE|SILENCED``) - писал только
``dialogue_node`` v1. При ``dialog_engine: v2`` топик пуст (issue #3911): ``SessionState`` его заменяет только по
сценам, а ходу и речи состояния в нём нет. Здесь состояние складывается из трёх контрактов v2, одна таблица на всех:

* ``TurnEvent`` (``/dialog/turn_event``) - ход: ``addressed`` слушает, ``thinking``/``executing`` думает;
* ``SpeechEvent`` (``/voice/speech/event``) - речь: пока реплика агента звучит (``SpeakingWatch``) - говорит. Стадии
  ``speaking`` ход v2 не публикует (``done`` уходит сразу после запроса речи), поэтому «говорит» берётся из речи;
* ``SessionState`` (``/dialog/session``) - сцена ``Silence`` в стеке: робот молчит.

Словарь выхода - словарь моста Quest (``docs/architecture/meta-quest-api.md`` §4): ``idle | listening | thinking |
speaking`` и ``detail="silenced"`` в тишине. ``VoiceStateTracker`` - чистый класс без ROS; ``watch()`` вешает
подписки на узел.

Адаптер v1 (``on_legacy``): строка старого топика попадает в тот же трекер, итог - наибольшая из активностей по
источникам (``idle < listening < thinking < speaking``), тишина - если молчит любой источник. Источники не
спорят: при v2 v1-топик молчит, при v1 события v2 дают то же самое или богаче. Адаптер и подписка на старый топик
уходят вместе с ``dialogue_node`` (PR-15 #3711, карта удаления 15-0a) - ``LEGACY_STATE`` вычищается одним блоком.

Агент и сцена продублированы литералами (``rob_box_dialog`` есть не во всех образах - quest его не несёт); drift
держит ``test_voice_state.py`` (сверка с ``rob_box_dialog.knowledge``).
"""

from __future__ import annotations

import logging
import time
from typing import Any, Callable, Mapping, NamedTuple, Optional, Tuple

from rob_box_core.expression_client import TURN_EVENT_TOPIC
from rob_box_core.speech_client import SPEECH_EVENT_TOPIC, SpeakingWatch

logger = logging.getLogger(__name__)

#: Топик снимка сессии (latched); писатель - ``dialog_node`` (``identity_question.publish``).
SESSION_TOPIC = "/dialog/session"
#: Агент, чей голос читают шлем и арбитр: ``rob_box_dialog.knowledge.AGENTS[0]`` (тот же, что говорит с людьми).
VOICE_AGENT = "personality"
#: Сцена тишины в стеке сессии (``knowledge.SCENE_KINDS``).
SILENCE_SCENE = "Silence"
#: ``detail`` в тишине.
SILENCED_DETAIL = "silenced"

#: Словарь выхода (``meta-quest-api.md`` §4); порядок - приоритет: побеждает наибольший по индексу.
BRIDGE_STATES: Tuple[str, ...] = ("idle", "listening", "thinking", "speaking")
FALLBACK_STATE = "idle"

#: ``TurnEvent.stage`` -> состояние. ``done``/``failed``/``timeout`` закрывают ход (любой статус, кроме ``skipped``);
#: остальные стадии считаются только на ``started``. Стадии ``speaking`` ход v2 не шлёт - речь читается из SpeechEvent.
TURN_STAGE_STATE: Mapping[str, str] = {
    "addressed": "listening",
    "thinking": "thinking",
    "executing": "thinking",
    "speaking": "speaking",
    "done": "idle",
    "failed": "idle",
    "timeout": "idle",
}

#: TEMP(ADR-0148, PR-15 15-0a): v1-строка ``/voice/dialogue/state`` (``DialogueStateKind.name``) -> (состояние,
#: ``detail``). Ключи UPPERCASE: чужое регистр-значение не должно прокрасться молча. Удаляется с ``dialogue_node``.
LEGACY_STATE: Mapping[str, Tuple[str, Optional[str]]] = {
    "IDLE": ("idle", None),
    "LISTENING": ("listening", None),
    "DIALOGUE": ("speaking", None),
    "SILENCED": ("idle", SILENCED_DETAIL),
}


class VoiceState(NamedTuple):
    """Итог для читателей: ``state`` из :data:`BRIDGE_STATES`, ``detail`` - ``"silenced"`` или ``None``."""

    state: str
    detail: Optional[str] = None


def legacy_state(raw: Any) -> VoiceState:
    """v1-строка -> :class:`VoiceState` (чистая функция адаптера; неизвестное - WARNING и ``idle``, не крах)."""
    text = str(raw or "").strip()
    mapped = LEGACY_STATE.get(text)
    if mapped is None:
        if text:
            logger.warning("voice_state: неизвестное состояние диалога %r -> fallback %r (обнови LEGACY_STATE в "
                           "rob_box_core/voice_state.py)", text, FALLBACK_STATE)
        else:
            logger.debug("voice_state: пустое состояние диалога -> %r", FALLBACK_STATE)
        return VoiceState(FALLBACK_STATE)
    return VoiceState(*mapped)


def _rank(state: str) -> int:
    return BRIDGE_STATES.index(state)


class VoiceStateTracker:
    """Складывает источники состояния голоса; ``on_*`` возвращают новое :class:`VoiceState`, если оно изменилось.

    ``on_legacy`` возвращает состояние всегда (v1 слал кадр на каждое сообщение топика, шлем дедуплицирует сам).
    """

    def __init__(self, agent: str = VOICE_AGENT, clock: Callable[[], float] = time.monotonic) -> None:
        self.agent = agent
        self._speech = SpeakingWatch(agent, clock)
        self._turn = FALLBACK_STATE
        self._scene_silenced = False
        self._legacy = VoiceState(FALLBACK_STATE)
        self._last = VoiceState(FALLBACK_STATE)

    @property
    def current(self) -> VoiceState:
        silenced = self._scene_silenced or self._legacy.detail == SILENCED_DETAIL
        if silenced:
            return VoiceState(FALLBACK_STATE, SILENCED_DETAIL)
        rank = max(_rank(self._turn), _rank(self._legacy.state), _rank("speaking") if self._speech.speaking else 0)
        return VoiceState(BRIDGE_STATES[rank])

    def _changed(self) -> Optional[VoiceState]:
        now = self.current
        if now == self._last:
            return None
        self._last = now
        return now

    def on_turn_event(self, event: Any) -> Optional[VoiceState]:
        """``TurnEvent`` своего агента: стадия по :data:`TURN_STAGE_STATE`; ``skipped`` и чужой агент - мимо."""
        stage, status = str(getattr(event, "stage", "")), str(getattr(event, "status", ""))
        state = TURN_STAGE_STATE.get(stage)
        if getattr(event, "agent", "") != self.agent or state is None or status == "skipped":
            return None
        if state != FALLBACK_STATE and status != "started":
            return None
        self._turn = state
        return self._changed()

    def on_speech_event(self, event: Any) -> Optional[VoiceState]:
        """``SpeechEvent``: звучание реплики своего агента (``started`` ... ``finished|cancelled|rejected``)."""
        self._speech.on_event(event)
        return self._changed()

    def on_session(self, state: Any) -> Optional[VoiceState]:
        """``SessionState`` своего агента: тишина - сцена :data:`SILENCE_SCENE` в стеке."""
        if getattr(state, "agent", "") != self.agent:
            return None
        self._scene_silenced = any(getattr(s, "kind", "") == SILENCE_SCENE for s in getattr(state, "scenes", ()))
        return self._changed()

    def on_legacy(self, raw: Any) -> VoiceState:
        """TEMP(ADR-0148, PR-15): строка старого ``/voice/dialogue/state`` (адаптер v1)."""
        self._legacy = legacy_state(raw)
        self._last = self.current
        return self._last


def watch(node: Any, tracker: VoiceStateTracker, on_change: Callable[[VoiceState], None]) -> None:
    """Подписать узел на три контракта v2; ``on_change(VoiceState)`` - на каждое изменение итога (и строка в логе
    узла ``voice_state -> <состояние>``: по ней видно, что читатель получает состояние на v2).

    ``rob_box_dialog_msgs`` и ``rclpy.qos`` импортируются здесь, не в модуле: трекер живёт и в тестах без ROS.
    """
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: PLC0415
    from rob_box_dialog_msgs.msg import SessionState, SpeechEvent, TurnEvent  # noqa: PLC0415

    def emit(new: Optional[VoiceState]) -> None:
        if new is not None:
            node.get_logger().info(f"voice_state -> {new.state}" + (f" ({new.detail})" if new.detail else ""))
            on_change(new)

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TurnEvent, TURN_EVENT_TOPIC, lambda m: emit(tracker.on_turn_event(m)), 10)
    node.create_subscription(SpeechEvent, SPEECH_EVENT_TOPIC, lambda m: emit(tracker.on_speech_event(m)), 10)
    node.create_subscription(SessionState, SESSION_TOPIC, lambda m: emit(tracker.on_session(m)), latched)


__all__ = [
    "BRIDGE_STATES",
    "FALLBACK_STATE",
    "LEGACY_STATE",
    "SESSION_TOPIC",
    "SILENCED_DETAIL",
    "SILENCE_SCENE",
    "TURN_STAGE_STATE",
    "VOICE_AGENT",
    "VoiceState",
    "VoiceStateTracker",
    "legacy_state",
    "watch",
]
