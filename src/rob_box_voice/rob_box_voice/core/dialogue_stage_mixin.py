"""Стадии хода в ``DialogueNode`` v1: клей между нодой и :class:`~rob_box_voice.core.turn_stages.TurnStages`.

ADR-0150 PR-3b (#3690). Миксин ``DialogueNode`` (бюджет класса ADR-0145 не растёт): допуск реплики, тело хода, тулы,
``SpeechEvent`` речи личности и ответы исполнителей рефлексов (``ExpressionEvent`` / ``SoundEvent``) сходятся в стадиях
хода. Ждёт от ноды ``get_logger()``, ``_turn_id`` и ``_stages`` (создаёт ``DialogueNode.__init__``).
"""

from __future__ import annotations

from typing import Any, Optional

from rob_box_core import expression_client
from rob_box_dialog_msgs.msg import ExpressionEvent, SoundEvent, SpeechEvent

from .greeting_window import on_speech_event as on_greeting_speech
from .identity_question import on_speech_event as on_identity_speech
from .llm_skip_reasons import LLMSkipReason
from .stt_admission import SttOutcome, SttOutcomeKind
from .turn_stages import TurnStages
from .turn_tools import error_reason

#: Причины ``HANDLED`` шага адресации «не для робота» (в бэклог, координатору): рефлекса «слушаю» им не положено.
_NOT_FOR_US_REASONS = (LLMSkipReason.BACKLOG_ACCUMULATED.value, LLMSkipReason.COORDINATOR.value)


class DialogueStageMixin:
    """Стадии хода личности: рефлексы, тулы, речь, ответы исполнителей."""

    def _turn_stages(self) -> TurnStages:
        """Стадии хода личности; у ноды, собранной без ``__init__`` (тестовый стенд), - пустые (ничего не публикуют)."""
        stages = getattr(self, "_stages", None)
        if stages is None:
            stages = self._stages = TurnStages(lambda **fields: None, lambda **fields: None)
        return stages

    def _stage_admission_closed(self, utterance_id: Optional[str], outcome: SttOutcome) -> None:
        """Допуск закрыл фразу без LLM: не для робота -> ход закрыт и лицо успокоено, код ответил сам -> дочитать речь."""
        if outcome.kind is SttOutcomeKind.DROP or outcome.reason in _NOT_FOR_US_REASONS:
            self._turn_stages().dropped(utterance_id, outcome.reason)
        elif outcome.reason != LLMSkipReason.BARE_WAKE.value:
            self._turn_stages().handled(utterance_id)

    def _stage_finish(self, result: Any, cancelled: bool, failure: str, retry_dispatched: bool) -> None:
        """Тело хода закончено: исход ``done|failed`` - из ошибки LLM / исключения; ретрай гуарда ход продолжает."""
        if retry_dispatched:
            return
        error = failure or (str(result.error)[:60] if result is not None and result.error is not None else "")
        self._turn_stages().finish(self._turn_id, ok=not error, reason=error, cancelled=cancelled)

    def _on_tool_stage(self, name: str, args: Any, result: Any) -> None:
        """Тул хода (``SchedulerToolExecutor``): до вызова - стадия ``executing{cls}``, после - его исход."""
        if result is None:
            self._turn_stages().tool_call(name, dict(args or {}))
            if name == "speak_text":  # #3765: что LLM сказала в ходе — для Chooser повторного вопроса личности
                self._turn_llm_said = [*getattr(self, "_turn_llm_said", ()), str(dict(args or {}).get("text") or "")]
        else:
            is_error = bool(result.is_error)
            self._turn_stages().tool_result(name, is_error, error_reason(result.content) if is_error else "")

    def _on_speech_event(self, msg: SpeechEvent) -> None:
        self._turn_stages().on_speech(int(msg.kind), msg.agent, msg.reason, msg.speech_id)
        watch = getattr(self, "_speaking_watch", None)  # ADR-0150 PR-6: канал «громче» по звучащей речи
        if watch is not None:
            watch.on_event(msg)  # до окна вопроса: «речь хода кончилась» читается по уже обновлённому watch (#3765)
        on_identity_speech(self, int(msg.kind), msg.speech_id, msg.agent)  # окно ответа на вопрос об имени (#3771)
        on_greeting_speech(self, int(msg.kind), msg.speech_id, msg.agent)  # окно ответа на приветствие (#3776)

    def _on_expression_event(self, msg: ExpressionEvent) -> None:
        self._turn_stages().on_expression_event(
            msg.executor, msg.turn_id, msg.kind == ExpressionEvent.KIND_STARTED, msg.reason)

    def _on_sound_event(self, msg: SoundEvent) -> None:
        """Пропуск звука виден в логе диалога (A23: молчаливых пропусков нет), остальное - ``ros2 topic echo``."""
        if msg.kind == SoundEvent.KIND_SKIPPED:
            self.get_logger().info(f"🔇 [sound] {msg.sound or '-'} пропущен: {msg.reason} request={msg.request_id[:8]}")

    def _express_earcon(self, earcon: str) -> None:
        """Earcon вне хода (приветствие при старте): тот же путь, что у рефлексов, - ``ExpressionRequest``."""
        expression_client.express(self, turn_id="", epoch=0, agent="personality", earcon=earcon)
