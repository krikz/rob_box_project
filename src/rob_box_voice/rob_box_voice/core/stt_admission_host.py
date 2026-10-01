"""stt_admission_host.py — ROS-bound host adapter for the STT admission pipeline.

ADR-0145 §4 (DialogueNode P1 step 1) — pure move out of
``dialogue_node.py`` (no behaviour change). Owns:

* ``_PENDING_USER_MESSAGES_MAX`` / ``_UTTERANCE_ID_WAIT_SEC`` — the two
  timing constants that gate the STT admission queue;
* ``_DialogueSttHost`` — the adapter that satisfies the
  :class:`rob_box_voice.core.stt_admission.SttAdmissionHost` Protocol for
  :class:`rob_box_voice.dialogue_node.DialogueNode`.

``dialogue_node.py`` re-exports both names so existing imports
(``from rob_box_voice.dialogue_node import _DialogueSttHost``) and
monkeypatches keep working unchanged.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Optional, Tuple

from std_msgs.msg import String

from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
)
from rob_box_voice.core.dialogue_guards import is_music_stop_command
from rob_box_voice.core.dialogue_helpers import sanitize_speaker_name
from rob_box_voice.core.dialogue_text import is_unsilence_command
from rob_box_voice.core.command_parser import IntentType
from rob_box_voice.observability import (
    is_metrics_enabled,
    record_quick_decide_verdict,
)
from rob_box_voice.scheduler.quick_decide import QuickVerdict, quick_decide

if TYPE_CHECKING:  # pragma: no cover -- avoids a circular import at runtime
    from rob_box_voice.dialogue_node import DialogueNode

class _NoIntake:
    """Заглушка для нод/харнессов без ``_material_intake``."""

    @staticmethod
    def on_text(_text: str) -> bool:
        return False


_NO_INTAKE = _NoIntake()

# S7 (scheduler-segments-merge, issue #968) — upper bound on
# ``_pending_user_messages`` so a run of barge-ins during one very long
# LLM turn cannot grow the queue unbounded. Appending past this cap
# drops the OLDEST queued phrase (keep the most recent user intent) and
# logs a warning — see ``_DialogueSttHost.enqueue_pending``.
_PENDING_USER_MESSAGES_MAX: int = 5

# Issue #2862 — сколько ``_on_stt`` ждёт ``/voice/stt/utterance`` своей
# фразы, если текст обогнал id. Оба топика публикуются stt_node подряд из
# одного потока, расхождение — задержка планировщика executor'а (мс), а не
# секунды. Текст без id вовсе (GUI/bench/инъекция харнесса) платит эту
# задержку один раз и идёт с ``utterance_id=None``.
_UTTERANCE_ID_WAIT_SEC: float = 0.5


# ---------------------------------------------------------------------------
# _DialogueSttHost — adapter from SttAdmissionHost Protocol to DialogueNode
# ---------------------------------------------------------------------------
# Issue #2628 / ADR-0021 R1 — bridges the pure SttAdmission pipeline in
# ``rob_box_voice.core.stt_admission`` to the ROS-bound surfaces of
# DialogueNode (locks, FSM, publishers, accumulators).
#
# Lock discipline (single source of truth):
#
#   * ``_speaker_lock``  — held only inside ``accumulate_without_wake``,
#     wrapping the snapshot of ``_current_speaker``. Released before any
#     logger call or accumulator mutation.
#   * ``_task_lock``     — held only inside ``enqueue_pending``, wrapping
#     the read of ``_run_task`` and the ``_pending_user_messages``
#     append (S7 segment-merge queue, issue #968). Released before
#     ``_dispatch_turn``.
#
# No other lock is acquired while either is held → no inversion. The
# orchestrator never sees a lock; it only calls into these methods.
# ---------------------------------------------------------------------------


class _DialogueSttHost:
    """Adapter that satisfies :class:`SttAdmissionHost` for DialogueNode.

    The orchestrator instantiates this once per ``_on_stt`` invocation
    so the side effects share the per-call snapshot (``state``,
    ``was_idle``, ``text``) without going through thread-locals.

    Methods follow the byte-for-byte semantics of the inline branches
    they replaced — see ``docs/adr/0021-cc-budget.md`` and issue #2628
    for the migration checklist.
    """

    __slots__ = ("_node", "media_miss")

    def __init__(self, node: "DialogueNode") -> None:
        self._node = node
        # Issue #3176 — ``callable(clean_text)``: вернуть реплику в приём
        # после MediaCommandStep (заказ по имени мимо базы мелодий).
        # ``None`` — возвращать некуда (тестовые харнессы без ``_on_stt``).
        self.media_miss = None

    # -- helpers --------------------------------------------------------

    def _bump_counter(self, key: str) -> None:
        """``self._llm_skipped_counter[key] += 1`` — log + counter."""
        node = self._node
        node._llm_skipped_counter[key] += 1

    def _log(self, msg: str) -> None:
        self._node.get_logger().info(msg)

    # -- SttAdmissionHost callbacks -------------------------------------

    def unsilence(self, text_lower: str) -> bool:
        node = self._node
        if not is_unsilence_command(text_lower):
            return False
        node._dsm.on_event(DialogueEvent.UNSILENCE)
        node._publish_state()
        return True

    def accumulate_without_wake(
        self,
        speaker_tag: Optional[str],
        text: str,
    ) -> bool:
        node = self._node
        accumulator = getattr(node, "_speech_accumulator", None)
        if not getattr(node, "_accumulate_no_wake_enabled", False):
            return False
        if accumulator is None:
            return False
        # Legacy L2256-2271 — speaker snapshot under _speaker_lock; emit
        # diag log; add to accumulator; log acceptance.
        with node._speaker_lock:
            sp = dict(getattr(node, "_current_speaker", {}) or {})
        sp_name = (
            sanitize_speaker_name(sp.get("name")) if sp.get("is_known") else ""
        )
        node._emit_backlog_diag_log(sp, sp_name, speaker_tag, text)
        accumulator.add(
            text,
            speaker_tag=speaker_tag,
            speaker_name=sp_name or None,
        )
        node.get_logger().info(
            f"🗒️ [backlog] accumulated (no_wake_word) "
            f"tag={speaker_tag!r} speaker={sp_name or 'незнакомец'!r} "
            f"text={text[:60]!r}"
        )
        return True

    def handle_silence_command(self) -> bool:
        node = self._node
        # Legacy L2304-2296 — only true silence phrases reach here
        # (music-stop override handled by SilenceCommandStep itself).
        node._llm_skipped_counter["silence_command"] += 1
        node._handle_silence()
        return True

    def is_music_stop_command(self, text_lower: str) -> bool:
        # Issue #1279 — «хватит диджеить» is music-stop, not silence.
        return is_music_stop_command(text_lower)

    def handle_command_intent(self, text: str, text_lower: str) -> bool:
        node = self._node
        if not getattr(node, "_command_intent_gate_enabled", False):
            return False
        # 🔴 FIX (issue #2971): раньше сверялись только с
        # ``node._MUSIC_STOP_OVERRIDES`` (голые фиксированные фразы) —
        # после того как #2971 убрал из списка «диджеить»/«диджея»/
        # «диджей режим» (ложные срабатывания на голое существительное
        # без стоп-глагола), «хватит диджеить» перестало матчить ЭТУ
        # проверку и команда уходила в command_intent gate вместо LLM.
        # ``is_music_stop_command`` (списки ФИКСИРОВАННЫХ фраз + общий
        # паттерн «стоп-глагол + муз. существительное») — единый
        # источник правды, используемый везде в этом модуле.
        if self.is_music_stop_command(text_lower):
            return False
        command = node._command_parser.parse(text)
        if (
            command.intent == IntentType.UNKNOWN
            or command.confidence < node._command_intent_gate_confidence
        ):
            return False
        node._llm_skipped_counter["command_intent"] += 1
        node._cancel_run("command intent (issue 1279)", stop_tts=True)
        node.get_logger().info(
            f"🎯 [issue 1279] command intent="
            f"{command.intent.value} conf={command.confidence:.2f} "
            f"— LLM dispatch skipped (command_node handles): "
            f"{text[:60]!r}"
        )
        return True

    def handle_media_command(self, text: str) -> bool:
        # Issue #3134 — медиакоманды кодом, до LLM (и в TG, и в DJ-режиме).
        # Issue #3227 — попутно: музыкальный материал в реплике принимается
        # кодом (реплика при этом идёт дальше — LLM ответит человеку).
        getattr(self._node, "_material_intake", _NO_INTAKE).on_text(text)
        return self._node._route_media_command(text, on_miss=self.media_miss)

    def reset_session(
        self,
        text: str,
        text_lower: str,
        tg_chat_id: Optional[int],
    ) -> bool:
        node = self._node
        # Legacy uses ``clean``; ``StripWakeWordStep`` may have rewritten
        # ``ctx.text``. We read ``text_lower`` as the orchestrator's
        # ``text_lower`` is the cleaned lowercased snapshot — matches
        # the legacy ``text_lower`` arg at L2342.
        if not node._is_new_session_command(text, text_lower, tg_chat_id):
            return False
        node._llm_skipped_counter["new_session"] += 1
        node._reset_dialogue_session()
        node.get_logger().info(
            f"🧹 [new-session] session reset: text={text[:60]!r} "
            f"tg={bool(tg_chat_id)}"
        )
        return True

    def flush_pending_backlog(self) -> None:
        self._node._pending_backlog_flush = True

    def quick_decide_verdict(
        self, clean: str
    ) -> Tuple[str, bool, bool]:
        node = self._node
        tg_chat_id = None  # set by caller via state, but quick_decide
        # only needs ``source="stt" | "tg"``. We use ``source="stt"``
        # because the orchestrator only calls us on mic path; TG input
        # bypasses barge-in by design (no wake word = no barge-in).
        verdict = quick_decide(
            clean, source="stt",
            active_group=None, clock=time.monotonic,
            previous_text=getattr(node, "_last_stt_text", None),
            previous_ts=getattr(node, "_last_stt_ts", None),
        )
        node._last_stt_text = clean
        node._last_stt_ts = time.monotonic()
        # W2-6 (issue #968) — record the verdict for metrics.
        if is_metrics_enabled():
            try:
                record_quick_decide_verdict(verdict.value)
            except Exception as _metric_exc:  # noqa: BLE001
                node.get_logger().debug(
                    f"⚠️ [metrics] record_quick_decide_verdict failed: "
                    f"{_metric_exc!r}"
                )
        ignored = verdict is QuickVerdict.IGNORE
        pending_llm = verdict is QuickVerdict.PENDING_LLM
        return (verdict.value, ignored, pending_llm)

    def enqueue_pending(self, clean: str) -> bool:
        node = self._node
        # S7 (scheduler-segments-merge) — under _task_lock snapshot the
        # live task; if alive, queue; if queue is full, drop oldest.
        with node._task_lock:
            live_task = node._run_task
        if live_task is None or live_task.done():
            return False
        if len(node._pending_user_messages) >= _PENDING_USER_MESSAGES_MAX:
            dropped, _dropped_ts = node._pending_user_messages.popleft()
            node.get_logger().warning(
                f"⚠️ [S7] pending_user_messages overflow "
                f"(max={_PENDING_USER_MESSAGES_MAX}), dropping "
                f"oldest: {dropped[:60]!r}"
            )
        node._pending_user_messages.append((clean, time.monotonic()))
        node.get_logger().info(
            f"📥 [S7] turn in flight — queued: {clean[:60]!r}"
        )
        return True

    def cancel_inflight(self, stop_tts: bool) -> None:
        # Issue #2939 — за отменой всегда идёт DispatchTriggerStep:
        # сессию принимает новый ход.
        self._node._cancel_run("new STT input", stop_tts=stop_tts, hand_over=True)

    def transition_idle_to_wake(self) -> bool:
        node = self._node
        if node._dsm.current_state != DialogueStateKind.IDLE:
            return False
        node._dsm.on_event(DialogueEvent.WAKE_WORD)
        node._publish_state()
        return True

    def transition_stt_result(self) -> None:
        node = self._node
        node._dsm.on_event(DialogueEvent.STT_RESULT)
        node._publish_state()

    def publish_state(self) -> None:
        self._node._publish_state()

    def trigger_thinking_sfx(self) -> None:
        node = self._node
        sfx = String()
        sfx.data = "thinking"
        node._sound_trigger_pub.publish(sfx)
