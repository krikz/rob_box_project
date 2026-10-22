"""stt_admission_host.py — ROS-bound host adapter for the STT admission pipeline.

ADR-0145 §4 (DialogueNode P1 step 1) — pure move out of
``dialogue_node.py`` (no behaviour change). Owns:

* ``_DialogueSttHost`` — the adapter that satisfies the
  :class:`rob_box_voice.core.stt_admission.SttAdmissionHost` Protocol for
  :class:`rob_box_voice.dialogue_node.DialogueNode`.

``dialogue_node.py`` re-exports the name so existing imports
(``from rob_box_voice.dialogue_node import _DialogueSttHost``) and
monkeypatches keep working unchanged.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, List, Optional

from rob_box_dialog_msgs.msg import Intent as IntentMsg

from rob_box_dialog.address import Addressed
from rob_box_dialog.backlog import Backlog, Heard, phrase_count, pickup_summary
from rob_box_dialog.grammar import Tier1Command
from rob_box_dialog.knowledge import ADDRESS_WAIT_S
from rob_box_dialog.words import STOP_NONE, stop_verdict
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
)
from rob_box_voice.core.dialogue_guards import is_music_stop_command
from rob_box_voice.core.greeting_window import not_a_reply, take_reply, window
from rob_box_voice.core import perform_ask
from rob_box_voice.core.identity_question import question, take_answer
from rob_box_voice.core.command_intent import fill_intent
from rob_box_voice.core.command_parser import IntentType
from rob_box_voice.core.tier1_host import Tier1Runner, runner_of

if TYPE_CHECKING:  # pragma: no cover -- avoids a circular import at runtime
    from rob_box_voice.core.stt_admission import SttContext
    from rob_box_voice.dialogue_node import DialogueNode


class _NoIntake:
    """Заглушка для нод/харнессов без ``_material_intake``."""

    @staticmethod
    def on_text(_text: str) -> bool:
        return False


_NO_INTAKE = _NoIntake()


# ---------------------------------------------------------------------------
# _DialogueSttHost — adapter from SttAdmissionHost Protocol to DialogueNode
# ---------------------------------------------------------------------------
# Issue #2628 / ADR-0021 R1 — bridges the pure SttAdmission pipeline in
# ``rob_box_voice.core.stt_admission`` to the ROS-bound surfaces of
# DialogueNode (locks, FSM, publishers, accumulators).
#
# Locks: the host takes none (ADR-0150 PR-5 — backlog entries carry their own
# DOA / person_ref, the ``_current_speaker`` snapshot is no longer read here).
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

    __slots__ = ("_node", "media_miss", "tier1_resume", "utterance_id")

    def __init__(self, node: "DialogueNode", utterance_id: Optional[str] = None) -> None:
        self._node = node
        # ADR-0150 PR-3b (#3690) — id фразы: рефлекс addressed не открывает второй ход, если ранний сигнал
        # (InterruptRequest) уже открыл его для этой же фразы.
        self.utterance_id = utterance_id
        # Issue #3176 — ``callable(clean_text)``: вернуть реплику в приём
        # после MediaCommandStep (заказ по имени мимо базы мелодий).
        # ``None`` — возвращать некуда (тестовые харнессы без ``_on_stt``).
        self.media_miss = None
        # ADR-0150 PR-6 (В19) — то же для спорного стопа: Chooser сказал
        # «часть команды», реплика продолжает приём после Tier1Step.
        self.tier1_resume = None

    # -- helpers --------------------------------------------------------

    def _bump_counter(self, key: str) -> None:
        """``self._llm_skipped_counter[key] += 1`` — log + counter."""
        node = self._node
        node._llm_skipped_counter[key] += 1

    def _log(self, msg: str) -> None:
        self._node.get_logger().info(msg)

    # -- SttAdmissionHost callbacks -------------------------------------

    def backlog(self) -> Optional[Backlog]:
        node = self._node
        if not getattr(node, "_accumulate_no_wake_enabled", False):
            return None
        return getattr(node, "_backlog", None)

    def remember_unaddressed(self, heard: Heard) -> bool:
        not_a_reply(self._node, heard)  # окно после приветствия открыто, а фраза не ответ (#3776)
        if self.identity_question() is not None:  # окно ответа открыто, а фраза не похожа на ответ (#3695)
            self._node.get_logger().info(
                f"❓ [Identity #3695] не ответ (грамматика) → бэклог: doa={heard.doa_deg!r} text={heard.text[:60]!r}")
        backlog = self.backlog()
        if backlog is None:
            return False
        backlog.add(heard)
        self._node.get_logger().info(
            f"🗒️ [backlog] accumulated (no_wake_word) doa={heard.doa_deg!r} "
            f"person={heard.person_ref or '-'} faces={len(heard.faces)} voice_s={heard.voice_s:.1f} "
            f"text={heard.text[:60]!r}"
        )
        return True

    def addressed(self, decision: Addressed, recent: Optional[List[Dict[str, Any]]], heard: Heard) -> None:
        # ADR-0150 §7.3 (В12, #3767): снимок LLM получает фразы бэклога с метками
        # «кто сказал» структурой — их вставит _build_dynamic_system_context.
        self._node._pending_unaddressed = recent
        self._node._last_heard = heard  # азимут фразы — собеседнику хода (SpeakerRef.azimuth_deg)
        take_reply(self._node, decision.source, heard)  # окно после приветствия снимает первая адресованная фраза (#3776)
        take_answer(self._node, decision.source, heard)  # кадр Identity снимает первая адресованная фраза (#3695)
        # PR-3b: рефлекс addressed (лицо «слушает», earcon, кольцо) - до LLM; ход этой фразы уже мог открыть ранний сигнал.
        self._node._turn_stages().addressed(self.utterance_id, heard.source or "respeaker")
        doa = "-" if heard.doa_deg is None else f"{heard.doa_deg:.0f}"
        self._node.get_logger().info(
            f"📮 [address] Addressed source={decision.source} doa={doa} person={heard.person_ref or '-'} "
            f"faces={len(heard.faces)} recent_unaddressed={phrase_count(recent)}"
        )
        if decision.pickup:
            self._node.get_logger().info(
                f"🗒️ [backlog] pickup {pickup_summary(decision.picked)} source=backlog_pickup "
                f"text={decision.text[:80]!r} — голое «робот», текст хода взят из бэклога кодом"
            )

    def identity_question(self) -> Optional[Any]:
        return question(self._node)

    def greeting_window(self) -> Optional[Any]:
        return window(self._node)

    def perform_ask(self) -> Optional[Any]:
        return perform_ask.window(self._node)

    def bare_wake(self) -> None:
        # В12: «робот» без фразы и без свежей просьбы того же направления —
        # рефлекс «слушаю» (LISTENING; лицо и earcon - рефлекс addressed в addressed() выше), 0 LLM.
        # Кадр ожидания ADDRESS_WAIT_S снимается молча, если хода не было.
        node = self._node
        node.get_logger().info(
            f"🙉 [address] голое «робот» без свежей фразы — рефлекс addressed, 0 LLM, ожидание {ADDRESS_WAIT_S:.0f} с"
        )
        if self.transition_idle_to_wake():
            loop = getattr(node, "_loop", None)
            if loop is not None:
                loop.call_soon_threadsafe(loop.call_later, ADDRESS_WAIT_S, self._end_wait)

    def _end_wait(self) -> None:
        node = self._node
        if node._dsm.current_state == DialogueStateKind.LISTENING:
            node._dsm.reset(DialogueStateKind.IDLE)
            node._publish_state()

    def _tier1(self) -> Tier1Runner:
        return runner_of(self._node)

    def tier1_match(self, ctx: "SttContext") -> Optional[Tier1Command]:
        return self._tier1().match(ctx)

    def run_tier1(self, command: Tier1Command, text: str) -> None:
        self._tier1().run(command, text)

    def dispute_stop(self, command: Tier1Command, ctx: "SttContext") -> None:
        self._tier1().dispute(command, ctx, self.tier1_resume)

    def is_music_stop_command(self, text_lower: str) -> bool:
        # Issue #1279 — «хватит диджеить» is music-stop, not silence.
        return is_music_stop_command(text_lower)

    def stop_music_quietly(self) -> bool:
        return self._tier1().stop_music()

    def handle_command_intent(self, text: str, text_lower: str) -> bool:
        node = self._node
        if not getattr(node, "_command_intent_gate_enabled", False):
            return False
        # 🔴 FIX (issue #2971): раньше сверялись только с
        # списком фиксированных стоп-фраз узла (голые фразы) —
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
        # ADR-0150 PR-6 (В19): «стоп/хватит/стой» решает Tier1Step — ясный стоп уже исполнен там, а сюда доходит
        # только стоп-слово внутри другой команды, которое Chooser счёл её частью («покажи анимацию стоп»).
        if command.intent == IntentType.STOP and stop_verdict(text) != STOP_NONE:
            return False
        node._llm_skipped_counter["command_intent"] += 1
        node._cancel_run("command intent (issue 1279)", stop_tts=True)
        # ADR-0150 PR-5 (#3694) — исполнитель получает решённое намерение по
        # /dialog/intent; сам реплики он больше не читает.
        gate = getattr(node, "_session_epoch_gate", None)
        msg = fill_intent(IntentMsg(), command, epoch=gate().current if gate else 0)
        node._intent_pub.publish(msg)
        node.get_logger().info(
            f"🎯 [issue 1279] command intent="
            f"{command.intent.value} conf={command.confidence:.2f} "
            f"→ /dialog/intent kind={msg.kind} epoch={msg.epoch} "
            f"(LLM dispatch skipped): {text[:60]!r}"
        )
        return True

    def handle_media_command(self, text: str) -> bool:
        # Issue #3134 — медиакоманды кодом, до LLM (и в TG, и в DJ-режиме).
        # Issue #3227 — попутно: музыкальный материал в реплике принимается
        # кодом (реплика при этом идёт дальше — LLM ответит человеку).
        getattr(self._node, "_material_intake", _NO_INTAKE).on_text(text)
        return self._node._route_media_command(text, on_miss=self.media_miss)

    def take_turn(self) -> None:
        # Issue #2939 — за отменой всегда идёт DispatchTriggerStep: сессию
        # принимает новый ход. ADR-0150 §6.2 (PR-5) — речь не трогаем: рвёт
        # её только ясная стоп-команда (/dialog/interrupt_request).
        self._node._cancel_run("new STT input", stop_tts=False, hand_over=True)

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
