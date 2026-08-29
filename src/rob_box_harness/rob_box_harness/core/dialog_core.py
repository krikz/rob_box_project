"""``DialogCore`` — high-level facade over the four dialogue ports.

Composes:

* :class:`rob_box_llm.LLMProvider`       — LLM client (``complete`` / ``stream``)
* :class:`rob_box_harness.tools.ToolProvider` — tool dispatch
* :class:`rob_box_harness.memory.MemoryStore` — conversation + facts persistence
* :class:`DialogueStateMachine`          — pure lifecycle state machine

into a single async ``process_input`` entry point so the dialogue
shell (a thin ROS2 node) can stay ≤350 LOC.

The class itself is pure Python — no ROS2, no ``rclpy``. The shell
calls ``process_input`` from its STT callback and publishes the
returned :class:`DialogResult` to ``/voice/dialogue/response``.

Per ``.planning/06-01-PLAN.md`` §W3, the class composes the ports
without absorbing their responsibilities. Tool dispatch and state
transitions still go through the existing port APIs — DialogCore is
the *orchestrator*, not a duplicate of the ports.
"""

from __future__ import annotations

import asyncio
import json
import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Iterable, Mapping

from rob_box_harness.core.confirmation_policy import ConfirmationKind
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
    DialogueStateMachine,
)
from rob_box_harness.memory import MemoryStore
from rob_box_harness.tools import ToolProvider, ToolSpec
from rob_box_llm.provider import LLMMessage, LLMProvider, LLMResponse, ToolCall, ToolResult

if TYPE_CHECKING:
    # Forward import — AcceptanceGate is in rob_box_harness.core.acceptance
    # and importing it eagerly here would create a circular dependency
    # with the package's __init__ exports.
    from rob_box_harness.core.acceptance import AcceptanceGate


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------


#: Hard cap on tool-loop iterations inside ``process_input``. Mirrors the
#: legacy OpenAI Agents SDK ``max_turns`` ceiling so a runaway tool loop
#: can't pin the dialogue thread forever. Five is enough for memory_context
#: + a follow-up explanation, and short enough that a misbehaving tool
#: fails loudly rather than running away.
_MAX_TOOL_ITERATIONS: int = 8

# W7a (issue #968, INSIGHT #1): a single LLM response may carry several
# tool_calls in one batch (e.g. ``speak_text`` + ``stop_music``). Executing
# them in the model's raw order fires destructive tools before the voice
# pipeline has even started — ``stop_music`` used to land on the robot
# milliseconds after ``speak_text`` was requested, killing the music mid-phrase
# (e2e v36). The scheduler (W7b) owns cross-batch ordering; this module-level
# re-ordering fixes the *intra-batch* race:
#
# * music tools (``execute_music_code`` / ``set_vibe_preset`` / ``load_track``)
#   run FIRST so the prelude starts before speech,
# * ordinary tools keep their relative order,
# * destructive tools (``stop_music`` / ``stop_navigation``) run LAST and are
#   flagged ``defer_until_voice_drained`` when the same batch also speaks —
#   the flag tells the scheduler not to fire the side effect until the voice
#   channel is empty.
_MUSIC_PRELUDE_TOOLS: frozenset[str] = frozenset(
    {"execute_music_code", "set_vibe_preset", "load_track"}
)
_VOICE_TOOLS: frozenset[str] = frozenset({"speak_text"})
_DEFER_TO_END_TOOLS: frozenset[str] = frozenset(
    {"stop_music", "stop_navigation"}
)


def _order_tool_calls(
    calls: Iterable[ToolCall],
) -> tuple[list[ToolCall], set[str]]:
    """Re-order one LLM batch for safe execution.

    Returns ``(ordered, deferred_call_ids)``:

    * ``ordered`` — stable partition: music prelude tools first, then the
      remaining tools in their original relative order, then destructive
      tools last.
    * ``deferred_call_ids`` — ids of destructive calls whose side effect
      must wait until the voice channel drains (only set when the batch
      also contains a voice tool such as ``speak_text``).

    The re-ordering changes execution order ONLY. Tool results are still
    returned to the LLM in the model's original order (see
    :meth:`DialogCore._run_with_tools`), so the OpenAI-style conversation
    history stays valid.
    """
    ordered = list(calls)

    def _partition_key(call: ToolCall) -> int:
        if call.name in _DEFER_TO_END_TOOLS:
            return 2
        if call.name in _MUSIC_PRELUDE_TOOLS:
            return 0
        return 1

    ordered.sort(key=_partition_key)
    has_voice = any(call.name in _VOICE_TOOLS for call in ordered)
    deferred_call_ids = {
        call.id
        for call in ordered
        if has_voice and call.name in _DEFER_TO_END_TOOLS
    }
    return ordered, deferred_call_ids


#: Completion markers the master prompt teaches the LLM to return AFTER the
#: last ``speak_text`` call. They are ONLY valid when the turn actually
#: performed work (tool calls happened in an earlier iteration). When the
#: model emits a bare marker as its FIRST response with zero tool calls it
#: is a silent failure — the user hears nothing and nothing happened
#: (issue #1217, deepseek-v4-flash intermittently does this).
_SILENT_DONE_MARKERS: frozenset[str] = frozenset(
    {"done", "task complete", "task_complete", "готово", "всё", "выполнено"}
)

#: ``finish_reason`` values that mean "the model produced NO usable output"
#: even though the HTTP call succeeded. DeepSeek documents
#: ``insufficient_system_resource`` (HTTP 200, generation interrupted by
#: provider resource pressure) and ``content_filter`` (content omitted by
#: filters); ``length`` with empty content means max_tokens was exhausted
#: before any token was emitted (issue #1253). All three are retryable —
#: the corrective retry in ``_run_with_tools`` gives the model a second
#: chance instead of shipping silence to the user.
_SILENT_FINISH_REASONS: frozenset[str] = frozenset(
    {"insufficient_system_resource", "content_filter", "length"}
)


# Issue #1708 — hallucinated-lyrics guard. After ``execute_music_code``
# (Renardo instrumental), the LLM sometimes also calls ``speak_text`` with
# invented lyrics («Нига-стайл, Колобок-флоу...»). The user hears a
# robotic voice reading hallucinated text on top of (or instead of) the
# beat. Master prompt only authorises ONE short accept phrase after
# ``execute_music_code`` («Ок, играю Бах»), not full lyrics. We drop
# ``speak_text`` calls that follow a music tool AND exceed the accept-
# phrase threshold AND the user request was NOT a vocal one (rap / poem /
# song — there backing mode legitimately calls speak_text × N).
#
# ⚠️ НЕ сводить с ``MUSIC_GUARD_VOCAL_KEYWORDS`` из
# :mod:`rob_box_voice.core.dialogue_guards`. Имена похожи, вопросы разные:
#
# * здесь — «просил ли пользователь голос ВООБЩЕ?». На «да» гард
#   галлюцинированных текстов не глушит ``speak_text``, потому что
#   бэкинг-режим законно зовёт его несколько раз;
# * там — «просил ли пользователь голос БЕЗ бита?». Список у́же
#   намеренно: для речитатива (рэп / зачитай / частушка) бит обязателен,
#   и music-guard Bug C (issue #992) должен нуднуть модель, если она не
#   вызвала ``execute_music_code``.
#
# Слить их — значит молча снять требование бита с рэпа. Инвариант
# «voice — строгое подмножество harness» закреплён тестом
# ``test_harness_vocal_keywords_are_a_strict_superset``
# (src/rob_box_voice/test/unit/core/test_dialogue_guards.py).
#
# Раньше здесь стояло «Mirrors rob_box_voice.core.dialogue_guards
# heuristic», что читалось как «списки обязаны совпадать» — неверно.
_VOCAL_REQUEST_KEYWORDS: tuple = (
    "спой", "пой ", "песня", "песню", "рэп", "реп", "rap",
    "зачитай", "зачита", "зачитывай", "стих", "стишок", "стихотворен",
    "прочитай", "прочти", "куплет", "частушк",
)
# Maximum length (chars) of a legitimate post-music accept phrase
# («Ок, играю Бах.», «Играю Бах», «Поехали»). Anything longer than
# this on a non-vocal request is a hallucination. The number matches the
# existing TRACK-mode accept test in test_issue_992_batch_cleanup.
_ACCEPT_PHRASE_MAX_CHARS: int = 40
#: Broader set than ``_MUSIC_PRELUDE_TOOLS`` — adds ``generate_music``
#: (MiniMax Music API) and ``gen_play_from_library``. The prelude set
#: only governs execution ORDER (music tools must run first), but the
#: hallucinated-lyrics guard must fire for ANY music tool that leaves
#: the user hearing lyrics on top of audio (issue #1708 / #1561).
_MUSIC_LAUNCH_TOOLS: frozenset[str] = frozenset({
    "execute_music_code", "set_vibe_preset", "load_track",
    "generate_music", "gen_play_from_library",
})


# ---------------------------------------------------------------------------
# Result type
# ---------------------------------------------------------------------------


@dataclass
class DialogResult:
    """Outcome of a single :meth:`DialogCore.process_input` call.

    The shell consumes these fields to decide what to publish on
    ``/voice/dialogue/response`` / ``/voice/dialogue/state``.

    Fields
    ------
    spoken_text:
        The assistant's reply (already through the LLM). Empty string
        when the LLM errored before producing output.
    new_state:
        The DSM state after processing (typically ``IDLE`` for a
        successful turn).
    tools_called:
        Names of tools the LLM invoked during this turn — populated
        by the in-core tool loop (see :meth:`DialogCore.process_input`
        for the full contract). Empty when the model answered in
        plain text or the tool loop did not run.
    error:
        ``None`` on success, otherwise the exception that aborted the
        turn — the original object, with its type intact. The shell
        logs this but does not raise it further.
    error_traceback:
        Formatted traceback of ``error``, or ``None``. Kept beside the
        exception rather than folded into it: ``result.error`` used to
        be ``Exception(f"{exc}\n{traceback}")``, which made the text
        available but threw the *type* away, and the shell had to
        recover "is this a provider outage?" by substring-matching the
        message (``dialogue_node._is_llm_unavailable_error``). The type
        is the contract; the traceback is diagnostics. Both, separately.
    """

    spoken_text: str = ""
    new_state: DialogueStateKind = DialogueStateKind.IDLE
    tools_called: list[str] = field(default_factory=list)
    # Issue #992 (TWO MUSIC MODES): how many times the LLM invoked
    # ``speak_text`` during this turn. ``tools_called`` only keeps
    # *unique* names, but the dialogue_node music-cleanup gate needs
    # the raw count to distinguish BACKING (спой/рэп — lyrics via 2+
    # speak_text calls, music must stop at tts_batch_complete) from
    # TRACK (сыграй баха — at most one short accept phrase, music
    # lives until the user stops it).
    speak_text_count: int = 0
    # Issue #1343 (silence after accept): how many ``speak_text`` tool
    # calls carried a REAL (non-empty) ``text`` argument this turn.
    # ``tools_called`` is populated from the LLM's requested tool-call
    # names BEFORE execution, so a phantom ``speak_text({})`` /
    # ``speak_text({"text": ""})`` from deepseek lands in
    # ``tools_called`` but never voices anything (validation rejects
    # empty text before the MCP request is sent). The issue-988
    # anti-duplicate guard in dialogue_node must only skip auto-TTS
    # when speak_text was REALLY called with content — otherwise the
    # user hears the accept sound and then silence.
    speak_text_real_count: int = 0
    error: BaseException | None = None
    error_traceback: str | None = None
    # ── LLM diagnostics (live 16:58) ────────────────────────────────────
    # When the LLM returns empty content (MiniMax M3 Interleaved Thinking
    # compaction, timeouts, finish_reason='length'), we want to know WHY
    # in the debug log. Populated by DialogCore from the last LLMResponse
    # and consumed by dialogue_node when spoken_text is empty.
    finish_reason: str | None = None
    raw_response: Any | None = None


@dataclass(frozen=True)
class _ToolLoopOutcome:
    """Что вернул тул-цикл :meth:`DialogCore._run_with_tools`.

    Раньше это был безымянный кортеж, который вызывающая сторона
    распаковывала одной строкой в 118 символов. Аннотация при этом
    обещала шесть элементов, а ``return`` отдавал семь — разъехались
    молча, потому что распаковка длину не проверяет.

    Fields
    ------
    spoken_text:
        Финальный текст модели. Пустая строка, когда ход подавлен
        (babble-фильтр issue #1253).
    tools_called:
        Уникальные имена вызванных тулов, в порядке первого вызова.
    finish_reason:
        ``finish_reason`` последнего ответа — нужен ноде, чтобы отличить
        пустой ответ от обрыва по ``length``.
    raw_response:
        Сырой ответ провайдера, для логов.
    speak_text_count:
        Сколько раз модель ЗВАЛА ``speak_text`` (issue #992: отличает
        BACKING-ход от TRACK-хода).
    speak_text_real_count:
        Сколько из них несли непустой ``text`` и реально бы прозвучали
        (issue #1343 — deepseek шлёт ``speak_text({})``).
    spoken_via_tool:
        Что реально произнесено через ``speak_text``, склеенное через
        перевод строки. Пишется в историю вместо маркера «done», иначе
        модель начинает отвечать «done» сама.
    """

    spoken_text: str
    tools_called: list[str]
    finish_reason: str | None
    raw_response: Any
    speak_text_count: int
    speak_text_real_count: int
    spoken_via_tool: str


# ---------------------------------------------------------------------------
# DialogCore
# ---------------------------------------------------------------------------


class DialogCore:
    """Orchestrator for a single conversation turn.

    The shell drives the loop:

    1. STT callback receives text → ``process_input(text, history)``
    2. Core persists the user turn, calls the LLM, persists the
       assistant turn, and returns a :class:`DialogResult`.
    3. Shell publishes the result.

    The core is intentionally stateless across turns — every turn is a
    self-contained ``process_input`` call. The DSM keeps the lifecycle
    state, the MemoryStore keeps the conversation, the LLM keeps no
    state at all (the shell passes the history explicitly).
    """

    def __init__(
        self,
        *,
        llm: LLMProvider,
        tools: ToolProvider,
        memory: MemoryStore,
        dsm: DialogueStateMachine,
        user_id: str = "default",
        history_trim_limit: int | None = None,
        inactivity_timeout: float | None = None,
        acceptance_gate: "AcceptanceGate | None" = None,
        system_prompt: str | None = None,
        use_streaming: bool = False,
    ) -> None:
        """Compose the four dialogue ports into a single facade.

        Args:
            llm: LLM client — required.
            tools: Tool dispatch port — required. DialogCore uses
                ``tools.discover()`` to fetch the OpenAI-style schema
                for ``llm.complete(messages, tools=...)`` and runs
                the tool loop in-process when the LLM asks for a
                tool call. Required even when the LLM never invokes
                a tool, because the loop is the only way the model
                can discover ``memory_context`` and friends.
            memory: Conversation + facts store — required.
            dsm: Dialogue state machine — required.
            user_id: Scope key used when calling the memory store.
                Defaults to ``"default"``.
            history_trim_limit: When set and the caller passes
                ``history=None`` to :meth:`process_input`, ask
                ``memory.load_recent(user_id, limit=history_trim_limit)``
                instead of running with an empty history. Lets the
                shell trim history purely through the memory port
                without re-implementing slicing.
            inactivity_timeout: When set, ``check_timeout()`` drops
                out of ``LISTENING`` after this many seconds of
                silence (forwarded to
                :meth:`DialogueStateMachine.check_inactivity_timeout`).
                ``None`` disables the inactivity check; the shell is
                then expected to drive ``TIMEOUT`` events manually.
            acceptance_gate: Optional :class:`AcceptanceGate` (issue
                #968 §8 / §11.2). When provided, every tool call
                emitted by the LLM is first classified: ``require``
                segments enter ``AWAITING_CONFIRMATION`` and the
                LLM receives an ``"awaiting_user_confirmation"``
                tool result; ``pass_through`` / ``notify`` segments
                are executed as before. When ``None`` the core runs
                the legacy un-gated path — backward-compatible with
                every existing test that doesn't construct a gate.
        """
        if llm is None:
            raise TypeError("DialogCore: llm is required")
        if tools is None:
            raise TypeError("DialogCore: tools is required")
        if memory is None:
            raise TypeError("DialogCore: memory is required")
        if dsm is None:
            raise TypeError("DialogCore: dsm is required")
        self._llm = llm
        self._tools = tools
        self._memory = memory
        self._dsm = dsm
        self._user_id = user_id
        self._system_prompt = system_prompt
        self._history_trim_limit = history_trim_limit
        # 🔴 FIX (live 06.08): стриминг управляется конфигом (dialogue_node.yaml
        # → llm_streaming). Дефолт False — консервативно, без стриминга.
        self._use_streaming = use_streaming
        self._inactivity_timeout = inactivity_timeout
        self._acceptance_gate = acceptance_gate

    # ---- main entry point -----------------------------------------------

    async def process_input(
        self,
        text: str,
        *,
        history: Iterable[LLMMessage] | None = None,
        is_dj_auto: bool = False,
        is_synthetic: bool = False,
        speaker_tag: str | None = None,
        speaker_context: str | None = None,
        dynamic_system: str | None = None,
        preclassified_event: DialogueEvent | None = None,
    ) -> DialogResult:
        """Process a single user turn.

        ``dynamic_system`` (live 10.08, two-system-prompt pattern) — XML
        ``<system_context>...</system_context>`` snapshot собирается
        dialogue_node каждый turn (текущий спикер, TTS-voice, session lock).
        Кладётся system-сообщением ПОСЛЕДНИМ перед user input — см.
        комментарий на месте вставки. Если None — не добавляется.

        ``is_synthetic`` — вход сгенерирован нами, а не человеком
        (``[CRITICAL]``-ретраи babble/music guard'ов). Такой turn НЕ
        пишется в историю как реплика пользователя: ответ модели —
        пишется, потому что его слышал человек, а сам «промпт» человек
        никогда не произносил. Тот же приём, что ``is_dj_auto`` — см.
        комментарий у append_turn ниже.

        ``preclassified_event`` (live 10.08, issue #1101) — если caller
        уже классифицировал вход (через ``dsm.on_user_input`` + DSM-переход)
        и знает финальный event, он передаёт его сюда чтобы не повторять
        классификацию. Иначе ``on_user_input(text)`` матчит wake-words
        внутри user-text (напр. «робот» в середине фразы) → возвращает
        ``WAKE_WORD`` → guard ``event == STT_RESULT`` ломается → LLM не
        вызывается → «акцепт есть, робот не отвечает».

        Без этого параметра (None) DialogCore сам вызывает
        ``dsm.on_user_input(text)`` и ``dsm.on_event(event)`` — backward
        compatible для тестов / DJ-auto / harness-вызовов.

        Returns a :class:`DialogResult` describing the assistant's
        reply, the new DSM state, and any error. Never raises — LLM
        exceptions are wrapped in ``result.error`` so the shell can
        log them without aborting the conversation loop.

        ``is_dj_auto`` (issue #992) — when True the input is treated
        as an autonomous DJ auto-transition: the wake-word classifier
        is bypassed and the DSM is force-transitioned to DIALOGUE so
        the LLM actually runs. Without this the DJ prompt
        ``"[DJ_AUTO] ... Роббокс ..."`` matches the "роббокс" wake
        trigger, ``on_user_input`` returns ``WAKE_WORD`` (instead of
        ``STT_RESULT``), the DSM transition is a no-op on LISTENING,
        and ``process_input`` silently skips the LLM call — which is
        exactly the production symptom in issue #992 ("DJ cycle never
        produces music, robot says 'задумался'").

        History trimming: if the caller passes ``history=None`` AND
        ``history_trim_limit`` was set at construction, the memory
        port's ``load_recent`` is queried for the trimmed window.
        The shell therefore never has to slice the history itself —
        it just decides whether to pass an explicit list or trust
        the memory port.

        Steps:
        1. Classify the input via ``dsm.on_user_input``.
        2. Drive the DSM via ``dsm.on_event``.
        3. If we're in ``DIALOGUE`` state (i.e. the input was real
           speech), persist the user turn, call the LLM **with the
           tool schemas discovered from** ``self._tools``, persist
           the assistant turn, and run any follow-up tool loop the
           LLM triggers until either the model returns a plain-text
           reply or ``_MAX_TOOL_ITERATIONS`` is reached.
        4. Return the result.

        Tool-loop contract
        ------------------

        We pass ``tools.discover()`` (translated to the OpenAI
        Chat-Completions ``{"type": "function", ...}`` shape) on
        every ``complete()`` call. When the LLM responds with
        ``LLMResponse.tool_calls``:

        * Each :class:`ToolCall` is executed via
          ``self._tools.execute(call)``.
        * The :class:`ToolResult` (success or function-level error)
          is fed back as an ``LLMMessage(role="tool", ...)`` and
          ``complete()`` is re-issued.
        * ``result.tools_called`` accumulates the *unique* names of
          tools that actually ran.

        The loop is capped at ``_MAX_TOOL_ITERATIONS`` to keep a
        runaway tool from pinning the dialogue thread. Tool
        transport failures (raised :class:`ToolExecutionError`) are
        wrapped into ``result.error`` the same way LLM errors are.
        """
        result = DialogResult()

        # 1. classify — DJ auto-transitions bypass the wake-word
        #    classifier because their prompt intentionally mentions
        #    "роббокс" / "диджей" (the persona), which would otherwise
        #    short-circuit into a WAKE_WORD event and skip the LLM
        #    call entirely (issue #992 root cause).
        # 🔴 FIX (issue #1101): если caller уже классифицировал и
        #    выполнил DSM-переход (dialogue_node делает это в _on_stt),
        #    пропускаем двойную классификацию. Иначе wake-word внутри
        #    текста ('робот' в середине фразы) вернёт WAKE_WORD и
        #    guard 'event == STT_RESULT' пропустит LLM → тишина.
        if preclassified_event is not None:
            event = preclassified_event
        elif is_dj_auto:
            event = DialogueEvent.STT_RESULT
        else:
            event = self._dsm.on_user_input(text)

        # 2. transition (skip if caller already transitioned)
        if preclassified_event is not None:
            # Caller already drove the DSM (dialogue_node._on_stt).
            # Trust its current_state — typically DIALOGUE.
            #
            # 🔴 FIX (issue #1217, DJ-auto regression from #1101):
            # DJ auto-turns do NOT come through ``_on_stt`` — they are
            # dispatched straight from ``DJModeController.tick`` via
            # ``_run_turn``, which passes ``preclassified_event=STT_RESULT``.
            # The else-branch below (with the DJ force-transition) is
            # therefore skipped, so when the previous turn left the DSM
            # in IDLE (DIALOGUE_END) the LLM gate
            # ``current_state == DIALOGUE`` silently drops the turn —
            # the LLM is never called and the robot answers nothing
            # (observed in e2e run4: DJ-auto turns returned in ~2 ms
            # with ``spoken='' tools=[] finish_reason=None`` and zero
            # ``[health] stream`` log lines). Force the same transition
            # the else-branch would have done.
            if is_dj_auto:
                if self._dsm.current_state == DialogueStateKind.IDLE:
                    self._dsm.on_event(DialogueEvent.WAKE_WORD)
                self._dsm.on_event(DialogueEvent.STT_RESULT)
        else:
            self._dsm.on_event(event)
            # DJ auto-turns: if DSM is LISTENING (no wake word yet) but the
            # shell already drove the cycle, force-transition into DIALOGUE
            # so the LLM gate fires. ``STT_RESULT`` from LISTENING
            # normally does this — but a previous turn may have left the
            # DSM in IDLE (e.g. silence timeout between transitions).
            if is_dj_auto and self._dsm.current_state == DialogueStateKind.IDLE:
                self._dsm.on_event(DialogueEvent.WAKE_WORD)
                self._dsm.on_event(DialogueEvent.STT_RESULT)
        result.new_state = self._dsm.current_state

        # 3. if we're now in DIALOGUE state, run the LLM
        if (
            self._dsm.current_state == DialogueStateKind.DIALOGUE
            and event == DialogueEvent.STT_RESULT
        ):
            from rob_box_harness.memory import Turn
            try:
                # Resolve the trimmed history BEFORE we append the
                # new turn — otherwise ``load_recent`` would echo
                # the just-stored user message back into the prompt.
                messages = await self._resolve_history(history)
                # Issue #1077 — контекст о спикере (профиль + факты из
                # scope=speaker:<tag>). Вставляем system-сообщением сразу
                # после основного системного промпта, чтобы LLM знала,
                # с кем разговаривает (имя, предпочтения, история).
                if speaker_context:
                    if messages and messages[0].role == "system":
                        messages.insert(
                            1,
                            LLMMessage(role="system", content=speaker_context),
                        )
                    else:
                        messages.insert(
                            0,
                            LLMMessage(role="system", content=speaker_context),
                        )
                # Two-system-prompt pattern (live 10.08) — dynamic
                # <system_context> snapshot: текущий спикер (resemblyzer),
                # TTS-voice (gender alignment), session lock state.
                #
                # Кладётся ПОСЛЕДНИМ system-сообщением, вплотную к текущей
                # реплике. Раньше он вставлялся в messages[1], то есть
                # ПЕРЕД двадцатью ходами истории: модель читала «вот что
                # происходит сейчас», а следом — два десятка ходов
                # прошлого разговора, и никакого признака, что снапшот
                # свежее всей этой истории, у неё не было. Волатильный
                # runtime-стейт должен стоять там, где он и по времени —
                # рядом с последним user-ходом.
                if dynamic_system:
                    messages.append(
                        LLMMessage(role="system", content=dynamic_system)
                    )
                messages.append(LLMMessage(role="user", content=text))
                # 🔴 FIX (live 11:19 DJ): DJ-переходы (is_dj_auto=True) НЕ
                # пишутся в долгую память — иначе каждый переход (#1..#N)
                # копит user+assistant пары в SQLite, history_max_turns
                # (20 пар) заполняется DJ-промптами, LLM тонет в 20+
                # одинаковых «[DJ_AUTO переход #N]» + [CRITICAL] ретраях
                # → пустые ответы → «Что-то я задумался». Системный
                # DJ-триггер — не реплика юзера, он не должен загрязнять
                # контекст диалога.
                user_metadata = {}
                if speaker_tag is not None:
                    user_metadata["speaker_tag"] = speaker_tag
                #
                # То же самое, слово в слово, верно для [CRITICAL]-ретраев
                # babble/music guard'ов (``is_synthetic``). Живой лог с
                # vision 29.08: из 20 ходов окна два — user-реплики вида
                # «[CRITICAL] В прошлом цикле ты НЕ вызвал ни один
                # музыкальный тул», которых человек не произносил. Модель
                # на каждом следующем ходу перечитывала транскрипт, где её
                # отчитывают, и отвечала «Менеджер не отвечает» вместо
                # вызова тула. Ответ на ретрай пишется как обычно — его
                # человек слышал.
                if not is_dj_auto and not is_synthetic:
                    await self._memory.append_turn(
                        self._user_id,
                        Turn(
                            role="user",
                            content=text,
                            metadata=user_metadata,
                        ),
                    )
                outcome = await self._run_with_tools(messages)
                result.spoken_text = outcome.spoken_text
                result.tools_called = list(outcome.tools_called)
                result.speak_text_count = outcome.speak_text_count
                result.speak_text_real_count = outcome.speak_text_real_count
                result.finish_reason = outcome.finish_reason
                result.raw_response = outcome.raw_response
                if not is_dj_auto:
                    # Persist an HONEST assistant turn: the text actually
                    # spoken via speak_text (or a real plain-text reply), NOT
                    # the bare "done"/"" completion marker. A history full of
                    # "done" misleads the model into (a) echoing old topics
                    # and (b) replying "done" itself (silent failures). Silent
                    # turns are dropped entirely — they add no useful context.
                    assistant_content = outcome.spoken_via_tool or outcome.spoken_text
                    if not self._is_silent_spoken(assistant_content, ()):
                        await self._memory.append_turn(
                            self._user_id,
                            Turn(
                                role="assistant",
                                content=assistant_content,
                                metadata=dict(user_metadata),
                            ),
                        )
            except Exception as exc:  # noqa: BLE001 — carry it in the result
                import traceback as _tb
                result.error = exc
                result.error_traceback = _tb.format_exc()
                # The user turn was already appended BEFORE the LLM call
                # (line above). On error, do NOT append again — that
                # would produce a duplicate row in the conversation
                # history. SQLiteVoiceMemory also has its own 5-second
                # dedup window as a safety net.
            # End-of-dialogue: drive the state machine back to IDLE.
            self._dsm.on_event(DialogueEvent.DIALOGUE_END)
            result.new_state = self._dsm.current_state

        return result

    @staticmethod
    def _is_silent_response(response: LLMResponse) -> bool:
        """Issue #1217/#1253 — did the LLM do nothing useful for the user?

        ``True`` when the response carries no tool calls AND its content is
        either empty or a bare completion marker (``done`` / ``готово`` /
        ...). The master prompt teaches those markers as the terminal reply
        AFTER ``speak_text``; when the model emits one as its first answer it
        means it skipped the actual work — the user hears nothing and nothing
        happened. Any substantive text is NOT silent even without tools
        (a plain-text answer is a valid response).

        Issue #1253 — additionally, a ``finish_reason`` that means
        "generation was interrupted / output omitted" (DeepSeek
        ``insufficient_system_resource``, ``content_filter``, ``length`` with
        empty content) is silent even when the aggregator carried a partial
        content: the user would hear a truncated fragment, so the corrective
        retry should fire instead.
        """
        if response.tool_calls:
            return False
        content = (response.content or "").strip().lower()
        if (not content) or content in _SILENT_DONE_MARKERS:
            return True
        # Issue #1253 — interrupted / filtered generation is not a valid
        # final answer even with a partial content fragment.
        return response.finish_reason in _SILENT_FINISH_REASONS

    @staticmethod
    def _is_silent_spoken(spoken: str, tools_called: Iterable[str]) -> bool:
        """Issue #1217 — is this turn's OUTCOME a silent failure?

        Mirrors :meth:`_is_silent_response` for the post-loop values: no
        tool ran AND the final text is empty or a bare completion marker.
        Such turns must not be persisted to conversation memory (they would
        teach the model that an empty marker is a normal assistant reply).
        """
        if tools_called:
            return False
        content = (spoken or "").strip().lower()
        return (not content) or content in _SILENT_DONE_MARKERS

    @staticmethod
    def _clean_history_turns(
        turns: Iterable[LLMMessage],
    ) -> list[LLMMessage]:
        """Collapse consecutive user turns and drop a trailing orphaned one.

        A barge-in (or a silent-failure turn whose assistant reply was not
        persisted) leaves an orphaned ``user`` turn with no answer in the
        stored history. Two consecutive user messages make the LLM answer the
        OLDER one instead of the current question. The current turn's user
        message is appended separately by ``process_input``, so a trailing
        user turn here is always an orphan and must be dropped.
        """
        out: list[LLMMessage] = []
        for turn in turns:
            if out and out[-1].role == "user" and turn.role == "user":
                out[-1] = turn
            else:
                out.append(turn)
        if out and out[-1].role == "user":
            out.pop()
        return out

    async def _run_with_tools(
        self,
        messages: list[LLMMessage],
    ) -> _ToolLoopOutcome:
        """Run the LLM tool loop and return a :class:`_ToolLoopOutcome`.

        ``messages`` is the live message list — tool-result messages
        are appended in-place so the LLM sees a coherent conversation
        history on every follow-up turn. The list is intentionally
        mutated rather than rebuilt: the upstream providers expect
        an ordered list and rebuilding from scratch would lose the
        interleaved tool/assistant ordering the wire format requires.

        The loop terminates when:

        * The model returns an empty ``tool_calls`` tuple (i.e. a
          plain-text reply) — that's the final answer.
        * ``_MAX_TOOL_ITERATIONS`` is reached — the last spoken
          text is used verbatim, and a warning is logged so
          operators can spot runaway loops in the wild.
        * The provider raises — the exception propagates to the
          ``except`` in :meth:`process_input`.

        Tool-level errors (i.e. the handler returned ``is_error=True``)
        are NOT loop-terminating: the LLM gets the error string as
        a ``tool`` message and can correct itself. Only a
        :class:`ToolExecutionError` (transport-level) aborts the
        turn because that's a wiring problem, not a tool problem.
        """
        tool_schemas = await self._tools.discover()
        openai_tools = [_tool_spec_to_openai(spec) for spec in tool_schemas]

        tools_called: list[str] = []
        seen: set[str] = set()
        speak_text_count: int = 0
        speak_text_real_count: int = 0
        # Actual text spoken via speak_text this turn — used for an honest
        # conversation history (persisting "done" instead of what was really
        # said made the LLM echo old topics; see process_input).
        spoken_texts: list[str] = []
        # Issue #1253 — any tool that returned ``is_error=True`` this turn.
        # When a tool failed and the LLM answers with ONLY words (no retry
        # tool-call, no speak_text) that is babble, not an answer — the
        # robot would voice «дан» / «бит не получился» instead of acting.
        tool_error_occurred: bool = False
        response: LLMResponse = await self._stream_response(
            messages, tools=openai_tools
        )

        # Issue #1217 — deepseek-v4-flash intermittently answers with a bare
        # completion marker ("done") or an EMPTY payload as its FIRST reply,
        # without any tool call. The user hears nothing and nothing happens.
        # Allow ONE corrective retry inside the same turn before accepting the
        # silent failure. The correction is appended to the live message list
        # (assistant echo when non-empty + a user-role instruction) so the
        # model sees exactly what went wrong; OpenAI-compatible providers
        # accept all the resulting shapes (verified against DeepSeek API).
        #
        # The retry fires ONLY when NO tool ran in this whole turn
        # (``not tools_called``): a final "done" AFTER speak_text is
        # legitimate per the master-prompt contract and must not be retried.
        _silent_retried = False

        for _ in range(_MAX_TOOL_ITERATIONS):
            if not response.tool_calls:
                if (
                    not _silent_retried
                    and not tools_called
                    and self._is_silent_response(response)
                ):
                    _silent_retried = True
                    if response.content:
                        messages.append(
                            LLMMessage(role="assistant", content=response.content)
                        )
                    messages.append(
                        LLMMessage(
                            role="user",
                            content=(
                                "[SYSTEM CORRECTION] Твой предыдущий ответ "
                                "был пустым: ни текста, ни tool-вызова. "
                                "Пользователь ничего не услышал, ничего не "
                                "произошло. ОБЯЗАТЕЛЬНО в ЭТОМ ответе вызови "
                                "нужный tool (speak_text — для речи) или дай "
                                "содержательный текстовый ответ."
                            ),
                        )
                    )
                    response = await self._stream_response(messages, tools=openai_tools)
                    continue
                break

            # Record unique tool names actually invoked, and count
            # speak_text occurrences (issue #992 — the raw count lets
            # dialogue_node tell BACKING sing/rap turns from TRACK
            # composition turns; unique names alone cannot).
            #
            # Issue #1343 — count REAL speak_text calls separately:
            # deepseek sometimes emits ``speak_text({})`` /
            # ``speak_text({"text": ""})`` with empty text. The name
            # still lands in ``tools_called`` (it was requested), but
            # the call is rejected by validation and NOTHING is voiced.
            # ``speak_text_real_count`` only counts calls that carry a
            # non-empty ``text`` argument — i.e. calls that would
            # actually reach the MCP tool and play audio. dialogue_node
            # uses this to skip auto-TTS only when speech REALLY
            # happened (issue #988 anti-duplicate), not when the LLM
            # merely *named* speak_text.
            for call in response.tool_calls:
                if call.name == "speak_text":
                    speak_text_count += 1
                    args = call.arguments or {}
                    text = args.get("text", "") if isinstance(args, Mapping) else ""
                    if isinstance(text, str) and text.strip():
                        speak_text_real_count += 1
                        spoken_texts.append(text.strip())
                if call.name not in seen:
                    seen.add(call.name)
                    tools_called.append(call.name)

            # Append the assistant turn that contained the tool_calls
            # (required by OpenAI Chat-Completions ordering rules).
            messages.append(
                LLMMessage(
                    role="assistant",
                    content=response.content,
                    tool_calls=response.tool_calls,
                )
            )

            # Execute every requested tool call and feed results back.
            # When an AcceptanceGate is wired in, require-classified calls
            # enter AWAITING_CONFIRMATION and the LLM receives a sentinel
            # tool result instead of an executor result — the LLM cycle
            # is NOT blocked (§4.4: MERGE/QUEUE/AWAITING don't cancel the
            # LLM cycle), and the user gets to confirm/reject before the
            # call ever reaches the executor.
            #
            # S2.3/S2.4 (scheduler-segments-merge, issue #968) — open a
            # fresh segment group for this batch BEFORE execution starts,
            # same wiring point as the W7a re-ordering below. Not every
            # ToolProvider is a SchedulerToolExecutor (e.g. plain
            # ToolProvider ports in tests), so the call is optional:
            # begin_group() only exists on the scheduler-backed wrapper.
            # Skipping it there is exactly today's behaviour
            # (group_id=None for every task).
            begin_group = getattr(self._tools, "begin_group", None)
            if begin_group is not None:
                begin_group()

            # W7a (issue #968): execution order is re-ordered so music
            # prelude tools run before voice, and destructive tools
            # (stop_music / stop_navigation) run last — otherwise a
            # ``[speak_text, stop_music]`` batch fires stop_music before
            # TTS even starts (e2e v36). Results are still appended in
            # the model's ORIGINAL order (keyed by tool_call_id) so the
            # OpenAI-style history stays valid.
            execution_order, _deferred = _order_tool_calls(
                response.tool_calls
            )
            # Issue #1708 — hallucinated-lyrics guard. Resolve the most
            # recent user message so the heuristic can tell a vocal
            # request («спой куплет» — backing mode) apart from a
            # non-vocal request («сыграй бит про колобка в нига стайле»
            # — instrumental). Walk the message list in reverse: the
            # LAST user-role entry is the current turn.
            _current_user_input: str = ""
            for _msg in reversed(messages):
                if _msg.role == "user" and _msg.content:
                    _current_user_input = str(_msg.content)
                    break
            # Music tools that appear ANYWHERE in this LLM batch (both
            # before and after the candidate speak_text). The order
            # inside the batch doesn't matter for the guard — the LLM
            # sometimes calls ``speak_text(title)`` BEFORE
            # ``execute_music_code`` (issue #1708 live shape) and
            # sometimes AFTER. Pre-compute the set from the model's
            # ORIGINAL tool_calls (before our re-ordering) so we cover
            # both shapes.
            same_batch_music_calls: set[str] = {
                call.name
                for call in response.tool_calls
                if call.name in _MUSIC_LAUNCH_TOOLS
            }
            results_by_call_id: dict[str, ToolResult] = {}
            for call in execution_order:
                if self._acceptance_gate is not None:
                    segment, decision = self._acceptance_gate.submit(
                        tool=call.name,
                        args=dict(call.arguments or {}),
                        call_id=call.id,
                    )
                    if decision.kind is ConfirmationKind.REQUIRE:
                        # Don't call the executor — gate will dispatch it
                        # later via the future scheduler (Фаза 2).
                        results_by_call_id[call.id] = ToolResult(
                            tool_call_id=call.id,
                            content=json.dumps(
                                {
                                    "status": "awaiting_user_confirmation",
                                    "segment_id": segment.segment_id,
                                    "tool": call.name,
                                    "plan_text": segment.decision.plan_text,
                                    "confirmation_timeout_ms": int(
                                        self._acceptance_gate.config.confirmation_timeout_ms
                                    ),
                                },
                                ensure_ascii=False,
                            ),
                            is_error=False,
                        )
                        continue

                # Issue #1708 — drop hallucinated lyrics after a music
                # tool. The check runs AFTER acceptance-gate but BEFORE
                # the real executor: a suppressed speak_text never
                # reaches the MCP tool, so the user hears no TTS for
                # it. The LLM still gets a sentinel result so it can
                # learn from the rejection on its next iteration.
                if _is_hallucinated_speak_text(
                    call=call,
                    same_batch_music_calls=frozenset(same_batch_music_calls),
                    user_input=_current_user_input,
                ):
                    logging.getLogger(__name__).warning(
                        "DialogCore [issue 1708]: suppressing "
                        "speak_text in batch with %s — hallucinated "
                        "lyrics would override music. text=%r user_input=%r",
                        sorted(same_batch_music_calls),
                        _extract_speak_text(call.arguments)[:80],
                        _current_user_input[:80],
                    )
                    results_by_call_id[call.id] = _suppressed_speak_text_result(call)
                    # Counter bookkeeping: the suppressed call must NOT
                    # count toward speak_text_count / speak_text_real_count
                    # (otherwise issue #988 anti-duplicate skips the
                    # final ``done`` text — silent user experience) and
                    # MUST NOT count toward ``spoken_texts`` (otherwise
                    # honest history records a phrase that was never
                    # voiced, biasing future turns).
                    if call.name == "speak_text":
                        speak_text_count -= 1
                        text = _extract_speak_text(call.arguments)
                        if text:
                            speak_text_real_count -= 1
                            # spoken_texts is appended above for every
                            # real speak_text — pop the last matching
                            # entry so the honest-history path stays
                            # honest. The list is small (<= N where N
                            # is the LLM batch size), linear scan is
                            # fine.
                            try:
                                spoken_texts.remove(text)
                            except ValueError:
                                pass
                    continue

                results_by_call_id[call.id] = await self._tools.execute(call)

            for call in response.tool_calls:
                tool_result = results_by_call_id[call.id]
                if tool_result.is_error:
                    tool_error_occurred = True
                messages.append(
                    LLMMessage(
                        role="tool",
                        content=tool_result.content,
                        tool_call_id=tool_result.tool_call_id,
                        tool_result=tool_result,
                    )
                )

            response = await self._stream_response(messages, tools=openai_tools)

        else:  # for-else: loop exhausted without breaking
            logging.getLogger(__name__).warning(
                "DialogCore: tool loop hit _MAX_TOOL_ITERATIONS=%d; "
                "returning the last spoken text as-is.",
                _MAX_TOOL_ITERATIONS,
            )

        # Issue #1253 — babble filter on tool error. A tool failed
        # (is_error=True) and the LLM answered with ONLY words: no retry
        # tool-call in this final response and no speak_text during the
        # turn. Voicing that text would make the robot say «дан» / «бит не
        # получился» while nothing actually happened. System transition:
        # return empty spoken so dialogue_node moves to the next round
        # instead of parroting the babble.
        if (
            tool_error_occurred
            and not response.tool_calls
            and "speak_text" not in seen
            and response.content
            and self._is_silent_response(response)
        ):
            logging.getLogger(__name__).warning(
                "DialogCore: tool error + babble-only final answer — "
                f"suppressing spoken text {response.content[:80]!r} "
                "(system transition)"
            )
            return _ToolLoopOutcome(
                spoken_text="",
                tools_called=tools_called,
                finish_reason=response.finish_reason,
                raw_response=response.raw,
                speak_text_count=speak_text_count,
                speak_text_real_count=speak_text_real_count,
                spoken_via_tool="\n".join(spoken_texts),
            )

        return _ToolLoopOutcome(
            spoken_text=response.content,
            tools_called=tools_called,
            finish_reason=response.finish_reason,
            raw_response=response.raw,
            speak_text_count=speak_text_count,
            speak_text_real_count=speak_text_real_count,
            spoken_via_tool="\n".join(spoken_texts),
        )

    async def _stream_response(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
    ) -> LLMResponse:
        """Streaming LLM completion aggregated into a full :class:`LLMResponse`.

        🔴 FIX (live 06.08): стриминг переключается конфигом (llm_streaming).
        False → полный complete() (как раньше, до стриминга); True → stream()
        с агрегацией tool-call deltas. Оба пути возвращают LLMResponse.
        """
        if not self._use_streaming:
            return await self._llm.complete(messages, tools=tools)
        parts: list[str] = []
        tool_calls: list[ToolCall] = []
        finish_reason: str | None = None
        raw: Any = None
        stream = self._llm.stream(messages, tools=tools)
        try:
            async for chunk in stream:
                # 🔴 FIX (issue #1280): barge-in — новый STT-инпут уже
                # отменил текущий run (task.cancel). Бросаем
                # CancelledError немедленно, ДО обработки чанка:
                # буферизованные чанки старой темы не должны
                # агрегироваться в ответ, который потом уйдёт в TTS
                # («робот добивает старую тему после смены»).
                # asyncio и так доставит CancelledError на ближайшем
                # await — здесь делаем это явно и раньше.
                task = asyncio.current_task()
                if task is not None and task.cancelled():
                    raise asyncio.CancelledError
                if chunk.content_delta:
                    parts.append(chunk.content_delta)
                if chunk.tool_call_delta is not None:
                    tool_calls.append(chunk.tool_call_delta)
                if chunk.finish_reason:
                    finish_reason = chunk.finish_reason
        finally:
            # 🔴 FIX (issue #1280): гарантированно закрываем stream при
            # ЛЮБОМ выходе — включая CancelledError при barge-in. Без
            # aclose() HTTP-запрос к провайдеру продолжает жить до конца
            # генерации: провайдер тратит квоту на старую тему, а ответ
            # старой темы может успеть попасть в TTS.
            aclose = getattr(stream, "aclose", None)
            if aclose is not None:
                try:
                    await aclose()
                except Exception:
                    pass
        return LLMResponse(
            content="".join(parts),
            tool_calls=tuple(tool_calls),
            finish_reason=finish_reason,
            usage=None,
            raw=raw,
        )

    async def _resolve_history(
        self,
        history: Iterable[LLMMessage] | None,
    ) -> list[LLMMessage]:
        """Build the LLM message list — either from ``history`` or.
        via ``memory.load_recent``.

        * When the caller passes an explicit iterable, that wins —
          the shell owns the trim there.
        * When ``history`` is ``None`` and ``history_trim_limit`` is
          set, the memory port provides the trimmed window.
        * When ``history`` is ``None`` and no trim limit is set, we
          return an empty list (the shell can fall back to whatever
          it has in mind).
        """
        if history is not None:
            return list(history)
        if self._history_trim_limit is None:
            out: list[LLMMessage] = []
            if self._system_prompt:
                out.append(LLMMessage(role="system", content=self._system_prompt))
            return out
        recent_turns = await self._memory.load_recent(
            self._user_id, limit=self._history_trim_limit
        )
        out: list[LLMMessage] = []
        # 🔴 FIX: system prompt обязателен первым сообщением — раньше
        # dialogue_node грузил _system_prompt, но никогда не передавал
        # его в LLM: deepseek работала БЕЗ системного промпта (все
        # правила «ALWAYS execute_music_code», NO METALANGUAGE и т.д.
        # не доходили до модели). Найдено через verbose-трейс 09:08:
        # в messages не было [0] system.
        if self._system_prompt:
            out.append(LLMMessage(role="system", content=self._system_prompt))
        history_messages = [
            LLMMessage(role=turn.role, content=turn.content)
            for turn in recent_turns
        ]
        out.extend(self._clean_history_turns(history_messages))
        return out

    # ---- handle_* shortcuts used by the shell --------------------------

    def is_wake_word(self, text: str) -> bool:
        """Return ``True`` if ``text`` looks like a wake-word utterance.

        Convenience for the shell's STT callback — the bool return
        matches the W3 plan §3 signature for ``handle_wake_word``
        even though the facade-level ``handle_wake_word`` below
        still returns a full :class:`DialogResult` for callers
        that want the new state too.
        """
        return self._dsm.on_user_input(text) == DialogueEvent.WAKE_WORD

    async def handle_silence(self) -> DialogResult:
        """Apply a ``SILENCE_COMMAND`` event to the DSM.

        The DSM now tracks both the silence deadline and the
        activity timestamp; this method just drives the
        ``SILENCE_COMMAND`` event and reports the resulting state.
        """
        self._dsm.on_event(DialogueEvent.SILENCE_COMMAND)
        return DialogResult(new_state=self._dsm.current_state)

    async def handle_wake_word(self, text: str) -> DialogResult:
        """Apply a ``WAKE_WORD`` event to the DSM.

        The text parameter is the raw STT transcript that triggered
        the wake word — useful for callers that want to log it.
        Returns a :class:`DialogResult` reporting the new state.

        Callers that only need the bool from the W3 plan §3
        signature should use :meth:`is_wake_word` instead.
        """
        del text  # currently unused; reserved for logging
        self._dsm.on_event(DialogueEvent.WAKE_WORD)
        return DialogResult(new_state=self._dsm.current_state)

    def check_timeout(self) -> bool:
        """Drive a ``TIMEOUT`` event into the DSM if appropriate.

        Two paths:

        * If ``inactivity_timeout`` was set at construction,
          forward to :meth:`DialogueStateMachine.check_inactivity_timeout`
          so ``LISTENING → IDLE`` after the configured silence.
        * Otherwise, fall back to the legacy ``TIMEOUT`` event
          — keeps the original behaviour for callers that don't
          opt in to the inactivity-tracking API.

        Returns:
            ``True`` if the state changed (i.e. a timeout fired).
        """
        if self._inactivity_timeout is not None:
            return self._dsm.check_inactivity_timeout(
                self._inactivity_timeout,
            )
        before = self._dsm.current_state
        self._dsm.on_event(DialogueEvent.TIMEOUT)
        return self._dsm.current_state != before


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------


def _is_vocal_request(user_input: str) -> bool:
    """Issue #1708 — does the user request contain a vocal cue?

    Mirrors ``rob_box_voice.core.dialogue_guards.is_vocal_request``
    without importing it (dialog_core lives in ``rob_box_harness`` and
    cannot depend on ``rob_box_voice``). Kept deliberately narrow: the
    hallucinated-lyrics guard must only fire on NON-vocal requests
    («сыграй бит», «включи трек»), never on legitimate rap/poem
    backing-mode turns («спой куплет»).

    ``user_input`` is matched case-insensitively as a substring scan —
    same algorithm as the upstream detector so the two stay in sync.
    """
    if not user_input:
        return False
    low = user_input.lower()
    return any(kw in low for kw in _VOCAL_REQUEST_KEYWORDS)


def _extract_speak_text(text: Any) -> str:
    """Issue #1708 — defensively pull the ``text`` arg out of a tool call.

    Tool arguments are nominally a ``Mapping`` per the OpenAI Chat-
    Completions contract, but live providers occasionally deliver a
    ``str`` / ``None`` (malformed JSON, partial stream, etc.). Anything
    we can't read is treated as the empty string — the guard never
    drops an empty speak_text (those are legitimate validation
    rejections handled by issue #1343, not hallucinated lyrics).
    """
    if not isinstance(text, Mapping):
        return ""
    raw = text.get("text", "")
    return raw.strip() if isinstance(raw, str) else ""


def _is_hallucinated_speak_text(
    *,
    call: ToolCall,
    same_batch_music_calls: frozenset[str],
    user_input: str,
) -> bool:
    """Issue #1708 — should this ``speak_text`` call be suppressed?

    Returns ``True`` (i.e. the call must NOT be executed — replace the
    tool result with a sentinel that teaches the LLM not to do this)
    when ALL of the following hold:

    * The call is ``speak_text`` (other tools are passed through).
    * A music-launch tool (``execute_music_code`` / ``set_vibe_preset``
      / ``load_track`` / ``generate_music`` / ``gen_play_from_library``)
      appears in the SAME LLM batch (either before OR after this
      ``speak_text`` call) — ``same_batch_music_calls`` carries those
      names. The order in the batch does NOT matter: the LLM
      sometimes calls ``speak_text(title)`` BEFORE
      ``execute_music_code`` (issue #1708 live shape) and sometimes
      AFTER (other variants). Either way the user hears robotic text
      on top of the beat.
    * The user request was NOT a vocal one (no «спой/пой/песня/рэп/
      зачитай/стих» cues) — rap / poem / song backing mode legitimately
      calls ``speak_text`` × N after ``execute_music_code``.
    * The ``text`` argument is longer than ``_ACCEPT_PHRASE_MAX_CHARS``
      — a short accept («Ок, играю Бах») is allowed even on non-vocal
      requests per the master prompt §5.

    The check is intentionally conservative: any ambiguity (no music
    tool in the same batch, vocal request, short accept) returns
    ``False`` and the ``speak_text`` runs normally. False negatives
    are tolerable — the audio still plays once, the user might hear
    lyrics on top of the beat. False positives would silence
    legitimate rap backing turns and break the master-prompt
    contract.
    """
    if call.name != "speak_text":
        return False
    if not same_batch_music_calls:
        return False
    if _is_vocal_request(user_input):
        return False
    text = _extract_speak_text(call.arguments)
    if not text:
        return False  # empty speak_text — issue #1343 path, not us
    return len(text) > _ACCEPT_PHRASE_MAX_CHARS


def _suppressed_speak_text_result(call: ToolCall) -> ToolResult:
    """Issue #1708 — sentinel result fed to the LLM instead of executing.

    The LLM must see WHY its ``speak_text`` call was dropped so it
    learns not to repeat the pattern. A short plain-text sentinel is
    cheaper than an ``is_error=True`` JSON payload — the LLM treats
    it as a tool result the same way and the next iteration will
    read it alongside the other tool messages.
    """
    return ToolResult(
        tool_call_id=call.id,
        content=(
            "[SYSTEM] speak_text подавлен после execute_music_code: "
            "нельзя читать сгенерированный текст поверх музыки. "
            "В этом режиме разрешён только КОРОТКИЙ accept "
            "(«Ок, играю <трек>», ≤40 символов). "
            "Верни 'done' сразу после execute_music_code."
        ),
        is_error=True,
    )


def _tool_spec_to_openai(spec: ToolSpec) -> dict[str, Any]:
    """Translate a :class:`ToolSpec` into the OpenAI Chat-Completions shape.

    The upstream DeepSeek / OpenAI provider expects a list of
    ``{"type": "function", "function": {"name", "description", "parameters"}}``
    dicts. We pass :class:`ToolSpec.parameters` through unchanged — it
    is already a JSON Schema fragment by contract.

    Specs without a description or schema still serialise cleanly:
    the fields are optional in the wire format.
    """
    return {
        "type": "function",
        "function": {
            "name": spec.name,
            "description": spec.description,
            "parameters": dict(spec.parameters),
        },
    }


__all__ = ["DialogCore", "DialogResult"]
