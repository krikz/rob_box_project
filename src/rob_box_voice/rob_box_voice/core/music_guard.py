"""music_guard.py — Issue #992 Bug B/C post-turn music guard (TD-2 extraction).

Extracted from :meth:`DialogueNode._apply_music_guard` (issue #1405 ARCH-review)
so the policy / retry-budget logic is unit-testable without a ROS2 node and
reusable by any future harness-style adapter. Pure Python: no rclpy, no I/O.

Owns:

* Two **retry budgets** (``_dj_retry_count``, ``_user_retry_count``) — the
  only place these counters live. Previously they were split between
  ``DialogueNode._dj_auto_retry_count`` and ``DialogueNode._music_guard_retry_count``
  with reset sites in two different scopes (``_dispatch_dj_turn`` vs.
  ``_run_turn``), which made new bug-class scenarios (e.g. «юзер попросил
  диджей, но LLM вернул tools_called=[] и текст с песней») require yet
  another budget and yet another ``if`` block in the guard.
* The full Bug B (DJ auto) + Bug C (user music) decision tree in
  :meth:`MusicGuard.evaluate`. The guard returns a :class:`MusicGuardVerdict`
  telling the caller exactly what to do (``SKIP``, ``DJ_RETRY``,
  ``USER_RETRY``, ``NUDGE``, ``SKIP_NOT_APPLICABLE``). The DialogueNode
  adapter dispatches each verdict (speak / retry / log) — it does NOT
  decide policy.

See also:

* :mod:`rob_box_voice.core.dialogue_guards` — keyword heuristics
  (``user_wants_music``, ``is_music_stop_command``, ``is_vocal_request``)
  used here, kept in the guards module so future bug-D-style fixes can
  extend the keyword lists without touching the policy module.
* ARCH-review #1405 / ADR-0021.
"""

import logging
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

from .dialogue_guards import (
    GENERATED_MUSIC_TOOLS,
    MUSIC_STOP_TOOLS,
    RENARDO_MUSIC_TOOLS,
    is_music_stop_command,
    is_vocal_request,
    user_wants_music,
)


class MusicGuardVerdictKind(str, Enum):
    """Action the adapter (:meth:`DialogueNode._apply_music_guard`) must take."""

    #: ``execute_music_code`` already in ``tools_called`` — guard did its
    #: job, nothing more to do. Counters reset on the way out.
    SKIP = "skip"

    #: ``was_dj_auto=True`` AND DJ enabled AND budget not exhausted —
    #: dispatch a synchronous DJ retry (Bug B path).
    DJ_RETRY = "dj_retry"

    #: User asked for music AND budget not exhausted — dispatch a
    #: synchronous user-music retry (Bug C path).
    USER_RETRY = "user_retry"

    #: User asked for music but **budget exhausted** — publish the
    #: spoken nudge fallback («Я тут растерялся — бит не запустился,
    #: попробуй ещё раз»). After a nudge the budget is reset so the
    #: next genuine user request gets a fresh one.
    NUDGE = "nudge"

    #: User asked to STOP music but the LLM called no stop tool — the
    #: adapter must force the stop itself (issue #992 Bug F, live 30.08).
    FORCE_STOP = "force_stop"

    #: Guard deliberately skipped (stop-command OR user did not ask for
    #: music OR DJ was off). Adapter only logs a diagnostic.
    SKIP_NOT_APPLICABLE = "skip_not_applicable"


@dataclass(frozen=True)
class MusicGuardVerdict:
    """Result of :meth:`MusicGuard.evaluate`.

    Attributes:
        kind: Action the adapter must take.
        reason: Short tag for the warning log line — e.g.
            ``"executed"``, ``"bug_b"``, ``"bug_c"``, ``"budget_exhausted"``,
            ``"stop_command"``, ``"not_applicable"``. The reason is
            intended for human log reading, not programmatic dispatch.
        prompt: Synthetic retry prompt when ``kind`` is ``DJ_RETRY`` or
            ``USER_RETRY``; ``None`` otherwise. The adapter dispatches
            ``self._dispatch_dj_turn(prompt)`` / ``self._dispatch_turn(prompt, ...)``.
    """

    kind: MusicGuardVerdictKind
    reason: str
    prompt: Optional[str] = None


class MusicGuard:
    """Policy + retry-budget holder for the post-turn music guard.

    Usage::

        guard = MusicGuard(logger=...)
        verdict = guard.evaluate(
            was_dj_auto=True,
            user_input="сыграй трек",
            tools_called=("speak_text",),  # NO execute_music_code
        )
        if verdict.kind is MusicGuardVerdictKind.DJ_RETRY:
            self._dispatch_dj_turn(verdict.prompt)
            return True  # caller should defer DIALOGUE_END
        ...

    The guard is **stateless across DialogueNode instances** (it stores
    nothing that outlives the node) but **stateful across turns** —
    the retry counters track how many times this *node* has already
    retried inside the current user request / DJ transition.

    Args:
        max_dj_retries: How many Bug B synchronous retries to allow
            inside one DJ auto-transition before giving up to the
            next 5 s tick. Default ``2`` matches the legacy
            ``DialogueNode.MAX_DJ_AUTO_RETRIES``.
        max_user_retries: How many Bug C synchronous retries to allow
            inside one user request before falling through to the
            spoken nudge. Default ``1`` matches the legacy
            ``DialogueNode.MAX_MUSIC_GUARD_RETRIES`` (LLM only needs
            one CRITICAL nudge; further nudges are noise).
        logger: Optional logger for diagnostic / warning lines.
            When ``None`` the guard silently swallows log calls — handy
            for unit tests.
    """

    #: Legacy defaults preserved verbatim from the original
    #: ``DialogueNode`` constants so behaviour does not regress.
    DEFAULT_MAX_DJ_RETRIES: int = 2
    DEFAULT_MAX_USER_RETRIES: int = 1

    def __init__(
        self,
        *,
        max_dj_retries: int = DEFAULT_MAX_DJ_RETRIES,
        max_user_retries: int = DEFAULT_MAX_USER_RETRIES,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self._max_dj_retries = max_dj_retries
        self._max_user_retries = max_user_retries
        self._dj_retry_count: int = 0
        self._user_retry_count: int = 0
        self._logger = logger

    # ------------------------------------------------------------------
    # Read-only accessors — used by the DialogueNode adapter for the
    # warning lines that used to read ``self._dj_auto_retry_count`` etc.
    # ------------------------------------------------------------------

    @property
    def dj_retry_count(self) -> int:
        return self._dj_retry_count

    @property
    def user_retry_count(self) -> int:
        return self._user_retry_count

    @property
    def max_dj_retries(self) -> int:
        return self._max_dj_retries

    @property
    def max_user_retries(self) -> int:
        return self._max_user_retries

    # ------------------------------------------------------------------
    # Budget control — only the DialogueNode adapter calls these.
    # ------------------------------------------------------------------

    def reset_for_new_user_request(self) -> None:
        """Reset the user-music budget on a fresh user-initiated turn.

        Mirrors the legacy ``_run_turn:2171`` reset site, which used to
        reset ``self._music_guard_retry_count = 0`` *only* when the turn
        was not a babble retry, not a DJ auto, and not the synthetic
        music-retry prompt. The DJ budget is intentionally NOT reset —
        a DJ transition is independent of the user-music budget.
        """
        self._user_retry_count = 0

    def reset_for_new_dj_transition(self) -> None:
        """Reset the DJ budget when a fresh 5 s tick fires a transition.

        Called from :meth:`DialogueNode._dispatch_dj_turn` (the
        ``from_tick`` branch). The user-music budget is intentionally
        NOT touched — DJ and user-music are independent counters.
        """
        self._dj_retry_count = 0

    # ------------------------------------------------------------------
    # Policy — the decision tree that used to live inline in
    # :meth:`DialogueNode._apply_music_guard`.
    # ------------------------------------------------------------------

    def evaluate(
        self,
        *,
        was_dj_auto: bool,
        user_input: str,
        tools_called: Optional[Tuple[str, ...]],
        dj_enabled: bool = False,
        build_music_retry_prompt=None,
        build_dj_retry_prompt=None,
    ) -> MusicGuardVerdict:
        """Decide what the post-turn music guard should do.

        Args:
            was_dj_auto: ``True`` when this is a DJ-mode tick transition
                (Bug B path active).
            user_input: The original user command (or DJ auto-prompt
                for tick transitions). Used by the keyword detectors
                (``user_wants_music``, ``is_music_stop_command``,
                ``is_vocal_request``) and by the Bug C retry prompt.
            tools_called: Tuple of tool names the LLM invoked this
                turn. ``"execute_music_code"`` presence short-circuits
                the guard to ``SKIP``.
            dj_enabled: Whether DJ mode is currently on. ``False``
                disables the Bug B path even if ``was_dj_auto`` somehow
                leaked. The adapter passes ``self._dj.state.enabled``.
            build_music_retry_prompt: Callable ``(user_input) -> str``
                that builds the Bug C synthetic retry prompt. The
                adapter passes ``self._build_music_retry_prompt`` —
                kept as a parameter (instead of imported) so the guard
                stays pure-Python testable.
            build_dj_retry_prompt: Callable ``() -> str`` for the
                Bug B synthetic retry prompt. The adapter passes
                ``self._build_dj_retry_prompt``.

        Returns:
            :class:`MusicGuardVerdict` whose ``kind`` tells the adapter
            what to do and ``prompt`` (when applicable) carries the
            synthetic prompt the adapter should dispatch.

        Notes:
            Counters are advanced **here** (not by the adapter) so the
            policy and the bookkeeping cannot drift apart. The
            ``SKIP`` verdict on success resets both counters atomically.
        """
        tools_set = set(tools_called or ())
        # Issue #1392 follow-up: MiniMax AI-генерация тоже «запустила музыку».
        # Без этого Bug C ретраил «сгенерируй трек про X» (не-vocal, без
        # execute_music_code) → retry-prompt гнал LLM в фантомный handle_music.
        _music_started = tools_set & (RENARDO_MUSIC_TOOLS | GENERATED_MUSIC_TOOLS)
        if _music_started:
            # Success — reset both budgets so a future failure gets a
            # fresh allocation. Mirrors the legacy 2787/2788 reset.
            self._dj_retry_count = 0
            self._user_retry_count = 0
            self._log_debug(
                f"🎵 [music_guard] music tool in tools_called "
                f"({sorted(_music_started)!r}) → SKIP (counters reset)"
            )
            return MusicGuardVerdict(
                kind=MusicGuardVerdictKind.SKIP,
                reason="executed",
            )

        # Bug B — DJ auto-transition completed without music.
        if was_dj_auto and dj_enabled:
            if self._dj_retry_count >= self._max_dj_retries:
                self._log_warning(
                    "🎵 [issue 992 Bug B] DJ retry budget exhausted "
                    f"({self._dj_retry_count}/{self._max_dj_retries}); "
                    "letting 5 s tick take over"
                )
                # Reset so the *next* transition gets a fresh budget —
                # mirrors the legacy 2798 reset.
                self._dj_retry_count = 0
                return MusicGuardVerdict(
                    kind=MusicGuardVerdictKind.SKIP_NOT_APPLICABLE,
                    reason="bug_b_budget_exhausted",
                )
            self._dj_retry_count += 1
            prompt = (
                build_dj_retry_prompt()
                if build_dj_retry_prompt is not None
                else "[CRITICAL] DJ retry — call execute_music_code"
            )
            self._log_warning(
                "🎵 [issue 992 Bug B] DJ auto-transition completed "
                "without execute_music_code — forcing synchronous retry "
                f"({self._dj_retry_count}/{self._max_dj_retries})"
            )
            return MusicGuardVerdict(
                kind=MusicGuardVerdictKind.DJ_RETRY,
                reason="bug_b",
                prompt=prompt,
            )

        # 🔴 FIX (live 30.08, vision-pi 12:33): «останови музыку» → LLM
        # ответила «Музыка выключена.» с ``tools=[]``. Ни ``stop_music``, ни
        # чего-либо ещё вызвано не было, и mp3 из ``gen_play_from_library``
        # доиграл до конца ещё 20 секунд после «выключена». Стоп —
        # идемпотентная операция, поэтому здесь мы не ретраим LLM, а
        # останавливаем музыку сами (адаптер публикует music_cleanup).
        if is_music_stop_command(user_input) and not (tools_set & MUSIC_STOP_TOOLS):
            self._log_warning(
                "🎵 [issue 992 Bug F] stop-command без stop-тула "
                f"(tools={sorted(tools_set)!r}) — принудительный стоп из кода"
            )
            return MusicGuardVerdict(
                kind=MusicGuardVerdictKind.FORCE_STOP,
                reason="stop_command_unbacked",
            )

        # Bug C — user asked for music but LLM skipped execute_music_code.
        if not user_wants_music(user_input):
            self._log_info(
                f"🎵 [music_guard] skip: was_dj_auto={was_dj_auto} "
                f"user_input={user_input!r} tools={sorted(tools_set)!r} "
                f"→ no action (user does NOT want music)"
            )
            return MusicGuardVerdict(
                kind=MusicGuardVerdictKind.SKIP_NOT_APPLICABLE,
                reason="not_music_request",
            )

        # 🔴 FIX (live 06.08): stop-commands («хватит диджеить»,
        # «выключи музыку») must NOT trigger a music retry — they ask
        # to STOP music, not START it. Bug C previously mis-classified
        # them as music requests and re-enabled music via the retry.
        # Сюда доходят только стопы, которые LLM уже отработала тулом
        # (безтуловые перехвачены веткой FORCE_STOP выше).
        if is_music_stop_command(user_input):
            self._log_debug(
                "🎵 [issue 992 Bug C] stop-command — skipping music "
                "guard entirely"
            )
            return MusicGuardVerdict(
                kind=MusicGuardVerdictKind.SKIP_NOT_APPLICABLE,
                reason="stop_command",
            )

        # 🔴 FIX (live 10:00): vocal requests («спой/пой/песня») — when
        # the LLM did ANY tool (speak_text, etc.) it has honoured the
        # request. Only nudge when the LLM did literally nothing.
        if is_vocal_request(user_input) and tools_set:
            self._log_debug(
                "🎵 [issue 992 Bug C] vocal request, LLM replied "
                f"(tools={sorted(tools_set)!r}) — no nudge needed"
            )
            return MusicGuardVerdict(
                kind=MusicGuardVerdictKind.SKIP_NOT_APPLICABLE,
                reason="vocal_satisfied",
            )

        if self._user_retry_count < self._max_user_retries:
            self._user_retry_count += 1
            prompt = (
                build_music_retry_prompt(user_input)
                if build_music_retry_prompt is not None
                else (
                    "[CRITICAL] В прошлом цикле ты НЕ вызвал "
                    "execute_music_code"
                )
            )
            self._log_warning(
                f"🎵 [issue 992 Bug C] user asked for music but LLM "
                f"skipped execute_music_code (tools={sorted(tools_set)!r}); "
                f"synchronous retry {self._user_retry_count}/"
                f"{self._max_user_retries}"
            )
            return MusicGuardVerdict(
                kind=MusicGuardVerdictKind.USER_RETRY,
                reason="bug_c",
                prompt=prompt,
            )

        # Budget exhausted — publish the spoken nudge and reset so the
        # *next* genuine user request gets a fresh allocation.
        self._log_warning(
            f"🎵 [issue 992 Bug C] user asked for music but LLM "
            f"skipped execute_music_code (tools={sorted(tools_set)!r}); "
            "publishing spoken nudge"
        )
        self._user_retry_count = 0
        return MusicGuardVerdict(
            kind=MusicGuardVerdictKind.NUDGE,
            reason="budget_exhausted",
        )

    # ------------------------------------------------------------------
    # Logging helpers — swallow silently when no logger is configured
    # so unit tests don't have to mock logging just to instantiate.
    # ------------------------------------------------------------------

    def _log_debug(self, msg: str) -> None:
        if self._logger is not None:
            self._logger.debug(msg)

    def _log_info(self, msg: str) -> None:
        if self._logger is not None:
            self._logger.info(msg)

    def _log_warning(self, msg: str) -> None:
        if self._logger is not None:
            self._logger.warning(msg)


__all__ = [
    "MusicGuard",
    "MusicGuardVerdict",
    "MusicGuardVerdictKind",
]