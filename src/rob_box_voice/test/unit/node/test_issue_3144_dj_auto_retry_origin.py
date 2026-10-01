"""Issue #3144 — ретрай гуарда на DJ_AUTO-ходе остаётся DJ_AUTO-ходом.

Живой прогон 28.09.2026 18:48 (``voice-assistant_1824-1852.log``)::

    18:48:53.2 Bug E: заявлено действие без тула (music_prose_action)
    18:48:53.2 [turn] user_input='[Speaker:unknown] [CRITICAL] …' was_dj_auto=False
    18:48:54.9 Bug C: user asked for music … retry 1/3
    18:48:57.8 … «Музыка играет, а вот эту просьбу я выполнить не смог»

Юзер ничего не просил. Проверяем:

* синтетический ретрай, отправленный изнутри DJ-хода, уходит с
  ``is_dj_auto=True`` (Bug E, Bug D, TurnGuards — общий путь
  ``_dispatch_turn``);
* реплика юзера из DJ-хода (дренаж очереди) DJ-ходом не становится;
* Bug C / tool-skipped не читают промпт DJ_AUTO как просьбу юзера;
* исчерпание бюджета на DJ-ходе молчит, фраза #3125 не звучит и чужой
  ответ из истории не отзывается.
"""

from __future__ import annotations

import ast
import asyncio
import contextvars
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.music_guard import (
    MusicGuard,
    MusicGuardVerdict,
    MusicGuardVerdictKind,
)
from rob_box_voice.core.turn_origin import TURN_IS_DJ_AUTO, retry_is_dj_auto
from rob_box_voice import dialogue_node as dialogue_node_module
from rob_box_voice.dialogue_node import DialogueNode

DJ_PROMPT = "[DJ_AUTO — ПЕРЕХОД #2] Ты Снупдог, держи сет, смени трек."
# Живая реплика из карточки #2548 — music_prose_action при dj_active=True.
PROSE_CLAIM = (
    "Ок, давай я снова перезапущу. Бочкинс с Григом наверху — стартуя заново."
)
NUDGE_WHILE_PLAYING = DialogueNode.MUSIC_RETRY_NUDGE_WHILE_PLAYING_TEXT


def _in_turn(is_dj_auto: bool, fn):
    """Вызвать ``fn`` так, будто идёт ход с данным происхождением."""
    ctx = contextvars.copy_context()

    def _run():
        TURN_IS_DJ_AUTO.set(is_dj_auto)
        return fn()

    return ctx.run(_run)


# ── чистая политика ─────────────────────────────────────────────────


class TestRetryIsDjAuto:
    @pytest.mark.parametrize("parent", [False, True])
    def test_explicit_dj_auto_wins(self, parent):
        assert _in_turn(
            parent,
            lambda: retry_is_dj_auto(is_dj_auto=True, is_synthetic=False),
        ) is True

    def test_synthetic_retry_inherits_dj_parent(self):
        assert _in_turn(
            True, lambda: retry_is_dj_auto(is_dj_auto=False, is_synthetic=True)
        ) is True

    def test_synthetic_retry_of_user_turn_stays_user(self):
        assert _in_turn(
            False, lambda: retry_is_dj_auto(is_dj_auto=False, is_synthetic=True)
        ) is False

    def test_user_phrase_drained_inside_dj_turn_is_not_dj(self):
        assert _in_turn(
            True, lambda: retry_is_dj_auto(is_dj_auto=False, is_synthetic=False)
        ) is False

    def test_outside_any_turn_is_not_dj(self):
        assert retry_is_dj_auto(is_dj_auto=False, is_synthetic=True) is False

    def test_asyncio_retry_task_inherits_parent_context(self):
        """Ретрай — отдельная asyncio-задача; контекст копируется из
        родителя в момент создания, как у ``TURN_EPOCH`` (#2835)."""

        async def parent():
            TURN_IS_DJ_AUTO.set(True)

            async def child():
                return retry_is_dj_auto(is_dj_auto=False, is_synthetic=True)

            return await asyncio.get_running_loop().create_task(child())

        assert asyncio.run(parent()) is True


# ── нода: диспатч ретраев ───────────────────────────────────────────


def _node(monkeypatch, *, dj_enabled: bool = True):
    n = object.__new__(DialogueNode)
    n.get_logger = lambda: MagicMock()
    n._loop = None
    n._pending_music_cleanup = False
    n._session_started_at = None
    n._dj = SimpleNamespace(state=SimpleNamespace(enabled=dj_enabled))
    calls: list[dict] = []

    def fake_run_turn(user_input, **kwargs):
        calls.append({"user_input": user_input, **kwargs})
        return None

    n._run_turn = fake_run_turn
    monkeypatch.setattr(
        dialogue_node_module.asyncio,
        "run_coroutine_threadsafe",
        lambda coro, loop: None,
    )
    return n, calls


class TestBugERetryOnDjAutoTurn:
    def _prime_guard(self, n):
        n._action_claim_retry_used = False
        n._synthetic_retries_left = DialogueNode.DEFAULT_SYNTHETIC_RETRIES
        n._retry_dispatched_in_turn = False
        n._dsm = MagicMock()
        n._publish_state = lambda: None

    def test_bug_e_retry_keeps_is_dj_auto(self, monkeypatch):
        n, calls = _node(monkeypatch)
        self._prime_guard(n)
        fired = _in_turn(
            True,
            lambda: n._check_unbacked_action_claim_and_retry(
                spoken=PROSE_CLAIM,
                user_input=DJ_PROMPT,
                tools_called=(),
                dj_active=True,
            ),
        )
        assert fired is True, "Bug E должен поймать prose-claim на DJ-ходе"
        assert len(calls) == 1
        assert calls[0]["is_dj_auto"] is True, (
            "ретрай Bug E на DJ_AUTO-ходе ушёл ходом юзера (#3144)"
        )
        assert calls[0]["is_synthetic"] is True

    def test_bug_e_retry_on_user_turn_stays_user(self, monkeypatch):
        n, calls = _node(monkeypatch)
        self._prime_guard(n)
        fired = _in_turn(
            False,
            lambda: n._check_unbacked_action_claim_and_retry(
                spoken=PROSE_CLAIM,
                user_input="давай старайся",
                tools_called=(),
                dj_active=True,
            ),
        )
        assert fired is True
        assert calls[0]["is_dj_auto"] is False


class TestEveryGuardRetryPath:
    """Bug D, TurnGuards и прочие ретраи идут через ``_dispatch_turn`` с
    ``is_synthetic=True`` — наследование живёт там, в одном месте."""

    @pytest.mark.parametrize(
        "retry_kwargs",
        [
            {"is_babble_retry": True},  # Bug D
            {"is_action_claim_retry": True},  # Bug E / TurnGuards
            {"is_code_retry": True},  # Bug C'
            {},  # tool-skipped, phantom, universal, markup, …
        ],
    )
    def test_synthetic_dispatch_inside_dj_turn(self, monkeypatch, retry_kwargs):
        n, calls = _node(monkeypatch)
        _in_turn(
            True,
            lambda: n._dispatch_turn(
                "[CRITICAL] retry",
                is_synthetic=True,
                raw_user_command=DJ_PROMPT,
                **retry_kwargs,
            ),
        )
        assert calls[0]["is_dj_auto"] is True

    def test_drained_user_phrase_inside_dj_turn_is_user(self, monkeypatch):
        n, calls = _node(monkeypatch)
        _in_turn(
            True,
            lambda: n._dispatch_turn("горный король погромче", raw_user_command="x"),
        )
        assert calls[0]["is_dj_auto"] is False

    def test_run_turn_publishes_its_origin(self):
        src = Path(dialogue_node_module.__file__).read_text(encoding="utf-8")
        tree = ast.parse(src)
        method = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.AsyncFunctionDef) and node.name == "_run_turn"
        )
        sets = [
            node
            for node in ast.walk(method)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr in {"set", "reset"}
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "TURN_IS_DJ_AUTO"
        ]
        assert {c.func.attr for c in sets} == {"set", "reset"}


# ── нода: Bug C / tool-skipped на DJ-ходе ────────────────────────────


class TestUserGuardsSkipDjAutoTurn:
    def _node(self, *, dj_enabled):
        n = object.__new__(DialogueNode)
        n.get_logger = lambda: MagicMock()
        n._dj = SimpleNamespace(state=SimpleNamespace(enabled=dj_enabled))
        n._apply_music_guard = MagicMock(return_value=False)
        n._apply_tool_skipped_guard = MagicMock(return_value=True)
        return n

    def _run(self, n, *, was_dj_auto):
        return n._apply_post_turn_retry_guards(
            result=SimpleNamespace(
                tools_called=(), spoken_text=PROSE_CLAIM,
                tool_error_occurred=False, succeeded_tools=(),
            ),
            was_dj_auto=was_dj_auto,
            user_input=DJ_PROMPT,
            retries_allowed=True,
        )

    def test_dj_off_dj_auto_turn_runs_no_user_guards(self):
        n = self._node(dj_enabled=False)
        assert self._run(n, was_dj_auto=True) == (False, False)
        n._apply_music_guard.assert_not_called()
        n._apply_tool_skipped_guard.assert_not_called()

    def test_dj_on_dj_auto_turn_keeps_bug_b_but_no_tool_skipped(self):
        n = self._node(dj_enabled=True)
        self._run(n, was_dj_auto=True)
        n._apply_music_guard.assert_called_once()
        n._apply_tool_skipped_guard.assert_not_called()

    def test_user_turn_unchanged(self):
        n = self._node(dj_enabled=True)
        assert self._run(n, was_dj_auto=False) == (False, True)
        n._apply_tool_skipped_guard.assert_called_once()

    def test_music_guard_on_dj_auto_retry_is_bug_b_not_bug_c(self):
        """Ретрай с ``is_dj_auto=True`` при включённом DJ — Bug B."""
        verdict = MusicGuard().evaluate(
            was_dj_auto=True,
            user_input=DJ_PROMPT,
            tools_called=(),
            dj_enabled=True,
            build_music_retry_prompt=lambda _u: "bug c",
            build_dj_retry_prompt=lambda: "bug b",
        )
        assert verdict.kind is MusicGuardVerdictKind.DJ_RETRY


class TestDjRetryBudgetExhaustedIsSilent:
    def test_no_nudge_no_discard_on_dj_turn(self):
        n = object.__new__(DialogueNode)
        n.get_logger = lambda: MagicMock()
        n._dj = SimpleNamespace(state=SimpleNamespace(enabled=True))
        n._retry_dispatched_in_turn = False
        n._dj_giveup_silent_in_turn = False
        n._music_guard = MagicMock()
        n._music_guard.evaluate.return_value = MusicGuardVerdict(
            kind=MusicGuardVerdictKind.DJ_RETRY, reason="bug_b", prompt="bug b",
        )
        n._build_music_retry_prompt = lambda _u: ""
        n._build_dj_retry_prompt = lambda: ""
        n._consume_synthetic_retry = MagicMock(return_value=False)
        n._speak_direct = MagicMock()
        n._discard_last_music_reply = MagicMock()
        n._dispatch_dj_turn = MagicMock()
        # Музыка играет — раньше здесь звучала фраза #3125.
        n._music_playing_now = lambda: True

        dispatched = n._apply_music_guard(
            was_dj_auto=True, user_input=DJ_PROMPT, tools_called=(),
            spoken=PROSE_CLAIM,
        )

        assert dispatched is False
        n._dispatch_dj_turn.assert_not_called()
        spoken = [c.args[0] for c in n._speak_direct.call_args_list]
        assert NUDGE_WHILE_PLAYING not in spoken
        assert spoken == []
        # DJ-ход ответов в историю не пишет — отзывать нечего, а
        # discard_last_reply снёс бы последний ответ ЮЗЕРУ.
        n._discard_last_music_reply.assert_not_called()
        assert n._dj_giveup_silent_in_turn is True
