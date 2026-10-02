"""Issue #3176 — адаптер ``DialogueNode``: заказ по имени кодом, до LLM.

Харнесс как в ``test_issue_3134_media_router_node`` (без ROS2,
``object.__new__``): реальные ``_on_stt`` → ``SttAdmission`` →
``MediaCommandStep`` → ``_route_media_command`` → ``_execute_play_named``;
мок — исполнитель тулов (вместо ``SchedulerToolExecutor`` →
``/mcp/execute``), ``_dispatch_turn`` (вход LLM-хода), ``_speak_direct``
(TTS). Ответ ``lookup_melody`` — в формате адаптера
(``message`` + ``repr(data)``, ``core_adapter._result_content``). Играет
``request_music`` движка v2 (PR-11; ``compose_music`` удалён в PR-13a).
"""

from __future__ import annotations

import asyncio
from unittest.mock import MagicMock

import pytest

from rob_box_llm.provider import ToolResult

from rob_box_voice.core.dialogue_text import DEFAULT_WAKE_WORDS
from rob_box_voice.core.llm_skip_reasons import new_llm_skip_counter
from rob_box_voice.core.music_player_state import MusicPlayerState
from rob_box_voice.dialogue_node import DialogueNode
import rob_box_voice.dialogue_node as dialogue_node_module

_FUR_ELISE = {
    "name": "furelise",
    "title": "Fur Elise",
    "display_title": "Fur Elise",
    "match": {"matched": ["fur", "elise"], "unmatched": [], "coverage": 1.0, "ignored": []},
}


class _MelodyExecutor:
    """``execute(ToolCall)`` с базой из одной мелодии — «к элизе»."""

    def __init__(self, *, play_ok: bool = True) -> None:
        self.calls = []
        self.turns_begun = 0
        self._play_ok = play_ok

    def begin_turn(self) -> None:
        self.turns_begun += 1

    async def execute(self, call):
        args = dict(call.arguments)
        self.calls.append((call.name, args))
        if call.name == "lookup_melody":
            if args["name"] == "к элизе":
                return self._ok(call, f"Нашёл «Fur Elise».\n{_FUR_ELISE!r}")
            return self._err(call, f"Мелодия {args['name']!r} не найдена в библиотеке.")
        if call.name == "request_music":
            if self._play_ok:
                return self._ok(call, "{'ok': True, 'track_id': 'mel:01:A:aa'}")
            return self._err(call, "мелодия не заиграла: not_started")
        return self._err(call, f"unexpected tool {call.name}")

    @staticmethod
    def _ok(call, content):
        return ToolResult(tool_call_id=call.id, content=content, is_error=False)

    @staticmethod
    def _err(call, content):
        return ToolResult(tool_call_id=call.id, content=content, is_error=True)


def _make_node(*, playing=False, dj=False, track=None, state_name="IDLE", **exec_kw):
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._wake_words = list(DEFAULT_WAKE_WORDS)
    n._command_intent_gate_enabled = False
    n._speaker_by_text = {}
    n._llm_skipped_counter = new_llm_skip_counter()
    n._maybe_log_skip_summary = MagicMock()
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    n._dsm.current_state.name = state_name
    n._cancel_run = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._speak_direct = MagicMock()
    n._verbose_llm = False
    n._active_tg_chat_id = None
    n._pending_music_cleanup = True  # хвост прошлого хода
    n._track_mode_music_active = False
    n._music_player_state = MusicPlayerState(
        state="playing" if playing else "idle", dj=dj,
        dj_info={"enabled": dj, "title": track} if track else {"enabled": dj},
    )
    n._scheduler_executor = _MelodyExecutor(**exec_kw)
    n._loop = MagicMock()
    return n


@pytest.fixture
def run_plans(monkeypatch):
    """Перехватить ``run_coroutine_threadsafe`` и выполнить корутину сразу."""
    pending = []

    def _capture(coro, loop):  # noqa: ARG001
        pending.append(coro)
        return MagicMock()

    monkeypatch.setattr(
        dialogue_node_module.asyncio, "run_coroutine_threadsafe", _capture
    )

    def _drain():
        loop = asyncio.new_event_loop()
        try:
            while pending:
                loop.run_until_complete(pending.pop(0))
        finally:
            loop.close()

    return _drain


def _stt(node: DialogueNode, text: str) -> None:
    msg = MagicMock()
    msg.data = text
    node._on_stt(msg)


# ── найдена → request_music без LLM ──────────────────────────────────────────


@pytest.mark.parametrize("playing", [False, True])
def test_found_melody_plays_without_llm(run_plans, playing):
    n = _make_node(playing=playing, track="Still Dre" if playing else None)
    _stt(n, "Робот, поставь к Элизе")
    # Реплика забрана сразу: ход LLM не стартует, пока ищем мелодию.
    n._dispatch_turn.assert_not_called()
    run_plans()
    n._dispatch_turn.assert_not_called()
    assert n._scheduler_executor.calls == [
        ("lookup_melody", {"name": "к элизе"}),
        ("request_music", {"intent": "melody", "text": "к элизе"}),
    ]
    assert n._scheduler_executor.turns_begun == 1  # лимит #2859 снят
    n._speak_direct.assert_called_once_with("Ставлю «К элизе».")
    n._cancel_run.assert_called_once()
    assert n._llm_skipped_counter["media_command"] == 1
    # отложенный cleanup прошлого хода не гасит заказ после фразы
    assert n._pending_music_cleanup is False
    assert n._track_mode_music_active is True


# ── общее название → LLM, роутер не трогает ────────────────────────────


def test_generic_request_goes_to_llm(run_plans):
    n = _make_node()
    _stt(n, "Робот, сыграй что-нибудь весёлое")
    run_plans()
    n._dispatch_turn.assert_called_once()
    assert n._scheduler_executor.calls == []
    n._speak_direct.assert_not_called()


# ── не найдена → LLM, роутер молчит ────────────────────────────────────


@pytest.mark.parametrize("state_name", ["IDLE", "DIALOGUE"])
def test_unknown_melody_goes_to_llm_silently(run_plans, state_name):
    n = _make_node(state_name=state_name)
    _stt(n, "Робот, поставь абракадабра")
    n._dispatch_turn.assert_not_called()  # пока база не ответила
    run_plans()
    assert n._scheduler_executor.calls == [("lookup_melody", {"name": "абракадабра"})]
    n._dispatch_turn.assert_called_once()
    args, kwargs = n._dispatch_turn.call_args
    assert args[0] == "поставь абракадабра"  # реплика без wake-слова
    assert kwargs["raw_user_command"] == "поставь абракадабра"
    n._speak_direct.assert_not_called()  # нет двойного ответа
    assert n._llm_skipped_counter["media_command"] == 0


def test_found_but_not_started_is_said_honestly(run_plans):
    n = _make_node(play_ok=False)
    _stt(n, "Робот, поставь к элизе")
    run_plans()
    n._dispatch_turn.assert_not_called()
    n._speak_direct.assert_called_once()
    assert "не получилось" in n._speak_direct.call_args[0][0]


# ── DJ-сет: заказ не останавливает сет ──────────────────────────────────


def test_order_mid_dj_set_does_not_stop_set(run_plans):
    n = _make_node(playing=True, dj=True, track="club #3", state_name="DIALOGUE")
    _stt(n, "Робот, поставь к элизе")
    run_plans()
    assert [c[0] for c in n._scheduler_executor.calls] == ["lookup_melody", "request_music"]
    assert "stop_music" not in [c[0] for c in n._scheduler_executor.calls]
    assert "dj_set" not in [c[0] for c in n._scheduler_executor.calls]
    n._speak_direct.assert_called_once_with("Ставлю «К элизе».")
    n._dispatch_turn.assert_not_called()


def test_route_without_resume_path_leaves_order_to_llm():
    """Без пути назад в приём (``on_miss``) роутер заказ не забирает."""
    n = _make_node()
    assert n._route_media_command("поставь к элизе") is False
    assert n._scheduler_executor.calls == []
