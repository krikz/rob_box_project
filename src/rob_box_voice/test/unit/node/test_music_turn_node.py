"""ADR-0148 на уровне ноды: ``DialogueNode._handle_result`` говорит о музыке фразу КОДА по итогу хода.

Итог хода — ``MusicTurn`` исполнителя тулов (как его заполняет ``SchedulerToolExecutor``), просьба о запуске —
грамматика по словам человека. Живые случаи 06.10: 14:50 set98207, 14:38 set97492, 15:30 UTC (``tools=[]``).
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

from rob_box_harness.core.agent_core import DialogResult
from rob_box_voice.core.llm_skip_reasons import new_llm_skip_counter
from rob_box_voice.core.media_phrases import (
    DJ_FAIL_TEXT,
    NOT_LAUNCHED_TEXT,
    SET_KEPT_TEXT,
    dj_started_text,
)
from rob_box_voice.core.music_player_state import MusicPlayerState
from rob_box_voice.core.music_turn import MusicTurn
from rob_box_voice.dialogue_node import DialogueNode

DJ_ARGS = {"action": "start", "theme": "ретро"}
DJ_OK = "{'ok': True, 'track_id': 'set98207:01:A:aa', 'started': True}"
RETRO_1530 = "ретро 8-бит Mario Tetris Aladdin Contra замути сэт на 30 минут"


def _make_node(turn: MusicTurn, set_tracks=None) -> DialogueNode:
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger
    for attr in ("_response_pub", "_state_pub", "_sound_trigger_pub", "_tts_control_pub", "_music_cleanup_pub",
                 "_dsm", "_effects", "_task_lock", "_dispatch_turn", "_loop", "_core", "_cancel_run",
                 "_speak_direct", "_retract_rejected_reply"):
        setattr(n, attr, MagicMock())
    n._active_tg_chat_id = None
    n._pending_music_cleanup = False
    n._active_batches = {}
    n._verbose_llm = False
    n._babble_retry_used = False
    n._action_claim_retry_used = False
    n._track_mode_music_active = False
    n._retry_dispatched_in_turn = False
    n._run_task = None
    n._startup_greeting_fired = False
    n._consume_synthetic_retry = MagicMock(return_value=True)
    n._check_babble_and_retry = MagicMock(return_value=False)
    n._generated_music_state = None
    n._music_player_state = MusicPlayerState(state="playing", track_id="t1", dj=True)
    n._llm_skipped_counter = new_llm_skip_counter()
    n._scheduler_executor = SimpleNamespace(music_turn=turn)
    n._turn_set_tracks = set_tracks
    return n


def _result(spoken: str, tools=(), speak_text_real: int = 0) -> DialogResult:
    r = DialogResult(spoken_text=spoken, tools_called=list(tools), finish_reason="stop")
    r.speak_text_real_count = speak_text_real
    r.speak_text_count = speak_text_real
    return r


def _turn(*launches) -> MusicTurn:
    turn = MusicTurn()
    for tool, args, is_error, content in launches:
        turn.record(tool, args, is_error=is_error, content=content)
    return turn


def _spoken_directly(n: DialogueNode) -> list:
    return [c.args[0] for c in n._speak_direct.call_args_list]


def test_set98207_lie_after_refused_request_is_replaced_by_code_phrase():
    turn = _turn(("dj_set", DJ_ARGS, False, DJ_OK),
                 ("request_music", {"intent": "melody"}, True, "музыка не заменена: set_playing"))
    n = _make_node(turn)

    n._handle_result(_result("done", tools=("dj_set", "request_music", "speak_text")),
                     user_input="сет ретро и сыграй 1812 Overture")

    assert _spoken_directly(n) == [f"{dj_started_text('', 'ретро')} {SET_KEPT_TEXT}"]
    n._retract_rejected_reply.assert_called_once_with()
    n._response_pub.publish.assert_not_called()  # текст модели в TTS не ушёл


def test_set97492_not_started_cannot_be_voiced_as_started():
    n = _make_node(_turn(("dj_set", DJ_ARGS, True, "сет не начался: not_started")))

    n._handle_result(_result("Сет собрался и стартовал!", tools=("dj_set",)), user_input="включи сет ретро")

    assert _spoken_directly(n) == [DJ_FAIL_TEXT]
    n._response_pub.publish.assert_not_called()
    n._dispatch_turn.assert_not_called()


def test_1530_tools_free_reply_to_a_set_request_is_the_honest_code_phrase():
    n = _make_node(MusicTurn(), set_tracks=24)

    n._handle_result(_result("Сет идёт — 30 минут ретро-ностальгии!"), user_input=RETRO_1530)

    assert _spoken_directly(n) == [NOT_LAUNCHED_TEXT]
    n._retract_rejected_reply.assert_called_once_with()
    n._response_pub.publish.assert_not_called()


def test_successful_set_without_model_speech_is_announced_by_code():
    n = _make_node(_turn(("dj_set", DJ_ARGS, False, DJ_OK)))

    n._handle_result(_result("done", tools=("dj_set",)), user_input="включи сет ретро")

    assert _spoken_directly(n) == [dj_started_text("", "ретро")]


def test_non_music_turn_keeps_the_model_reply():
    n = _make_node(MusicTurn())

    n._handle_result(_result("Нормально, а у тебя?"), user_input="как дела")

    n._speak_direct.assert_not_called()
    n._retract_rejected_reply.assert_not_called()
    assert "Нормально" in " ".join(c.args[0].data for c in n._response_pub.publish.call_args_list)
