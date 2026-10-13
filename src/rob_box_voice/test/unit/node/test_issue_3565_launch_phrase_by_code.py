"""Issue #3565 / #3461 — фраза запуска музыки не копируется из истории и не доходит до человека без ``started``.

1. Ход роутера с запуском (``dj_set`` / ``request_music``) не кладётся в окно модели парой «реплика → фраза кода»
   (``AgentCore.record_external_turn``): иначе фраза — образец, который модель повторяет с ``tools=[]``.
2. Просили пьесу по имени (``PLAY_NAMED``), роутер промахнулся, модель в ходе ничего не запустила — фразу говорит
   код (:data:`NOT_PLAYED_TEXT`), а не текст модели.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.agent_core import DialogResult
from rob_box_voice.core.llm_skip_reasons import new_llm_skip_counter
from rob_box_voice.core.media_command_grammar import MediaIntent
from rob_box_voice.core.media_phrases import NOT_LAUNCHED_TEXT, NOT_PLAYED_TEXT
from rob_box_voice.core.music_player_state import MusicPlayerState
from rob_box_voice.core.music_turn import MusicTurn, piece_requested, turn_reply
from rob_box_voice.dialogue_node import DialogueNode


def _make_node() -> DialogueNode:
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
    n._music_player_state = MusicPlayerState(state="idle")
    n._llm_skipped_counter = new_llm_skip_counter()
    n._scheduler_executor = SimpleNamespace(music_turn=MusicTurn())
    n._turn_set_tracks = None
    return n


def _result(spoken: str, tools=()) -> DialogResult:
    r = DialogResult(spoken_text=spoken, tools_called=list(tools), finish_reason="stop")
    r.speak_text_real_count = 0
    r.speak_text_count = 0
    return r


def _said(n: DialogueNode) -> str:
    direct = [c.args[0] for c in n._speak_direct.call_args_list]
    published = [c.args[0].data for c in n._response_pub.publish.call_args_list]
    return " | ".join(direct + published)


@pytest.mark.parametrize("user", ["сыграй Баха на трамбоне", "сыграй Токкату ре минор", "сыграй К Элизе"])
def test_named_piece_request_is_a_piece_request(user):
    assert piece_requested(user)


@pytest.mark.parametrize("user", ["ну что, будет Бах?", "стоп музыка", "как дела", "включи диджей сет на тему космос"])
def test_other_replies_are_not_piece_requests(user):
    assert not piece_requested(user)


def test_turn_reply_piece_without_launch_is_code_phrase():
    assert turn_reply(MusicTurn(), requested=False, voiced=0, piece=True) == NOT_PLAYED_TEXT


def test_turn_reply_piece_launched_by_another_music_tool_is_left_alone():
    assert turn_reply(MusicTurn(), requested=False, voiced=0, piece=True, other_launch=True) is None


@pytest.mark.parametrize("user,spoken", [
    ("сыграй Баха на трамбоне", "Ставлю Баха."),
    ("сыграй Токкату ре минор", "Токкату ре минор Баха ставлю мелодией."),
    ("сыграй К Элизе", "Ставлю «К Элизе»."),
])
def test_node_piece_request_without_tool_never_voices_launch_claim(user, spoken):
    n = _make_node()
    n._handle_result(_result(spoken), user_input=user)
    said = _said(n)
    assert said == NOT_PLAYED_TEXT
    assert "тавлю" not in said and "ставлю" not in said


def test_node_set_request_without_tool_keeps_set_phrase():
    n = _make_node()
    n._handle_result(_result("Включаю диджей-сет. Тема — космос."), user_input="включи диджей сет на тему космос")
    assert _said(n) == NOT_LAUNCHED_TEXT


def test_node_piece_request_with_legacy_music_tool_is_not_overridden():
    n = _make_node()
    n._handle_result(_result("Поиграем.", tools=("gen_play_from_library",)), user_input="сыграй Токкату ре минор")
    assert NOT_PLAYED_TEXT not in _said(n)


def _record(done, phrase="Ставлю «К Элизе».", intent=MediaIntent.PLAY_NAMED):
    n = _make_node()
    n._core = MagicMock()
    n._record_media_turn(SimpleNamespace(command=SimpleNamespace(intent=intent)), "сыграй к элизе", phrase, list(done))
    return n


@pytest.mark.parametrize("done,intent", [
    (("lookup_melody", "request_music"), MediaIntent.PLAY_NAMED), (("dj_set",), MediaIntent.DJ),
    (("request_music",), MediaIntent.REQUEST_MUSIC),
])
def test_router_launch_phrase_is_not_put_into_model_history(done, intent):
    n = _record(done, intent=intent)
    n._core.record_external_turn.assert_not_called()
    assert [name for _at, name in n._media_actions] == list(done)  # гарды #2549/#2559 его по-прежнему видят


def test_router_stop_phrase_still_goes_into_model_history():
    n = _record(("dj_set", "stop_music"), phrase="Выключил музыку.", intent=MediaIntent.STOP)
    n._core.record_external_turn.assert_called_once_with("сыграй к элизе", "Выключил музыку.", ["dj_set", "stop_music"])
