"""Issue #3455 — фраза запуска музыки (шаблон кода) в ответе LLM без тула запуска → честная фраза.

Живой прогон 06.10 06:56Z: ``spoken='Включаю диджей-сет. Тема — интерстеллар.' tools=[]`` — дважды, музыки нет.
Модуль :mod:`rob_box_voice.core.media_phrases` — одна таблица шаблонов; сверка ответа — по их основам.
Уровень node — настоящий ``DialogueNode._handle_result`` (как ``test_issue_3165_live_state_answer``).
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.agent_core import DialogResult
from rob_box_voice.core.dialogue_guards import detect_phantom_action_claim, detect_universal_action_claim
from rob_box_voice.core.llm_skip_reasons import new_llm_skip_counter
from rob_box_voice.core.media_phrases import (
    LAUNCH_PHRASE_STEMS,
    NOT_LAUNCHED_TEXT,
    dj_started_text,
    honest_launch_reply,
    launch_claim,
    play_ok_text,
    request_ok_text,
)
from rob_box_voice.core.music_player_state import MusicPlayerState
from rob_box_voice.dialogue_node import DialogueNode

LIVE_SPOKEN = "Включаю диджей-сет. Тема — интерстеллар."
LIVE_INPUT = "[TG] Так что там с темой интерстеллар? почему не сылшно ничего"


def test_every_template_carries_a_stem_of_the_table():
    """Шаблон и его основа не расходятся: правка фразы ломает тест, а не проверку."""
    rendered = [dj_started_text("", "космос"), dj_started_text("диджей Вася", ""), request_ok_text(""),
                play_ok_text("К Элизе")]
    assert [launch_claim(t) for t in rendered] == list(LAUNCH_PHRASE_STEMS)


@pytest.mark.parametrize("spoken", [
    LIVE_SPOKEN, "Включаю диджей сет. Тема — космос.", "Ок! Я диджей Вася, включаю сет.", "Ставлю «К Элизе», погнали",
])
def test_launch_phrase_without_launch_tool_is_replaced(spoken):
    assert honest_launch_reply(spoken, ()) == NOT_LAUNCHED_TEXT
    assert honest_launch_reply(spoken, ("speak_text", "lookup_melody")) == NOT_LAUNCHED_TEXT


@pytest.mark.parametrize("tool", ["dj_set", "request_music"])
def test_launch_tool_in_the_turn_backs_the_phrase(tool):
    assert honest_launch_reply(LIVE_SPOKEN, (tool,)) == LIVE_SPOKEN


def test_other_replies_pass():
    for spoken in ("Интерстеллар — фильм Нолана.", "Сет уже играет, тема — космос.", ""):
        assert honest_launch_reply(spoken, ()) == spoken


def test_replacement_retracts_and_logs():
    log, retract = MagicMock(), MagicMock()
    honest_launch_reply(LIVE_SPOKEN, (), log=log, on_replace=retract)
    retract.assert_called_once_with()
    log.assert_called_once()


def test_old_guards_miss_present_tense_and_honest_text_does_not_trip_them():
    """Почему #2549/#2559 молчали 06.10: «включаю» нет в их словарях. Честная фраза их тоже не будит."""
    for spoken in (LIVE_SPOKEN, NOT_LAUNCHED_TEXT):
        assert detect_universal_action_claim(spoken=spoken, tools_called=()) is None
        assert not detect_phantom_action_claim(user_input="сыграй интерстеллар сэт", spoken=spoken, tools_called=())


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
    return n


def _said(node: DialogueNode) -> str:
    return " | ".join(c.args[0].data for c in node._response_pub.publish.call_args_list)


def test_node_llm_turn_without_tools_speaks_honest_phrase():
    n = _make_node()
    n._handle_result(DialogResult(spoken_text=LIVE_SPOKEN, tools_called=[], finish_reason="stop"),
                     user_input=LIVE_INPUT)
    said = _said(n)
    assert NOT_LAUNCHED_TEXT in said and "Включаю" not in said
    n._retract_rejected_reply.assert_called_once_with()
    n._dispatch_turn.assert_not_called()


def test_node_llm_turn_with_dj_set_keeps_its_phrase():
    n = _make_node()
    n._handle_result(DialogResult(spoken_text=LIVE_SPOKEN, tools_called=["dj_set"], finish_reason="stop"),
                     user_input=LIVE_INPUT)
    assert LIVE_SPOKEN in _said(n)
    n._retract_rejected_reply.assert_not_called()
