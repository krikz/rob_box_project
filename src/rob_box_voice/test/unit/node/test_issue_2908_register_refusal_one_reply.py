"""test_issue_2908_register_refusal_one_reply.py

Issue #2908 — при отказе регистрации робот в одном ходе говорил две
противоречащие реплики.

Живой лог (E2E акт 2c, run 35916899022, n722 попытка 1)::

    [mcp_server]      📥 Запрос выполнения: register_speaker … {'name': 'Борис', 'utterance_id': '5505eca40cea'}
    [speaker_id_node] ⚠️ [issue #2769] Registration of 'Борис' rejected — … 2.97s < 3.0s required
    [dialogue_node]   ⚠️ [issue #2769] Регистрация 'Борис' отклонена — реплика 2.97с короче требуемых 3.0с
    [tts_node]        🔊 TTS: text='Не расслышал — скажи, пожалуйста, ещё пару слов, …'
    [dialogue_node]   ✅ [turn] process_input returned: spoken='Добрый вечер, Борис! Рад знакомству. …'
    [tts_node]        🔊 TTS: text='Добрый вечер, Борис! Рад знакомству. Чем занимаешься?'

Контракт после фикса (механизм «придержать и заменить» #2828/#2888):

* отказ посреди хода → в TTS ровно одна реплика — просьба повторить,
  приветствие LLM не звучит;
* отказ после выдачи ответа хода / вне хода → после ответа звучит
  согласованное «голос запомнить не успел», без «не расслышал» поверх
  приветствия по имени;
* просьба повторить — не вопрос о склейке: следующая реплика не читается
  как ответ на переспрос.

Ход проходит по-настоящему через ``_run_turn`` (тот же стенд, что в
test_issue_2828_identity_question_collision.py).
"""

from __future__ import annotations

import asyncio
import json
import sys
import threading
import types
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402

GREETING = "Добрый вечер, Борис! Рад знакомству. Чем занимаешься?"
NOT_HEARD = "Не расслышал"
REFUSAL = {
    "event": "register_error",
    "error": "too_short",
    "name": "Борис",
    "duration_s": 2.97,
    "min_required_s": 3.0,
    "utterance_id": "5505eca40cea",
}
INTRO = "добрый вечер, меня зовут Борис"


def _msg(payload: dict):
    return type("Msg", (), {"data": json.dumps(payload, ensure_ascii=False)})()


class _Result:
    spoken_text = GREETING
    tools_called = ("register_speaker",)
    error = None


def _turn_node(ack_during_llm=None, ack_after_result=None):
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._task_lock = threading.Lock()
    n._run_task = None
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": False}
    n._speaker_merge_pub = MagicMock()
    n.tts = []
    n._speak_direct = MagicMock(
        side_effect=lambda text, language=None: n.tts.append(text)
    )

    def handle_result(result, **_kw):
        n.tts.append(result.spoken_text)
        if ack_after_result is not None:
            n._on_speaker_result(_msg(ack_after_result))

    n._handle_result = MagicMock(side_effect=handle_result)

    async def invoke_llm(**_kw):
        # register_speaker вернул 'pending', отказ прилетает, пока LLM
        # дописывает приветствие — тайминг run 35916899022.
        if ack_during_llm is not None:
            n._on_speaker_result(_msg(ack_during_llm))
        return _Result()

    n._invoke_llm_with_telemetry = invoke_llm
    n._reset_turn_retry_budgets = MagicMock()
    n._prepare_user_input_context = AsyncMock(
        side_effect=lambda **kw: (kw["user_input"], "<system_context/>")
    )
    n._llm = MagicMock()
    n._retry_dispatched_in_turn = False
    n._drain_pending_user_messages = MagicMock(return_value=False)
    n._finalize_music_cleanup_policy = MagicMock()
    n._apply_music_guard = MagicMock(return_value=False)
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._finalize_turn_dsm = MagicMock()
    return n


def _greets_by_name(text: str) -> bool:
    return "Борис" in text and "знакомству" in text


def test_refusal_mid_turn_only_one_reply():
    """Регресс n722: «Не расслышал» и «Рад знакомству, Борис» не звучат подряд."""
    n = _turn_node(ack_during_llm=REFUSAL)

    asyncio.run(n._run_turn(INTRO))

    assert len(n.tts) == 1, f"в TTS ушло {len(n.tts)} реплик: {n.tts!r}"
    assert NOT_HEARD in n.tts[0]
    assert not any(_greets_by_name(t) for t in n.tts), n.tts


def test_refusal_after_reply_is_consistent_follow_up():
    """Отказ опоздал к выдаче ответа: приветствие уже ушло — после него
    звучит согласованная просьба, без «не расслышал»."""
    n = _turn_node(ack_after_result=REFUSAL)

    asyncio.run(n._run_turn(INTRO))

    assert n.tts[0] == GREETING
    assert len(n.tts) == 2, n.tts
    assert NOT_HEARD not in n.tts[1]
    assert "голос" in n.tts[1] and "пару слов" in n.tts[1]


def test_refusal_outside_turn_does_not_say_not_heard_over_greeting():
    """Ход уже закончился (ответ прозвучал) — та же согласованная форма."""
    n = _turn_node()

    n._on_speaker_result(_msg(REFUSAL))

    assert len(n.tts) == 1
    assert NOT_HEARD not in n.tts[0]
    assert "пару слов" in n.tts[0]


def test_register_retry_is_not_an_identity_question():
    """Просьба повторить не взводит ожидание ответа на переспрос #2828:
    следующее «да» не читается как согласие на склейку."""
    n = _turn_node(ack_during_llm=REFUSAL)
    asyncio.run(n._run_turn(INTRO))

    assert n._identity_ack_state().consume("да, это я") is None
    n._speaker_merge_pub.publish.assert_not_called()


def test_plain_turn_without_refusal_keeps_reply():
    n = _turn_node()

    asyncio.run(n._run_turn(INTRO))

    assert n.tts == [GREETING]
