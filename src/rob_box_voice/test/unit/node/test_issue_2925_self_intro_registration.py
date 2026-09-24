"""test_issue_2925_self_intro_registration.py

Issue #2925 — регистрация диктора и вопрос о личности расходятся с тем,
что человек сказал о себе в реплике. Последовательности — из issue:

1. n708 попытка 1 (run 35931295672): незнакомый голос, «привет я борис
   заглянул проверить проводку…» — LLM здоровается «Здравствуй, Борис!»,
   ``register_speaker`` не вызван → голос не записан;
2. n708 попытка 2: тот же голос похож на Сашу (``single``, 0.729) —
   робот спрашивает «Саша, это ты?», хотя человек назвался Борисом;
3. акт 2 (run 35933157262): «Робот, запомни про меня: Дарья болит за
   Спартак…», голос уверенно Борис 0.796 — LLM зовёт
   ``register_speaker('Дарья')``. Здесь проверяется сторона dialogue_node:
   в контекст хода для тула уходит «кто это по голосу» и «представлялся
   ли»; сам отказ тула — в
   ``rob_box_mcp_tools/test/test_issue_2925_register_speaker_gate.py``.

Ход идёт через настоящие ``_run_turn`` → ``_prepare_user_input_context``
→ ``_apply_speaker_identity``; замоканы LLM, гарды, DSM, TTS. Ack
speaker_id_node моделируется колбэком на публикацию в
``/voice/speaker/register`` (как на роботе: приходит, пока ход идёт).
"""

from __future__ import annotations

import asyncio
import json
import sys
import threading
import types
from pathlib import Path
from unittest.mock import MagicMock

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402

SASHA_ID = "d8844740-0000-0000-0000-000000000000"
BORIS_ID = "4de3d7cc-0000-0000-0000-000000000000"
NEW_ID = "bb6c8d24-0000-0000-0000-000000000000"

# Дословно из issue #2925.
N708_TEXT = "привет я борис заглянул проверить проводку"
N708_LLM = "Здравствуй, Борис! Потрескивание в щитке — это серьёзно."
DARYA_TEXT = (
    "Робот, запомни про меня: Дарья болит за Спартак и всегда "
    "приносит пиццу."
)


def _unknown():
    return {"is_known": False}


def _tentative_sasha():
    return {
        "is_known": True,
        "speaker_id": SASHA_ID,
        "name": None,
        "tentative_name": "Саша",
        "tentative_conf": 0.729,
        "tentative_kind": "single",
        "confidence": 0.729,
    }


def _known(name, sid, conf):
    return {"is_known": True, "speaker_id": sid, "name": name,
            "confidence": conf}


class _Result:
    def __init__(self, text):
        self.spoken_text = text
        self.tools_called = ()
        self.error = None


class _Biometry:
    def __init__(self):
        self.result = _unknown()

    async def resolve(self, utterance_id, timeout):
        return dict(self.result)


def _node():
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._task_lock = threading.Lock()
    n._run_task = None
    n._speaker_lock = threading.Lock()
    n._current_speaker = _unknown()
    n._speaker_id_enabled = True
    n._utterance_speaker = _Biometry()
    n._speaker_resolve_timeout_sec = 1.0
    n._last_resolved_utterance_id = None
    n._publish_speaker_observation = MagicMock()
    n.registered = []
    n.ack = None  # что «ответит» speaker_id_node на запрос регистрации

    def _publish_register(msg):
        payload = json.loads(msg.data)
        n.registered.append(payload)
        if n.ack is not None:
            ack = dict(n.ack, name=payload["name"],
                       utterance_id=payload.get("utterance_id"))
            n._on_speaker_result(types.SimpleNamespace(
                data=json.dumps(ack, ensure_ascii=False)))

    n._speaker_register_pub = MagicMock()
    n._speaker_register_pub.publish.side_effect = _publish_register
    n._speaker_merge_pub = MagicMock()
    n._identity_confirmations = {}
    n._pending_identity_hint = None
    n._identity_question_session_gap_sec = 120.0
    n._identity_answer_window_sec = 180.0
    n.tts = []
    n.llm_reply = N708_LLM
    n.turn_context = []
    n._speak_direct = MagicMock(
        side_effect=lambda text, language=None: n.tts.append(text)
    )
    n._handle_result = MagicMock(
        side_effect=lambda result, **_kw: n.tts.append(result.spoken_text)
    )
    n._build_dynamic_system_context = lambda: "\n".join(
        n._pending_identity_hint_lines()
    )

    async def invoke_llm(**_kw):
        # Контекст хода, который получил бы register_speaker (#2842).
        n.turn_context.append(n._mcp_turn_context())
        return _Result(n.llm_reply)

    n._invoke_llm_with_telemetry = invoke_llm
    n._reset_turn_retry_budgets = MagicMock()
    n._llm = MagicMock()
    n._retry_dispatched_in_turn = False
    n._drain_pending_user_messages = MagicMock(return_value=False)
    n._finalize_music_cleanup_policy = MagicMock()
    n._apply_music_guard = MagicMock(return_value=False)
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._finalize_turn_dsm = MagicMock()
    return n


def _turn(n, text, speaker, utterance_id):
    n._utterance_speaker.result = speaker
    asyncio.run(n._run_turn(text, utterance_id=utterance_id))


def _info_lines(n):
    return [str(c.args[0]) for c in n.get_logger().info.call_args_list]


# ─────────────────────────────────────────────────────────────────────────
#  Случай 1 — представление незнакомым голосом регистрируется без LLM-тула
# ─────────────────────────────────────────────────────────────────────────


class TestCase1IntroRegistersWithoutTool:
    def test_n708_attempt1_intro_registers_voice(self):
        n = _node()
        n.ack = {"event": "registered", "speaker_id": NEW_ID,
                 "reused_profile": False}

        _turn(n, N708_TEXT, _unknown(), "utt-708a")

        assert n.registered == [{"name": "Борис", "utterance_id": "utt-708a"}]
        # Обычная регистрация без конфликта — ответ LLM звучит как был.
        assert n.tts == [N708_LLM]

    def test_llm_tool_in_same_turn_is_told_registration_already_sent(self):
        n = _node()
        _turn(n, N708_TEXT, _unknown(), "utt-708a")
        ctx = n.turn_context[-1]
        assert ctx["utterance_id"] == "utt-708a"
        assert ctx["self_intro_name"] == "Борис"
        assert ctx["intro_registered"] is True

    def test_speech_gate_waits_for_ack_of_robot_registration(self):
        """#2913: речь хода ждёт исход и авто-регистрации тоже."""
        n = _node()  # ack не приходит
        _turn(n, N708_TEXT, _unknown(), "utt-708a")
        assert n._turn_speech_gate().ticket().pending == 1

    def test_already_known_under_same_name_is_not_reregistered(self):
        """#2863: «я Борис» от уверенно узнанного Бориса — не регистрация."""
        n = _node()
        _turn(n, N708_TEXT, _known("Борис", BORIS_ID, 0.91), "utt-1")
        assert n.registered == []

    def test_turn_without_utterance_does_not_register(self):
        """Telegram/синтетический ход — фразы с голосом нет (ADR-0131)."""
        n = _node()
        _turn(n, N708_TEXT, _unknown(), None)
        assert n.registered == []

    def test_plain_phrase_does_not_register(self):
        n = _node()
        n.llm_reply = "Привет!"
        _turn(n, "робот привет как дела", _unknown(), "utt-2")
        assert n.registered == []


# ─────────────────────────────────────────────────────────────────────────
#  Случай 2 — «Саша, это ты?» не звучит, когда человек назвался Борисом
# ─────────────────────────────────────────────────────────────────────────


class TestCase2NoHypothesisQuestionOnOtherName:
    def test_n708_attempt2_no_sasha_question(self):
        n = _node()
        n.ack = {"event": "registered", "speaker_id": NEW_ID,
                 "reused_profile": False}

        _turn(n, N708_TEXT, _tentative_sasha(), "utt-708b")

        assert "Саша, это ты?" not in n.tts, n.tts
        assert n.registered == [{"name": "Борис", "utterance_id": "utt-708b"}]
        assert n.tts == [N708_LLM]
        assert n._identity_confirmations[SASHA_ID]["asked"] is False

    def test_similar_voice_goes_to_2828_conflict_question(self):
        """Голос выше порога слияния — ADR-0127 заводит отдельный профиль,
        ack с voice_conflict → вопрос #2828 заменяет ответ хода."""
        n = _node()
        n.ack = {"event": "registered", "speaker_id": NEW_ID,
                 "reused_profile": False,
                 "voice_conflict": {"name": "Саша", "speaker_id": SASHA_ID,
                                    "score": 0.78}}

        _turn(n, N708_TEXT, _tentative_sasha(), "utt-708b")

        assert len(n.tts) == 1, n.tts
        assert "Саша, это ты?" not in n.tts
        assert "Вы разные люди" in n.tts[0]
        assert N708_LLM not in n.tts

    def test_same_name_intro_confirms_hypothesis_without_question(self):
        """«я Саша» при гипотезе «Саша» — подтверждение (#2809), вопрос
        не нужен, новой регистрации нет — только рост галереи."""
        n = _node()
        n.llm_reply = "Привет, Саша!"

        _turn(n, "привет я саша", _tentative_sasha(), "utt-3")

        assert n.tts == ["Привет, Саша!"]
        assert n._current_speaker["name"] == "Саша"
        assert n.registered == [{"name": "Саша", "speaker_id": SASHA_ID,
                                 "purpose": "growth",
                                 "utterance_id": "utt-3"}]

    def test_no_intro_still_asks_2888(self):
        """#2888 не сломан: без представления вопрос звучит."""
        n = _node()
        _turn(n, "робот ты меня узнал", _tentative_sasha(), "utt-4")
        assert n.tts == ["Саша, это ты?"]
        assert n.registered == []


# ─────────────────────────────────────────────────────────────────────────
#  Случай 3 — контекст хода для register_speaker на «запомни про меня»
# ─────────────────────────────────────────────────────────────────────────


class TestCase3TurnContextForRegisterGate:
    def test_darya_phrase_carries_confident_boris_and_no_intro(self):
        n = _node()
        n.llm_reply = "Запомнил."

        _turn(n, DARYA_TEXT, _known("Борис", BORIS_ID, 0.796), "ed597ace38e7")

        ctx = n.turn_context[-1]
        assert ctx["utterance_id"] == "ed597ace38e7"
        assert ctx["known_speaker_name"] == "Борис"
        assert ctx["self_intro_name"] is None
        assert ctx["intro_registered"] is False
        assert n.registered == []

    def test_tentative_voice_is_not_confident(self):
        n = _node()
        _turn(n, DARYA_TEXT, _tentative_sasha(), "utt-5")
        assert n.turn_context[-1]["known_speaker_name"] is None

    def test_turn_without_utterance_has_no_known_speaker(self):
        """Прошлый диктор не выдаётся за диктора хода без реплики."""
        n = _node()
        n._current_speaker = _known("Борис", BORIS_ID, 0.9)
        _turn(n, DARYA_TEXT, _known("Борис", BORIS_ID, 0.9), None)
        assert n.turn_context[-1]["known_speaker_name"] is None
