"""test_issue_3801_one_voice_one_person.py

#3801 (ADR-0150 PR-5d, решение Шифу 21.10): имя меняется фразой, один голос — одна личность, подтверждение шаблоном.

Живые случаи: «Денчик» STT записал как «Эйджик», поменять было трудно; короткое «да, это я» тем же голосом биометрия
не узнаёт (0.673 < 0.72, прогон 2 #3765) — склейка не проходила.

Ход идёт по-настоящему через ``_run_turn``; прелюдия хода — настоящие ``_note_self_intro`` и ``_register_self_intro``
(биометрия фразы — ``_current_speaker``), замоканы только LLM, гарды, DSM. Сценарии карточки:

(а) «меня зовут Эйджик» → «меня зовут не Эйджик, а Денчик» → профиль переименован, второй не заведён, 0 LLM;
(б) голос уверенно Саша, «я Боря» → регистрации нет, после ответа хода — фраза; «мы разные люди» → фраза, профиль один;
(в) короткое «да, это я» с неуверенным голосом → просьба шаблона → шаблон тем же голосом → склейка.
"""

from __future__ import annotations

import asyncio
import json
import sys
import threading
import time
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

from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog import phrases  # noqa: E402
from rob_box_voice.core import identity_question as iq  # noqa: E402
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402
from ros_stubs import make_speaker_result  # noqa: E402

SASHA_ID = "5d99f474-0000-0000-0000-000000000000"
BORIS_ID = "f65d3050-0000-0000-0000-000000000000"
EIDZHIK_ID = "e1d2e3f4-0000-0000-0000-000000000000"
REPLY = "Привет! Рад тебя слышать."
_FINISHED = 4

UNKNOWN = {"is_known": False}
SASHA_SURE = {"is_known": True, "speaker_id": SASHA_ID, "name": "Саша", "confidence": 0.93}
EIDZHIK_SURE = {"is_known": True, "speaker_id": EIDZHIK_ID, "name": "Эйджик", "confidence": 0.91}
SHORT_YES = {"is_known": False, "candidate_name": "Борис", "candidate_score": 0.673}
CONFLICT_ACK = {
    "event": "registered", "name": "Борис", "speaker_id": BORIS_ID, "reused_profile": False,
    "voice_conflict": {"name": "Саша", "speaker_id": SASHA_ID, "score": 0.907},
}


class _Result:
    spoken_text = REPLY
    tools_called = ()
    error = None


def _node(ack_during_llm=None):
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._task_lock = threading.Lock()
    n._run_task = None
    n._speaker_lock = threading.Lock()
    n._current_speaker = dict(UNKNOWN)
    n._speaker_merge_pub = MagicMock()
    n._speaker_rename_pub = MagicMock()
    n._speaker_register_pub = MagicMock()
    n.tts, n.llm_calls, n.hidden_tools = [], [], []

    def speak(text, language=None):
        n.tts.append(text)
        return f"speech-{len(n.tts)}"

    n._speak_direct = MagicMock(side_effect=speak)
    n._handle_result = MagicMock(side_effect=lambda result, **_kw: n.tts.append(result.spoken_text))

    async def invoke_llm(**kw):
        n.llm_calls.append(kw["user_input"])
        n.hidden_tools.append(tuple(getattr(n, "_turn_hidden_tools", ()) or ()))
        if ack_during_llm is not None and len(n.llm_calls) == 1:
            n._on_speaker_result(make_speaker_result(ack_during_llm))
        return _Result()

    async def prelude(**kw):
        # настоящая прелюдия по представлению (#2925) — биометрия фразы уже в _current_speaker
        uid = kw["utterance_id"]
        n._last_resolved_utterance_id = uid
        n._note_self_intro(kw["user_input"], uid)
        n._register_self_intro(uid)
        return kw["user_input"], "<system_context/>"

    n._invoke_llm_with_telemetry = invoke_llm
    n._prepare_user_input_context = AsyncMock(side_effect=prelude)
    n._reset_turn_retry_budgets = MagicMock()
    n._llm = MagicMock()
    n._retry_dispatched_in_turn = False
    n._finalize_turn_dsm = MagicMock()
    return n


_UID = iter(range(1, 10_000))


def _say(n, text, speaker, *, answer=False):
    """Фраза человека с биометрией ``speaker``; ``answer`` — без вейка в окне ответа (снимает кадр, как admission)."""
    if answer:
        heard = types.SimpleNamespace(t=time.time() + 3.0, doa_deg=None, source="respeaker", text=text)
        iq.take_answer(n, "identity_answer", heard)
    n._current_speaker = dict(speaker)
    asyncio.run(n._run_turn(text, utterance_id=f"utt-{next(_UID)}"))


def _spoken_question(n):
    """Фраза кода договорена (``SpeechEvent finished``) — окно ответа открыто."""
    iq.on_speech_event(n, _FINISHED, f"speech-{len(n.tts)}", "personality")
    return iq.question(n)


def _published(pub):
    return [json.loads(c.args[0].data) for c in pub.publish.call_args_list]


class TestRenameByPhrase:
    def test_stt_mistake_fixed_by_phrase_scenario_a(self):
        n = _node()
        _say(n, "Робот, меня зовут Эйджик", UNKNOWN)
        assert [r["name"] for r in _published(n._speaker_register_pub)] == ["Эйджик"]

        _say(n, "Робот, меня зовут не Эйджик, а Денчик", EIDZHIK_SURE)

        assert _published(n._speaker_rename_pub) == [{"speaker_id": EIDZHIK_ID, "new_name": "Денчик"}]
        assert len(_published(n._speaker_register_pub)) == 1, "второй профиль не заводится"
        assert n.tts[-1] == phrases.text("identity.renamed", name="Денчик")
        assert len(n.llm_calls) == 1, "смена имени — кодом, LLM не зовётся"
        assert n._current_speaker["name"] == "Денчик"

    def test_call_me_renames_sure_voice(self):
        n = _node()

        _say(n, "Робот, зови меня Боря", SASHA_SURE)

        assert _published(n._speaker_rename_pub) == [{"speaker_id": SASHA_ID, "new_name": "Боря"}]
        assert n.llm_calls == []

    def test_stranger_voice_does_not_rename_anyone(self):
        n = _node()

        _say(n, "Робот, зови меня Боря", {"is_known": False, "candidate_name": "Саша", "candidate_score": 0.4})

        assert _published(n._speaker_rename_pub) == []
        assert n.llm_calls == ["Робот, зови меня Боря"], "обычный ход"

    def test_unsure_voice_asks_template_then_renames(self):
        n = _node()
        _say(n, "Робот, зови меня Денчик", SHORT_YES)
        assert _published(n._speaker_rename_pub) == []
        assert phrases.text("identity.template", name="Денчик") in n.tts[-1]
        frame = _spoken_question(n)
        assert frame is not None and frame.payload["kind"] == kn.IDENTITY_VOICE_CHECK_KIND

        _say(n, "Меня зовут Денчик, и это мой голос, запомни его", EIDZHIK_SURE, answer=True)

        assert _published(n._speaker_rename_pub) == [{"speaker_id": EIDZHIK_ID, "new_name": "Денчик"}]
        assert _published(n._speaker_register_pub) == [], "шаблон — не представление нового человека"
        assert n.llm_calls == []


class TestOneVoiceOnePerson:
    def test_sure_voice_other_name_no_second_profile_scenario_b(self):
        n = _node()

        _say(n, "Робот, привет, я Боря", SASHA_SURE)

        assert _published(n._speaker_register_pub) == [], "второй профиль на тот же голос не заводится"
        assert n.hidden_tools == [("register_speaker",)], "LLM не переименовывает и не регистрирует сама"
        assert n.tts == [REPLY, phrases.text("identity.same_voice", known="Саша", name="Боря")], "фраза — после ответа"
        frame = _spoken_question(n)
        assert frame is not None and frame.payload["kind"] == kn.IDENTITY_SAME_VOICE_KIND

        _say(n, "мы разные люди", SASHA_SURE, answer=True)

        assert n.tts[-1] == phrases.text("identity.one_voice", name="Боря")
        assert _published(n._speaker_register_pub) == [] and _published(n._speaker_merge_pub) == []
        assert len(n.llm_calls) == 1, "ответ решает код"

    def test_call_me_in_window_renames(self):
        n = _node()
        _say(n, "Робот, привет, я Боря", SASHA_SURE)
        _spoken_question(n)

        _say(n, "зови меня Боря", SASHA_SURE, answer=True)

        assert _published(n._speaker_rename_pub) == [{"speaker_id": SASHA_ID, "new_name": "Боря"}]
        assert n.tts[-1] == phrases.text("identity.renamed", name="Боря")

    def test_same_name_registers_nothing_and_says_nothing(self):
        n = _node()

        _say(n, "Робот, привет, я Саша", SASHA_SURE)

        assert _published(n._speaker_register_pub) == []
        assert n.tts == [REPLY]


class TestTemplateConfirmsShortAnswer:
    def _asked(self):
        n = _node(ack_during_llm=CONFLICT_ACK)
        _say(n, "Робот, добрый вечер, меня зовут Борис", UNKNOWN)
        assert "Вы разные люди" in n.tts[-1]
        _spoken_question(n)
        return n

    def test_short_its_me_asks_template_then_merges_scenario_c(self):
        n = self._asked()

        _say(n, "да, это я", SHORT_YES, answer=True)

        assert _published(n._speaker_merge_pub) == []
        assert n.tts[-1] == phrases.text(
            "identity.say_template", template=phrases.text("identity.template", name="Саша"))
        frame = _spoken_question(n)
        assert frame is not None and frame.payload["kind"] == kn.IDENTITY_VOICE_CHECK_KIND
        assert frame.ttl_until - frame.payload["asked_at"] < kn.REPLY_WINDOW_S + 1, "то же окно ответа"

        _say(n, "Меня зовут Саша, и это мой голос, запомни его", SASHA_SURE, answer=True)

        assert _published(n._speaker_merge_pub) == [{"src_speaker_id": BORIS_ID, "dst_speaker_id": SASHA_ID}]
        assert n.tts[-1] == phrases.text("identity.merged", name="Саша")
        assert len(_published(n._speaker_register_pub)) == 1, "шаблон не регистрируется"
        assert len(n.llm_calls) == 1

    def test_template_in_other_voice_merges_nothing(self):
        n = self._asked()
        _say(n, "да, это я", SHORT_YES, answer=True)
        _spoken_question(n)

        _say(n, "Меня зовут Саша, и это мой голос, запомни его", SHORT_YES, answer=True)

        assert _published(n._speaker_merge_pub) == []
        assert n.tts[-1] == phrases.text("identity.voice_check_failed")
