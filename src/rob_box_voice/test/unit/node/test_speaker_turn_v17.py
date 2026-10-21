"""В17 на ходе ``dialogue_node`` (ADR-0150 §7.5; #3695, решения Шифу 21.10).

Живой лог 20.10 (эпизод А): «робот покажи анимацию сон» → анимация, и сразу «Как тебя зовут?» вместо реплики;
«робот ещё раз покажи анимацию ускорения» → ни одного тула, снова «Как тебя зовут?»; эпизод Б: ``spoken=''`` →
третий «Как тебя зовут?». Контракт: на команду — выполнение без вопроса; вопрос копит признаки (стартово 3 фразы
или 60 с — ``knowledge``), звучит после ответа хода и один раз на собеседника.

Ход идёт через настоящий ``_run_turn`` → ``_apply_speaker_identity`` (``object.__new__``, без ROS2).
"""

from __future__ import annotations

import asyncio
import json
import sys
import threading
import time
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)
for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_dialog.speaker_adapter import FaceSightings  # noqa: E402
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402
from rob_box_voice.core.speaker_turn import speaker_section, tentative_states  # noqa: E402

ASK_NAME = "Как тебя зовут?"


class _Result:
    def __init__(self, text, tools=()):
        self.spoken_text = text
        self.tools_called = tuple(tools)
        self.error = None


class _Biometry:
    """Биометрия по utterance_id: ``result`` задаёт тест (по умолчанию — незнакомый голос этой фразы)."""

    def __init__(self):
        self.result = None

    async def resolve(self, utterance_id, timeout):
        return dict(self.result or {"is_known": False, "utterance_id": utterance_id})


def _node():
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._task_lock = threading.Lock()
    n._run_task = None
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": False}
    n._utterance_speaker = _Biometry()
    n._speaker_resolve_timeout_sec = 1.0
    n._last_resolved_utterance_id = None
    n._publish_speaker_observation = MagicMock()
    n._speaker_register_pub = MagicMock()
    n._speaker_merge_pub = MagicMock()
    n._identity_question_session_gap_sec = 120.0
    n._identity_answer_window_sec = 180.0
    n._faces = FaceSightings()
    n.tts = []
    n.llm_inputs = []
    n.reply = _Result("Хорошо.")
    n._speak_direct = MagicMock(side_effect=lambda text, language=None: (n.tts.append(text), f"sid-{len(n.tts)}")[1])
    n._handle_result = MagicMock(side_effect=lambda result, **_kw: n.tts.append(result.spoken_text))

    async def prepare(**kw):
        user_input = await n._apply_speaker_identity(kw["user_input"], None, kw.get("utterance_id"))
        return user_input, ""

    async def invoke_llm(**kw):
        n.llm_inputs.append(kw.get("user_input"))
        return n.reply

    n._prepare_user_input_context = prepare
    n._invoke_llm_with_telemetry = invoke_llm
    n._reset_turn_retry_budgets = MagicMock()
    n._llm = MagicMock()
    n._retry_dispatched_in_turn = False
    n._finalize_music_cleanup_policy = MagicMock()
    n._apply_tool_skipped_guard = MagicMock(return_value=False)
    n._finalize_turn_dsm = MagicMock()
    return n


_seq = iter(range(10 ** 6))


def _turn(n, text, reply=None):
    if reply is not None:
        n.reply = reply
    asyncio.run(n._run_turn(text, utterance_id=f"utt-{next(_seq)}"))


def _verdicts(n):
    return [str(c.args[0]) for c in n.get_logger().info.call_args_list if "[В17 #3695]" in str(c.args[0])]


def test_command_from_stranger_runs_without_question_and_question_waits_for_third_phrase():
    """Сценарий (а) робота под В17: «покажи анимацию сон» → исполнение и реплика, вопроса нет."""
    n = _node()
    _turn(n, "покажи анимацию сон", _Result("Показываю анимацию «сон».", ["play_animation", "speak_text"]))
    assert n.tts == ["Показываю анимацию «сон»."]
    _turn(n, "покажи анимацию ускорения", _Result("Показываю.", ["play_animation"]))
    assert ASK_NAME not in n.tts
    _turn(n, "а ты кто вообще", _Result("Я Роббокс."))
    assert n.tts[-2:] == ["Я Роббокс.", ASK_NAME], n.tts
    assert [v.split(": ")[1].split(" ")[0] for v in _verdicts(n)] == ["command", "command", "ask"]


def test_question_only_once_per_speaker():
    n = _node()
    for i in range(6):
        _turn(n, f"фраза {i}", _Result("Ок."))
    assert n.tts.count(ASK_NAME) == 1


def test_empty_answer_is_not_answered_with_name_question():
    """Эпизод Б 20.10: ``spoken=''`` после 8 итераций LLM — вопроса об имени нет."""
    n = _node()
    for i in range(3):
        _turn(n, f"включай {i}", _Result(""))
    assert ASK_NAME not in n.tts


def test_named_face_in_frame_means_no_name_question():
    """Голос незнаком, но в кадре лицо с именем — собеседник опознан лицом, спрашивать нечего."""
    import time

    n = _node()
    for i in range(4):
        n._faces.note("f1", "Литвин", time.time())
        _turn(n, f"фраза {i}", _Result("Ок."))
    assert ASK_NAME not in n.tts


def test_tentative_voice_asks_confirm_after_accumulation_not_in_first_turn():
    n = _node()
    n._utterance_speaker.result = {"is_known": True, "speaker_id": "sid-sasha", "name": None,
                                   "tentative_name": "Саша", "tentative_kind": "single", "confidence": 0.75}
    _turn(n, "ты меня узнал", _Result("Привет."))
    assert n.tts == ["Привет."]
    _turn(n, "как дела", _Result("Нормально."))
    _turn(n, "что нового", _Result("Всё по-старому."))
    assert n.tts[-1] == "Саша, это ты?"
    assert tentative_states(n).get("sid-sasha")["asked"] is True


def test_live_robot_21_10_chat_with_reads_and_llm_animation_asks_once_after_third_phrase():
    """Живой FAIL 21.10 08:10Z: болтовня, где LLM сама зовёт чтение или анимацию, — не команда. Ровно один вопрос
    после 3-й фразы; просьба «покажи анимацию сон» — команда, без вопроса; повтор — без вопроса."""
    n = _node()
    _turn(n, "робот как у тебя дела", _Result("Всё отлично.", ["get_robot_status", "speak_text"]))
    _turn(n, "робот расскажи что нибудь интересное про луну", _Result("Луна уходит.", ["speak_text", "play_animation"]))
    assert ASK_NAME not in n.tts
    _turn(n, "робот который час", _Result("Восемь утра.", ["get_current_time", "speak_text"]))
    assert n.tts[-2:] == ["Восемь утра.", ASK_NAME], n.tts
    _turn(n, "робот покажи анимацию сон", _Result("Показываю.", ["play_animation", "speak_text"]))
    _turn(n, "робот как у тебя дела", _Result("Хорошо.", ["get_robot_status", "speak_text"]))
    assert n.tts.count(ASK_NAME) == 1
    verdicts = [v.split(": ")[1].split(" ")[0] for v in _verdicts(n)]
    assert verdicts == ["accumulating", "accumulating", "ask", "command", "asked_already"], verdicts
    assert "command_by=animation" in _verdicts(n)[3]


def test_command_first_then_repeat_never_asks():
    n = _node()
    for _ in range(4):
        _turn(n, "робот покажи анимацию сон", _Result("Показываю.", ["play_animation", "speak_text"]))
    assert ASK_NAME not in n.tts
    assert all("command command_by=animation" in v for v in _verdicts(n))


# -- кто говорит: структурой, а не префиксом в тексте (ADR-0150 §7.3, A30; #3695 ч.2b) ---------------------


def test_llm_gets_the_words_of_the_person_without_speaker_prefixes():
    n = _node()
    _turn(n, "робот как дела")
    n._utterance_speaker.result = {"is_known": True, "speaker_id": "sid-a", "name": "Антон", "confidence": 0.93}
    _turn(n, "а ты меня помнишь")
    n._utterance_speaker.result = {"is_known": True, "speaker_id": "sid-l", "name": None,
                                   "tentative_name": "Литвин", "tentative_kind": "single", "confidence": 0.75}
    _turn(n, "привет ещё раз")
    assert n.llm_inputs == ["робот как дела", "а ты меня помнишь", "привет ещё раз"]


@pytest.mark.parametrize("speaker, block", [
    ({"is_known": False}, {"name": None, "name_state": "unknown"}),
    ({"is_known": True, "speaker_id": "s", "name": "Саша", "confidence": 0.9},
     {"name": "Саша", "name_state": "confirmed"}),
    ({"is_known": True, "speaker_id": "s", "name": None, "tentative_name": "Литвин", "tentative_kind": "single"},
     {"name": None, "name_state": "tentative"}),
    ({"is_known": True, "speaker_id": "s", "name": "Null", "confidence": 0.9}, {"name": None, "name_state": "unknown"}),
])
def test_snapshot_speaker_section_names_only_confirmed(speaker, block):
    """Эпизод Б 20.10: гипотеза «Литвин» к Дэнчику — имени в снимке нет, пока оно не подтверждено (A30)."""
    n = _node()
    n._current_speaker = speaker
    assert speaker_section(n) == block


def test_verbal_yes_makes_the_name_confirmed_in_the_snapshot():
    n = _node()
    n._current_speaker = {"is_known": True, "speaker_id": "sid-sasha", "name": None, "tentative_name": "Саша",
                          "tentative_kind": "single", "confidence": 0.75}
    tentative_states(n).fresh("sid-sasha", time.monotonic())["asked"] = True
    n._confirm_tentative_speaker("sid-sasha", "Саша", "да, это я", "utt-x")
    assert speaker_section(n) == {"name": "Саша", "name_state": "confirmed"}


def test_meeting_marker_no_longer_makes_the_robot_speak_first():
    """В12′ (Шифу 21.10): робот сам в разговор не вступает — приветствие Встречи лицом удалено."""
    n = _node()
    n._face_voice_hint_enabled = False
    n._publish_response = MagicMock()
    marker = json.dumps({"encounter": "start", "person_id": "p1", "name": "Денис", "is_new": False,
                         "similarity": 0.9, "encounter_count": 2}, ensure_ascii=False)
    n._on_vision_event(types.SimpleNamespace(event_type="face", attributes_json=marker, source_camera="main_camera",
                                             embedding_id="p1", display_name="Денис"))
    n._publish_response.assert_not_called()
    assert not hasattr(DialogueNode, "_handle_meeting")


def test_question_opens_identity_frame_facing_the_speaker():
    """#3695 ч.2c: прозвучавший вопрос — кадр Identity с азимутом фразы собеседника (ответ ждётся оттуда ≤ 15 с)."""
    from rob_box_dialog.backlog import Heard

    n = _node()
    for i in range(3):
        utterance_id = f"utt-face-{i}"
        n._last_heard = Heard(text="x", t=0.0, doa_deg=75.0, utterance_id=utterance_id)
        n.reply = _Result("Ок.")
        asyncio.run(n._run_turn(f"фраза {i}", utterance_id=utterance_id))
    assert n.tts[-1] == ASK_NAME
    frame = n._session.snapshot.top("Identity")
    assert (frame.payload["kind"], frame.payload["azimuth_deg"]) == ("unknown", 75.0)
    assert frame.payload["asked_at"] is None, "вопрос ещё звучит — кадр ждёт его конца (#3771)"
    from rob_box_voice.core.identity_question import on_speech_event, question

    assert question(n) is None
    on_speech_event(n, 4, frame.payload["speech_id"], "personality")  # SpeechEvent finished вопроса
    assert question(n) is not None
    assert n._session.snapshot.speaker.azimuth_deg == 75.0
