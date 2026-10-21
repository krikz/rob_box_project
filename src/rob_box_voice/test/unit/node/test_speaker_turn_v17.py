"""В17 на ходе ``dialogue_node`` (ADR-0150 §7.5; #3695, решения Шифу 21.10).

Живой лог 20.10 (эпизод А): «робот покажи анимацию сон» → анимация, и сразу «Как тебя зовут?» вместо реплики;
«робот ещё раз покажи анимацию ускорения» → ни одного тула, снова «Как тебя зовут?»; эпизод Б: ``spoken=''`` →
третий «Как тебя зовут?». Контракт: на команду — выполнение без вопроса; вопрос копит признаки (стартово 3 фразы
или 60 с — ``knowledge``), звучит после ответа хода и один раз на собеседника.

Ход идёт через настоящий ``_run_turn`` → ``_apply_speaker_identity`` (``object.__new__``, без ROS2).
"""

from __future__ import annotations

import asyncio
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

from rob_box_dialog.speaker_adapter import FaceSightings  # noqa: E402
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402

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
    n._identity_confirmations = {}
    n._pending_identity_hint = None
    n._identity_question_session_gap_sec = 120.0
    n._identity_answer_window_sec = 180.0
    n._faces = FaceSightings()
    n.tts = []
    n.reply = _Result("Хорошо.")
    n._speak_direct = MagicMock(side_effect=lambda text, language=None: n.tts.append(text))
    n._handle_result = MagicMock(side_effect=lambda result, **_kw: n.tts.append(result.spoken_text))

    async def prepare(**kw):
        user_input = await n._apply_speaker_identity(kw["user_input"], None, kw.get("utterance_id"))
        return user_input, "\n".join(n._pending_identity_hint_lines())

    async def invoke_llm(**_kw):
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
    assert n._identity_confirmations["sid-sasha"]["asked"] is True
