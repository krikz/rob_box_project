"""Issue #4145 — эхо собственной речи робота не заводит и не растит профиль голоса.

28.10 08:58:17.402 (агитация из Telegram, speech_id=46fd3b27): ``speaker_id_node`` завёл из эха речи робота новый
безымянный профиль «Наблюдатель» 76b59285 (речь 4.98 с, безымянных 20/20, вытеснено 1) — раньше, чем STT
распознал в этой фразе слова робота (08:58:18.657). Решение «эхо» одно — у ``stt_node``
(``core/echo_window.Admission.echo``); биометрия его слушает (``EchoVerdicts``): фраза, звучавшая рядом с речью
робота, меняет профили только после вердикта, эхо — никогда, фраза человека — как раньше.

Нода собрана через ``object.__new__`` (без ROS2 и resemblyzer), как в тестах #3858; фраза идёт настоящим
``_on_speech_audio`` (utterance_id из PCM) в однопоточный пул. Боевая speakers.db не трогается: БД — во временном
каталоге теста.
"""

from __future__ import annotations

import collections
import sys
import threading
import types
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Deque, Optional, Tuple
from unittest.mock import MagicMock

import numpy as np
import pytest

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_core.msg_stub import install_dialog_msgs_stub  # noqa: E402
from rob_box_core.speaker_wire import speaker_result_to_payload  # noqa: E402
from rob_box_voice import speaker_id_node as sid_node  # noqa: E402
from rob_box_voice.core import echo_window  # noqa: E402
from rob_box_voice.core.utterance_id import compute_utterance_id  # noqa: E402
from rob_box_voice.utils.speaker_embeddings import SpeakerDatabase  # noqa: E402

M = install_dialog_msgs_stub()
UNNAMED_MAX = 3


class _Embed:
    def __init__(self, embedding, voiced_sec: float) -> None:
        self.embedding = embedding
        self.voiced_sec = voiced_sec
        self.raw_sec = voiced_sec


def _embedding(seed: int) -> np.ndarray:
    v = np.random.default_rng(seed).standard_normal(256).astype(np.float32)
    return v / np.linalg.norm(v)


class _FakePublisher:
    def __init__(self) -> None:
        self.messages: list[dict] = []

    def publish(self, msg) -> None:
        self.messages.append(speaker_result_to_payload(msg))


class Clock:
    def __init__(self) -> None:
        self.t = 2000.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture()
def node(tmp_path, monkeypatch):
    import rob_box_voice.utils.speaker_embeddings as se_mod

    monkeypatch.setattr(se_mod, "IDENTIFY_THRESHOLD", 0.72)
    monkeypatch.setattr(se_mod, "REGISTER_MATCH_THRESHOLD", 0.75)
    monkeypatch.setattr(se_mod, "GALLERY_WARMUP_SIZE", 5)
    monkeypatch.setattr(se_mod, "MIN_REGISTER_AUDIO_DURATION_SEC", 3.0)

    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._db = SpeakerDatabase(str(tmp_path / "speakers.db"))
    instance._speech_log = {}
    instance._speech_log_lock = threading.Lock()
    instance._sample_rate = 16000
    instance._recent_embeddings: Deque[
        Tuple[float, np.ndarray, float, Optional[str]]
    ] = collections.deque(maxlen=20)
    instance._MAX_EMBED_AGE_SEC = 30.0
    instance._pending_register_name = None
    instance._pending_register_lock = threading.Lock()
    instance._result_pub = _FakePublisher()
    logger = MagicMock()
    instance.get_logger = MagicMock(return_value=logger)
    instance._epithet_request_pub = None
    instance._epithet_llm_reasked = set()
    instance._growth_session_gap_sec = 30.0
    instance._growth_session = None
    instance._growth_owner_min_score = 0.65
    instance._name_confidence_min_gap = 0.15
    instance._name_confidence_band_high = 0.80
    instance._identify_min_voiced_sec = 1.0
    instance._unnamed_profiles_enabled = True
    instance._unnamed_profiles_max = UNNAMED_MAX
    instance._executor = ThreadPoolExecutor(max_workers=1)
    clock = Clock()
    instance._echo_verdicts = echo_window.EchoVerdicts(
        2.5, submit=instance._executor.submit, log=logger, clock=clock
    )
    instance.test_clock, instance.test_log = clock, logger
    yield instance
    instance._executor.shutdown(wait=True)
    instance._db.close()


def _robot_speaks(node) -> None:
    """Речь робота началась (``SpeechEvent started`` агитации); фраза доедет через 5,45 с."""
    node._echo_verdicts.journal.on_event(
        M.SpeechEvent(
            kind=M.SpeechEvent.KIND_STARTED, speech_id="46fd3b27", agent="telegram"
        )
    )
    node.test_clock.t += 5.45


def _hear(node, embedding: np.ndarray, voiced_sec: float, seed: int) -> str:
    """Фраза ReSpeaker через ``_on_speech_audio`` (биометрия досчитана); вернуть её utterance_id."""
    node._db.embed_audio_ex = lambda *_a, **_k: _Embed(embedding, voiced_sec)
    pcm = (
        (np.random.default_rng(seed).normal(0, 0.1, int(16000 * 5.38)) * 32767)
        .astype(np.int16)
        .tobytes()
    )
    node._on_speech_audio(types.SimpleNamespace(data=pcm))
    _drain(node)
    return compute_utterance_id(pcm)


def _drain(node) -> None:
    node._executor.submit(lambda: None).result(timeout=10)


def _verdict(node, utterance_id: str, echo: bool) -> None:
    node._echo_verdicts.on_verdict(utterance_id, echo)
    _drain(node)


def test_echo_of_robot_speech_creates_no_profile_and_no_result(node):
    """Сцена лога: незнакомый «голос» 4.98 с во время речи робота. До вердикта — ни профиля, ни результата; вердикт
    «эхо» — профиля нет и не будет, результат не публикуется (это не человек)."""
    _robot_speaks(node)
    uid = _hear(node, _embedding(76), voiced_sec=4.98, seed=1)
    assert node._db.list_speakers() == [] and node._result_pub.messages == []
    _verdict(node, uid, echo=True)
    node._echo_verdicts.expire()
    assert node._db.list_speakers() == []
    assert node._db.unnamed.count() == 0
    assert node._result_pub.messages == []
    lines = [c.args[0] for c in node.test_log.info.call_args_list]
    assert any(
        f"utterance_id={uid} — эхо речи робота, профиль не завожу и не обновляю" in line
        for line in lines
    )


def test_echo_with_a_full_table_evicts_nobody(node):
    """«безымянных 20/20, вытеснено 1»: эхо больше не вытесняет живого безымянного гостя."""
    for seed in range(UNNAMED_MAX):
        _hear(
            node, _embedding(100 + seed), voiced_sec=4.5, seed=100 + seed
        )  # робот молчит — как раньше
    before = sorted(p["id"] for p in node._db.list_speakers())
    assert len(before) == UNNAMED_MAX
    _robot_speaks(node)
    uid = _hear(node, _embedding(76), voiced_sec=4.98, seed=2)
    _verdict(node, uid, echo=True)
    assert sorted(p["id"] for p in node._db.list_speakers()) == before


def test_echo_does_not_grow_an_open_gallery(node):
    """Growth-сессия открыта (человек только что представился, робот ему отвечает): эхо ответа — не его голос."""
    node._apply_growth_session = MagicMock()
    _robot_speaks(node)
    uid = _hear(node, _embedding(76), voiced_sec=4.98, seed=3)
    _verdict(node, uid, echo=True)
    node._apply_growth_session.assert_not_called()


def test_human_phrase_near_robot_speech_gets_its_profile_after_the_verdict(node):
    """Человек перебил робота: фраза ушла в диалог (вердикт «человек») — профиль без имени и результат, как раньше."""
    _robot_speaks(node)
    uid = _hear(node, _embedding(42), voiced_sec=4.5, seed=4)
    assert node._result_pub.messages == []
    _verdict(node, uid, echo=False)
    (profile,) = node._db.list_speakers()
    (result,) = node._result_pub.messages
    assert (
        result["source"] == "enroll"
        and result["speaker_id"] == profile["id"]
        and result["utterance_id"] == uid
    )


def test_phrase_in_silence_gets_its_profile_at_once_as_before(node):
    """Робот молчит: биометрия ничего не ждёт — профиль и результат сразу, как до #4145."""
    uid = _hear(node, _embedding(43), voiced_sec=4.5, seed=5)
    (result,) = node._result_pub.messages
    assert result["source"] == "enroll" and result["utterance_id"] == uid
    assert len(node._db.list_speakers()) == 1
