#!/usr/bin/env python3
"""
test_issue_2906_confirmed_growth_not_registration.py

Issue #2906 — после словесного «да, это я» робот говорил «Не расслышал».

Живой лог (E2E акт 2b, run 35913592795):

    [dialogue_node]   🌱 [issue #2809/#2757] запрошен рост галереи после
                      словесного подтверждения: name='Саша' speaker_id=4e9bd5aa
    [speaker_id_node] 📝 [issue #2829] Registering 'Саша' from utterance …
    [speaker_id_node] ⚠️ [issue #2769] Registration of 'Саша' rejected —
                      audio too short for a reliable anchor: 1.65s < 3.0s
    [tts_node]        🔊 TTS: text='Не расслышал — скажи, пожалуйста, …'

Причина: служебный рост галереи шёл тем же путём, что регистрация по
просьбе LLM (``_do_register`` → ``register_or_merge`` → гейт длины →
``register_error: too_short`` → dialogue_node озвучивает отказ).

Контракт после фикса: dialogue_node помечает запрос ``purpose: "growth"``;
speaker_id_node ведёт его через ``_do_confirmed_growth``:
  * короткая фраза — не пишется, только лог; на /voice/speaker/result
    ничего (ни register_error, ни registered);
  * длинная фраза владельца — дописывается в ЕГО галерею по правилам роста
    владельца (#2833: top-1 владелец, growth_owner_min_score, отрыв от
    конкурента), новый профиль не создаётся;
  * регистрация по просьбе LLM (без ``purpose``) по-прежнему честно
    отказывает too_short, и «Не расслышал» звучит.

Тесты сквозные на двух нодах без ROS2 (``object.__new__``): то, что
dialogue_node реально публикует, подаётся в speaker_id_node, а то, что
публикует speaker_id_node, — обратно в dialogue_node._on_speaker_result.
"""

from __future__ import annotations

import asyncio
import collections
import json
import sys
import threading
import time
import types
from pathlib import Path
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

from rob_box_voice import speaker_id_node as sid_node  # noqa: E402
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402
from rob_box_voice.utils.speaker_embeddings import SpeakerDatabase  # noqa: E402

UTT_ID = "c4a4d9c4772c"
NOT_HEARD = "Не расслышал"


def _embedding(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(256).astype(np.float32)
    return v / np.linalg.norm(v)


def _voice_at(base: np.ndarray, score: float, noise_seed: int) -> np.ndarray:
    """Эмбеддинг с cosine≈``score`` к ``base`` (шум почти ортогонален)."""
    alpha = float((1.0 / score ** 2 - 1.0) ** 0.5)
    v = base + alpha * _embedding(noise_seed)
    return v / np.linalg.norm(v)


class _FakePublisher:
    def __init__(self) -> None:
        self.messages: list[dict] = []

    def publish(self, msg) -> None:
        self.messages.append(json.loads(msg.data))


class _SyncExecutor:
    """Выполняет задачу сразу — тест детерминирован без ожиданий."""

    def submit(self, fn, *args, **kwargs):
        fn(*args, **kwargs)

    def shutdown(self, wait: bool = True) -> None:
        pass


@pytest.fixture()
def speaker_node(tmp_path, monkeypatch):
    import rob_box_voice.utils.speaker_embeddings as se_mod

    monkeypatch.setattr(se_mod, "IDENTIFY_THRESHOLD", 0.72)
    monkeypatch.setattr(se_mod, "REGISTER_MATCH_THRESHOLD", 0.75)
    monkeypatch.setattr(se_mod, "GALLERY_WARMUP_SIZE", 5)
    monkeypatch.setattr(se_mod, "MIN_REGISTER_AUDIO_DURATION_SEC", 3.0)

    n = object.__new__(sid_node.SpeakerIdNode)
    n._db = SpeakerDatabase(str(tmp_path / "speakers.db"))
    n._speech_log = {}
    n._speech_log_lock = threading.Lock()
    n._sample_rate = 16000
    n._recent_embeddings = collections.deque(maxlen=20)
    n._MAX_EMBED_AGE_SEC = 30.0
    n._REGISTER_UTTERANCE_WAIT_SEC = 0.2
    n._REGISTER_UTTERANCE_POLL_SEC = 0.02
    n._pending_register_name = None
    n._pending_register_lock = threading.Lock()
    n._result_pub = _FakePublisher()
    n.get_logger = MagicMock(return_value=MagicMock())
    n._epithet_request_pub = None
    n._growth_session_gap_sec = 30.0
    n._growth_session = None
    n._growth_owner_min_score = 0.65
    n._name_confidence_min_gap = 0.15
    n._executor = _SyncExecutor()
    yield n
    n._db.close()


@pytest.fixture()
def dialogue_node():
    n = object.__new__(DialogueNode)
    n.get_logger = MagicMock(return_value=MagicMock())
    n._speaker_lock = threading.Lock()
    n._publish_speaker_observation = MagicMock()
    n._speaker_register_pub = MagicMock()
    n._identity_confirmations = {}
    n._pending_identity_hint = None
    n._task_lock = threading.Lock()
    n._run_task = MagicMock()
    n._identity_question_session_gap_sec = 120.0
    n._identity_answer_window_sec = 180.0
    n._speak_direct = MagicMock()
    n._ask_identity_if_ambiguous = MagicMock()

    # Снимок диктора задаёт сам тест (``_current_speaker``); резолв по
    # utterance_id через registry (#2829) здесь не нужен.
    async def _no_resolve(_utterance_id):
        return None

    n._resolve_speaker_for_utterance = _no_resolve
    return n


def _tentative(speaker_id: str, name: str) -> dict:
    return {
        "is_known": True,
        "speaker_id": speaker_id,
        "name": None,
        "tentative_name": name,
        "tentative_conf": 0.757,
        "tentative_kind": "single",
        "confidence": 0.757,
    }


def _confirm_yes(dialogue_node, speaker_id: str, name: str) -> dict:
    """Вопрос «<имя>, это ты?» → ответ «да, это я» → опубликованный запрос."""
    dialogue_node._current_speaker = _tentative(speaker_id, name)
    asyncio.run(
        dialogue_node._apply_speaker_identity(
            "привет", speaker_context=None, utterance_id="utt-q"
        )
    )
    dialogue_node._current_speaker = _tentative(speaker_id, name)
    asyncio.run(
        dialogue_node._apply_speaker_identity(
            "робот да это я", speaker_context=None, utterance_id=UTT_ID
        )
    )
    assert dialogue_node._speaker_register_pub.publish.call_count == 1
    return json.loads(
        dialogue_node._speaker_register_pub.publish.call_args.args[0].data
    )


def _roundtrip(speaker_node, dialogue_node, payload: dict) -> list:
    """payload dialogue_node → speaker_id_node → acks → dialogue_node."""
    msg = type("Msg", (), {"data": json.dumps(payload, ensure_ascii=False)})()
    speaker_node._on_register_request(msg)
    acks = list(speaker_node._result_pub.messages)
    for ack in acks:
        if ack.get("event"):
            dialogue_node._on_speaker_result(
                type("Msg", (), {"data": json.dumps(ack, ensure_ascii=False)})()
            )
    return acks


def _spoken(dialogue_node) -> str:
    return "\n".join(
        str(c.args[0]) for c in dialogue_node._speak_direct.call_args_list
    )


def _setup_owner(speaker_node, seed: int = 2906):
    base = _embedding(seed)
    sid = speaker_node._db.register("Саша", base)
    return base, sid


def _put_utterance(speaker_node, embedding, duration: float,
                   utterance_id: str = UTT_ID) -> None:
    speaker_node._recent_embeddings.append(
        (time.time(), embedding, duration, utterance_id)
    )


# ---------------------------------------------------------------------------


def test_dialogue_marks_confirmation_growth_request(dialogue_node):
    """dialogue_node помечает запрос как служебный рост, не регистрацию."""
    payload = _confirm_yes(dialogue_node, "sid-sasha", "Саша")
    assert payload.get("purpose") == "growth"
    assert payload["speaker_id"] == "sid-sasha"
    assert payload["utterance_id"] == UTT_ID


def test_short_yes_is_silent_and_creates_no_anchor(speaker_node, dialogue_node):
    """Живой случай: «да это я» 1.65с — ни «Не расслышал», ни нового якоря."""
    base, sid = _setup_owner(speaker_node)
    _put_utterance(speaker_node, _voice_at(base, 0.80, 1), duration=1.65)

    payload = _confirm_yes(dialogue_node, sid, "Саша")
    acks = _roundtrip(speaker_node, dialogue_node, payload)

    assert NOT_HEARD not in _spoken(dialogue_node)
    dialogue_node._speak_direct.assert_not_called()
    assert not any(a.get("event") == "register_error" for a in acks), acks
    assert len(speaker_node._db.list_speakers()) == 1
    assert speaker_node._db.gallery_size(sid) == 1, "короткую фразу не пишем"


def test_long_yes_grows_owner_gallery_by_owner_rules(speaker_node, dialogue_node):
    """Длинная фраза-подтверждение владельца — +1 в ЕГО галерею, без нового
    профиля и без ack регистрации."""
    base, sid = _setup_owner(speaker_node)
    _put_utterance(speaker_node, _voice_at(base, 0.80, 2), duration=4.0)

    payload = _confirm_yes(dialogue_node, sid, "Саша")
    acks = _roundtrip(speaker_node, dialogue_node, payload)

    dialogue_node._speak_direct.assert_not_called()
    assert acks == [], "служебный рост не публикует ack регистрации"
    assert len(speaker_node._db.list_speakers()) == 1
    assert speaker_node._db.gallery_size(sid) == 2
    session = speaker_node._growth_session
    assert session is not None and session["speaker_id"] == sid


def test_long_yes_below_owner_floor_is_not_added(speaker_node, dialogue_node):
    """Правила #2833: похожесть ниже growth_owner_min_score — не пишем."""
    base, sid = _setup_owner(speaker_node)
    _put_utterance(speaker_node, _voice_at(base, 0.55, 3), duration=4.0)

    payload = _confirm_yes(dialogue_node, sid, "Саша")
    acks = _roundtrip(speaker_node, dialogue_node, payload)

    dialogue_node._speak_direct.assert_not_called()
    assert acks == []
    assert speaker_node._db.gallery_size(sid) == 1
    assert len(speaker_node._db.list_speakers()) == 1


def test_long_yes_closer_to_other_person_is_not_added(speaker_node, dialogue_node):
    """Правила #2833: фраза ближе к другому человеку — в галерею владельца нет."""
    base, sid = _setup_owner(speaker_node)
    boris = _embedding(777)
    boris_id = speaker_node._db.register("Борис", boris)
    _put_utterance(speaker_node, _voice_at(boris, 0.85, 4), duration=4.0)

    payload = _confirm_yes(dialogue_node, sid, "Саша")
    _roundtrip(speaker_node, dialogue_node, payload)

    assert speaker_node._db.gallery_size(sid) == 1
    assert speaker_node._db.gallery_size(boris_id) == 1
    assert len(speaker_node._db.list_speakers()) == 2


def test_phrase_already_grown_by_session_is_not_duplicated(speaker_node):
    base, sid = _setup_owner(speaker_node)
    phrase = _voice_at(base, 0.80, 5)
    speaker_node._db.append_reference_embedding(sid, "Саша", phrase)

    assert speaker_node._do_confirmed_growth("Саша", sid, phrase, 4.0, UTT_ID) is False
    assert speaker_node._db.gallery_size(sid) == 2


def test_growth_request_without_embedding_is_silent(speaker_node):
    _, sid = _setup_owner(speaker_node)
    msg = type("Msg", (), {"data": json.dumps(
        {"name": "Саша", "speaker_id": sid, "purpose": "growth",
         "utterance_id": "utt-missing"}
    )})()
    speaker_node._on_register_request(msg)
    assert speaker_node._result_pub.messages == []
    assert speaker_node._db.gallery_size(sid) == 1


def test_llm_registration_too_short_still_says_not_heard(
    speaker_node, dialogue_node
):
    """«Не расслышал» остаётся для регистрации по просьбе LLM (без purpose)."""
    _put_utterance(speaker_node, _embedding(99), duration=1.65, utterance_id="utt-llm")

    acks = _roundtrip(
        speaker_node,
        dialogue_node,
        {"name": "Глеб", "utterance_id": "utt-llm"},
    )

    assert any(
        a.get("event") == "register_error" and a.get("error") == "too_short"
        for a in acks
    ), acks
    # Фикстура моделирует «ход идёт» (_run_task задан): с #2908 просьба
    # повторить придерживается и ЗАМЕНИТ ответ хода (_deliver_turn_result).
    held = dialogue_node._identity_ack_state().take_held()
    assert held is not None and NOT_HEARD in held["question"], held
    assert speaker_node._db.list_speakers() == []


def test_already_known_repeat_register_unchanged(speaker_node, dialogue_node):
    """#2863 не сломан: повторный register_speaker узнанного — «уже знаю»."""
    base, sid = _setup_owner(speaker_node)
    _put_utterance(speaker_node, _voice_at(base, 0.88, 6), duration=2.67,
                   utterance_id="utt-known")

    acks = _roundtrip(
        speaker_node, dialogue_node, {"name": "Саша", "utterance_id": "utt-known"}
    )

    assert any(a.get("already_known") for a in acks), acks
    dialogue_node._speak_direct.assert_not_called()
    assert len(speaker_node._db.list_speakers()) == 1
