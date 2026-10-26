"""Issue #3858 — незнакомому голосу заводится профиль без имени, с кличкой (решение Шифу 22.10, вариант А).

«Режим мастерской» для голоса (как для лиц, решение 16.09): пишем всех подряд, имя потом. Решает код
``speaker_id_node`` (ADR-0148), не LLM. Пороги опознания (0.72) и слияния (0.75) не тронуты.

Контракт:

1. Незнакомый голос с речью не короче порога регистрации → профиль ``name=''`` + кличка; результат фразы —
   ``is_known=true, name=None, epithet=<кличка>, source='enroll'`` (диалог видит «знакомый, не представлялся»).
2. Повторная реплика того же голоса → узнан тем же профилем, по той же кличке; нового профиля нет.
3. Представился (``register_speaker`` с именем) → имя получает ЭТОТ ЖЕ профиль, дубля нет, кличка остаётся.
4. Короткая/шумная фраза → профиля нет, результат прежний (``is_known=false``).
5. Лимит безымянных → вытеснение самых старых; именованных не трогаем.
6. Уже опознанный голос (порог 0.72 пройден) профиль не плодит при ЛЮБОМ решении об имени: уверенное, одинокий
   кандидат в полосе («single», свой на плохом канале) и «contested» (n709/n210, акт 2b «Саша» 0.944 / «Борис» 0.661
   при band_high 0.99 — решение Шифу 26.10: спорному голосу двойника не заводить); открытая growth-сессия — нет;
   выключатель — нет.

Нода собрана через ``object.__new__`` (без ROS2 и resemblyzer), как в соседних тестах #2829/#2863.
"""

from __future__ import annotations

import collections
import json
import sys
import threading
import time
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

from rob_box_voice import speaker_id_node as sid_node  # noqa: E402
from rob_box_voice.core import unnamed_voice  # noqa: E402
from rob_box_voice.utils.speaker_embeddings import SpeakerDatabase  # noqa: E402
from rob_box_core.speaker_wire import speaker_result_to_payload  # noqa: E402

UNNAMED_MAX = 3


class _Embed:
    def __init__(self, embedding, voiced_sec: float) -> None:
        self.embedding = embedding
        self.voiced_sec = voiced_sec
        self.raw_sec = voiced_sec


def _embedding(seed: int) -> np.ndarray:
    v = np.random.default_rng(seed).standard_normal(256).astype(np.float32)
    return v / np.linalg.norm(v)


def _degraded(base: np.ndarray, alpha: float, noise_seed: int) -> np.ndarray:
    v = base + alpha * _embedding(noise_seed)
    return (v / np.linalg.norm(v)).astype(np.float32)


def _cos(a, b) -> float:
    return float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))


class _FakePublisher:
    def __init__(self) -> None:
        self.messages: list[dict] = []

    def publish(self, msg) -> None:
        self.messages.append(speaker_result_to_payload(msg))


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
    instance._recent_embeddings: Deque[Tuple[float, np.ndarray, float, Optional[str]]] = collections.deque(maxlen=20)
    instance._MAX_EMBED_AGE_SEC = 30.0
    instance._REGISTER_UTTERANCE_WAIT_SEC = 0.2
    instance._REGISTER_UTTERANCE_POLL_SEC = 0.02
    instance._pending_register_name = None
    instance._pending_register_lock = threading.Lock()
    instance._result_pub = _FakePublisher()
    instance.get_logger = MagicMock(return_value=MagicMock())
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
    instance._executor = ThreadPoolExecutor(max_workers=2)
    yield instance
    instance._executor.shutdown(wait=True)
    instance._db.close()


def _hear(node, embedding: np.ndarray, voiced_sec: float, utterance_id: str) -> dict:
    """Фраза прошла через ``_process_utterance``; возвращает опубликованный результат биометрии."""
    node._db.embed_audio_ex = lambda *_a, **_k: _Embed(embedding, voiced_sec)
    before = len(node._result_pub.messages)
    node._process_utterance(b"\x00" * 3200, utterance_id)
    published = node._result_pub.messages[before:]
    assert len(published) == 1, published
    return published[0]


def _wait_event(node, name: str, timeout: float = 2.0) -> dict:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for message in node._result_pub.messages:
            if message.get("event") == name:
                return message
        time.sleep(0.01)
    raise AssertionError(f"событие {name!r} не пришло: {node._result_pub.messages}")


def _profiles(node) -> list[dict]:
    return node._db.list_speakers()


# ---------------------------------------------------------------------------
# 1. Незнакомый голос → профиль без имени с кличкой
# ---------------------------------------------------------------------------


def test_unknown_voice_gets_nameless_profile_with_nickname(node):
    result = _hear(node, _embedding(1), voiced_sec=4.5, utterance_id="u1")

    (profile,) = _profiles(node)
    assert profile["name"] == ""
    assert profile["epithet"], "кличка выдана сразу"
    assert result["is_known"] is True
    assert result["speaker_id"] == profile["id"]
    assert result["name"] is None
    assert result["epithet"] == profile["epithet"]
    assert result["source"] == "enroll"
    assert result["utterance_id"] == "u1"
    assert "tentative_name" not in result and "tentative_kind" not in result


# ---------------------------------------------------------------------------
# 2. Повторная реплика того же голоса — узнан по кличке, профиль один
# ---------------------------------------------------------------------------


def test_same_voice_again_is_recognised_by_nickname_without_new_profile(node):
    base = _embedding(2)
    first = _hear(node, base, 4.5, "u1")
    phrase = _degraded(base, 0.4, noise_seed=91)
    assert _cos(base, phrase) >= 0.72, "предусловие: фраза проходит обычный порог опознания"

    second = _hear(node, phrase, 4.0, "u2")

    assert len(_profiles(node)) == 1
    assert second["is_known"] is True
    assert second["speaker_id"] == first["speaker_id"]
    assert second["epithet"] == first["epithet"]
    assert second["name"] is None
    assert second.get("source") is None, "это опознание, а не заведение профиля"


def test_two_different_unknown_voices_get_two_profiles_and_two_nicknames(node):
    first = _hear(node, _embedding(3), 4.5, "u1")
    second = _hear(node, _embedding(4), 4.5, "u2")

    assert first["speaker_id"] != second["speaker_id"]
    assert first["epithet"] != second["epithet"], "тёзок-безымянных различает кличка"
    assert len(_profiles(node)) == 2


# ---------------------------------------------------------------------------
# 3. Представился позже — имя достаётся тому же профилю
# ---------------------------------------------------------------------------


def test_introduction_names_the_same_profile_without_duplicate(node):
    base = _embedding(5)
    first = _hear(node, base, 4.5, "u1")
    _hear(node, _degraded(base, 0.4, noise_seed=92), 4.5, "u2")
    assert len(_profiles(node)) == 1

    node._on_register_request(type("Msg", (), {"data": json.dumps({"name": "Гриша", "utterance_id": "u2"})})())
    ack = _wait_event(node, "registered")

    assert ack["speaker_id"] == first["speaker_id"], "тот же профиль, не новый"
    assert ack["name"] == "Гриша"
    (profile,) = _profiles(node)
    assert profile["id"] == first["speaker_id"]
    assert profile["name"] == "Гриша"
    assert profile["epithet"] == first["epithet"], "кличка осталась внутренней меткой"

    third = _hear(node, _degraded(base, 0.4, noise_seed=93), 4.5, "u3")
    assert third["speaker_id"] == first["speaker_id"]
    assert third["name"] == "Гриша", "дальше робот знает имя"


def test_introduction_in_the_very_phrase_that_created_the_profile(node):
    """«Меня зовут Гриша» — первая же реплика: профиль заведён этой фразой и сразу получает имя."""
    base = _embedding(6)
    first = _hear(node, base, 4.5, "u1")

    node._on_register_request(type("Msg", (), {"data": json.dumps({"name": "Гриша", "utterance_id": "u1"})})())
    ack = _wait_event(node, "registered")

    assert ack["speaker_id"] == first["speaker_id"]
    assert [row["name"] for row in _profiles(node)] == ["Гриша"]


# ---------------------------------------------------------------------------
# 4. Короткая/шумная фраза — профиля нет
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("voiced_sec", [0.5, 1.4, 2.99])
def test_short_phrase_creates_no_profile(node, voiced_sec):
    result = _hear(node, _embedding(7), voiced_sec, "u1")

    assert _profiles(node) == []
    assert result["is_known"] is False


def test_threshold_boundary_three_seconds_creates_profile(node):
    _hear(node, _embedding(8), 3.0, "u1")

    assert len(_profiles(node)) == 1


# ---------------------------------------------------------------------------
# 5. Лимит безымянных — вытеснение старейших
# ---------------------------------------------------------------------------


def test_unnamed_limit_evicts_oldest_and_cleans_node_state(node):
    node._db.register("Борис", _embedding(500), duration_sec=5.0)
    results = []
    for i in range(UNNAMED_MAX + 2):
        results.append(_hear(node, _embedding(600 + i), 4.5, f"u{i}"))
        if i == 0:
            node._speech_log[results[0]["speaker_id"]] = collections.deque(["давно сказанное"])

    ids = {row["id"]: row["name"] for row in _profiles(node)}
    unnamed = [sid for sid, name in ids.items() if not name]
    assert len(unnamed) == UNNAMED_MAX
    assert "Борис" in ids.values(), "именованные не вытесняются"
    assert results[0]["speaker_id"] not in ids and results[1]["speaker_id"] not in ids, "старейшие вытеснены"
    assert all(result["speaker_id"] in ids for result in results[2:]), "новые на месте"
    assert node._db.orphaned_embedding_count() == 0
    assert results[0]["speaker_id"] not in node._speech_log, "служебное состояние вытесненного забыто"


# ---------------------------------------------------------------------------
# 6. Когда профиль НЕ заводится
# ---------------------------------------------------------------------------


def test_confidently_recognised_named_speaker_is_not_cloned(node):
    base = _embedding(9)
    sid = node._db.register("Борис", base, duration_sec=5.0)

    result = _hear(node, _degraded(base, 0.2, noise_seed=94), 4.5, "u1")

    assert result["speaker_id"] == sid and result["name"] == "Борис"
    assert [row["name"] for row in _profiles(node)] == ["Борис"]


def _stranger_in_band(boris: np.ndarray) -> np.ndarray:
    """Незнакомец с cos к Борису в полосе сомнения [0.72, 0.80)."""
    for seed in range(1000, 1400):
        cand = _degraded(boris, 0.95, noise_seed=seed)
        if 0.735 <= _cos(boris, cand) < 0.79:
            return cand
    raise AssertionError("не нашли вектор в полосе")


def test_contested_stranger_gets_no_clone_and_no_name(node):
    """n709/n210: голос в полосе между двумя людьми с разными именами (contested) — двойника нет (решение Шифу
    26.10), имени нет; результат честно «спорный»: диалог сочтёт его незнакомцем без профиля (#4023)."""
    boris = _embedding(10)
    node._db.register("Борис", boris, duration_sec=5.0)
    node._db.register("Саша", _embedding(11), duration_sec=5.0)
    stranger = _stranger_in_band(boris)

    for n, phrase in enumerate((stranger, _degraded(stranger, 0.25, noise_seed=95))):
        result = _hear(node, phrase, 4.5, f"u{n}")
        assert result["is_known"] is True and result["name"] is None
        assert result["tentative_kind"] == "contested" and "tentative_name" not in result
        assert "source" not in result, "это опознание, а не заведение профиля"

    assert sorted(row["name"] for row in _profiles(node)) == ["Борис", "Саша"], "безымянных двойников нет"


def _phrase_at(anchor: np.ndarray, cos: float, seed: int) -> np.ndarray:
    """Единичный вектор с косинусом ``cos`` к единичному ``anchor``."""
    noise = _embedding(seed)
    orth = noise - float(np.dot(noise, anchor)) * anchor
    orth /= np.linalg.norm(orth)
    return (cos * anchor + np.sqrt(1.0 - cos * cos) * orth).astype(np.float32)


def test_known_person_in_contested_band_is_not_cloned_act_2b(node):
    """Акт 2b марафона 23.10: «Саша» 0.944, «Борис» 0.661, band_high 0.99 → contested на каждой фразе. Раньше каждая
    фраза длиннее гейта речи заводила безымянную копию Саши; теперь база не растёт, результат — без имени и без
    гипотезы-имени."""
    node._name_confidence_band_high = 0.99
    sasha = _embedding(20)
    sasha_id = node._db.register("Саша", sasha, duration_sec=5.0)
    phrases = [_phrase_at(sasha, 0.944, seed=200 + i) for i in range(4)]
    boris = _phrase_at(phrases[0], 0.661, seed=300)
    node._db.register("Борис", boris, duration_sec=5.0)
    assert abs(_cos(phrases[0], sasha) - 0.944) < 1e-3 and abs(_cos(phrases[0], boris) - 0.661) < 1e-3

    results = [_hear(node, p, 6.0, f"u{i}") for i, p in enumerate(phrases)]

    assert sorted(row["name"] for row in _profiles(node)) == ["Борис", "Саша"], "двойников Саши нет"
    first = results[0]
    assert first["is_known"] is True and first["name"] is None and first["tentative_kind"] == "contested"
    assert first["speaker_id"] == sasha_id, "в результате — лишь ближайший кандидат, приписывать его решает потребитель"
    assert all(r.get("source") != "enroll" for r in results)


def test_lone_candidate_in_band_is_not_cloned(node):
    """Единственный кандидат в полосе («single») — скорее свой на плохом канале: безымянные клоны не плодим."""
    boris = _embedding(12)
    node._db.register("Борис", boris, duration_sec=5.0)

    _hear(node, _stranger_in_band(boris), 4.5, "u1")

    assert [row["name"] for row in _profiles(node)] == ["Борис"]


def test_open_growth_session_blocks_enrollment(node):
    node._growth_session = {"speaker_id": "owner", "name": "Борис", "last_utterance_at": time.time(), "count": 0}
    node._owner_growth_veto = lambda *_a, **_k: None  # вето не сработало: голос считается голосом владельца сессии
    node._db.append_reference_embedding = lambda *_a, **_k: True

    result = _hear(node, _embedding(13), 4.5, "u1")

    assert _profiles(node) == []
    assert result["is_known"] is False


def test_switch_off_keeps_old_behaviour(node):
    node._unnamed_profiles_enabled = False

    result = _hear(node, _embedding(14), 4.5, "u1")

    assert _profiles(node) == []
    assert result["is_known"] is False


# ---------------------------------------------------------------------------
# Политика — чистая функция
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "kwargs, expected",
    [
        ({}, None),
        ({"enabled": False}, unnamed_voice.SKIP_DISABLED),
        ({"voiced_sec": 2.9}, unnamed_voice.SKIP_TOO_SHORT),
        ({"voiced_sec": None}, unnamed_voice.SKIP_TOO_SHORT),
        ({"recognised": True}, unnamed_voice.SKIP_RECOGNISED),
        ({"growth_session_open": True}, unnamed_voice.SKIP_GROWTH_SESSION),
    ],
)
def test_enroll_skip_reason(kwargs, expected):
    """Решение об имени (confident/single/contested) в правило не входит: узнанный голос клона не получает."""
    args = dict(
        enabled=True,
        voiced_sec=4.0,
        min_voiced_sec=3.0,
        recognised=False,
        growth_session_open=False,
    )
    args.update(kwargs)

    assert unnamed_voice.enroll_skip_reason(**args) == expected


def test_nameless_match_does_not_count_as_other_name_in_gap_rule():
    """Безымянный профиль не «человек с другим именем»: именованный голос рядом с ним не становится contested."""
    best = sid_node.SpeakerMatch(speaker_id="a", name="Борис", confidence=0.76)
    nameless = sid_node.SpeakerMatch(speaker_id="b", name="", confidence=0.70)

    assert sid_node._gap_to_other_name(best, [best, nameless]) is None
    assert (
        sid_node.classify_name_confidence(best, [best, nameless], band_high=0.80, min_gap=0.15)
        == sid_node.NAME_TENTATIVE_SINGLE
    )


# ---------------------------------------------------------------------------
# Вердикт в логе = опубликованное (то, что читает must_not_identify_as, ADR-0134)
# ---------------------------------------------------------------------------


def _verdict_names(line: str) -> list[str]:
    """Тот же регэксп, что у харнесса (.github/workflows/scripts/e2e_tool_match.py::_IDENTIFY_VERDICT_RE)."""
    import re

    return re.findall(r"(?:👤\s*)?Speaker:\s*['\"]([^'\"]+)['\"]", line)


def test_verdict_line_does_not_name_a_suppressed_candidate():
    """n210: незнакомец в полосе сомнения не «опознан как Борис», хотя Борис — ближайший кандидат."""
    boris = sid_node.SpeakerMatch(speaker_id="b" * 8, name="Борис", confidence=0.78)

    line = sid_node._verdict_line(boris, sid_node.NAME_TENTATIVE_CONTESTED, 12.0)

    assert _verdict_names(line) == []
    assert "Борис" in line, "кандидат остаётся в диагностике"


def test_verdict_line_for_confident_nameless_and_unknown():
    boris = sid_node.SpeakerMatch(speaker_id="b" * 8, name="Борис", confidence=0.91)
    nameless = sid_node.SpeakerMatch(speaker_id="c" * 8, name="", confidence=0.0, epithet="Гость")

    assert _verdict_names(sid_node._verdict_line(boris, sid_node.NAME_CONFIDENT, 5.0)) == ["Борис"]
    assert _verdict_names(sid_node._verdict_line(None, None, 5.0)) == []
    assert _verdict_names(sid_node._verdict_line(None, None, 5.0, enrolled=nameless)) == []
    assert _verdict_names(sid_node._verdict_line(nameless, None, 5.0)) == []
