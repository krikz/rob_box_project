#!/usr/bin/env python3
"""#4054 (решение Шифу 27.10, порт #3801 в v2): уверенно узнанный голос под другим именем — один человек.

Марафон 27.10, акт 2c (run 40476941395, raw в #4054)::

    🔍 identify: best='Нина'(e35517b2) score=0.930 | second='Борис' 0.746
    🎤 STT: робот привет я тамара мы с тобой еще не знакомы запомни мой голос пожалуйста
    ⚠️ register_or_merge: голос похож на 'Нина' (score=0.930 >= 0.75), но регистрируют как 'Тамара' —
       конфликт имён, слияния НЕ делаю, завожу отдельный профиль
    🔍 identify: best='Тамара'(5f2ce7a5) 0.888 | second='Нина' 0.877 | gap=0.011

Контракт после #4054 (сторона ``speaker_id_node``, владельца голосов):

1. Голос фразы регистрации УВЕРЕННО узнан (``classify_name_confidence`` = confident, ``name_confidence_band_high``)
   как профиль P с другим именем — P переименовывается, второго профиля нет; ``speaker_id``, галерея и кличка те же
   (факты памяти живут в scope ``speaker:<id>`` — переезжать им некуда). Ack ``registered`` несёт ``new_name``.
2. Полоса сомнения (single/contested, голос ниже уверенного) — как раньше, ADR-0127: отдельный профиль и
   ``voice_conflict``, без вопросов (решения 26.10 #3993/#3858).
3. Новый незнакомый голос + имя — обычная регистрация.

Сборка ноды — как в test_issue_2863_keep_known_speaker.py (``object.__new__``, без ROS2 и resemblyzer).
"""

from __future__ import annotations

import collections
import json
import math
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
from rob_box_voice.utils.speaker_embeddings import SpeakerDatabase  # noqa: E402
from rob_box_core.speaker_wire import speaker_result_to_payload  # noqa: E402


def _embedding(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(256).astype(np.float32)
    return v / np.linalg.norm(v)


def _at_cosine(base: np.ndarray, cos: float, seed: int) -> np.ndarray:
    """Вектор с ТОЧНЫМ косинусом ``cos`` к ``base`` (шум ортогонализован к ``base``)."""
    noise = _embedding(seed)
    noise = noise - float(np.dot(noise, base)) * base
    noise = noise / np.linalg.norm(noise)
    return (cos * base + math.sqrt(1.0 - cos * cos) * noise).astype(np.float32)


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
    instance._pending_register_name: Optional[str] = None
    instance._pending_register_lock = threading.Lock()
    instance._result_pub = _FakePublisher()
    instance.get_logger = MagicMock(return_value=MagicMock())
    instance._epithet_request_pub = None
    instance._growth_session_gap_sec = 30.0
    instance._growth_session = None
    instance._growth_owner_min_score = 0.65
    instance._name_confidence_min_gap = 0.15
    instance._name_confidence_band_high = 0.80
    instance._identify_min_voiced_sec = 1.0
    instance._executor = ThreadPoolExecutor(max_workers=2)
    yield instance
    instance._executor.shutdown(wait=True)
    instance._db.close()


def _events(node, name):
    return [m for m in node._result_pub.messages if m.get("event") == name]


def _matches(node):
    return [m for m in node._result_pub.messages if not m.get("event") and m.get("is_known")]


def _profiles(node):
    return {p["id"]: p["name"] for p in node._db.list_speakers()}


def _wait_for_event(node, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if any(m.get("event") for m in node._result_pub.messages):
            return True
        time.sleep(0.01)
    return False


def _known(node, name, seed, duration=6.0):
    """Профиль ``name``, зарегистрированный штатно (как первый шаг акта: «меня зовут Нина…»)."""
    base = _embedding(seed)
    assert node._do_register(name, base, speaker_id=None, duration_sec=duration, utterance_id=f"u-{name}")
    node._result_pub.messages.clear()
    sid = next(p for p, n in _profiles(node).items() if n == name)
    return base, sid


# ---------------------------------------------------------------------------
# 1. Уверенный голос + другое имя → переименование, второго профиля нет
# ---------------------------------------------------------------------------


def test_act_2c_confident_nina_says_tamara_renames_the_one_profile(node):
    """n722: голос Нины 0.930 → «я Тамара, мы не знакомы, запомни мой голос» — один профиль, имя Тамара."""
    base, sid = _known(node, "Нина", seed=1)
    epithet = node._db.get_epithet(sid)
    phrase = _at_cosine(base, 0.93, seed=2)
    node._recent_embeddings.append((time.time(), phrase, 6.8, "u722"))

    node._on_register_request(type("Msg", (), {"data": json.dumps({"name": "Тамара", "utterance_id": "u722"})})())
    assert _wait_for_event(node)
    node._executor.shutdown(wait=True)

    assert _profiles(node) == {sid: "Тамара"}, "второго профиля нет, первый переименован"
    assert node._db.gallery_size(sid) == 1, "галерея та же: регистрация не стала новым эталоном"
    assert node._db.get_epithet(sid) == epithet, "кличка у профиля осталась"
    ack = _events(node, "registered")
    assert len(ack) == 1 and _events(node, "register_error") == []
    assert ack[0]["speaker_id"] == sid and ack[0]["name"] == "Тамара" and ack[0]["new_name"] == "Тамара"
    assert ack[0]["reused_profile"] is True and ack[0]["utterance_id"] == "u722"
    assert "voice_conflict" not in ack[0] and not ack[0].get("already_known")
    match = _matches(node)
    assert [(m["speaker_id"], m["name"], m.get("source"), m.get("utterance_id")) for m in match] == [
        (sid, "Тамара", "register", "u722")], "новое имя доезжает до диалога этой же фразой"
    # Лицо (FaceStore) берёт имя из этого результата, только если в кадре ровно одно лицо этого голоса
    # (FaceRecognizer.note_voice_identification); иначе остаётся старое имя — как и при /voice/speaker/rename.
    assert node._growth_session["speaker_id"] == sid and node._growth_session["name"] == "Тамара"


def test_after_rename_the_voice_is_tamara_and_nina_is_gone(node):
    """n723 «как меня зовут?»: голос узнаётся одним профилем — Тамара; Нины рядом нет (gap 0.011 больше не бывает)."""
    base, sid = _known(node, "Нина", seed=3)
    assert node._do_register("Тамара", _at_cosine(base, 0.93, seed=4), speaker_id=None, duration_sec=6.8,
                             utterance_id="u722")
    next_phrase = _at_cosine(base, 0.90, seed=5)
    assert node._db.identify(next_phrase).name == "Тамара"
    assert [c.name for c in node._db.identify_candidates(next_phrase, top_n=5)] == ["Тамара"]
    node._db.append_reference_embedding(sid, "Нина", next_phrase)  # рост галереи сессией, открытой до rename
    assert _profiles(node) == {sid: "Тамара"}, "рост галереи имя назад не возвращает (#3801)"


@pytest.mark.parametrize("old, new", [("Нина", "Тамара"), ("Эйджик", "Денчик")])
def test_name_correction_by_the_same_voice_renames(node, old, new):
    """«меня зовут Денчик» при профиле «Эйджик» (ослышка STT) — поправка имени (Шифу 27.10: ДА, переименовать)."""
    base, sid = _known(node, old, seed=6)
    assert node._do_register(new, _at_cosine(base, 0.88, seed=7), speaker_id=None, duration_sec=5.0,
                             utterance_id="u1")
    assert _profiles(node) == {sid: new}
    assert _events(node, "registered")[0]["new_name"] == new


def test_short_phrase_of_a_confident_voice_still_renames(node):
    """Короткая фраза («я Тамара», 1.6 с) эталоном не станет, но переименование эталона не пишет: голос уверен —
    имя меняется, ``too_short`` нет."""
    base, sid = _known(node, "Нина", seed=8)
    assert node._do_register("Тамара", _at_cosine(base, 0.85, seed=9), speaker_id=None, duration_sec=1.6,
                             utterance_id="u1")
    assert _profiles(node) == {sid: "Тамара"} and node._db.gallery_size(sid) == 1
    assert _events(node, "register_error") == []


def test_other_profiles_are_untouched(node):
    base_n, sid_n = _known(node, "Нина", seed=10)
    _, sid_b = _known(node, "Борис", seed=11)
    assert node._do_register("Тамара", _at_cosine(base_n, 0.93, seed=12), speaker_id=None, duration_sec=6.0,
                             utterance_id="u1")
    assert _profiles(node) == {sid_n: "Тамара", sid_b: "Борис"}
    node.get_logger().warning.assert_not_called()  # тёзок нет — предупреждения нет


def test_rename_into_a_taken_name_makes_namesakes_and_says_so_in_the_log(node):
    """Ревью PR #4087: профили «Нина»(A) и «Тамара»(B), голос A 0.93 + «Тамара» — A переименован (голос фразы - A),
    профилей «Тамара» два; молча не склеиваем (слияние необратимо, ADR-0127), а предупреждаем в логе с подсказкой
    склейки обоих id."""
    base_n, sid_n = _known(node, "Нина", seed=28)
    _, sid_t = _known(node, "Тамара", seed=29)
    assert node._do_register("тамара", _at_cosine(base_n, 0.93, seed=30), speaker_id=None, duration_sec=6.0,
                             utterance_id="u1")
    assert _profiles(node) == {sid_n: "Тамара", sid_t: "Тамара"}
    assert _events(node, "registered")[0]["speaker_id"] == sid_n
    warned = [c.args[0] for c in node.get_logger().warning.call_args_list]
    assert len(warned) == 1 and sid_t[:8] in warned[0] and f"merge_speakers(src={sid_t}, dst={sid_n})" in warned[0]


# ---------------------------------------------------------------------------
# 2. Полоса сомнения — без изменений (ADR-0127, решения 26.10)
# ---------------------------------------------------------------------------


def test_single_band_voice_keeps_the_adr0127_separate_profile(node):
    """0.77: порог слияния пройден, но имя этой фразы вслух не звучало бы (single) — отдельный профиль, конфликт."""
    base, sid = _known(node, "Нина", seed=13)
    assert node._do_register("Тамара", _at_cosine(base, 0.77, seed=14), speaker_id=None, duration_sec=6.0,
                             utterance_id="u1")
    profiles = _profiles(node)
    assert len(profiles) == 2 and profiles[sid] == "Нина"
    ack = _events(node, "registered")[0]
    assert ack["voice_conflict"]["speaker_id"] == sid and "new_name" not in ack


def test_contested_voice_keeps_the_adr0127_separate_profile(node):
    """Акт 2c n721: голос 0.774 к Борису при конкуренте с другим именем (contested) — «Нина» заводится отдельно, как
    и было; вопросов человеку нет, двойника без имени нет (#3858)."""
    base_b, sid_b = _known(node, "Борис", seed=15)
    _, sid_s = _known(node, "Саша", seed=16)
    assert node._do_register("Нина", _at_cosine(base_b, 0.774, seed=17), speaker_id=None, duration_sec=8.4,
                             utterance_id="u721")
    profiles = _profiles(node)
    assert len(profiles) == 3 and profiles[sid_b] == "Борис" and profiles[sid_s] == "Саша"
    ack = _events(node, "registered")[0]
    assert ack["name"] == "Нина" and ack["voice_conflict"]["name"] == "Борис" and "new_name" not in ack


def test_confidence_is_the_live_band_high_parameter_not_a_new_threshold(node):
    """«Уверенно» — ``name_confidence_band_high``: поднят до 0.95 — тот же голос 0.93 уже полоса сомнения."""
    node._name_confidence_band_high = 0.95
    base, sid = _known(node, "Нина", seed=18)
    assert node._do_register("Тамара", _at_cosine(base, 0.93, seed=19), speaker_id=None, duration_sec=6.0,
                             utterance_id="u1")
    assert len(_profiles(node)) == 2 and _profiles(node)[sid] == "Нина"
    assert "voice_conflict" in _events(node, "registered")[0]


# ---------------------------------------------------------------------------
# 3. Новый незнакомый голос, то же имя, безымянный профиль — как раньше
# ---------------------------------------------------------------------------


def test_new_stranger_voice_with_a_name_registers_normally(node):
    _, sid = _known(node, "Нина", seed=20)
    assert node._do_register("Тамара", _embedding(21), speaker_id=None, duration_sec=6.0, utterance_id="u1")
    profiles = _profiles(node)
    assert len(profiles) == 2 and profiles[sid] == "Нина" and "Тамара" in profiles.values()
    ack = _events(node, "registered")[0]
    assert "new_name" not in ack and "voice_conflict" not in ack and ack["reused_profile"] is False


def test_same_name_same_voice_is_not_a_rename(node):
    base, sid = _known(node, "Нина", seed=22)
    assert node._do_register("нина", _at_cosine(base, 0.93, seed=23), speaker_id=None, duration_sec=6.0,
                             utterance_id="u1")
    assert _profiles(node) == {sid: "Нина"}
    assert "new_name" not in _events(node, "registered")[0]


def test_unnamed_profile_gets_the_name_by_claim_not_by_rename(node):
    """#3858: безымянному профилю имя присваивает ``register_or_merge`` (claim), а не переименование #4054."""
    base = _embedding(24)
    sid = node._db.unnamed.register(base, duration_sec=5.0).speaker_id
    assert node._do_register("Тамара", _at_cosine(base, 0.93, seed=25), speaker_id=None, duration_sec=6.0,
                             utterance_id="u1")
    assert _profiles(node) == {sid: "Тамара"}
    assert "new_name" not in _events(node, "registered")[0]


def test_junk_name_does_not_rename(node):
    """Мусорное имя («Зовут») профиль не переименовывает — решает обычный путь (там же валидатор имён)."""
    base, sid = _known(node, "Нина", seed=26)
    with pytest.raises(ValueError):
        node._do_register("Зовут", _at_cosine(base, 0.93, seed=27), speaker_id=None, duration_sec=6.0,
                          utterance_id="u1")
    assert _profiles(node) == {sid: "Нина"}
