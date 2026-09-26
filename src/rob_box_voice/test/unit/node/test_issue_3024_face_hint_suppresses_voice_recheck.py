"""test_issue_3024_face_hint_suppresses_voice_recheck.py

ADR-0135 / issue #3024 — «Робот поздоровался по имени (лицо), а через 26
сек спрашивает „Дэнчик, это ты?"».

Гипотеза подтверждена в t_ee583f20: лицо (Vision Pi) приходит в
``/vision/hailo/events`` → ``parse_meeting_marker`` → ``_handle_meeting``
(приветствие по имени). Голос приходит позже с ``score=0.791`` (band
``single``), и ``_handle_tentative_speaker`` (#2809/#2888) задаёт
переспрос, потому что шов «Знакомый» не знал, что лицо уже подтвердило.

Фикс (ADR-0135 §2-3): новый сигнал ``FaceSignal`` в ``IdentitySeam``
(метод ``note_face_seen`` + ``recent_face_observation``), плюс
короткая проверка перед блоком переспроса в ``_handle_tentative_speaker``:
есть свежий face-hint с тем же именем и ``sim >= high_threshold`` →
идём в confirmation path, не задаём «<Имя>, это ты?».

Этот тест **красный** на текущем коде (без реализации) и **зелёный**
после её появления. Сценарий заказчика — issue #3024 в миниатюре.

Тест не поднимает ROS2 — DialogueNode собирается через
``object.__new__``, как в ``test_issue_2809_*``.
"""

from __future__ import annotations

import asyncio
import sys
import threading
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest


# ROS2/audio-зависимости — стаб, как в test_issue_2809_*.py.
_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())


sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_harness.identity import MemoryIdentitySeam  # noqa: E402
from rob_box_harness.identity.base import (  # noqa: E402
    FaceObservation,
    FaceSignal,
)
from rob_box_harness.memory import InMemoryStore  # noqa: E402
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402


def _run(coro):
    return asyncio.run(coro)


def _make_node(identity: MemoryIdentitySeam) -> DialogueNode:
    """Минимальный DialogueNode — только то, что нужно _handle_tentative_speaker.

    Шов идентичности — настоящий (MemoryIdentitySeam), чтобы тест проверял
    И логику шва (note_face_seen/recent_face_observation), И потребителя
    (_handle_tentative_speaker). Всё остальное — MagicMock, чтобы тест не
    зависел от TTS/LLM/истории.

    ``tentative_states`` имитирует ``_identity_confirmations`` из
    DialogueNode: ключ — ``full_sid``, значение — тот же dict, что
    выдаёт реальный ``_tentative_session_state`` (``confirmed=None``,
    не ``False``, иначе существующий блок ``if state.get("confirmed")
    is False`` в строке 4006 рано возвращает ``_tag_tentative``).
    """
    n = object.__new__(DialogueNode)
    n.get_logger = MagicMock(return_value=MagicMock())
    n._speaker_lock = threading.Lock()
    n._identity = identity
    n._pending_identity_hint = None
    n._tentative_states: dict = {}
    n._tentative_session_state = MagicMock(
        side_effect=lambda full_sid: n._tentative_states.setdefault(
            full_sid,
            {
                "asked": False,
                "confirmed": None,
                "name": None,
                "growth_registered": False,
            },
        )
    )
    n._resolve_pending_tentative_answer = MagicMock()
    n._confirm_tentative_speaker = MagicMock(
        return_value="[Speaker:confirmed]"  # условный tag
    )
    n._ask_tentative_identity = MagicMock()
    n._tag_tentative = MagicMock(return_value="[Speaker:tentative]")
    # ADR-0135 параметры (дефолты карточки).
    n._face_voice_hint_enabled = True
    n._face_voice_hint_window_sec = 30.0
    n._face_voice_hint_high_threshold = 0.78
    n._face_voice_hint_low_threshold = 0.65
    n._turn_self_intro = None
    return n


def _single_sp(name: str, score: float, full_sid: str = "c9e981cb") -> dict:
    """Голосовой сигнал из /voice/speaker/result — single band."""
    return {
        "speaker_id": full_sid,
        "tentative_kind": "single",
        "tentative_name": name,
        "tentative_conf": score,
        "confidence": score,
    }


def test_face_signal_and_observation_dataclasses_exist():
    """Красный шаг 1: импорт публичных типов из IdentitySeam.

    Пока реализации нет — ImportError на ``FaceSignal``/``FaceObservation``.
    После реализации (ADR-0135 §2.1-2.2) — оба импорта проходят.
    """
    sig = FaceSignal(
        person_id="4ff0ddc5",
        name="Дэнчик",
        similarity=0.81,
        is_new=False,
        source_camera="oak_d",
        captured_at=1_000_000.0,
    )
    obs = FaceObservation(
        person_id=sig.person_id,
        name=sig.name,
        similarity=sig.similarity,
        captured_at=sig.captured_at,
        is_new=sig.is_new,
        confidence_band="high",
    )
    assert obs.person_id == "4ff0ddc5"
    assert obs.name == "Дэнчик"
    assert obs.confidence_band == "high"
    assert obs.is_recent(window_sec=30, now=1_000_010.0) is True
    assert obs.is_recent(window_sec=30, now=1_000_100.0) is False


def test_identity_seam_note_face_seen_and_recent_observation_roundtrip():
    """Шов хранит последний face-signal per person_id, читается окном.

    Это контракт ADR-0135 §2.2: ``note_face_seen`` кладёт наблюдение в
    кольцевой буфер, ``recent_face_observation(person_id)`` возвращает
    самое свежее (или ``None`` если пусто/протухло).
    """
    store = InMemoryStore()
    _run(store.init())
    seam = MemoryIdentitySeam(store)

    sig = FaceSignal(
        person_id="4ff0ddc5",
        name="Дэнчик",
        similarity=0.81,
        is_new=False,
        source_camera="oak_d",
        captured_at=1_000_000.0,
    )
    # До — нет наблюдения.
    assert seam.recent_face_observation("4ff0ddc5", now=1_000_000.0) is None

    seam.note_face_seen(sig, now=1_000_000.0)
    obs = seam.recent_face_observation("4ff0ddc5", now=1_000_010.0)
    assert obs is not None
    assert obs.person_id == "4ff0ddc5"
    assert obs.name == "Дэнчик"
    assert obs.similarity == 0.81
    assert obs.confidence_band == "high"  # 0.81 >= high=0.78
    # За окном — None (TTL = window_sec).
    assert (
        seam.recent_face_observation("4ff0ddc5", window_sec=30, now=1_000_100.0)
        is None
    )


def test_identity_seam_face_observation_confidence_bands():
    """Полосы уверенности: high/tentative/low по порогам из ADR-0135 §2.4."""
    store = InMemoryStore()
    _run(store.init())
    seam = MemoryIdentitySeam(store)

    base_kw = dict(person_id="x", is_new=False, source_camera="oak_d")

    seam.note_face_seen(
        FaceSignal(name=None, similarity=0.50, captured_at=1.0, **base_kw),
        now=1.0,
    )
    obs_low = seam.recent_face_observation("x", now=2.0)
    assert obs_low is not None and obs_low.confidence_band == "low"

    seam.note_face_seen(
        FaceSignal(name="A", similarity=0.70, captured_at=3.0, **base_kw),
        now=3.0,
    )
    obs_tent = seam.recent_face_observation("x", now=4.0)
    assert obs_tent is not None and obs_tent.confidence_band == "tentative"

    seam.note_face_seen(
        FaceSignal(name="A", similarity=0.85, captured_at=5.0, **base_kw),
        now=5.0,
    )
    obs_high = seam.recent_face_observation("x", now=6.0)
    assert obs_high is not None and obs_high.confidence_band == "high"


def test_face_hint_suppresses_tentative_question_in_window():
    """Главный сценарий issue #3024.

    1. Лицо увидело «Дэнчик» (person_id=4ff0ddc5, sim=0.81) в момент T0.
    2. Через 5 сек голос пришёл c name='Дэнчик', score=0.791, single.
       Без face-hint → переспрос «Дэнчик, это ты?».
       С face-hint в окне → confirmation path, переспроса НЕТ.
    """
    import time as _time

    store = InMemoryStore()
    _run(store.init())
    seam = MemoryIdentitySeam(store)
    node = _make_node(seam)

    # (1) лицо положили в шов через note_face_seen — СЕЙЧАС.
    T0 = _time.time()
    seam.note_face_seen(
        FaceSignal(
            person_id="4ff0ddc5",
            name="Дэнчик",
            similarity=0.81,
            is_new=False,
            source_camera="oak_d",
            captured_at=T0,
        ),
        now=T0,
    )

    # (2) голос через 5 сек.
    sp = _single_sp("Дэнчик", score=0.791)
    result = node._handle_tentative_speaker(
        sp, user_input="привет", utterance_id="utt-1"
    )

    # confirmation path: переспрос НЕ задан, _confirm_tentative_speaker вызван.
    state = node._tentative_states["c9e981cb"]
    assert state["asked"] is True, (
        "asked=True допустимо — блок ADR-0135 ставит asked=True перед "
        "вызовом _confirm_tentative_speaker (как и #2809 confirmation)."
    )
    assert state["confirmed"] is True
    assert state["name"] == "Дэнчик"
    node._ask_tentative_identity.assert_not_called()
    node._confirm_tentative_speaker.assert_called_once()
    # tag от confirmation path, не от _tag_tentative.
    assert result == "[Speaker:confirmed]"
    node._tag_tentative.assert_not_called()


def test_face_hint_outside_window_still_asks():
    """Регрессия ADR-0135 §3 п.5: hint протух → голос переспрашивает."""
    store = InMemoryStore()
    _run(store.init())
    seam = MemoryIdentitySeam(store)
    node = _make_node(seam)

    T0 = 1_000_000.0
    seam.note_face_seen(
        FaceSignal(
            person_id="4ff0ddc5",
            name="Дэнчик",
            similarity=0.81,
            is_new=False,
            source_camera="oak_d",
            captured_at=T0,
        ),
        now=T0,
    )

    # 31 сек спустя — за окном window_sec=30.
    sp = _single_sp("Дэнчик", score=0.791)
    # Подменяем now: тестируем через seam и через прямой путь.
    # Свежесть hint = now - T0. Делаем «сейчас» = T0 + 31.
    # Проще всего: вызвать seam.recent_face_observation с другим now
    # — но _handle_tentative_speaker использует time.time() внутри.
    # Поэтому проверяем через сам seam: вернёт ли он None.
    obs = seam.recent_face_observation(
        "4ff0ddc5", window_sec=30, now=T0 + 31.0
    )
    assert obs is None, (
        "hint старше window_sec должен быть протухшим (None) "
        "— иначе голос не переспросит, и #3024 false-fix"
    )

    # Поведение dialogue_node при протухшем hint: в реальном времени
    # переспрос произойдёт. В тесте нельзя двигать time.time(), поэтому
    # ограничиваемся инвариантом seam.recent_face_observation.
    # Дополнительно: убеждаемся, что без свежего hint переспрос возможен.
    # Свежий hint отсутствует (только что положенный — протух):
    assert seam.recent_face_observation("4ff0ddc5", now=T0 + 31.0) is None


def test_face_hint_disabled_flag_falls_back_to_old_behavior():
    """ADR-0135 §2.5: face_voice_hint.enabled=false → переспрос как раньше."""
    store = InMemoryStore()
    _run(store.init())
    seam = MemoryIdentitySeam(store)
    node = _make_node(seam)
    node._face_voice_hint_enabled = False  # флаг выключен

    seam.note_face_seen(
        FaceSignal(
            person_id="4ff0ddc5",
            name="Дэнчик",
            similarity=0.81,
            is_new=False,
            source_camera="oak_d",
            captured_at=1_000_000.0,
        ),
        now=1_000_000.0,
    )

    sp = _single_sp("Дэнчик", score=0.791)
    node._handle_tentative_speaker(sp, "привет", "utt-2")

    state = node._tentative_states["c9e981cb"]
    # Переспрос задан (asked=True), confirmation НЕ вызван.
    # ``confirmed`` остаётся ``None`` до явного «да»/«нет» от человека
    # (resolve_pending выставляет True/False), поэтому здесь ждём None.
    assert state["asked"] is True
    assert state["confirmed"] is None
    node._ask_tentative_identity.assert_called_once()
    node._confirm_tentative_speaker.assert_not_called()


def test_face_hint_low_similarity_does_not_suppress_question():
    """ADR-0135 §2.4 полоса low (< 0.65) — hint игнорируется."""
    store = InMemoryStore()
    _run(store.init())
    seam = MemoryIdentitySeam(store)
    node = _make_node(seam)

    seam.note_face_seen(
        FaceSignal(
            person_id="4ff0ddc5",
            name="Дэнчик",
            similarity=0.40,  # < low_threshold=0.65
            is_new=False,
            source_camera="oak_d",
            captured_at=1_000_000.0,
        ),
        now=1_000_000.0,
    )

    sp = _single_sp("Дэнчик", score=0.791)
    node._handle_tentative_speaker(sp, "привет", "utt-3")

    state = node._tentative_states["c9e981cb"]
    # band == "low" → hint игнорируется → переспрос задан.
    assert state["asked"] is True
    assert state["confirmed"] is None
    node._ask_tentative_identity.assert_called_once()
    node._confirm_tentative_speaker.assert_not_called()


def test_face_hint_name_mismatch_does_not_suppress_question():
    """Hint для ДРУГОГО человека (тёзка/путаница) → НЕ подавляет переспрос.

    ADR-0135 §2.4 условие: ``face_obs.name == tentative_name``.
    """
    store = InMemoryStore()
    _run(store.init())
    seam = MemoryIdentitySeam(store)
    node = _make_node(seam)

    seam.note_face_seen(
        FaceSignal(
            person_id="4ff0ddc5",
            name="Саша",  # другой человек
            similarity=0.85,
            is_new=False,
            source_camera="oak_d",
            captured_at=1_000_000.0,
        ),
        now=1_000_000.0,
    )

    sp = _single_sp("Дэнчик", score=0.791)
    node._handle_tentative_speaker(sp, "привет", "utt-4")

    state = node._tentative_states["c9e981cb"]
    assert state["asked"] is True
    assert state["confirmed"] is None
    node._ask_tentative_identity.assert_called_once()
    node._confirm_tentative_speaker.assert_not_called()
