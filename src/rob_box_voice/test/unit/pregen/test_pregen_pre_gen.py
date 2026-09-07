"""Tests for :func:`pre_gen.build_pregen_task` (ADR-0056 §6.4 #1-2)."""
from __future__ import annotations

import pytest

from rob_box_voice.scheduler.pregen import PreGenTask, build_pregen_task


def test_build_pregen_task_none_when_no_pregenerate_field():
    """§6.4 #1: legacy publisher → no pre-gen."""
    chunk = {
        "speech_id": "current",
        "ssml": "<speak>привет</speak>",
        "dialogue_id": "d1",
        "batch_index": 1,
        "batch_total": 3,
    }
    assert build_pregen_task(chunk) is None


def test_build_pregen_task_returns_when_next_available():
    """§6.4 #2: full payload → PreGenTask."""
    chunk = {
        "speech_id": "current",
        "ssml": "<speak>привет</speak>",
        "dialogue_id": "d1",
        "batch_index": 1,
        "batch_total": 3,
        "pregenerate": {
            "next_speech_id": "next",
            "next_ssml": "<speak>мир</speak>",
            "next_voice": "anton",
            "next_language": "ru",
        },
    }
    task = build_pregen_task(
        chunk,
        fallback_voice="yandex_voice",
        fallback_language="ru",
    )
    assert task is not None
    assert isinstance(task, PreGenTask)
    assert task.next_speech_id == "next"
    assert task.next_ssml == "<speak>мир</speak>"
    assert task.voice == "anton"
    assert task.language == "ru"
    assert task.dialogue_id == "d1"
    assert task.priority == "normal"
    assert task.source == ""


def test_build_pregen_task_returns_none_for_last_chunk():
    """Last chunk in batch → nothing to speculate on."""
    chunk = {
        "speech_id": "current",
        "ssml": "<speak>привет</speak>",
        "batch_index": 3,
        "batch_total": 3,
        "pregenerate": {
            "next_speech_id": "next",
            "next_ssml": "<speak>не будет</speak>",
        },
    }
    assert build_pregen_task(chunk) is None


def test_build_pregen_task_returns_none_on_self_feedback():
    """next_speech_id == current speech_id → self-feedback guard."""
    chunk = {
        "speech_id": "abc",
        "ssml": "x",
        "batch_index": 1,
        "batch_total": 3,
        "pregenerate": {
            "next_speech_id": "abc",  # same as current
            "next_ssml": "<speak>yo</speak>",
        },
    }
    assert build_pregen_task(chunk) is None


def test_build_pregen_task_handles_malformed_payload_gracefully():
    """Garbage in → None out (never raise)."""
    chunk = {
        "speech_id": "x",
        "ssml": "y",
        "pregenerate": "not-a-dict",
    }
    assert build_pregen_task(chunk) is None

    chunk2 = {
        "speech_id": "x",
        "ssml": "y",
        "pregenerate": {
            "next_speech_id": 12345,  # wrong type
            "next_ssml": "   ",
        },
    }
    assert build_pregen_task(chunk2) is None

    chunk3 = {
        "speech_id": "x",
        "ssml": "y",
        "pregenerate": {
            "next_speech_id": "",
            "next_ssml": "valid",
        },
    }
    assert build_pregen_task(chunk3) is None


def test_build_pregen_task_uses_fallbacks_when_voice_missing():
    chunk = {
        "speech_id": "cur",
        "ssml": "x",
        "batch_index": 1,
        "batch_total": 3,
        "pregenerate": {
            "next_speech_id": "nxt",
            "next_ssml": "<speak>y</speak>",
        },
    }
    task = build_pregen_task(
        chunk, fallback_voice="anton", fallback_language="ru"
    )
    assert task is not None
    assert task.voice == "anton"
    assert task.language == "ru"


def test_build_pregen_task_priority_normalised_to_known_values():
    """Unknown priority string falls back to 'normal'."""
    chunk = {
        "speech_id": "cur",
        "ssml": "x",
        "batch_index": 1,
        "batch_total": 3,
        "pregenerate": {
            "next_speech_id": "nxt",
            "next_ssml": "<speak>y</speak>",
            "priority": "bogus",
        },
    }
    task = build_pregen_task(chunk, fallback_voice="anton")
    assert task.priority == "normal"


def test_build_pregen_task_accepts_known_priorities():
    chunk = {
        "speech_id": "cur",
        "ssml": "x",
        "batch_index": 1,
        "batch_total": 3,
        "pregenerate": {
            "next_speech_id": "nxt",
            "next_ssml": "<speak>y</speak>",
            "priority": "operator",
        },
    }
    task = build_pregen_task(chunk)
    assert task.priority == "operator"


def test_pregen_task_rejects_invalid_construction():
    """PreGenTask dataclass itself rejects empty fields."""
    with pytest.raises(ValueError):
        PreGenTask(
            next_speech_id="",
            next_ssml="<speak>x</speak>",
            voice="anton",
            language="ru",
            ssml_attributes={},
            dialogue_id="d",
        )
    with pytest.raises(ValueError):
        PreGenTask(
            next_speech_id="abc",
            next_ssml="",
            voice="anton",
            language="ru",
            ssml_attributes={},
            dialogue_id="d",
        )
    with pytest.raises(ValueError):
        PreGenTask(
            next_speech_id="abc",
            next_ssml="<speak>x</speak>",
            voice="anton",
            language="ru",
            ssml_attributes={},
            dialogue_id="d",
            priority="nonsense",
        )