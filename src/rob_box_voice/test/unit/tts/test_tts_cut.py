"""Обрыв TTS-потока после звука даёт одну фразу из факта, не из текста LLM (ADR-0148)."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_core.speech_client import SYSTEM_AGENT  # noqa: E402
from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog import phrases  # noqa: E402
from rob_box_voice.core import tts_cut, tts_stream  # noqa: E402

_CUT_TEXT = "Речь оборвалась, хвост не прозвучал."


@pytest.fixture(autouse=True)
def _fresh_said():
    tts_cut._said.clear()
    yield
    tts_cut._said.clear()


def test_phrase_is_the_catalog_line_not_an_llm_string():
    assert tts_cut.phrase() == _CUT_TEXT
    assert tts_cut.phrase() != phrases.text(kn.SPEECH_CUT_PHRASE)
    assert "хвост не прозвучал" in tts_cut.phrase()


def test_import_survives_core_without_tts_cut_phrase(monkeypatch):
    """Пин v0.7.8: имени TTS_CUT_PHRASE нет — старт tts_node не ImportError."""
    monkeypatch.delattr(kn, "TTS_CUT_PHRASE", raising=False)
    reloaded = importlib.reload(tts_cut)
    try:
        source = Path(reloaded.__file__).read_text(encoding="utf-8")
        assert "from rob_box_dialog.knowledge import TTS_CUT_PHRASE" not in source
        assert "from rob_box_dialog.knowledge import SPEECH_CUT_PHRASE" not in source
        assert reloaded.phrase() == _CUT_TEXT
        assert reloaded.phrase() != phrases.text(kn.SPEECH_CUT_PHRASE)
    finally:
        importlib.reload(tts_cut)


def test_phrase_when_catalog_has_no_tts_cut_key(monkeypatch):
    def missing_key(key, *a, **k):
        raise KeyError(key)

    monkeypatch.delattr(kn, "TTS_CUT_PHRASE", raising=False)
    monkeypatch.setattr(phrases, "text", missing_key)
    assert tts_cut.phrase() == _CUT_TEXT
    assert tts_cut.phrase() != "и так далее."


def test_only_a_broken_stream_after_sound_is_named():
    assert tts_cut.notice(tts_stream.BROKEN, "sid-1") == tts_cut.phrase()
    assert tts_cut.notice("stopped", "sid-2") is None
    assert tts_cut.notice("cancelled", "sid-2") is None
    assert tts_cut.notice("Device unavailable", "sid-2") is None
    assert tts_cut.notice(None, "sid-2") is None
    assert tts_cut.notice(tts_stream.BROKEN, "") is None
    assert tts_cut.notice(tts_stream.BROKEN, None) is None


def test_the_same_reply_is_named_once():
    assert tts_cut.notice(tts_stream.BROKEN, "sid-1") == tts_cut.phrase()
    assert tts_cut.notice(tts_stream.BROKEN, "sid-1") is None
    assert tts_cut.notice(tts_stream.BROKEN, "sid-2") == tts_cut.phrase()


def test_say_publishes_one_system_request_from_the_fact(monkeypatch):
    published = []

    def say(node, utterance, **kwargs):
        published.append((utterance.text, kwargs))
        return "cut-1"

    monkeypatch.setattr("rob_box_core.speech_client.say", say)
    monkeypatch.setattr("rob_box_core.speech_client.prepare", lambda *a, **k: None)
    node = MagicMock()
    assert tts_cut.say(node, tts_stream.BROKEN, "sid-9") is True
    assert tts_cut.say(node, tts_stream.BROKEN, "sid-9") is False
    assert published == [(tts_cut.phrase(), {"agent": SYSTEM_AGENT})]
    node.get_logger().info.assert_called_once()


def test_say_does_not_invent_a_cut_for_a_stop(monkeypatch):
    say = MagicMock()
    monkeypatch.setattr("rob_box_core.speech_client.say", say)
    assert tts_cut.say(MagicMock(), "stopped", "sid-3") is False
    say.assert_not_called()


def test_a_failed_publish_is_logged_not_swallowed(monkeypatch):
    monkeypatch.setattr("rob_box_core.speech_client.prepare", MagicMock(side_effect=RuntimeError("no pub")))
    node = MagicMock()
    assert tts_cut.say(node, tts_stream.BROKEN, "sid-4") is False
    node.get_logger().error.assert_called_once()
    assert "не назван" in node.get_logger().error.call_args.args[0]
