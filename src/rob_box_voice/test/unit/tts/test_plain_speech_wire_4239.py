"""ADR-0160 §2.3 (PR-2, #4239): реплика несёт голый текст, SSML-конверт собирает только адаптер Silero.

Путь: ``Utterance`` → ``speech_client.build_request`` (``SpeechRequest.text`` — голый текст) →
``speech_owner.chunk_from_request`` (чанк ``{"text": …}``) → ``tts_text_clean.speech_text`` (теги модели, ``[…]``,
Markdown) → провайдер. ``<speak>`` с экранированием — только ``tts_ssml.silero_ssml`` в адаптере ``SileroTTSProvider``.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_core import speech_client  # noqa: E402
from rob_box_core.utterance import Utterance  # noqa: E402
from rob_box_voice.core.speech_owner import chunk_from_request  # noqa: E402
from rob_box_voice.tts_providers.silero import SileroTTSProvider  # noqa: E402
from rob_box_voice.tts_ssml import silero_ssml  # noqa: E402
from rob_box_voice.tts_text_clean import speech_text, strip_markup_tags  # noqa: E402

TRICKY = "Батарея < 20% & сигнал > нормы"
VERSE = "Первая строка куплета\nВторая строка куплета"


class _Node:
    """Нода без ROS для ``build_request``: часы и словарь публикаторов."""

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=0, nanosec=0)))


class _Req(SimpleNamespace):
    pass


@pytest.fixture
def fake_msgs(monkeypatch):
    monkeypatch.setattr(speech_client, "_msgs", lambda: SimpleNamespace(SpeechRequest=_Req))


# ── провод: голый текст ─────────────────────────────────────────────────────


@pytest.mark.parametrize("text", [TRICKY, VERSE, "Привет, мир!"])
def test_speech_request_text_is_plain_text_as_is(fake_msgs, text):
    req = speech_client.build_request(_Node(), Utterance(text=text), agent="personality")
    assert req.text == text
    assert "<speak" not in req.text and "&amp;" not in req.text


@pytest.mark.parametrize("text", [TRICKY, VERSE])
def test_chunk_from_request_keeps_plain_text(fake_msgs, text):
    req = speech_client.build_request(_Node(), Utterance(text=text), agent="personality")
    req.sink, req.priority = "", 0
    chunk = chunk_from_request(req)
    assert chunk["text"] == text and "ssml" not in chunk


def test_build_request_has_no_ssml_override():
    import inspect

    assert "ssml" not in inspect.signature(speech_client.build_request).parameters


# ── очистка перед синтезом ──────────────────────────────────────────────────


@pytest.mark.parametrize(
    "text, expected",
    [
        ("<Текст>У РОББОКСа нет Вагнера. Хочешь?</Текст>", "У РОББОКСа нет Вагнера. Хочешь?"),  # #3598, лог 13.10
        ("Отлично, на связи. Что делаем?</text>", "Отлично, на связи. Что делаем?"),
        ('<music_state playing="no" last_ended="finished"/> тишина', "тишина"),
        ("<br/>привет", "привет"),
        ("x<y & z>0", "x<y & z>0"),
        ("2 < 3 и 5 > 4", "2 < 3 и 5 > 4"),
        ("<3 люблю", "<3 люблю"),
        ("строка <#1.06#> строка", "строка <#1.06#> строка"),  # метка паузы рэпа — не тег
        ("", ""),
    ],
)
def test_strip_markup_tags(text, expected):
    assert strip_markup_tags(text) == expected


def test_speech_text_keeps_special_chars_and_lines():
    assert speech_text(TRICKY) == TRICKY
    assert speech_text(VERSE) == VERSE


# ── конверт Silero ──────────────────────────────────────────────────────────


@pytest.mark.parametrize("text", [TRICKY, "AT&T и Tom & Jerry", "ёлка × ÷ ∞", ""])
def test_silero_ssml_is_valid_xml_and_keeps_every_char(text):
    root = ET.fromstring(silero_ssml(text))
    assert root.tag == "speak"
    (prosody,) = list(root)
    assert prosody.get("pitch") == "medium"
    assert (prosody.text or "") == text


class _Silero:
    def __init__(self):
        self.ssml: list[str] = []

    def apply_tts(self, *, ssml_text: str, **kwargs):
        ET.fromstring(ssml_text)  # настоящий Silero разбирает XML: сырой & или < его роняет
        self.ssml.append(ssml_text)
        return SimpleNamespace(numpy=lambda: np.zeros(4, dtype=np.float32))


def test_synthesize_silero_wraps_plain_text_with_escaping():
    model = _Silero()
    node = SileroTTSProvider(logger=SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None))
    node.model = model

    node.synthesize_np(TRICKY)

    assert model.ssml == ['<speak><prosody pitch="medium">Батарея &lt; 20% &amp; сигнал &gt; нормы</prosody></speak>']
