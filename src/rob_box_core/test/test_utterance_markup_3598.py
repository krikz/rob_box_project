"""Issue #3598 — теги разметки в реплике не произносятся (живой лог 13.10 09:17 UTC).

Финальный текст хода ``'<Текст>У РОББОКСа нет Вагнера…</Текст>'`` и
``speak_text {'text': 'Отлично, на связи. Что делаем?</text>'}`` ушли в TTS
с тегами: ``Utterance.ssml`` их экранировал, ``tts_node`` раскрыл entity.
"""

from __future__ import annotations

import pytest

from rob_box_core.utterance import Utterance, strip_markup_tags

FINAL_TEXT = (
    "<Текст>У РОББОКСа нет Вагнера в библиотеке. Могу сгенерировать "
    "что-нибудь мрачное и эпичное в духе классики — хочешь?</Текст>"
)
SPEAK_TEXT = "Отлично, на связи. Что делаем?</text>"


def test_final_text_tags_removed_from_ssml():
    ssml = Utterance(text=FINAL_TEXT).ssml
    assert "Текст&gt;" not in ssml and "&lt;" not in ssml
    assert ssml.startswith("<speak>У РОББОКСа нет Вагнера")
    assert ssml.endswith("хочешь?</speak>")


def test_speak_text_closing_tag_removed():
    assert Utterance(text=SPEAK_TEXT).ssml == "<speak>Отлично, на связи. Что делаем?</speak>"


@pytest.mark.parametrize(
    "text, expected",
    [
        ('<music_state playing="no" last_ended="finished"/> тишина', "тишина"),
        ("<br/>привет", "привет"),
        ("x<y & z>0", "x<y & z>0"),
        ("2 < 3 и 5 > 4", "2 < 3 и 5 > 4"),
        ("<3 люблю", "<3 люблю"),
        ("", ""),
    ],
)
def test_strip_markup_tags(text, expected):
    assert strip_markup_tags(text) == expected


def test_plain_text_still_escaped():
    assert Utterance(text="x<y & z>0").ssml == "<speak>x&lt;y &amp; z&gt;0</speak>"
