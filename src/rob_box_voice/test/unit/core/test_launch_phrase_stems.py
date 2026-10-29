"""Основы шаблонов запуска (``media_phrases.LAUNCH_PHRASE_STEMS``) не расходятся с самими шаблонами.

По этой таблице аудит честности (``scripts/dialog/honesty_audit.py``, ADR-0150 §14 A3) узнаёт фразу запуска в речи
робота. Правка фразы в ``phrases/ru.yaml`` ломает этот тест, а не аудит молча. Проверка ответа LLM в рантайме по
этим основам («фраза запуска без тула», #3455) снята в ADR-0150 PR-7 ч.7d-4 (#3698).
"""

from __future__ import annotations

from rob_box_voice.core.media_phrases import (
    LAUNCH_PHRASE_STEMS,
    dj_started_text,
    piece_text,
    play_ok_text,
    request_ok_text,
)


def test_every_template_carries_its_stem():
    rendered = [dj_started_text("", "космос"), dj_started_text("диджей Вася", ""), request_ok_text(""),
                play_ok_text("К Элизе"), piece_text({"title": "Toccata", "source": "rtttl"})]
    for stem, text in zip(LAUNCH_PHRASE_STEMS, rendered):
        assert stem in text.lower(), (stem, text)
    assert len(rendered) == len(LAUNCH_PHRASE_STEMS)
