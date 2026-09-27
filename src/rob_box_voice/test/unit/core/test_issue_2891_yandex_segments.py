"""Unit-тесты :mod:`rob_box_voice.core.yandex_stt_segments` (issue #2891).

Yandex STT v3 шлёт ``final`` (индекс — ``audio_cursors.final_index``) и
``final_refinement`` (индекс — ``final_refinement.final_index``) на каждый
сегмент фразы. Сборщик обязан склеить все сегменты по порядку и не
задвоить сегмент, у которого пришли и final, и refinement.

Чистые unit-тесты: без ROS2, grpc и protobuf — ответы на SimpleNamespace.
"""

from __future__ import annotations

from types import SimpleNamespace

from rob_box_voice.core.yandex_stt_segments import YandexSegmentCollector


def _final(text, index):
    alts = [SimpleNamespace(text=text)] if text else []
    return SimpleNamespace(
        final=SimpleNamespace(alternatives=alts),
        audio_cursors=SimpleNamespace(final_index=index),
    )


def _refinement(text, index):
    alts = [SimpleNamespace(text=text)] if text else []
    return SimpleNamespace(
        final_refinement=SimpleNamespace(
            final_index=index,
            normalized_text=SimpleNamespace(alternatives=alts),
        ),
    )


def _collect(events):
    c = YandexSegmentCollector()
    for kind, resp in events:
        c.feed(resp, kind)
    return c


def test_two_segments_final_and_refinement_each():
    c = _collect(
        [
            ("final", _final("робот здравствуй", 0)),
            ("final_refinement", _refinement("Робот, здравствуй.", 0)),
            ("final", _final("я саша", 1)),
            ("final_refinement", _refinement("Я Саша.", 1)),
        ]
    )
    assert c.text() == "Робот, здравствуй. Я Саша."
    assert c.segment_count == 2


def test_last_segment_without_refinement_uses_final():
    c = _collect(
        [
            ("final", _final("робот здравствуй", 0)),
            ("final_refinement", _refinement("Робот, здравствуй.", 0)),
            ("final", _final("я саша", 1)),
        ]
    )
    assert c.text() == "Робот, здравствуй. я саша"


def test_single_segment_as_before():
    c = _collect(
        [
            ("final", _final("робот привет", 0)),
            ("final_refinement", _refinement("Робот, привет.", 0)),
        ]
    )
    assert c.text() == "Робот, привет."
    assert c.segment_count == 1


def test_finals_only_are_joined():
    """Нормализация выключена — уточнений нет вовсе."""
    c = _collect(
        [
            ("final", _final("робот здравствуй", 0)),
            ("final", _final("я саша", 1)),
        ]
    )
    assert c.text() == "робот здравствуй я саша"


def test_refinement_arriving_after_next_final_matches_by_index():
    c = _collect(
        [
            ("final", _final("робот здравствуй", 0)),
            ("final", _final("я саша", 1)),
            ("final_refinement", _refinement("Робот, здравствуй.", 0)),
            ("final_refinement", _refinement("Я Саша.", 1)),
        ]
    )
    assert c.text() == "Робот, здравствуй. Я Саша."


def test_refinement_index_shifted_by_one_is_not_duplicated():
    """audio_cursors.final_index «после инкремента» — сегмент не задваивается."""
    c = _collect(
        [
            ("final", _final("робот здравствуй", 1)),
            ("final_refinement", _refinement("Робот, здравствуй.", 0)),
            ("final", _final("я саша", 2)),
            ("final_refinement", _refinement("Я Саша.", 1)),
        ]
    )
    assert c.text() == "Робот, здравствуй. Я Саша."


def test_empty_final_segment_is_skipped():
    c = _collect(
        [
            ("final", _final("", 0)),
            ("final", _final("робот стоп", 1)),
        ]
    )
    assert c.text() == "робот стоп"
    assert c.segment_count == 1


def test_empty_refinement_keeps_final_text():
    c = _collect(
        [
            ("final", _final("робот стоп", 0)),
            ("final_refinement", _refinement("", 0)),
        ]
    )
    assert c.text() == "робот стоп"


def test_nothing_gives_none():
    assert YandexSegmentCollector().text() is None


def test_other_events_ignored():
    c = YandexSegmentCollector()
    c.feed(SimpleNamespace(), "partial")
    c.feed(SimpleNamespace(), "eou_update")
    assert c.text() is None
