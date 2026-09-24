"""Issue #2931 — Yandex STT потерял первое слово «робот».

E2E run 35943180077 (акт 2, n201 ×3): реплика «Робот, привет, давай
знакомиться как следует…» трижды пришла как «привет давай знакомиться …
механик» — без «робот», без дубля и без перестановки. «Робот,» + пауза →
у Yandex отдельный короткий первый сегмент.

Здесь — разные порядки событий v3 с коротким первым сегментом «робот».
Сборщик обязан отдать ВСЕ сегменты по порядку, без потерь и без дублей.

Поля — как в ``yandex/cloud/ai/stt/v3/stt.proto``: ``partial``/``final`` —
``AlternativeUpdate`` (``alternatives[0].text/start_time_ms/end_time_ms``),
индекс final — ``audio_cursors.final_index``, уточнение —
``final_refinement.final_index`` + ``normalized_text``.

Чистые unit-тесты: без ROS2, grpc и protobuf — ответы на SimpleNamespace.
"""

from __future__ import annotations

from types import SimpleNamespace

from rob_box_voice.core.yandex_stt_segments import YandexSegmentCollector

ROBOT = "робот"
REST = "привет давай знакомиться как следует меня зовут саша"
FULL = f"{ROBOT} {REST}"


def _alt(text, start=0, end=0):
    return SimpleNamespace(text=text, start_time_ms=start, end_time_ms=end)


def _final(text, index, start=0, end=0, empty_alternatives=False):
    alts = [] if empty_alternatives else [_alt(text, start, end)]
    return (
        "final",
        SimpleNamespace(
            final=SimpleNamespace(alternatives=alts),
            audio_cursors=SimpleNamespace(final_index=index),
        ),
    )


def _partial(text, start=0, end=0):
    return (
        "partial",
        SimpleNamespace(partial=SimpleNamespace(alternatives=[_alt(text, start, end)])),
    )


def _refinement(text, index):
    return (
        "final_refinement",
        SimpleNamespace(
            final_refinement=SimpleNamespace(
                final_index=index,
                normalized_text=SimpleNamespace(alternatives=[_alt(text)]),
            ),
        ),
    )


def _eou():
    return ("eou_update", SimpleNamespace())


def _collect(events):
    c = YandexSegmentCollector()
    for kind, resp in events:
        c.feed(resp, kind)
    return c


# ── охранные: эти порядки работали и до #2931 ───────────────────────────


def test_short_first_segment_final_without_refinement():
    c = _collect(
        [
            _partial(ROBOT, 0, 600),
            _final(ROBOT, 0, 0, 640),
            _eou(),
            _partial("привет давай", 1900, 2600),
            _final(REST, 1, 1900, 6100),
            _eou(),
        ]
    )
    assert c.text() == FULL
    assert c.segment_count == 2


def test_short_first_segment_refinement_same_index():
    c = _collect(
        [
            _final(ROBOT, 0),
            _refinement("Робот,", 0),
            _final(REST, 1),
            _refinement("Привет, давай знакомиться как следует. Меня зовут Саша.", 1),
        ]
    )
    assert c.text() == "Робот, Привет, давай знакомиться как следует. Меня зовут Саша."


def test_partial_revised_inside_same_segment_is_not_duplicated():
    """partial «робо» и final «робот привет…» — один и тот же кусок аудио."""
    c = _collect([_partial("робо", 0, 600), _final(FULL, 0, 0, 6100)])
    assert c.text() == FULL


def test_partial_without_timings_before_final_is_not_inserted():
    """Без таймингов нельзя отличить «другой сегмент» от «ранней гипотезы
    того же» — не вставляем, чтобы не задвоить."""
    c = _collect([_partial("робот привет"), _final(FULL, 0)])
    assert c.text() == FULL


def test_stale_partial_without_timings_after_final_is_not_appended():
    c = _collect([_final(FULL, 0), _partial("саша")])
    assert c.text() == FULL


# ── порядки, на которых develop терял/переставлял «робот» ────────────────


def test_first_segment_only_in_partial_then_next_final():
    """«робот» сервер показал partial'ом, final сегмента не прислал, а
    следующий final начинается позже по времени — это отдельный сегмент."""
    c = _collect(
        [
            _partial(ROBOT, 0, 640),
            _partial("привет давай", 1900, 2600),
            _final(REST, 0, 1900, 6100),
        ]
    )
    # последний partial перед final — «привет давай» (тот же кусок, что
    # final), «робот» из более раннего partial'а обязан сохраниться
    assert c.text() == FULL


def test_first_segment_partial_then_final_without_intermediate_partial():
    c = _collect([_partial(ROBOT, 0, 640), _final(REST, 0, 1900, 6100)])
    assert c.text() == FULL
    assert c.segment_count == 2


def test_first_segment_final_with_empty_alternatives():
    c = _collect(
        [
            _partial(ROBOT, 0, 640),
            _final("", 0, empty_alternatives=True),
            _final(REST, 1, 1900, 6100),
        ]
    )
    assert c.text() == FULL


def test_first_segment_final_with_empty_text():
    c = _collect([_partial(ROBOT), _final("", 0), _final(REST, 1)])
    assert c.text() == FULL


def test_refinements_after_next_final_with_cursor_after_increment():
    """audio_cursors.final_index «после инкремента» (1, 2), уточнения — 0, 1,
    оба пришли после второго final: develop ставил «робот» в конец."""
    c = _collect(
        [
            _final(ROBOT, 1),
            _final(REST, 2),
            _refinement("Робот,", 0),
            _refinement("Привет, давай знакомиться как следует, меня зовут Саша.", 1),
        ]
    )
    assert c.text() == "Робот, Привет, давай знакомиться как следует, меня зовут Саша."


def test_missing_refinement_of_first_segment_with_shifted_index():
    """Уточнения «робота» нет, у второго индекс совпал с курсором первого:
    develop отдавал «привет … привет …» — дубль и потеря «робот»."""
    c = _collect(
        [
            _final(ROBOT, 1),
            _final(REST, 2),
            _refinement("Привет, давай знакомиться как следует, меня зовут Саша.", 1),
        ]
    )
    assert c.text() == "робот Привет, давай знакомиться как следует, меня зовут Саша."


def test_trailing_partial_without_final_is_kept():
    """Хвост фразы без EOU: partial после последнего final не теряется."""
    c = _collect(
        [
            _final(ROBOT, 0, 0, 640),
            _partial(REST, 1900, 6100),
        ]
    )
    assert c.text() == FULL


def test_trace_shows_stream_events():
    """INFO-лог stt_node: по trace видно, каким путём шёл «робот»."""
    c = _collect(
        [
            _partial(ROBOT, 0, 600),
            _partial(ROBOT, 0, 640),
            _final(ROBOT, 0, 0, 640),
            _eou(),
            _final(REST, 1, 1900, 6100),
            _refinement("Привет, давай знакомиться как следует, меня зовут Саша.", 1),
        ]
    )
    trace = c.trace()
    assert "P×2'робот'@0-640" in trace
    assert "F#0'робот'@0-640" in trace
    assert " E " in trace
    assert "F#1'" in trace
    assert "R#1→#1" in trace
