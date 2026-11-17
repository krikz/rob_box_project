"""issue #2135: сегментация wake-потока шлема (кадры 20 мс → фраза).

Дефект, который эти тесты фиксируют: мост публиковал КАЖДЫЙ 20мс-кадр
(640 байт) отдельным ``AudioData``, а ``stt_node`` запускает полный цикл
распознавания на каждое сообщение — распознавание 0.02 с всегда возвращает
пусто, поэтому вейк «ТАРС» из шлема не мог сработать ни разу (live-лог
робота 2026-09-08).

issue #2199: эти тесты переведены с ``rob_box_quest.core.wake_segmenter``
(legacy shim) на ``rob_box_core.speech_segmentation.PhraseSegmenter``.
Контракт сегментатора тот же, поэтому старые проверки остаются в силе,
но источник истины теперь общий для wake / robot_voice / будущих путей.
"""

import pytest

from rob_box_core.speech_segmentation import (
    DEFAULT_WAKE_CONFIG,
    PhraseSegmenter,
)

FRAME = b"\x11\x22" * 320  # 640 байт = 320 сэмплов int16 = 20 мс @ 16 кГц
FRAME_PERIOD_S = 0.02

WAKE_PHRASE_GAP_TIMEOUT_S = DEFAULT_WAKE_CONFIG.gap_timeout_s
WAKE_PHRASE_MAX_BYTES = DEFAULT_WAKE_CONFIG.max_phrase_bytes
WAKE_PHRASE_MIN_BYTES = DEFAULT_WAKE_CONFIG.min_phrase_bytes


def _feed(seg: PhraseSegmenter, n_frames: int, start: float = 1000.0):
    """Скормить n кадров подряд «в реальном темпе». Вернуть (фразы, время)."""
    phrases = []
    now = start
    for _ in range(n_frames):
        out = seg.add_frame(FRAME, now)
        if out is not None:
            phrases.append(out)
        now += FRAME_PERIOD_S
    return phrases, now


def test_frames_then_pause_produce_exactly_one_phrase():
    """DoD #2135: последовательность кадров + пауза = ровно одна AudioData
    с суммарным payload (а не 50 сообщений по 640 байт в секунду)."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    phrases, now = _feed(seg, 50)  # 50 кадров = 1.0 с речи
    assert phrases == [], "фраза не должна закрываться, пока идут кадры"

    closed = seg.tick(now + WAKE_PHRASE_GAP_TIMEOUT_S)
    assert closed is not None
    assert closed == FRAME * 50
    assert len(closed) == 32000  # 1.0 с @ 16 кГц int16
    assert seg.buffered_bytes == 0


def test_no_phrase_before_gap_timeout():
    """Пауза короче таймаута — это межслоговая тишина, фразу не рвём."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    _phrases, now = _feed(seg, 30)
    assert seg.tick(now + WAKE_PHRASE_GAP_TIMEOUT_S / 2) is None
    assert seg.buffered_bytes == 30 * len(FRAME)


def test_repeated_ticks_after_flush_are_silent():
    """Пустой буфер не должен «дозакрываться» повторными тиками таймера
    (30 Гц: тиков между фразами тысячи)."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    _phrases, now = _feed(seg, 50)
    assert seg.tick(now + WAKE_PHRASE_GAP_TIMEOUT_S) is not None
    for i in range(10):
        assert seg.tick(now + WAKE_PHRASE_GAP_TIMEOUT_S + i) is None


def test_two_phrases_separated_by_gap_are_not_glued():
    """Вторая реплика после паузы — отдельная фраза, а не хвост первой."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    _phrases, now = _feed(seg, 40)
    first = seg.tick(now + WAKE_PHRASE_GAP_TIMEOUT_S)
    assert first == FRAME * 40

    phrases2, now2 = _feed(seg, 20, start=now + 5.0)
    assert phrases2 == []
    second = seg.tick(now2 + WAKE_PHRASE_GAP_TIMEOUT_S)
    assert second == FRAME * 20


def test_gap_detected_on_next_frame_closes_previous_phrase():
    """Если следующая реплика началась раньше, чем сработал таймер, фразу
    закрывает сам кадр — он открывает новую, а не продолжает старую."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    _phrases, now = _feed(seg, 40)
    late = seg.add_frame(FRAME, now + WAKE_PHRASE_GAP_TIMEOUT_S + 0.1)
    assert late == FRAME * 40  # старая фраза
    assert seg.buffered_bytes == len(FRAME)  # новая уже началась


def test_buffer_cap_flushes_and_does_not_grow_unbounded():
    """Потолок буфера: «залипший» клиентский VAD не должен съесть память —
    фраза режется по WAKE_PHRASE_MAX_BYTES и уходит как есть."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    frames_to_cap = WAKE_PHRASE_MAX_BYTES // len(FRAME)
    phrases, now = _feed(seg, frames_to_cap)
    assert len(phrases) == 1
    assert len(phrases[0]) == WAKE_PHRASE_MAX_BYTES
    assert seg.truncated_phrases == 1
    assert seg.buffered_bytes == 0

    # Поток продолжается — буфер снова копится с нуля, а не поверх старого.
    _phrases2, _now2 = _feed(seg, 10, start=now)
    assert seg.buffered_bytes == 10 * len(FRAME)


def test_buffer_never_exceeds_cap_on_long_stream():
    """Инвариант потолка на длинном потоке: 60 с непрерывных кадров."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    now = 500.0
    for _ in range(3000):  # 3000 * 20 мс = 60 с
        seg.add_frame(FRAME, now)
        now += FRAME_PERIOD_S
        assert seg.buffered_bytes <= WAKE_PHRASE_MAX_BYTES


def test_reset_drops_unfinished_phrase():
    """Разрыв WS-сессии: недособранная фраза выбрасывается, а не склеивается
    с речью следующего оператора."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    _phrases, now = _feed(seg, 40)
    dropped = seg.reset()
    assert dropped == 40 * len(FRAME)
    assert seg.buffered_bytes == 0
    # Ни таймер, ни новый кадр не должны воскресить выброшенное.
    assert seg.tick(now + WAKE_PHRASE_GAP_TIMEOUT_S) is None

    phrases2, now2 = _feed(seg, 30, start=now + 10.0)
    assert phrases2 == []
    assert seg.tick(now2 + WAKE_PHRASE_GAP_TIMEOUT_S) == FRAME * 30


def test_reset_on_empty_buffer_reports_zero():
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    assert seg.reset() == 0
    assert seg.tick(1234.0) is None


def test_single_frame_blip_is_dropped_not_published():
    """Ровно тот сегмент, который ломал вейк: 0.02 с / 640 байт. В нём не
    помещается «ТАРС» — в STT он не уходит вовсе."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    assert seg.add_frame(FRAME, 100.0) is None
    assert seg.tick(100.0 + WAKE_PHRASE_GAP_TIMEOUT_S) is None
    assert seg.dropped_short_phrases == 1
    assert seg.buffered_bytes == 0


def test_shortest_publishable_phrase_crosses_min_threshold():
    """Граница min_bytes: первый сегмент, дотянувший до порога, уже уходит
    в STT (а всё, что короче — блип VAD, см. предыдущий тест)."""
    frames_needed = -(-WAKE_PHRASE_MIN_BYTES // len(FRAME))  # ceil
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    _phrases, now = _feed(seg, frames_needed)
    closed = seg.tick(now + WAKE_PHRASE_GAP_TIMEOUT_S)
    assert closed is not None
    assert len(closed) >= WAKE_PHRASE_MIN_BYTES
    assert seg.dropped_short_phrases == 0

    # На один кадр меньше — уже блип, наружу ничего не идёт.
    seg2 = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    _p2, now2 = _feed(seg2, frames_needed - 1)
    assert seg2.tick(now2 + WAKE_PHRASE_GAP_TIMEOUT_S) is None
    assert seg2.dropped_short_phrases == 1


@pytest.mark.parametrize("extra_s", [0.01, 1.0, 30.0])
def test_any_gap_over_timeout_closes_phrase(extra_s):
    """Пауза длиннее таймаута закрывает фразу, сколько бы она ни длилась."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    _phrases, now = _feed(seg, 50)
    assert seg.tick(now + WAKE_PHRASE_GAP_TIMEOUT_S + extra_s) is not None
