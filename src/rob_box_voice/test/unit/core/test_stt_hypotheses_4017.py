"""#4017 шаг C1: адаптер Yandex STT v3 -> гипотезы фразы -> поля ``Utterance``.

Ответы — ``SimpleNamespace`` по ``yandex/cloud/ai/stt/v3/stt.proto``: ``Alternative{text, start_time_ms, end_time_ms,
confidence}``; ``confidence`` в proto описан как «Currently is not used» (на проводе 0 — значение proto3 по умолчанию).
"""

from __future__ import annotations

from types import SimpleNamespace

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_core.stt_hypotheses import CONFIDENCE_UNKNOWN, NO_HYPOTHESES, SttHypotheses
from rob_box_voice.core.stt_utterance import fill_respeaker_utterance, phrase_hypotheses
from rob_box_voice.core.yandex_stt_segments import YandexSegmentCollector

M = install_dialog_msgs_stub()


def _alt(text, confidence=0.0, start=0, end=0):
    return SimpleNamespace(text=text, confidence=confidence, start_time_ms=start, end_time_ms=end)


def _final(index, *alts):
    return SimpleNamespace(
        final=SimpleNamespace(alternatives=list(alts)), audio_cursors=SimpleNamespace(final_index=index)
    )


def _partial(*alts):
    return SimpleNamespace(partial=SimpleNamespace(alternatives=list(alts)))


def _collect(*events):
    c = YandexSegmentCollector()
    for kind, resp in events:
        c.feed(resp, kind)
    return c


def test_yandex_zero_confidence_is_unknown():
    """То, что v3 шлёт сегодня (поле не используется → 0): уверенность -1, альтернатив нет."""
    c = _collect(("final", _final(0, _alt("робот стоп"))), ("eou_update", SimpleNamespace()))
    assert c.text() == "робот стоп"
    assert c.hypotheses() == NO_HYPOTHESES
    assert c.final_confidences == (0.0,)


def test_known_confidence_of_one_segment_is_taken():
    c = _collect(("final", _final(0, _alt("робот стоп", 0.91))))
    assert c.hypotheses() == SttHypotheses(0.91, ())


def test_phrase_confidence_is_the_weakest_segment():
    c = _collect(
        ("final", _final(0, _alt("робот", 0.95, 0, 500))), ("final", _final(1, _alt("всё глуши", 0.42, 1700, 2500)))
    )
    assert c.text() == "робот всё глуши"
    assert c.hypotheses().confidence == 0.42


def test_one_segment_without_confidence_makes_the_phrase_unknown():
    c = _collect(("final", _final(0, _alt("робот", 0.95))), ("final", _final(1, _alt("всё глуши", 0.0))))
    assert c.hypotheses().confidence == CONFIDENCE_UNKNOWN


def test_text_promoted_from_partial_has_no_confidence():
    """Пустой final, текст взят из partial (#2931): сервер этот текст не утвердил — уверенность неизвестна."""
    c = _collect(("partial", _partial(_alt("робот стоп", 0.9))), ("final", _final(0, _alt("", 0.9))))
    assert c.text() == "робот стоп"
    assert c.hypotheses().confidence == CONFIDENCE_UNKNOWN
    assert c.final_confidences == ()


def test_tail_partial_after_the_last_final_makes_the_phrase_unknown():
    c = _collect(("final", _final(0, _alt("робот", 0.95, 0, 500))), ("partial", _partial(_alt("стоп", 0.0, 900, 1300))))
    assert c.text() == "робот стоп"
    assert c.hypotheses().confidence == CONFIDENCE_UNKNOWN


def test_other_alternatives_are_substituted_into_the_phrase():
    c = _collect(
        ("final", _final(0, _alt("робот", 0.9, 0, 500))),
        ("final", _final(1, _alt("всох в глуши", 0.4, 1700, 2500), _alt("всё глуши"), _alt("всох в глуши"))),
    )
    assert c.hypotheses() == SttHypotheses(0.4, ("робот всё глуши",))


def test_empty_stream_gives_no_hypotheses():
    assert YandexSegmentCollector().hypotheses() == NO_HYPOTHESES


def test_respeaker_utterance_carries_the_hypotheses():
    msg = fill_respeaker_utterance(
        M.Utterance(),
        text="робот всох в глуши",
        utterance_id="u",
        duration_s=1.0,
        now_s=10.0,
        provider="yandex",
        hypotheses=SttHypotheses(0.4, ("робот всё глуши",)),
    )
    assert (msg.confidence, list(msg.alternatives)) == (0.4, ["робот всё глуши"])


def test_respeaker_utterance_without_hypotheses_says_unknown():
    msg = fill_respeaker_utterance(M.Utterance(), text="а", utterance_id="u", duration_s=1.0, now_s=10.0)
    assert (msg.confidence, list(msg.alternatives)) == (-1.0, [])


def test_hypotheses_of_yandex_do_not_leak_to_a_fallback_phrase():
    """Yandex ответил, но фразу принял фолбек (Vosk, MiniMax): гипотезы Yandex не про этот текст."""
    yandex = SttHypotheses(0.8, ("x",))
    assert phrase_hypotheses("yandex", yandex) == yandex
    assert phrase_hypotheses("vosk", yandex) == NO_HYPOTHESES
    assert phrase_hypotheses("minimax", yandex) == NO_HYPOTHESES
    assert phrase_hypotheses("", yandex) == NO_HYPOTHESES
