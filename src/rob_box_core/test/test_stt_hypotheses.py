"""#4017 шаг C1: гипотезы распознавания <-> поля ``Utterance`` (тип из настоящего ``.msg``)."""

from __future__ import annotations

import math
import types

import pytest

from rob_box_core.msg_stub import build_dialog_msgs_module
from rob_box_core.stt_hypotheses import (
    CONFIDENCE_UNKNOWN,
    MAX_ALTERNATIVES,
    NO_HYPOTHESES,
    SttHypotheses,
    clean_confidence,
    fill_hypotheses,
    hypotheses,
    hypotheses_from_msg,
)

M = build_dialog_msgs_module()


def test_utterance_without_the_new_fields_means_unknown_not_zero_confidence():
    """Издатель, не знающий полей (Telegram, inject.sh, e2e), даёт -1 из умолчания ``.msg``, а не 0."""
    msg = M.Utterance(text="робот стоп", source="telegram")
    assert msg.confidence == CONFIDENCE_UNKNOWN == -1.0
    assert list(msg.alternatives) == []
    assert hypotheses_from_msg(msg) == NO_HYPOTHESES


def test_message_without_fields_parses_as_before():
    """Старая заглушка / чужой объект без полей: разбор как до шага C1 — гипотезы неизвестны."""
    old = types.SimpleNamespace(text="робот привет", source="respeaker")
    assert hypotheses_from_msg(old) == NO_HYPOTHESES


@pytest.mark.parametrize(
    "raw, expected",
    [
        (0.87, 0.87),
        (1.0, 1.0),
        (0.123456, 0.1235),
        (0.0, CONFIDENCE_UNKNOWN),  # proto3: «не заполнено»; Yandex v3 «Currently is not used»
        (-1.0, CONFIDENCE_UNKNOWN),
        (1.5, CONFIDENCE_UNKNOWN),
        (math.nan, CONFIDENCE_UNKNOWN),
        (None, CONFIDENCE_UNKNOWN),
        ("abc", CONFIDENCE_UNKNOWN),
    ],
)
def test_clean_confidence_keeps_only_meaningful_values(raw, expected):
    assert clean_confidence(raw) == expected


def test_alternatives_drop_empty_duplicates_and_the_best_text_and_are_capped():
    got = hypotheses(
        0.5,
        ["  ", "Робот, всё глуши", " Робот всох в  глуши", "робот, всё глуши ", "а", "б", "в"],
        text="робот всох в глуши",
    )
    assert got.alternatives == ("Робот, всё глуши", "а", "б")
    assert len(got.alternatives) == MAX_ALTERNATIVES


def test_fill_and_read_back_through_the_real_message_type():
    value = SttHypotheses(0.64, ("робот, стоп", "робот, сто"))
    msg = fill_hypotheses(M.Utterance(text="робот стоп"), value)
    assert (msg.confidence, msg.alternatives) == (0.64, ["робот, стоп", "робот, сто"])
    assert hypotheses_from_msg(msg) == value


def test_float32_wire_value_reads_back_rounded():
    """rclpy отдаёт float32: 0.64 приходит как 0.6399999856948853 — читатель видит 0.64."""
    msg = M.Utterance(text="x", confidence=0.6399999856948853, alternatives=[])
    assert hypotheses_from_msg(msg).confidence == 0.64
