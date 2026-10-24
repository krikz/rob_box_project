"""Перебивание по вейку: stt_node спрашивает, диалог решает (ADR-0150 §6.2, PR-5 #3694; приёмка A12/A16)."""

import math
from types import SimpleNamespace

from rob_box_core.msg_stub import install_dialog_msgs_stub

from rob_box_voice.core.interrupt_request import InterruptGate, InterruptOffer

msgs = install_dialog_msgs_stub()
WAKES = ["роб бокс", "роббокс", "робота", "робот", "робби"]


def _offer(doa=None):
    sent = []
    offer = InterruptOffer(sent.append, msgs.InterruptRequest, lambda: WAKES, lambda: doa, clock=lambda: 100.25)
    return offer, sent


def test_offer_publishes_segment_with_wake_word_once_per_text():
    offer, sent = _offer(doa=42.0)
    assert offer.offer("робот", "u1")
    assert not offer.offer("робот", "u1"), "тот же текст той же фразы второй раз не шлём"
    assert offer.offer("робот хватит", "u1"), "новый partial той же фразы — шлём"
    assert [m.text_partial for m in sent] == ["робот", "робот хватит"]
    msg = sent[-1]
    assert msg.utterance_id == "u1" and msg.doa_deg == 42.0
    assert (msg.stamp.sec, msg.stamp.nanosec) == (100, 250000000)


def test_offer_ignores_phrases_without_wake_word_on_word_boundaries():
    offer, sent = _offer()
    assert not offer.offer("у меня всё работает", "u1")
    assert not offer.offer("системный промт для робота-диджея", "u2")
    assert not offer.offer("", "u3")
    assert sent == []


def test_offer_unknown_direction_is_nan():
    offer, sent = _offer(doa=None)
    offer.offer("Робот, стоп", "")
    assert math.isnan(sent[0].doa_deg) and sent[0].utterance_id == ""


def _gate():
    cancels, logs = [], []
    return InterruptGate(cancels.append, lambda: WAKES, logs.append), cancels, logs


def _req(text, uid="u1"):
    return SimpleNamespace(utterance_id=uid, text_partial=text)


def test_clear_stop_cancels_speech_once_per_utterance():
    gate, cancels, logs = _gate()
    assert gate.on_request(_req("робот хватит")) == "clear"
    assert gate.on_request(_req("робот хватит уже")) == "duplicate"
    assert cancels == ["interrupt:u1"]
    assert "SpeechCancel за" in logs[0]


def test_new_request_and_disputed_stop_do_not_touch_speech():
    gate, cancels, logs = _gate()
    assert gate.on_request(_req("робот расскажи анекдот", "u2")) == "none"
    assert gate.on_request(_req("робот спой песню про стоп", "u3")) == "disputed"
    assert gate.on_request(_req("робот", "u4")) == "none"
    assert cancels == []
    assert any("спорный стоп" in line for line in logs)


def test_stop_without_utterance_id_still_cancels():
    gate, cancels, _ = _gate()
    gate.on_request(_req("Робот, стой", ""))
    gate.on_request(_req("Робот, стой", ""))
    assert cancels == ["interrupt:-", "interrupt:-"]


def test_wake_segment_fires_the_early_addressed_reflex_for_any_verdict():
    """PR-3b (#3690): место бывшего «булька» stt_node - ранний сигнал «услышал» с id фразы, до STT-финала."""
    heard, cancels = [], []
    gate = InterruptGate(cancels.append, lambda: WAKES, lambda _t: None, heard=heard.append)
    assert gate.on_request(_req("робот расскажи анекдот", "u7")) == "none"
    assert gate.on_request(_req("робот хватит", "u8")) == "clear"
    assert heard == ["u7", "u8"] and cancels == ["interrupt:u8"]


def test_segment_without_wake_word_and_repeated_stop_do_not_fire_the_reflex():
    heard = []
    gate = InterruptGate(lambda _r: None, lambda: WAKES, lambda _t: None, heard=heard.append)
    gate.on_request(_req("системный промт для робота-диджея", "u1"))
    assert heard == []
    gate.on_request(_req("робот хватит", "u2"))
    gate.on_request(_req("робот хватит уже", "u2"))          # duplicate: фраза уже открыла ход
    assert heard == ["u2"]
