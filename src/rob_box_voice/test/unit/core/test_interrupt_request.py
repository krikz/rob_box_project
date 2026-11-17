"""Перебивание по вейку: stt_node спрашивает, диалог решает (ADR-0150 §6.2, PR-5 #3694; приёмка A12/A16)."""

import math

from rob_box_core.msg_stub import install_dialog_msgs_stub

from rob_box_voice.core.interrupt_request import InterruptOffer

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


def test_offer_waits_for_the_echo_window_to_pass_the_phrase():
    """Ревью #4122: partial с вейком из фразы, которую окно пускает только как стоп, запроса не шлёт, пока текст не стал
    ясным стопом; фраза, пропущенная окном целиком (или без окна), - как раньше."""
    from rob_box_voice.core import echo_window as ew

    offer, sent = _offer()
    window_only_stop = ew.Admission(ew.STOP_ONLY)
    assert not offer.offer("робот расскажи", "u1", window_only_stop)
    assert not offer.offer("робот", "u1", window_only_stop)
    assert offer.offer("робот хватит", "u1", window_only_stop)
    assert offer.offer("робот расскажи", "u2", ew.OPEN) and offer.offer("робот стоп", "u3", None)
    assert [m.text_partial for m in sent] == ["робот хватит", "робот расскажи", "робот стоп"]


def test_offer_unknown_direction_is_nan():
    offer, sent = _offer(doa=None)
    offer.offer("Робот, стоп", "")
    assert math.isnan(sent[0].doa_deg) and sent[0].utterance_id == ""
