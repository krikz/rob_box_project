"""SpeechReceipts: квитанция /say по SpeechEvent той же реплики (#3727)."""

from __future__ import annotations

import unittest

from rob_box_telegram.speech_receipts import KIND_CANCELLED, KIND_FINISHED, KIND_REJECTED, SpeechReceipts


class SpeechReceiptsTest(unittest.TestCase):
    def setUp(self):
        self.now = [0.0]
        self.receipts = SpeechReceipts(clock=lambda: self.now[0], timeout_s=100.0)
        self.receipts.expect("s1", 42)

    def test_finished_says_spoken_with_seconds(self):
        chat, text = self.receipts.on_event(KIND_FINISHED, "s1", seconds=12.4)
        assert chat == 42 and text == "Озвучил (12 с)."

    def test_rejected_reason_gets_its_own_phrase(self):
        chat, text = self.receipts.on_event(KIND_REJECTED, "s1", reason="unsupported_script")
        assert chat == 42 and text.startswith("Не озвучил:") and "не читает" in text

    def test_rejected_unknown_reason_falls_back_to_generic_phrase(self):
        _, text = self.receipts.on_event(KIND_REJECTED, "s1", reason="something_new")
        assert text == "Не озвучил: динамики заняты."

    def test_cancelled(self):
        _, text = self.receipts.on_event(KIND_CANCELLED, "s1")
        assert text == "Не озвучил: перебили."

    def test_non_terminal_and_foreign_events_do_not_close_the_receipt(self):
        assert self.receipts.on_event(2, "s1") is None  # started
        assert self.receipts.on_event(KIND_FINISHED, "other") is None
        assert self.receipts.on_event(KIND_FINISHED, "s1") is not None  # всё ещё ждали

    def test_receipt_is_given_once(self):
        assert self.receipts.on_event(KIND_FINISHED, "s1") is not None
        assert self.receipts.on_event(KIND_FINISHED, "s1") is None

    def test_silence_becomes_an_honest_dont_know(self):
        assert self.receipts.expired() == []
        self.now[0] = 100.0
        assert self.receipts.expired() == [(42, "Не знаю, прозвучало ли: итога от динамиков нет.")]
        assert self.receipts.expired() == []
