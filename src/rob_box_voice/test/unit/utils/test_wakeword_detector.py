#!/usr/bin/env python3
"""
test_wakeword_detector.py - Unit tests for WakeWordDetector and audio_node DOA gate logic

Tests:
    - WakeWordDetector no-op mode (openWakeWord not installed)
    - WakeWordDetector mock inference
    - WakeWordDetector score threshold
    - WakeWordDetector cooldown
    - AudioNode._should_accept_phrase() all gate scenarios
"""

import time
import types
import unittest
from unittest.mock import MagicMock, patch


# ---------------------------------------------------------------------------
# WakeWordDetector tests
# ---------------------------------------------------------------------------

class TestWakeWordDetectorNoOp(unittest.TestCase):
    """When openWakeWord is not installed, detector is a safe no-op."""

    def _make_detector(self, **kwargs):
        """Import detector with OWW patched as unavailable."""
        with patch(
            "rob_box_voice.utils.wakeword_detector._OWW_AVAILABLE", False
        ):
            from rob_box_voice.utils.wakeword_detector import WakeWordDetector
            return WakeWordDetector(**kwargs)

    def test_available_false_when_oww_missing(self):
        with patch("rob_box_voice.utils.wakeword_detector._OWW_AVAILABLE", False):
            from rob_box_voice.utils.wakeword_detector import WakeWordDetector
            d = WakeWordDetector()
        self.assertFalse(d.available)

    def test_process_chunk_returns_none_when_unavailable(self):
        with patch("rob_box_voice.utils.wakeword_detector._OWW_AVAILABLE", False):
            from rob_box_voice.utils.wakeword_detector import WakeWordDetector
            d = WakeWordDetector()
        result = d.process_chunk(b"\x00" * 1024)
        self.assertIsNone(result)

    def test_stop_start_do_not_raise(self):
        with patch("rob_box_voice.utils.wakeword_detector._OWW_AVAILABLE", False):
            from rob_box_voice.utils.wakeword_detector import WakeWordDetector
            d = WakeWordDetector()
        d.stop()
        d.start()


class TestWakeWordDetectorMock(unittest.TestCase):
    """Tests with a mock OWWModel that returns controllable scores."""

    def _make_detector_with_mock_model(self, model_score: float = 0.0):
        """Create a WakeWordDetector with a mock OWWModel."""
        mock_model = MagicMock()
        mock_model.predict.return_value = {"robbot": model_score}

        fake_oww_module = types.ModuleType("openwakeword")
        fake_model_module = types.ModuleType("openwakeword.model")
        fake_oww_module.model = fake_model_module
        fake_model_module.Model = MagicMock(return_value=mock_model)

        with patch.dict(
            "sys.modules",
            {
                "openwakeword": fake_oww_module,
                "openwakeword.model": fake_model_module,
            },
        ):
            with patch("rob_box_voice.utils.wakeword_detector._OWW_AVAILABLE", True):
                from rob_box_voice.utils.wakeword_detector import WakeWordDetector

                # Re-import to pick up patched _OWW_AVAILABLE
                import importlib
                import rob_box_voice.utils.wakeword_detector as m
                m._OWW_AVAILABLE = True
                m.OWWModel = fake_model_module.Model

                d = WakeWordDetector.__new__(WakeWordDetector)
                d.model_paths = []
                d.threshold = 0.5
                d.callback = None
                d.enabled = True
                d._model = mock_model
                d._last_detection_time = 0.0
                d._cooldown_sec = 1.0
                return d, mock_model

    def test_returns_result_when_score_above_threshold(self):
        d, mock_model = self._make_detector_with_mock_model(0.9)
        mock_model.predict.return_value = {"robbot": 0.9}
        result = d.process_chunk(b"\x00" * 1024)
        self.assertIsNotNone(result)
        self.assertEqual(result.model_name, "robbot")
        self.assertAlmostEqual(result.score, 0.9, places=5)

    def test_returns_none_when_score_below_threshold(self):
        d, mock_model = self._make_detector_with_mock_model(0.3)
        mock_model.predict.return_value = {"robbot": 0.3}
        result = d.process_chunk(b"\x00" * 1024)
        self.assertIsNone(result)

    def test_cooldown_suppresses_repeated_detection(self):
        d, mock_model = self._make_detector_with_mock_model(0.9)
        mock_model.predict.return_value = {"robbot": 0.9}
        r1 = d.process_chunk(b"\x00" * 1024)
        r2 = d.process_chunk(b"\x00" * 1024)  # Within cooldown
        self.assertIsNotNone(r1)
        self.assertIsNone(r2)

    def test_cooldown_resets_after_interval(self):
        d, mock_model = self._make_detector_with_mock_model(0.9)
        mock_model.predict.return_value = {"robbot": 0.9}
        d.process_chunk(b"\x00" * 1024)
        # Fake cooldown expiry
        d._last_detection_time = time.time() - 2.0
        r2 = d.process_chunk(b"\x00" * 1024)
        self.assertIsNotNone(r2)

    def test_disabled_returns_none(self):
        d, mock_model = self._make_detector_with_mock_model(0.9)
        mock_model.predict.return_value = {"robbot": 0.9}
        d.stop()
        result = d.process_chunk(b"\x00" * 1024)
        self.assertIsNone(result)

    def test_callback_invoked_on_detection(self):
        d, mock_model = self._make_detector_with_mock_model(0.9)
        mock_model.predict.return_value = {"robbot": 0.9}
        cb = MagicMock()
        d.callback = cb
        d.process_chunk(b"\x00" * 1024)
        cb.assert_called_once()

    def test_reset_clears_cooldown(self):
        d, mock_model = self._make_detector_with_mock_model(0.9)
        mock_model.predict.return_value = {"robbot": 0.9}
        d.process_chunk(b"\x00" * 1024)
        d.reset()
        r2 = d.process_chunk(b"\x00" * 1024)
        self.assertIsNotNone(r2)


# ---------------------------------------------------------------------------
# AudioNode._should_accept_phrase tests (pure logic, no ROS)
# ---------------------------------------------------------------------------

class FakeAudioNode:
    """
    Minimal stub that reproduces _should_accept_phrase() logic
    without requiring a full ROS node. Mirrors audio_node.py exactly.
    """

    def __init__(
        self,
        use_wake_word_engine: bool = True,
        doa_lock_enabled: bool = True,
        doa_tolerance_degrees: int = 45,
        wake_word_timeout_sec: float = 8.0,
        dialogue_window_seconds: float = 30.0,
        wakeword_detector_available: bool = True,
    ):
        self._use_wake_word_engine = use_wake_word_engine
        self._doa_lock_enabled = doa_lock_enabled
        self._doa_tolerance_degrees = doa_tolerance_degrees
        self._wake_word_timeout_sec = wake_word_timeout_sec
        self._dialogue_window_seconds = dialogue_window_seconds

        self._last_wake_word_time: float = 0.0
        self._locked_doa_angle = None
        self._last_dialogue_response_time: float = 0.0
        self._in_active_dialogue: bool = False

        mock_det = MagicMock()
        mock_det.available = wakeword_detector_available
        self._wakeword_detector = mock_det

    def _should_accept_phrase(self, phrase_doa: int) -> bool:
        """Copy of audio_node._should_accept_phrase (kept in sync)."""
        if not self._use_wake_word_engine:
            return True
        if self._wakeword_detector is None or not self._wakeword_detector.available:
            return True

        now = time.time()

        if now - self._last_wake_word_time <= self._wake_word_timeout_sec:
            if self._locked_doa_angle is None:
                self._locked_doa_angle = phrase_doa
            return True

        def _doa_matches(current: int, locked: int) -> bool:
            if not self._doa_lock_enabled:
                return True
            diff = abs(current - locked)
            diff = min(diff, 360 - diff)
            return diff <= self._doa_tolerance_degrees

        if self._locked_doa_angle is None:
            return False

        if self._in_active_dialogue:
            return _doa_matches(phrase_doa, self._locked_doa_angle)

        if now - self._last_dialogue_response_time <= self._dialogue_window_seconds:
            return _doa_matches(phrase_doa, self._locked_doa_angle)

        return False


class TestShouldAcceptPhrase(unittest.TestCase):
    """Tests for AudioNode._should_accept_phrase() DOA gate logic."""

    # --- Engine disabled (legacy passthrough) --------------------

    def test_passthrough_when_engine_disabled(self):
        node = FakeAudioNode(use_wake_word_engine=False)
        self.assertTrue(node._should_accept_phrase(180))

    # --- Engine enabled but unavailable (graceful degradation) ---

    def test_passthrough_when_detector_unavailable(self):
        node = FakeAudioNode(wakeword_detector_available=False)
        self.assertTrue(node._should_accept_phrase(90))

    # --- Wake word recently fired --------------------------------

    def test_accept_immediately_after_wake_word(self):
        node = FakeAudioNode()
        node._last_wake_word_time = time.time()  # Just fired
        self.assertTrue(node._should_accept_phrase(120))

    def test_locks_doa_on_first_phrase_after_wake_word(self):
        node = FakeAudioNode()
        node._last_wake_word_time = time.time()
        node._locked_doa_angle = None
        node._should_accept_phrase(200)
        self.assertEqual(node._locked_doa_angle, 200)

    def test_reject_after_wake_word_timeout_no_lock(self):
        node = FakeAudioNode()
        node._last_wake_word_time = 0.0  # Expired long ago
        node._locked_doa_angle = None   # No lock established
        self.assertFalse(node._should_accept_phrase(180))

    def test_reject_after_wake_word_timeout_expired(self):
        node = FakeAudioNode(wake_word_timeout_sec=5.0)
        node._last_wake_word_time = time.time() - 10.0  # 10s ago > 5s timeout
        node._locked_doa_angle = 90
        # Not in dialogue, dialogue window also expired
        node._last_dialogue_response_time = 0.0
        self.assertFalse(node._should_accept_phrase(90))

    # --- Active dialogue DOA matching ----------------------------

    def test_accept_same_doa_in_active_dialogue(self):
        node = FakeAudioNode()
        node._last_wake_word_time = 0.0
        node._locked_doa_angle = 90
        node._in_active_dialogue = True
        self.assertTrue(node._should_accept_phrase(100))  # 100-90=10 < 45

    def test_reject_different_doa_in_active_dialogue(self):
        node = FakeAudioNode()
        node._last_wake_word_time = 0.0
        node._locked_doa_angle = 90
        node._in_active_dialogue = True
        self.assertFalse(node._should_accept_phrase(200))  # 200-90=110 > 45

    def test_accept_wraparound_doa(self):
        """Test DOA wraparound: 359° and 1° are only 2° apart."""
        node = FakeAudioNode()
        node._last_wake_word_time = 0.0
        node._locked_doa_angle = 359
        node._in_active_dialogue = True
        self.assertTrue(node._should_accept_phrase(5))  # diff=6 < 45

    def test_reject_opposite_direction_doa(self):
        """180° away should always be rejected."""
        node = FakeAudioNode()
        node._last_wake_word_time = 0.0
        node._locked_doa_angle = 0
        node._in_active_dialogue = True
        self.assertFalse(node._should_accept_phrase(180))

    # --- Dialogue window (after bot response) --------------------

    def test_accept_in_dialogue_window(self):
        node = FakeAudioNode(dialogue_window_seconds=30.0)
        node._last_wake_word_time = 0.0
        node._locked_doa_angle = 90
        node._in_active_dialogue = False
        node._last_dialogue_response_time = time.time() - 5.0  # 5s ago < 30s
        self.assertTrue(node._should_accept_phrase(95))  # 5 < 45

    def test_reject_outside_dialogue_window(self):
        node = FakeAudioNode(dialogue_window_seconds=30.0)
        node._last_wake_word_time = 0.0
        node._locked_doa_angle = 90
        node._in_active_dialogue = False
        node._last_dialogue_response_time = time.time() - 60.0  # 60s > 30s
        self.assertFalse(node._should_accept_phrase(90))

    def test_reject_wrong_angle_in_dialogue_window(self):
        node = FakeAudioNode(dialogue_window_seconds=30.0)
        node._last_wake_word_time = 0.0
        node._locked_doa_angle = 90
        node._in_active_dialogue = False
        node._last_dialogue_response_time = time.time() - 5.0
        self.assertFalse(node._should_accept_phrase(250))  # 250-90=160 > 45

    # --- DOA lock disabled ---------------------------------------

    def test_doa_lock_disabled_accepts_any_angle_in_dialogue(self):
        node = FakeAudioNode(doa_lock_enabled=False)
        node._last_wake_word_time = 0.0
        node._locked_doa_angle = 90
        node._in_active_dialogue = True
        self.assertTrue(node._should_accept_phrase(270))  # Opposite side but no lock


if __name__ == "__main__":
    unittest.main()
