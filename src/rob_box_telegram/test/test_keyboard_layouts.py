#!/usr/bin/env python3
"""Tests for rob_box_telegram.keyboard_layouts module."""

import unittest

from rob_box_telegram.keyboard_layouts import (
    MAIN_MENU_KEYBOARD,
    MOVE_VELOCITIES,
    MOVEMENT_KEYBOARD,
)


class TestMovementVelocities(unittest.TestCase):
    """Tests for movement velocity presets."""

    def test_all_directions_defined(self):
        expected = {
            "forward", "backward", "left", "right",
            "forward_left", "forward_right",
            "backward_left", "backward_right",
            "stop",
        }
        self.assertEqual(set(MOVE_VELOCITIES.keys()), expected)

    def test_stop_is_zero(self):
        self.assertEqual(MOVE_VELOCITIES["stop"], (0.0, 0.0))

    def test_forward_positive_linear(self):
        self.assertGreater(MOVE_VELOCITIES["forward"][0], 0)
        self.assertEqual(MOVE_VELOCITIES["forward"][1], 0)

    def test_backward_negative_linear(self):
        self.assertLess(MOVE_VELOCITIES["backward"][0], 0)

    def test_left_positive_angular(self):
        self.assertGreater(MOVE_VELOCITIES["left"][1], 0)

    def test_right_negative_angular(self):
        self.assertLess(MOVE_VELOCITIES["right"][1], 0)

    def test_safe_speeds(self):
        """All velocities should be within safety limits."""
        for direction, (lin, ang) in MOVE_VELOCITIES.items():
            self.assertLessEqual(abs(lin), 0.3, f"{direction} linear too fast: {lin}")
            self.assertLessEqual(abs(ang), 0.5, f"{direction} angular too fast: {ang}")


class TestKeyboardStructure(unittest.TestCase):
    """Tests for keyboard markup structure."""

    def test_movement_keyboard_has_buttons(self):
        rows = MOVEMENT_KEYBOARD.inline_keyboard
        self.assertGreater(len(rows), 0)
        # Check all callback_data starts with expected prefix
        for row in rows:
            for btn in row:
                self.assertTrue(
                    btn.callback_data.startswith("move:") or btn.callback_data.startswith("quick:"),
                    f"Unexpected callback_data: {btn.callback_data}",
                )

    def test_main_menu_has_buttons(self):
        rows = MAIN_MENU_KEYBOARD.inline_keyboard
        self.assertGreater(len(rows), 0)


if __name__ == "__main__":
    unittest.main()
