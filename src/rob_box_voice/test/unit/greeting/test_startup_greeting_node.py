#!/usr/bin/env python3
"""
test_startup_greeting_node.py — Unit/Integration тесты для StartupGreetingNode (issue #1003).

Тесты адаптированы под новое расположение (rob_box_voice) и новый критерий
готовности (подписчики на /voice/dialogue/response, а не /perception/context_update).
"""

from __future__ import annotations

import json
import time
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rob_box_voice.startup_greeting_node import StartupGreetingNode


def _make_msg_string(data: str) -> String:
    msg = String()
    msg.data = data
    return msg


class TestStartupGreetingNodeUnit(unittest.TestCase):
    """Юнит-тесты: создание ноды, параметры, publishers, GREETINGS."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self) -> None:
        self.node = StartupGreetingNode()

    def tearDown(self) -> None:
        # Защищаем от висящих таймеров — иначе teardown может упасть
        # на повторном создании ноды.
        for attr in ("check_timer", "shutdown_timer"):
            timer = getattr(self.node, attr, None)
            if timer is not None:
                try:
                    timer.cancel()
                except Exception:
                    pass
        self.node.destroy_node()

    # --------------------------------------------------------- creation / params
    def test_node_creation(self) -> None:
        self.assertEqual(self.node.get_name(), "startup_greeting_node")

    def test_parameters_exist(self) -> None:
        for param in (
            "wait_time",
            "check_timeout",
            "enable_greeting",
            "readiness_check_interval",
        ):
            self.assertTrue(self.node.has_parameter(param))

    def test_default_parameters(self) -> None:
        self.assertEqual(self.node.get_parameter("wait_time").value, 5.0)
        self.assertEqual(self.node.get_parameter("check_timeout").value, 30.0)
        self.assertTrue(self.node.get_parameter("enable_greeting").value)
        self.assertEqual(
            self.node.get_parameter("readiness_check_interval").value, 1.0
        )

    def test_publishers_created(self) -> None:
        pubs = self.node.get_publisher_names_and_types_by_node(
            self.node.get_name(), self.node.get_namespace()
        )
        topics = [name for name, _ in pubs]
        self.assertIn("/voice/sound/trigger", topics)
        self.assertIn("/voice/dialogue/response", topics)

    # ----------------------------------------------------------- initial state
    def test_initial_state(self) -> None:
        self.assertFalse(self.node.greeting_done)
        self.assertGreater(self.node._start_time, 0.0)

    def test_greetings_pool(self) -> None:
        """6–10 прикольных фраз, все — непустые строки."""
        greetings = StartupGreetingNode.GREETINGS
        self.assertIsInstance(greetings, (list, tuple))
        self.assertGreaterEqual(len(greetings), 6)
        self.assertLessEqual(len(greetings), 10)
        for g in greetings:
            self.assertIsInstance(g, str)
            self.assertGreater(len(g.strip()), 0)

    def test_finish_sounds_pool(self) -> None:
        """cute / very_cute — финальные звуки."""
        for s in StartupGreetingNode.FINISH_SOUNDS:
            self.assertIn(s, ("cute", "very_cute"))

    # ---------------------------------------------------------------- play_sound
    def test_play_sound_publishes(self) -> None:
        with patch.object(self.node.sound_pub, "publish") as mock_pub:
            self.node.play_sound("thinking")
        mock_pub.assert_called_once()
        msg = mock_pub.call_args[0][0]
        self.assertIsInstance(msg, String)
        self.assertEqual(msg.data, "thinking")

    def test_play_sound_three_different(self) -> None:
        with patch.object(self.node.sound_pub, "publish") as mock_pub:
            for s in ("thinking", "cute", "very_cute"):
                self.node.play_sound(s)
        self.assertEqual(mock_pub.call_count, 3)
        for i, s in enumerate(("thinking", "cute", "very_cute")):
            self.assertEqual(mock_pub.call_args_list[i][0][0].data, s)

    # ---------------------------------------------------- readiness check logic
    def test_check_readiness_skips_before_wait_time(self) -> None:
        self.node._start_time = time.monotonic()
        self.node.wait_time = 5.0
        with patch.object(self.node, "_tts_subscribers_present", return_value=True), \
             patch.object(self.node, "say_greeting") as mock_say:
            self.node.check_readiness()
        mock_say.assert_not_called()

    def test_check_readiness_says_when_ready(self) -> None:
        self.node._start_time = time.monotonic() - 6.0
        self.node.wait_time = 5.0
        with patch.object(self.node, "_tts_subscribers_present", return_value=True), \
             patch.object(self.node, "say_greeting") as mock_say:
            self.node.check_readiness()
        mock_say.assert_called_once()

    def test_check_readiness_timeout_says_anyway(self) -> None:
        self.node._start_time = time.monotonic() - 35.0
        self.node.check_timeout = 30.0
        with patch.object(self.node, "_tts_subscribers_present", return_value=False), \
             patch.object(self.node, "say_greeting") as mock_say:
            self.node.check_readiness()
        mock_say.assert_called_once()

    def test_check_readiness_skips_when_done(self) -> None:
        self.node.greeting_done = True
        with patch.object(self.node, "say_greeting") as mock_say:
            self.node.check_readiness()
        mock_say.assert_not_called()

    def test_check_readiness_not_ready_no_say(self) -> None:
        """tts_node ещё не подключился — ждём."""
        self.node._start_time = time.monotonic() - 6.0
        self.node.wait_time = 5.0
        with patch.object(self.node, "_tts_subscribers_present", return_value=False), \
             patch.object(self.node, "say_greeting") as mock_say:
            self.node.check_readiness()
        mock_say.assert_not_called()

    # ---------------------------------------------------------- say_greeting
    def test_say_greeting_disabled(self) -> None:
        self.node.enable_greeting = False
        with patch.object(self.node.sound_pub, "publish"), \
             patch.object(self.node.tts_pub, "publish"):
            self.node.say_greeting()
        self.assertFalse(self.node.greeting_done)

    def test_say_greeting_plays_cute_or_very_cute(self) -> None:
        with patch.object(self.node, "play_sound") as mock_play, \
             patch.object(self.node.tts_pub, "publish"), \
             patch("time.sleep"):
            self.node.say_greeting()
        self.assertEqual(mock_play.call_count, 1)
        sound_name = mock_play.call_args[0][0]
        self.assertIn(sound_name, ("cute", "very_cute"))

    def test_say_greeting_publishes_ssml(self) -> None:
        with patch.object(self.node, "play_sound"), \
             patch.object(self.node.tts_pub, "publish") as mock_pub, \
             patch("time.sleep"):
            self.node.say_greeting()
        mock_pub.assert_called_once()
        msg = mock_pub.call_args[0][0]
        self.assertIsInstance(msg, String)
        data = json.loads(msg.data)
        self.assertIn("ssml", data)
        self.assertTrue(data["ssml"].startswith("<speak>"))
        self.assertTrue(data["ssml"].endswith("</speak>"))
        # Текст внутри SSML — одна из наших фраз.
        inner = data["ssml"][len("<speak>"):-len("</speak>")]
        self.assertIn(inner, StartupGreetingNode.GREETINGS)

    def test_say_greeting_random_picks_varied_phrases(self) -> None:
        """За 20 попыток реально используется >1 фраза (рандомность)."""
        used = set()
        for _ in range(20):
            self.node.greeting_done = False
            self.node.enable_greeting = True
            with patch.object(self.node, "play_sound"), \
                 patch("time.sleep"), \
                 patch.object(self.node.tts_pub, "publish") as mock_pub:
                self.node.say_greeting()
            msg = mock_pub.call_args[0][0]
            data = json.loads(msg.data)
            inner = data["ssml"][len("<speak>"):-len("</speak>")]
            used.add(inner)
        # Если random.choice не сломан, попадётся как минимум 2 разных.
        self.assertGreater(len(used), 1)
        for phrase in used:
            self.assertIn(phrase, StartupGreetingNode.GREETINGS)

    def test_say_greeting_creates_shutdown_timer(self) -> None:
        with patch.object(self.node, "play_sound"), \
             patch.object(self.node.tts_pub, "publish"), \
             patch("time.sleep"):
            self.node.say_greeting()
        self.assertTrue(hasattr(self.node, "shutdown_timer"))
        self.assertIsNotNone(self.node.shutdown_timer)

    def test_say_greeting_sets_done_flag(self) -> None:
        with patch.object(self.node, "play_sound"), \
             patch.object(self.node.tts_pub, "publish"), \
             patch("time.sleep"):
            self.assertFalse(self.node.greeting_done)
            self.node.say_greeting()
            self.assertTrue(self.node.greeting_done)

    def test_say_greeting_only_once(self) -> None:
        with patch.object(self.node, "play_sound") as mock_play, \
             patch.object(self.node.tts_pub, "publish") as mock_pub, \
             patch("time.sleep"):
            self.node.say_greeting()
            n_pub_after_first = mock_pub.call_count
            n_play_after_first = mock_play.call_count
            self.node.say_greeting()  # повтор — не должно ничего сделать
            self.assertEqual(mock_pub.call_count, n_pub_after_first)
            self.assertEqual(mock_play.call_count, n_play_after_first)

    # --------------------------------------------------------- shutdown_node
    def test_shutdown_node_cancels_timer(self) -> None:
        self.node.shutdown_timer = MagicMock()
        with patch.object(self.node, "destroy_node"), \
             patch("rclpy.shutdown"):
            self.node.shutdown_node()
        self.node.shutdown_timer.cancel.assert_called_once()


class TestStartupGreetingNodeIntegration(unittest.TestCase):
    """Интеграционные тесты: реальные pub/sub через мок-DDS из conftest.py.

    Mock-DDS из conftest не доставляет сообщения между нодами автоматически —
    подписчик лишь хранит callback. Эти тесты проверяют контракт через
    прямое интроспектирование publishers: что было опубликовано, в каком
    порядке и с какими данными. Достаточно для acceptance #1003.
    """

    @classmethod
    def setUpClass(cls) -> None:
        if not rclpy.ok():
            rclpy.init()

    def setUp(self) -> None:
        self.node = StartupGreetingNode()

    def tearDown(self) -> None:
        for attr in ("check_timer", "shutdown_timer"):
            t = getattr(self.node, attr, None)
            if t is not None:
                try:
                    t.cancel()
                except Exception:
                    pass
        self.node.destroy_node()

    # --------------------------------------------------------- helpers
    def _published_sound_names(self) -> list[str]:
        """Что звуковая нода получила через /voice/sound/trigger."""
        # Каждый .publish() вызывался с одним String-аргументом.
        calls = self.node.sound_pub.publish.call_args_list
        out = []
        for c in calls:
            args = c.args or (c.kwargs or {}).values()
            for arg in args:
                msg = arg
                if hasattr(msg, "data"):
                    out.append(msg.data)
        return out

    def _published_tts_payloads(self) -> list[str]:
        calls = self.node.tts_pub.publish.call_args_list
        out = []
        for c in calls:
            args = c.args or (c.kwargs or {}).values()
            for arg in args:
                msg = arg
                if hasattr(msg, "data"):
                    out.append(msg.data)
        return out

    # --------------------------------------------------------- tests
    def test_thinking_sound_published_at_init(self) -> None:
        """Сразу при старте проигрывается thinking (acceptance #1003)."""
        # Конструктор уже отработал к этому моменту — publish был вызван.
        names = self._published_sound_names()
        self.assertEqual(names, ["thinking"])

    def test_say_greeting_publishes_sound_and_tts(self) -> None:
        """Прямой вызов say_greeting: звук + TTS-фраза."""
        self.node.enable_greeting = True
        self.node.greeting_done = False
        with patch("time.sleep"):
            self.node.say_greeting()
        names = self._published_sound_names()
        payloads = self._published_tts_payloads()
        # Должны быть оба: thinking (из __init__) + cute/very_cute (из say_greeting).
        self.assertEqual(names[0], "thinking")
        self.assertIn(names[-1], ("cute", "very_cute"))
        # TTS — ровно один chunk.
        self.assertEqual(len(payloads), 1)
        data = json.loads(payloads[0])
        self.assertIn("ssml", data)
        inner = data["ssml"][len("<speak>"):-len("</speak>")]
        self.assertIn(inner, StartupGreetingNode.GREETINGS)


if __name__ == "__main__":
    unittest.main()
