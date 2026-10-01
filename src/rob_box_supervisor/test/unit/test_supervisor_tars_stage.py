"""issue #3253 (Ш2) — supervisor публикует стадии ТАРС в /avatar/tars/stage.

* ``thinking`` — перед ходом LLM (``_run_agent_sync``).
* ``idle`` — ход закончился БЕЗ реплики в шлем (агент выключен, текстовый
  вход, битый вход). Если реплика ушла в /avatar/tts/request — idle ведёт
  quest_node, supervisor его НЕ шлёт.
* Реплика в шлем несёт ``speech_id = "tars-<request_id>"``: по нему
  quest_node сопоставляет /voice/tts/finished с репликой.

mock-rclpy из conftest.py; LLM подменён (``_run_agent_sync``).
"""

from __future__ import annotations

import json
import unittest
from unittest.mock import MagicMock

from rob_box_supervisor.supervisor_node import (
    AVATAR_TARS_STAGE_TOPIC,
    AvatarSupervisor,
)


def _msg(payload) -> MagicMock:
    m = MagicMock()
    m.data = payload if isinstance(payload, str) else json.dumps(payload, ensure_ascii=False)
    return m


def _stages(node: AvatarSupervisor) -> list[dict]:
    return [json.loads(m.data) for m in node._tars_stage_pub.published]


def _avatar_tts(node: AvatarSupervisor) -> list[dict]:
    return [json.loads(m.data) for m in node._avatar_tts_request_pub.published]


def _command(source: str) -> MagicMock:
    return _msg({"source": source, "client_id": "c1", "text": "который час", "ts_ms": 7})


class TestTarsStagePublisher(unittest.TestCase):
    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._agent_enabled = True
        self.node._agent_core = object()
        self.node._operator_dsm = None
        self.node._operator_journal = None
        self.node._run_agent_sync = lambda core, payload: {
            "ok": True,
            "summary": "Сейчас семь часов.",
            "tool_calls": [],
        }

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_publisher_declared(self) -> None:
        self.assertEqual(AVATAR_TARS_STAGE_TOPIC, "/avatar/tars/stage")
        self.assertIn(AVATAR_TARS_STAGE_TOPIC, self.node._publishers)

    def test_quest_turn_publishes_thinking_and_leaves_idle_to_quest_node(self) -> None:
        self.node._on_avatar_command(_command("quest"))
        stages = _stages(self.node)
        self.assertEqual([s["stage"] for s in stages], ["thinking"])
        self.assertEqual(stages[0]["request_id"], "c1:7")
        self.assertIsInstance(stages[0]["ts_ms"], int)
        # Реплика ушла в шлем со speech_id для корреляции с finished.
        tts = _avatar_tts(self.node)
        self.assertEqual(len(tts), 1)
        self.assertEqual(tts[0]["speech_id"], f"tars-{tts[0]['request_id']}")

    def test_thinking_is_published_before_llm_runs(self) -> None:
        seen: list[list[str]] = []

        def fake_run(core, payload):
            seen.append([s["stage"] for s in _stages(self.node)])
            return {"ok": True, "summary": "ок", "tool_calls": []}

        self.node._run_agent_sync = fake_run
        self.node._on_avatar_command(_command("quest"))
        self.assertEqual(seen, [["thinking"]])

    def test_text_turn_without_speech_ends_with_idle(self) -> None:
        self.node._on_avatar_command(_command("telegram"))
        stages = _stages(self.node)
        self.assertEqual([s["stage"] for s in stages], ["thinking", "idle"])
        self.assertEqual(stages[1]["reason"], "no_speech")
        self.assertEqual(_avatar_tts(self.node), [])

    def test_empty_reply_ends_with_idle(self) -> None:
        self.node._run_agent_sync = lambda core, payload: {
            "ok": False, "summary": "", "tool_calls": [],
        }
        self.node._on_avatar_command(_command("quest"))
        self.assertEqual([s["stage"] for s in _stages(self.node)], ["thinking", "idle"])

    def test_llm_exception_still_ends_turn(self) -> None:
        """LLM упал → summary outer_error озвучивается; без речи был бы idle."""

        def boom(core, payload):
            raise RuntimeError("llm down")

        self.node._run_agent_sync = boom
        self.node._on_avatar_command(_command("telegram"))
        self.assertEqual([s["stage"] for s in _stages(self.node)], ["thinking", "idle"])

    def test_disabled_agent_publishes_idle_without_thinking(self) -> None:
        self.node._agent_enabled = False
        self.node._on_avatar_command(_command("quest"))
        stages = _stages(self.node)
        self.assertEqual([s["stage"] for s in stages], ["idle"])
        self.assertEqual(stages[0]["reason"], "agent_disabled")

    def test_malformed_input_publishes_idle(self) -> None:
        self.node._on_avatar_command(_msg("not-json"))
        stages = _stages(self.node)
        self.assertEqual([s["stage"] for s in stages], ["idle"])
        self.assertEqual(stages[0]["reason"], "malformed_input")

    def test_speak_agent_replies_off_ends_with_idle(self) -> None:
        self.node._param_bool = lambda name, default: False
        self.node._on_avatar_command(_command("quest"))
        self.assertEqual([s["stage"] for s in _stages(self.node)], ["thinking", "idle"])

    def test_publish_failure_does_not_break_turn(self) -> None:
        self.node._tars_stage_pub.publish = MagicMock(side_effect=RuntimeError("ros down"))
        self.node._on_avatar_command(_command("quest"))
        self.assertEqual(len(_avatar_tts(self.node)), 1)


if __name__ == "__main__":
    unittest.main()
