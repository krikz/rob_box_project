"""Unit-тесты супервизор-агента ТАРС (issue #1988, шаг 4а; AV-21/issue #1913).

Покрывает acceptance-критерии из docs/plans/2026-09-02-avatar-supervisor-agent-design.md §6
(движок с issue #1988 — AgentCore вместо OperatorHarness):

1. **AC #3** agent_enabled=false → /avatar/command_result{ok=false,
   summary="agent_disabled"}, LLM НЕ вызван (``agent_core is None``
   после отправки команды). Default параметра — **true** (решение Шифу).
2. **AC #6** «скажи привет» + agent_enabled=true → ok=true,
   summary="ok", tool_calls=[{name="say", ...}].
3. **AC #7** команда без инструмента → ok=false, summary="no_tool:*",
   tool_calls=[] (НЕ выдумываем действие).
4. **AC #8** LLM исключение → dialogue_control_swap НЕ блокирует
   последующие команды (try/finally восстанавливает личность через
   ``resume`` в ``/dialogue/control``, ADR-0054 §6.7).
5. **AC #9** метрики инкрементнулись после успешной команды.
6. **AC #3 (negative)** невалидный JSON → ok=false,
   summary="malformed_input".
7. **AC #8 + infra** swap.apply+exit последовательность:
   ``_publish_dialogue_control`` вызывается дважды (pause на входе,
   resume в finally).
8. **AC #infra** agent_enabled=false → НЕ вызывается ``_ensure_agent_core``
   (нет лишних импортов/LLM-init-ов в monitor-режиме).

Используем mock-rclpy из conftest.py. Реального AgentCore/LLM в тестах НЕТ:
``_agent_core``/``_run_agent_sync`` подменяются (см. ``_stub_agent_core``),
чтобы не дёргать LLM и не требовать async-loop-а / ключей API.
"""

from __future__ import annotations

import json
import unittest
from unittest.mock import MagicMock

from rob_box_supervisor.supervisor_node import (
    AGENT_COMMAND_SOURCES,
    AVATAR_COMMAND_RESULT_TOPIC,
    AVATAR_COMMAND_TOPIC,
    DIALOGUE_CONTROL_PAUSE,
    DIALOGUE_CONTROL_RESUME,
    MONITOR_MODE_REASON,
    AvatarSupervisor,
)


def _make_string_msg(data: str) -> MagicMock:
    """Создать фейковый std_msgs/String с .data."""
    m = MagicMock()
    m.data = data
    return m


def _published_results(node: AvatarSupervisor) -> list[dict]:
    """Достать все опубликованные /avatar/command_result payloads (parsed JSON)."""
    pub = node._agent_result_pub
    return [json.loads(m.data) for m in pub.published]


def _stub_agent_core(node: AvatarSupervisor) -> None:
    """Заглушить сборку реального AgentCore (нет ключей/ROS в unit-тестах).

    ``_ensure_agent_core`` возвращает готовый ``_agent_core``, реальная
    сборка (LLM/Tools/Memory) не запускается. ``_run_agent_sync`` тесты
    подменяют отдельно (scripted-mapping результата).
    """
    node._agent_core = object()
    node._operator_dsm = None
    node._operator_journal = None


# ─────────────────────────────────────────────────────────────────────
# AC #3: agent_enabled gate
# ─────────────────────────────────────────────────────────────────────


class TestAgentEnabledGate(unittest.TestCase):
    """При agent_enabled=false нода ведёт себя как до 4а — НЕ
    инстанцирует AgentCore, НЕ вызывает LLM, публикует agent_disabled.
    Default параметра — true (issue #1988, решение Шифу)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        # Default agent_enabled=true (шаг 4а).
        self.assertTrue(self.node._agent_enabled)
        self.assertIsNone(self.node._agent_core)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_agent_enabled_default_true(self) -> None:
        """Параметр agent_enabled default true."""
        self.assertTrue(self.node.get_parameter("agent_enabled").value)
        self.assertTrue(self.node._agent_enabled)

    def test_command_with_disabled_agent_publishes_agent_disabled(self) -> None:
        """AC #3: enabled=false → /avatar/command_result{ok=false, summary='agent_disabled'}."""
        self.node._agent_enabled = False
        msg = _make_string_msg(
            json.dumps(
                {
                    "source": "quest",
                    "client_id": "test-client",
                    "text": "скажи привет",
                    "ts_ms": 1234567890,
                }
            )
        )
        self.node._on_avatar_command(msg)

        results = _published_results(self.node)
        self.assertEqual(len(results), 1)
        self.assertFalse(results[0]["ok"])
        self.assertEqual(results[0]["summary"], "agent_disabled")
        self.assertEqual(results[0]["tool_calls"], [])
        self.assertEqual(results[0]["request_id"], "test-client:1234567890")

    def test_disabled_agent_does_not_instantiate_core(self) -> None:
        """AC #3 (negative): enabled=false → _ensure_agent_core НЕ вызван,
        ``_agent_core`` остаётся None."""
        self.node._agent_enabled = False
        msg = _make_string_msg(
            json.dumps(
                {
                    "source": "telegram",
                    "client_id": "alice",
                    "text": "play animation wave",
                    "ts_ms": 999,
                }
            )
        )
        self.node._on_avatar_command(msg)

        # Core не создан.
        self.assertIsNone(self.node._agent_core)
        # Результат — agent_disabled, не попытка вызвать LLM.
        results = _published_results(self.node)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["summary"], "agent_disabled")


# ─────────────────────────────────────────────────────────────────────
# AC #4: топики объявлены
# ─────────────────────────────────────────────────────────────────────


class TestAvatarAgentTopology(unittest.TestCase):
    """Топики /avatar/command (sub) и /avatar/command_result (pub)
    объявлены на этапе __init__ ноды (AC #2)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_avatar_command_subscription(self) -> None:
        topics = [s.topic for s in self.node._subscriptions]
        self.assertIn(AVATAR_COMMAND_TOPIC, topics)
        self.assertEqual(AVATAR_COMMAND_TOPIC, "/avatar/command")

    def test_avatar_command_result_publisher(self) -> None:
        self.assertIn(AVATAR_COMMAND_RESULT_TOPIC, self.node._publishers)
        self.assertEqual(AVATAR_COMMAND_RESULT_TOPIC, "/avatar/command_result")

    def test_avatar_stt_result_subscription_dormant(self) -> None:
        """Шаг 4а: подписка на /avatar/stt/result (вейк-вход, шаг 05/#1990)
        объявлена. Топик появится после #1990; сейчас подписка дремлет."""
        topics = [s.topic for s in self.node._subscriptions]
        self.assertIn("/avatar/stt/result", topics)

    def test_agent_parameters_declared(self) -> None:
        """Параметры ``agent_enabled`` / ``system_prompt_file`` объявлены
        через ``declare_parameter``.

        Параметр ``agent_during_voice_mode`` удалён вместе с
        ``voice_input_mode`` (ADR-0054 §6.7): пауза личности теперь
        жёстко зашита в ``_dialogue_control_swap`` (pause на входе,
        resume в finally), переопределение через параметр больше не нужно.
        """
        self.assertTrue(self.node.has_parameter("agent_enabled"))
        self.assertTrue(self.node.has_parameter("system_prompt_file"))
        self.assertFalse(
            self.node.has_parameter("agent_during_voice_mode"),
            "agent_during_voice_mode удалён (ADR-0054 §6.7)",
        )
        # default
        self.assertEqual(
            self.node.get_parameter("system_prompt_file").value,
            "operator_system_prompt.txt",
        )


# ─────────────────────────────────────────────────────────────────────
# AC #6: «скажи привет» → say tool-call → ok=true
# ─────────────────────────────────────────────────────────────────────


class TestSayCommandExecutes(unittest.TestCase):
    """Команда «скажи привет» → tool-call say → ok=true.

    Реальный AgentCore не строится (_stub_agent_core); ``_run_agent_sync``
    подменяется scripted-mapping — эквивалент mock-LLM без async-loop-а.
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._agent_enabled = True
        _stub_agent_core(self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_say_command_returns_ok(self) -> None:
        """AC #6: «скажи привет» → один tool-call say → ok=true."""

        def fake_run(core, payload):
            return {
                "ok": True,
                "summary": "ok",
                "tool_calls": [
                    {"name": "say", "arguments": {"text": "Привет"}, "result": "ok"},
                ],
            }

        msg = _make_string_msg(
            json.dumps(
                {
                    "source": "quest",
                    "client_id": "quest-1",
                    "text": "скажи привет",
                    "ts_ms": 100,
                }
            )
        )
        original = self.node._run_agent_sync
        self.node._run_agent_sync = fake_run
        try:
            self.node._on_avatar_command(msg)
        finally:
            self.node._run_agent_sync = original

        results = _published_results(self.node)
        self.assertEqual(len(results), 1)
        self.assertTrue(results[0]["ok"])
        self.assertEqual(results[0]["summary"], "ok")
        self.assertEqual(len(results[0]["tool_calls"]), 1)
        self.assertEqual(results[0]["tool_calls"][0]["name"], "say")
        self.assertEqual(results[0]["tool_calls"][0]["arguments"], {"text": "Привет"})


# ─────────────────────────────────────────────────────────────────────
# AC #7: команда без инструмента → ok=false, summary="no_tool:*"
# ─────────────────────────────────────────────────────────────────────


class TestNoToolReturnsNoTool(unittest.TestCase):
    """Если LLM не запросил ни одного tool_call — ok=false,
    summary содержит 'no_tool'. НЕ выдумываем действие (ADR-0018)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._agent_enabled = True
        _stub_agent_core(self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_no_tool_call_returns_no_tool_summary(self) -> None:
        """AC #7: LLM вернул 0 tool_calls → ok=false, summary='no_tool:*'."""

        def fake_run(core, payload):
            return {
                "ok": False,
                "summary": "no_tool: unknown_action",
                "tool_calls": [],
                "error": "LLM did not call any tool",
            }

        msg = _make_string_msg(
            json.dumps(
                {
                    "source": "telegram",
                    "client_id": "alice",
                    "text": "сделай что-нибудь",
                    "ts_ms": 200,
                }
            )
        )
        original = self.node._run_agent_sync
        self.node._run_agent_sync = fake_run
        try:
            self.node._on_avatar_command(msg)
        finally:
            self.node._run_agent_sync = original

        results = _published_results(self.node)
        self.assertEqual(len(results), 1)
        self.assertFalse(results[0]["ok"])
        self.assertTrue(results[0]["summary"].startswith("no_tool"))
        self.assertEqual(results[0]["tool_calls"], [])


# ─────────────────────────────────────────────────────────────────────
# AC #8: LLM бросил исключение → swap НЕ блокирует, голосовой режим не «залипает»
# ─────────────────────────────────────────────────────────────────────


class TestDialogueControlSwapTryFinally(unittest.TestCase):
    """AC #8 (ADR-0054 §6.7): try/finally ``_dialogue_control_swap``
    корректно отрабатывает даже если core/agent бросает исключение.

    Swap публикует ``pause`` на входе и ``resume`` в finally — личность
    гарантированно возвращается в IDLE после обработки команды.
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._agent_enabled = True
        _stub_agent_core(self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _published_actions(self) -> list[str]:
        """Достать последовательность ``action``-значений из опубликованного
        в ``/dialogue/control``. Удобно для проверки порядка pause→resume.
        """
        pub = self.node._dialogue_control_pub
        return [json.loads(m.data)["action"] for m in pub.published]

    def test_swap_publishes_pause_then_resume(self) -> None:
        """Свап.enter публикует ``pause``, swap.exit — ``resume``."""
        with self.node._dialogue_control_swap():
            pass
        actions = self._published_actions()
        self.assertEqual(
            actions, [DIALOGUE_CONTROL_PAUSE, DIALOGUE_CONTROL_RESUME]
        )

    def test_swap_resume_always_runs_on_normal_exit(self) -> None:
        """После нормального yield в swap последовательность строго
        pause→resume, даже если внутри ничего не делали."""
        with self.node._dialogue_control_swap():
            _ = 1 + 1  # yield без побочек
        actions = self._published_actions()
        self.assertEqual(len(actions), 2)
        self.assertEqual(actions[0], DIALOGUE_CONTROL_PAUSE)
        self.assertEqual(actions[1], DIALOGUE_CONTROL_RESUME)

    def test_swap_resume_always_runs_on_exception(self) -> None:
        """AC #8: исключение внутри swap → finally всё равно шлёт resume."""
        with self.assertRaises(RuntimeError):
            with self.node._dialogue_control_swap():
                raise RuntimeError("simulated LLM crash")
        actions = self._published_actions()
        # Два publish'а даже при исключении: pause на входе, resume в finally.
        self.assertEqual(len(actions), 2)
        self.assertEqual(actions[0], DIALOGUE_CONTROL_PAUSE)
        self.assertEqual(actions[1], DIALOGUE_CONTROL_RESUME)

    def test_swap_payload_format(self) -> None:
        """Payload соответствует wire-контракту (ADR-0054 §2.1):
        ``{action, reason, ts_s}``, причём ``ts_s`` — float."""
        with self.node._dialogue_control_swap():
            pass
        pub = self.node._dialogue_control_pub
        for msg in pub.published:
            payload = json.loads(msg.data)
            self.assertIn("action", payload)
            self.assertIn("reason", payload)
            self.assertIn("ts_s", payload)
            self.assertIsInstance(payload["ts_s"], float)
            # reason содержит диагностическую подсказку.
            self.assertTrue(payload["reason"])
        # reason для pause и resume разный (чтобы логи dialogue_node
        # отличали «оператор начал работать» от «оператор закончил»).
        pause_payload = json.loads(pub.published[0].data)
        resume_payload = json.loads(pub.published[1].data)
        self.assertIn("agent_during_command", pause_payload["reason"])
        self.assertIn("agent_command_done", resume_payload["reason"])

    def test_llm_exception_swap_finally_publishes_resume(self) -> None:
        """AC #8 end-to-end: исключение из ``_run_agent_sync`` →
        ``/dialogue/control`` всё равно получает resume, и результат
        публикуется с ``ok=False, summary="outer_error: ..."``.
        """
        original_publish = self.node._publish_dialogue_control
        published_actions: list[str] = []

        def spy_publish(action: str, reason: str = "") -> bool:
            published_actions.append(action)
            return True

        self.node._publish_dialogue_control = spy_publish

        def exploding_run(core, payload):
            raise RuntimeError("simulated LLM crash")

        original_run = self.node._run_agent_sync
        self.node._run_agent_sync = exploding_run
        try:
            msg = _make_string_msg(
                json.dumps(
                    {
                        "source": "quest",
                        "client_id": "q1",
                        "text": "boom",
                        "ts_ms": 300,
                    }
                )
            )
            self.node._on_avatar_command(msg)
        finally:
            self.node._run_agent_sync = original_run
            self.node._publish_dialogue_control = original_publish

        # Pause+resume произошли (порядок важен — pause до resume).
        self.assertIn(DIALOGUE_CONTROL_PAUSE, published_actions)
        self.assertIn(DIALOGUE_CONTROL_RESUME, published_actions)
        pause_idx = published_actions.index(DIALOGUE_CONTROL_PAUSE)
        resume_idx = published_actions.index(DIALOGUE_CONTROL_RESUME)
        self.assertLess(pause_idx, resume_idx)
        # Публикация — ok=False, summary указывает на outer_error
        # (try/except в _on_avatar_command ловит исключение из swap).
        results = _published_results(self.node)
        self.assertEqual(len(results), 1)
        self.assertFalse(results[0]["ok"])
        self.assertTrue(
            "outer_error" in results[0]["summary"],
            f"expected outer_error summary, got {results[0]['summary']!r}",
        )


# ─────────────────────────────────────────────────────────────────────
# AC #9: метрики инкрементнулись
# ─────────────────────────────────────────────────────────────────────


class TestAgentMetrics(unittest.TestCase):
    """AC #9: avatar_agent_commands_total / avatar_agent_tool_calls_total
    инкрементнулись после одной успешной команды."""

    def setUp(self) -> None:
        # Форсируем ``MetricsDisabled`` (singleton) — реальный
        # prometheus_client.Counter не позволяет setattr на ``labels``
        # (property без setter), поэтому spy на инстансе не сработает.
        # Тест-инвариант: метрики best-effort и в CI-env (без prom-client)
        # тесты должны работать над той же заглушкой, что и прод-код.
        import unittest.mock as _mock

        self._prom_patch = _mock.patch(
            "rob_box_voice.observability.metrics.is_metrics_enabled",
            return_value=False,
        )
        self._prom_patch.start()
        self.node = AvatarSupervisor()
        self.node._agent_enabled = True
        _stub_agent_core(self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()
        self._prom_patch.stop()

    def test_metrics_incremented_on_say(self) -> None:
        """После «скажи привет» инкрементнулась и command, и tool_call."""
        self._command_inc_count = 0
        self._tool_inc_count = 0
        commands_counter = self.node._agent_metrics["commands"]
        tools_counter = self.node._agent_metrics["tool_calls"]
        original_commands_inc = commands_counter.labels("quest", "ok").inc
        original_tool_inc = tools_counter.labels("say").inc

        def inc_commands(*a, **kw):
            self._command_inc_count += 1
            return original_commands_inc(*a, **kw)

        def inc_tools(*a, **kw):
            self._tool_inc_count += 1
            return original_tool_inc(*a, **kw)

        # Spy on labels() — wraps each .inc() через closure
        original_commands_labels = commands_counter.labels
        original_tool_labels = tools_counter.labels

        def spy_commands_labels(*args, **kwargs):
            inner = original_commands_labels(*args, **kwargs)
            inner.inc = inc_commands
            return inner

        def spy_tool_labels(*args, **kwargs):
            inner = original_tool_labels(*args, **kwargs)
            inner.inc = inc_tools
            return inner

        commands_counter.labels = spy_commands_labels
        tools_counter.labels = spy_tool_labels

        def fake_run(core, payload):
            return {
                "ok": True,
                "summary": "ok",
                "tool_calls": [
                    {"name": "say", "arguments": {"text": "Привет"}, "result": "ok"},
                ],
            }

        original = self.node._run_agent_sync
        self.node._run_agent_sync = fake_run
        try:
            msg = _make_string_msg(
                json.dumps(
                    {
                        "source": "telegram",
                        "client_id": "alice",
                        "text": "скажи привет",
                        "ts_ms": 401,
                    }
                )
            )
            self.node._on_avatar_command(msg)
        finally:
            self.node._run_agent_sync = original
            commands_counter.labels = original_commands_labels
            tools_counter.labels = original_tool_labels

        self.assertEqual(self._command_inc_count, 1)
        self.assertEqual(self._tool_inc_count, 1)

    def test_metrics_incremented_on_no_tool(self) -> None:
        """AC #9 (negative): после no_tool — commands inc с result=no_tool, tool_calls нет."""
        self._labels_seen: list[dict] = []
        commands_counter = self.node._agent_metrics["commands"]
        original_labels = commands_counter.labels

        def spy_labels(*args, **kwargs):
            # Перехватываем kwargs (source/result).
            self._labels_seen.append(dict(kwargs))
            return original_labels(*args, **kwargs)

        commands_counter.labels = spy_labels

        def fake_run(core, payload):
            return {
                "ok": False,
                "summary": "no_tool: unknown",
                "tool_calls": [],
            }

        original = self.node._run_agent_sync
        self.node._run_agent_sync = fake_run
        try:
            msg = _make_string_msg(
                json.dumps(
                    {
                        "source": "quest",
                        "client_id": "q2",
                        "text": "что-то непонятное",
                        "ts_ms": 500,
                    }
                )
            )
            self.node._on_avatar_command(msg)
        finally:
            self.node._run_agent_sync = original
            commands_counter.labels = original_labels

        no_tool_labels = [
            entry for entry in self._labels_seen if entry.get("source") == "quest" and entry.get("result") == "no_tool"
        ]
        self.assertEqual(len(no_tool_labels), 1, f"labels seen: {self._labels_seen}")


# ─────────────────────────────────────────────────────────────────────
# AC #infra: malformed JSON → malformed_input
# ─────────────────────────────────────────────────────────────────────


class TestMalformedInput(unittest.TestCase):
    """Невалидный JSON или отсутствующие обязательные поля →
    ok=False, summary='malformed_input'."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_non_json_publishes_malformed_input(self) -> None:
        msg = _make_string_msg("not a json")
        self.node._on_avatar_command(msg)

        results = _published_results(self.node)
        self.assertEqual(len(results), 1)
        self.assertFalse(results[0]["ok"])
        self.assertEqual(results[0]["summary"], "malformed_input")
        self.assertEqual(results[0]["tool_calls"], [])

    def test_empty_payload_publishes_malformed_input(self) -> None:
        msg = _make_string_msg("")
        self.node._on_avatar_command(msg)

        results = _published_results(self.node)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["summary"], "malformed_input")

    def test_missing_text_field_publishes_malformed_input(self) -> None:
        msg = _make_string_msg(json.dumps({"source": "quest", "client_id": "x"}))
        self.node._on_avatar_command(msg)

        results = _published_results(self.node)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["summary"], "malformed_input")

    def test_missing_source_field_publishes_malformed_input(self) -> None:
        msg = _make_string_msg(json.dumps({"client_id": "x", "text": "hello"}))
        self.node._on_avatar_command(msg)

        results = _published_results(self.node)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["summary"], "malformed_input")


# ─────────────────────────────────────────────────────────────────────
# Pure logic helper tests
# ─────────────────────────────────────────────────────────────────────


class TestParseCommandPayload(unittest.TestCase):
    """Pure-логика _parse_command_payload."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_valid_payload(self) -> None:
        parsed = self.node._parse_command_payload(
            json.dumps(
                {
                    "source": "quest",
                    "client_id": "q1",
                    "text": "  hello  ",
                    "ts_ms": 123,
                }
            )
        )
        self.assertTrue(parsed["ok"])
        self.assertEqual(parsed["payload"]["source"], "quest")
        self.assertEqual(parsed["payload"]["client_id"], "q1")
        self.assertEqual(parsed["payload"]["text"], "hello")  # stripped
        self.assertEqual(parsed["payload"]["ts_ms"], 123)

    def test_missing_text_returns_error(self) -> None:
        parsed = self.node._parse_command_payload(json.dumps({"source": "q"}))
        self.assertFalse(parsed["ok"])
        self.assertIn("text", parsed["error"])

    def test_empty_text_returns_error(self) -> None:
        parsed = self.node._parse_command_payload(json.dumps({"source": "q", "text": "   "}))
        self.assertFalse(parsed["ok"])

    def test_missing_source_returns_error(self) -> None:
        parsed = self.node._parse_command_payload(json.dumps({"text": "x"}))
        self.assertFalse(parsed["ok"])
        self.assertIn("source", parsed["error"])

    def test_non_dict_returns_error(self) -> None:
        parsed = self.node._parse_command_payload(json.dumps(["array"]))
        self.assertFalse(parsed["ok"])

    def test_empty_string_returns_error(self) -> None:
        parsed = self.node._parse_command_payload("")
        self.assertFalse(parsed["ok"])


class TestGenerateRequestId(unittest.TestCase):
    """request_id детерминирован из client_id+ts_ms если есть, иначе UUID."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_deterministic_with_client_id_and_ts(self) -> None:
        rid = self.node._generate_request_id({"client_id": "alice", "ts_ms": 12345})
        self.assertEqual(rid, "alice:12345")

    def test_unique_when_no_client_id(self) -> None:
        rid1 = self.node._generate_request_id({"ts_ms": 100})
        rid2 = self.node._generate_request_id({"ts_ms": 100})
        self.assertNotEqual(rid1, rid2)

    def test_unique_when_no_ts(self) -> None:
        rid1 = self.node._generate_request_id({"client_id": "alice"})
        rid2 = self.node._generate_request_id({"client_id": "alice"})
        self.assertNotEqual(rid1, rid2)


class TestDialogueControlSwapPayload(unittest.TestCase):
    """Дополнительный sanity-чек: payload ``/dialogue/control`` содержит
    корректные ``ts_s`` (float > 0) и reason с человекочитаемой подсказкой.

    ``_capture_current_voice_mode`` удалён вместе с ``voice_input_mode``
    (ADR-0054 §6.7): новая модель — pause↔resume через
    ``_publish_dialogue_control``, snapshot предыдущего состояния не
    нужен (dialogue_node сам хранит FSM-стейт).
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_publish_pause_payload(self) -> None:
        """Прямой вызов ``_publish_dialogue_control('pause', ...)`` → JSON
        с ``action=='pause'`` и ``ts_s`` (float)."""
        ok = self.node._publish_dialogue_control(
            DIALOGUE_CONTROL_PAUSE, reason="test"
        )
        self.assertTrue(ok)
        pub = self.node._dialogue_control_pub
        self.assertEqual(len(pub.published), 1)
        payload = json.loads(pub.published[0].data)
        self.assertEqual(payload["action"], DIALOGUE_CONTROL_PAUSE)
        self.assertEqual(payload["reason"], "test")
        self.assertIsInstance(payload["ts_s"], float)
        self.assertGreater(payload["ts_s"], 0)

    def test_publish_invalid_action_returns_false(self) -> None:
        """Защита от опечаток: ``action='bogus'`` → False, ничего не публикуется."""
        pub = self.node._dialogue_control_pub
        published_before = len(pub.published)
        ok = self.node._publish_dialogue_control("bogus", reason="x")
        self.assertFalse(ok)
        self.assertEqual(len(pub.published), published_before)


class TestAgentCommandSources(unittest.TestCase):
    """AGENT_COMMAND_SOURCES содержит quest/telegram; расширяемо через логирование."""

    def test_known_sources_listed(self) -> None:
        self.assertIn("quest", AGENT_COMMAND_SOURCES)
        self.assertIn("telegram", AGENT_COMMAND_SOURCES)


if __name__ == "__main__":
    unittest.main()


# ─────────────────────────────────────────────────────────────────────
# issue #2116: ответ агента озвучивается в шлем, а не только в текст
# ─────────────────────────────────────────────────────────────────────


def _published_avatar_tts(node: AvatarSupervisor) -> list[dict]:
    """Достать всё опубликованное в /avatar/tts/request (parsed JSON)."""
    return [json.loads(m.data) for m in node._avatar_tts_request_pub.published]


class TestAgentReplyIsSpoken(unittest.TestCase):
    """Регресс #2116: ТАРС думал и молчал.

    ``_publish_avatar_tts`` был написан по ADR-0055 («все supervisor-ответы
    строго в /avatar/tts/request»), покрыт целым файлом тестов — и не имел
    НИ ОДНОГО вызова в проде. Замер на роботе 2026-09-08: агент вернул
    ``content='ТАРС, агент оператора.'``, а ``/avatar/tts/request`` остался
    пуст, ``/avatar/tts/audio`` отдал 0 байт. Ответ упирался в текстовый
    ``/avatar/command_result`` и умирал там.
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._agent_enabled = True
        _stub_agent_core(self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _run(self, source: str, summary: str = "ТАРС, агент оператора.") -> None:
        def fake_run(core, payload):
            return {"ok": True, "summary": summary, "tool_calls": []}

        msg = _make_string_msg(
            json.dumps(
                {
                    "source": source,
                    "client_id": "quest-1",
                    "text": "ты здесь?",
                    "ts_ms": 100,
                }
            )
        )
        original = self.node._run_agent_sync
        self.node._run_agent_sync = fake_run
        try:
            self.node._on_avatar_command(msg)
        finally:
            self.node._run_agent_sync = original

    def test_quest_reply_goes_to_headset_tts(self) -> None:
        """Голосовой вход → ответ уходит в /avatar/tts/request, sink=headset."""
        self._run("quest")
        spoken = _published_avatar_tts(self.node)
        self.assertEqual(len(spoken), 1, "ответ агента не был озвучен")
        self.assertEqual(spoken[0]["sink"], "headset")
        self.assertIn("ТАРС, агент оператора.", spoken[0]["ssml"])

    def test_text_result_is_still_published(self) -> None:
        """Озвучка НЕ заменяет /avatar/command_result — текст нужен UI шлема."""
        self._run("quest")
        results = _published_results(self.node)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["summary"], "ТАРС, агент оператора.")

    def test_telegram_reply_is_not_spoken(self) -> None:
        """Текстовый вход → речи нет: оператор ждёт текст, а не звук в ушах."""
        self._run("telegram")
        self.assertEqual(_published_avatar_tts(self.node), [])

    def test_speak_agent_replies_false_disables_speech(self) -> None:
        """Параметр speak_agent_replies=false выключает озвучку (стенд)."""
        original = self.node._param_bool
        self.node._param_bool = lambda name, default=False: (
            False if name == "speak_agent_replies" else original(name, default)
        )
        try:
            self._run("quest")
        finally:
            self.node._param_bool = original
        self.assertEqual(_published_avatar_tts(self.node), [])

    def test_empty_summary_is_not_spoken(self) -> None:
        """Пустой summary не уходит в синтез (иначе провайдеры мрут, #2096)."""
        self._run("quest", summary="")
        self.assertEqual(_published_avatar_tts(self.node), [])
