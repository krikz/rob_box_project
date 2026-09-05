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
4. **AC #8** LLM исключение → voice_input_mode swap НЕ блокирует
   последующие команды (try/finally возвращает в исходное состояние).
5. **AC #9** метрики инкрементнулись после успешной команды.
6. **AC #3 (negative)** невалидный JSON → ok=false,
   summary="malformed_input".
7. **AC #8 + infra** swap.apply+exit последовательность: _apply_voice_mode
   вызывается для обоих концов, и в finally НЕ происходит «лишнего»
   restore когда prev_mode=None.
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
    AGENT_DURING_VOICE_MODE_DEFAULT,
    AVATAR_COMMAND_RESULT_TOPIC,
    AVATAR_COMMAND_TOPIC,
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
        """Параметры agent_enabled / system_prompt_file / agent_during_voice_mode
        объявлены через declare_parameter."""
        self.assertTrue(self.node.has_parameter("agent_enabled"))
        self.assertTrue(self.node.has_parameter("system_prompt_file"))
        self.assertTrue(self.node.has_parameter("agent_during_voice_mode"))
        # defaults
        self.assertEqual(
            self.node.get_parameter("system_prompt_file").value,
            "operator_system_prompt.txt",
        )
        self.assertEqual(
            self.node.get_parameter("agent_during_voice_mode").value,
            AGENT_DURING_VOICE_MODE_DEFAULT,
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


class TestVoiceModeSwapTryFinally(unittest.TestCase):
    """AC #8: try/finally _voice_mode_swap корректно отрабатывает
    даже если core/agent бросает исключение. Свап вызывает
    _apply_voice_mode для apply+exit, и prev_mode=None НЕ вызывает
    лишнего restore."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._agent_enabled = True
        _stub_agent_core(self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_swap_calls_apply_voice_mode(self) -> None:
        """Свап.enter вызывает _apply_voice_mode(agent_during_voice_mode)."""
        calls: list[str] = []
        original_apply = self.node._apply_voice_mode

        def spy_apply(mode):
            calls.append(("apply", mode))
            return (False, MONITOR_MODE_REASON)

        self.node._apply_voice_mode = spy_apply
        try:
            with self.node._voice_mode_swap():
                pass
        finally:
            self.node._apply_voice_mode = original_apply

        # Должен быть минимум один вызов apply для входа (apply=off).
        applied_modes = [m for tag, m in calls if tag == "apply"]
        self.assertIn(self.node._agent_during_voice_mode, applied_modes)

    def test_swap_no_restore_when_prev_unknown(self) -> None:
        """AC #8 (negative): prev_mode=None → в finally НЕ делаем restore."""
        # _voice_input_mode_before_swap is None by default.
        self.assertIsNone(self.node._voice_input_mode_before_swap)
        calls: list[str] = []
        original_apply = self.node._apply_voice_mode

        def spy_apply(mode):
            calls.append(mode)
            return (False, MONITOR_MODE_REASON)

        self.node._apply_voice_mode = spy_apply
        try:
            with self.node._voice_mode_swap():
                pass
        finally:
            self.node._apply_voice_mode = original_apply

        # Только ОДИН вызов apply (на входе). Restore НЕ вызван.
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0], self.node._agent_during_voice_mode)
        # Snapshot очищен.
        self.assertIsNone(self.node._voice_input_mode_before_swap)

    def test_swap_restores_when_prev_known_and_different(self) -> None:
        """Когда знаем prev_mode и он != agent_during_voice_mode →
        в finally вызываем restore."""
        self.node._voice_input_mode_before_swap = "respeaker"
        # Чтобы apply отличался, agent_during_voice_mode должен быть != "respeaker".
        # Default "off" != "respeaker" → restore будет вызван.
        calls: list[tuple[str, str]] = []
        original_apply = self.node._apply_voice_mode

        def spy_apply(mode):
            calls.append(("apply", mode))
            return (False, MONITOR_MODE_REASON)

        self.node._apply_voice_mode = spy_apply
        try:
            with self.node._voice_mode_swap():
                pass
        finally:
            self.node._apply_voice_mode = original_apply

        applied_modes = [m for tag, m in calls if tag == "apply"]
        self.assertEqual(len(applied_modes), 2)
        # Первый — apply=off (вход), второй — restore=respeaker (выход).
        self.assertEqual(applied_modes[0], "off")
        self.assertEqual(applied_modes[1], "respeaker")

    def test_swap_skips_restore_when_prev_equals_current(self) -> None:
        """Если prev_mode == agent_during_voice_mode → restore не нужен (noop)."""
        self.node._agent_during_voice_mode = "off"
        self.node._voice_input_mode_before_swap = "off"
        calls: list[str] = []
        original_apply = self.node._apply_voice_mode

        def spy_apply(mode):
            calls.append(mode)
            return (False, MONITOR_MODE_REASON)

        self.node._apply_voice_mode = spy_apply
        try:
            with self.node._voice_mode_swap():
                pass
        finally:
            self.node._apply_voice_mode = original_apply

        # Один вызов (на входе). Restore пропущен.
        self.assertEqual(calls, ["off"])

    def test_llm_exception_swap_finally_runs(self) -> None:
        """AC #8: исключение внутри swap → finally всё равно выполняется.
        Outer-error catch возвращает ok=False, summary='outer_error: <type>'."""
        calls: list[str] = []
        original_apply = self.node._apply_voice_mode

        def spy_apply(mode):
            calls.append(mode)
            return (False, MONITOR_MODE_REASON)

        self.node._apply_voice_mode = spy_apply

        def exploding_run(core, payload):
            raise RuntimeError("simulated LLM crash")

        original = self.node._run_agent_sync
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
            self.node._run_agent_sync = original
            self.node._apply_voice_mode = original_apply

        # _apply_voice_mode был вызван минимум 1 раз (apply на входе).
        # Если finally не отработал — будет 0.
        self.assertGreaterEqual(len(calls), 1)
        # Публикация — ok=False, summary указывает на llm_error (через
        # try/except в _on_avatar_command — exception из _run_agent_sync
        # ловится там).
        results = _published_results(self.node)
        self.assertEqual(len(results), 1)
        self.assertFalse(results[0]["ok"])
        self.assertTrue(
            "llm_error" in results[0]["summary"] or "outer_error" in results[0]["summary"],
            f"expected llm_error or outer_error summary, got {results[0]['summary']!r}",
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


class TestCaptureCurrentVoiceMode(unittest.TestCase):
    """В Phase 1 (без GetParameters клиента) capture возвращает None —
    swap НЕ делает restore в finally."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_capture_returns_none_phase1(self) -> None:
        self.assertIsNone(self.node._capture_current_voice_mode())


class TestAgentCommandSources(unittest.TestCase):
    """AGENT_COMMAND_SOURCES содержит quest/telegram; расширяемо через логирование."""

    def test_known_sources_listed(self) -> None:
        self.assertIn("quest", AGENT_COMMAND_SOURCES)
        self.assertIn("telegram", AGENT_COMMAND_SOURCES)


if __name__ == "__main__":
    unittest.main()
