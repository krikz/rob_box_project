"""Unit-тесты пайплайна грипа (issue #1989, шаг 4б; §7.5).

Покрывает Definition of Done карточки #1989 (инвариант 6c целевой
архитектуры: «в пайплайне грипа нет инструментов и нет истории»):

1. **Путь грипа не имеет доступа к ToolProvider** — ``tool_calls`` от
   заглушки-LLM игнорируются: в ``/voice/tts/request`` уходит только
   ``content``, ``complete`` вызывается с пустым ``tools=()``.
2. **«Без стиля»** — STT-текст уходит в TTS дословно, 0 LLM-вызовов
   (замер счётчика: stub.call_count == 0 + метрика не инкрементится).
3. **Пресет «Перевод»** — ровно 1 LLM-вызов, текст на целевом языке и
   ``language``-override в ``/voice/tts/request``.
4. **Память/история** — после прогона ``_agent_core`` остаётся ``None``,
   ``_ensure_agent_core``/журнал не вызываются (SQLite не трогается).

Используем mock-rclpy из conftest.py. Реального LLM в тестах НЕТ: подменяем
``node._grip_llm`` заглушкой с async ``complete``. Загрузка пресетов идёт из
source-tree ``src/rob_box_voice/config`` (resolve_voice_presets_path).
"""
from __future__ import annotations

import json
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from rob_box_supervisor.supervisor_node import (
    GRIP_OFF_PRESETS,
    GRIP_PTT_RESULT_TOPIC,
    GRIP_TTS_REQUEST_TOPIC,
    GRIP_VOICE_PIPELINE_TOPIC,
    VOICE_PRESET_IDS,
    AvatarSupervisor,
)


def _make_string_msg(data: str) -> MagicMock:
    """Создать фейковый std_msgs/String с .data."""
    m = MagicMock()
    m.data = data
    return m


def _pipeline_config_msg(
    llm_enabled: bool = True, preset: str = "translate", language: str = "en"
) -> MagicMock:
    return _make_string_msg(
        json.dumps(
            {"llm_enabled": llm_enabled, "preset": preset, "language": language},
            ensure_ascii=False,
        )
    )


def _published_tts(node: AvatarSupervisor) -> list[dict]:
    """Достать все опубликованные /voice/tts/request payloads (parsed JSON)."""
    pub = node._tts_request_pub
    return [json.loads(m.data) for m in pub.published]


class _FakeResponse:
    """Минимальный ответ LLM (как rob_box_llm LLMResponse)."""

    def __init__(self, content: str = "", tool_calls: tuple = ()) -> None:
        self.content = content
        self.tool_calls = tool_calls
        self.finish_reason = "stop"


class _StubGripLLM:
    """Заглушка LLM грипа: записывает число вызовов и переданные tools.

    Не является LLMProvider — пайплайн вызывает только ``complete`` с
    ``tools=()``, поэтому заглушка не обязана реализовывать весь контракт.
    """

    def __init__(self, content: str = "translated", tool_calls: tuple = ()) -> None:
        self.content = content
        self.tool_calls = tool_calls
        self.call_count: int = 0
        self.last_messages: list = []
        self.last_tools = "unset"

    async def complete(self, messages, *, tools=(), settings=None):  # noqa: ANN001
        self.call_count += 1
        self.last_messages = list(messages)
        self.last_tools = tools
        return _FakeResponse(content=self.content, tool_calls=self.tool_calls)


class TestGripPipelineTopology(unittest.TestCase):
    """Топики пайплайна грипа объявлены на этапе __init__ ноды."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_ptt_result_subscription_declared(self) -> None:
        topics = [s.topic for s in self.node._subscriptions]
        self.assertIn(GRIP_PTT_RESULT_TOPIC, topics)
        self.assertEqual(GRIP_PTT_RESULT_TOPIC, "/avatar/ptt/result")

    def test_voice_pipeline_subscription_declared(self) -> None:
        topics = [s.topic for s in self.node._subscriptions]
        self.assertIn(GRIP_VOICE_PIPELINE_TOPIC, topics)
        self.assertEqual(GRIP_VOICE_PIPELINE_TOPIC, "/avatar/voice_pipeline")

    def test_tts_request_publisher_declared(self) -> None:
        self.assertIn(GRIP_TTS_REQUEST_TOPIC, self.node._publishers)
        self.assertEqual(GRIP_TTS_REQUEST_TOPIC, "/voice/tts/request")

    def test_default_config_is_no_style(self) -> None:
        """До первого /avatar/voice_pipeline грип «без стиля»: LLM выкл."""
        self.assertFalse(self.node._pipeline_llm_enabled)
        self.assertEqual(self.node._pipeline_preset, "")
        self.assertEqual(self.node._pipeline_language, "ru")

    def test_grip_metrics_built(self) -> None:
        self.assertIn("utterances", self.node._grip_metrics)
        self.assertIn("llm_calls", self.node._grip_metrics)


class TestGripNoStyleDirect(unittest.TestCase):
    """DoD: «Без стиля» — STT-текст → TTS дословно, 0 LLM-вызовов."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _send_ptt(self, text: str = "мы начинаем") -> None:
        self.node._on_grip_ptt_result(_make_string_msg(text))

    def test_direct_verbatim_zero_llm_calls(self) -> None:
        """Default (llm off) + явный «Без стиля» из панели → дословно, 0 вызовов."""
        stub = _StubGripLLM(content="never used")
        self.node._grip_llm = stub
        self._send_ptt()

        pub = _published_tts(self.node)
        self.assertEqual(len(pub), 1)
        self.assertEqual(pub[0]["ssml"], "<speak>мы начинаем</speak>")
        self.assertEqual(pub[0]["source"], "operator")
        # Дословный текст — без language-override (язык оператора).
        self.assertNotIn("language", pub[0])
        # Замер счётчика: 0 LLM-вызовов.
        self.assertEqual(stub.call_count, 0)

    def test_panel_style_off_is_direct(self) -> None:
        """Панель шлёт llm_enabled=false + пресет — грип всё равно дословно."""
        stub = _StubGripLLM(content="never used")
        self.node._grip_llm = stub
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(False, "technical", "ru"))
        self._send_ptt("дословно")

        pub = _published_tts(self.node)
        self.assertEqual(len(pub), 1)
        self.assertEqual(pub[0]["ssml"], "<speak>дословно</speak>")
        self.assertNotIn("language", pub[0])
        self.assertEqual(stub.call_count, 0)

    def test_preset_none_is_direct(self) -> None:
        """preset=none (семантика карточки) даже при llm_enabled → 0 вызовов."""
        stub = _StubGripLLM(content="never used")
        self.node._grip_llm = stub
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(True, "none", "ru"))
        self._send_ptt("просто фраза")

        pub = _published_tts(self.node)
        self.assertEqual(len(pub), 1)
        self.assertEqual(pub[0]["ssml"], "<speak>просто фраза</speak>")
        self.assertEqual(stub.call_count, 0)

    def test_empty_ptt_ignored(self) -> None:
        stub = _StubGripLLM()
        self.node._grip_llm = stub
        self._send_ptt("")
        self._send_ptt("   ")
        self.assertEqual(_published_tts(self.node), [])
        self.assertEqual(stub.call_count, 0)


class TestGripTranslateOneCall(unittest.TestCase):
    """DoD: «Перевод» — 1 LLM-вызов, текст на целевом языке в /voice/tts/request."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._grip_llm = _StubGripLLM(content="We are starting now.")

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_translate_calls_llm_once_and_publishes_target_language(self) -> None:
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(True, "translate", "en"))
        self.node._on_grip_ptt_result(_make_string_msg("мы начинаем"))

        stub = self.node._grip_llm
        # Замер счётчика: ровно один вызов.
        self.assertEqual(stub.call_count, 1)
        pub = _published_tts(self.node)
        self.assertEqual(len(pub), 1)
        self.assertEqual(pub[0]["ssml"], "<speak>We are starting now.</speak>")
        self.assertEqual(pub[0]["source"], "operator")
        # Текст на целевом языке + language-override для tts_node (AV-28).
        self.assertEqual(pub[0]["language"], "en")

    def test_translate_with_json_envelope_ptt(self) -> None:
        """/avatar/ptt/result в JSON-обёртке {text: ...} тоже принимается."""
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(True, "translate", "en"))
        self.node._on_grip_ptt_result(
            _make_string_msg(json.dumps({"text": "доброе утро"}, ensure_ascii=False))
        )
        stub = self.node._grip_llm
        self.assertEqual(stub.call_count, 1)
        # Заглушка вернула content="We are starting now." — см. setUp.
        pub = _published_tts(self.node)
        self.assertEqual(pub[0]["ssml"], "<speak>We are starting now.</speak>")

    def test_memory_and_history_untouched(self) -> None:
        """DoD: запись в память/историю отсутствует — AgentCore/журнал не создаются."""
        self.node._ensure_agent_core = MagicMock(  # type: ignore[method-assign]
            wraps=self.node._ensure_agent_core
        )
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(True, "translate", "en"))
        self.node._on_grip_ptt_result(_make_string_msg("мы начинаем"))

        # Грип не строит AgentCore/DSM/журнал и не пишет в SQLite.
        self.assertIsNone(self.node._agent_core)
        self.assertIsNone(self.node._operator_journal)
        self.node._ensure_agent_core.assert_not_called()  # type: ignore[attr-defined]


class TestGripToolCallsIgnored(unittest.TestCase):
    """DoD (инвариант 6c): tool_calls от заглушки-LLM игнорируются.

    У пути грипа нет ToolProvider: complete вызывается с tools=(), а в TTS
    уходит только content — даже если модель «захотела» инструмент.
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._grip_llm = _StubGripLLM(
            content="переписанная фраза",
            tool_calls=(SimpleNamespace(id="c1", name="navigate_to_landmark", arguments={}),),
        )

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_tool_calls_are_dropped_content_is_published(self) -> None:
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(True, "lenin", "ru"))
        self.node._on_grip_ptt_result(_make_string_msg("поехали к воротам"))

        stub = self.node._grip_llm
        self.assertEqual(stub.call_count, 1)
        # Полный ответ модели содержал tool_call — в TTS ушёл только content.
        pub = _published_tts(self.node)
        self.assertEqual(len(pub), 1)
        self.assertEqual(pub[0]["ssml"], "<speak>переписанная фраза</speak>")
        # complete вызван без инструментов (нет ToolProvider).
        self.assertEqual(list(stub.last_tools or []), [])


class TestGripStyleSingleCall(unittest.TestCase):
    """Стилизующий пресет — 1 LLM-вызов, content уходит в TTS."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()
        self.node._grip_llm = _StubGripLLM(content="фраза в стиле")

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_style_preset_single_call(self) -> None:
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(True, "technical", "ru"))
        self.node._on_grip_ptt_result(_make_string_msg("ну короче поехали"))

        stub = self.node._grip_llm
        self.assertEqual(stub.call_count, 1)
        pub = _published_tts(self.node)
        self.assertEqual(len(pub), 1)
        self.assertEqual(pub[0]["ssml"], "<speak>фраза в стиле</speak>")

    def test_llm_failure_falls_back_to_direct(self) -> None:
        """Сбой LLM → честный fallback на дословный TTS (0 доп. вызовов)."""

        class _FailingLLM:
            call_count = 0

            async def complete(self, messages, *, tools=(), settings=None):  # noqa: ANN001
                _FailingLLM.call_count += 1
                raise RuntimeError("provider down")

        self.node._grip_llm = _FailingLLM()
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(True, "lenin", "ru"))
        self.node._on_grip_ptt_result(_make_string_msg("исходная фраза"))

        self.assertEqual(_FailingLLM.call_count, 1)
        pub = _published_tts(self.node)
        self.assertEqual(len(pub), 1)
        # Дословный текст без language-override.
        self.assertEqual(pub[0]["ssml"], "<speak>исходная фраза</speak>")
        self.assertNotIn("language", pub[0])


class TestGripConfigHandling(unittest.TestCase):
    """Валидация /avatar/voice_pipeline — битый ввод не роняет ноду."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_malformed_config_keeps_previous_state(self) -> None:
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(True, "translate", "en"))
        # Битый JSON — конфиг не меняется.
        self.node._on_grip_voice_pipeline(_make_string_msg("{not json"))
        self.assertTrue(self.node._pipeline_llm_enabled)
        self.assertEqual(self.node._pipeline_preset, "translate")
        self.assertEqual(self.node._pipeline_language, "en")

    def test_unknown_language_normalized_to_default(self) -> None:
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(True, "translate", "xx"))
        self.assertEqual(self.node._pipeline_language, "ru")

    def test_unknown_preset_treated_as_no_style(self) -> None:
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(True, "no_such_style", "ru"))
        self.assertEqual(self.node._pipeline_preset, "")

    def test_known_style_preset_accepted(self) -> None:
        for preset in VOICE_PRESET_IDS:
            self.node._on_grip_voice_pipeline(
                _pipeline_config_msg(True, preset, "ru")
            )
            self.assertEqual(self.node._pipeline_preset, preset)

    def test_off_presets_accepted(self) -> None:
        for preset in GRIP_OFF_PRESETS:
            self.node._on_grip_voice_pipeline(_pipeline_config_msg(True, preset, "ru"))
            self.assertEqual(self.node._pipeline_preset, preset)


class TestGripMetricsCounter(unittest.TestCase):
    """DoD «замер счётчика»: avatar_grip_llm_calls_total инкрементится ровно
    на реальные LLM-вызовы (direct → 0, translate/style → 1)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _spy_counter(self, name: str) -> tuple[list, Any]:
        """Обернуть .labels() счётчика, считая инкременты по label'ам."""
        counter = self.node._grip_metrics[name]
        original_labels = counter.labels
        seen: list = []

        def spy_labels(*args, **kwargs):
            inner = original_labels(*args, **kwargs)
            original_inc = inner.inc
            seen.append(dict(kwargs))

            def inc(*a, **kw):
                seen.append(("inc", dict(kwargs)))
                return original_inc(*a, **kw)

            inner.inc = inc
            return inner

        counter.labels = spy_labels  # type: ignore[attr-defined]
        return seen, counter

    def test_direct_zero_llm_call_counter(self) -> None:
        """«Без стиля»: счётчик LLM-вызовов НЕ инкрементится (0)."""
        seen, _ = self._spy_counter("llm_calls")
        stub = _StubGripLLM(content="never used")
        self.node._grip_llm = stub
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(False, "technical", "ru"))
        self.node._on_grip_ptt_result(_make_string_msg("мы начинаем"))

        incs = [entry for entry in seen if isinstance(entry, tuple)]
        self.assertEqual(incs, [])
        self.assertEqual(stub.call_count, 0)

    def test_translate_one_llm_call_counter(self) -> None:
        """«Перевод»: счётчик инкрементится ровно один раз с mode=translate."""
        seen, _ = self._spy_counter("llm_calls")
        self.node._grip_llm = _StubGripLLM(content="We are starting now.")
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(True, "translate", "en"))
        self.node._on_grip_ptt_result(_make_string_msg("мы начинаем"))

        incs = [entry for entry in seen if isinstance(entry, tuple)]
        self.assertEqual(len(incs), 1, f"llm_calls seen: {seen}")
        labels = [entry for entry in seen if not isinstance(entry, tuple)]
        self.assertIn({"mode": "translate"}, labels)

    def test_utterance_counter_records_mode(self) -> None:
        seen, _ = self._spy_counter("utterances")
        self.node._grip_llm = _StubGripLLM(content="x")
        self.node._on_grip_voice_pipeline(_pipeline_config_msg(False, "", "ru"))
        self.node._on_grip_ptt_result(_make_string_msg("дословно"))

        labels = [entry for entry in seen if not isinstance(entry, tuple)]
        self.assertIn({"mode": "direct"}, labels)


class TestGripPureHelpers(unittest.TestCase):
    """Pure-хелперы пайплайна (без rclpy): классификация и извлечение текста."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_classify_off_when_llm_disabled(self) -> None:
        for preset in ("translate", "lenin", "technical"):
            self.assertEqual(
                self.node._classify_grip_preset(preset, llm_enabled=False), "direct"
            )

    def test_classify_translate(self) -> None:
        self.assertEqual(
            self.node._classify_grip_preset("translate", llm_enabled=True), "translate"
        )

    def test_classify_style(self) -> None:
        for preset in ("technical", "lenin", "street", "business"):
            self.assertEqual(
                self.node._classify_grip_preset(preset, llm_enabled=True), "style"
            )

    def test_extract_ptt_text_plain_and_json(self) -> None:
        self.assertEqual(self.node._extract_grip_ptt_text("привет мир"), "привет мир")
        self.assertEqual(
            self.node._extract_grip_ptt_text(json.dumps({"text": "  хай  "})), "хай"
        )
        self.assertEqual(self.node._extract_grip_ptt_text(""), "")
        # JSON без text → пусто.
        self.assertEqual(self.node._extract_grip_ptt_text("{}"), "")


if __name__ == "__main__":
    unittest.main()
