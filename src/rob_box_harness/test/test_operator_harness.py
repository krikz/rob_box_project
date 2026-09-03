"""Tests for ``OperatorHarness`` — каркас супервизор-агента (AV-21).

Acceptance (1:1 с ``docs/plans/2026-09-02-avatar-supervisor-agent-design.md``):

- Системный промпт загружается из файла и непустой.
- Инструменты берут схемы из ``rob_box_core.tool_catalog``.
- **Тест-инвариант**: ни одного собственного объявления JSON-Schema
  в ``operator.py`` (grep-проверка, аналог ``_validate_tools_in_prompt``
  в dialogue_node для master prompt).
- init() падает, если в каталоге нет инструмента из ``OPERATOR_TOOL_NAMES``
  (защита от drift).
- ``"скажи привет"`` → один tool-call ``say`` → ``ok=True``.
- Команда, на которую нет инструмента → ``ok=False``, ``summary="no_tool"``,
  **без** выдумывания действия.
- ``empty input`` → ``ok=False``, ``summary="empty_input"``.

Запускается через mock-LLM (``FakeLLMProvider``). Реальных вызовов
провайдера в тестах **не** должно быть (AGENTS.md + карточка).
"""

from __future__ import annotations

import re
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from rob_box_harness.config import HarnessConfig
from rob_box_harness.harnesses.operator import (
    OPERATOR_TOOL_NAMES,
    OperatorHarness,
    OperatorState,
)
from rob_box_llm.providers.fake import FakeLLMProvider
from rob_box_llm.provider import LLMMessage, ToolCall


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────


def _make_config() -> HarnessConfig:
    return HarnessConfig(harness="operator", name="operator_agent")


def _say_call(text: str) -> ToolCall:
    return ToolCall(id="t1", name="say", arguments={"text": text})


def _play_animation_call(name: str) -> ToolCall:
    return ToolCall(id="t2", name="play_animation", arguments={"animation": name})


def _make_fake_llm_with_calls(*calls: ToolCall) -> FakeLLMProvider:
    """Build a FakeLLMProvider pre-loaded with one or more scripted tool_calls.

    The fake yields ``LLMResponse(tool_calls=calls)`` — picked up by
    ``complete()`` (which is what ``OperatorHarness.step`` calls).
    """
    from rob_box_llm.provider import LLMResponse

    fake = FakeLLMProvider()
    fake.queue_response(LLMResponse(content="", tool_calls=list(calls), finish_reason="tool_calls"))
    return fake


# ─────────────────────────────────────────────────────────────────────────────
# AC #4: System prompt is loaded and non-empty
# ─────────────────────────────────────────────────────────────────────────────


class TestSystemPrompt:
    """Промпт — отдельный файл, не строка в коде."""

    @pytest.mark.asyncio
    async def test_default_prompt_file_loads(self):
        h = OperatorHarness(_make_config())
        await h.init()
        assert h._system_prompt, "system prompt must be non-empty (AV-21 AC #4)"
        assert "оператор" in h._system_prompt.lower()
        await h.teardown()

    @pytest.mark.asyncio
    async def test_missing_prompt_file_raises(self, tmp_path):
        # Override the package-relative prompts dir by passing an explicit
        # filename that doesn't exist in the bundled prompts/ directory.
        h = OperatorHarness(_make_config(), prompt_file="nope.txt")
        with pytest.raises(RuntimeError, match="operator system prompt not found"):
            await h.init()
        await h.teardown()

    @pytest.mark.asyncio
    async def test_empty_prompt_file_raises(self, monkeypatch):
        """Если файл промпта пустой — init() падает.

        Делаем это через monkey-patch ``_load_system_prompt``, чтобы не
        трогать реальный файл в репо (другие тесты могут читать его
        параллельно).
        """
        from rob_box_harness.harnesses import operator as op_module

        h = OperatorHarness(_make_config())

        def _empty_loader(_self, _filename):
            raise RuntimeError("operator system prompt is empty: test stub")

        monkeypatch.setattr(op_module.OperatorHarness, "_load_system_prompt", _empty_loader)
        with pytest.raises(RuntimeError, match="operator system prompt is empty"):
            await h.init()
        await h.teardown()


# ─────────────────────────────────────────────────────────────────────────────
# AC #5: Tools come from rob_box_core.tool_catalog
# ─────────────────────────────────────────────────────────────────────────────


class TestToolSpecsFromCatalog:
    """``_tool_specs`` собирается из catalog-а, без локальных объявлений."""

    @pytest.mark.asyncio
    async def test_operator_tool_names_match_catalog(self):
        """``OPERATOR_TOOL_NAMES`` ⊆ TOOL_CATALOG."""
        from rob_box_core.tool_catalog import TOOL_CATALOG

        catalog_names = {entry.name for entry in TOOL_CATALOG}
        for name in OPERATOR_TOOL_NAMES:
            assert name in catalog_names, (
                f"operator tool {name!r} missing from rob_box_core.tool_catalog"
            )

    @pytest.mark.asyncio
    async def test_tool_specs_have_openai_shape(self):
        h = OperatorHarness(_make_config())
        await h.init()
        assert len(h._tool_specs) == len(OPERATOR_TOOL_NAMES)
        for spec in h._tool_specs:
            assert spec["type"] == "function"
            assert "function" in spec
            fn = spec["function"]
            assert "name" in fn
            assert "description" in fn
            assert "parameters" in fn
            assert fn["parameters"]["type"] == "object"
        await h.teardown()

    def test_no_local_schema_declaration_in_operator_module(self):
        """Тест-инвариант (см. design.md §6.5): в operator.py нет
        собственного объявления JSON-Schema. Grep-проверка.

        Разрешено: упоминание ``"type": "object"`` в комментариях/docstring.
        Запрещено: литерал-объявление параметров тул (мы должны брать
        их из rob_box_core.tool_catalog).
        """
        from rob_box_harness.harnesses import operator as op_module

        src = Path(op_module.__file__).read_text(encoding="utf-8")
        # No dict literal with "properties" + "type" combo outside of
        # a triple-quoted docstring/comment.
        # Strip docstrings and comments to avoid false positives.
        cleaned = re.sub(r'"""[\s\S]*?"""', "", src)
        cleaned = re.sub(r"'''[\s\S]*?'''", "", cleaned)
        # Search for the schema-shaped substring.
        schema_pat = re.compile(r'"properties"\s*:\s*\{')
        matches = schema_pat.findall(cleaned)
        assert not matches, (
            f"operator.py must not declare local JSON-Schema for tools "
            f"(found {len(matches)} schema literals; "
            "use rob_box_core.tool_catalog instead — see design.md §2 + §6.5)"
        )

    @pytest.mark.asyncio
    async def test_unknown_tool_name_raises_during_init(self):
        """Защита от drift: если OPERATOR_TOOL_NAMES содержит имя,
        которого нет в каталоге — init() падает."""
        from rob_box_core.tool_catalog import get_tool

        # Sanity check: "definitely_not_a_tool" is not in the catalog.
        with pytest.raises(KeyError):
            get_tool("definitely_not_a_tool")

        # Simulate drift by monkey-patching OPERATOR_TOOL_NAMES.
        from rob_box_harness.harnesses import operator as op_module

        original = op_module.OPERATOR_TOOL_NAMES
        op_module.OPERATOR_TOOL_NAMES = ("say", "definitely_not_a_tool")
        try:
            h = OperatorHarness(_make_config())
            with pytest.raises(KeyError):
                await h.init()
        finally:
            op_module.OPERATOR_TOOL_NAMES = original
            await h.teardown()


# ─────────────────────────────────────────────────────────────────────────────
# AC #6: «Скажи привет» → один tool-call → ok=True
# AC #7: Не-tool → ok=False, summary="no_tool"
# AC #4: empty input → ok=False, summary="empty_input"
# ─────────────────────────────────────────────────────────────────────────────


class TestStepBehavior:
    """Поведение ``step()`` под FakeLLM."""

    @pytest.mark.asyncio
    async def test_say_tool_call_returns_ok(self):
        h = OperatorHarness(_make_config(), llm=_make_fake_llm_with_calls(_say_call("Привет")))
        await h.init()
        out = await h.step({"source": "telegram", "client_id": "u1", "text": "скажи привет"})
        assert out["ok"] is True
        assert out["summary"] == "ok"
        assert len(out["tool_calls"]) == 1
        call = out["tool_calls"][0]
        assert call["name"] == "say"
        assert call["arguments"]["text"] == "Привет"
        assert call["result"] == "ok"  # stub handler returns "ok"
        # state was updated
        assert h.state.last_command_text == "скажи привет"
        assert h.state.last_command_source == "telegram"
        assert h.state.turn_count == 1
        await h.teardown()

    @pytest.mark.asyncio
    async def test_no_tool_call_returns_no_tool(self):
        """AC #7: LLM не запросил инструментов → ok=False, summary='no_tool'."""
        from rob_box_llm.provider import LLMResponse

        # Queue a response that explicitly has empty tool_calls.
        fake = FakeLLMProvider()
        fake.queue_response(LLMResponse(content="plain text answer", tool_calls=()))
        h = OperatorHarness(_make_config(), llm=fake)
        await h.init()
        out = await h.step({"source": "quest", "client_id": "q1", "text": "привет"})
        assert out["ok"] is False
        assert out["summary"] == "no_tool"
        assert out["tool_calls"] == []
        await h.teardown()

    @pytest.mark.asyncio
    async def test_empty_text_returns_empty_input(self):
        h = OperatorHarness(_make_config(), llm=_make_fake_llm_with_calls(_say_call("ignored")))
        await h.init()
        out = await h.step({"source": "telegram", "text": "   "})
        assert out["ok"] is False
        assert out["summary"] == "empty_input"
        assert out["tool_calls"] == []
        await h.teardown()

    @pytest.mark.asyncio
    async def test_unknown_tool_name_returns_no_tool_in_catalog(self):
        """AC #7 (расширенный): LLM попросил инструмент, которого нет
        в каталоге — supervisor не выдумывает результат (ADR-0018 в
        рантайме)."""
        from rob_box_core.tool_catalog import TOOL_CATALOG
        catalog_names = {entry.name for entry in TOOL_CATALOG}
        # Pick a name that's definitely not in the catalog
        bogus = "fly_to_the_moon"
        assert bogus not in catalog_names
        h = OperatorHarness(_make_config(), llm=_make_fake_llm_with_calls(
            ToolCall(id="t1", name=bogus, arguments={})
        ))
        await h.init()
        out = await h.step({"source": "quest", "text": "полетели"})
        assert out["ok"] is False
        assert out["summary"] == "no_tool_in_catalog"
        # The bogus call is still recorded (so the caller knows what was asked)
        assert len(out["tool_calls"]) == 1
        assert out["tool_calls"][0]["name"] == bogus
        await h.teardown()

    @pytest.mark.asyncio
    async def test_llm_exception_returns_llm_error(self):
        """LLM бросил исключение — supervisor не падает, отдаёт llm_error."""

        class _BoomProvider(FakeLLMProvider):
            async def complete(self, *args, **kwargs):
                raise RuntimeError("simulated 502 from upstream")

        h = OperatorHarness(_make_config(), llm=_BoomProvider())
        await h.init()
        out = await h.step({"source": "telegram", "text": "тест"})
        assert out["ok"] is False
        assert out["summary"].startswith("llm_error:")
        assert "RuntimeError" in out["summary"]
        assert out["tool_calls"] == []
        await h.teardown()


# ─────────────────────────────────────────────────────────────────────────────
# Lifecycle: state survives step(), snapshot includes last_command
# ─────────────────────────────────────────────────────────────────────────────


class TestLifecycle:
    @pytest.mark.asyncio
    async def test_state_is_operator_state(self):
        h = OperatorHarness(_make_config())
        await h.init()
        assert isinstance(h.state, OperatorState)
        await h.teardown()

    @pytest.mark.asyncio
    async def test_multiple_steps_increment_turn_count(self):
        from rob_box_llm.provider import LLMResponse

        # Queue three separate responses, one per step().
        fake = FakeLLMProvider()
        for _ in range(3):
            fake.queue_response(
                LLMResponse(content="", tool_calls=(_say_call("x"),), finish_reason="tool_calls")
            )
        h = OperatorHarness(_make_config(), llm=fake)
        await h.init()
        for i in range(3):
            await h.step({"source": "telegram", "text": f"команда {i}"})
        assert h.state.turn_count == 3
        await h.teardown()
