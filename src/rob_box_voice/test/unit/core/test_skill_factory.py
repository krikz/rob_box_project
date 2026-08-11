"""Unit tests for :mod:`rob_box_voice.core.skill_factory`.

The factory collapses 4–5 near-identical ``try / except / as_tool`` blocks
from :py:meth:`DialogueNode._build_skills` into one loop. Tests cover:

* Successful construction (prompt file → real prompt, missing → fallback).
* Per-spec failure isolation (one bad spec does NOT abort the others).
* ``prompt_kwarg`` override (MusicSkill uses ``prompt_template``,
  the rest use ``prompt``).
* Output ordering matches input spec order.
* Empty specs list returns empty list.
"""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.skill_factory import (
    SkillFactorySpec,
    build_skill_tools,
)


# ---------------------------------------------------------------------------
# Fixtures and helpers
# ---------------------------------------------------------------------------


class _FakeLogger:
    """In-memory logger that captures messages for assertions."""

    def __init__(self) -> None:
        self.records: list[tuple[str, str]] = []

    def info(self, msg: str) -> None:
        self.records.append(("info", msg))

    def warning(self, msg: str) -> None:
        self.records.append(("warning", msg))

    def error(self, msg: str) -> None:
        self.records.append(("error", msg))


class _FakeSkill:
    """Stand-in for a BaseSkill subclass.

    Records constructor kwargs, returns a sentinel tool from ``as_tool()``.
    Can be configured to raise from ``__init__`` or ``as_tool``.
    """

    def __init__(
        self,
        *,
        adapter: Any,
        model: Any,
        prompt: str = "",
        name: str = "",
        raise_on_init: Exception | None = None,
        raise_on_as_tool: Exception | None = None,
        tool_marker: str = "tool",
        **kwargs: Any,
    ) -> None:
        if raise_on_init is not None:
            raise raise_on_init
        self.adapter = adapter
        self.model = model
        self.prompt = prompt
        self.name = name
        self.kwargs = kwargs
        self._raise_on_as_tool = raise_on_as_tool
        self._tool_marker = tool_marker

    def as_tool(self, *, tool_name: str, tool_description: str) -> Any:
        if self._raise_on_as_tool is not None:
            raise self._raise_on_as_tool
        return f"<{self._tool_marker} {tool_name}: {tool_description}>"


@pytest.fixture
def logger() -> _FakeLogger:
    return _FakeLogger()


def _spec(
    name: str = "TestSkill",
    *,
    prompt_file: str = "skills/test_prompt.txt",
    fallback: str = "fallback prompt",
    prompt_kwarg: str = "prompt",
    factory_kwargs: dict[str, Any] | None = None,
    skill_class: type[_FakeSkill] = _FakeSkill,
    info_label: str | None = None,
) -> SkillFactorySpec:
    return SkillFactorySpec(
        display_name=name,
        skill_class=skill_class,
        prompt_file=prompt_file,
        fallback_prompt=fallback,
        tool_name=f"handle_{name.lower()}",
        tool_description=f"{name} does things",
        factory_kwargs=factory_kwargs or {},
        prompt_kwarg=prompt_kwarg,
        info_label=info_label,
    )


# ---------------------------------------------------------------------------
# Successful path
# ---------------------------------------------------------------------------


class TestSuccessfulBuild:
    def test_returns_one_tool_per_spec(
        self, logger: _FakeLogger
    ) -> None:
        specs = [_spec("Alpha"), _spec("Beta"), _spec("Gamma")]
        prompts = {s.prompt_file: f"prompt-for-{s.display_name}" for s in specs}

        def loader(path: str) -> str:
            return prompts.get(path, "")

        tools = build_skill_tools(
            specs, adapter=object(), model=object(),
            prompt_loader=loader, logger=logger,
        )
        assert len(tools) == 3
        assert tools[0] == "<tool handle_alpha: Alpha does things>"
        assert tools[1] == "<tool handle_beta: Beta does things>"
        assert tools[2] == "<tool handle_gamma: Gamma does things>"

    def test_preserves_input_order(
        self, logger: _FakeLogger
    ) -> None:
        specs = [_spec("Z"), _spec("A"), _spec("M")]
        tools = build_skill_tools(
            specs, adapter=object(), model=object(),
            prompt_loader=lambda _: "p", logger=logger,
        )
        names = [t.split()[1].split(":")[0] for t in tools]
        assert names == ["handle_z", "handle_a", "handle_m"]

    def test_logs_info_per_loaded_skill(
        self, logger: _FakeLogger
    ) -> None:
        build_skill_tools(
            [_spec("Foo")], adapter=object(), model=object(),
            prompt_loader=lambda _: "prompt", logger=logger,
        )
        assert any(
            level == "info" and "✅" in msg and "Foo" in msg
            for level, msg in logger.records
        )


# ---------------------------------------------------------------------------
# Prompt loading
# ---------------------------------------------------------------------------


class TestPromptLoading:
    def test_uses_file_when_present(
        self, logger: _FakeLogger
    ) -> None:
        captured: dict[str, Any] = {}

        class _Capture(_FakeSkill):
            def __init__(self, **kwargs: Any) -> None:
                captured.update(kwargs)
                super().__init__(**kwargs)

        build_skill_tools(
            [_spec(skill_class=_Capture)],
            adapter=object(), model=object(),
            prompt_loader=lambda _: "real prompt from file",
            logger=logger,
        )
        assert captured["prompt"] == "real prompt from file"

    def test_uses_fallback_when_loader_returns_empty(
        self, logger: _FakeLogger
    ) -> None:
        captured: dict[str, Any] = {}

        class _Capture(_FakeSkill):
            def __init__(self, **kwargs: Any) -> None:
                captured.update(kwargs)
                super().__init__(**kwargs)

        build_skill_tools(
            [_spec(skill_class=_Capture, fallback="fallback text")],
            adapter=object(), model=object(),
            prompt_loader=lambda _: "",
            logger=logger,
        )
        assert captured["prompt"] == "fallback text"

    def test_uses_fallback_when_loader_returns_whitespace(
        self, logger: _FakeLogger
    ) -> None:
        captured: dict[str, Any] = {}

        class _Capture(_FakeSkill):
            def __init__(self, **kwargs: Any) -> None:
                captured.update(kwargs)
                super().__init__(**kwargs)

        build_skill_tools(
            [_spec(skill_class=_Capture, fallback="FALLBACK")],
            adapter=object(), model=object(),
            prompt_loader=lambda _: "   \n  ",
            logger=logger,
        )
        assert captured["prompt"] == "FALLBACK"

    def test_logs_warning_when_using_fallback(
        self, logger: _FakeLogger
    ) -> None:
        build_skill_tools(
            [_spec(fallback="FB")],
            adapter=object(), model=object(),
            prompt_loader=lambda _: "",
            logger=logger,
        )
        assert any(
            level == "warning" and "fallback" in msg.lower()
            for level, msg in logger.records
        )


# ---------------------------------------------------------------------------
# Failure isolation
# ---------------------------------------------------------------------------


class TestFailureIsolation:
    def test_one_bad_spec_does_not_abort_others(
        self, logger: _FakeLogger
    ) -> None:
        good = _spec("Good")
        bad = _spec(
            "Bad",
            factory_kwargs={"raise_on_init": RuntimeError("boom")},
        )
        tools = build_skill_tools(
            [good, bad],
            adapter=object(), model=object(),
            prompt_loader=lambda _: "p",
            logger=logger,
        )
        assert len(tools) == 1
        assert "Good" in tools[0]

    def test_logs_error_for_failed_spec(
        self, logger: _FakeLogger
    ) -> None:
        build_skill_tools(
            [_spec("Broken", factory_kwargs={"raise_on_init": RuntimeError("oh no")})],
            adapter=object(), model=object(),
            prompt_loader=lambda _: "p",
            logger=logger,
        )
        assert any(
            level == "error" and "Broken" in msg and "oh no" in msg
            for level, msg in logger.records
        )

    def test_failure_in_as_tool_skips_skill(
        self, logger: _FakeLogger
    ) -> None:
        good = _spec("Good")
        bad = _spec(
            "Tool",
            factory_kwargs={"raise_on_as_tool": RuntimeError("as_tool failed")},
        )
        tools = build_skill_tools(
            [good, bad],
            adapter=object(), model=object(),
            prompt_loader=lambda _: "p",
            logger=logger,
        )
        assert len(tools) == 1
        assert "Good" in tools[0]

    def test_failed_specs_are_skipped_not_aborted(
        self, logger: _FakeLogger
    ) -> None:
        bad1 = _spec("Bad1", factory_kwargs={"raise_on_init": ValueError("e1")})
        good = _spec("Good")
        bad2 = _spec("Bad2", factory_kwargs={"raise_on_init": ValueError("e2")})
        tools = build_skill_tools(
            [bad1, good, bad2],
            adapter=object(), model=object(),
            prompt_loader=lambda _: "p",
            logger=logger,
        )
        assert len(tools) == 1
        assert "Good" in tools[0]
        error_msgs = [m for l, m in logger.records if l == "error"]
        assert sum("Bad1" in m for m in error_msgs) == 1
        assert sum("Bad2" in m for m in error_msgs) == 1


# ---------------------------------------------------------------------------
# prompt_kwarg override
# ---------------------------------------------------------------------------


class TestPromptKwargOverride:
    def test_music_style_prompt_template_kwarg(
        self, logger: _FakeLogger
    ) -> None:
        captured: dict[str, Any] = {}

        class _Capture(_FakeSkill):
            def __init__(self, **kwargs: Any) -> None:
                # MusicSkill uses prompt_template, not prompt.
                captured.update(kwargs)
                # Bypass _FakeSkill's prompt= assertion by deleting it
                kwargs.pop("prompt", None)
                super().__init__(**kwargs)

        build_skill_tools(
            [_spec(skill_class=_Capture, prompt_kwarg="prompt_template")],
            adapter=object(), model=object(),
            prompt_loader=lambda _: "music prompt",
            logger=logger,
        )
        assert "prompt_template" in captured
        assert "prompt" not in captured
        assert captured["prompt_template"] == "music prompt"


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------


class TestPromptRender:
    def test_prompt_render_called_when_provided(
        self, logger: _FakeLogger
    ) -> None:
        captured: dict[str, Any] = {}

        class _Capture(_FakeSkill):
            def __init__(self, **kwargs: Any) -> None:
                captured.update(kwargs)
                super().__init__(**kwargs)

        def render(text: str) -> str:
            return f"[RENDERED] {text}"

        spec = SkillFactorySpec(
            display_name="X",
            skill_class=_Capture,
            prompt_file="x",
            fallback_prompt="F",
            tool_name="handle_x",
            tool_description="x",
            prompt_render=render,
        )
        build_skill_tools(
            [spec], adapter=object(), model=object(),
            prompt_loader=lambda _: "raw", logger=logger,
        )
        assert captured["prompt"] == "[RENDERED] raw"

    def test_no_render_means_identity(
        self, logger: _FakeLogger
    ) -> None:
        captured: dict[str, Any] = {}

        class _Capture(_FakeSkill):
            def __init__(self, **kwargs: Any) -> None:
                captured.update(kwargs)
                super().__init__(**kwargs)

        build_skill_tools(
            [_spec(skill_class=_Capture)],
            adapter=object(), model=object(),
            prompt_loader=lambda _: "raw",
            logger=logger,
        )
        assert captured["prompt"] == "raw"


class TestEdgeCases:
    def test_empty_specs_returns_empty_list(
        self, logger: _FakeLogger
    ) -> None:
        tools = build_skill_tools(
            [], adapter=object(), model=object(),
            prompt_loader=lambda _: "", logger=logger,
        )
        assert tools == []

    def test_adapter_and_model_forwarded_to_skill(
        self, logger: _FakeLogger
    ) -> None:
        sentinel_adapter = object()
        sentinel_model = object()
        captured: dict[str, Any] = {}

        class _Capture(_FakeSkill):
            def __init__(self, **kwargs: Any) -> None:
                captured.update(kwargs)
                super().__init__(**kwargs)

        build_skill_tools(
            [_spec(skill_class=_Capture)],
            adapter=sentinel_adapter,
            model=sentinel_model,
            prompt_loader=lambda _: "p",
            logger=logger,
        )
        assert captured["adapter"] is sentinel_adapter
        assert captured["model"] is sentinel_model

    def test_factory_kwargs_forwarded(
        self, logger: _FakeLogger
    ) -> None:
        captured: dict[str, Any] = {}

        class _Capture(_FakeSkill):
            def __init__(self, **kwargs: Any) -> None:
                captured.update(kwargs)
                super().__init__(**kwargs)

        build_skill_tools(
            [_spec(skill_class=_Capture, factory_kwargs={"temperature": 0.85})],
            adapter=object(), model=object(),
            prompt_loader=lambda _: "p",
            logger=logger,
        )
        # factory_kwargs are flattened into the kwargs dict before reaching the skill,
        # so the test verifies they're present at the top level.
        assert captured["temperature"] == 0.85

    def test_info_label_override(
        self, logger: _FakeLogger
    ) -> None:
        build_skill_tools(
            [_spec("MusicSkill", info_label="🎵 MusicSkill")],
            adapter=object(), model=object(),
            prompt_loader=lambda _: "p",
            logger=logger,
        )
        assert any(
            level == "info" and "🎵 MusicSkill" in msg
            for level, msg in logger.records
        )