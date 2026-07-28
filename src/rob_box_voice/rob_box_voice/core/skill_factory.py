"""``skill_factory`` — pull the repetitive skill-construction boilerplate out of ``DialogueNode``.

ADR-0001 §2.7.3 (D8) calls for a single ``SkillBase`` with a uniform
construction path. The Compositor's :py:meth:`DialogueNode._build_skills`
currently instantiates 4–5 skills (``MusicSkill``, ``NavigationSkill``,
``MemorySkill``, ``StatusSkill``, ``FAQSkill``) with five nearly-identical
``try / except / as_tool`` blocks. The repetition is the smell — not the
classes themselves (each skill still has its own ``_make_tools()``).

This module keeps the existing skill classes intact (they own real
``@function_tool`` definitions that the Compositor cannot replace) and
collapses only the construction boilerplate into one loop.

**Public API:**

* :class:`SkillFactorySpec` — declarative spec for one skill.
* :func:`build_skill_tools` — given a list of specs, an MCP adapter and
  a shared LLM model, instantiate each skill (or skip on failure) and
  return its ``FunctionTool`` wrapper.

**Backwards-compat:** the original ``_build_skills`` method is rewritten
as a 1-liner that calls ``build_skill_tools``. ROS2 topics and behavior
are unchanged — only the source-of-truth for "what skills exist" moves
out of the god-object.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Optional, Protocol


class _LoggerLike(Protocol):
    """Subset of ``rclpy.node.Node.get_logger()`` we depend on."""

    def info(self, msg: str) -> None: ...
    def warning(self, msg: str) -> None: ...
    def error(self, msg: str) -> None: ...


@dataclass(frozen=True)
class SkillFactorySpec:
    """Declarative description of one skill in the Compositor's tool belt.

    Attributes:
        display_name:   Human-readable name used in log lines ("MusicSkill").
        skill_class:    Constructor to call — usually one of the BaseSkill
                        subclasses (MusicSkill, NavigationSkill, …).
        prompt_file:    Relative path under ``prompts/``, e.g.
                        ``"skills/music_skill_prompt.txt"``. Tried first.
        fallback_prompt: Used when ``prompt_file`` cannot be loaded.
        tool_name:      Name the Compositor will use to invoke the skill
                        (passed to ``function_tool(..., name_override=...)``).
        tool_description: One-line description shown to the Compositor LLM.
        factory_kwargs: Extra kwargs forwarded to ``skill_class(...)``.
                        ``adapter`` and ``model`` are injected automatically;
                        supply everything else here. The prompt is injected
                        under the key given by ``prompt_kwarg``.
        prompt_kwarg:   Name of the kwarg that carries the prompt text.
                        ``"prompt"`` for Navigation/Memory/Status/FAQ,
                        ``"prompt_template"`` for MusicSkill (which does
                        an extra ``{renardo_ref}`` substitution).
        prompt_render:  Optional callable taking the loaded prompt text
                        and returning a (possibly transformed) prompt.
                        Used by FAQSkill to prepend the active event
                        context (``render_faq_skill_prompt``).
                        Default: identity (return text unchanged).
        info_label:     Short label for the success log line
                        (default: ``display_name``).
    """

    display_name: str
    skill_class: Callable[..., Any]
    prompt_file: str
    fallback_prompt: str
    tool_name: str
    tool_description: str
    factory_kwargs: dict[str, Any] = field(default_factory=dict)
    prompt_kwarg: str = "prompt"
    prompt_render: Optional[Callable[[str], str]] = None
    info_label: Optional[str] = None

    def resolved_info_label(self) -> str:
        return self.info_label or self.display_name


def _load_prompt(
    spec: SkillFactorySpec,
    *,
    prompt_loader: Callable[[str], str],
    logger: _LoggerLike,
) -> str:
    """Return the prompt text for this spec, falling back on failure.

    The loader is injected (rather than called directly) so the factory
    can be unit-tested with a fake prompt loader that doesn't touch the
    filesystem.
    """
    prompt = prompt_loader(spec.prompt_file)
    if not prompt.strip():
        logger.warning(
            f"⚠️ Skill '{spec.display_name}': using fallback prompt "
            f"(file '{spec.prompt_file}' was empty)"
        )
        return spec.fallback_prompt
    return prompt


def build_skill_tools(
    specs: list[SkillFactorySpec],
    *,
    adapter: Any,
    model: Any,
    prompt_loader: Callable[[str], str],
    logger: _LoggerLike,
) -> list[Any]:
    """Instantiate each spec's skill and return the ``FunctionTool`` wrappers.

    Each spec is wrapped in its own ``try / except`` — a single broken
    skill does NOT abort the whole Compositor. Failed skills are logged
    and skipped, matching the legacy ``_build_skills`` behaviour exactly.

    Args:
        specs: Ordered list of skill specs. Order = tool order in the
               Compositor's tool list.
        adapter: Shared LLMToolCallAdapter (self._mcp in the legacy code).
        model:   Shared OpenAIChatCompletionsModel instance.
        prompt_loader: Callable taking a relative prompt-file path and
                       returning its contents (or empty string on failure).
                       Injected so tests can skip filesystem I/O.
        logger:  Logger-like object (ROS2 ``get_logger()`` works).

    Returns:
        List of ``FunctionTool`` objects, one per spec that loaded
        successfully. Order matches ``specs``.
    """
    tools: list[Any] = []
    for spec in specs:
        raw_prompt = _load_prompt(spec, prompt_loader=prompt_loader, logger=logger)
        prompt = spec.prompt_render(raw_prompt) if spec.prompt_render else raw_prompt
        try:
            skill = spec.skill_class(
                adapter=adapter,
                model=model,
                **{spec.prompt_kwarg: prompt},
                name=spec.display_name,
                **spec.factory_kwargs,
            )
            tool = skill.as_tool(
                tool_name=spec.tool_name,
                tool_description=spec.tool_description,
            )
            tools.append(tool)
            logger.info(f"✅ {spec.resolved_info_label()} loaded")
        except Exception as exc:  # noqa: BLE001 — log + skip, like legacy
            logger.error(
                f"❌ {spec.resolved_info_label()} build failed: "
                f"{type(exc).__name__}: {exc}"
            )
    return tools


__all__ = ["SkillFactorySpec", "build_skill_tools"]