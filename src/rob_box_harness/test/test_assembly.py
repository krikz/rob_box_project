"""Tests for ``core.assembly`` (ADR-0083): :class:`AgentSpec`, :func:`load_system_prompt`, :func:`build_llm_chain`.

``build_agent`` и цикл v1 ``AgentCore`` удалены (#4354 A3b): диалог v2 берёт только цепочку LLM и промпт.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import pytest

from rob_box_harness.core.assembly import (
    AgentSpec,
    load_system_prompt,
)
from rob_box_harness.providers import DummyLLMProvider


# ---------------------------------------------------------------------------
# Stub the LLM provider builder — the real ``build_provider`` needs API keys
# (DEEPSEEK_API_KEY / MINIMAX_API_KEY) and would crash in CI without secrets.
# Tests are about AgentSpec / build_agent wiring, not about a real LLM.
# ---------------------------------------------------------------------------


@pytest.fixture
def stub_llm_builder(monkeypatch: pytest.MonkeyPatch) -> DummyLLMProvider:
    """Replace ``build_provider`` with a deterministic DummyLLMProvider.

    The replacement ignores the provider name and base URL — that's the
    whole point of a stub. Returns the single provider instance so tests
    can inspect ``call_count`` if they need to assert "the LLM was
    wired through".

    Autouse so every test in this file gets a stubbed LLM without
    threading the fixture through every signature.
    """
    provider = DummyLLMProvider()

    def _fake(name: str, **_kwargs: object) -> DummyLLMProvider:
        # ``name`` is preserved on the call so a test can assert
        # ``provider.last_messages`` if needed; we don't echo it back.
        return provider

    monkeypatch.setattr(
        "rob_box_harness.core.assembly.build_provider", _fake
    )
    return provider


@pytest.fixture(autouse=True)
def _stub_llm_builder_auto(stub_llm_builder: DummyLLMProvider) -> DummyLLMProvider:
    """Autouse wrapper: every assembly test gets a stubbed LLM provider.

    The real ``build_provider`` needs API keys (DEEPSEEK_API_KEY /
    MINIMAX_API_KEY) and would crash in CI without secrets. Tests here
    are about AgentSpec / build_agent wiring, not a real LLM.
    """
    return stub_llm_builder


@pytest.fixture
def tmp_prompt_dirs(tmp_path: Path) -> tuple[Path, Path]:
    """Create two prompt directories: personality + operator.

    Each has its own ``<system_prompt_file>``.
    """
    personality_dir = tmp_path / "personality_prompts"
    operator_dir = tmp_path / "operator_prompts"
    for d in (personality_dir, operator_dir):
        d.mkdir(parents=True)
    (personality_dir / "master.txt").write_text(
        "Ты — личность робота. Добрый, тёплый.", encoding="utf-8"
    )
    (operator_dir / "operator.txt").write_text(
        "Ты — ТАРС. Деловой, по делу.", encoding="utf-8"
    )
    return personality_dir, operator_dir


# ---------------------------------------------------------------------------
# AgentSpec dataclass contract
# ---------------------------------------------------------------------------


def test_agent_spec_is_frozen(tmp_prompt_dirs: tuple[Path, Path]) -> None:
    """``AgentSpec`` is a frozen dataclass — runtime mutation is forbidden."""
    personality_dir, operator_dir = tmp_prompt_dirs
    spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
    )
    with pytest.raises(dataclasses.FrozenInstanceError):
        spec.name = "operator"  # type: ignore[misc]


def test_agent_spec_field_count_is_pinned(tmp_prompt_dirs: tuple[Path, Path]) -> None:
    """Pin the field count so accidental additions are caught by CI.

    The ADR-0083 §2.1 spec listed 18 fields; ADR-0150 PR-6 removed
    ``skill_slice`` and ``narrow_tools_to_skill`` (skills are gone); ADR-0150 §4.5
    (#3696 ч.5) added ``wrap_provider`` (command mode per provider); PR-7b (#3699)
    removed the dead ``use_scheduler`` flag; #4354 A3b removed the fields only ``build_agent`` read
    (``memory_namespace``, ``on_prompt``, ``use_streaming``, ``history_trim_limit``, ``user_id``, ``dsm``).
    A future ADR must accompany any new field.
    """
    personality_dir, _ = tmp_prompt_dirs
    spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
    )
    assert len(spec.__dataclass_fields__) == 10


def test_agent_spec_required_fields_have_no_default(
    tmp_prompt_dirs: tuple[Path, Path],
) -> None:
    """``name`` is mandatory — it describes *which* agent we are. The prompt
    file (``prompt_dir`` / ``system_prompt_file``) is optional since ADR-0167
    Ш1б-3: the personality prompt is composed by the host (core base + the
    product persona), only file-based agents set it; without it
    :func:`load_system_prompt` returns ``""`` (no prompt — no agent)."""
    personality_dir, _ = tmp_prompt_dirs
    fields = AgentSpec.__dataclass_fields__
    assert "name" in fields
    assert fields["name"].default is dataclasses.MISSING
    assert fields["prompt_dir"].default is None
    assert fields["system_prompt_file"].default == ""
    assert load_system_prompt(AgentSpec(name="personality")) == ""


# ---------------------------------------------------------------------------
# Prompt loader
# ---------------------------------------------------------------------------


def test_load_system_prompt_reads_correct_file(
    tmp_prompt_dirs: tuple[Path, Path],
) -> None:
    personality_dir, operator_dir = tmp_prompt_dirs
    p_spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
    )
    o_spec = AgentSpec(
        name="operator",
        prompt_dir=operator_dir,
        system_prompt_file="operator.txt",
    )
    assert "личность робота" in load_system_prompt(p_spec)
    assert "ТАРС" in load_system_prompt(o_spec)


def test_load_system_prompt_missing_dir_returns_empty(
    tmp_path: Path,
) -> None:
    """Missing prompt_dir → empty string (best-effort, like the legacy
    behaviour of ``dialogue_node._load_system_prompt``)."""
    spec = AgentSpec(
        name="personality",
        prompt_dir=tmp_path / "nonexistent",
        system_prompt_file="master.txt",
    )
    assert load_system_prompt(spec) == ""


@pytest.mark.asyncio
async def test_build_llm_chain_returns_provider(
    tmp_prompt_dirs: tuple[Path, Path],
) -> None:
    """``build_llm_chain`` — цепочка LLM хода v2 (``dialog_v2_ports``, ТАРС ``operator_turns``)."""
    from rob_box_harness.core.assembly import build_llm_chain

    personality_dir, _ = tmp_prompt_dirs
    spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
        provider_chain=("deepseek",),
    )
    llm = build_llm_chain(spec)
    # Single-provider path returns the provider as-is (no wrapper).
    # ``deepseek`` always builds even without an API key (registry
    # doesn't gate on env at construction time — that's a runtime
    # concern), so we just check the object exists and isn't None.
    assert llm is not None


def test_build_llm_chain_wraps_every_built_provider(
    tmp_prompt_dirs: tuple[Path, Path], monkeypatch: pytest.MonkeyPatch,
) -> None:
    """ADR-0150 §4.5 (#3696 ч.5): ``wrap_provider`` оборачивает КАЖДЫЙ провайдер цепочки до фолбек-обёртки,
    поэтому форму запроса выбирают возможности того провайдера, который отвечает, а не primary."""
    from rob_box_harness.core.assembly import build_llm_chain
    from rob_box_harness.health import HealthAwareFallbackLLM

    built: list[str] = []

    def _named(name: str, **_kwargs: object) -> DummyLLMProvider:
        provider = DummyLLMProvider()
        provider.name = name
        built.append(name)
        return provider

    monkeypatch.setattr("rob_box_harness.core.assembly.build_provider", _named)
    wrapped: list[str] = []

    def _wrap(provider: DummyLLMProvider) -> DummyLLMProvider:
        wrapped.append(provider.name)
        return provider

    personality_dir, _ = tmp_prompt_dirs
    spec = AgentSpec(
        name="personality", prompt_dir=personality_dir, system_prompt_file="master.txt",
        provider_chain=("deepseek", "minimax"), wrap_provider=_wrap,
    )
    llm = build_llm_chain(spec)
    assert isinstance(llm, HealthAwareFallbackLLM)
    assert built == wrapped == ["deepseek", "minimax"]


def test_agent_spec_wrap_provider_defaults_to_identity(tmp_prompt_dirs: tuple[Path, Path]) -> None:
    personality_dir, _ = tmp_prompt_dirs
    spec = AgentSpec(name="personality", prompt_dir=personality_dir, system_prompt_file="master.txt")
    sentinel = object()
    assert spec.wrap_provider(sentinel) is sentinel
