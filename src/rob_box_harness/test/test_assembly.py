"""Tests for ``core.assembly`` (ADR-0083): :class:`AgentSpec`, :func:`build_llm_chain`.

``build_agent`` и цикл v1 ``AgentCore`` удалены (#4354 A3b): диалог v2 берёт только цепочку LLM. Загрузчик промпта
из файла (``load_system_prompt``, поля ``prompt_dir`` / ``system_prompt_file``) удалён в ADR-0167 Ш1в: промпт собирает
хост.
"""

from __future__ import annotations

import dataclasses

import pytest

from rob_box_harness.core import assembly
from rob_box_harness.core.assembly import AgentSpec
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


# ---------------------------------------------------------------------------
# AgentSpec dataclass contract
# ---------------------------------------------------------------------------


def test_agent_spec_is_frozen() -> None:
    """``AgentSpec`` is a frozen dataclass — runtime mutation is forbidden."""
    spec = AgentSpec(name="personality")
    with pytest.raises(dataclasses.FrozenInstanceError):
        spec.name = "operator"  # type: ignore[misc]


def test_agent_spec_field_count_is_pinned() -> None:
    """Pin the field count so accidental additions are caught by CI.

    The ADR-0083 §2.1 spec listed 18 fields; ADR-0150 PR-6 removed
    ``skill_slice`` and ``narrow_tools_to_skill`` (skills are gone); ADR-0150 §4.5
    (#3696 ч.5) added ``wrap_provider`` (command mode per provider); PR-7b (#3699)
    removed the dead ``use_scheduler`` flag; #4354 A3b removed the fields only ``build_agent`` read
    (``memory_namespace``, ``on_prompt``, ``use_streaming``, ``history_trim_limit``, ``user_id``, ``dsm``);
    ADR-0167 Ш1в removed ``prompt_dir`` and ``system_prompt_file`` (the host composes the prompt).
    A future ADR must accompany any new field.
    """
    assert len(AgentSpec(name="personality").__dataclass_fields__) == 8


def test_agent_spec_required_fields_have_no_default() -> None:
    """``name`` is mandatory — it describes *which* agent we are."""
    fields = AgentSpec.__dataclass_fields__
    assert fields["name"].default is dataclasses.MISSING


def test_prompt_file_loader_is_gone() -> None:
    """ADR-0167 Ш1в: промпт собирает хост (основа ядра + персона профиля); загрузчика из файла и его полей нет -
    вторая реализация не держится («не держать две реализации»)."""
    assert not hasattr(assembly, "load_system_prompt")
    assert {"prompt_dir", "system_prompt_file"}.isdisjoint(AgentSpec.__dataclass_fields__)


@pytest.mark.asyncio
async def test_build_llm_chain_returns_provider() -> None:
    """``build_llm_chain`` — цепочка LLM хода v2 (``dialog_v2_ports``, ТАРС ``operator_turns``)."""
    from rob_box_harness.core.assembly import build_llm_chain

    spec = AgentSpec(name="personality", provider_chain=("deepseek",))
    llm = build_llm_chain(spec)
    # Single-provider path returns the provider as-is (no wrapper).
    # ``deepseek`` always builds even without an API key (registry
    # doesn't gate on env at construction time — that's a runtime
    # concern), so we just check the object exists and isn't None.
    assert llm is not None


def test_build_llm_chain_wraps_every_built_provider(monkeypatch: pytest.MonkeyPatch) -> None:
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

    spec = AgentSpec(name="personality", provider_chain=("deepseek", "minimax"), wrap_provider=_wrap)
    llm = build_llm_chain(spec)
    assert isinstance(llm, HealthAwareFallbackLLM)
    assert built == wrapped == ["deepseek", "minimax"]


def test_agent_spec_wrap_provider_defaults_to_identity() -> None:
    spec = AgentSpec(name="personality")
    sentinel = object()
    assert spec.wrap_provider(sentinel) is sentinel
