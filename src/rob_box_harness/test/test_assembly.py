"""Tests for ``build_agent`` (ADR-0083).

The contract under test:

* :class:`rob_box_harness.core.assembly.AgentSpec` — frozen dataclass
  with 18 fields covering identity (always-different), LLM (partly shared)
  and common (always-shared).
* :func:`build_agent` — single public assembler for ``AgentCore``;
  the only place outside ``core/agent_core.py`` and the test fixtures
  where ``AgentCore(`` may appear in ``src/`` (ADR-0083 §2.1).
* Two specs (personality + operator) with disjoint ``prompt_dir`` /
  ``skill_slice`` / ``memory_namespace`` must yield two distinct
  ``AgentCore`` instances with their prompts, skill slices and memory
  namespaces intact (ADR-0083 §1.4 DoD bullet #3).
* ``SQLiteVoiceMemory(..., agent=...)`` keeps namespaces disjoint
  inside a single DB file (ADR-0083 §2.4 / §1.4 DoD bullet #2).
"""

from __future__ import annotations

import asyncio
import dataclasses
import tempfile
from collections.abc import Iterator
from pathlib import Path
from typing import TYPE_CHECKING

import pytest

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.assembly import (
    AgentSpec,
    build_agent,
    load_skill_prompts,
    load_system_prompt,
    normalize_skill_slice,
)
from rob_box_harness.core.dialogue_state_machine import DialogueStateMachine
from rob_box_harness.memory import Fact, SQLiteVoiceMemory
from rob_box_harness.providers import DummyLLMProvider
from rob_box_harness.tools import FakeToolProvider

if TYPE_CHECKING:
    from rob_box_harness.tools import ToolSpec


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
# Fakes (mirrors test_agent_core.py / test_sqlite_voice_memory.py patterns)
# ---------------------------------------------------------------------------


def _make_tools() -> FakeToolProvider:
    """Test stub tool provider — implements the full ``ToolProvider`` ABC.

    We could subclass the existing ``FakeToolProvider`` and add a single
    ``noop`` tool, but the framework default is already empty + harmless.
    """
    return FakeToolProvider()


@pytest.fixture
def tmp_prompt_dirs(tmp_path: Path) -> tuple[Path, Path]:
    """Create two prompt directories: personality + operator.

    Each has its own ``<system_prompt_file>`` and a ``skills/``
    subdirectory with one fragment.
    """
    personality_dir = tmp_path / "personality_prompts"
    operator_dir = tmp_path / "operator_prompts"
    for d in (personality_dir, operator_dir):
        (d / "skills").mkdir(parents=True)
    (personality_dir / "master.txt").write_text(
        "Ты — личность робота. Добрый, тёплый.", encoding="utf-8"
    )
    (personality_dir / "skills" / "player.txt").write_text(
        "Инструкции по скиллу 'player'.", encoding="utf-8"
    )
    (operator_dir / "operator.txt").write_text(
        "Ты — ТАРС. Деловой, по делу.", encoding="utf-8"
    )
    (operator_dir / "skills" / "operator.speech.txt").write_text(
        "Фрагмент среза 'operator.speech'.", encoding="utf-8"
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


def test_agent_spec_field_count_is_18(tmp_prompt_dirs: tuple[Path, Path]) -> None:
    """Pin the field count so accidental additions are caught by CI.

    The ADR-0083 §2.1 spec lists exactly 18 fields; a future ADR must
    accompany any new field.
    """
    personality_dir, _ = tmp_prompt_dirs
    spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
    )
    assert len(spec.__dataclass_fields__) == 18


def test_agent_spec_required_fields_have_no_default(
    tmp_prompt_dirs: tuple[Path, Path],
) -> None:
    """``name`` / ``prompt_dir`` / ``system_prompt_file`` are mandatory —
    they describe *which* agent we are. Everything else is optional and
    can fall back to a sensible default."""
    personality_dir, _ = tmp_prompt_dirs
    fields = AgentSpec.__dataclass_fields__
    assert "name" in fields
    assert fields["name"].default is dataclasses.MISSING
    assert "prompt_dir" in fields
    assert fields["prompt_dir"].default is dataclasses.MISSING
    assert "system_prompt_file" in fields
    assert fields["system_prompt_file"].default is dataclasses.MISSING


def test_normalize_skill_slice_handles_list_and_none() -> None:
    """Frozen dataclass requires tuple, but YAML gives lists — normalise."""
    assert normalize_skill_slice(None) == ()
    assert normalize_skill_slice([]) == ()
    assert normalize_skill_slice(("a", "b")) == ("a", "b")
    assert normalize_skill_slice(["a", "b"]) == ("a", "b")


# ---------------------------------------------------------------------------
# Prompt / skill loaders
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


def test_load_skill_prompts_respects_disjoint_slice(
    tmp_prompt_dirs: tuple[Path, Path],
) -> None:
    """Personality loads only ``player``; operator loads only
    ``operator.speech`` — never the other side's fragments
    (ADR-0083 §1.3 #3)."""
    personality_dir, operator_dir = tmp_prompt_dirs
    p_spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
        skill_slice=("player",),
    )
    o_spec = AgentSpec(
        name="operator",
        prompt_dir=operator_dir,
        system_prompt_file="operator.txt",
        skill_slice=("operator.speech",),
    )
    p_skills = load_skill_prompts(p_spec)
    o_skills = load_skill_prompts(o_spec)
    assert "player" in p_skills
    assert "operator.speech" not in p_skills
    assert "operator.speech" in o_skills
    assert "player" not in o_skills


def test_load_skill_prompts_absent_fragment_is_warning(
    tmp_prompt_dirs: tuple[Path, Path],
) -> None:
    """A missing fragment is a warning, not an error — the slice can
    keep growing without breaking existing builds."""
    personality_dir, _ = tmp_prompt_dirs
    spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
        skill_slice=("player", "missing_skill"),
    )
    loaded = load_skill_prompts(spec)
    assert "player" in loaded
    assert "missing_skill" not in loaded


# ---------------------------------------------------------------------------
# build_agent — single assembler contract (ADR-0083 §2.1, DoD bullet #1, #3)
# ---------------------------------------------------------------------------


@pytest.fixture
def sqlite_db_path(tmp_path: Path) -> Iterator[Path]:
    """Fresh SQLite DB for namespace tests."""
    db = tmp_path / "harness_voice.db"
    yield db
    # Cleanup — tempfile fixture removes the dir.


async def _make_memory(db_path: Path, agent: str) -> SQLiteVoiceMemory:
    """Construct a SQLiteVoiceMemory bound to ``agent`` namespace.

    Mirrors the supervisor/dialogue_node construction flow:
    ``SQLiteVoiceMemory(db_path=...)`` + ``await store.init()``.
    """
    store = SQLiteVoiceMemory(db_path=str(db_path), agent=agent)
    await store.init()
    return store


@pytest.mark.asyncio
async def test_build_agent_returns_agent_core(
    tmp_prompt_dirs: tuple[Path, Path],
    sqlite_db_path: Path,
) -> None:
    """Smoke: ``build_agent`` constructs an ``AgentCore`` with all four
    ports wired."""
    personality_dir, _ = tmp_prompt_dirs
    spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
    )
    memory = await _make_memory(sqlite_db_path, agent="personality")
    tools = _make_tools()
    core = build_agent(spec, tools=tools, memory=memory)
    assert isinstance(core, AgentCore)


@pytest.mark.asyncio
async def test_build_agent_passes_system_prompt_to_core(
    tmp_prompt_dirs: tuple[Path, Path],
    sqlite_db_path: Path,
) -> None:
    """The loaded system prompt reaches ``AgentCore._system_prompt``.

    Regression target — the legacy ``dialogue_node`` used to load the
    system prompt but lost the reference before passing it to
    ``AgentCore`` (issue #992 fix). The assembly bench must preserve
    that wiring."""
    personality_dir, _ = tmp_prompt_dirs
    spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
    )
    memory = await _make_memory(sqlite_db_path, agent="personality")
    core = build_agent(spec, tools=_make_tools(), memory=memory)
    assert "личность робота" in core._system_prompt  # type: ignore[attr-defined]


@pytest.mark.asyncio
async def test_build_agent_keeps_narrow_tools_to_skill(
    tmp_prompt_dirs: tuple[Path, Path],
    sqlite_db_path: Path,
) -> None:
    """The ``narrow_tools_to_skill`` flag from the spec is forwarded to
    ``AgentCore`` verbatim (ADR-0083 §2.2 #J — operator used to
    hardcode ``False``)."""
    personality_dir, _ = tmp_prompt_dirs
    spec = AgentSpec(
        name="operator",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
        narrow_tools_to_skill=False,
    )
    memory = await _make_memory(sqlite_db_path, agent="operator")
    core = build_agent(spec, tools=_make_tools(), memory=memory)
    assert core._narrow_tools_to_skill is False  # type: ignore[attr-defined]


@pytest.mark.asyncio
async def test_build_agent_two_specs_two_cores_disjoint(
    tmp_prompt_dirs: tuple[Path, Path],
    sqlite_db_path: Path,
) -> None:
    """ADR-0083 §1.4 DoD bullet #3 — ``build_agent`` for two specs gives
    two cores with different prompts, different skill slices and
    different memory namespaces.

    This is THE acceptance test for the contract."""
    personality_dir, operator_dir = tmp_prompt_dirs

    p_spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
        skill_slice=("player",),
        narrow_tools_to_skill=True,
    )
    o_spec = AgentSpec(
        name="operator",
        prompt_dir=operator_dir,
        system_prompt_file="operator.txt",
        skill_slice=("operator.speech",),
        narrow_tools_to_skill=False,
    )

    # Two distinct memory handles — both pointing at the same SQLite
    # file but with disjoint ``agent`` namespaces (ADR-0083 §2.4).
    p_memory = await _make_memory(sqlite_db_path, agent="personality")
    o_memory = await _make_memory(sqlite_db_path, agent="operator")

    p_core = build_agent(p_spec, tools=_make_tools(), memory=p_memory)
    o_core = build_agent(o_spec, tools=_make_tools(), memory=o_memory)

    # Distinct core instances.
    assert p_core is not o_core

    # Different system prompts.
    assert "личность робота" in p_core._system_prompt  # type: ignore[attr-defined]
    assert "ТАРС" in o_core._system_prompt  # type: ignore[attr-defined]
    assert "ТАРС" not in p_core._system_prompt  # type: ignore[attr-defined]
    assert "личность робота" not in o_core._system_prompt  # type: ignore[attr-defined]

    # Different skill slices (forwarded to AgentCore via skill_prompts).
    assert "player" in p_core._skill_prompts  # type: ignore[attr-defined]
    assert "operator.speech" not in p_core._skill_prompts  # type: ignore[attr-defined]
    assert "operator.speech" in o_core._skill_prompts  # type: ignore[attr-defined]
    assert "player" not in o_core._skill_prompts  # type: ignore[attr-defined]

    # Different ``narrow_tools_to_skill`` values — explicit, not hardcoded.
    assert p_core._narrow_tools_to_skill is True  # type: ignore[attr-defined]
    assert o_core._narrow_tools_to_skill is False  # type: ignore[attr-defined]

    # Disjoint memory namespaces in a single SQLite file.
    await p_memory.save_fact(
        "user:1", Fact(key="hobby", value="hockey"), agent="personality"
    )
    await o_memory.save_fact(
        "user:1", Fact(key="hobby", value="юбисочные"), agent="operator"
    )

    p_facts = await p_memory.search_facts(
        "user:1", "hobby", top_k=5, agent="personality"
    )
    o_facts = await o_memory.search_facts(
        "user:1", "hobby", top_k=5, agent="operator"
    )
    assert len(p_facts) == 1
    assert p_facts[0].value == "hockey"
    assert len(o_facts) == 1
    assert o_facts[0].value == "юбисочные"


@pytest.mark.asyncio
async def test_build_agent_rejects_empty_provider_chain(
    tmp_prompt_dirs: tuple[Path, Path],
    sqlite_db_path: Path,
) -> None:
    """``provider_chain=()`` must fail loudly — silent fallback to
    a default chain is exactly the class of bug ADR-0083 closes."""
    personality_dir, _ = tmp_prompt_dirs
    spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
        provider_chain=(),
    )
    memory = await _make_memory(sqlite_db_path, agent="personality")
    with pytest.raises(RuntimeError, match="provider_chain"):
        build_agent(spec, tools=_make_tools(), memory=memory)


@pytest.mark.asyncio
async def test_build_agent_respects_user_id_and_dsm(
    tmp_prompt_dirs: tuple[Path, Path],
    sqlite_db_path: Path,
) -> None:
    """``user_id`` / ``dsm`` are forwarded verbatim to AgentCore
    (ADR-0083 §2.2 #K, #L)."""
    personality_dir, _ = tmp_prompt_dirs
    dsm = DialogueStateMachine()
    spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
        user_id="session-uuid-42",
        dsm=dsm,
    )
    memory = await _make_memory(sqlite_db_path, agent="personality")
    core = build_agent(spec, tools=_make_tools(), memory=memory)
    assert core._user_id == "session-uuid-42"  # type: ignore[attr-defined]
    assert core._dsm is dsm  # type: ignore[attr-defined]


@pytest.mark.asyncio
async def test_build_llm_chain_returns_provider(
    tmp_prompt_dirs: tuple[Path, Path],
) -> None:
    """``build_llm_chain`` публичный API для сборки LLM ДО ``build_agent``.

    Нужен нодам, которые держат ссылку на LLM для метрик
    ``record_voice_llm_request`` / OTel span ``dialogue.llm_call``
    (issue #1160, ADR-0083 §2.3). Возвращённый LLM — тот же, что
    потом использует AgentCore (single provider path).
    """
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


@pytest.mark.asyncio
async def test_build_agent_accepts_prebuilt_llm(
    tmp_prompt_dirs: tuple[Path, Path],
    sqlite_db_path: Path,
) -> None:
    """``build_agent(..., llm=...)`` использует переданный LLM вместо
    внутренней сборки.

    Контракт (ADR-0083 §2.3): если нода уже собрала LLM
    (``self._llm`` для метрик), она передаёт его явно, чтобы
    не дублировать ``build_provider()``-цепочку. Проверяем, что
    AgentCore использует тот же объект (``is`` identity, а не
    ``==``).
    """
    from rob_box_harness.core.assembly import build_llm_chain

    personality_dir, _ = tmp_prompt_dirs
    spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
        provider_chain=("deepseek",),
    )
    prebuilt = build_llm_chain(spec)
    memory = await _make_memory(sqlite_db_path, agent="personality")
    core = build_agent(
        spec, tools=_make_tools(), memory=memory, llm=prebuilt
    )
    # Identity check: core must hold the exact LLM we passed in,
    # not a freshly built one. This is the contract that prevents
    # double build (and double env-key resolution) when the node
    # wires metrics alongside the core.
    assert core._llm is prebuilt  # type: ignore[attr-defined]


@pytest.mark.asyncio
async def test_build_agent_without_llm_keeps_backward_compatible_path(
    tmp_prompt_dirs: tuple[Path, Path],
    sqlite_db_path: Path,
) -> None:
    """``build_agent(spec, tools=..., memory=...)`` без ``llm=`` собирает
    LLM внутри — обратная совместимость для тестов и supervisor'а
    (ADR-0083 §2.3, §2.1).

    Default branch must keep working: ``llm=None`` (явно или
    неявно) → внутренний ``_build_llm_chain`` создаёт provider-chain
    и core держит свежесобранный LLM.
    """
    personality_dir, _ = tmp_prompt_dirs
    spec = AgentSpec(
        name="personality",
        prompt_dir=personality_dir,
        system_prompt_file="master.txt",
        provider_chain=("deepseek",),
    )
    memory = await _make_memory(sqlite_db_path, agent="personality")
    core = build_agent(spec, tools=_make_tools(), memory=memory)
    assert core._llm is not None  # type: ignore[attr-defined]