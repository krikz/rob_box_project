# Phase 6: Harness P0 Finalization — Research

**Researched:** 2026-07-27
**Domain:** Harness framework finalization — documentation merge, Docker integration, concrete harness adapters (Dialog/Persistent/Telegram), port implementations (ROS2Transport/SQLiteVoiceMemory), test coverage gap closure, PR audit
**Confidence:** HIGH

## Summary

Phase 6 finalizes the `feature/harness-p0-foundation` branch (PR #907) for merge readiness. The P0 harness framework (`rob_box_harness` — 88 tests, 90%+ coverage, mypy strict-clean) is already built. What remains is documentation consolidation (W1–W5), Docker integration (W6–W7), three concrete harness adapters wrapping existing ROS2 nodes (W8–W11), two port implementations (W12–W13), test coverage gap closure (W14–W17), and final PR audit + quality gates (W18–W22).

The phase consists of 22 atomic waves grouped into 8 dependency chains. All canonical ADRs (0001–0009) are accepted and match the current code. The phase is greenfield execution on top of a solid P0 foundation — no architectural ambiguity.

**Primary recommendation:** Execute waves in dependency order (A→B→C→D→E→F→G→H), parallelize within documentation group (W1–W5), build ports (W12–W13) before or alongside harness adapters (W8–W11), and keep quality gates (W21–W22) as the final pre-merge checkpoint.

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| Documentation merge (W1–W4) | docs/adr/ | docs/architecture/ | ADR directory is canonical source; architecture/ holds cross-refs only |
| SPEC_CURRENT.md update (W5) | Repository root | — | Single source of truth for P0/P1 state |
| Docker harness integration (W6–W7) | docker/vision/voice_assistant/ | docker/vision/voice_base/ | voice_assistant is the deployment target; voice_base is build dependency |
| DialogHarness adapter (W8–W9) | src/rob_box_harness/ | src/rob_box_voice/ | Harness wraps dialogue_node; thin ROS2 wrapper stays in rob_box_voice |
| PersistentHarness (W10) | src/rob_box_harness/ | src/rob_box_voice/ | Unified hardware lifecycle across 6 persistent nodes |
| TelegramHarness (W11) | src/rob_box_harness/ | src/rob_box_telegram/ | Harness wraps telegram_node; thin ROS2 wrapper stays in rob_box_telegram |
| ROS2Transport port (W12) | src/rob_box_harness/ | — | Transport port implementation; ROS2-specific |
| SQLiteVoiceMemory port (W13) | src/rob_box_harness/ | src/rob_box_voice/ | MemoryStore implementation; may reuse existing VoiceMemory SQLite schema |
| Test coverage (W14–W17) | src/rob_box_harness/test/ | tests/unit/harness/ | Tests exercise harness ports with fakes; integration tests validate real nodes |
| PR audit + linting (W18–W22) | Repository root | src/rob_box_harness/ | Quality gates before merge |

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

- **D-01: Documentation strategy** — All architecture docs → `docs/adr/` as canonical source; `docs/architecture/` holds overview docs with cross-references only (NO duplicates). Merge patterns specified for each pair.
- **D-02: ADR compliance verified** — ADR-0001 M1–M10 all implemented in `rob_box_harness/providers/minimax.py`. No discrepancies found.
- **D-03: Two MiniMax providers are NOT duplicates** — `rob_box_llm/providers/minimax.py` (upstream) + `rob_box_harness/providers/minimax.py` (harness wrapper) — intentional architecture.
- **D-04/D-06: Docker integration** — Add `rob_box_harness` to `docker/vision/voice_assistant/Dockerfile`; verify build.
- **D-05: Full node migration on harness** — DialogHarness, PersistentHarness, TelegramHarness, ROS2Transport, SQLiteVoiceMemory all within Phase 6.
- **D-08: PR #907 merge** — Done by USER manually after testing. Phase 6 goal is merge READINESS, not the merge itself.

### the agent's Discretion

- Wave order within groups: A→B→C→D→E→F→G→H (dependency-driven)
- W1–W5 can be parallelized (different files)
- W8 must precede W9 (DialogHarness → DialogueStateMachine)
- W12, W13 needed for W8–W11 (ports used by harnesses)
- W14–W17 after implementation (W8–W13)
- W21–W22 last (quality gates)
- Specific adapter designs within ADR-0001 constraints

### Deferred Ideas (OUT OF SCOPE)

- Merge of PR #907 — user does this manually
- Capability filtering in fallback wrapper — P1 (ADR-0001 §2.6.2)
</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| DOC-MERGE-01 | Merge `docs/architecture/minimax-tts-architecture.md` (401 lines) → `docs/adr/0003-minimax-tts-architecture.md` (325 lines) | §Documentation Merge Strategy |
| DOC-MERGE-02 | Merge `docs/architecture/minimax-tts-integration-design.md` (530 lines) → `docs/adr/0004-minimax-tts-integration-design.md` (659 lines) | §Documentation Merge Strategy |
| DOC-MERGE-03 | Merge fragments `0007a/b/c` (739 lines total) → final `0007-minimax-tts-integration-final.md` | §Documentation Merge Strategy |
| DOC-DEDUP-04 | Remove duplicates from `docs/architecture/`, keep cross-refs to `adr/` | §Documentation Merge Strategy |
| DOC-SPEC-05 | Update `SPEC_CURRENT.md`: P0→Done, describe P1, remove Hermes references | §SPEC_CURRENT.md Update |
| DOCKER-06 | Add `rob_box_harness` to `docker/vision/voice_assistant/Dockerfile` | §Docker Integration |
| DOCKER-07 | Verify Docker build with harness | §Docker Integration |
| HARN-DIALOG-08 | DialogHarness adapter: wrapper over Harness[StateT], LLM→LLMProvider, 30 tools→ToolExecutor, voice_memory→MemoryStore | §DialogHarness |
| HARN-DSM-09 | DialogueStateMachine: migrate DialogueManager + IDLE/LISTENING/DIALOGUE/SILENCED → DSM | §DialogueStateMachine |
| HARN-PERSIST-10 | PersistentHarness: unify 6 nodes via HardwareLifecycle, StatePublisher, Clock, LoggerAdapter, ParameterGuard | §PersistentHarness |
| HARN-TG-11 | TelegramHarness: LLMChat→LLMProvider, MCPBridge→ToolExecutor, 25 handlers→TelegramCommandRegistry, voice_processor→skill, camera_cache→SnapshotStore, auth→middleware | §TelegramHarness |
| PORT-ROS2-12 | ROS2Transport: real Transport implementation for ROS2 topics | §ROS2Transport Port |
| PORT-SQLITE-13 | SQLiteVoiceMemory: MemoryStore implementation for persistent history | §SQLiteVoiceMemory Port |
| TEST-DIALOG-14 | DialogueNode test coverage: 9% → 80%+ | §Test Coverage Strategy |
| TEST-TG-15 | TelegramNode test coverage: 0% → 50%+ | §Test Coverage Strategy |
| TEST-MCP-16 | MCP tools test coverage: → 70%+ | §Test Coverage Strategy |
| TEST-E2E-17 | Integration E2E tests harness + real nodes (after W8–W13) | §Test Coverage Strategy |
| PR-FINAL-18 | PR #907: publish final summary comment | §PR Audit |
| ADR-0008-19 | ADR-0008 audit: verify tts-provider-extension-points-landed | §ADR Audits |
| ADR-0009-20 | ADR-0009 audit: verify integration-test-report | §ADR Audits |
| MYPY-21 | mypy strict-clean on all rob_box_harness | §Quality Gates |
| LINT-22 | black --line-length 120, isort --profile black, flake8 on all changed files | §Quality Gates |
</phase_requirements>

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| rob_box_harness | 0.1.0 (editable) | Harness framework — Harness[StateT], lifecycle, 5 ports, registry | ADR-0001 P0 deliverable; 88 tests, mypy strict-clean [VERIFIED: src/rob_box_harness/setup.py] |
| rob_box_llm | >=0.2.1 (editable) | LLMProvider ABC, TTSProvider ABC, MiniMax providers, errors | Dependency of rob_box_harness; re-exports LLMProvider port [VERIFIED: src/rob_box_harness/setup.py] |
| PyYAML | >=6.0 | YAML config parsing with ${ENV} interpolation | Listed in setup.py install_requires [VERIFIED: src/rob_box_harness/setup.py] |
| httpx | >=0.27 | Async HTTP client for MiniMax API | Used by rob_box_llm providers [VERIFIED: src/rob_box_llm/setup.py] |
| openai | >=1.0 | OpenAI-compatible client (used by DeepSeek, MiniMax) | Used by rob_box_llm for LLMProvider implementations |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| pytest | >=7.4 | Test framework | All tests (W14–W17) |
| pytest-asyncio | >=0.21 | Async test support | Async harness tests |
| pytest-cov | >=4.0 | Coverage reporting | Coverage gates (W14–W17) |
| mypy | latest | Static type checking | W21 quality gate |
| black | 26.3.1 [VERIFIED: env] | Code formatting | W22 quality gate |
| isort | 8.0.1 [VERIFIED: env] | Import sorting | W22 quality gate |
| flake8 | latest | Linting | W22 quality gate |

### Alternatives Considered
| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| Custom Dockerfile setup.py install | COPY rob_box_harness as pip install | Project standard forbids COPY config/scripts; pip install -e . is preferred |

**Installation:**
```bash
pip install -e src/rob_box_llm -e src/rob_box_harness
pip install mypy flake8  # for W21-W22 quality gates
```

**Version verification:** `rob_box_harness` 0.1.0 and `rob_box_llm` 0.2.1 confirmed via `setup.py` files. Both are installed as editable (`pip install -e`) in the workspace.

## Architecture Patterns

### System Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────┐
│  W1–W5: Documentation                                                │
│  docs/architecture/*.md ──merge──▶ docs/adr/0003/0004/0007-*.md     │
│  SPEC_CURRENT.md ──update──▶ P0→Done, P1 described                   │
└──────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│  W6–W7: Docker Integration                                           │
│  docker/vision/voice_assistant/Dockerfile ──add──▶ rob_box_harness   │
│  docker build ──verify──▶ image builds with harness                  │
└──────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│  W12–W13: Port Implementations (shared dependency)                   │
│  ROS2Transport ──implements──▶ Transport (subscribe/publish ROS2)    │
│  SQLiteVoiceMemory ──implements──▶ MemoryStore (append_turn, etc.)   │
└──────────────────────────────────────────────────────────────────────┘
                              │
          ┌───────────────────┼───────────────────┐
          ▼                   ▼                   ▼
┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
│ W8–W9: Dialog    │ │ W10: Persistent  │ │ W11: Telegram    │
│ DialogHarness    │ │ PersistentHarness│ │ TelegramHarness  │
│ + DialogueState  │ │ (6 nodes unified)│ │ + CommandRegistry│
│ Machine          │ │                  │ │ + AuthMiddleware │
└──────────────────┘ └──────────────────┘ └──────────────────┘
          │                   │                   │
          └───────────────────┼───────────────────┘
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│  W14–W17: Test Coverage                                              │
│  DialogNode 9%→80% │ TelegramNode 0%→50% │ MCP tools 0%→70% │ E2E   │
└──────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│  W18–W22: PR Audit + Quality Gates                                   │
│  PR #907 final comment │ ADR-0008/0009 audit │ mypy │ black/isort/flake8│
└──────────────────────────────────────────────────────────────────────┘
```

### Recommended Project Structure
```
src/rob_box_harness/
├── rob_box_harness/
│   ├── harness.py              # Harness[StateT] ABC (existing)
│   ├── harnesses/
│   │   ├── _base.py            # run_request_response_loop helper (existing)
│   │   ├── echo.py             # EchoHarness (existing)
│   │   ├── upper.py            # UpperHarness (existing)
│   │   ├── dialog.py           # DialogHarness (NEW — W8)
│   │   ├── persistent.py       # PersistentHarness (NEW — W10)
│   │   └── telegram.py         # TelegramHarness (NEW — W11)
│   ├── core/
│   │   └── dialogue_state_machine.py  # DSM (NEW — W9)
│   ├── transport/
│   │   └── ros2_transport.py   # ROS2Transport (NEW — W12)
│   ├── memory/
│   │   └── sqlite_voice.py     # SQLiteVoiceMemory (NEW — W13)
│   ├── tools.py                # ToolProvider ABC, FakeToolProvider (existing)
│   ├── memory.py               # MemoryStore ABC, InMemoryStore (existing)
│   ├── transport.py            # Transport ABC, FakeTransport (existing)
│   ├── registry.py             # HarnessRegistry, HarnessFactory (existing)
│   └── providers/
│       └── minimax.py          # MiniMaxProvider harness wrapper (existing)
└── test/
    ├── test_harness_lifecycle.py   (existing)
    ├── test_dialog_harness.py      (NEW — W14)
    ├── test_telegram_harness.py    (NEW — W15)
    ├── test_mcp_tools.py           (NEW — W16)
    ├── test_integration_e2e.py     (NEW — W17)
    ├── test_ros2_transport.py      (NEW — W12)
    └── test_sqlite_voice_memory.py (NEW — W13)
```

### Pattern 1: Harness[StateT] Lifecycle
**What:** Every harness follows: `__init__(config)` → `init()` → `run(input)` → `teardown()`. Idempotent at every stage. Async context manager compatible.
**When to use:** All concrete harnesses (DialogHarness, PersistentHarness, TelegramHarness).
**Example:**
```python
# Source: src/rob_box_harness/rob_box_harness/harness.py (existing)
class MyHarness(Harness[Mapping[str, Any]]):
    name = "my_harness"

    async def step(self, input_data: Any) -> str:
        return await run_request_response_loop(self, input_data, post_process=str.upper)
```

### Pattern 2: Port Dependency Injection
**What:** Ports are supplied via constructor kwargs or built in `init()`. Concrete harnesses override `init()` to inject real providers (DeepSeek, ROS2, SQLite).
**When to use:** Wiring real providers for production harnesses.
**Example:**
```python
# Source: src/rob_box_harness/rob_box_harness/harness.py:134-154 (existing pattern)
async def init(self) -> None:
    if self._initialized: return
    if self.llm is None: self.llm = self._default_llm()
    if self.tools is None: self.tools = self._default_tools()
    # ... concrete harness overrides and injects real providers
```

### Pattern 3: Fake-for-Test Substitution
**What:** Every port has a fake implementation: `FakeToolProvider`, `InMemoryStore`, `FakeTransport`, `NoopBus`, `MockClock`, `DummyLLMProvider`.
**When to use:** All unit tests. No ROS2, no network, no Telegram bot needed.
**Example:**
```python
# Source: src/rob_box_harness/rob_box_harness/tools.py (existing)
class FakeToolProvider(ToolProvider):
    def register(self, spec: ToolSpec, handler: ToolHandler) -> None:
        self._tools[spec.name] = (spec, handler)
```

### Anti-Patterns to Avoid
- **Direct ROS2 publish in harness:** Use `SideEffectBus.dispatch()` or `Transport` port, never `node.publish()` directly. Prevents testability.
- **Secrets in YAML:** ENV-only. YAML placeholder `${MINIMAX_API_KEY}` resolved at load time. Literal keys in YAML → violation of ADR-0001 M7.
- **Blocking I/O in `__init__`:** `__init__` stores config only. All I/O (network, file, ROS2 subscribe) goes in `init()`.
- **Non-idempotent teardown:** `teardown()` must be safe to call multiple times. Check `_initialized` flag.
- **Skipping `super().init()` in overrides:** Concrete harnesses MUST call `await super().init()` to set up default ports.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| LLM client with retry | Custom httpx retry loop | `MiniMaxProvider` (already in rob_box_harness) + `RetryPolicy` | 56 tests, env-auth, redaction, capabilities — battle-tested |
| Tool execution dispatch | dict of name→callable | `ToolProvider` ABC + `FakeToolProvider` / `MCPBridgeProvider` | Standardized ToolCall/ToolResult; LLM-agnostic |
| Conversation history storage | Custom SQLite or JSON file | `MemoryStore` ABC + `InMemoryStore` (tests) / `SQLiteVoiceMemory` (prod) | Four-method contract (load_recent/append_turn/save_fact/search_facts) |
| ROS2 topic subscribe/publish | Direct rclpy calls in harness | `Transport` port + `ROS2Transport` | Transport isolates ROS2; harness stays testable with FakeTransport |
| Side-effect orchestration | Direct `publish_tts()` / `bot.send_message()` | `SideEffectBus.dispatch()` + `CompositeBus` / `RecordingBus` / `NoopBus` | Single fan-out point; recordable in tests |
| Config loading with env interpolation | Custom env parser | `HarnessConfig.from_dict()` / `load_config()` from rob_box_harness | Already handles YAML + ${ENV} + layered configs |
| YAML parsing | Manual yaml.safe_load | `rob_box_harness.load_config()` | Already handles validation, env interpolation, frozen dataclasses |

**Key insight:** The harness framework already provides abstractions for all five ports. Phase 6 is about *concrete implementations* of those ports and *concrete harness subclasses* — NOT about building new abstractions.

## Documentation Merge Strategy

### W1–W2: Architecture → ADR Merges

**W1:** `docs/architecture/minimax-tts-architecture.md` (401 lines, design doc) → merge into `docs/adr/0003-minimax-tts-architecture.md` (325 lines, ADR)
- **Strategy:** ADR-0003 is the canonical; merge any unique content from the architecture version. The architecture file contains ROS-specific mapping tables (§2.1, §2.2) that may supplement the ADR's more abstract descriptions.
- **Post-merge:** Replace `docs/architecture/minimax-tts-architecture.md` with a stub containing only a cross-reference: `See [ADR-0003](../adr/0003-minimax-tts-architecture.md).`
- **Risk:** LOW. Both files describe the same architecture; ADR-0003 is more formal/prose, architecture version is more table-driven. Merge is additive.

**W2:** `docs/architecture/minimax-tts-integration-design.md` (530 lines) → merge into `docs/adr/0004-minimax-tts-integration-design.md` (659 lines)
- **Strategy:** ADR-0004 is larger and more complete. Architecture file has design considerations not captured in ADR. Merge unique content.
- **Post-merge:** Replace with cross-reference stub.
- **Risk:** LOW. ADR-0004 is the canonical design contract.

### W3: Fragment Merge (0007a/b/c → 0007-final)

**Files:** `0007a` (210 lines, reliability), `0007b` (99 lines, ROS2 audio contract), `0007c` (430 lines, runtime operations) → `0007-minimax-tts-integration-final.md`
- **Strategy:** `0007-final` already exists as the synthesis document (§1 explicitly states it incorporates a/b/c content). Verify completeness, remove fragments.
- **Risk:** MEDIUM. Need to verify no unique content is lost. The final ADR already references the fragments; the merge is consolidation, not net-new writing.

### W4: Deduplication

**Files to check:**
- `docs/architecture/minimax-provider.md` vs `docs/adr/0002-minimax-provider.md` — D-01 says these are *different documents* (overview vs ADR). Keep both, add cross-refs.
- `docs/guides/MINIMAX.md` vs `docs/guides/MINIMAX_TTS.md` vs `docs/guides/MINIMAX_TTS_GETTING_STARTED.md` — D-01 says these are *different* (LLM guide, TTS guide, quickstart). Keep all.
- **Action:** Remove only the two architecture files being merged (W1, W2). Keep everything else with cross-references.

### W5: SPEC_CURRENT.md Update

- Mark P0 items as ✅ Done (Harness Framework, MiniMax Provider, PR #907 reviews)
- Describe P1 scope: DialogHarness, PersistentHarness, TelegramHarness, ROS2Transport, SQLiteVoiceMemory (from ADR-0001 §2.7)
- Remove Hermes-specific references: `hermes kanban create`, `kanban_complete`, `t_*` task IDs, worktree references
- **Risk:** LOW. SPEC_CURRENT.md already has an accurate P0/P1 split; this is a status update.

## Docker Integration

### W6: Dockerfile Modification

**Target:** `docker/vision/voice_assistant/Dockerfile`

**Required change:** Before `colcon build`, add `COPY src/rob_box_harness /ws/src/rob_box_harness` and `COPY src/rob_box_llm /ws/src/rob_box_llm`. Add both to `--packages-select`.

**Current Dockerfile structure:**
- Builds from `voice-base-humble-latest` (contains Python deps, ReSpeaker, SuperCollider)
- Copies `rob_box_voice`, `rob_box_animations`, `rob_box_mcp_tools`, `rob_box_perception_msgs`
- Does NOT copy `rob_box_harness` or `rob_box_llm`
- Uses `colcon build --packages-select rob_box_perception_msgs rob_box_mcp_tools rob_box_voice rob_box_animations`

**Required additions:**
```dockerfile
# After existing package.xml copies, add:
COPY src/rob_box_harness/package.xml /ws/src/rob_box_harness/
COPY src/rob_box_llm/package.xml /ws/src/rob_box_llm/

# After existing setup.py copies, add:
COPY src/rob_box_harness/setup.py /ws/src/rob_box_harness/
COPY src/rob_box_harness/setup.cfg /ws/src/rob_box_harness/
COPY src/rob_box_llm/setup.py /ws/src/rob_box_llm/
COPY src/rob_box_llm/setup.cfg /ws/src/rob_box_llm/

# After existing Python source copies, add:
COPY src/rob_box_harness/rob_box_harness /ws/src/rob_box_harness/rob_box_harness
COPY src/rob_box_llm/rob_box_llm /ws/src/rob_box_llm/rob_box_llm

# Update colcon build:
RUN ... colcon build --packages-select rob_box_perception_msgs rob_box_mcp_tools rob_box_llm rob_box_harness rob_box_voice rob_box_animations ...
```

**Project constraints (from copilot-instructions.md):**
- ❌ NEVER `COPY config/` or `COPY scripts/` in Dockerfile
- ✅ Always `network_mode: host` (this is in docker-compose, not Dockerfile)
- ✅ `rob_box_harness` depends on `rob_box_llm>=0.2.1` — build order matters in colcon

**Risk:** MEDIUM. `rob_box_llm` has dependencies (httpx, openai, PyYAML) that must be in the base image. `voice-base` already has these via `requirements.txt`, but need to verify.

### W7: Docker Build Verification

**Command:** `docker build -f docker/vision/voice_assistant/Dockerfile -t rob_box_voice:test .`

**Risk factors:**
- Package dependency resolution in colcon (rob_box_llm must build before rob_box_harness)
- Python import conflicts between editable and colcon installs
- Build time: voice-base image is large (~2GB+)

## DialogHarness (W8)

### Scope

Create `src/rob_box_harness/rob_box_harness/harnesses/dialog.py` — a `Harness[StateT]` subclass that wraps the dialogue logic currently in `dialogue_node.py` (2316 lines, 9% coverage).

### What stays in dialogue_node.py (thin ROS2 wrapper)
- ROS2 subscribers: `/voice/stt/result`, `/audio/vad`, `/voice/tts/finished`
- ROS2 publishers: `/voice/dialogue/response`, `/voice/dialogue/state`, `/voice/sound/trigger`
- ROS2 lifecycle: `__init__` / `destroy_node`
- Asyncio loop driver (single-worker ThreadPoolExecutor)

### What moves into DialogHarness
- LLM client + fallback → `LLMProvider` (already exists: MiniMaxProvider, DeepSeekProvider)
- 30 tools / 5 skills → `ToolExecutor` + `SkillRegistry` (port already defined)
- `DialogueManager` + IDLE/LISTENING/DIALOGUE/SILENCED → `DialogueStateMachine` (W9)
- `voice_memory`, `faq_store` → `MemoryStore` (port already defined)
- `_ask_llm_streaming` / `_ask_llm_non_streaming` / `_handle_volume_command` → skills
- `voice_processor`-like logic → `VoiceSettingsSkill`

### Key interfaces
```python
class DialogHarness(Harness[DialogState]):
    name = "dialog"

    async def init(self) -> None:
        await super().init()
        # Inject real providers
        self.llm = MiniMaxProvider(api_key=os.environ["MINIMAX_API_KEY"])
        self.tools = MCPBridgeProvider(...)  # or LocalSkillProvider
        self.memory = SQLiteVoiceMemory(path="~/.rob_box/voice.db")
        self.transport = ROS2Transport(...)
        # Build skill registry
        self._skills = SkillRegistry([VoiceSettingsSkill(), DJPlaylistSkill(), MappingSkill()])

    async def step(self, input_data: Any) -> str:
        # input_data is STT text
        return await self._agent_session.on_user_input(input_data)
```

### Risk Assessment
- **HIGH risk:** `dialogue_node.py` is 2316 lines with 9% coverage. Extracting logic without breaking behavior requires deep understanding of the existing code flow.
- **Mitigation:** Build DialogHarness as a *parallel* implementation — don't modify dialogue_node.py. Ship both side-by-side, switch via config flag. This allows incremental migration.
- **Key challenge:** The existing `Runner.run_streamed()` from OpenAI Agents SDK is deeply embedded. DialogHarness must either wrap it or provide an alternative agent loop.

## DialogueStateMachine (W9)

### Scope

Extract `DialogueManager` and state transitions (IDLE/LISTENING/DIALOGUE/SILENCED) from `dialogue_node.py` into a separate state machine class.

### Design
- Pure state machine — no I/O, no ROS2, no LLM
- States: IDLE → LISTENING (wake word detected) → DIALOGUE (user speech processing) → SILENCED (user said "тихо" / "молчи")
- Transitions triggered by events (wake_word, stt_result, silence_command, dialogue_end)
- Side-effect callbacks for TTS abort, LED state

### Risk: MEDIUM. State logic is well-bounded in the existing `DialogueManager` class.

## PersistentHarness (W10)

### Scope

Unify 6 persistent nodes (`audio_node`, `stt_node`, `tts_node`, `sound_node`, `led_node`, `command_node`) under a shared `HardwareLifecycle`.

### What PersistentHarness provides
- `HardwareLifecycle` — connect/disconnect/health-check/restart-on-error
- `StatePublisher` — unified `State { name, status, last_error, uptime }` on `/<node>/state`
- `Clock` — DI time interface (already in harness)
- `LoggerAdapter` — structured logging (already in harness)
- `ParameterGuard` — declare/validate/reload ROS parameters

### What each node keeps
- Its own device-specific code (ReSpeaker for audio, Yandex gRPC for STT, etc.)
- Its own ROS2 topics (subscribers/publishers)
- Its specific `run_hardware_loop()` / `process(ros_msg)` method

### Risk: LOW. PersistentHarness is a lightweight unification layer — no LLM, no agent loop. Most work is extracting common lifecycle patterns from 6 existing nodes.

## TelegramHarness (W11)

### Scope

Wrap `telegram_node.py` (407 lines) + handlers (`commands.py` 534 lines, `messages.py` 199 lines, `callbacks.py` 165 lines) under a `Harness` subclass.

### Migration Map
| Current Code | Lines | Target |
|-------------|-------|--------|
| `LLMChat` | 469 | `LLMProvider` port (reuse existing) |
| `MCPBridge` | 137 | `ToolExecutor` port |
| `commands.py` (25 handlers) | 534 | `TelegramCommandRegistry` (declarative: command → skill) |
| `messages.py` | 199 | `AgentSession.on_user_input` |
| `voice_processor.py` | 134 | `STTForTelegramSkill` |
| `camera_cache.py` | 76 | `SnapshotStore` port |
| `auth.py` | 89 | AuthMiddleware at dispatcher level |

### Risk: MEDIUM. Telegram code is well-structured but has 0% test coverage. The `python-telegram-bot` library has its own async event loop — integration with harness lifecycle needs careful design.

## ROS2Transport Port (W12)

### Scope

Implement `Transport` ABC with real ROS2 subscriptions/publishers.

### Required methods (from Transport ABC)
- `on_stt_result(text, confidence)` — subscribe to `/voice/stt/result`
- `on_vad(event)` — subscribe to `/audio/vad`
- `on_telegram_update(update)` — subscribe to `/telegram/updates`
- `on_key_event(event)` — subscribe to keyboard events

### Design considerations
- Must work within rclpy spinning (callback-based)
- Should bridge ROS2 callbacks → async harness events
- Must handle ROS2 QoS (reliable/volatile, queue depth)
- Test with `FakeTransport` (already exists) for unit tests

### Risk: LOW. Transport ABC is well-defined. The main challenge is bridging sync ROS2 callbacks to async harness methods.

## SQLiteVoiceMemory Port (W13)

### Scope

Implement `MemoryStore` ABC with SQLite persistence.

### Required methods (from MemoryStore ABC)
- `load_recent(scope, limit)` → `list[Turn]`
- `append_turn(scope, turn)` → idempotent append
- `save_fact(scope, fact)` → persist structured fact
- `search_facts(scope, query, top_k)` → best-effort semantic search

### Existing code to reuse
- `rob_box_voice/core/voice_memory.py` — `VoiceMemory` class with SQLite schema
- Check if existing schema matches `Turn`/`Fact` dataclasses from harness

### Risk: LOW. MemoryStore ABC is well-defined. Existing VoiceMemory provides a reference SQLite implementation.

## Test Coverage Strategy

### Current State
| Module | Current Coverage | Target | Lines |
|--------|-----------------|--------|-------|
| `dialogue_node.py` | 9% | 80%+ | 2316 |
| `telegram_node.py` + handlers | 0% | 50%+ | ~2300 combined |
| MCP tools (`rob_box_mcp_tools`) | unknown | 70%+ | various |
| `rob_box_harness` | 90%+ | maintain | ~1500 |

### Testing Approach

**W14 — DialogueNode (9% → 80%+):**
- Use `FakeLLMProvider` / `DummyLLMProvider` for LLM calls
- Use `FakeToolProvider` for tool execution
- Use `InMemoryStore` for voice_memory
- Use `RecordingBus` for side-effect verification
- Use `FakeTransport` for input events
- Test: state transitions, tool dispatch, response formatting, error handling, barge-in

**W15 — TelegramNode (0% → 50%+):**
- Mock `python-telegram-bot` Application
- Use fake ports for LLM/MCP/Memory
- Test: command routing, message handling, auth middleware, callback processing

**W16 — MCP tools (0% → 70%+):**
- Test each tool function with mock ROS2 service calls
- Test ToolProvider integration (discover/execute)
- Error handling and timeout behavior

**W17 — Integration E2E:**
- Spin up real harness + fake ports
- Test: voice input → LLM → TTS side-effect chain
- Test: Telegram message → skill → response
- Test: wake word → state transition → dialogue → silence

### Test Infrastructure
- Framework: pytest (6.2.5) + pytest-asyncio (≥0.21) + pytest-cov (≥4.0)
- Config: `pytest.ini` at repo root (asyncio_mode=auto, coverage gate 85%)
- Test paths: `src/rob_box_harness/test/` and `tests/unit/harness/`
- Coverage target: 85% on `rob_box_harness` (existing gate)

## PR Audit (W18–W20)

### W18: PR #907 Final Comment
- Publish a structured summary comment covering: review status, remaining work (Phase 6 waves), merge readiness criteria
- Use `gh pr comment` or `gh pr review`

### W19: ADR-0008 Audit
- ADR-0008 documents `tts-provider-extension-points-landed`
- Verify: 5 extension points (capabilities, list_voices, healthcheck, _http_client_factory, _build_request_payload) are all implemented in `src/rob_box_llm/rob_box_llm/tts_provider_base.py` and `tts_provider_registry.py`
- `BaseTTSProvider` is subclassed by `MiniMaxTTSProvider` — verify inheritance chain

### W20: ADR-0009 Audit
- ADR-0009 documents `harness-tts-contract` — the TTSProvider contract in harness
- Verify: `HarnessMiniMaxTTSProvider` in `src/rob_box_harness/rob_box_harness/tts/minimax_tts.py` follows the contract
- Verify: harness-side registry (`src/rob_box_harness/rob_box_harness/tts/registry.py`) matches ADR-0009 §3
- Verify: 64 TTS tests passing

## Quality Gates (W21–W22)

### W21: mypy strict-clean
- Command: `mypy src/rob_box_harness/rob_box_harness/`
- Current state: ✅ mypy strict-clean (per SPEC_CURRENT.md)
- Scope: ensure NO regressions from new code in W8–W13
- Tool availability: mypy NOT installed [VERIFIED: env]; requires `pip install mypy`

### W22: Linters
- `black --line-length 120` — format all changed Python files
- `isort --profile black` — sort imports
- `flake8` — lint check
- Scope: all files modified in Phase 6 waves (W8–W13 code + any touched files)
- Tool availability: black 26.3.1 ✓, isort 8.0.1 ✓, flake8 NOT installed ✗ [VERIFIED: env]

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Python 3.10+ | All code | ✓ | 3.10.12 | — |
| pip | Package install | ✓ | (bundled) | — |
| pytest | W14–W17 tests | ✓ | 6.2.5 | — |
| pytest-asyncio | Async tests | ✓ | (installed) | — |
| pytest-cov | Coverage | ✓ | (installed) | — |
| rob_box_llm | W6–W11, W14–W17 | ✓ | 0.2.1 (editable) | — |
| rob_box_harness | W6–W11 | ✓ | 0.1.0 (editable) | — |
| mypy | W21 | ✗ | — | `pip install mypy` |
| black | W22 | ✓ | 26.3.1 | — |
| isort | W22 | ✓ | 8.0.1 | — |
| flake8 | W22 | ✗ | — | `pip install flake8` |
| Docker | W7 | ✓ | 29.1.3 | — |
| gh CLI | W18 | ✓ | 2.74.0 | — |
| git | All | ✓ | (installed) | — |

**Missing dependencies with no fallback:**
- `mypy` — must be installed for W21. Command: `pip install mypy`
- `flake8` — must be installed for W22. Command: `pip install flake8`

**Missing dependencies with fallback:**
- None. All missing tools are pip-installable.

## Common Pitfalls

### Pitfall 1: Breaking dialogue_node.py During Migration
**What goes wrong:** Modifying `dialogue_node.py` in-place while building DialogHarness breaks the existing voice assistant on the robot.
**Why it happens:** 2316-line file with tightly coupled logic; easy to introduce subtle regressions.
**How to avoid:** Build DialogHarness as a **parallel implementation** in `src/rob_box_harness/`. Do not modify `dialogue_node.py` at all. Ship both, switch via config flag (`harness.kind: dialog` vs legacy mode). Only deprecate the old path after test parity is achieved.
**Warning signs:** Tests failing in `dialogue_node.py` after harness commits; robot voice stops responding.

### Pitfall 2: Dependency Resolution in colcon Build
**What goes wrong:** `rob_box_harness` depends on `rob_box_llm>=0.2.1`, but colcon builds in alphabetical order and may try to build harness first.
**Why it happens:** colcon uses package.xml `<depend>` tags; if not properly declared, build order is wrong.
**How to avoid:** Ensure `rob_box_harness/package.xml` declares `<depend>rob_box_llm</depend>`. Add both to `--packages-select` in correct order. Test with `colcon build --packages-up-to rob_box_harness`.
**Warning signs:** `ModuleNotFoundError: No module named 'rob_box_llm'` during colcon build.

### Pitfall 3: ROS2 callback → async harness bridge
**What goes wrong:** ROS2 subscriptions use sync callbacks, but harness methods are async. Direct `await` in sync callback → `RuntimeError: no running event loop`.
**Why it happens:** rclpy spins in its own thread/executor; asyncio loop may be in a different thread.
**How to avoid:** Use `asyncio.run_coroutine_threadsafe()` to bridge sync ROS2 callbacks to the harness's async event loop. ROS2Transport must own a reference to the harness's event loop.
**Warning signs:** `RuntimeError` about event loops in ROS2 node logs.

### Pitfall 4: Documentation Merge Losing Unique Content
**What goes wrong:** Merging architecture docs into ADRs discards unique tables, diagrams, or implementation notes present only in the architecture version.
**Why it happens:** Both files evolved independently; some content may exist in only one.
**How to avoid:** Diff both files before merge. Create a checklist of unique sections from each. Ensure all unique content is preserved in the final ADR. The architecture stub file should link to the ADR, not duplicate.
**Warning signs:** Post-merge, a diagram or table referenced in another doc is no longer findable.

### Pitfall 5: Test Coverage Inflation Without Real Value
**What goes wrong:** Reaching 80% coverage targets with superficial tests that don't actually verify behavior.
**Why it happens:** Pressure to hit coverage numbers; large file with many code paths.
**How to avoid:** Focus tests on behavior, not line coverage. Test: state transitions, tool dispatch correctness, error recovery paths, barge-in handling. Use `RecordingBus` to verify correct side-effects. Integration tests (W17) as capstone.
**Warning signs:** 80% coverage but all tests use `DummyLLMProvider` and only test the happy path.

### Pitfall 6: mypy Regression From New Code
**What goes wrong:** Adding new harness implementations (W8–W11) introduces type errors that break the existing mypy strict-clean status.
**Why it happens:** New code may have loose typing; mypy strict mode catches many issues.
**How to avoid:** Run `mypy --strict src/rob_box_harness/` after each wave. Fix type errors incrementally. Don't batch all type fixes to W21.
**Warning signs:** mypy reports new errors after any W8–W13 commit.

## Code Examples

### Minimal DialogHarness Skeleton
```python
# Source: Pattern from src/rob_box_harness/rob_box_harness/harnesses/echo.py (existing)
from __future__ import annotations

from typing import Any, Mapping

from rob_box_harness.harness import Harness
from rob_box_harness.harnesses._base import run_request_response_loop
from rob_box_harness.memory import MemoryStore, Turn
from rob_box_harness.tools import ToolProvider


class DialogHarness(Harness[Mapping[str, Any]]):
    """Thin harness wrapping the voice dialog agent."""

    name = "dialog"

    async def init(self) -> None:
        await super().init()
        # Real providers are injected here (or by caller before async with).
        # Default ports from super().init() are in-memory/fake — fine for tests.

    async def step(self, input_data: Any) -> str:
        """Process one user utterance (STT text) and return the assistant's reply."""
        # 1. Append user turn to memory
        await self.memory.append_turn(
            scope=self._current_scope(),
            turn=Turn(role="user", content=str(input_data)),
        )
        # 2. Load recent context
        history = await self.memory.load_recent(
            scope=self._current_scope(), limit=20
        )
        # 3. Discover tools
        tool_specs = await self.tools.discover()
        # 4. Call LLM
        response = await self.llm.complete(
            messages=self._build_messages(history),
            tools=[{"type": "function", "function": {**s.parameters, "name": s.name}} for s in tool_specs],
        )
        # 5. Append assistant turn
        await self.memory.append_turn(
            scope=self._current_scope(),
            turn=Turn(role="assistant", content=response.content),
        )
        # 6. Dispatch side-effect (TTS)
        await self.effects.dispatch(EchoEffect(response.content))
        return response.content

    def _current_scope(self) -> str:
        return f"dialog:{self.config.harness.name}"

    def _build_messages(self, history: list[Turn]) -> list[dict[str, str]]:
        return [{"role": t.role, "content": t.content} for t in history]
```

### State Machine Pattern for DialogueStateMachine
```python
# Source: Pattern from ADR-0001 §2.7.1 (pseudocode, to be implemented)
from __future__ import annotations

from enum import Enum, auto
from typing import Callable, Awaitable


class DialogueState(Enum):
    IDLE = auto()
    LISTENING = auto()
    DIALOGUE = auto()
    SILENCED = auto()


class DialogueStateMachine:
    """Pure state machine — no I/O, no ROS2, no LLM."""

    def __init__(self) -> None:
        self._state: DialogueState = DialogueState.IDLE
        self._silenced_until: float | None = None
        self._callbacks: dict[DialogueState, list[Callable[[DialogueState, DialogueState], Awaitable[None]]]] = {}

    @property
    def state(self) -> DialogueState:
        return self._state

    def on_transition(self, from_state: DialogueState, to_state: DialogueState):
        """Decorator: register a callback for state transition."""
        def decorator(fn):
            self._callbacks.setdefault(to_state, []).append(fn)
            return fn
        return decorator

    async def transition(self, to: DialogueState) -> None:
        """Transition to a new state, firing registered callbacks."""
        if to == self._state:
            return
        from_state = self._state
        self._state = to
        for cb in self._callbacks.get(to, []):
            await cb(from_state, to)
```

### Dockerfile Addition Pattern
```dockerfile
# Source: Pattern from existing docker/vision/voice_assistant/Dockerfile (existing)
# Add after existing rob_box_voice package.xml COPY:
COPY src/rob_box_harness/package.xml /ws/src/rob_box_harness/
COPY src/rob_box_llm/package.xml /ws/src/rob_box_llm/

# ... (setup.py copies) ...
COPY src/rob_box_harness/setup.py /ws/src/rob_box_harness/
COPY src/rob_box_llm/setup.py /ws/src/rob_box_llm/

# ... (source code copies) ...
COPY src/rob_box_harness/rob_box_harness /ws/src/rob_box_harness/rob_box_harness
COPY src/rob_box_llm/rob_box_llm /ws/src/rob_box_llm/rob_box_llm

# Update colcon build:
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ws/install/setup.sh && \
    colcon build \
    --packages-select rob_box_perception_msgs rob_box_mcp_tools rob_box_llm rob_box_harness rob_box_voice rob_box_animations \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| dialogue_node does everything (LLM, tools, state, memory) | DialogHarness delegates to 5 ports + DSM | Phase 6 (in progress) | Testable, swappable LLM, shared memory with TG |
| Each persistent node manages its own lifecycle | PersistentHarness provides unified HardwareLifecycle | Phase 6 (in progress) | Consistent startup/shutdown, health monitoring |
| Telegram has isolated LLM/MCP/handlers | TelegramHarness reuses LLMProvider + ToolExecutor | Phase 6 (in progress) | Shared context with voice, common tools |
| Transport is implicit (direct ROS2 calls) | Transport port with ROS2Transport + FakeTransport | Phase 6 (in progress) | Testable without ROS2, clean input boundary |
| Voice memory is dialogue_node-specific SQLite | MemoryStore port with SQLiteVoiceMemory | Phase 6 (in progress) | Reusable across dialog/telegram, swappable backend |

**Deprecated/outdated:**
- Direct `node.publish()` for side-effects — replaced by `SideEffectBus.dispatch()`
- `LLMChat` class in telegram — replaced by `LLMProvider` port
- `MCPBridge` class — replaced by `ToolExecutor` port
- Ad-hoc state management in dialogue_node — replaced by `DialogueStateMachine`

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `docker/vision/voice_base` image already contains httpx, openai, PyYAML (dependencies of rob_box_llm) | Docker Integration | Docker build fails; need to add deps to voice_base Dockerfile |
| A2 | `rob_box_voice/core/voice_memory.py` SQLite schema is compatible with harness `Turn`/`Fact` dataclasses | SQLiteVoiceMemory | Schema migration needed; breaking change to existing voice data |
| A3 | `python-telegram-bot` Application's async loop can coexist with harness async lifecycle | TelegramHarness | Event loop conflict; need to run harness in same loop as PTB |
| A4 | Existing tools/skills in dialogue_node can be registered as `FakeToolProvider` handlers without modification | DialogHarness | Tool interface mismatch; need adapter layer for existing tool signatures |
| A5 | The `rob_box_harness` package.xml declares `<depend>rob_box_llm</depend>` or colcon resolves order from setup.py | Docker Integration | Build order wrong; colcon fails |
| A6 | The 0007a/b/c fragments don't contain unique content that isn't already in 0007-final | Documentation Merge | Data loss; fragments should be preserved or re-incorporated |

**If this table is empty:** Not applicable — assumptions have been identified.

## Open Questions

1. **Should DialogHarness be a parallel implementation or in-place refactor of dialogue_node.py?**
   - What we know: dialogue_node.py is 2316 lines with 9% coverage. ADR-0001 says "thin ROS2 wrapper remains; logic moves inside."
   - What's unclear: Is the intent to (a) modify dialogue_node.py to delegate to DialogHarness, or (b) create DialogHarness as a separate file and keep dialogue_node.py unchanged?
   - Recommendation: **Parallel implementation (option b).** Better for risk management and incremental adoption. The CONTEXT.md (D-05) says "DialogHarness адаптер: создать класс-обёртку над Harness[StateT]" — this suggests a wrapper, not a rewrite.

2. **What is the `rob_box_harness/package.xml` dependency declaration for rob_box_llm?**
   - What we know: setup.py declares `rob_box_llm>=0.2.1` as install_requires. package.xml may not have the corresponding `<depend>`.
   - What's unclear: colcon uses package.xml for build order. If `<depend>rob_box_llm</depend>` is missing, colcon may build harness before llm.
   - Recommendation: Verify and add if missing BEFORE W6 Dockerfile work.

3. **What is the current MCP tools test coverage baseline?**
   - What we know: SPEC_CURRENT says target is 70%+ but doesn't state current coverage.
   - What's unclear: Is coverage truly 0% or is there some existing test infrastructure?
   - Recommendation: Run `pytest --cov=rob_box_mcp_tools` before W16 to establish baseline.

4. **Are the 0007 fragments truly fully incorporated into 0007-final?**
   - What we know: 0007-final references 0007a and 0007b as "sub-fragments (детализация, не самостоятельные ADR)". 0007c is not listed in the header but exists on disk.
   - What's unclear: Does 0007c contain unique content not in 0007-final? The final ADR's header doesn't list 0007c as a sub-fragment.
   - Recommendation: Diff 0007c against 0007-final before merging. If unique, incorporate; otherwise, note and remove.

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest 6.2.5 + pytest-asyncio >=0.21 + pytest-cov >=4.0 |
| Config file | `pytest.ini` (repo root) |
| Quick run command | `python3 -m pytest src/rob_box_harness/test -x -q` |
| Full suite command | `python3 -m pytest src/rob_box_harness/test tests/unit/harness --cov=rob_box_harness --cov-fail-under=85` |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| HARN-DIALOG-08 | DialogHarness wraps LLM+tools+memory | unit | `pytest src/rob_box_harness/test/test_dialog_harness.py -x` | ❌ Wave 0 |
| HARN-DSM-09 | DialogueStateMachine state transitions | unit | `pytest src/rob_box_harness/test/test_dsm.py -x` | ❌ Wave 0 |
| HARN-PERSIST-10 | PersistentHarness lifecycle unification | unit | `pytest src/rob_box_harness/test/test_persistent_harness.py -x` | ❌ Wave 0 |
| HARN-TG-11 | TelegramHarness command routing | unit | `pytest src/rob_box_harness/test/test_telegram_harness.py -x` | ❌ Wave 0 |
| PORT-ROS2-12 | ROS2Transport subscribe/publish | unit | `pytest src/rob_box_harness/test/test_ros2_transport.py -x` | ❌ Wave 0 |
| PORT-SQLITE-13 | SQLiteVoiceMemory CRUD | unit | `pytest src/rob_box_harness/test/test_sqlite_voice_memory.py -x` | ❌ Wave 0 |
| TEST-DIALOG-14 | dialogue_node 9%→80% | unit | `pytest src/rob_box_voice/test/ -x --cov=rob_box_voice.dialogue_node` | ❌ Wave 0 |
| TEST-TG-15 | telegram_node 0%→50% | unit | `pytest src/rob_box_telegram/test/ -x --cov=rob_box_telegram` | ❌ Wave 0 |
| TEST-MCP-16 | MCP tools 0%→70% | unit | `pytest src/rob_box_mcp_tools/test/ -x --cov=rob_box_mcp_tools` | ❌ Wave 0 |
| TEST-E2E-17 | Integration E2E harness+real nodes | integration | `pytest tests/integration/ -x -m integration` | ❌ Wave 0 |
| MYPY-21 | mypy strict-clean | static | `mypy src/rob_box_harness/rob_box_harness/` | ✅ existing |
| LINT-22 | black/isort/flake8 | static | `black --check ... && isort --check ... && flake8 ...` | ✅ existing |

### Sampling Rate
- **Per task commit:** `python3 -m pytest src/rob_box_harness/test -x -q` (fast unit tests)
- **Per wave merge:** `python3 -m pytest src/rob_box_harness/test tests/unit/harness --cov=rob_box_harness` (full unit suite)
- **Phase gate:** Full suite green + coverage ≥85% + mypy strict-clean + linters pass

### Wave 0 Gaps
- [ ] `test/test_dialog_harness.py` — covers HARN-DIALOG-08
- [ ] `test/test_dsm.py` — covers HARN-DSM-09
- [ ] `test/test_persistent_harness.py` — covers HARN-PERSIST-10
- [ ] `test/test_telegram_harness.py` — covers HARN-TG-11
- [ ] `test/test_ros2_transport.py` — covers PORT-ROS2-12
- [ ] `test/test_sqlite_voice_memory.py` — covers PORT-SQLITE-13
- [ ] `src/rob_box_voice/test/` — covers TEST-DIALOG-14 (may need directory creation)
- [ ] `src/rob_box_telegram/test/` — covers TEST-TG-15 (may need directory creation)
- [ ] `src/rob_box_mcp_tools/test/` — covers TEST-MCP-16 (may need directory creation)
- [ ] `tests/integration/` — covers TEST-E2E-17 (may need directory creation)
- [ ] `pip install mypy flake8` — tools for W21-W22
- [ ] `conftest.py` shared fixtures — for new test files (FakeLLMProvider, RecordingBus, etc.)

## Security Domain

### Applicable ASVS Categories

| ASVS Category | Applies | Standard Control |
|---------------|---------|-----------------|
| V2 Authentication | yes | `MINIMAX_API_KEY` via ENV only; YAML literals forbidden (ADR-0001 M7) |
| V3 Session Management | no | N/A — robot is single-user |
| V4 Access Control | yes | `AuthMiddleware` for Telegram (W11); capability registry for tool access |
| V5 Input Validation | yes | `Pydantic` / dataclass validation in config layer; `TTSSettings` frozen dataclass; `extra` key whitelist |
| V6 Cryptography | no | N/A — no custom crypto; TLS via httpx to external APIs |

### Known Threat Patterns for harness stack

| Pattern | STRIDE | Standard Mitigation |
|---------|--------|---------------------|
| API key leak via YAML literals | Information Disclosure | ENV-only; `ConfigError` on literal keys; `MiniMaxRedactedLogFilter` |
| Tool injection via unvalidated extra fields | Tampering | `TTSSettings.extra` whitelist (9 allowed keys); reserved key rejection |
| Side-effect bus allowing unauthorized ROS2 publish | Elevation of Privilege | `SideEffectBus.dispatch()` as single choke point; `RecordingBus` for audit |
| Memory store SQL injection (if using raw SQL) | Tampering | Parameterized queries in SQLiteVoiceMemory (use `?` placeholders, never f-strings) |
| Telegram bot token leak in logs | Information Disclosure | `redact` config in `LoggingConfig`; structured logging with field-level redaction |

## Sources

### Primary (HIGH confidence)
- `docs/adr/0001-harness-architecture.md` — complete ADR: Harness[StateT], 5 ports, lifecycle, P0/P1 boundary [VERIFIED: read full 956 lines]
- `docs/adr/0009-harness-tts-contract.md` — TTSProvider contract in harness [VERIFIED: read full ADR]
- `src/rob_box_harness/rob_box_harness/harness.py` — Harness[StateT] ABC implementation [VERIFIED: read implementation]
- `src/rob_box_harness/rob_box_harness/tools.py` — ToolProvider ABC + FakeToolProvider [VERIFIED: read implementation]
- `src/rob_box_harness/rob_box_harness/memory.py` — MemoryStore ABC + InMemoryStore [VERIFIED: read implementation]
- `src/rob_box_harness/rob_box_harness/transport.py` — Transport ABC + FakeTransport [VERIFIED: read implementation]
- `src/rob_box_harness/setup.py` — Package dependencies [VERIFIED: read file]
- `SPEC_CURRENT.md` — Current P0/P1 state [VERIFIED: read full file]
- `docker/vision/voice_assistant/Dockerfile` — Target Dockerfile [VERIFIED: read full file]
- `.planning/phases/06-harness-p0-finalization/06-CONTEXT.md` — User decisions [VERIFIED: read full file]

### Secondary (MEDIUM confidence)
- `docs/adr/0002-minimax-provider.md` — MiniMax provider ADR [VERIFIED: read §1-3]
- `docs/adr/0007-minimax-tts-integration-final.md` — TTS integration final ADR [VERIFIED: read §1-2]
- `docs/adr/0008-tts-provider-extension-points-landed.md` — Landed extension points [VERIFIED: read §1-3]
- `docs/adr/0003-minimax-tts-architecture.md` — TTS architecture ADR [VERIFIED: read §1-2]
- `docs/architecture/minimax-tts-architecture.md` — TTS architecture design doc [VERIFIED: read §1-2]
- `src/rob_box_harness/README.md` — Framework API docs [VERIFIED: read full file]
- `docs/guides/harness-quickstart.md` — Harness creation guide [VERIFIED: read full file]
- `src/rob_box_voice/rob_box_voice/dialogue_node.py` — Migration target (2316 lines) [VERIFIED: lines 1-100]
- `.planning/config.json` — GSD configuration [VERIFIED: read file]

### Tertiary (LOW confidence)
- `src/rob_box_voice/core/voice_memory.py` — Existing SQLite schema (not fully read; assumed compatible) [ASSUMED A2]
- `src/rob_box_telegram/` — Telegram node code structure (not fully verified; size estimates from ADR-0001) [CITED: ADR-0001 §2.7.3]
- Tool availability: docker/gh physical network access (not tested) [ASSUMED]

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — all libraries verified via setup.py, environment, and existing imports
- Architecture: HIGH — ADR-0001 through ADR-0009 are all accepted and match code; harness framework is built and tested
- Pitfalls: HIGH — most risks are well-understood (large file extraction, Docker build order, async bridge)
- Documentation merge scope: MEDIUM — exact diff between architecture and ADR versions not fully analyzed; unique content may exist

**Research date:** 2026-07-27
**Valid until:** 2026-08-27 (stable — harness framework and ADRs are locked decisions)

**Researcher notes:**
- All 6 assumptions (A1–A6) identified in the Assumptions Log need validation in the first planning wave
- The dialogue_node.py migration (W8) is the highest-risk wave — strongly recommend parallel implementation approach
- Missing tools (mypy, flake8) are trivial to install; documented in Environment Availability
- No knowledge graph available for cross-document relationship discovery
- The harness test suite cannot be run locally due to pytest marker configuration mismatch (pytest 6.2.5 vs newer pytest-asyncio); tests pass in CI per SPEC_CURRENT.md
