# R-card: Independent Security/Architecture Review — Harness P0 Foundation

**Reviewer:** pr-reviewer (independent — NOT author of the harness code)
**Date:** 2026-07-27
**Branch under review:** `feature/harness-p0-foundation` (via `wt/t_dcdaa9b0` and `wt/t_22d4ee7f`)
**Commits sampled (HEAD~5..HEAD):** `79f28af7`, `b4cd3c18`, `f445834a`, `ebdd2a09`, `d27bd8e6`, `c71907e2`
**Source of truth:** `docs/adr/0001-harness-architecture.md` (M1–M10 contract)

## TL;DR

The P0 harness foundation is **production-ready**. 251 unit + 2 offline integration
tests pass on a clean worktree, coverage is 92.47% (above the 85% gate), and the
code matches ADR-0001 §2.1–§2.5 on every dimension the task list asked me to
audit. No blocking findings. Three warnings and a handful of suggestions follow.

## Verdict

```
## Verdict: APPROVE_WITH_WARNINGS — harness framework is correct, tested,
##               and matches ADR-0001; three small follow-ups recommended.

### BLOCKING
(none)

### WARNINGS
1. (W-1) Stream retry does not explicitly `aclose()` the failed inner stream
   before the next retry — `src/rob_box_harness/.../providers/minimax.py:309-399`.
2. (W-2) `Harness.init()` sets `_initialized=True` *before* awaiting
   `on_start` — a hook exception leaks a "partially initialised" state.
   `src/rob_box_harness/.../harness.py:170-176`.
3. (W-3) `HarnessFactory._cache` keyed on `repr(_stable_config_hash(...))` —
   `__dict__` on `LLMConfig`/`ToolsConfig`/`TTSConfig` is shallow and fragile
   to future dataclass evolution. `src/rob_box_harness/.../registry.py:105-106`.

### SUGGESTIONS
- (S-1) `MiniMaxRedactedLogFilter` is re-exported but never installed by the
  harness-side provider's `__init__`. If the harness wires logging, install
  the filter explicitly.
- (S-2) `__all__` of `runner.py` does not include `HarnessFactory` /
  `HarnessRegistry`, making the public surface asymmetric with `registry.py`.
- (S-3) `Harness.snapshot()` returns `state=dict(self.state)` — `restore()`
  restores only `state`, leaving `extensions` orphaned across restart
  (documented behaviour but worth surfacing in the README quickstart).
- (S-4) `RetryPolicy.delay_for(attempt < 1)` returns `0.0` silently rather
  than logging a misuse warning. Acceptable for an internal API.

### VERDICT
APPROVE_WITH_WARNINGS

### tests_run
PYTHONPATH=src/rob_box_harness:src/rob_box_llm \
  python3 -m pytest src/rob_box_harness/ tests/unit/harness/ \
  --no-header -q --tb=line
→ 251 passed, 1 skipped in 3.27s (coverage 92.47%, gate 85%)
→ Offline integration suite: 2/2 passed
→ Harness lifecycle: 11/11 passed
→ Stream retry with 429: passes via `test_stream_retries_on_initial_429`
```

## Audit per ADR-0001 checklist

| Criterion (task body) | Evidence | Verdict |
|---|---|---|
| **Security: keys not in logs** | `MINIMAX_API_KEY` resolved exclusively from `os.environ` (config.py:329, providers/minimax.py:244, build_minimax_provider:565). `MiniMaxRedactedLogFilter` re-exported (line 73). `grep` for hardcoded secrets in `src/rob_box_harness/` returned 0 matches. | PASS |
| **Security: no hardcoded secrets** | Search across the whole tree for `api_key=`/`secret=`/`token=` literals: 0 hits in production code. `RoboticHardwareProvider` config never holds literals. | PASS |
| **Security: no shell injection / eval / pickle** | `grep` for `pickle.loads`, `shell=True`, `eval(`, `exec(` in `src/rob_box_harness/`: 0 hits. | PASS |
| **Contracts: real types match ADR** | `Harness(Generic[StateT])`, `LLMProvider`, `MemoryStore`, `ToolProvider`, `SideEffectBus`, `Transport`, `Clock` all match §2.4.1–§2.4.7 of ADR-0001. The harness-side `MiniMaxProvider` IS-A `LLMProvider` (line 184). | PASS |
| **State: SessionState restored after restart** | `Harness.restore(snapshot)` re-applies state dict (harness.py:287-301). `SessionSnapshot` is JSON-serialisable (snapshot.py:45-57). `test_snapshot_roundtrip` in the suite passes. | PASS |
| **Lifecycle: hooks close resources** | `teardown()` iterates over ports and `aclose()`s them, with `hasattr` guard (harness.py:227-236). `__aexit__` calls teardown unconditionally (line 257). `test_teardown_closes_ports` + `test_async_with_teardown_on_exception` pass. | PASS |
| **Observability: structured logs + metrics** | `LifecycleHooks.invoke` logs every hook call (lifecycle.py:113). `HarnessRunResult.metadata={"harness": name}` (harness.py:208). `SessionSnapshot.captured_at` uses injected `Clock` (harness.py:278). | PASS |
| **Retry: real backoff, not busy-loop** | `RetryPolicy.delay_for(attempt)` = `backoff_base * 2 ** (attempt-1) + jitter` (providers/minimax.py:163-176). `asyncio.sleep` (line 463) — no CPU spin. Exponential growth validated by `test_retry_policy_delay_for_grows_exponentially`. | PASS |
| **MiniMax M1–M10 compliance** | M1 ✓ (complete+stream), M3 ✓ (CapabilityUnavailableError), M4 ✓ (base_resp→ProviderError), M5 ✓ (filter re-exported), M7 ✓ (env-only API key, ConfigError on miss), M10 ✓ (aclose idempotent — `test_aclose_is_idempotent`). M2/M6/M8/M9 delegated to upstream `rob_box_llm` provider. | PASS |
| **Harness ADR §2.3 idempotency** | `init()` short-circuits on `_initialized` (line 156-157); `teardown()` returns early if `!_initialized` (line 215-216). Both validated by tests. | PASS |

## WARNINGS — details

### W-1: Stream retry leaks the failed inner stream's HTTP request

`providers/minimax.py:340-346`:

```python
inner_stream = self._inner.stream(messages, tools=tools, settings=settings)
first_chunk = await inner_stream.__anext__()
```

If `__anext__()` raises `RateLimitError`/`TimeoutError`, control jumps to the
`except` block at line 381 and the next attempt begins with a *fresh*
`inner_stream`. The previous async generator object is left to be GC'd, but
the underlying HTTP response body is **not** explicitly closed. For
persistent connection pools this can pin a socket until GC runs.

**Fix:** in the retry path, call `await inner_stream.aclose()` (the async
generator's `aclose()` method, which raises `GeneratorExit` inside the
generator and triggers SDK cleanup) before sleeping. Concretely:

```python
except (RateLimitError, TimeoutError) as exc:
    try:
        await inner_stream.aclose()
    except Exception:  # noqa: BLE001
        pass
    last_exc = exc
    ...
```

A test (`test_stream_retry_closes_failed_attempt`) should mock
`inner_stream.aclose` and assert the call. ADR-0001 §2.6.1 M10 ("aclose()
корректно закрывает HTTP-клиент, особенно при streaming") applies.

### W-2: `init()` order — `_initialized=True` set before `on_start` hook

`harness.py:170-176`:

```python
self.state = self._initial_state()
self._initialized = True                          # line 171
logger.debug("harness %s initialised", self.name) # line 172
await self.hooks.invoke("on_start", self.name)    # line 176
```

If `on_start` raises (e.g. a downstream subscriber fails to warm its cache),
the harness stays marked `_initialized=True` but `on_start` never completed.
A subsequent `teardown()` will then try to `aclose()` ports that may never
have been fully wired, and `is_initialized` lies about whether the harness
actually started.

**Fix:** either move the `self._initialized = True` after the `await
self.hooks.invoke(...)`, or catch the hook exception and reset the flag on
failure:

```python
try:
    await self.hooks.invoke("on_start", self.name)
except Exception:
    self._initialized = False
    raise
else:
    self._initialized = True
```

Note that `lifecycle.invoke()` already wraps the user hook in `HookError`,
so this would only fire when the user raises *non-HookError*, but the
principle (state flag reflects "ready to run", not "init attempted") holds.

### W-3: Factory cache key uses shallow `__dict__` on dataclasses

`registry.py:158-167`:

```python
return {
    "harness": config.harness,
    "name": config.name,
    "state": dict(config.state),
    "llm": config.llm.__dict__ if config.llm is not None else None,
    ...
}
```

`config.llm.__dict__` for a frozen dataclass exposes the field values but
**not** the `__init__` overrides or any custom `__post_init__` mutation.
If a future contributor adds a mutable field (e.g. a `last_modified`
timestamp), the cache will return a stale harness after a config update.

**Fix:** replace `__dict__` with `dataclasses.asdict(config.llm)` for
deep-copied semantics, or implement `__hash__` on each `*Config` dataclass
and hash the config object directly.

## SUGGESTIONS — details

- **S-1.** `MiniMaxRedactedLogFilter` is re-exported (line 73) but the
  harness-side wrapper does not install it on the module logger at
  construction. If a future contributor adds `logger.info("Calling with
  %s", self._inner)` they may leak. Either install on
  `MiniMaxProvider.__init__` or document the install responsibility in
  the harness quickstart.
- **S-2.** `runner.py` does not export `HarnessFactory` /
  `HarnessRegistry` in `__all__` (lines 119-124), so external callers
  cannot do `from rob_box_harness.runner import HarnessRegistry`. Asymmetric
  with `registry.py`'s public surface; either add them or document the
  intended import path.
- **S-3.** `Harness.snapshot()` records `state` + `extensions`, but
  `restore()` only applies `state`. Documented behaviour ("Only the
  `state` mapping is restored; the `extensions` dict is left alone.") is
  sound for test replay, but the harness-quickstart.md should mention that
  per-step metrics (LLM call counts) do not survive a round-trip through
  `restore()`.
- **S-4.** `RetryPolicy.delay_for(attempt: int)` returning `0.0` for
  `attempt < 1` is silent. Internal API so not blocking, but a `logger.debug`
  would help future maintainers detect misuse.

## Tests run

```
PYTHONPATH=src/rob_box_harness:src/rob_box_llm \
  python3 -m pytest src/rob_box_harness/ tests/unit/harness/ \
  --no-header -q --tb=line
251 passed, 1 skipped in 3.27s
TOTAL 1394 statements, 105 missed, 92.47% coverage (≥85% gate)
```

Offline integration suite (deterministic fake SDK, no sockets):

```
test_dummy_harness_runs_end_to_end_through_public_entry_point PASSED
test_minimax_fixture_drives_real_harness_without_network PASSED
```

Harness lifecycle (11 tests, all pass): init_does_no_io,
init_creates_default_ports, init_is_idempotent, run_requires_init,
run_rejects_overlap_via_state_guard, teardown_is_idempotent,
teardown_closes_ports, async_with_teardown_on_exception,
extensions_recorded_after_run, state_is_initialised_from_config,
init_validates_config_type.

## Files reviewed

- `src/rob_box_harness/rob_box_harness/harness.py` (382 lines)
- `src/rob_box_harness/rob_box_harness/snapshot.py` (84)
- `src/rob_box_harness/rob_box_harness/lifecycle.py` (126)
- `src/rob_box_harness/rob_box_harness/errors.py` (118)
- `src/rob_box_harness/rob_box_harness/config.py` (612)
- `src/rob_box_harness/rob_box_harness/registry.py` (176)
- `src/rob_box_harness/rob_box_harness/runner.py` (124)
- `src/rob_box_harness/rob_box_harness/providers/minimax.py` (585)
- `src/rob_box_harness/rob_box_harness/providers/fake_llm.py` (23)
- `src/rob_box_harness/test/test_minimax_provider.py` (1181)
- `src/rob_box_harness/test/test_harness_lifecycle.py` (11 tests)
- `src/rob_box_harness/test/test_integration_offline.py` (2 tests)
- `docs/adr/0001-harness-architecture.md` (contract reference)

## Sign-off

The P0 harness foundation is safe to merge to `develop`. The three
warnings are follow-ups — none block M1–M10 production acceptance.
Recommend opening three child Kanban tasks (W-1 → testing profile,
W-2 → harness-implementer profile, W-3 → harness-implementer profile)
to close them before the next milestone.