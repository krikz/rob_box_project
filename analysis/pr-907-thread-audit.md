# PR #907 — Unbounded Daemon-Thread Spawn Audit (voice nodes)

**Scope:** `git diff 91abcbca..6e75eb39` (PR #907 base → head)
**Files audited:** `src/rob_box_voice/rob_box_voice/tts_node.py`, `src/rob_box_voice/rob_box_voice/dialogue_node.py`
**Patterns searched:** `threading.Thread(target=..., daemon=True)`, dynamic `Thread(...)` without a bounded executor, equivalent fan-out via `asyncio.run_coroutine_threadsafe` / `run_in_executor(None, ...)` / `ThreadPoolExecutor(max_workers=1)` ad-hoc wrappers.
**Result at PR head (`6e75eb39`, branch `pr-907`):** BLK-9 fix (`c0865e05`) is **landed** in the PR — the original unbounded thread-per-request in `tts_node.dialogue_callback` is replaced by a bounded `ThreadPoolExecutor` + `Semaphore`. One single-shot `daemon=True` thread remains in `dialogue_node.__init__` (asyncio-loop driver); it is bounded by node lifecycle (1 per process) and is **not** a fan-out. No other unbounded spawn sites found.

---

## Summary table

| # | Site | Status at PR head | Concurrency model | Workload | Bounded primitive in place |
|---|------|-------------------|-------------------|----------|----------------------------|
| 1 | `tts_node.dialogue_callback` (synth dispatch) — **originally** spawned `threading.Thread(target=_run_synthesis_worker, daemon=True).start()` per `/voice/dialogue/response` message | **Remediated by BLK-9** (`c0865e05`) — now `self._submit_synthesis(...)` → bounded `ThreadPoolExecutor` + `Semaphore` | per-message callback → bounded worker pool | I/O-bound + ALSA playback (TTS HTTP + audio device) | `concurrent.futures.ThreadPoolExecutor(max_workers=2, max_queue=16)` + `threading.Semaphore(18)` (defaults), gated by `threading.Lock` for the actual blocking work |
| 2 | `dialogue_node.__init__` — `threading.Thread(target=self._loop.run_forever, daemon=True).start()` | **Remains** — one-shot, lifecycle-bounded (1 thread per node, dies with process) | per-process long-running asyncio event loop driver | CPU/event-loop scheduler (never blocks on I/O itself) | Not needed — bounded by node lifecycle. Could be replaced with `asyncio.run()`-driven ROS executor integration, but that's an architectural refactor, not a security/perf fix |
| 3 | `dialogue_node._on_stt` / `_on_dj_tick_check` — `asyncio.run_coroutine_threadsafe(self._agent_run(...), self._loop)` | **Implicitly bounded** — `_on_stt` calls `self._cancel_run(...)` before scheduling; `_on_dj_tick_check` skips if `self._run_task` is still running | per-callback with single-flight semantics | I/O-bound (LLM HTTP, MCP tool calls) | Single-flight via `self._run_task` + `_task_lock`. No new threads spawned here — work is dispatched onto the existing event-loop thread (#2) |
| 4 | `dialogue_node` MCP tool wrappers (lines 649, 1111) — `loop.run_in_executor(None, lambda: mcp.execute_tool_call_sync(...))` | **Remains** — uses default executor (`None` → `ThreadPoolExecutor` with `os.cpu_count()+4` workers, process-global, shared with the entire event loop) | per-async-tool-call → default executor | I/O-bound (sync MCP tool call wrapped into async) | Default executor size is fine in practice (8–32 threads on typical boxes, plenty for I/O-bound MCP calls), but the executor is shared across the loop. For strict per-call admission control, switch to a module-private `ThreadPoolExecutor(max_workers=N)` and pass it explicitly to `run_in_executor` |

No other `threading.Thread(...)`, `Thread(...)`, or `ThreadPoolExecutor(...)` constructions were introduced by the PR diff in the two audited files.

---

## Detailed findings

### Finding 1 — `tts_node.dialogue_callback` synth dispatch (the original BLK-9 offender, now fixed)

**Original code (introduced by PR #907, replaced by BLK-9):**

```python
# tts_node.py — inside dialogue_callback (PR #907 original)
threading.Thread(
    target=self._run_synthesis_worker,
    args=(ssml, text, dialogue_id, ssml_attributes, speech_id),
    name=f"tts-{speech_id[:8]}",
    daemon=True,
).start()
```

**Trigger:** `dialogue_callback` is a ROS2 subscription callback on `/voice/dialogue/response` and `/voice/tts/request` (see `tts_node.py:350–355`). Every incoming message → one daemon thread.

**Concurrency model:** **unbounded** — one new OS thread per message, never awaited, never reaped until process exit.

**Workload:** `_run_synthesis_worker` runs `_synthesize_and_play`, which does:
- HTTP call to TTS provider (Yandex gRPC, Silero local, or MiniMax T2A v2) — **blocking, I/O-bound, external service**
- ALSA playback via `AudioPlaybackManager.play_audio(...)` — **blocking on audio device**
- Both can take hundreds of ms to several seconds each.

**Risk (BLK-9 root cause):** Under bursty `/voice/dialogue/response` traffic (e.g. STT flapping, re-deliveries, or a buggy upstream publisher), the node spawns an unbounded number of zombie threads that all queue on `_synthesis_lock` and on the ALSA device. Symptom: RSS growth, eventual `pthread_create` failure / `Resource temporarily unavailable`, and ROS2 callback executor starvation (control/new-dialogue callbacks get queued behind the synth callback and stop responding to STOP / barge-in). OWASP A04:2021 (Unrestricted Resource Consumption).

**Remediation (BLK-9, now in the PR):**

```python
# tts_node.py:413–417 — bounded pool + semaphore slot cap
max_workers  = max(1, min(4, int(self.get_parameter("synthesis_max_workers").value)))  # default 2
max_queue    = max(1, int(self.get_parameter("synthesis_max_queue").value)))           # default 16
self._synthesis_slots    = threading.Semaphore(max_queue + max_workers)
self._synthesis_executor = concurrent.futures.ThreadPoolExecutor(
    max_workers=max_workers, thread_name_prefix="tts-synth",
)
```

```python
# tts_node.py:756–798 — _submit_synthesis (admission control)
def _submit_synthesis(self, fn, speech_id, *args):
    if self._synthesis_executor_shutdown:
        ...return  # refuse post-shutdown
    if not self._synthesis_slots.acquire(blocking=False):   # non-blocking slot check
        self.get_logger().warning(
            f"⚠️ TTS synth pool full (speech_id={speech_id[:8]}) — dropping"
        )
        return  # drop on overflow instead of queueing forever
    self._synthesis_in_flight += 1
    try:
        future = self._synthesis_executor.submit(fn, *args)
    except RuntimeError as exc:
        self._synthesis_slots.release()
        self._synthesis_in_flight -= 1
        ...return
    future.add_done_callback(self._on_synthesis_done)   # releases slot on success/failure
```

```python
# tts_node.py:820–841 — worker body, gated by per-node lock
def _run_synthesis_worker(self, ssml, text, dialogue_id=None, ssml_attributes=None, speech_id=None):
    with self._synthesis_lock:                            # serializes HTTP+ALSA work
        if dialogue_id and self.current_dialogue_id != dialogue_id:
            return                                        # stale dialogue → drop
        self._synthesize_and_play(ssml, text, dialogue_id, ssml_attributes, speech_id)
```

**Verdict:** ✅ **BLK-9 fix is correct and complete for this site.**
- Bounded thread count: `synthesis_max_workers` (1..4, default 2).
- Bounded in-flight work: `synthesis_max_queue + synthesis_max_workers` slots via `Semaphore` (default 18); overflow drops with warning.
- Stale-dialogue check inside the worker prevents redundant work after barge-in.
- Idempotent shutdown (`shutdown_synthesis_executor` at `tts_node.py:1621`).
- Lock-serialized body prevents two workers from racing the ALSA device.

**Suggested follow-up (non-blocking, Suggestion tier):**
- Expose `synthesis_in_flight` as a ROS topic for diagnostics (already kept as a counter at `tts_node.py:421`, just not published).
- The internal `ThreadPoolExecutor` queue is unbounded when slots are available; the `Semaphore` is the actual back-pressure. That's intentional but worth a comment in the code — it already exists (`tts_node.py:400–402`), good.

---

### Finding 2 — `dialogue_node.__init__` asyncio loop driver (one-shot daemon thread)

**Code (`dialogue_node.py:151–153`):**

```python
# ── asyncio loop in daemon thread ────────────────────────────
self._loop = asyncio.new_event_loop()
threading.Thread(target=self._loop.run_forever, daemon=True).start()
```

**Trigger:** Node construction (once per process lifetime). The thread runs `loop.run_forever()` until the loop is stopped at shutdown.

**Concurrency model:** **one thread, per-process, lifecycle-bounded.** ROS2 callbacks post work into this loop via `asyncio.run_coroutine_threadsafe(coro, self._loop)` (see Finding 3). The thread is daemonic so the process can exit even if the loop is still running, which is the intended ROS2-shutdown behavior.

**Workload:** Pure event-loop scheduler — runs callbacks, I/O multiplexing, scheduled coroutines. Not CPU-bound, not blocking on external services (those run inside coroutines scheduled onto this loop, e.g. via `run_in_executor`, see Finding 4).

**Risk:** None as written. The thread is created exactly once in `__init__`, never recreated. Even if `__init__` were somehow called twice on the same node (it shouldn't be), it would still be bounded by the number of node instances (typically 1).

**Verification it predates PR #907:** `git merge-base --is-ancestor 8fe6217c 91abcbca` returns true — the `asyncio loop in daemon thread` block was introduced by `feat(voice): rewrite dialogue_node on OpenAI Agents SDK`, which is on the PR base (`origin/main` lineage), not added by this PR. PR #907 inherits it via the rewrite merge.

**Recommended primitive:** None. The pattern is the canonical "asyncio bridge from a sync ROS2 node" idiom. Replacing it with `asyncio.run()` would require driving the ROS2 executor from inside the asyncio loop (architectural change, out of scope for this PR).

**Optional improvement (Suggestion tier):**
- Wrap the thread spawn in a try/except so `__init__` failure leaves the node in a clean state if `new_event_loop()` ever raises.
- Consider `loop.run_in_executor(...)` inside ROS callbacks vs. `run_coroutine_threadsafe` — both are fine, but mixing them inconsistently (this file uses both) is mildly confusing. Not a blocker.

---

### Finding 3 — `dialogue_node._on_stt` / `_on_dj_tick_check` → `run_coroutine_threadsafe`

**Code (multiple sites in `dialogue_node.py`):**

```python
# dialogue_node.py:1530 — _on_stt (per-user-utterance)
asyncio.run_coroutine_threadsafe(self._agent_run(clean), self._loop)

# dialogue_node.py:2118 — _on_dj_tick_check (periodic timer)
asyncio.run_coroutine_threadsafe(self._agent_run(prompt), self._loop)
```

**Trigger:**
- `_on_stt` — ROS2 subscription on `/voice/stt/text` (gated by wake-word detection). One call per user utterance.
- `_on_dj_tick_check` — `create_timer(5.0, ...)` (every-5s ROS2 timer; only acts when DJ mode is enabled and `time.time() >= self._dj_next_transition_at`).

**Concurrency model:** **single-flight, bounded by design.**
- `_on_stt` calls `self._cancel_run("new STT input")` (`dialogue_node.py:1793`) **before** scheduling. `_cancel_run` cancels `_run_task` and waits on its stop event, so at most one `_agent_run` is in-flight at a time.
- `_on_dj_tick_check` re-schedules itself by `+15s` if `_run_task is not None and not _run_task.done()` (`dialogue_node.py:2084`). So at most one DJ auto-transition in-flight at a time.

**Workload:** `_agent_run` is an async coroutine driving the OpenAI Agents SDK loop. It is **I/O-bound** (LLM HTTP, MCP tool calls) — coroutines, not threads. No new threads are spawned here; work runs on the asyncio loop from Finding 2.

**Risk:** Negligible. Burst input is throttled by the single-flight gate. `_agent_run` itself can spawn tool calls (e.g. `speak_text`), which schedule more coroutines on the same loop — still bounded by the loop's single-thread execution.

**Recommended primitive:** None. `run_coroutine_threadsafe` onto a long-lived loop is the right primitive. The single-flight gate (`_cancel_run` / `running` check) is the correct admission control for this workload.

**Optional improvement (Suggestion tier):**
- The DJ tick check uses an **implicit** guard (`running` flag) while the STT path uses an **explicit** cancel (`_cancel_run`). Unifying these — e.g. always calling `_cancel_run` before `run_coroutine_threadsafe` — would make the model more obvious and protect against a future caller that forgets the check.
- Expose `_run_task` lifecycle (created / done / cancelled) as a ROS topic for diagnostics.

---

### Finding 4 — `dialogue_node` MCP tool wrappers → `run_in_executor(None, ...)`

**Code (`dialogue_node.py:649` and `:1111`):**

```python
async def _call(tool_name: str, params: dict, timeout: float = 10.0) -> str:
    self._tools_called.append(tool_name)
    result = await asyncio.get_running_loop().run_in_executor(
        None,                                              # ← default executor
        lambda: mcp.execute_tool_call_sync(tool_name, params, timeout=timeout),
    )
    ...
```

**Trigger:** Inside `@function_tool async def _call(...)` wrappers used by `speak_text` and other agent tools. Called once per tool invocation, which itself is gated by `_agent_run` (Finding 3 → single-flight).

**Concurrency model:** **shared default executor.** `run_in_executor(None, ...)` uses the loop's default `ThreadPoolExecutor`, lazily created with `max_workers = min(32, os.cpu_count() + 4)` (asyncio docs). This is **process-global** within the asyncio loop — shared across every coroutine on the same loop.

**Workload:** **I/O-bound / blocking on external service** — `mcp.execute_tool_call_sync` blocks on the MCP subprocess / socket / whatever MCP backend is wired in.

**Risk:** Low. Default executor size is 5–32 workers depending on CPU count, which is more than enough for single-flight agent tools. The shared pool is a feature, not a bug, for I/O-bound MCP calls. The only failure mode would be if a tool accidentally became CPU-bound (e.g. heavy in-process computation), which would let it hog a default-executor worker and starve other coroutines — but MCP tools are by design out-of-process.

**Verification in PR #907 diff:** Both sites are introduced by PR #907 (in the `speak_text` / agent-tools block added by the OpenAI Agents SDK rewrite). They are **not** part of BLK-9.

**Recommended primitive:** Keep `run_in_executor(None, ...)` here. It is the correct primitive for "wrap a sync I/O call inside an async function". Switching to a private `ThreadPoolExecutor` would just trade the default pool for an equally-sized pool with worse diagnostics.

**Optional improvement (Suggestion tier):**
- The `_call` wrapper does **not** apply a `try/except` around `mcp.execute_tool_call_sync`. If the MCP call raises (timeout, broken pipe), the exception propagates into the agent loop and may abort `_agent_run`. A `try/except` that returns a synthetic error string (so the LLM sees "tool X failed: …" and can recover) would be more robust. Not a thread-spawn issue, but related to "blocking on external service" hygiene.

---

## Items NOT changed by BLK-9 (and why they're acceptable)

| Site | File:Line | Why it didn't need fixing |
|------|-----------|---------------------------|
| `threading.Thread(target=self._loop.run_forever, daemon=True).start()` | `dialogue_node.py:153` | One-shot, per-process, lifecycle-bounded. Not a fan-out. Predates PR #907 (inherited from the OpenAI Agents SDK rewrite). |
| `asyncio.run_coroutine_threadsafe(...)` in `_on_stt` / `_on_dj_tick_check` | `dialogue_node.py:1530, 2118` | Single-flight gated by `_cancel_run` / `_run_task.done()` check. Work runs on the existing loop, not in a new thread. |
| `loop.run_in_executor(None, ...)` in MCP tool wrappers | `dialogue_node.py:649, 1111` | Default executor is sized for I/O-bound work; tool calls are gated by single-flight `_agent_run`. |

---

## Verification commands

The following reproduce this audit on the PR head (`pr-907 = 6e75eb39`):

```bash
# All Thread(...) constructions introduced by PR #907 in the two target files:
git diff 91abcbca..6e75eb39 -- src/rob_box_voice/rob_box_voice/tts_node.py \
                              src/rob_box_voice/rob_box_voice/dialogue_node.py \
  | grep -nE '^\+.*threading\.Thread|^\+.*daemon=True|^\+.*ThreadPoolExecutor|^\+.*run_in_executor|^\+.*run_coroutine_threadsafe'

# At PR head, no live threading.Thread(daemon=True) outside comments in the two files:
grep -nE 'threading\.Thread\(' src/rob_box_voice/rob_box_voice/tts_node.py \
                                 src/rob_box_voice/rob_box_voice/dialogue_node.py
# → only BLK-9 explanatory comments remain; no live spawn sites.
```

---

## Verdict

**BLK-9 (unbounded daemon thread fan-out in `tts_node.dialogue_callback`) is correctly remediated in PR #907** (`c0865e05 fix(voice): bound synthesis thread fan-out in tts_node (BLK-9, PR #907)`). The fix uses the right primitive (`ThreadPoolExecutor` + `Semaphore`-based slot cap) with appropriate defaults (`max_workers=2`, `max_queue=16`, total in-flight cap = 18), non-blocking admission control (drop with warning on overflow), and idempotent shutdown.

The two other concurrency primitives in scope (`asyncio.run_coroutine_threadsafe` in `dialogue_node`, and `loop.run_in_executor(None, ...)` for MCP tools) are not unbounded thread spawns — they dispatch onto already-existing infrastructure and are correctly single-flight or bounded by the default executor's sizing.

**No new CRITICAL/WARNING findings.** One optional Suggestion per Finding 4 (wrap MCP tool calls in try/except) — not blocking.
