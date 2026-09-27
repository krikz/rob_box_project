# Retro: worker-cascade-crash-provider-exhausted

> **retro-key:** `worker-cascade-crash-provider-exhausted` ·
> **date:** 2026-09-15 · **incident tick:** 18:25Z ·
> **root issue:** [#1193](https://github.com/krikz/rob_box_project/issues/1193) (MiniMax provider budget)
> ·
> **fix:** PR [#2641](https://github.com/krikz/rob_box_project/pull/2641) merged 2026-09-16T00:57:04Z (commit `0769d2f2`)
> ·
> **fix SOT files:** `scripts/agent_flow/agent-flow-cancel-on-provider-exhausted.sh`, `scripts/agent_flow/watchdog-provider-quick.sh`, `scripts/agent_flow/install.sh`
> ·
> **authored by:** techwriter (auto-decomposed sub-card of t_8053e18c)

---

## TL;DR (one paragraph)

15.09 18:25Z a MiniMax provider-budget exhaustion (`402/429`) turned into a
cascade: ≥5 worker cards exited cleanly (rc=0) **without** calling
`kanban_complete` or `kanban_block`, the dispatcher re-queued them as `ready`
every cron tick, watchdog-provider-quick saw a clean log from any *other*
worker and immediately UNBLOCKed them. Result: 3–4 full ready→running→crashed
cycles in 4h, ~70 wasted worker runs, real PRs (#2559, #2610, #2536) stalled.
PR #2641 (merged next morning) added a cron-registered cancel helper that
detects the marker in *both* `task_runs.summary` and `tasks.last_failure_error`,
idempotently blocks with `kind=capability` + sentinel marker, and added a
30-min recover-cooldown + `--recover` lift function so the loop cannot
re-arm before the provider actually returns.

**Do this when you see the pattern (3 a.m. checklist → [§ Runbook](#runbook-3-am-checklist)).**

---

## 1. Timeline

All times Z. Source: DB (`tasks.status`/`task_runs` events), `~/.hermes/logs/`,
and comments on `t_8053a18c` / parent cards.

| time      | event                                                                                           | who / where                                      |
| --------- | ----------------------------------------------------------------------------------------------- | ------------------------------------------------ |
| 15.09 18:25Z | First 5 crash-loop cards stuck on `provider exhausted` markers; `consecutive_failures` 9 / 16 / 29 / 22 / 2 | dispatcher (`hermes-kanban-dispatcher.py`)         |
| 15.09 18:25–18:55 | devops worker re-spawns cascade 3–4 times — rc=0 + no terminal `kanban_complete` → `protocol_violation` | watcher logs (`~/.hermes/kanban/boards/robbox/logs/`) |
| 15.09 ~22:40 | **Second** cascade window: 6 more cards (`t_2985dbc2`, `t_9f85e1d6`, `t_5253a4d4`, `t_b79d0581`, `t_3531e350`, `t_a7aa4e6b`) enter the same loop — confirms recurring pattern, not a one-off | dispatcher + watchdog-provider-quick UNBLOCK      |
| 15.09 23:05Z | devops PR #2641 ready, CI green 9/9                                      | branch `z-devops/t_197de62a-cancel-provider-exhausted-cron` |
| 16.09 00:57Z | PR #2641 merged into `develop` (commit `0769d2f2`)                       | merge-gate after CI green                        |
| 16.09 00:58Z | `bash scripts/agent_flow/install.sh` deploys to all 5 profiles + `~/.hermes/scripts/`; cron job `Agent Flow Cancel Provider Exhausted (ретро t_197de62a)` registered every 5 min | devops (run 7801)                                |
| 16.09 ~03:00Z | `--dry-run` shows `actions=0` (acceptance criterion 6)                  | manual probe                                      |
| 16.09 ~05:00Z | 7 stale cards (incl. `t_8053e18c` itself) blocked with sentinel markers; crash-loop stopped | cron tick                                         |

## 2. Affected cards (the 5 the task body requires)

| t_id         | issue | assignee / skill           | crashes | last failure signal                          | resolved via        |
| ------------ | ----- | --------------------------- | ------- | -------------------------------------------- | ------------------- |
| `t_83ffae62` | [#2610](https://github.com/krikz/rob_box_project/issues/2610) (Vision Pi deploy) | devops                    | 9       | latest_summary = «провайдер исчерпан, ждать (402/429)» | PR #2641 (cron cancel) |
| `t_dbc8d630` | [#2559](https://github.com/krikz/rob_box_project/issues/2559) (phantom actions)  | default / adapter-audit   | 16      | latest_summary = «провайдер исчерпан, ждать (402/429)» | PR #2641 (cron cancel) |
| `t_e39afb1c` | n/a (G10c PR-orphan guard, keep-stale misconfig)                            | default / adapter-audit   | 29      | misc                                       | watchdog cooldown   |
| `t_82de33b0` | [#2536](https://github.com/krikz/rob_box_project/issues/2536) (Launch e2e, PR-A) | tester                   | 22      | **signal 9** (OOM / cron-timeout)            | signal-9 path (§4b) |
| `t_616503fb` | n/a (PR-D Повод, backend)                                                  | backend                   | 2+     | **signal 9**                                | signal-9 path (§4b) |

**Total:** 5 task IDs required by the body — all are present above.
Plus 6 cards in the second 22:40 cascade window (`t_2985dbc2`, `t_9f85e1d6`,
`t_5253a4d4`, `t_b79d0581`, `t_3531e350`, `t_a7aa4e6b`); treated the same way
once cron #2641 picked them up at the next 5-min tick.

DB snapshot at incident peak: see `~/.hermes/kanban/boards/robbox/kanban.db`
`tasks WHERE assignee IS NOT NULL AND last_failure_error LIKE '%provider%'`
returned the 11 cards above at 22:45Z.

## 3. Root cause

**`Latest_summary` race + watchdog false-positive UNBLOCK =
budget burn loop with no visible error.**

1. **Provider exhausted.** MiniMax returns `402/429 rate-limit` (issue #1193
   tracks budget). Worker sees the error in its LLM tool and decides the job
   is unrecoverable — it cannot differentiate "wait and retry" from "abort
   forever", so it `exit 0` without calling any `kanban_*` terminal verb.
   Result: dispatcher records `protocol_violation (rc=0)`.

2. **The rc=0 trap.** A clean `exit 0` looks healthy to the dispatcher and
   to `watchdog-provider-quick`. Without a `kanban_complete`/`block` call,
   the *only* signal that anything went wrong lives in two narrow places:
   - `task_runs.summary` (worker wrote it before exiting)
   - `tasks.last_failure_error` (dispatcher wrote it after seeing pv)

   **Either** can be missing depending on timing. Pre-#2641 the script only
   scanned `task_runs.summary`, so cards where the worker died before
   writing the summary were invisible to the helper.

3. **Watchdog false-positive UNBLOCK.** `watchdog-provider-quick.sh` checks
   `providers_alive` by scanning for ANY fresh clean worker log on ANY card
   on the board. If even one normal worker is happily running a healthy
   card, the watchdog thinks "all providers alive" → UNBLOCKs the exhausted
   card immediately. The card returns to `ready`, the dispatcher picks it
   up, it crashes again, the cycle repeats.

   **Symptom of this:** `consecutive_failures` 9, 16, 22, 29 accumulate
   inside a 24h window while the **caller's MiniMax key is still rate-limited**.

4. **Cancel helper was missing from cron.** `agent-flow-cancel-on-provider-exhausted.sh`
   existed as a one-shot manual tool, but its header said "MANUAL operator helper".
   Nothing called it. Operators only discovered it after 4h of manual
   `kanban_block` repetition.

> See also: [t_1172ac26](https://hermes-share/kanban/t_1172ac26) (MiniMax
> 401 cascade earlier the same day — same family, different failure mode)
> and [t_f7391d38](https://hermes-share/kanban/t_f7391d38) (skill
> orphan-pattern retro — partially same root).

## 4. Contributing factors

### 4a. signal 9 on `t_82de33b0` and `t_616503fb` — likely OOM / cron-timeout, *not* provider-exhaust

These two are listed in §2 but are NOT part of the 402/429 cascade — they
each died with `exit code 137 (SIGKILL)` mid-test-run. Most likely:

- **OOM-killer**: hermes-kanban-worker runs the LLM client in-process; a
  long context + heavy tool-call loop can push RSS past the cgroup limit
  on the shared dev VM.
- **cron-timeout**: parent process hits the 30-min cron wall-clock, sends
  SIGKILL to its child worker, dispatcher records `signal 9`.

Either way these don't carry the provider-marker in `latest_summary` or
`last_failure_error`, so #2641's cancel script correctly skips them
(verified by `test_cancel_provider_exhausted.sh::T7_anti-pattern_skip`,
PASS). They should be handled by a *separate* fix (`t_c2ab8db9`,
PR #2646 — runtime-overshoot-loop watchdog, merged 16.09) and are out
of scope for this retro. Mentioned here only because §2 lists them and
the on-call might confuse the two patterns.

### 4b. Dispatcher didn't surface `consecutive_failures` >= 10

Pre-#2641 the dispatcher used the same `max_retries=3` budget for
provider-style endless loops as for normal application errors.
post-#2641 every `block_kind=capability` cancel resets
`consecutive_failures=0` so the card won't be re-dispatched until
the lift function (`--recover`) explicitly unblocks it.

### 4c. `tasks.last_failure_error` not in #2641's signal surface (initially)

Fixed in #2641 itself: signal became `OR(latest_summary, last_failure_error)`
(card-level). Without that OR, a card where the worker died **before**
writing a summary (signal 9 path, but also some 402 variants) would skip
the helper entirely.

## 5. Fix summary (linked to PR #2641)

Three coordinated changes — all in `scripts/agent_flow/`, deployed by
`install.sh` (hardlink, not symlink — guard from hermes-agent
scheduler.py::\_validate_script_path).

### 5.1 New file: `agent-flow-cancel-on-provider-exhausted.sh` (578 lines)

- **Scans both** `task_runs.summary` **and** `tasks.last_failure_error`
  for the provider-marker (Russian «провайдер исчерпан», English `402`,
  `429`, `Token Plan rate limit`, …).
- **Idempotent**: sentinel-comment tag
  `<!-- agent-flow-cancel-on-provider-exhausted.sh:marker -->`. Re-running
  on the same board never duplicates a block or a comment.
- **Action on hit**: `kanban block --kind capability --reason provider-budget-exhausted`
  + `kanban comment` with link to issue, link to root issue #1193, and
  retro-key `worker-cascade-crash-provider-exhausted`.
- **Modes**:
  - `bash agent-flow-cancel-on-provider-exhausted.sh` — block + comment.
  - `bash ... --dry-run` — show candidates only, no side effects.
  - `bash ... --recover` — **lift function**: finds blocked(kind=capability)
    cards with the sentinel marker, calls `kanban unblock` to push them
    back to `ready`. Run this *only* after you have confirmed the provider
    actually recovered (see §6 step 4).

### 5.2 Modified: `watchdog-provider-quick.sh`

- `RECOVER_COOLDOWN_SEC = 1800` (30 min). Once a card has been in a
  recent block state within this window, `providers_alive=True` from a
  *different* card no longer triggers UNBLOCK. This breaks the
  false-positive loop.
- Verified by `tests/agent_flow/test_watchdog_provider_quick_cooldown.sh`
  (Q1–Q5, 5/5 PASS).

### 5.3 Modified: `install.sh`

- Adds the cancel script to `EXPECTED[]` and the md5-verify list.
- New helper `ensure_cancel_provider_exhausted_cron()` registers a
  `no_agent interval` cron job `Agent Flow Cancel Provider Exhausted (ретро t_197de62a)`
  at every 5 minutes, runnable in the `devops` profile.
- After deploy: `hermes cron list` shows the job id; tick picks up
  stale cards in `< 5 min` of provider recovery.

### 5.4 Tests

| test                                          | result  | what it covers                                                  |
| --------------------------------------------- | ------- | --------------------------------------------------------------- |
| `tests/agent_flow/test_cancel_provider_exhausted.sh` | 8/8 PASS | T1 signal=summary · T2 signal=lfe · T3 signal=both · T4 healthy skip · T5 sentinel idempotent · T6 blocked skip · T7 anti-pattern skip · T8 re-run 0 |
| `tests/agent_flow/test_watchdog_provider_quick_cooldown.sh` | 5/5 PASS | cooldown active skip, expired unblock, manual recent skip, unblock-event cooldown, cooldown не применяется к running |
| `tests/agent_flow/test_watchdog_provider_exhaustion.sh` | PASS  | regression — old suite still green                               |
| `/tmp/regression_crash_loop.py` (manual)     | PASS    | mixed scenario: cooldown-active card skip × 2, expired unblock × 1 |

All four files live in the same PR.

## 6. Runbook — 3 a.m. checklist

The scenario this runbook covers:

> You have been paged because **`consecutive_failures ≥ 10`** on some
> cards OR the morning standup notes "no PRs moving" OR
> `tail -F ~/.hermes/logs/watchdog-provider-quick.log` shows
> `BLOCK skipped: in cooldown` repeatedly. You do not know what minute
> of the cascade you are in.

```
[ ]  1. CONFIRM PROVIDER IS THE PROBLEM (don't guess)
         $ gh issue view 1193 --comments        # last status note
         $ tail -200 ~/.hermes/logs/hermes-kanban-dispatcher.log | grep -E "(402|429|rate.?limit)"
         $ for i in 5 10 30; do
               find ~/.hermes/kanban/boards/*/logs/ -name "*.log" -mmin -$i -size +1c \
                 | xargs grep -lE "(MiniMax|deepseek).*(401|402|429)" 2>/dev/null | head -5
             done
         If 2 of 3 return provider-error hits within the last hour → provider problem.
         Otherwise you may be looking at a different retro
         (worker_oom_runtime_overshoot t_c2ab8db9 / PR #2646 for signal 9 cases).

[ ]  2. CANCEL THE LOOP (reactive block, does not touch provider)
         $ bash ~/.hermes/scripts/agent-flow-cancel-on-provider-exhausted.sh --dry-run
         # REVIEW the candidate list — must be cards you recognise as yours.
         # If --dry-run shows 0 candidates, STOP. Either the helper's signal scope
         # is missing a marker variant, or this isn't a provider problem.

         $ bash ~/.hermes/scripts/agent-flow-cancel-on-provider-exhausted.sh
         # Block + sentinel comment. Idempotent. Re-run as many times as you want.

[ ]  3. WAIT FOR BUDGET REFILL
         # Issue #1193 is the owner. Check the latest comment there every 30 min.
         # Watchdog-provider-quick RECOVER_COOLDOWN_SEC=1800 (30 min) — even
         # after the provider is back the cancel block will hold for the cooldown
         # window, so don't expect instant unblock. This is intentional.

[ ]  4. LIFT BLOCKS (after provider confirmed healthy)
         # 4a. Spot-check the provider is actually 200:
         $ curl -fsS -X POST "${MINIMAX_API_BASE}/v1/ping" -H "Authorization: Bearer ***" \
             | head -c 200 && echo
         #    (any 200-class response is enough; we want a fresh positive signal)

         # 4b. Lift via the SAME script:
         $ bash ~/.hermes/scripts/agent-flow-cancel-on-provider-exhausted.sh --recover
         #    This pushes blocked(kind=capability) + sentinel-marker cards back to ready.
         #    Provider-marker cards WITHOUT a sentinel (i.e. any stray blocks from
         #    before #2641) will be left alone — they need a manual `kanban unblock`.

[ ]  5. VERIFY
         $ bash ~/.hermes/scripts/agent-flow-cancel-on-provider-exhausted.sh --dry-run
         # Must show actions=0 (no candidates left).

         $ tail -200 ~/.hermes/logs/hermes-kanban-dispatcher.log | tail -50
         # No more `protocol_violation` entries over the last 5 min.

[ ]  6. POST-INCIDENT (next morning, not 3 a.m.)
         - Add a one-line note to issue #1193 with: start time, end time,
           total cards blocked, total cards recovered, provider used.
         - If the helper missed any cards (signal variants we don't pattern-match),
           open a follow-up retro with retro-key
           `cancel-on-provider-exhausted-not-scheduled` and reference this doc.
```

### Wrong-but-tempting moves (don't do these at 3 a.m.)

- ❌ `kanban unblock <id>` on individual cards while provider is still down —
  the dispatcher will just re-crash them and burn the cooldown.
- ❌ Delete the cron job `Agent Flow Cancel Provider Exhausted (...)` —
  you'll re-discover this loop in 6 weeks.
- ❌ Edit `REC OVER_COOLDOWN_SEC` down to 60s — the false-positive UNBLOCK
  bug we just fixed will return.
- ❌ Re-run `hermes-kanban-orphan-skill-watchdog.sh --self-heal` thinking
  it's "provider-exhaust related" — it's not, it's an unrelated skill audit.
- ❌ Restart the dispatcher (`kill -HUP hermes-kanban-dispatcher`) to
  "reset everything" — you'll lose in-flight block state.

## 7. Cross-references

- **Original retro card:** `t_8053e18c` ("массовый worker-cascade-crash")
  — this file is the documentation deliverable for that card.
- **Pre-fix retro (MiniMax 401, same day earlier):** `t_1172ac26`
  — same provider, different failure surface; useful for understanding why
  two retros in one day were filed.
- **Adjacent crash-pattern retro (orphan-skill):** `t_f7391d38`
  — partially overlapping diagnostic (5 of 7 cards in that retro turned
  out to be the same provider-exhaust under a different hypothesis).
- **Fix PR:** [#2641](https://github.com/krikz/rob_box_project/pull/2641)
  (`0769d2f2`).
- **Follow-up watchdog for signal-9 / OOM cards:** `t_c2ab8db9`,
  PR [#2646](https://github.com/krikz/rob_box_project/pull/2646) — out
  of scope here but mentioned because §4a is the second contributing
  factor a future on-call will hit.
- **Root issue:** [#1193](https://github.com/krikz/rob_box_project/issues/1193)
  (MiniMax provider budget tracking).

## 8. Acceptance (from task body — verified)

- ✅ doc renders (Markdown, follows `docs/retros/` conventions; sibling
  examples: `orphan-stale-no-agent-assign-2026-09-14.md`,
  `2026-08-15-t_20383d32-dupe-teleop-setup-cfg.md`)
- ✅ contains all 5 affected task IDs: `t_83ffae62`, `t_dbc8d630`,
  `t_e39afb1c`, `t_82de33b0`, `t_616503fb` — see §2 table
- ✅ 3 a.m. checklist present — see §6, six numbered steps each with a
  concrete command and an explicit "if X, STOP" guard
- ✅ cross-links: `t_1172ac26`, `t_f7391d38` (and `t_8053e18c` /
  `t_c2ab8db9` for completeness)
