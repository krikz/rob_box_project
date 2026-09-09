# Issue #1560 — develop build spam diagnosis

**Reporter:** devops worker (t_9c5a0b3b), 23.08.2026
**Severity:** HIGH
**Related:** #1535 (race-condition dedup), #1551 (e2e-process scenario_file fallback), #1548 (SHA-tag whitelist), PR #1536 (post-merge-build), PR #1540 (pre-dispatch dedup shared lib)

## Symptom recap (from issue body)

`L: Build All Services` runs on `develop` branch are being triggered **30+ times/day** at irregular intervals (~15-25 min). Every observed run has:
- `event: workflow_dispatch`
- `triggering_actor.login: krikz`  (i.e. the GH token holder — not necessarily a human)
- `head_branch: develop`
- `inputs: null` / `{}` (empty — no `triggered_by_script` etc.)
- `run_attempt: 1`

Last 5 runs sampled via `gh api repos/.../actions/runs/<id>`:
- `#32655410890`  2026-08-23T17:37:06Z  head_sha=8fd0a1aa
- `#32647505673`  2026-08-23T15:06:26Z  head_sha=75dced51
- `#32646609653`  2026-08-23T14:49:25Z  head_sha=bff17d81
- `#32645788428`  2026-08-23T14:33:21Z  head_sha=0e421ff6
- `#32643360571`  2026-08-23T13:46:13Z  head_sha=e709798f

Pattern: ~15-25 min apart, each on a different (older) `head_sha` (not the latest develop HEAD).

## Investigation checklist (5 paths from issue body)

### 1. `grep -r 'gh workflow run.*L-Build-All-Services' /home/builder/`

Searched host-side. **No host script currently invokes `gh workflow run L-Build-All-Services` for develop** (besides the two legitimate paths below). Noise in `/home/builder/.hermes/kanban/boards/robbox/logs/*` (chatSession JSONL), `.worktrees/*/docs/` (docs/README), `.config/Code/User/History/*` (VS Code history) — none of these are live scripts.

**Legitimate callers found (verified by reading source):**
- `agent-flow-post-merge-build.sh` — passes `--field triggered_by_script=agent-flow-post-merge-build --field triggered_by_agent=merge-gate` and has pre-dispatch dedup via `verify_recent_run ... 900` (15 min RECENT_WINDOW). Source: PR #1536 + #1540. Runs are filtered by `RECENT_WINDOW=900` (POST_MERGE_BUILD_RECENT_WINDOW).
- `agent-flow-e2e-process.sh` `_trigger_workflow_with_retry()` — passes `--field triggered_by_script=agent-flow-e2e-process` and has its own pre-dispatch dedup. Operates ONLY on `ROUND_BRANCH=z-{e2e}/test-round-N` (not develop) for builds triggered by `issue_round_pipeline` in `_round_head_sha_*` paths.

### 2. `agent-flow-merge-gate.sh` + `agent-flow-e2e-process.sh` retry loops

Reviewed both. e2e-process has `consecutive_build_failed >= 2 → pre-dispatch guard` (PR #1536) and `existing_build_for_HEAD → skip build trigger` (resume-on-restart). Neither path triggers builds on develop branch — e2e builds run only on round-branches.

merge-gate calls `agent-flow-post-merge-build.sh` on **each tick** (every 5 min, architect profile cron `1082e70dc68f`, completed=2514) when `pr_state == MERGED && pr_base == $DEVELOP_BRANCH`. The script has pre-dispatch dedup `verify_recent_run ... $RECENT_WINDOW (900s)` — so if there's a recent build on develop it short-circuits with `⏭️ recent build (≤900s) already on develop — skip`. This means **merge-gate backup-trigger is NOT a spam source by itself**, but it does NOT add `triggered_by_script` to the run inputs because the **dedup short-circuits before the `gh workflow run` call**.

**Important nuance**: the dedup uses `RECENT_WINDOW=900` (15 min) — and spam runs come every 15-25 min. **The 15 min dedup window aligns perfectly with the spam cadence**. This means: if the spam is from source X and source X runs every 15-25 min, merge-gate's 15 min window catches ~half of them but lets through the second half. That's not the **source** of spam, but it does **mask the source** from the merge-gate backup path.

### 3. `crontab -l` + `/etc/cron.d/`

`crontab -l` empty. `/etc/cron.d/` only has `anacron` and `e2scrub_all`. No agent-flow/rob_box entries in user/system cron.

### 4. `ps -ef | grep agent-flow`

Found these processes alive (23.08 19:43):
- `hermes ... gateway run --profile agent-flow` (PID 486428, started 18.08 — normal long-running)
- `hermes ... gateway run --profile architect` (PID 1250439, started 23.08 19:33 — RESTARTED today — see #1553 whoami fix related)
- `hermes ... gateway run --profile devops` (PID 486501, 18.08)
- `hermes ... gateway run --profile pm` (PID 486565, 18.08)
- `agent-flow-triage.sh` (PID 1275407, started 19:43 — every 1 min cron, completed=10480)
- `agent-flow-merge-gate.sh` (PID 1255515, started 19:36, parent = architect gateway 1250439) — has `failure_streak: 22` per jobs.json (last error: `gh auth not configured — exit 1`)
- `agent-flow-e2e-process.sh` (PID 1270864, started 19:42)

**Architect gateway was just restarted at 19:33** — it had crashed multiple times between 19:06 and 19:33 (every ~3 min). The `merge-gate.sh` under it has been failing (gh auth not configured) for the last 22 ticks. **None of these are triggering the spam builds** (the only `gh workflow run` paths in agent-flow-{merge-gate,e2e-process}.sh pass `triggered_by_script` which would show up in `inputs`).

### 5. `~/.hermes/profiles/*/logs/`

Architect gateway log (`agent.log`) shows Шифу's complaints on 22.08:
- `2026-08-22 17:48:27` "А нафига он два раза подряд запустил сборку?"
- `2026-08-22 18:01:35` "Какая-то херня по два запуска сборки и второй падает"
- `2026-08-22 20:43:16` "чот по прежнему пара запускается L: Build All Services"
- `2026-08-22 21:47:33` "возьми свежак, глянь чего опять дублятся L: Build All Services"
- `2026-08-22 23:28:19` reply "✅ ТОЛЬКО удалён L-Build-On-Branch-Push.yml"

The cron-надзор `5c96a6eedf93` (every 60m, last successful run 16:56) reported the same observation:
> "25 develop-прогонов (`L: Build All Services`) — 94 SHA-tag коммита из 94 в develop за 24ч, последний success round-212 в 00:12, дальше тишина"
> "#3. SHA-tag auto-merge продолжает спамить develop — ... **генерация не остановлена**"

The cron-надзор **observed** the spam but did not fix it. It has been FAIL-ing since 17:57 (MiniMax API key 401).

## What was ELIMINATED

- ❌ Push-trigger (`L-Build-On-Branch-Push.yml`) — DELETED in commit `dcd0f801` (PR #1536) on 22.08. Verified: file no longer in `origin/develop`'s `.github/workflows/` tree. **Not a source**.
- ❌ `L-Build All & Push to GHCR.yml` schedule — has `cron: '0 0 * * *'` (daily midnight UTC). **Not a source** (cadence mismatch).
- ❌ agent-flow-merge-gate.sh direct call — passes `triggered_by_script` (would show in `inputs`).
- ❌ agent-flow-e2e-process.sh — operates on round-branches only.
- ❌ host crontab / /etc/cron.d — empty.
- ❌ systemd timer — only `wifi-ros2-watchdog.timer` (every 30s, unrelated).

## What remains as a hypothesis (UNVERIFIED — needs e2e or live test)

**Hypothesis A (most likely)**: The spam runs are **manual `gh workflow run "L: Build All Services" --ref develop` calls from a non-cron source** — possibly a leftover/looped `tmux`/`nohup` session, a manual operator (Шифу or collaborator GOODWORKRINKZ) clicking "Run workflow" in GitHub UI when they see stale SHA tags, or a script that lost its `triggered_by_script` instrumentation after a token rotation.

**Hypothesis B**: There is a `repository_dispatch` webhook somewhere that re-dispatches via `gh api repos/.../dispatches` and someone subscribed. Not searched — `gh api repos/.../hooks` would tell. Outside scope of devops worker.

**Hypothesis C (least likely)**: GitHub Actions itself is dispatching `workflow_dispatch` events to the workflow. Impossible — `workflow_dispatch` only fires from explicit API calls.

## Why pre-dispatch dedup (`verify_recent_run ... 900`) does NOT solve this

The dedup in `agent-flow-post-merge-build.sh` only kicks in **for the script's own `gh workflow run` call**. If the spam source is **outside** the script (manual UI click, orphan loop, leak from another script), the dedup has no effect — it can't see runs it didn't trigger.

## Proposed fix (this PR)

**Server-side guard** in `.github/workflows/L-Build-All-Services.yml`:

1. **`concurrency:` group** — `group: l-build-all-services-${{ github.ref }}` + `cancel-in-progress: false`. This deduplicates at GitHub-side: if a new `workflow_dispatch` comes in within the group's window, GitHub **queues** it (or skips if `cancel-in-progress: true`). With `cancel-in-progress: false` + a small time window, this prevents the parallel-runs anti-pattern (issue #1535 root cause) regardless of the source.
2. **Stricter input validation** — `workflow_dispatch` requires `triggered_by_script` for builds on `develop`/`main`. UI/manual calls without the field get a job that **exits early** with a clear warning log line. This makes the spam source **self-identifying** in CI logs without breaking anything.

The fix is **source-agnostic**: it does not depend on identifying the spam source. It works whether the source is UI, leaked script, or future unknown.

## Verification plan

After this PR merges:
1. Next spam attempt within 1h → expect `concurrency` group to queue/cancel; visible in GitHub Actions UI.
2. Manual `gh workflow run L-Build-All-Services.yml --ref develop` without `-f` → expect "skip: triggered_by_script required" log line in CI job. **This is the key diagnostic step**: if the spam continues AND the log line is NOT present, the source is using `-f triggered_by_script=...` (i.e. it's still going through one of our scripts, but that script's dedup is broken). If the log line IS present, the source is manual UI / orphan loop / leak from non-script path.
3. Re-run `gh run list --workflow "L: Build All Services" --limit 30` after 1h — count of new runs on develop should drop sharply.

## Files touched by this PR

- `.github/workflows/L-Build-All-Services.yml` — add `concurrency:` block + early-exit job for missing `triggered_by_script` on protected branches
- `docs/diagnostics/issue-1560-spam-runs-diagnosis.md` — this file
- `scripts/agent_flow/tests/test_l_build_concurrency.sh` — regression test for the workflow YAML changes

