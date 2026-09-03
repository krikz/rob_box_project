# AV-11 Bring-Up Inventory — t_63ede363

Generated: 2026-08-28 (session run 2687).
Worker: devops.
Task scope: prepare branch + inventory hardware/config for AV-11 e2e bring-up
(Vision Pi + Main Pi, 3 containers: quest + supervisor + telegram).

## 1. Repo / branch

| Item | Value |
|---|---|
| Repo root | `/home/builder/rob_box_project` |
| Workspace | `/home/builder/rob_box_project/.worktrees/t_63ede363` |
| Working branch | `feature/av-11-avatar-mixed-e2e` |
| Base | `origin/feature/avatar` @ `c8e141ea` (AV-11 attempt-8) |
| Worktree HEAD | `c8e141ea` |
| Local `develop` | `7d37ccd8` (behind origin/develop by 4 commits → `7b4f1ed0`) |
| Worktree state | clean |

> **Note (vs original brief):** the task body said the repo lives at
> `/home/builder/hermes-share/rob_box_project`. That path does not exist —
> the actual primary repo is `/home/builder/rob_box_project`. This matches
> every existing worktree on disk and all prior kanban context, so it is
> treated as the canonical path. Flagged here so a future auditor doesn't
> chase a phantom path.
>
> **Base-branch choice:** the brief says "off `feature/avatar`". Local
> `feature/avatar` is at `da429fb4`; `origin/feature/avatar` is at
> `c8e141ea` and contains 9 additional commits ahead of the local copy,
> including the **AV-11 attempt-8 work (#1705)** that documents the
> current real-e2e blocker (deployed supervisor is Phase-1 monitor-only;
> `/avatar/state` wire-payload truncated to 8 bytes; FSM active-mode
> logic in source not yet built into the deployed image). Branched off
> `origin/feature/avatar` so this attempt inherits those fixes.

## 2. Compose / launch scripts inventory

### docker-compose files (4 + 1 test)
- `docker/vision/docker-compose.yaml` — Vision Pi fleet
  (zenoh-router, oak-d, led-matrix, ceiling-camera, supercollider,
   voice-resources-init, voice-assistant, voice-action-server, quest,
   telegram-bot, plus monitoring profile: cadvisor + promtail, plus ai
   profile: ollama). **network_mode: host** on every service.
- `docker/main/docker-compose.yaml` — Main Pi fleet
  (zenoh-router, twist-mux, robot-state-publisher, rtabmap, ros2-control,
   lslidar, perception, nav2, teleop, monitoring profile: cadvisor +
   promtail).
- `docker/monitoring/docker-compose.yaml` — builder-side observability
  (loki-init, prometheus, loki, grafana, otel-collector, tempo).
- `docker/build/docker-compose.yaml` — registry / apt-cacher infrastructure
  on builder.
- `docker/vision/test/docker-compose.test.yml` — Vision Pi test harness.

### launch_*.sh files
- `local_test/launch_rviz.sh` — RViz2 + Zenoh middleware on dev machine.
- `local_test/launch_rviz_fpv.sh` — first-person RViz variant.
- No `launch_*.sh` under `docker/vision/` or `docker/main/` — those
  compose files call `start_*.sh` instead.

### Per-service start scripts (called by compose `command:`)
- Vision: `docker/vision/scripts/{zenoh-router,oak-d,led_matrix,
  ceiling-camera,supercollider,voice_assistant,quest,telegram_bot,
  voice-resources-init}/start_*.sh`
- Main: `docker/main/scripts/{zenoh-router,twist_mux,robot_state_publisher,
  rtabmap,ros2_control,lslidar,perception,nav2,teleop}/start_*.sh`

## 3. SSH / ansible reachability to Pis

| Target | DNS | IP | Ping | SSH (key) | Docker daemon |
|---|---|---|---|---|---|
| Vision Pi | `visionpi.lan` | `10.1.1.21` | ✓ (1.0 ms) | ✗ Permission denied (no key) | not exposed on `2375` |
| Main Pi   | (not in /etc/hosts) | `10.1.1.10` | ✓ (0.9 ms) | ✗ Permission denied (no key) | not exposed on `2375` |

- `/home/builder/.ssh/` contains only `known_hosts` / `known_hosts.old` —
  **no private key on this builder**. SSH attempts with `BatchMode=yes`
  for users `ros2`, `builder`, `pi`, `ubuntu` all fail with
  `Permission denied (publickey,password)`.
- No ansible inventory in repo (`grep -r inventory` in repo → none).
  Deployment historically goes through `scripts/setup_{main,vision}_pi.sh`
  plus the CI/CD pipeline, not ad-hoc ansible.
- Local docker registry on builder is alive: `curl
  http://localhost:5000/v2/_catalog` returns
  `{"repositories":["krikz/rob_box","krikz/rob_box_base","rob_box/telegram-bot"]}`.

### Bring-up strategy decision (per task body step 3)

**Per-Pi (each host runs its own compose).** Rationale:

- All four compose files use `network_mode: host`, mounting physical
  devices (`/dev/bus/usb`, `/dev/spidev0.0`, `/dev/ttyACM0`, `/dev/snd`,
  `/dev/video0`, `/dev/kmsg`, `/var/run/docker.sock`). Centralised
  compose (builder drives the daemon remotely) is technically possible
  only if Pis expose `2375` (insecure) or `2376` (TLS) — **neither is
  exposed today** (curl on `10.1.1.21:2375` and `10.1.1.10:2375` is
  unreachable from builder).
- All `start_*.sh` scripts assume local `/config` and `/scripts` mounts
  (relative `./config`, `./scripts`) — they live on the Pi's clone of
  the repo.
- The current ROS_DOMAIN_ID (`0`) + Zenoh router pattern uses
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` on Vision Pi services and
  the Main Pi services connect to their own local router — there is no
  cross-Pi service sharing at the compose level (cross-Pi traffic
  happens at the Zenoh bridge layer, not via shared compose).
- Builder does not have SSH key, so it cannot push/pull images to
  remote daemons even if they were exposed. Per-Pi deploy = builder
  pushes images to local registry `10.1.1.249:5000`, then the Pis pull
  them. This is the pattern `.env` already uses
  (`REGISTRY=10.1.1.249:5000`).

**Action item for next step:** add an SSH key (or get one deployed)
before the bring-up worker can run `docker compose up -d` on each Pi.
Without it, no container will start on a Pi from builder.

## 4. Config inventory & gaps

### Existing files (real, not template)
- `docker/vision/.env` — ROS_DISTRO=humble, IMAGE_TAG=dev,
  REGISTRY=10.1.1.249:5000, ROBOT_ID=RBXU100001, LOKI_HOST=monitoring-machine.
- `docker/main/.env` — same content (humble, dev, 10.1.1.249:5000,
  RBXU100001, monitoring-machine).

### Templates (not deployed)
- `docker/vision/.env.secrets.template` — voice-assistant LLM keys.
- `docker/main/.env.secrets.example` — perception LLM keys.
- `docker/monitoring/.env.example` — IP plan + grafana password.
- `docker/build/.env.secrets` — exists on builder only (apt-cacher
  creds etc.).

### Config gaps that block starting the three AV-11 containers

| # | Gap | Effect | Container(s) blocked |
|---|---|---|---|
| 1 | **No `docker/vision/.env.secrets` and `docker/main/.env.secrets`** (only templates exist). Missing: `MIMO_API_KEY`, `DEEPSEEK_API_KEY`, `YANDEX_API_KEY`, `YANDEX_FOLDER_ID`, `TELEGRAM_BOT_TOKEN`, `TELEGRAM_ALLOWED_USERS`. | Voice/telegram containers will fail healthchecks / 503 on first LLM call. `docker compose config` warns "env_file not found". | `voice-assistant`, `telegram-bot`, `perception`, `voice-action-server` |
| 2 | **`/home/builder/.ssh/` has no private key**; SSH to `10.1.1.21` and `10.1.1.10` fails with `Permission denied (publickey,password)`. | Cannot `rsync` repo to Pis, cannot run `docker compose up` on Pi daemon from builder, cannot read `docker logs` / `ros2` state on Pi for verification. | All three AV-11 containers (cannot deploy or verify) |
| 3 | **`monitoring/.env.example` says `VISION_PI_IP=10.1.1.11`**, but actual Vision Pi is `10.1.1.21`. Same drift likely for `MAIN_PI_IP` (says `10.1.1.10`, matches reality). | If `prometheus.yml` or `promtail-config.yaml` scrape targets use `VISION_PI_IP`, monitoring will fail for Vision Pi (cadvisor / node-exporter endpoints). | `prometheus` on builder (if scraping Vision Pi directly) |
| 4 | **No deploy script for AV-11 trio** — only `setup_main_pi.sh` / `setup_vision_pi.sh` exist, which do OS-level provisioning, not "start quest+supervisor+telegram". The orchestrator must roll a custom `docker compose -f docker/vision/docker-compose.yaml up -d quest telegram-bot` per host. | Brings extra manual error surface. | workflow risk, not container-side |
| 5 | **Deployed supervisor is Phase-1 monitor-only** (documented in `origin/feature/avatar` attempt-8 commit `c8e141ea` / `.hermes/av-11-analysis/blocker-state-t_42d98188.md`). Active-FSM source exists in `src/rob_box_supervisor/` (86 unit-tests GREEN per attempt-7), but is **NOT** in the deployed image. | Live e2e acceptance #3–#7 of the AV-11 plan (acquire_floor applied=true, /avatar/state wire-payload correct, mixed-mode log line) cannot pass. | `avatar-supervisor` container |

### Pre-existing supervisor e2e blocker (not in this task's scope)

The AV-11 bring-up also has a deeper functional blocker surfaced by
the attempt-8 work: the deployed supervisor image (built 2026-08-27)
is Phase-1 monitor-only; the active-FSM source code is merged into
`feature/avatar` but not yet built into the registry image. To unblock
live e2e, **devops** must rebuild the supervisor image after the
active-FSM merge lands, then trigger the "L: Deploy and Verify"
follow-up. This is the next-devops-step handoff, not this inventory
task.

## 5. Acceptance summary

- ✅ Repo confirmed at `/home/builder/rob_box_project`, base branch
  `develop` is present and up-to-date in spirit (4 commits behind
  origin/develop, can fast-forward).
- ✅ Branch `feature/av-11-avatar-mixed-e2e` created from
  `origin/feature/avatar` (HEAD `c8e141ea`), checked out in worktree
  `t_63ede363`. Worktree is clean.
- ✅ Docker-compose + launch scripts enumerated (4 compose files + 2
  `launch_*.sh` + 16 per-service `start_*.sh`).
- ⚠ SSH reachability: both Pis reachable by ping, **not reachable by
  SSH** from builder (no key). Bring-up must use the
  push-to-registry / pull-on-Pi pattern.
- ⚠ Docker daemon on Pis not exposed on `2375`/`2376`, confirmed.
- ✅ Bring-up strategy recorded: **per-Pi**, leveraging existing
  `10.1.1.249:5000` local registry and `IMAGE_TAG=dev`.
- ⚠ Five config gaps inventoried (env.secrets missing, no SSH key,
  monitoring IP drift, no AV-11 deploy playbook, supervisor image
  stale). All flagged as blockers before any container can be started
  by the next worker.
