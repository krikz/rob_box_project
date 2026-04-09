# Main Docker Layout Normalization Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Normalize `docker/main` so its runtime file layout matches the `docker/vision` convention without changing service behavior.

**Architecture:** Keep container images and service behavior unchanged, and focus only on runtime file placement plus compose/doc alignment. The only functional path change in this pass is moving `rtabmap` startup from the shared scripts root into `scripts/rtabmap/`, while preserving explicit documented exceptions for the `rob_box_description` source bind and shared monitoring config.

**Tech Stack:** Docker Compose, bash startup scripts, ROS 2 Humble, Markdown documentation

---

### Task 1: Normalize `rtabmap` startup script placement

**Files:**
- Create: `docker/main/scripts/rtabmap/`
- Modify: `docker/main/scripts/start_rtabmap.sh`
- Create: `docker/main/scripts/rtabmap/start_rtabmap.sh`
- Modify: `docker/main/docker-compose.yaml`

**Step 1: Write the failing check**

Run:
- `test -f docker/main/scripts/start_rtabmap.sh && echo legacy-present`
- `test -f docker/main/scripts/rtabmap/start_rtabmap.sh && echo normalized-present`
- `grep -n "start_rtabmap.sh\|/scripts/rtabmap" docker/main/docker-compose.yaml`

Expected before changes:
- `legacy-present`
- no `normalized-present`
- compose contains `/ros_scripts/start_rtabmap.sh`

**Step 2: Run check to verify current mismatch**

Run:
- `ls docker/main/scripts`
- `grep -n "start_rtabmap.sh" docker/main/docker-compose.yaml`

Expected:
- `start_rtabmap.sh` exists in the shared root
- `rtabmap` command still points to `/ros_scripts/start_rtabmap.sh`

**Step 3: Write minimal implementation**

Implement only the structural move:
- create `docker/main/scripts/rtabmap/`
- move `docker/main/scripts/start_rtabmap.sh` to `docker/main/scripts/rtabmap/start_rtabmap.sh`
- update `docker/main/docker-compose.yaml` so `rtabmap` mounts `./scripts/rtabmap:/scripts:ro`
- update the `rtabmap` command from `/ros_scripts/start_rtabmap.sh` to `/scripts/start_rtabmap.sh`
- keep `./scripts:/ros_scripts:ro` because `patch_rtabmap_launch.py` and `ros_with_namespace.sh` are shared infrastructure

**Step 4: Run verification**

Run:
- `test -f docker/main/scripts/rtabmap/start_rtabmap.sh && echo normalized-present`
- `test ! -f docker/main/scripts/start_rtabmap.sh && echo legacy-removed`
- `grep -n "start_rtabmap.sh\|/scripts/rtabmap" docker/main/docker-compose.yaml`

Expected:
- `normalized-present`
- `legacy-removed`
- compose contains `./scripts/rtabmap:/scripts:ro`
- compose contains `/scripts/start_rtabmap.sh`

**Step 5: Commit**

Run:
- `git add docker/main/docker-compose.yaml docker/main/scripts/rtabmap/start_rtabmap.sh docker/main/scripts/start_rtabmap.sh`
- `git commit -m "refactor(docker): normalize main rtabmap startup path"`

---

### Task 2: Document canonical layout and approved exceptions

**Files:**
- Modify: `docs/development/DOCKER_STANDARDS.md`
- Modify: `docs/development/BUILD_OPTIMIZATION.md`
- Modify: `docs/plans/2026-03-08-main-docker-layout-normalization-design.md`

**Step 1: Write the failing check**

Run:
- `grep -n "start_rtabmap.sh\|config/monitoring\|rob_box_description\|approved exceptions\|source bind" docs/development/DOCKER_STANDARDS.md`
- `grep -n "docker/main\|docker/vision\|monitoring\|shared" docs/development/BUILD_OPTIMIZATION.md`

Expected before changes:
- docs describe the general standard but do not clearly record the new `rtabmap` script location and the two approved `docker/main` exceptions together

**Step 2: Verify current doc gaps**

Read and confirm these points are missing or under-specified:
- `rtabmap` startup script should live under `scripts/rtabmap/`
- `config/monitoring/...` is a valid shared subsystem config
- `../../src/rob_box_description` is an allowed source bind exception

**Step 3: Write minimal implementation**

Update docs so they explicitly state:
- `docker/main` and `docker/vision` follow one runtime layout convention
- service startup scripts live in `scripts/<service>/`
- shared reusable helper scripts may stay in `scripts/`
- `config/monitoring/` is a valid shared subsystem config
- `robot-state-publisher` source bind remains an approved exception because it is not runtime config/script data

**Step 4: Run verification**

Run:
- `grep -n "scripts/rtabmap/start_rtabmap.sh\|config/monitoring\|rob_box_description\|approved exceptions\|source bind" docs/development/DOCKER_STANDARDS.md`
- `grep -n "docker/main\|docker/vision\|shared subsystem\|monitoring" docs/development/BUILD_OPTIMIZATION.md`

Expected:
- all required rules and exceptions are discoverable via grep

**Step 5: Commit**

Run:
- `git add docs/development/DOCKER_STANDARDS.md docs/development/BUILD_OPTIMIZATION.md docs/plans/2026-03-08-main-docker-layout-normalization-design.md`
- `git commit -m "docs(docker): document main layout normalization rules"`

---

### Task 3: Verify compose rendering and record completion

**Files:**
- Modify: `progress.md`
- Read: `docker/main/.env`
- Read: `docker/main/.env.secrets.example`

**Step 1: Write the failing check**

Define completion evidence:
- `docker/main/docker-compose.yaml` must render successfully after the script move
- changed files must have no new editor diagnostics
- `progress.md` must record the normalization work and verification evidence

**Step 2: Verify prerequisites**

Run:
- `test -f docker/main/.env && echo env-present`
- `test -f docker/main/.env.secrets || echo secrets-missing`

Expected:
- `.env` exists
- `.env.secrets` may be missing locally; if so, create a temporary placeholder only for compose validation and delete it afterwards

**Step 3: Write minimal implementation**

- if needed, create temporary `docker/main/.env.secrets` with placeholder content for local validation only
- run `docker compose -f docker/main/docker-compose.yaml config`
- update `progress.md` with the `docker/main` normalization summary and the verification command used
- delete temporary `docker/main/.env.secrets` if one was created

**Step 4: Run verification**

Run:
- `docker compose -f docker/main/docker-compose.yaml config >/tmp/main-compose.out && tail -n 20 /tmp/main-compose.out`
- editor diagnostics on changed files
- `grep -n "DOCKER-STRUCT\|main layout\|rtabmap" progress.md`

Expected:
- compose render exits 0
- diagnostics are clean or only pre-existing unrelated issues remain
- `progress.md` contains the new normalization entry

**Step 5: Commit**

Run:
- `git add progress.md`
- `git commit -m "docs(progress): record main docker layout normalization"`
