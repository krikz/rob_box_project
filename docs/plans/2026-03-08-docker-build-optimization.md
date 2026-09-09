# Docker Build Optimization Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Reduce Docker rebuild time for Rob Box, starting with high-impact workflow/context fixes and then separating heavy immutable layers from fast-changing voice application code.

**Architecture:** Keep Rob Box runtime rules intact: configs/scripts remain volume-mounted, host networking stays unchanged, and optimization focuses on Docker layer boundaries. The first pass hardens CI triggers and build context; the second pass restructures `voice-assistant` into slower-changing dependency/resource layers and faster-changing application layers.

**Tech Stack:** Docker Buildx, GitHub Actions, ROS 2 Humble, docker-compose, GHCR

---

### Task 1: Baseline and CI input alignment

**Files:**
- Modify: `.github/workflows/G-Build Vision Pi Services.yml`
- Modify: `.dockerignore`
- Doc: `docs/plans/2026-03-08-docker-build-optimization.md`

**Step 1: Add failing expectation as validation target**

Define expected behavior before editing:
- A change in `src/rob_box_voice/**` must trigger the hosted Vision build workflow.
- Large local build-runner artifacts under `docker/build/**` must not enter root Docker build context.

**Step 2: Verify current mismatch**

Run:
- `grep -n "paths:" -A8 .github/workflows/G-Build\ Vision\ Pi\ Services.yml`
- `grep -n "docker/build" .dockerignore`

Expected:
- Workflow path filter only watches `docker/vision/**` and the workflow file.
- Root `.dockerignore` does not exclude `docker/build/**`.

**Step 3: Write minimal implementation**

Change workflow triggers to include real `voice-assistant` build inputs:
- `src/rob_box_voice/**`
- `src/rob_box_animations/**`
- `src/rob_box_perception_msgs/**`
- `src/rob_box_mcp_tools/**`
- `sound_pack/**`
- `migrations/**`
- `docker/vision/scripts/voice_assistant/**`
- `docker/vision/config/voice_assistant/**`

Change root `.dockerignore` to exclude local build infrastructure artifacts:
- `docker/build/`
- `docker/build/**`

**Step 4: Verify implementation**

Run:
- `grep -n "paths:" -A20 .github/workflows/G-Build\ Vision\ Pi\ Services.yml`
- `grep -n "docker/build" .dockerignore`

Expected:
- Workflow now watches the actual source inputs for `voice-assistant`.
- Root `.dockerignore` excludes `docker/build`.

**Step 5: Commit**

Run:
- `git add .github/workflows/G-Build\ Vision\ Pi\ Services.yml .dockerignore docs/plans/2026-03-08-docker-build-optimization.md`
- `git commit -m "fix(docker): align voice build inputs"`

---

### Task 2: Remove cache-hostile runtime data from `voice-assistant` build

**Files:**
- Modify: `docker/vision/voice_assistant/Dockerfile`
- Read: `docker/vision/docker-compose.yaml`
- Read: `docker/vision/test/docker-compose.test.yml`

**Step 1: Write the failing check**

Define the desired rule:
- Files already supplied via runtime volume should not be baked into the image unless required for build/test correctness.

**Step 2: Verify current duplication**

Run:
- `grep -n "COPY sound_pack\|COPY src/rob_box_voice/config\|COPY src/rob_box_voice/prompts" docker/vision/voice_assistant/Dockerfile`
- `grep -n "/ws/sound_pack\|/config/voice" -n docker/vision/docker-compose.yaml`

Expected:
- Dockerfile copies runtime-oriented assets.
- Compose already mounts at least part of those assets.

**Step 3: Write minimal implementation**

Refactor only the assets that are safe to externalize first:
- remove duplicated `sound_pack` copy if runtime mounts are always present for production and tests remain unaffected;
- keep only build-required package files in the image;
- if prompts/config must remain for package correctness, postpone them to a later task instead of forcing a risky change.

**Step 4: Verify**

Run targeted build validation:
- `docker buildx build --platform linux/amd64 --file docker/vision/voice_assistant/Dockerfile --target=default .`
  - if no target exists, run full build with `--progress=plain` and confirm the Dockerfile still resolves required files.

Expected:
- Build succeeds after removing only truly redundant runtime data.

**Step 5: Commit**

Run:
- `git add docker/vision/voice_assistant/Dockerfile`
- `git commit -m "refactor(docker): trim voice runtime assets"`

---

### Task 3: Reintroduce heavy cached base for `voice-assistant`

**Files:**
- Modify: `docker/vision/voice_assistant/Dockerfile`
- Modify: `.github/workflows/G-Build Vision Pi Services.yml`
- Modify: `.github/workflows/L-Build Vision Pi Services.yml`
- Modify: `.github/workflows/G-Build Base Images.yml`
- Modify: `.github/workflows/L-Build Base Images.yml`
- Modify: `docker/vision/scripts/build_voice_assistant.sh`

**Step 1: Write the failing check**

Desired behavior:
- `voice-assistant` should build on top of a dedicated heavy base containing slow-changing dependencies and models.
- Fast-changing application changes should not re-run apt/pip/model download layers.

**Step 2: Verify current drift**

Run:
- `grep -n "BASE_IMAGE" docker/vision/voice_assistant/Dockerfile`
- `grep -n "voice-assistant" -A20 .github/workflows/G-Build\ Vision\ Pi\ Services.yml`
- `grep -n "voice_base" -R .github/workflows docker/vision`

Expected:
- `voice_base` exists but active builds do not consistently use/publish it.

**Step 3: Write minimal implementation**

- standardize a publishable `voice-base` image/tag;
- move or keep heavy slow-changing dependencies in `docker/vision/voice_base/Dockerfile`;
- switch `voice-assistant` build arg default and workflows to consume `voice-base`;
- preserve existing offline model behavior unless explicitly changed later.

**Step 4: Verify**

Run:
- base image build command
- `voice-assistant` build command using the newly published/local `voice-base`

Expected:
- base and app builds both succeed;
- app rebuild after a source-only change reuses heavy base layers.

**Step 5: Commit**

Run:
- `git add docker/vision/voice_base/Dockerfile docker/vision/voice_assistant/Dockerfile .github/workflows/G-Build\ Base\ Images.yml .github/workflows/L-Build\ Base\ Images.yml .github/workflows/G-Build\ Vision\ Pi\ Services.yml .github/workflows/L-Build\ Vision\ Pi\ Services.yml docker/vision/scripts/build_voice_assistant.sh`
- `git commit -m "feat(docker): restore voice base image pipeline"`

---

### Task 4: Split Renardo samples into a resource/init path

**Files:**
- Create: `docker/vision/voice_resources/Dockerfile`
- Modify: `docker/vision/docker-compose.yaml`
- Modify: `docker/vision/scripts/voice_assistant/start_voice_assistant.sh`
- Modify: `.github/workflows/G-Build Vision Pi Services.yml`
- Modify: `.github/workflows/L-Build Vision Pi Services.yml`

**Step 1: Write the failing check**

Desired behavior:
- Renardo sample download should not run during normal `voice-assistant` rebuilds.
- `voice-assistant` and `supercollider` must still share the same ready-to-use sample volume.

**Step 2: Verify current coupling**

Run:
- `grep -n "download_samples\|renardo_samples_builtin" docker/vision/voice_assistant/Dockerfile`
- `grep -n "renardo_samples" -A6 docker/vision/docker-compose.yaml`
- `grep -n "renardo" docker/vision/scripts/voice_assistant/start_voice_assistant.sh`

Expected:
- Sample acquisition is coupled to `voice-assistant` build.
- Volume seeding is coupled to `voice-assistant` startup.

**Step 3: Write minimal implementation**

- create a small resource image that downloads samples;
- seed the named volume from that image or an init-style service;
- keep `voice-assistant` startup idempotent.

**Step 4: Verify**

Run:
- build `voice_resources` image;
- `docker compose config` for `docker/vision/docker-compose.yaml`;
- manual smoke check that both services see the same sample path.

Expected:
- Sample population no longer depends on rebuilding `voice-assistant`.

**Step 5: Commit**

Run:
- `git add docker/vision/voice_resources/Dockerfile docker/vision/docker-compose.yaml docker/vision/scripts/voice_assistant/start_voice_assistant.sh .github/workflows/G-Build\ Vision\ Pi\ Services.yml .github/workflows/L-Build\ Vision\ Pi\ Services.yml`
- `git commit -m "feat(docker): split renardo resources image"`

---

### Task 5: Final verification and documentation

**Files:**
- Modify: `docs/development/DOCKER_STANDARDS.md`
- Modify: `docs/development/BUILD_OPTIMIZATION.md`
- Modify: `progress.md`

**Step 1: Verify all changed build paths**

Run:
- `docker buildx build --platform linux/amd64 --file docker/vision/voice_base/Dockerfile .`
- `docker buildx build --platform linux/amd64 --file docker/vision/voice_assistant/Dockerfile .`
- `docker compose -f docker/vision/docker-compose.yaml config >/tmp/vision-compose.out && tail -n 20 /tmp/vision-compose.out`

Expected:
- both builds succeed;
- compose renders valid configuration.

**Step 2: Verify warm rebuild behavior**

Run one source-only rebuild after touching a Python file and confirm dependency/model/sample layers are cached.

**Step 3: Document the new rules**

Update docs with:
- heavy base image rule for slow-changing dependencies;
- resource/init image rule for shared large immutable assets;
- runtime volume rule for mutable text/config/script assets;
- CI trigger rule for non-Docker source inputs.

**Step 4: Record summary**

Add concise entry to `progress.md` with files changed and verification commands executed.

**Step 5: Commit**

Run:
- `git add docs/development/DOCKER_STANDARDS.md docs/development/BUILD_OPTIMIZATION.md progress.md`
- `git commit -m "docs(docker): document optimized build architecture"`
