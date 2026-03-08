# Main Docker Layout Normalization Design

**Date:** 2026-03-08
**Status:** Approved
**Related work:** [Docker build optimization](2026-03-08-docker-build-optimization.md)

## Goal

Bring `docker/main` to the same runtime layout convention now used in `docker/vision`, so service directories contain only build artifacts and runtime files are consistently stored under shared `config/` and `scripts/` trees.

## Problem

`docker/main` is already close to the target structure, but it still has a few legacy inconsistencies:

- `rtabmap` startup is launched through `./scripts/start_rtabmap.sh` instead of `./scripts/rtabmap/start_rtabmap.sh`.
- Some compose mounts use direct file binds instead of the standard subtree pattern.
- `docker/main` and `docker/vision` now communicate the same intent but not with the same concrete structure, which makes maintenance and future refactors harder.

## Design Summary

### 1. Canonical runtime layout

`docker/main` follows the same layout rules as `docker/vision`:

- `docker/main/<service>/` contains only `Dockerfile`.
- Service-specific runtime configs live in `docker/main/config/<service>/...`.
- Service-specific runtime scripts live in `docker/main/scripts/<service>/...`.
- Shared configs used by multiple services may live in subsystem folders such as `docker/main/config/monitoring/...`.
- Root-level utility scripts may stay in `docker/main/scripts/` when they are not startup scripts for one service.

### 2. Scope of normalization

This pass normalizes runtime script/config placement without changing intended runtime behavior.

#### In scope

- Move `start_rtabmap.sh` into `scripts/rtabmap/start_rtabmap.sh`.
- Update `docker/main/docker-compose.yaml` so `rtabmap` mounts and runs the service-local startup script from `scripts/rtabmap/`.
- Update docs so `docker/main` and `docker/vision` describe the same standard and the same exception model.
- Validate that compose rendering still works after the path update.

#### Out of scope for this pass

- Renaming legacy folder names such as `robot_state_publisher`, `twist_mux`, or `micro_ros_agent`.
- Changing image names, CI behavior, or build layering.
- Refactoring non-runtime source binds.

### 3. Approved exceptions

The following remain intentional exceptions and must be documented, not removed:

#### Source bind for `robot-state-publisher`

The bind mount of `../../src/rob_box_description:/workspace/src/rob_box_description:ro` stays as-is. It is a source/package mount for URDF assets, not a runtime config or runtime script. Treating it as an exception avoids unnecessary packaging changes in this task.

#### Shared monitoring config

`config/monitoring/promtail-config.yaml` remains a shared subsystem config, not a service-local config. This matches the existing approved rule for shared resources such as `config/audio/...` in `docker/vision`.

## Data Flow / Runtime Impact

This change is layout-only:

- `rtabmap` still uses the same launch command and parameters.
- `ros_with_namespace.sh` remains in the shared scripts root because it is reusable infrastructure.
- No container image content changes are required for this normalization.

## Risks

### Risk: broken `rtabmap` startup path

Mitigation:
- move only the script path;
- update compose mount and command in the same change;
- render compose configuration after the edit.

### Risk: documentation drift

Mitigation:
- update Docker standards and any active optimization docs that describe the canonical layout.

## Verification

The work is complete when all of the following are true:

1. `docker/main/*/` service folders still contain only `Dockerfile`.
2. `rtabmap` startup script lives at `docker/main/scripts/rtabmap/start_rtabmap.sh`.
3. `docker/main/docker-compose.yaml` references the normalized `rtabmap` script path.
4. `docker compose -f docker/main/docker-compose.yaml config` succeeds.
5. Changed files have no new editor diagnostics.
6. Documentation clearly states the shared standard plus the approved exceptions.

## Files Expected To Change

- `docker/main/docker-compose.yaml`
- `docker/main/scripts/start_rtabmap.sh` → move to `docker/main/scripts/rtabmap/start_rtabmap.sh`
- `docs/development/DOCKER_STANDARDS.md`
- `docs/development/BUILD_OPTIMIZATION.md` if it references the canonical layout for both stacks
- `progress.md` if this work is folded into the current optimization stream
