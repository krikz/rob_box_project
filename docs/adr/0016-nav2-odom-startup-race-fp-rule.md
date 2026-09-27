# ADR-0016: Nav2 critical_log "frame odom does not exist" — startup race + stale detector FP rule

Status: accepted (deployment-critical fix, issue #774)
Date: 2026-08-12
Deciders: architect (kanban t_f3ca3377)

## Context

Deploy workflow (`L-Deploy and Verify.yml`) flagged `nav2` container with
`critical_log` on every staging deploy since ~2026-04:

```
[local_costmap.local_costmap]: Timed out waiting for transform from
base_footprint to odom to become available, tf error: Invalid frame ID "odom"
passed to canTransform argument target_frame - frame does not exist
```

Live inspection (10.1.1.10 == 10.1.1.20, same host via two addresses) showed:

- All containers healthy: nav2, ros2-control, rtabmap, robot-state-publisher.
- `diff_drive_controller` is `active`, publishes `/odom` topic AND TF
  `odom → base_footprint` (`enable_odom_tf: true`, `odom_frame_id: odom`,
  `base_frame_id: base_footprint` in `controller_manager.yaml`).
- The critical message appears exactly **twice at container start** (t=0.35s,
  t=0.85s after activation), then `start` — the costmap comes up fine.
- `tf2_echo odom base_footprint` works after startup.

## Root cause (two layers)

### 1. Startup race in `start_nav2_direct.sh`

`docker/main/scripts/nav2/start_nav2_direct.sh` launches Nav2 nodes directly
(lifecycle_manager + controller/planner/bt/behavior/waypoint/velocity/smoother)
with **no wait for odometry readiness**. The old `start_nav2.sh` waited for the
`/odom` topic; the direct variant (introduced by cd54ac8e to bypass
`RewrittenYaml`) dropped the readiness check entirely. On cold start the
costmap activates before `diff_drive_controller` publishes its first TF
(~1–2s), so `local_costmap` logs the "frame does not exist" timeout. This is a
known, benign startup pattern — the transform appears moments later and the
costmap proceeds.

### 2. Stale false-positive rule in the deploy detector

`.github/scripts/deployment_issue_dedup.py` has an FP exclusion:

```python
r"timed out waiting for transform from base_link to odom",
```

But commit afbb8793 (2026-03-09) changed `robot_base_frame` from `base_link`
to `base_footprint` across `nav2_params.yaml`. The detector still excludes
only `base_link`, so the actual log line (`base_footprint to odom`) is treated
as a real critical error → a new deployment issue is created on every deploy.

## Decision

1. **Remove the race**: add a bounded odometry-readiness wait to
   `start_nav2_direct.sh` before launching Nav2 nodes. It waits for the
   `/odom` topic (published by `diff_drive_controller` at the same time as the
   TF) with a timeout. On timeout it **warns and continues** (same semantics as
   the old `start_nav2.sh`) — Nav2 still starts so the robot can be operated,
   and a genuinely missing odometry remains visible in logs instead of being
   silently swallowed.

2. **Fix the detector FP rule**: extend the exclusion to match both frame
   names — `base_link` (legacy) and `base_footprint` (current):
   `r"timed out waiting for transform from (base_link|base_footprint) to odom"`.
   Add a regression test for the `base_footprint` variant.

3. **Do NOT** start `start_odom_relay.sh` (rtabmap/odom → /odom). It is a
   dead artifact (added accidentally in 3ca296a7) and would relay the *topic*
   only — it does not publish the TF `odom → base_footprint` that Nav2 needs.
   Odometry TF already comes from `diff_drive_controller`.

## Trade-offs

- Waiting for `/odom` topic instead of querying TF directly keeps the script
  simple and dependency-free (`ros2 topic list` is available in the base
  image). A TF lookup would require `tf2_ros` CLI gymnastics in a loop.
- Continuing on timeout (rather than `exit 1`) mirrors the old script and
  avoids a container restart loop (the teleop incident showed restart loops
  are worse than a warning).
- Fixing the detector rule alone would silence the alert but leave the log
  noise; fixing the race alone would still alert on deploys where the costmap
  wins the race by milliseconds. Both are needed.

## Consequences

- Deploys no longer create false `critical_log` issues for Nav2 TF startup
  timeouts.
- Nav2 starts only after odometry is present, so `local_costmap` activation
  is deterministic.
- The detector keeps flagging genuine missing-odometry cases when the
  `start_nav2_direct.sh` wait times out (message still appears in logs but the
  FP rule now covers the benign startup variant).

## Verification

- Unit: `pytest .github/scripts/tests/test_deployment_issue_dedup.py`.
- e2e (after merge): deploy to staging, check `docker logs nav2 --tail 50`
  after restart shows no `frame "odom" does not exist` critical line, and
  `ros2 run tf2_ros tf2_echo odom base_footprint` succeeds.
