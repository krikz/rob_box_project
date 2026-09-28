# ROB-BOX Architecture Audit

## Evidence chain

container -> node -> ROS interface -> class -> responsibility -> capability -> feature

The reverse view is also required:

feature -> capability -> node -> publisher/subscriber/service/action

The tooling separates evidence from architectural decisions. A duplicate or unpaired interface is a review candidate, not an automatic refactor instruction.

## Audit layers

1. Static inventory: Compose services (every `docker/*/docker-compose*.y*ml` except `docker/build/`, which is CI/registry infrastructure), ROS packages, Python classes, node classes, launch entries and ROS interfaces. Tests and examples (`test/`, `tests/`, `test_*.py`, `conftest.py`, `scripts/test_*`, `scripts/example_*`) are skipped and only counted as `skipped_test_files`.
   ROS interface names come from the topic/service argument of `create_publisher`, `create_subscription`, `create_service`, `create_client`, `create_action_client` and `shared_publisher`. A string literal is taken as is. A `self.<attr>` argument is resolved statically inside the innermost enclosing class (Node or not) and the entry is marked, because the name may be overridden at launch:
   - `"name_source": "parameter_default"` plus `"name_parameter": "<p>"` when the class assigns `self.<attr> = self.get_parameter('<p>').value` (optionally wrapped in `str(...)`) and calls `self.declare_parameter('<p>', '<string literal>')`;
   - `"name_source": "class_attr"` when `<attr>` is a class-level string constant (`topic = '/camera/...'`, `ODOM_TOPIC = '/odom'`).
   An attribute with more than one candidate value (e.g. a class constant also reassigned in a method, or two different declared defaults) is ambiguous and skipped, as are nested classes, inherited attributes, local variables and non-literal defaults. The markers are carried into `topics[].publishers/subscribers/...` and shown in `inventory.md` as *(default of `p`)* / *(class attr)*; literal entries have no `name_source`.
2. Structural review: multiple publishers, unpaired interfaces, duplicate class names, semantic identity-topic names and node packages without literal launch entries.
3. Runtime graph: real ros2 node/topic/service/action output and verbose topic endpoints.
4. Static/runtime diff: declared topics absent at runtime and runtime-only topics.
5. Ownership: architecture/ownership.yml records canonical owner only after human verification.
6. Capability/feature overlay: implementation evidence is reconciled with the feature matrix and acceptance evidence.
7. Semantic review: each topic/service/action needs one contract, owner and meaning.

## Runtime collection

The runtime collector runs on the self-hosted `rob-box` runner, where the robot ROS2 network is reachable. GitHub-hosted runners must not be used for live ROS2 collection.

Examples:

    python tools/architecture_runtime_snapshot.py
    python tools/architecture_runtime_snapshot.py --topics /scan,/cmd_vel
    source /opt/ros/humble/setup.bash
    python tools/architecture_runtime_snapshot.py --topics /scan,/cmd_vel
    python tools/architecture_runtime_diff.py architecture/inventory.json architecture/runtime.json

The `L: Architecture Audit` workflow runs on the self-hosted `rob-box` runner and reaches the robot over password SSH via `sshpass`: Main Pi `10.1.1.20` (container `nav2`) and, with `capture_vision`, Vision Pi `10.1.1.21` (container `oak-d`), user `ros2` by default. The password comes only from the `ROBOT_SSH_PASS` repository secret; the first step fails with an `::error::` when it is empty. Workflow inputs reach the scripts through `env:`, never as `${{ }}` expressions inside `run:`. GitHub-hosted runners are never used for live robot access.

Remote capture inspects all topics in one `ssh` + one `docker exec`: a loop inside the container prints each `ros2 topic info --verbose` between `=====TOPIC <name>=====` / `=====RC <n>=====` markers, and the same call records `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` from the container. Topics whose block is empty, failed or missing are retried one `ssh` each; `--per-topic` skips the batch entirely. The batch call times out after at most 300 s, and `--budget` (480 s; 240 s per Pi with `capture_vision`) caps batch + fallback so the job stays inside its 15 min: topics left when it runs out are recorded as `present: false, returncode: null, skipped: "budget"`, with a `::warning::` and a count in the job summary.

Both Pis see one shared Zenoh graph (run 36422274238: 52 nodes on each Pi, union 52), so the workflow captures from the Main Pi only. The `capture_vision` input adds the Vision Pi capture; `tools/architecture_runtime_merge.py` merges one or more snapshots into `runtime.json`:

    python tools/architecture_runtime_merge.py --output architecture/runtime.json \
      --capture "Main Pi|10.1.1.20|nav2|architecture/runtime-main.json" \
      --capture "Vision Pi|10.1.1.21|oak-d|architecture/runtime-vision.json"

## Node review questions

- What problem does this node solve?
- What capability does it own?
- Why is it a separate node?
- Which interfaces does it own?
- What state does it own?
- Which other nodes overlap?
- Which features depend on it?
- What breaks if it is removed?
- Is ROS2 the right boundary?

## Topic contract

For every interface record:

- name
- type
- semantic owner
- publishers/subscribers or servers/clients
- purpose
- what it must not mean
- static evidence
- runtime evidence

Identity-related names such as person, speaker, face and identity deserve explicit contracts. They must not silently mix detection, known identity, current speaker and current user.

## Decision rule

The audit produces evidence and review candidates. Merge, delete, rename, split and keep decisions are made only after checking runtime behavior, tests, ADRs and feature acceptance evidence.

## Runtime diagrams

`tools/architecture_runtime_graph.py` turns `runtime.json` + `inventory.json` into:

- `runtime-graph.md` — job summary: container map (one box per Docker container, colour = Pi, arrows = topics between containers), one collapsible diagram per container, node placement evidence table.
- `runtime-overview.mmd` — the container map alone.
- `runtime-graph.mmd` — full node-level graph.

Readability rules:

- Placement comes only from repo evidence: compose `command`/`entrypoint` → start script → `ros2 run` / `ros2 launch` / `-r __node:=` → launch file `name=` / `setup.py` entry point. Library-hosted nodes (costmaps, ros2_control controllers) follow their host node; `/rtabmap/*` follows the namespace. A node without evidence is shown as "unplaced", never guessed.
- Hidden as noise and listed in the report: `/tf`, `/tf_static`, `/diagnostics`, `/rosout`, `/parameter_events`, `/clock`, `/robot_description`, `/bond`, `*/transition_event`; tf2 `transform_listener_impl_*`, `launch_ros_*`, nav2 `*_rclcpp_node` helpers.
- A topic with one publisher and one subscriber is an arrow label; a shared topic is a green pill.

## Class metrics

`tools/architecture_class_metrics.py` measures production classes (tests and `scripts/example_*`/`test_*` excluded) and writes `class-metrics.json` / `class-metrics.md`:

- `loc`, `methods`, `attributes` (distinct `self.<name>` the class assigns);
- `wmc` — sum of method cyclomatic complexity, `max_cc` / `max_cc_method`;
- `tcc` — Tight Class Cohesion: share of method pairs using a common attribute the class assigns;
- `responsibilities` — LCOM4: groups of methods linked by shared state or `self.method()` calls. Dunders and methods that touch no own state (`stateless_methods`: Protocol/ABC stubs, helpers) are left out;
- `ros_endpoints` — `create_publisher/subscription/service/client/timer` calls;
- `god_class` — `wmc >= 47 and tcc < 1/3 and loc >= 500` (Lanza & Marinescu without ATFD);
- `test_refs` — test files that mention the class name. A proxy, not coverage;
- `line_coverage` — only with `--coverage <coverage.py JSON>`; otherwise `null`, never guessed.

Metrics are review candidates: a god class or several responsibilities is a reason to ask the node review questions above, not an automatic split.

## Runtime findings

`tools/architecture_runtime_findings.py` reads `runtime.json` (the live graph captured by `L: Architecture Audit`) and writes `runtime-findings.json` / `runtime-findings.md`:

- `type_mismatch` — endpoints of one topic use different message types, so data never flows;
- `qos_incompatible` — BEST_EFFORT publisher → RELIABLE subscriber, or VOLATILE → TRANSIENT_LOCAL;
- `dead_input` / `dead_output` — subscribers without a publisher / publishers without a subscriber;
- `multiple_writers` — several nodes publish one topic;
- `duplicate_publishers` — one node holds several publishers on one topic.

Endpoints are re-parsed from the raw `ros2 topic info --verbose` text. Infra topics and helper nodes (including the `_ros2cli_daemon_*` the audit itself starts) are ignored. `scope: repo` means one of the endpoint nodes is created in `src/`, or the topic is declared in our code; the rest is `external` (nav2, rtabmap, drivers).

`architecture/runtime-baseline.json` lists known findings. Findings that are not in it are marked 🆕; keys that disappeared are listed as resolved. The `fail_on_new` input of `L: Architecture Audit` fails the run on new findings, after the reports are published. A snapshot shows only what was connected at capture time, so a node that was down produces findings too.
