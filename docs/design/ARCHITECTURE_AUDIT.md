# ROB-BOX Architecture Audit

The audit is a read-only observability layer. Its first question is what exists, before asking whether it should exist.

## Chain

container → node → interface → class → responsibility → capability → feature

Reverse interface trace:

topic/service/action → publishers/subscribers/servers/clients → node → capability

## First milestone

tools/architecture_audit.py deterministically scans Docker Compose and Python source and produces:

- architecture/inventory.json
- architecture/inventory.md

It records containers/services, ROS package directories, Python classes, detected ROS node classes, and literal ROS publisher/subscriber/service/client/action declarations.

It does not decide that a node or class should be deleted or merged.

## Later layers

1. Map compose services to launch scripts and actual ROS nodes.
2. Ingest runtime ros2 node/topic/service/action snapshots.
3. Compare static and runtime graphs.
4. Overlay capabilities and the existing feature matrix.
5. Detect duplicate responsibilities, shadow implementations, orphan interfaces, and semantic collisions.
6. Produce architecture-debt findings for human review.

## Cadence

- Every PR: deterministic static scan + architecture diff.
- Nightly: repository snapshot; optionally compare with runtime snapshot.
- Weekly / before release: AI-assisted semantic review using inventory, feature matrix, ADRs, tests/E2E evidence and recent changes.

Human engineers remain responsible for architectural decisions such as “is this node necessary?” and “should these responsibilities be merged?”


## Local commands

Static inventory:

    python tools/architecture_audit.py

Structural findings:

    python tools/architecture_findings.py architecture/inventory.json

Runtime snapshot (run on a machine with ROS 2 and the target graph visible):

    python tools/architecture_runtime_snapshot.py

The runtime snapshot is evidence from an actual ROS graph. It must not be generated on a generic CI runner and presented as robot evidence.
