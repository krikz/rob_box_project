# ROB-BOX Architecture Audit

## Evidence chain

container -> node -> ROS interface -> class -> responsibility -> capability -> feature

The reverse view is also required:

feature -> capability -> node -> publisher/subscriber/service/action

The tooling separates evidence from architectural decisions. A duplicate or unpaired interface is a review candidate, not an automatic refactor instruction.

## Audit layers

1. Static inventory: Compose services, ROS packages, Python classes, node classes, launch entries and ROS interfaces.
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

The `L: Architecture Audit` workflow runs on the self-hosted `rob-box` runner and uses passwordless SSH to the robot (`10.1.1.21` by default). GitHub-hosted runners are never used for live robot access.

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
