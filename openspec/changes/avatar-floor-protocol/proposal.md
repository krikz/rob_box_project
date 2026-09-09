## Why

Wire contract for `acquire_floor` / `release_floor` / `set_avatar_mode`
on `avatar_supervisor` is currently `std_srvs/Trigger` + JSON-in-data
(ADR-0028 §4.2 "honest technical debt"). The previous W3-2 card
documented that this only worked in-process; inter-process and
inter-language clients could not talk to the supervisor at all.

This change moves the three services to typed IDs from a new
`rob_box_supervisor_msgs` `ament_cmake` package, retires the JSON
fallback (R13), and freezes the contract so Quest/Telegram clients
can be switched on a stable surface (AV-15, out of scope).

## What changes

- New `src/rob_box_supervisor_msgs/` IDL package (3 srv, 3 msg).
- `supervisor_node.py` registers services on the typed IDL; falls
  back to `std_srvs/Trigger` only if the IDL Python module is
  unavailable at startup (CI mock-rclpy).
- `_extract_floor_request` / `_extract_avatar_mode_request`: JSON
  parsing removed; `bad_request` semantics preserved.
- Response adapter: typed fields (`granted`, `held_by`, `reason`,
  `applied`) instead of `Trigger.message` JSON.
- Tests: 11 new acceptance tests cover empty client_id,
  held_by_other, idempotent reacquire, dead-man trip, monitor-mode
  behaviour, no-JSON regression. **112 passed, 0 failed** in mock
  environment.
- `docker/vision/{voice_assistant,supervisor}/Dockerfile`: copy and
  `colcon build` of the new IDL package.
- OpenSpec `specs/avatar-floor-protocol/spec.md` + `design.md`.

## Impact

- `rob_box_quest`, `rob_box_telegram` clients: no breakage for the
  W3-2 contract until they migrate (AV-15, AV-16).
- `LockManager` internals: untouched.
- CI: no new dependencies. Mock-rclpy conftest extended for the IDL
  types so unit tests still pass without a full `colcon build`.

## Out of scope

- AV-13 (`/teleop_heartbeat` wire usage — IDL is here, integration
  is separate).
- AV-14 (`/avatar/state` typed alternative — IDL is here, parallel
  publication is separate).
- AV-15 (Telegram client switch).
- AV-16 (Meta-Quest webxr wire-format `0x30..0x33` payload).

Refs: krikz/rob_box_project#1904 (AV-12),
[ADR-0028 §4.3](../../../docs/adr/0028-avatar-supervisor.md),
[meta-quest-api.md §3 §5.1](../../../docs/architecture/meta-quest-api.md).
