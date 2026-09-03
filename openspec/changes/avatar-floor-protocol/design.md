# avatar-floor-protocol — design rationale

Companion to [spec.md](spec.md). Why we did what we did for
`rob_box_supervisor_msgs` and the typed floor services on top.

## Why a separate `ament_cmake` package?

`rob_box_supervisor` is built with `ament_python` (its `package.xml`
declares `<build_type>ament_python</build_type>`). ROS 2 IDL generation
requires `ament_cmake` + `rosidl_default_generators` because the IDL
artifacts (`.idl` files, language-specific stubs) are produced by
`rosidl_generate_interfaces()` inside a CMake build.

Options we considered:

1. **Convert `rob_box_supervisor` to `ament_cmake`**. Rejected: it
   would force every Python module in the package through CMake
   configuration (each `.py` becomes a CMake target), would break the
   already-working `console_scripts` entrypoint pattern that ships the
   `ros2 run rob_box_supervisor supervisor_node`, and would block
   Python-only contributors (e.g. the `core/` pure-Python building
   blocks) from the lightweight iterate-test loop.
2. **Two packages**: pure-Python `rob_box_supervisor` (node + logic)
   depends on `rob_box_supervisor_msgs` (CMake IDL). Chosen. Pattern
   already used in this repo: `rob_box_quest` (Python) depends on
   `rob_box_perception_msgs` (CMake IDL) — see `src/rob_box_perception_msgs/`.
3. **No IDL, keep JSON-in-data**. Already rejected in W3-2
   (ADR-0028 §4.2): "honest technical debt that only worked in-process".

The split adds one extra package to the workspace but keeps each
component doing one job:

- `rob_box_supervisor_msgs` owns the wire types (srv / msg) and is
  versioned independently — bumping fields only requires re-publishing
  the IDL, not the supervisor node.
- `rob_box_supervisor` keeps the pure-Python build + `pytest` loop
  intact: no CMake step is needed to run unit tests. CI without
  `ament_cmake` infrastructure (Debian mock-rclpy) still works because
  the package degrades gracefully (see fallback below).

## Why drop the `request.data` JSON fallback?

W3-2 used `std_srvs/Trigger` (whose Request has **no** fields in real
ROS 2) and smuggled `client_id`/`floor` as JSON in `request.data`.
That contract was:

- **Process-local only**: the supervisor could read its own request
  because it had the JSON via `getattr`, but cross-process or
  cross-language clients (ros2 CLI, Zenoh, native C++) cannot send
  arbitrary attribute payloads on a `Trigger.Request`.
- **Invisible breakage**: "looks like it works" in unit tests that
  constructed synthetic requests, but never worked over the wire.
  `core/test_locks.py` passed (LockManager works in-process); the
  external-contract test was impossible to write.
- **Two parallel code paths** (`_extract_floor_request` and
  `_extract_avatar_mode_request` tried attribute-then-JSON, keeping
  the contract-split alive and ensuring future contributors would
  inherit the confusion).

AV-12 retires both paths by switching the three services to typed
IDL. The fallback below is a transient compatibility layer, not a
restored fallback for non-trivial clients.

## Why the dual-mode fallback (`typed` ↔ `std_srvs/Trigger`)?

CI mock-rclpy does not have `rob_box_supervisor_msgs` built — it
provides only `rclpy`/`std_msgs`/`std_srvs` mocks. The unit-test
environment must still be able to instantiate the supervisor to
exercise its `_acquire_floor_logic` and friends (these don't need IDL).

Concretely `_try_load_supervisor_msgs()` returns a dict whose values
are `None` when `from rob_box_supervisor_msgs.srv import …` raises
`ImportError`. The node flips `_use_typed_floor_services = False` and
delegates to legacy `std_srvs/Trigger` callbacks that produce the
W3-2 JSON-in-message response. The active-mode path is **disabled**
in fallback (always monitor) so we don't accidentally trust a
contract that never made it to the wire.

Fail-safe posture (ADR-0028 §4.5): if the IDL is missing, the
supervisor is a safe observer and refuses to act on requests via
the fallback path.

## Why a wire-name ↔ LockManager-name mapping?

`meta-quest-api.md` §3 (frame types `0x31 ACQUIRE_FLOOR` /
`0x32 RELEASE_FLOOR`) and the `supervisor_acquire_floor` /
`supervisor_release_floor` JSON commands §5.1 use **wire-name**
shorthand `"teleop"` / `"voice"`. `core.locks.LockManager` (ADR-0028
§4.2) uses **domain-name** shorthand `"teleop_floor"` / `"voice_floor"`.

The IDL declares wire-name `floor: string` to match the client
contract; `_wire_to_lock_floor()` translates once at the request
boundary. This keeps:

- The IDL stable for clients (no breaking change vs. W3-2).
- `LockManager` unchanged — its test surface, dead-man semantics,
  conflict reporting and FSM-mirroring logic stay exactly as they
  were validated in W3-2 / W3-4.
- The mapping explicit and unit-testable (see
  `test_supervisor_node.py::TestAvatarSupervisorFloorLockManager`).

## Out of scope (next cards)

- AV-13 — `/teleop_heartbeat` topic: IDL is defined here
  (`TeleopHeartbeat.msg`), but wire-usage is a separate card.
- AV-14 — typed alternative to `/avatar/state` msgpack
  (`AvatarStateMsg.msg`). IDL declared; serializer-on-msgpack-and-typed
  parallel is the topic of AV-14.
- AV-15 — Telegram client switch to typed services.
- AV-16 — Meta-Quest webxr wire frames `0x30..0x33` for the new
  payload (already merged in `meta-quest-api.md` §3 / §5.1; this
  card's IDL matches).

## References

- [ADR-0028 §4.2 LockManager](../../../docs/adr/0028-avatar-supervisor.md)
- [ADR-0028 §4.3 ROS 2 API](../../../docs/adr/0028-avatar-supervisor.md)
- [ADR-0028 §4.5 Monitor mode](../../../docs/adr/0028-avatar-supervisor.md)
- [ADR-0028 §6 Q4 Dead-man timeout](../../../docs/adr/0028-avatar-supervisor.md)
- [meta-quest-api.md §3 / §5.1](../../../docs/architecture/meta-quest-api.md)
- [ADR-0039 OpenSpec integration](../../../docs/adr/0039-openspec-integration.md) (this change's home)
