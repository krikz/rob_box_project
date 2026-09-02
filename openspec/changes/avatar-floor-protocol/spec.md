# avatar-floor-protocol — wire contract for avatar supervisor floor services

> Spec delta (ADR-0039): ADDED Requirements + scenarios for the typed
> floor protocol between avatar clients (Quest, Telegram) and the
> `avatar_supervisor` ROS 2 node (ADR-0028 §4.3).

## Purpose

Replace the previous `std_srvs/Trigger`-based service hack with a typed
IDL. Before this card:

- `std_srvs/Trigger.Request` is empty in real ROS 2 (no fields before
  `---`), so `client_id`/`floor` were smuggled through JSON in
  `request.data` (or in `Trigger.Request.data` if the service was
  re-typed). That worked only inside the supervisor process — inter-process
  and inter-language clients could not talk to `avatar_supervisor` at all.
- The supervisor itself documented this as "honest technical debt"
  (ADR-0028 §4.2) and promised the AV-5/AV-12 card as the real fix.

After this card, all three floor services (`acquire_floor`,
`release_floor`, `set_avatar_mode`) are typed contracts on
`rob_box_supervisor_msgs`:

- `AcquireFloor.srv` — `Request {client_id, floor}`, `Response {granted, held_by, reason, applied}`
- `ReleaseFloor.srv` — `Request {client_id, floor}`, `Response {success, reason, applied}`
- `SetAvatarMode.srv` — `Request {client_id, mode}`, `Response {success, mode, reason, applied}`

`floor` is **wire-name** short form: `"teleop"` or `"voice"` (matches
`meta-quest-api.md` §3 / §5.1). The supervisor maps wire names to
LockManager internal names (`teleop_floor` / `voice_floor`) so existing
`LockManager`-based dead-man and FSM synchronization keeps working
unchanged (ADR-0028 §4.2).

## ADDED Requirements

### Requirement: AcquireFloor request MUST carry `client_id` and `floor` as typed fields

The service `acquire_floor` (`rob_box_supervisor_msgs/srv/AcquireFloor.srv`)
MUST take a `Request` with two string fields:

- `client_id` — non-empty caller identity, conventionally
  `"<transport>:<session>"` (`"quest:<uuid>"`, `"telegram:<chat_id>"`,
  `"cli:operator"`). Empty `client_id` is rejected by the server as
  `granted=false, reason="bad_request"`.
- `floor` — exactly one of `"teleop"` or `"voice"`. Any other value is
  rejected with `granted=false, reason="bad_request"`.

The server MUST NOT parse JSON from `request.data` /
`request.message`: the previous W3-2 JSON fallback has been removed
(card AV-12, R13). Requests with no `client_id`/`floor` attributes are
classified as `bad_request`.

#### Scenario: free floor, valid request

- GIVEN an `avatar_supervisor` in `mode=active` and
  `LockManager` holds no floor
- WHEN client calls `acquire_floor(client_id="quest:abc", floor="teleop")`
- THEN the response is `granted=true`, `held_by="quest:abc"`,
  `applied=true`, `reason="granted"`
- AND `LockManager` records `quest:abc` as the `teleop_floor` holder.

#### Scenario: floor held by another client

- GIVEN `LockManager` already holds `teleop_floor` for `quest:abc`
- WHEN client `telegram:42` calls `acquire_floor(client_id="telegram:42", floor="teleop")`
- THEN the response is `granted=false`, `held_by="quest:abc"`,
  `applied=true`, `reason="held_by_other"`
- AND the supervisor does NOT mutate `LockManager`.

#### Scenario: invalid request — empty `client_id`

- WHEN any client calls
  `acquire_floor(client_id="", floor="teleop")` or
  `acquire_floor(client_id="quest", floor="lidar_floor")`
- THEN the response is `granted=false`, `held_by=""`, `applied=false`,
  `reason="bad_request"`
- AND the supervisor does NOT reach `LockManager`.

### Requirement: ReleaseFloor request MUST carry `client_id` and `floor`

The service `release_floor` MUST take a typed `Request` of the same
shape as `acquire_floor`. The response confirms
`success=true, applied=true, reason="released"` when the holder matches
the `client_id`, or
`success=true, applied=false, reason="permission_denied"` when it does
not.

#### Scenario: release by owner

- GIVEN `LockManager` holds `teleop_floor` for `quest:abc`
- WHEN `quest:abc` calls `release_floor(client_id="quest:abc", floor="teleop")`
- THEN the response is `success=true, applied=true, reason="released"`
- AND the floor becomes free for the next acquire.

#### Scenario: release by wrong client

- GIVEN `LockManager` holds `teleop_floor` for `quest:abc`
- WHEN `telegram:42` calls `release_floor(client_id="telegram:42", floor="teleop")`
- THEN the response is `success=true, applied=false, reason="permission_denied"`.

### Requirement: SetAvatarMode request MUST carry `client_id` and `mode`

`set_avatar_mode` (`rob_box_supervisor_msgs/srv/SetAvatarMode.srv`)
takes a `Request` with:

- `client_id` — string (may be empty; FSM events without a holder e.g.
  `force_off` are allowed)
- `mode` — FSM event name (NOT target mode — see ADR-0028 §4.1 mermaid).
  Allowed values mirror `core.fsm.EVENT_*` (see supervisor docstring):
  `telegram_acquire_floor`, `quest_acquire_floor`,
  `quest_acquire_floor_teleop_only`, `telegram_acquire_voice_floor`,
  `quest_acquire_full_floor`, `telegram_release`, `quest_release`,
  `quest_release_teleop`, `telegram_release_voice`, `both_release`,
  `force_off`.

Response fields:

- `success` — bool
- `mode` — current avatar mode after the transition (echoes
  `ModeManager.mode.value`). Lets clients observe FSM without
  subscribing to `/avatar/state`.
- `reason` — `"applied"`, `"conflict"`, `"invalid_event"`,
  `"bad_request"`, or `"monitor"` (in monitor mode).
- `applied` — false in monitor mode (`applied=false, reason="monitor"`).

#### Scenario: valid FSM event in active mode

- GIVEN `avatar_supervisor` in `mode=active`, FSM starts at `off`
- WHEN client calls `set_avatar_mode(client_id="telegram:1", mode="telegram_acquire_floor")`
- THEN the response is `success=true, applied=true, mode="telegram_active", reason="applied"`
- AND `ModeManager.mode == Mode.TELEGRAM_ACTIVE`.

#### Scenario: invalid event name

- WHEN client calls `set_avatar_mode(client_id="x", mode="not_a_real_event")`
- THEN the response is `success=true, applied=false, mode=<previous>, reason="invalid_event"`.

### Requirement: monitor mode MUST accept but NOT mutate

If `avatar_supervisor` parameter `mode != "active"` (i.e. `monitor`):

- `acquire_floor` returns `granted=false, held_by="", applied=false, reason="monitor"`
- `release_floor` returns `success=true, applied=false, reason="monitor"`
- `set_avatar_mode` returns `success=true, applied=false, mode=<current_fsm_mode>, reason="monitor"`
- LockManager and ModeManager are NOT mutated.

This satisfies ADR-0028 §4.5: the supervisor can be deployed safely as
a monitor before clients switch to the typed contract.

### Requirement: pre-IDL compatibility fallback (fail-safe)

If the `rob_box_supervisor_msgs` Python module is unavailable at startup
(e.g. CI mock-rclpy, partially-built workspace):

- The supervisor MUST log a single `WARN` line at startup identifying
  the fallback (ADR-0028 §4.5).
- The three services are declared on `std_srvs/Trigger`, accepting the
  W3-2/W3-4 contract: empty `Trigger.Request`, JSON in `request.data`,
  response in `response.message`.
- The supervisor MUST stay in `mode=monitor` regardless of the
  requested mode, because the typed `active` path requires the IDL.

The fallback path is documented as backwards-compatible for legacy test
clients; new clients SHOULD target the typed IDL.

### Requirement: dead-man timeout is exactly ADR-0028 §6 Q4

If a client holding `teleop_floor` does not send a `TeleopHeartbeat`
within 500 ms, `LockManager` MUST release the floor. The next
`acquire_floor` from any other client MUST succeed against the
released floor. See `core/test_locks.py::test_dead_man_releases_floor_*`
(W3-2 acceptance).

`TeleopHeartbeat.msg` is defined in `rob_box_supervisor_msgs/msg/` for
AV-13 to use; the typed service contract for AV-13 is out of scope for
this card.

## MODIFIED Requirements

(none)

## REMOVED Requirements

### Requirement: JSON-in-`request.data` fallback for floor services (REMOVED)

The previous W3-2 fallback that parsed JSON strings from
`std_srvs/Trigger.request.data` (or `.message`) has been removed.
Documented since W3-2 as "honest technical debt" — never worked
inter-process — and as the source of the "everything-works" illusion
that masked the missing wire contract (ADR-0028 §4.2).

The fallback only re-appears in the pre-IDL compatibility path
(fail-safe `monitor`-mode fallback above) for backwards-compatible test
clients. New clients MUST NOT use JSON-in-data.

## Why

See [design.md](design.md) for the architectural choices (separate
`ament_cmake` IDL package, dual-mode fallback, etc.) and
[../../../docs/adr/0028-avatar-supervisor.md](../../../docs/adr/0028-avatar-supervisor.md)
§4.2 §4.3 §4.5 §6 Q4 for the underlying supervisor protocol.
