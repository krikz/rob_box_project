# specs/avatar-floor-protocol/spec.md — capability delta

This is the **authoritative capability delta** merged from
`openspec/changes/avatar-floor-protocol/`. Tracking:

- ADDED Requirements: avatar-floor-protocol (see below)
- MODIFIED Requirements: none
- REMOVED Requirements: JSON-in-request.data for floor services

## ADDED Requirements

### Capability: avatar-floor-protocol

The avatar_supervisor node (ADR-0028 §4.3) MUST expose three services
on the typed `rob_box_supervisor_msgs` IDL:

- `acquire_floor` (`AcquireFloor.srv`) — `Request {client_id, floor}`,
  `Response {granted, held_by, reason, applied}`.
- `release_floor` (`ReleaseFloor.srv`) — `Request {client_id, floor}`,
  `Response {success, reason, applied}`.
- `set_avatar_mode` (`SetAvatarMode.srv`) — `Request {client_id, mode}`,
  `Response {success, mode, reason, applied}`.

`floor` MUST be one of `"teleop"` or `"voice"`. `mode` MUST be a
known FSM event (see `core.fsm.EVENT_*`).

#### Requirement: typed requests only

The server MUST NOT accept JSON-in-data as a wire contract for any of
the three services. Requests with empty `client_id`/`floor`/`mode`
MUST be classified as `bad_request`.

#### Requirement: response reasons are stable strings

The supervisor MUST use the following reason strings verbatim so
clients can switch on them without locale hazards:

- `granted` — successful acquire (and idempotent same-client re-acquire).
- `held_by_other` — acquire denied because another `client_id` holds.
- `bad_request` — invalid input (empty client_id, unknown floor, etc.).
- `released` — release succeeded for the holder.
- `permission_denied` — release denied because caller is not the holder.
- `conflict` — FSM rejected the transition (current FSM state does not
  allow the requested event).
- `invalid_event` — event name is not in `core.fsm.EVENT_*`.
- `applied` — set_avatar_mode succeeded.
- `monitor` — server is in monitor mode; request accepted but not
  applied to LockManager/ModeManager.

#### Requirement: monitor mode is a no-op

When `mode=monitor`, all three services MUST accept requests but NOT
mutate `LockManager` or `ModeManager`. Responses MUST carry
`applied=false` and `reason=monitor`.

#### Requirement: fail-safe when IDL is unavailable

When `rob_box_supervisor_msgs` Python module is unavailable at
startup, the supervisor MUST:

1. Log a single `WARN` identifying the fallback.
2. Register the three services on `std_srvs/Trigger` with the W3-2
   JSON contract (transient compatibility for legacy test clients).
3. Force `mode=monitor` regardless of any requested active mode.

The fallback path is documented as backwards compatibility only and
is not part of the steady-state contract.

## REMOVED Requirements

- JSON-in-`request.data` parsing for floor services: REMOVED. The
  W3-2 W3-4 fallback has been retired (ADR-0028 §4.2 §R13).
