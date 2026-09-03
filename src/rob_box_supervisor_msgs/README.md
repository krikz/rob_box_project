# rob_box_supervisor_msgs

ROS 2 IDL package for the Avatar Supervisor (see [ADR-0028](../../docs/adr/0028-avatar-supervisor.md)).

## What it provides

Typed services and messages that retire the previous `std_srvs/Trigger`
+ JSON-in-`request.data` transition hack:

| Type | Kind | Used for |
|------|------|----------|
| `AcquireFloor.srv` | service | Replace `acquire_floor` (Trigger) — typed `client_id`/`floor` |
| `ReleaseFloor.srv` | service | Replace `release_floor` (Trigger) — typed `client_id`/`floor` |
| `SetAvatarMode.srv` | service | Replace `set_avatar_mode` (Trigger) — typed `client_id`/`mode` (FSM event) |
| `TeleopHeartbeat.msg` | topic | `/teleop_heartbeat` (AV-13 dead-man ping) |
| `FloorState.msg` | msg | Single-floor snapshot, nested inside `AvatarStateMsg` |
| `AvatarStateMsg.msg` | msg | Typed alternative to `/avatar/state` (Phase 2, alongside msgpack) |

## Build

```bash
colcon build --packages-select rob_box_supervisor_msgs
```

Builds with `ament_cmake` + `rosidl_default_generators`. The user-facing
Python types live in `rob_box_supervisor_msgs.srv.{AcquireFloor,ReleaseFloor,SetAvatarMode}`
and `rob_box_supervisor_msgs.msg.{TeleopHeartbeat,FloorState,AvatarStateMsg}`.

## Why a separate package?

`rob_box_supervisor` is `ament_python` and cannot generate rosidl
interfaces on its own (IDL generation requires `ament_cmake` +
`rosidl_default_generators`). Splitting the IDL into its own package
keeps the supervisor pure Python while still exposing a typed wire
contract to external clients (`rob_box_quest`, `rob_box_telegram`).
