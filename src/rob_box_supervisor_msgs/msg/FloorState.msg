# FloorState.msg — one floor's ownership snapshot (ADR-0028 §4.3 /avatar/state)
#
# Mirrors the dataclass rob_box_supervisor.core.state.FloorState (msgpack
# payload). Empty client_id means "floor is free" — used for both
# teleop_floor and voice_floor in the same /avatar/state frame so clients
# can tell teleop-occupied vs voice-occupied at a glance.

string client_id          # "" == floor is free
uint64 since_ms           # when this client_id acquired the floor
uint64 last_heartbeat_ms  # most recent heartbeat; dead-man check uses this
