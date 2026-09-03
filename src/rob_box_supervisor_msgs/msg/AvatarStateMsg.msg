# AvatarStateMsg.msg — /avatar/state payload (ADR-0028 §4.3)
#
# This is the typed counterpart of the existing msgpack-encoded
# /avatar/state (rob_box_supervisor.core.state.AvatarState). Phase 1 keeps
# the msgpack path for backward compatibility; this message gives Phase 2
# clients a typed alternative (latched, transient_local) without forcing
# a JSON / msgpack decode on every frame.
#
# Mirrors core.state.AvatarState fields exactly:
#   mode, teleop_floor, voice_floor, last_event, since_ms, version.

string      mode

# "" client_id == floor is free.
FloorState  teleop_floor
FloorState  voice_floor

# Most recent meaningful supervisor event (FSM transition, dead-man trip,
# mode conflict). Empty string when no event has happened yet.
string      last_event

uint64      since_ms

# Schema version of this message. Bumped on backwards-incompatible changes
# so older clients can detect and reject (see core.state.StateVersionError).
uint32      version
