# TeleopHeartbeat.msg — dead-man ping from teleop client (ADR-0028 §4.4 / §6 Q4)
#
# Client sends this on /teleop_heartbeat at >= 10 Hz while holding the
# teleop_floor. Without a fresh heartbeat for > 500 ms the supervisor
# auto-releases the floor (LockManager DEAD_MAN_TIMEOUT_MS).

string client_id
uint64 ts_ms       # client-side monotonic-ish timestamp; informational
uint32 seq         # monotonic per-client sequence, lets supervisor spot gaps
