# Root cause analysis: bug(teleop) #1344

## Evidence from live robot (10.1.1.10, containers were up 6h, logs pulled before they were removed ~18:00 MSK)

### Claim "поворот приходит сам при нейтральном руле" — MISDIAGNOSED
- ros2-control log 1786876408.849: wheels ramped left=+0.0333→0.5531, right=-0.0333→-0.5531 m/s
  at exactly angular.z max_acceleration (3.0 rad/s^2 → 0.0333 m/s per 20ms tick), target = 1.0 rad/s (max_angular_speed).
- SBUS teleop log at the same instant: Ch4 (yaw) = 276 → 178 → 408 → 1811 (HARD OVER, not neutral!).
  Ch4=178 → axes[3]=-0.993 → angular.z=-0.99. The architect read Ch4=992 from a LATER packet (6444)
  and assumed the stick was neutral the whole time. The "spontaneous rotation" was the operator's own yaw input.

### Claim "газ не доходит до моторов" — CONFIRMED
- 1786876444-6448: SBUS Ch2=1811 (full gas), ARM=1792 (armed), Ch4=992 (neutral), 5 s.
- ros2-control log 1786876430-6469: COMPLETELY EMPTY — no sendSpeed, no "Motors activated".
  Motors were relaxed since 1786876421.6 and gas never woke them.
- Same pattern at 1786879279: gas held from 9270, motors activated only at 9279.6 (9 s late) then worked.

### The dead window: 1786876421.6 → 1786879279.6 (47.6 min)
- Node ALIVE throughout: SBUS packets logged continuously, /joy published (architect confirmed),
  params correct (ch_pitch=1, ch_yaw=3, max_angular=1.0), code identical to repo (diffed), 0 exceptions/tracebacks.
- ARM stable HIGH (1792) 1786876345→9803 (no LOW events in between — verified from ARM transition log).
- So cmd_vel_joy simply STOPPED reaching the controller for 47.6 min while the node was armed and alive,
  then self-healed. No restart, no ARM toggle, no zenoh reconnect log at recovery.
- Most consistent explanation: RELIABLE-QoS publish from joystick node blocked on a stalled subscriber
  (twist_mux subscribes SystemDefaultsQoS=RELIABLE; rmw_zenoh_cpp transport glitch) → executor froze →
  cmd_vel_joy + /joy stopped → twist_mux joystick source (timeout 0.5 s) fell through.

### Safety design flaw (the "поворот приходит сам" danger, real)
- twist_mux joystick timeout=0.5s. ANY gap >0.5s in cmd_vel_joy (stall, ARM glitch, crash)
  switches control to lower-priority sources: web(50), voice(25), nav2(10).
- Nav2 is ALWAYS running on the robot. If Nav2 is navigating when the joystick source drops,
  the robot moves autonomously — operator sees "поворот приходит сам".

## Fix
1. twist_mux lock `joystick_lock` (topic /joystick_lock, std_msgs/Bool, timeout 0.5, priority 100):
   - joystick node publishes True at 20 Hz while ARMED → blocks web/voice/nav2 (isMasked: priority < 100).
   - False while disarmed → lower sources work normally.
   - FAIL-SAFE: if joystick node stalls/crashes, lock EXPIRES → isLocked()=true (hasExpired()) →
     lower sources stay BLOCKED → robot cannot move autonomously during a joystick fault.
2. Diagnostic heartbeat: log published cmd_vel_joy every ~2 s while armed, so the next stall
   is visible in logs within seconds (SBUS keeps flowing while heartbeat stops).
3. Unit tests for the armed/lock logic (repo has no tests for rob_box_teleop).

## Other notes
- All main containers (nav2, ros2-control, teleop, twist-mux, ...) were REMOVED on 10.1.1.10
  around 17:5x-18:00 MSK (docker ps showed only zenoh-router) — presumably operator/architect action.
  Not restarted by this worker.
- Leftover `ros2 topic echo /cmd_vel_joy` (PID 7392, started 11:21, architect's session) was observed.
