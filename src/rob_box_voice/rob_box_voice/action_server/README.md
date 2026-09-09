# Phase 4 action sidecar

The action protocol is implemented in `rob_box_voice.action_server` and is
transport-neutral. `ActionServer` owns goals, feedback, cooperative/task
cancellation, health state, and bounded graceful shutdown. A goal handler has
this signature:

```python
async def handler(goal, feedback, cancelled):
    feedback({"progress": 0.25})
    ...
    if cancelled.is_set():
        return None
    return {"audio_uri": "..."}
```

PASTE integration uses `PastePlanner`: `speculate(id, goal)` starts N+1 work
in a shadow queue; `commit(id)` consumes the result; `discard(id)` cancels
stale work. Speculative handlers MUST be side-effect free. Only `commit` may
publish/play audio or invoke an external tool.

## HTTP contract

The optional `create_app(server)` adapter exposes:

* `GET /healthz` -> `{ok, active_goals, shutting_down}` (503 while stopping)
* `POST /actions` with `{goal: {...}}` -> `202 {goal_id, state}`
* `GET /actions/{goal_id}` -> state, feedback, result, error
* `POST /actions/{goal_id}/cancel` -> cancellation acknowledgement

The existing ROS topic interface remains compatible during migration. Deploy
this adapter as a separate sidecar/process; do not put API credentials in the
request payload or logs. Container health checks should call `/healthz` and
SIGTERM should await `ActionServer.shutdown(timeout=2.0)` before exit.
