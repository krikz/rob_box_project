# Action Protocol — API Reference

> Canonical source of truth: [`../schemas/action-protocol/openapi.yaml`](../schemas/action-protocol/openapi.yaml) (HTTP+JSON, OpenAPI 3.0.3).
> Protobuf alternative: [`../schemas/action-protocol/proto/v1/action-protocol.proto`](../schemas/action-protocol/proto/v1/action-protocol.proto).
> Architecture: [`../architecture/action-protocol.md`](../architecture/action-protocol.md).
> ADR: [`../adr/0011-action-protocol/README.md`](../adr/0011-action-protocol/README.md).

This page is a **human-readable index** for the action protocol HTTP
binding. The OpenAPI document is the authoritative contract — every
endpoint, request schema, and response schema below is mirrored there.
If this page disagrees with `openapi.yaml`, the YAML is correct.

## Base URL

| Environment | URL | Notes |
|---|---|---|
| Local sidecar (Phase 4 default) | `http://127.0.0.1:8765` | Set by `ACTION_SERVER_HOST`/`ACTION_SERVER_PORT` in `http_server.py`. |
| docker-compose service | `http://voice-action-server:8765` | Same host network (`network_mode: host`). |

`/healthz` is the only unauthenticated endpoint. Production deployments
**must** add an authentication reverse proxy in front (Nginx/Caddy/Traefik)
— the sidecar itself does not check tokens. See
[`../runbooks/action-server.md`](../runbooks/action-server.md) §"Security".

## Endpoints at a glance

| Method | Path | Purpose | Status codes |
|---|---|---|---|
| `GET` | `/healthz` | Liveness + readiness | `200`, `503` |
| `POST` | `/actions` | Submit a goal | `202`, `503` (shutting down) |
| `GET` | `/actions/{goal_id}` | Poll goal state, feedback, result | `200`, `404`, `503` |
| `POST` | `/actions/{goal_id}/cancel` | Cooperative cancel | `200`, `404` |

OpenAPI tags: `health`, `actions`. Every payload below is also in
`openapi.yaml` with full JSON-Schema validation.

## `GET /healthz`

Liveness probe. Docker healthcheck hits this every 10s.

**Response 200** (healthy):

```json
{
  "ok": true,
  "active_goals": 0,
  "shutting_down": false
}
```

**Response 503** (graceful shutdown in progress):

```json
{
  "ok": false,
  "active_goals": 3,
  "shutting_down": true
}
```

`active_goals` is exposed so monitoring can alert on stuck load before
shutdown even begins.

## `POST /actions`

Submit a new goal. The handler runs asynchronously; this endpoint returns
the assigned `goal_id` immediately.

**Request body:**

```json
{
  "goal": {
    "kind": "tts.synthesize",
    "params": {
      "text": "Привет, я робот.",
      "voice": "female_01",
      "format": "pcm_22050"
    }
  }
}
```

| Field | Type | Required | Notes |
|---|---|---|---|
| `goal.kind` | string | yes | Plugin registry key (e.g. `tts.synthesize`). New handlers add new keys without bumping the wire version. |
| `goal.params` | object | yes | Handler-defined JSON object. Strongly-typed params are convention; the wire layer does not enforce them. |
| `goal.goal_id` | string (UUID v4) | no | If omitted, the server assigns one. Provide one only to make `goal_id` idempotent across retries. |

**Response 202 Accepted:**

```json
{
  "goal_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
  "state": "accepted"
}
```

The goal is in `ACCEPTED` state until the handler is scheduled. There is no
synchronous transition to `RUNNING` — clients should poll `GET /actions/{id}`.

## `GET /actions/{goal_id}`

Poll the current state of a goal, including any feedback emitted by the
handler so far. This is the canonical way to drain feedback; there is no
push channel in Phase 4 (WebSocket push is deferred to Phase 5).

**Response 200:**

```json
{
  "goal_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
  "state": "running",
  "feedback": [
    { "progress": 0.25, "tts_partial_pcm_ms": 320 },
    { "progress": 0.75, "tts_partial_pcm_ms": 980 }
  ],
  "result": null,
  "error": null
}
```

For terminal states (`SUCCEEDED`, `FAILED`, `CANCELLED`), `result` or
`error` is set:

```json
{
  "goal_id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
  "state": "succeeded",
  "feedback": [ { "progress": 1.0 } ],
  "result": { "audio_uri": "file:///tmp/tts_xyz.wav", "duration_ms": 1840 },
  "error": null
}
```

`404` means the `goal_id` is unknown (never submitted or already evicted
from the in-memory dict after a long enough retention window — currently
the dict is unbounded; see runbook for memory hygiene).

## `POST /actions/{goal_id}/cancel`

Cooperative + task cancellation. Always returns `200` if the goal is
known, regardless of whether cancellation succeeded — this is intentional,
because the cancellation is observed asynchronously.

**Request body:** empty.

**Response 200:**

```json
{ "cancelled": true }
```

**Response 404:** `goal_id` unknown.

After a successful cancel, the next `GET /actions/{goal_id}` will report
`state: "cancelled"`. Handlers MUST check `cancelled.is_set()` at safe
points and MUST handle `asyncio.CancelledError`.

## State machine (HTTP)

```
                 POST /actions
   (none) ─────────────────────────► ACCEPTED ──► RUNNING ──► SUCCEEDED
                                        │            │   │
                                        │            │   └──► FAILED
                                        │            │
                                        │            └──► CANCELLED
                                        │                  ▲
                                        ▼                  │
                              POST /actions/{id}/cancel ───┘
```

A `503` on `POST /actions` indicates `shutting_down=true`; clients should
back off and retry against another instance (or wait for the sidecar to
restart).

## Protobuf binding

For non-Python clients or future gRPC transport, the same wire schema is
expressed in protobuf v3:

```proto
package rob_box_voice.action.v1;

service ActionService {
  rpc Submit (SubmitRequest)  returns (SubmitResponse);
  rpc GetStatus(StatusRequest) returns (StatusResponse);
  rpc Cancel  (CancelRequest)  returns (CancelResponse);
  rpc Health  (HealthRequest)  returns (HealthResponse);
}
```

See the full schema at
[`../schemas/action-protocol/proto/v1/action-protocol.proto`](../schemas/action-protocol/proto/v1/action-protocol.proto).
The HTTP endpoints map 1:1 to the four RPCs. Field names mirror the JSON
keys (snake_case in proto, snake_case in JSON).

## Versioning & compatibility

| Bump | When | Examples |
|---|---|---|
| **Major** | Breaking change to enum values or required fields | Renaming a state, removing a required JSON field. |
| **Minor** | Backward-compatible additions | New `ActionState` value appended, new optional field, new endpoint. |
| **Patch** | Documentation or non-wire clarifications | Comments, schema descriptions. |

Clients should ignore unknown fields in responses (per JSON convention) and
treat unknown `state` values as `RUNNING`-equivalent for retry purposes.

## Validation tooling

The OpenAPI document can be rendered with:

```bash
# Redocly CLI (renders HTML)
npx @redocly/cli preview-docs docs/schemas/action-protocol/openapi.yaml

# Swagger UI (Docker)
docker run -p 8081:8080 \
  -e SWAGGER_JSON=/api/openapi.yaml \
  -v "$PWD/docs/schemas/action-protocol:/api" \
  swaggerapi/swagger-ui
```

Contract tests live in
`src/rob_box_voice/rob_box_voice/action_server/test/test_action_server.py`.

## See also

- [`../architecture/action-protocol.md`](../architecture/action-protocol.md) — component diagram, lifecycle, PASTE integration
- [`../guides/action-server-plugin-getting-started.md`](../guides/action-server-plugin-getting-started.md) — registering a handler
- [`../runbooks/action-server.md`](../runbooks/action-server.md) — deploy / monitor / shutdown
