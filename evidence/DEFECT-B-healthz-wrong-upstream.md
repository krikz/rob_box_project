# DEFECT B — `/healthz` proxied to voice-action-server, not quest_node (FIXED)

Task: t_0b926b23
File: `docker/vision/quest/Caddyfile`
Introduced by: `f9bf3f68` "fix(quest): rebind WS to :8766 to avoid EADDRINUSE
against voice-action-server"
Fixed in: this branch (`z-devops/0b926b23-quest-bringup`)

## Symptom — a "красивый PASS" (ADR-0018 violation)

`GET /healthz` through Caddy returns 200 with the WRONG service's body.
Measured on VisionPi 10.1.1.21, 2026-08-28 13:23 UTC:

    via Caddy  https://localhost:443/healthz  -> {"ok": true, "active_goals": 0, "shutting_down": false}
    quest_node http://localhost:8766/healthz  -> {"status": "ok", "sessions_active": 0, "server_version": "0.1.0"}
    voice-act  http://localhost:8765/healthz  -> {"ok": true, "active_goals": 0, "shutting_down": false}

Caddy's answer is byte-identical to voice-action-server's, NOT quest_node's.

## Root cause

`quest` and `voice-action-server` both run `network_mode: host` on the Vision Pi,
so they share one port namespace:

    voice-action-server -> 127.0.0.1:8765   (ACTION_SERVER_PORT, compose line 283)
    quest_node          -> 0.0.0.0:8766     (WS_PORT, compose line 380)

Commit `f9bf3f68` moved quest_node off 8765 to stop the EADDRINUSE clash and
updated `handle /quest` to 8766 — but left `handle /healthz` on 8765. Since
voice-action-server happily answers 200 there, nothing looked broken.

Confirmed on the live container (env + processes):

    $ docker inspect rob-box-quest ... | grep -E '^WS_PORT|^QUEST_'
    WS_PORT=8766
    QUEST_HOST=quest.rob_box.local
    $ docker exec rob-box-quest pgrep -af rob_box_quest
    quest_node --ros-args -p quest_host:=quest.rob_box.local ... -p ws_port:=8766

Both endpoints come from ONE aiohttp app, so they can never legitimately live on
different ports — `src/rob_box_quest/rob_box_quest/server/ws_server.py`:

    def build_app(server: WSSServer):
        ...
        app.router.add_get("/healthz", healthz)
        app.router.add_get("/quest", quest_ws)

## Why this matters

`local_test/quest_smoke_lib.py::check_healthz` asserts `body["status"] == "ok"`.
voice-action-server's body has no `status` key, so the smoke section would report
a confusing FAIL for the wrong reason — while the real problem (quest_node's
health never being probed at all) stayed invisible. If quest_node were dead and
voice-action-server alive, `/healthz` would still return 200: the smoke test
would be reporting on a service it isn't testing.

Note the compose healthcheck does NOT cover this: it uses `pgrep -f 'caddy run'`
+ `pgrep -f 'ros2 run rob_box_quest'`, i.e. process liveness only, never an HTTP
probe. That is why the container reads `Up 21 hours (healthy)` while `/healthz`
was pointing at the wrong service the whole time.

## Fix

    handle /healthz {
    -   reverse_proxy localhost:8765
    +   reverse_proxy localhost:8766
    }

## Regression test — `local_test/test_quest_caddyfile.py`

Pure text-parsing of the Caddyfile, no network/Docker/ROS2 needed. 8 tests:

  * `/healthz` must not point at 8765 (voice-action-server)
  * `/healthz` and `/quest` must share one upstream (one aiohttp app)
  * Caddyfile upstream must equal compose's `WS_PORT=${QUEST_WS_PORT:-8766}`
  * `/quest` keeps `flush_interval -1` and must not regress to `h2c` (450aa895)
  * site block keeps both `:443` (Quest browser default) and `:8443` (smoke script)

RED verified by reintroducing the exact defect (`8766` -> `8765`):

    3 failed, 5 passed
    FAILED ...::test_healthz_does_not_point_at_voice_action_server
    FAILED ...::test_healthz_and_quest_share_one_upstream
    FAILED ...::test_caddyfile_upstream_matches_compose_ws_port

GREEN after restoring the fix:

    8 passed in 0.05s

Existing suite unaffected: `local_test/test_quest_smoke_lib.py` -> 17 passed, 1 skipped.

## Deployment note (NOT yet live on the Pi)

`Caddyfile` is baked into the image (`COPY docker/vision/quest/Caddyfile
/etc/caddy/Caddyfile`, Dockerfile line 65) — it is NOT a compose bind-mount.
So this fix does NOT take effect from a `docker restart`; the quest image must be
rebuilt and re-pulled:

  1. `L: Build Vision Pi Services` -> new `:quest-humble-test`
  2. `L: Deploy and Verify` -> pull + `docker compose up -d quest`
  3. verify: `curl -sk https://localhost:443/healthz` must return
     `{"status": "ok", "sessions_active": ..., "server_version": "0.1.0"}`

Post-deploy verification is a separate step and is honestly NOT claimed here:
the running container still serves the old (8765) Caddyfile as of this commit.
