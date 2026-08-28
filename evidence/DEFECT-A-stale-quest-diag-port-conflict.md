# DEFECT A — stale `quest-diag` orphan shares `*:8443` with `rob-box-quest`

Task: t_0b926b23 (bring up rob_box_quest on Vision Pi)
Host: VisionPi 10.1.1.21 (Linux 6.8.0-1060-raspi aarch64), Docker 29.1.3
Captured: 2026-08-28 ~13:20-13:25 UTC by devops agent (SSH ros2@10.1.1.21)

**Status: NOT FIXED — requires a human action on the robot. Deliberately not
touched, per remote-robot-inspect: "НИКОГДА не перезапускать сервисы без явной
просьбы пользователя". Nothing was stopped, restarted or removed on the Pi.**

## Symptom

`https://<vision-pi>:8443/` returns HTTP 502 roughly 40% of the time, at random.
`:443` is 100% clean. Measured on the Pi itself (20 sequential requests each):

    GET / on :443   -> 200 x20                       (0% error)
    GET / on :8443  -> 200 200 200 502 502 502 502 200 502 502
                       200 200 200 200 200 200 502 502 200 200   (40% error)
    GET /quest on :8443 -> 400 400 502 400 400 502 400 400 502 400

## Root cause

TWO containers run `caddy run` in `network_mode: host`, and both bind `*:8443`:

    $ ss -ltn | grep -E ':443 |:8443'
    LISTEN 0 4096 *:8443 *:*      <-- two independent sockets,
    LISTEN 0 4096 *:8443 *:*          same port (SO_REUSEPORT)
    LISTEN 0 4096 *:443  *:*      <-- only the NEW container listens here

    $ pgrep -af caddy
    1306860 caddy run --config /etc/caddy/Caddyfile   <-- quest-diag (Aug 26)
    3866600 caddy run --config /etc/caddy/Caddyfile   <-- rob-box-quest (Aug 27)

Caddy sets `SO_REUSEPORT`, so the kernel load-balances new connections between
the two listeners. Whichever process accepts the connection serves it.

    $ docker inspect quest-diag --format '...'
    net=host status=running health=unhealthy started=2026-08-25T23:29:51Z
    restartPolicy=no  autoRemove=false
    image=10.1.1.249:5000/krikz/rob_box:quest-humble-test (imageID a1bd69397ba3, now untagged)
    cmd=["/scripts/start_quest.sh"]

`quest-diag` is an ad-hoc diagnostic container (name is not in any compose file,
no compose labels) left running since Aug 26. It carries the OLD Caddyfile:

    :8443 {                        # no :443
        root * /srv/quest_static   # file_server at top level
        handle /healthz { reverse_proxy localhost:8765 }
        reverse_proxy localhost:8765 { transport http { versions h2c 1.1 } }
    }

Its catch-all `reverse_proxy localhost:8765` sends `/` to voice-action-server,
which has no route for `/` -> Caddy reports 502. And because its quest_node
predates commit f9bf3f68, it has NO `ws_port` override:

    quest-diag  : ros2 run rob_box_quest quest_node ... (no -p ws_port)
    rob-box-quest: ros2 run rob_box_quest quest_node ... -p ws_port:=8766

So the 502/400 mix on `/quest` is the two Caddyfiles disagreeing about the
upstream. `:443` is unaffected only because the old Caddyfile never listened there.

## Second-order effect: duplicate emergency-stop publisher

`quest-diag`'s quest_node is still alive on `ROS_DOMAIN_ID=0` and still
publishing. Its watchdog trips continuously (no Quest client connected):

    $ docker logs quest-diag --tail 8
    [WARN] [quest_node]: 🛑 Watchdog tripped — emergency stop (Quest client silent)
    [WARN] [quest_node]: 🛑 EMERGENCY STOP from Quest client — publishing cmd_vel_emergency
    ... repeating ~10x/second

`rob-box-quest` logs the identical pattern (~2s cadence, see `quest.log`).
`/cmd_vel_emergency` is consumed by `twist_mux`, i.e. this reaches the drive
chain. Note ROS2 currently shows only ONE `/quest_node` because both containers
use `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and identical node names — the
name collides, so the duplicate is invisible to `ros2 node list`:

    $ ros2 node list | grep -c quest_node
    1
    $ ros2 topic info -v /cmd_vel_emergency
    Publisher count: 1     <-- only the one this container's session sees
    Subscription count: 1  <-- twist_mux

An unmanaged container publishing emergency-stop to a live drive topic is a
safety-relevant condition, which is why this is reported rather than silently
cleaned up.

## Recommended fix (needs товарищ Шифу / human on the robot)

`quest-diag` has `restartPolicy=no`, so it will NOT come back after a reboot —
but it survives until then. Suggested action on the Pi:

    docker stop quest-diag && docker rm quest-diag
    # then confirm a single listener remains:
    ss -ltn | grep -E ':443 |:8443'
    pgrep -af caddy
    for i in $(seq 1 20); do curl -sk -o /dev/null -w '%{http_code} ' https://localhost:8443/; done

Expected after cleanup: one `*:8443` socket, one caddy pid, 20x `200`.

Not done here on purpose: stopping a container on live robot hardware is a
human decision, and `quest-diag` may be somebody's in-progress debug session.
