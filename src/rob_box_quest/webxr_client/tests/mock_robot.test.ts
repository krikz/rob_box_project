// Мок-робот симулятора (#3149) против НАСТОЯЩЕГО Connection: рукопожатие,
// подписки, стримы и команды идут по штатному wire-протоколу через шов
// `WebSocketCtor` — без форка логики соединения.

import { describe, it, expect, afterEach } from "vitest";
import { Connection } from "../src/wire/connection";
import { createMockWebSocketCtor, MockRobot, SIM_EMERGENCY_HOLD_MS } from "../src/dev/mock_robot";
import { parseLidar2d } from "../src/scene/lidar_payload";
import { parseMapFrame } from "../src/scene/map_payload";
import { parseRobotStatus } from "../src/scene/status_hud";
import { parseVoiceState } from "../src/ui/voice_state_indicator";
import { createDefaultWorld, isFree, ROBOT_RADIUS_M, stepRobot, createRobotState } from "../src/dev/sim_world";

function sleep(ms: number): Promise<void> {
  return new Promise((r) => setTimeout(r, ms));
}

async function until(cond: () => boolean, timeoutMs = 2000): Promise<void> {
  const t0 = Date.now();
  while (!cond()) {
    if (Date.now() - t0 > timeoutMs) throw new Error("timeout");
    await sleep(5);
  }
}

describe("MockRobot over the real Connection", () => {
  let conn: Connection | null = null;
  let robot: MockRobot | null = null;

  afterEach(() => {
    conn?.close();
    robot?.stop();
    conn = null;
    robot = null;
  });

  function connect(pin = "000000", robotPin: string | null = null) {
    robot = new MockRobot({ pin: robotPin });
    robot.start();
    const frames: Array<{ topic: string | undefined; payload: Uint8Array }> = [];
    const events: Array<Record<string, unknown>> = [];
    const errors: string[] = [];
    const states: string[] = [];
    let supervisorMode: string | null = null;
    let welcomeHeldBy: string | null | undefined;
    let sessionId = "";
    conn = new Connection(
      {
        url: "sim://mock",
        clientVersion: "test",
        pin,
        autoReconnect: false,
        WebSocketCtor: createMockWebSocketCtor(robot)
      },
      {
        onStateChange: (s) => states.push(s),
        onBinaryFrame: (sid, payload) => frames.push({ topic: conn!.getTopicForStream(sid), payload }),
        onJsonEvent: (ev) => events.push(ev as Record<string, unknown>),
        onError: (code) => errors.push(code),
        onSupervisorState: (st) => (supervisorMode = st.mode),
        onWelcome: (sid, _t, held) => {
          sessionId = sid;
          welcomeHeldBy = held;
        }
      }
    );
    conn.connect();
    return {
      frames,
      events,
      errors,
      states,
      get supervisorMode() {
        return supervisorMode;
      },
      get welcome() {
        return { sessionId, heldBy: welcomeHeldBy };
      }
    };
  }

  it("HELLO → WELCOME (v2, floor ours) + STATE_UPDATE", async () => {
    const h = connect();
    await until(() => conn!.getState() === "connected");
    expect(conn!.getNegotiatedVersion()).toBe("v2");
    expect(h.welcome.sessionId).toMatch(/^sim-/);
    expect(h.welcome.heldBy).toBe(h.welcome.sessionId);
    await until(() => h.supervisorMode !== null);
    expect(h.supervisorMode).toBe("avatar_present");
  });

  it("wrong PIN → auth_failed when the robot has a PIN", async () => {
    const h = connect("111111", "222222");
    await until(() => conn!.getState() === "auth_failed");
    expect(h.errors).toContain("AUTH_FAIL");
  });

  it("ping → pong gives an RTT", async () => {
    connect();
    await until(() => conn!.getRttMs() !== null, 3000);
    expect(conn!.getRttMs()).toBeGreaterThanOrEqual(0);
  });

  it("subscribed streams arrive with real encodings", async () => {
    const h = connect();
    await until(() => conn!.getState() === "connected");
    for (const t of ["lidar_2d", "map_2d", "robot_status", "voice_state"]) conn!.subscribe(t);
    await until(() => ["lidar_2d", "map_2d", "robot_status", "voice_state"].every((t) => h.frames.some((f) => f.topic === t)), 3000);
    const lidar = h.frames.find((f) => f.topic === "lidar_2d")!;
    expect(parseLidar2d(lidar.payload).header.n_points).toBe(360);
    const map = h.frames.find((f) => f.topic === "map_2d")!;
    const mf = parseMapFrame(map.payload)!;
    expect(mf.png).not.toBeNull(); // первый кадр подписки — полный
    expect(mf.robot).not.toBeNull();
    const st = parseRobotStatus(h.frames.find((f) => f.topic === "robot_status")!.payload)!;
    expect(st.mode).toBe("idle");
    expect(st.battery_pct).toBeGreaterThan(0);
    const vs = parseVoiceState(h.frames.find((f) => f.topic === "voice_state")!.payload)!;
    expect(vs.state).toBe("idle");
    // Без canvas (jsdom) видеокадров нет — и мок этого не скрывает ошибкой.
    conn!.subscribe("camera_rear");
    await sleep(150);
    expect(h.frames.some((f) => f.topic === "camera_rear")).toBe(false);
    expect(h.errors).toEqual([]);
  });

  it("unknown topic → ERROR TOPIC_UNKNOWN; unknown cmd → UNKNOWN_COMMAND", async () => {
    const h = connect();
    await until(() => conn!.getState() === "connected");
    conn!.subscribe("no_such_topic");
    conn!.send({ cmd: "definitely_not_a_cmd", ts_ms: Date.now() });
    await until(() => h.errors.length >= 2);
    expect(h.errors).toEqual(["TOPIC_UNKNOWN", "UNKNOWN_COMMAND"]);
  });

  it("stream_list lists the simulated streams", async () => {
    const h = connect();
    await until(() => conn!.getState() === "connected");
    conn!.requestStreamList();
    await until(() => h.events.some((e) => e.type === "stream_list"));
    const ev = h.events.find((e) => e.type === "stream_list")!;
    const topics = (ev.items as Array<{ topic: string }>).map((i) => i.topic);
    expect(topics).toEqual(expect.arrayContaining(["camera_rear", "camera_ceiling", "lidar_2d", "map_2d"]));
  });

  it("teleop_twist with deadman drives the robot; deadman=false stops it", async () => {
    connect();
    await until(() => conn!.getState() === "connected");
    const x0 = robot!.state.x;
    const twist = (x: number, deadman: boolean) =>
      conn!.send({
        cmd: "teleop_twist",
        ts_ms: Date.now(),
        linear: { x, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
        deadman
      });
    for (let i = 0; i < 12; i += 1) {
      twist(0.5, true);
      await sleep(33);
    }
    expect(robot!.state.x).toBeGreaterThan(x0 + 0.05);
    twist(0, false);
    await sleep(700);
    expect(Math.abs(robot!.state.v)).toBeLessThan(0.01);
  });

  it("stop_emergency latches for the sim hold time and ignores twists", async () => {
    connect();
    await until(() => conn!.getState() === "connected");
    conn!.send({ cmd: "stop_emergency", ts_ms: Date.now(), source: "ui_button" });
    await until(() => robot!.isEmergency());
    expect(robot!.mode()).toBe("emergency_stop");
    conn!.send({ cmd: "teleop_twist", ts_ms: Date.now(), linear: { x: 0.5, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 }, deadman: true });
    await sleep(100);
    expect(robot!.state.v).toBe(0);
    expect(SIM_EMERGENCY_HOLD_MS).toBeGreaterThan(1000);
  });

  it("close() detaches the session from the robot", async () => {
    connect();
    await until(() => conn!.getState() === "connected");
    expect(robot!.sessionCount()).toBe(1);
    conn!.close();
    await until(() => robot!.sessionCount() === 0);
  });
});

describe("sim world kinematics", () => {
  it("robot cannot drive through a wall (bump), but may turn", () => {
    const world = createDefaultWorld();
    const st = createRobotState(world);
    st.x = 1;
    st.y = 3;
    st.yaw = Math.PI; // на запад, стена x=0 в 1 м
    let now = 0;
    for (let i = 0; i < 200; i += 1) {
      now += 20;
      st.cmdV = 0.5;
      st.cmdW = 0;
      st.lastCmdMs = now;
      stepRobot(world, st, 0.02, now);
    }
    expect(st.x).toBeGreaterThanOrEqual(ROBOT_RADIUS_M - 1e-6);
    expect(st.bumped).toBe(true);
    expect(isFree(world, st.x, st.y, ROBOT_RADIUS_M)).toBe(true);
  });

  it("command times out → robot decelerates to zero", () => {
    const world = createDefaultWorld();
    const st = createRobotState(world);
    st.cmdV = 0.5;
    st.lastCmdMs = 0;
    stepRobot(world, st, 0.1, 50);
    expect(st.v).toBeGreaterThan(0);
    for (let t = 400; t < 2000; t += 20) stepRobot(world, st, 0.02, t);
    expect(st.v).toBe(0);
  });
});
