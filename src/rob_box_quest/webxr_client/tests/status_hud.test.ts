// Тесты status HUD (Wave 3.A / R8): разбор robot_status и формат строк.
// Только чистая логика — без Three.js и canvas.

import { describe, it, expect } from "vitest";
import {
  parseRobotStatus,
  formatStatusLines,
  BATTERY_LOW_PCT,
  WIFI_WEAK_DBM,
  RTT_WARN_MS,
  RTT_BAD_MS,
  type RobotStatus
} from "../src/scene/status_hud";

const hex = (s: string) => new Uint8Array(s.match(/../g)!.map((b) => parseInt(b, 16)));

// Реальный payload сервера:
// msgpack.packb({'battery_pct': -1, 'battery_v': None, 'wifi_rssi': -48,
//                'mode': 'idle', 'vel_linear': 0.42, 'vel_angular': 0.0,
//                'ts_ms': 1700000000123}, use_bin_type=True)
const STATUS_PAYLOAD = hex(
  "87ab626174746572795f706374ffa9626174746572795f76c0a9776966695f72737369d0d0" +
    "a46d6f6465a469646c65aa76656c5f6c696e656172cb3fdae147ae147ae1ab76656c5f616e" +
    "67756c6172cb0000000000000000a574735f6d73cf0000018bcfe5687b"
);

const status = (over: Partial<RobotStatus> = {}): RobotStatus => ({
  battery_pct: 80,
  battery_v: null,
  wifi_rssi: -50,
  mode: "idle",
  vel_linear: 0,
  vel_angular: 0,
  ts_ms: 1_700_000_000_000,
  ...over
});

const valueOf = (lines: ReturnType<typeof formatStatusLines>, label: string) =>
  lines.find((l) => l.label === label)!;

describe("parseRobotStatus", () => {
  it("decodes a real server payload", () => {
    const s = parseRobotStatus(STATUS_PAYLOAD)!;
    expect(s.battery_pct).toBe(-1);
    expect(s.battery_v).toBeNull();
    expect(s.wifi_rssi).toBe(-48);
    expect(s.mode).toBe("idle");
    expect(s.vel_linear).toBeCloseTo(0.42, 5);
    expect(s.ts_ms).toBe(1_700_000_000_123);
  });

  it("returns null on a broken payload", () => {
    expect(parseRobotStatus(new Uint8Array([0xcd, 0x30]))).toBeNull();
  });

  it("falls back to sentinels for missing fields", () => {
    // {"mode": "idle"} — остальных ключей нет.
    const s = parseRobotStatus(hex("81a46d6f6465a469646c65"))!;
    expect(s.battery_pct).toBe(-1);
    expect(s.wifi_rssi).toBe(0);
    expect(s.vel_linear).toBe(0);
  });
});

describe("formatStatusLines — battery", () => {
  it("shows percent when the source reports it", () => {
    const line = valueOf(formatStatusLines(status({ battery_pct: 76 }), null), "BAT");
    expect(line.value).toBe("76%");
    expect(line.level).toBe("ok");
  });

  it("marks a low battery bad", () => {
    const line = valueOf(formatStatusLines(status({ battery_pct: BATTERY_LOW_PCT }), null), "BAT");
    expect(line.level).toBe("bad");
  });

  it("falls back to volts when percent is unavailable", () => {
    const line = valueOf(formatStatusLines(status({ battery_pct: -1, battery_v: 24.14 }), null), "BAT");
    expect(line.value).toBe("24.1 V");
  });

  it("shows a dash when no battery source exists", () => {
    const line = valueOf(formatStatusLines(status({ battery_pct: -1, battery_v: null }), null), "BAT");
    expect(line.value).toBe("—");
    expect(line.level).toBe("unknown");
  });
});

describe("formatStatusLines — wifi", () => {
  it("shows dBm", () => {
    expect(valueOf(formatStatusLines(status({ wifi_rssi: -48 }), null), "WIFI").value).toBe("-48 dBm");
  });

  it("warns on a weak link", () => {
    expect(valueOf(formatStatusLines(status({ wifi_rssi: WIFI_WEAK_DBM }), null), "WIFI").level).toBe("warn");
  });

  it("treats sentinel 0 as no source", () => {
    const line = valueOf(formatStatusLines(status({ wifi_rssi: 0 }), null), "WIFI");
    expect(line.value).toBe("—");
    expect(line.level).toBe("unknown");
  });
});

describe("formatStatusLines — rtt", () => {
  it("shows a dash before the first pong", () => {
    expect(valueOf(formatStatusLines(status(), null), "RTT").value).toBe("—");
  });

  it("is ok below the comfort budget", () => {
    expect(valueOf(formatStatusLines(status(), 40), "RTT").level).toBe("ok");
  });

  it("warns at the ADR-0027 comfort limit", () => {
    expect(valueOf(formatStatusLines(status(), RTT_WARN_MS), "RTT").level).toBe("warn");
  });

  it("is bad past the tolerable limit", () => {
    expect(valueOf(formatStatusLines(status(), RTT_BAD_MS), "RTT").level).toBe("bad");
  });
});

describe("formatStatusLines — mode and speed", () => {
  it("renders speed with two decimals", () => {
    expect(valueOf(formatStatusLines(status({ vel_linear: 0.4242 }), null), "SPD").value).toBe("0.42 m/s");
  });

  it("marks emergency mode bad", () => {
    expect(valueOf(formatStatusLines(status({ mode: "emergency" }), null), "MODE").level).toBe("bad");
  });

  it("renders every row as unknown before the first status frame", () => {
    const lines = formatStatusLines(null, null);
    // BAT, WIFI, SPD, RTT, FPS, MODE = 6 строк.
    expect(lines).toHaveLength(6);
    expect(lines.every((l) => l.level === "unknown" && l.value === "—")).toBe(true);
  });
});

describe("formatStatusLines — fps (AV-25)", () => {
  it("renders dash and unknown before the first frame sample", () => {
    const line = valueOf(formatStatusLines(status(), null, null), "FPS");
    expect(line.value).toBe("—");
    expect(line.level).toBe("unknown");
  });

  it("treats 0 / negative fps as 'no data' (unknown)", () => {
    expect(valueOf(formatStatusLines(status(), null, 0), "FPS").level).toBe("unknown");
    expect(valueOf(formatStatusLines(status(), null, -1), "FPS").level).toBe("unknown");
  });

  it("treats non-finite fps as 'no data'", () => {
    expect(valueOf(formatStatusLines(status(), null, NaN), "FPS").level).toBe("unknown");
    expect(valueOf(formatStatusLines(status(), null, Infinity), "FPS").level).toBe("unknown");
  });

  it("rounds fps to integer for display", () => {
    expect(valueOf(formatStatusLines(status(), null, 89.6), "FPS").value).toBe("90");
    expect(valueOf(formatStatusLines(status(), null, 60.4), "FPS").value).toBe("60");
  });

  it("marks fps below 15 as bad (red)", () => {
    expect(valueOf(formatStatusLines(status(), null, 14), "FPS").level).toBe("bad");
  });

  it("marks fps 15..29 as warn (yellow)", () => {
    expect(valueOf(formatStatusLines(status(), null, 15), "FPS").level).toBe("warn");
    expect(valueOf(formatStatusLines(status(), null, 29), "FPS").level).toBe("warn");
  });

  it("marks fps >= 30 as ok (green)", () => {
    expect(valueOf(formatStatusLines(status(), null, 30), "FPS").level).toBe("ok");
    expect(valueOf(formatStatusLines(status(), null, 90), "FPS").level).toBe("ok");
  });

  it("FPS row appears after RTT and before MODE", () => {
    const labels = formatStatusLines(status(), 50, 72).map((l) => l.label);
    const rtt = labels.indexOf("RTT");
    const fps = labels.indexOf("FPS");
    const mode = labels.indexOf("MODE");
    expect(fps).toBe(rtt + 1);
    expect(mode).toBe(fps + 1);
  });
});
