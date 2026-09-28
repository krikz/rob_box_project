import { describe, it, expect } from "vitest";
import {
  SubscriptionManager,
  SUBSCRIPTIONS_STORAGE_KEY,
  nextRate,
  parsePersisted,
  profileConfig,
  type SubscriptionStorage,
  type SubscriptionTransport
} from "../src/state/subscription_manager";

const TOPICS = [
  "camera_rear",
  "camera_ceiling",
  "camera_oak_depth",
  "lidar_2d",
  "map_2d",
  "robot_status",
  "voice_state"
];

class FakeTransport implements SubscriptionTransport {
  calls: string[] = [];
  subscribe(topic: string, maxHz: number | null): void {
    this.calls.push(`sub ${topic} ${maxHz ?? "-"}`);
  }
  unsubscribe(topic: string): void {
    this.calls.push(`unsub ${topic}`);
  }
}

class MemStorage implements SubscriptionStorage {
  data = new Map<string, string>();
  getItem(k: string): string | null {
    return this.data.get(k) ?? null;
  }
  setItem(k: string, v: string): void {
    this.data.set(k, v);
  }
}

function make(storage: SubscriptionStorage | null = null): SubscriptionManager {
  return new SubscriptionManager({ topics: TOPICS, mainVideoTopic: "camera_rear", storage });
}

describe("profiles", () => {
  it("LAN: everything on, no limit", () => {
    for (const t of TOPICS) expect(profileConfig("lan", t, "camera_rear")).toEqual({ enabled: true, maxHz: null });
  });

  it("Интернет: main camera 5 Hz, other cameras off, lidar 5 Hz", () => {
    const m = make();
    m.applyProfile("internet");
    expect(m.config("camera_rear")).toEqual({ enabled: true, maxHz: 5 });
    expect(m.config("camera_ceiling")?.enabled).toBe(false);
    expect(m.config("camera_oak_depth")?.enabled).toBe(false);
    expect(m.config("lidar_2d")).toEqual({ enabled: true, maxHz: 5 });
    for (const t of ["map_2d", "robot_status", "voice_state"]) {
      expect(m.config(t)).toEqual({ enabled: true, maxHz: null });
    }
  });

  it("Минимум: no video, lidar 2 Hz", () => {
    const m = make();
    m.applyProfile("minimum");
    for (const t of TOPICS.filter((x) => x.startsWith("camera_"))) expect(m.config(t)?.enabled).toBe(false);
    expect(m.config("lidar_2d")).toEqual({ enabled: true, maxHz: 2 });
    expect(m.config("robot_status")?.enabled).toBe(true);
    expect(m.config("voice_state")?.enabled).toBe(true);
    expect(m.config("map_2d")?.enabled).toBe(true);
  });

  it("manual edit switches to «Свой»", () => {
    const m = make();
    m.applyProfile("internet");
    m.toggle("camera_ceiling");
    expect(m.profile()).toBe("custom");
    expect(m.config("camera_ceiling")?.enabled).toBe(true);
  });
});

describe("transport sync", () => {
  it("attach subscribes all enabled streams with their rate", () => {
    const m = make();
    m.applyProfile("minimum");
    const t = new FakeTransport();
    m.attach(t);
    expect(t.calls).toEqual(["sub lidar_2d 2", "sub map_2d -", "sub robot_status -", "sub voice_state -"]);
  });

  it("profile switch sends only the diff (unsub, resub with new rate)", () => {
    const m = make();
    const t = new FakeTransport();
    m.attach(t);
    t.calls = [];
    m.applyProfile("internet");
    expect(t.calls).toEqual([
      "unsub camera_ceiling",
      "unsub camera_oak_depth",
      "sub camera_rear 5",
      "sub lidar_2d 5"
    ]);
    t.calls = [];
    m.applyProfile("internet");
    expect(t.calls).toEqual([]);
  });

  it("cycleRate resubscribes with the next step", () => {
    const m = make();
    const t = new FakeTransport();
    m.attach(t);
    t.calls = [];
    m.cycleRate("camera_rear");
    expect(t.calls).toEqual(["sub camera_rear 15"]);
    expect(m.profile()).toBe("custom");
  });

  it("detach + attach (reconnect) resubscribes everything", () => {
    const m = make();
    m.applyProfile("minimum");
    const t1 = new FakeTransport();
    m.attach(t1);
    m.detach();
    m.toggle("camera_rear"); // без транспорта — ничего не шлём
    const t2 = new FakeTransport();
    m.attach(t2);
    expect(t2.calls).toContain("sub camera_rear -");
    expect(t2.calls).toHaveLength(5);
  });

  it("replaceTopic carries the slot config and drops unused old topic", () => {
    const m = make();
    m.applyProfile("internet");
    const t = new FakeTransport();
    m.attach(t);
    t.calls = [];
    m.replaceTopic("camera_rear", "camera_front", false);
    expect(m.config("camera_front")).toEqual({ enabled: true, maxHz: 5 });
    expect(m.config("camera_rear")).toBeUndefined();
    expect(t.calls).toEqual(["unsub camera_rear", "sub camera_front 5"]);
    expect(m.topics()[0]).toBe("camera_front");
  });
});

describe("bandwidth meter", () => {
  it("computes kbit/s and fps with EMA, total sums streams", () => {
    const m = make();
    m.tick(0);
    for (let i = 0; i < 10; i++) m.recordFrame("camera_rear", 12_500); // 10 × 100 кбит
    m.recordFrame("lidar_2d", 1_250);
    m.tick(1000);
    let v = m.view();
    const cam = v.streams.find((s) => s.topic === "camera_rear")!;
    // alpha 0.5: 0.5 × 1000 kbit/s
    expect(cam.kbps).toBeCloseTo(500);
    expect(cam.fps).toBeCloseTo(5);
    expect(v.totalKbps).toBeCloseTo(505);
    // тишина — EMA затухает
    m.tick(2000);
    v = m.view();
    expect(v.streams.find((s) => s.topic === "camera_rear")!.kbps).toBeCloseTo(250);
  });

  it("ignores unknown topics and non-advancing clock", () => {
    const m = make();
    m.recordFrame("nope", 100);
    m.tick(1000);
    m.tick(1000);
    expect(m.totalKbps()).toBe(0);
  });

  it("notifies listeners on tick and on edits", () => {
    const m = make();
    let n = 0;
    m.onChange(() => (n += 1));
    m.tick(0);
    m.tick(500);
    m.toggle("map_2d");
    expect(n).toBe(2);
  });
});

describe("persistence", () => {
  it("profile survives restart", () => {
    const s = new MemStorage();
    make(s).applyProfile("minimum");
    const m2 = make(s);
    expect(m2.profile()).toBe("minimum");
    expect(m2.config("camera_rear")?.enabled).toBe(false);
  });

  it("custom configs survive restart", () => {
    const s = new MemStorage();
    const m = make(s);
    m.setMaxHz("camera_rear", 10);
    m.setEnabled("camera_ceiling", false);
    const m2 = make(s);
    expect(m2.profile()).toBe("custom");
    expect(m2.config("camera_rear")).toEqual({ enabled: true, maxHz: 10 });
    expect(m2.config("camera_ceiling")?.enabled).toBe(false);
  });

  it("garbage or throwing storage falls back to LAN", () => {
    const s = new MemStorage();
    s.setItem(SUBSCRIPTIONS_STORAGE_KEY, "{not json");
    expect(make(s).profile()).toBe("lan");
    const throwing: SubscriptionStorage = {
      getItem: () => {
        throw new Error("denied");
      },
      setItem: () => {
        throw new Error("quota");
      }
    };
    const m = make(throwing);
    expect(m.profile()).toBe("lan");
    expect(() => m.applyProfile("internet")).not.toThrow();
  });

  it("parsePersisted sanitizes bad entries", () => {
    const p = parsePersisted(
      JSON.stringify({ profile: "custom", custom: { a: { enabled: true, maxHz: -1 }, b: { enabled: "x" } } })
    );
    expect(p).toEqual({ profile: "custom", custom: { a: { enabled: true, maxHz: null } } });
    expect(parsePersisted(JSON.stringify({ profile: "wat" }))).toBeNull();
  });
});

describe("nextRate", () => {
  it("cycles through steps", () => {
    expect(nextRate(null)).toBe(15);
    expect(nextRate(1)).toBe(null);
    expect(nextRate(7)).toBe(null);
  });
});
