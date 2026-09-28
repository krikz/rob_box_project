// Панель «ПОТОКИ» (issue #3150): разбор id целей, раскладка, форматирование,
// пересборка хит-мешей при смене набора потоков.
//
// jsdom не реализует canvas.getContext — ставим заглушку, как в
// tars1_text_panel.test.ts.

import { describe, it, expect, beforeAll } from "vitest";
import {
  computeStreamsLayout,
  createStreamsPanel,
  formatKbps,
  formatRate,
  parseStreamsTargetId,
  streamsTargetId,
  STREAMS_PANEL_ANGLE_DEG
} from "../src/scene/streams_panel";
import {
  SUPERVISOR_PANEL_ANGLE_DEG,
  panelGeometry
} from "../src/scene/supervisor_panel";
import { VOICE_PIPELINE_ANGLE_DEG } from "../src/scene/voice_pipeline_panel";
import type { SubscriptionsView } from "../src/state/subscription_manager";

beforeAll(() => {
  const stubCtx = {
    fillStyle: "",
    font: "",
    textBaseline: "",
    fillRect: () => {},
    fillText: () => {},
    clearRect: () => {},
    measureText: (t: string) => ({ width: t.length * 7 })
  } as unknown as CanvasRenderingContext2D;
  HTMLCanvasElement.prototype.getContext = function () {
    return stubCtx;
  } as unknown as typeof HTMLCanvasElement.prototype.getContext;
});

function view(topics: string[]): SubscriptionsView {
  return {
    profile: "lan",
    totalKbps: 0,
    streams: topics.map((topic) => ({ topic, video: topic.startsWith("camera_"), enabled: true, maxHz: null, kbps: 0, fps: 0 }))
  };
}

describe("target ids", () => {
  it("round-trips all actions", () => {
    for (const a of [
      { kind: "profile", profile: "internet" },
      { kind: "toggle", topic: "camera_rear" },
      { kind: "rate", topic: "lidar_2d" }
    ] as const) {
      expect(parseStreamsTargetId(streamsTargetId(a))).toEqual(a);
    }
  });

  it("rejects foreign / malformed ids", () => {
    expect(parseStreamsTargetId("sup:mode:off")).toBeNull();
    expect(parseStreamsTargetId("str:profile:wat")).toBeNull();
    expect(parseStreamsTargetId("str:toggle:")).toBeNull();
    expect(parseStreamsTargetId("str:boom:x")).toBeNull();
    expect(parseStreamsTargetId("main_screen")).toBeNull();
  });
});

describe("layout", () => {
  it("rows fit inside the canvas and do not overlap", () => {
    const topics = ["a", "b", "c", "d", "e", "f", "g", "h"];
    const l = computeStreamsLayout(topics, 512, 640);
    expect(l.profiles).toHaveLength(4);
    for (let i = 0; i < l.rows.length; i++) {
      const r = l.rows[i];
      expect(r.toggle.y + r.toggle.h).toBeLessThanOrEqual(640);
      expect(r.toggle.x + r.toggle.w).toBeLessThan(r.rate.x);
      expect(r.rate.x + r.rate.w).toBeLessThan(r.stats.x);
      expect(r.stats.x + r.stats.w).toBeLessThanOrEqual(512);
      if (i > 0) expect(r.toggle.y).toBeGreaterThanOrEqual(l.rows[i - 1].toggle.y + l.rows[i - 1].toggle.h);
    }
    expect(l.rows[0].toggle.y).toBeGreaterThan(l.profiles[0].rect.y + l.profiles[0].rect.h);
  });

  it("sits on the right flank, clear of the pipeline and supervisor panels", () => {
    expect(STREAMS_PANEL_ANGLE_DEG).toBe(-SUPERVISOR_PANEL_ANGLE_DEG);
    // 0.95 м на радиусе 2.4 м ≈ ±11.3° — между панелями больше 22.6°
    expect(STREAMS_PANEL_ANGLE_DEG - VOICE_PIPELINE_ANGLE_DEG).toBeGreaterThan(23);
    const g = panelGeometry(STREAMS_PANEL_ANGLE_DEG, 2.4, 1.45);
    expect(g.position.x).toBeGreaterThan(0);
    // facing — к оператору (в центр)
    expect(g.facing.x * g.position.x + g.facing.z * g.position.z).toBeLessThan(0);
  });
});

describe("format", () => {
  it("kbps", () => {
    expect(formatKbps(0)).toBe("0 кбит/с");
    expect(formatKbps(3.24)).toBe("3.2 кбит/с");
    expect(formatKbps(512.4)).toBe("512 кбит/с");
    expect(formatKbps(2500)).toBe("2.5 Мбит/с");
    expect(formatKbps(Number.NaN)).toBe("0 кбит/с");
  });
  it("rate", () => {
    expect(formatRate(null)).toBe("макс");
    expect(formatRate(5)).toBe("5 Гц");
  });
});

describe("createStreamsPanel", () => {
  it("builds profile + per-stream targets, rebuilds only when topics change", () => {
    const p = createStreamsPanel();
    expect(p.render(view(["camera_rear", "lidar_2d"]))).toBe(true);
    const ids = p.targets().map((t) => t.id).sort();
    expect(ids).toEqual(
      [
        "str:profile:lan",
        "str:profile:internet",
        "str:profile:minimum",
        "str:profile:custom",
        "str:toggle:camera_rear",
        "str:rate:camera_rear",
        "str:toggle:lidar_2d",
        "str:rate:lidar_2d"
      ].sort()
    );
    expect(p.render(view(["camera_rear", "lidar_2d"]))).toBe(false);
    expect(p.render(view(["camera_front", "lidar_2d"]))).toBe(true);
    expect(p.targets().map((t) => t.id)).toContain("str:toggle:camera_front");
    p.dispose();
  });
});
