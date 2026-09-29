// Раскладка мостика целиком (ребаланс 29.09): фланговые панели не
// закрывают крылья TARS и друг друга, HUD-полоса не лезет на видео и под
// потолок, оболочка не заходит в рабочий объём оператора.

import { describe, it, expect } from "vitest";
import {
  CEILING_SCREEN_POS,
  HUD_STRIP_SLOTS,
  HUD_STRIP_Y,
  MAIN_SCREEN_CENTER,
  MAIN_SCREEN_SIZE,
  SIDE_PANEL_ANGLES_DEG,
  hudStripBezel,
  panelHalfSpanDeg,
  tarsWingSectorDeg
} from "../src/scene/captain_bridge";
import { SUPERVISOR_PANEL_ANGLE_DEG, SUPERVISOR_PANEL_RADIUS_M, SUPERVISOR_PANEL_W_M } from "../src/scene/supervisor_panel";
import { STREAMS_PANEL_ANGLE_DEG, STREAMS_PANEL_RADIUS_M, STREAMS_PANEL_W_M } from "../src/scene/streams_panel";
import {
  VOICE_PIPELINE_ANGLE_DEG,
  VOICE_PIPELINE_RADIUS_M,
  VOICE_PIPELINE_W_M
} from "../src/scene/voice_pipeline_panel";
import { bridgeShellBoxes, SHELL_HALF_D, SHELL_HALF_W, SHELL_HEIGHT } from "../src/scene/bridge_shell";

// PanelManager по умолчанию: радиус 2.0 м, панель 1.2 м (panel_manager.ts).
const VIDEO_PANEL_RADIUS_M = 2.0;
const VIDEO_PANEL_W_M = 1.2;

interface Span {
  name: string;
  center: number;
  half: number;
}

function flankPanels(): Span[] {
  return [
    ...SIDE_PANEL_ANGLES_DEG.map((a, i) => ({
      name: `video#${i}`,
      center: a,
      half: panelHalfSpanDeg(VIDEO_PANEL_W_M, VIDEO_PANEL_RADIUS_M)
    })),
    { name: "voice", center: VOICE_PIPELINE_ANGLE_DEG, half: panelHalfSpanDeg(VOICE_PIPELINE_W_M, VOICE_PIPELINE_RADIUS_M) },
    {
      name: "supervisor",
      center: SUPERVISOR_PANEL_ANGLE_DEG,
      half: panelHalfSpanDeg(SUPERVISOR_PANEL_W_M, SUPERVISOR_PANEL_RADIUS_M)
    },
    { name: "streams", center: STREAMS_PANEL_ANGLE_DEG, half: panelHalfSpanDeg(STREAMS_PANEL_W_M, STREAMS_PANEL_RADIUS_M) }
  ];
}

describe("TARS wing sector", () => {
  it("covers the flared wing from the main-screen edge to its far edge", () => {
    const s = tarsWingSectorDeg();
    // Кромка главного экрана (2.4, −3.9) ≈ 31.6°, дальняя кромка ≈ 87.7°.
    expect(s.min).toBeCloseTo(31.6, 0);
    expect(s.max).toBeCloseTo(87.7, 0);
  });
});

describe("flank panels", () => {
  it("never cover a TARS wing (outside its azimuth sector on both sides)", () => {
    const wing = tarsWingSectorDeg();
    for (const p of flankPanels()) {
      const near = Math.abs(p.center) - p.half;
      const far = Math.abs(p.center) + p.half;
      // Панель целиком дальше крыла по азимуту…
      expect(near, `${p.name} заходит в крыло TARS`).toBeGreaterThan(wing.max);
      // …и не уходит за спину (180°) — её ещё видно поворотом головы.
      expect(far, `${p.name} за спиной`).toBeLessThan(180);
    }
  });

  it("do not overlap each other", () => {
    const panels = flankPanels();
    for (let i = 0; i < panels.length; i += 1) {
      for (let j = i + 1; j < panels.length; j += 1) {
        const a = panels[i];
        const b = panels[j];
        const gap = Math.abs(a.center - b.center) - a.half - b.half;
        expect(gap, `${a.name} ↔ ${b.name}`).toBeGreaterThan(0);
      }
    }
  });

  it("voice pipeline sits next to TARS1 (left, speech) and depth next to TARS2 (right)", () => {
    expect(VOICE_PIPELINE_ANGLE_DEG).toBeLessThan(0);
    for (const a of SIDE_PANEL_ANGLES_DEG) expect(a).toBeGreaterThan(0);
  });
});

describe("HUD strip", () => {
  const screenTop = MAIN_SCREEN_CENTER.y + MAIN_SCREEN_SIZE.height / 2;
  const slots = Object.values(HUD_STRIP_SLOTS);

  it("sits above the wall-screen video and its frame (frame top ≈ 3.04 m)", () => {
    for (const s of slots) {
      expect(HUD_STRIP_Y - s.height / 2).toBeGreaterThan(screenTop + 0.2);
    }
  });

  it("stays under the shell ceiling beams", () => {
    for (const s of slots) {
      expect(HUD_STRIP_Y + s.height / 2).toBeLessThan(SHELL_HEIGHT - 0.16);
    }
  });

  it("slots do not overlap and stay within the wall-screen width", () => {
    const sorted = [...slots].sort((a, b) => a.x - b.x);
    for (let i = 1; i < sorted.length; i += 1) {
      const prevRight = sorted[i - 1].x + sorted[i - 1].width / 2;
      const left = sorted[i].x - sorted[i].width / 2;
      expect(left).toBeGreaterThan(prevRight);
    }
    const bz = hudStripBezel();
    expect(bz.width).toBeLessThanOrEqual(MAIN_SCREEN_SIZE.width + 0.01);
  });

  it("is not hidden by the ceiling screen", () => {
    // Потолочный экран висит над оператором (z = 0), полоса — у стены.
    expect(Math.abs(CEILING_SCREEN_POS.z - MAIN_SCREEN_CENTER.z)).toBeGreaterThan(3);
  });
});

describe("bridge shell", () => {
  const boxes = bridgeShellBoxes({ hudBezel: hudStripBezel() });

  it("keeps out of the operator's working volume", () => {
    // Всё, что ближе стен комнаты, — выше 3 м (шапка, подложка HUD, потолок):
    // панели, крылья TARS и видео стоят внутри |x| < 5.5, |z| < 4.3, y < 3.
    for (const b of boxes) {
      const nearWall =
        Math.abs(b.center.x) > SHELL_HALF_W - 0.3 || Math.abs(b.center.z) > SHELL_HALF_D - 0.3;
      const aboveWork = b.center.y - b.size.y / 2 >= 3.0;
      expect(nearWall || aboveWork, JSON.stringify(b.center)).toBe(true);
    }
  });

  it("encloses the sides and the back (walls on x = ±W and z = +D)", () => {
    const tall = boxes.filter((b) => b.size.y >= SHELL_HEIGHT - 0.01 && b.layer === "solid");
    expect(tall.some((b) => b.center.x < -SHELL_HALF_W + 0.3)).toBe(true);
    expect(tall.some((b) => b.center.x > SHELL_HALF_W - 0.3)).toBe(true);
    expect(tall.some((b) => b.center.z > SHELL_HALF_D - 0.3)).toBe(true);
  });

  it("stays light for Quest (few hundred boxes → two merged meshes)", () => {
    expect(boxes.length).toBeLessThan(300);
    expect(new Set(boxes.map((b) => b.layer))).toEqual(new Set(["solid", "glow"]));
  });
});
