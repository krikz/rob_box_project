// Тесты персонажа ТАРС 1 и раскладки экрана (issue #3253, Ш1).
// Всё чистое — без часов, DOM и canvas.

import { describe, it, expect } from "vitest";
import { tarsFigurePose, TARS_SEGMENTS, TARS_BREATH_PERIOD_MS } from "../src/state/tars_figure";
import {
  buildInfoRows,
  computeTars1Layout,
  formatClock,
  INFO_DASH
} from "../src/state/tars1_layout";
import type { TarsActivity } from "../src/state/tars_activity";

const STATES: TarsActivity[] = ["idle", "listening", "thinking", "speaking"];

describe("tarsFigurePose", () => {
  it("всегда 4 сегмента, конечные числа, h и glow в 0..1", () => {
    for (const a of STATES) {
      for (const t of [0, 123, 1000, 99_999]) {
        const pose = tarsFigurePose(a, t);
        expect(pose).toHaveLength(TARS_SEGMENTS);
        for (const s of pose) {
          for (const v of [s.dx, s.dy, s.h, s.glow]) expect(Number.isFinite(v)).toBe(true);
          expect(s.h).toBeGreaterThan(0);
          expect(s.h).toBeLessThanOrEqual(1);
          expect(s.glow).toBeGreaterThanOrEqual(0);
          expect(s.glow).toBeLessThanOrEqual(1);
        }
      }
    }
  });

  it("детерминирована: тот же вход — та же поза", () => {
    for (const a of STATES) {
      expect(tarsFigurePose(a, 4321)).toEqual(tarsFigurePose(a, 4321));
    }
  });

  it("позы разных состояний различимы в один и тот же момент", () => {
    const t = 2750;
    const keys = STATES.map((a) => JSON.stringify(tarsFigurePose(a, t)));
    expect(new Set(keys).size).toBe(STATES.length);
  });

  it("idle: дыхание периодично (период ~4 с) и едва заметно", () => {
    const a = tarsFigurePose("idle", 700);
    const b = tarsFigurePose("idle", 700 + TARS_BREATH_PERIOD_MS);
    a.forEach((s, i) => {
      expect(s.h).toBeCloseTo(b[i].h, 9);
      expect(s.glow).toBeCloseTo(b[i].glow, 9);
    });
    // Внутри периода поза меняется, но мало.
    const c = tarsFigurePose("idle", 700 + TARS_BREATH_PERIOD_MS / 4);
    expect(c[0].glow).not.toBeCloseTo(a[0].glow, 3);
    for (const s of c) {
      expect(Math.abs(s.dy)).toBeLessThan(0.02);
      expect(s.dx).toBe(0);
    }
  });

  it("listening: колонны раздвигаются симметрично, величина зависит от pulse", () => {
    const pose = tarsFigurePose("listening", 1000);
    expect(pose[0].dx).toBeCloseTo(-pose[3].dx, 9);
    expect(pose[1].dx).toBeCloseTo(-pose[2].dx, 9);
    expect(pose[0].dx).toBeLessThan(pose[1].dx);
    const other = tarsFigurePose("listening", 1200);
    expect(other[0].dx).not.toBeCloseTo(pose[0].dx, 3);
  });

  it("thinking: в каждый момент поднят один сегмент, номер идёт по кругу", () => {
    const raised: number[] = [];
    for (let k = 0; k < 8; k += 1) {
      const pose = tarsFigurePose("thinking", k * 300 + 150);
      const up = pose.map((s, i) => (s.dy < -0.05 ? i : -1)).filter((i) => i >= 0);
      expect(up).toHaveLength(1);
      raised.push(up[0]);
    }
    expect(new Set(raised.slice(0, 4)).size).toBe(4);
    expect(raised.slice(4)).toEqual(raised.slice(0, 4));
  });

  it("speaking: высота сегментов меняется как у эквалайзера и они не синхронны", () => {
    const p1 = tarsFigurePose("speaking", 100);
    const p2 = tarsFigurePose("speaking", 400);
    expect(p1.map((s) => s.h)).not.toEqual(p2.map((s) => s.h));
    expect(new Set(p1.map((s) => s.h.toFixed(3))).size).toBeGreaterThan(1);
  });
});

describe("computeTars1Layout", () => {
  for (const [w, h] of [
    [1280, 720],
    [256, 144]
  ] as const) {
    it(`${w}x${h}: консоль ниже рамки, колонки не пересекаются, всё внутри canvas`, () => {
      const l = computeTars1Layout(w, h);
      expect(l.console.y).toBeGreaterThanOrEqual(l.frame.y + l.frame.h);
      expect(l.left.x + l.left.w).toBeLessThanOrEqual(l.dividerX);
      expect(l.right.x).toBeGreaterThan(l.dividerX);
      expect(l.service.y + l.service.h).toBeLessThanOrEqual(l.rightDividerY);
      expect(l.context.y).toBeGreaterThan(l.rightDividerY);
      expect(l.context.y + l.context.h).toBeLessThanOrEqual(l.frame.y + l.frame.h);
      expect(l.right.x + l.right.w).toBeLessThanOrEqual(l.frame.x + l.frame.w);
      expect(l.console.y + l.console.h).toBeLessThanOrEqual(h);
      expect(l.console.x + l.console.w).toBeLessThanOrEqual(w);
      expect(l.console.h).toBeGreaterThan(0);
    });
  }
});

describe("buildInfoRows", () => {
  it("пустой info — все поля прочерк, ничего не выдумано", () => {
    const r = buildInfoRows({});
    for (const row of [...r.service, ...r.context]) expect(row.value).toBe(INFO_DASH);
  });

  it("известные поля показываются, неизвестные (llm/tts/wake/тема/рядом) — прочерк", () => {
    const r = buildInfoRows({
      link: "CONNECTED",
      floor: "teleop my · voice free",
      mode: "mixed",
      ptt: "none",
      lastReplyAt: "12:34:56",
      llm: "  "
    });
    const m = new Map([...r.service, ...r.context].map((x) => [x.label, x.value]));
    expect(m.get("связь")).toBe("CONNECTED");
    expect(m.get("ptt")).toBe("none");
    expect(m.get("реплика")).toBe("12:34:56");
    expect(m.get("llm")).toBe(INFO_DASH);
    for (const k of ["tts", "wake", "тема", "рядом"]) expect(m.get(k)).toBe(INFO_DASH);
  });

  it("formatClock даёт чч:мм:сс", () => {
    expect(formatClock(Date.now())).toMatch(/^\d{2}:\d{2}:\d{2}$/);
  });
});
