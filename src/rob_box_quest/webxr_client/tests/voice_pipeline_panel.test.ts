// voice_pipeline_panel: pure-логика (без Three.js).
// Формат-разбор, геометрия раскладки и hit-test — детерминированные
// функции; render/WebGL не тестируем (jsdom не даёт WebGL, как в
// supervisor_panel.test.ts и status_hud.test.ts).

import { describe, it, expect } from "vitest";
import {
  LLM_TARGET_ID,
  STT_TARGET_ID,
  TTS_TARGET_ID,
  computePipelineLayout,
  hitTest,
  langTargetId,
  parsePipelineTargetId,
  presetTargetId,
  renderHud,
  VOICE_PIPELINE_ANGLE_DEG,
  VOICE_PIPELINE_H_M,
  VOICE_PIPELINE_RADIUS_M,
  VOICE_PIPELINE_W_M,
  VOICE_PIPELINE_Y_M
} from "../src/scene/voice_pipeline_panel";
import type { VoiceLanguage, VoicePreset } from "../src/wire/messages";

describe("renderHud", () => {
  it("пусто → ST:--", () => {
    expect(renderHud(null, null)).toBe("ST:--");
  });
  it("пресет без языка → ST:LENIN", () => {
    expect(renderHud("lenin", null)).toBe("ST:LENIN");
  });
  it("пресет + язык → ST:LENIN@RU", () => {
    expect(renderHud("lenin", "ru")).toBe("ST:LENIN@RU");
  });
  it("legacy-пресет тоже апперкейсится", () => {
    expect(renderHud("Lenin" as VoicePreset, "ru")).toBe("ST:LENIN@RU");
  });
});

describe("parsePipelineTargetId", () => {
  it("stt / llm / tts", () => {
    expect(parsePipelineTargetId(STT_TARGET_ID)).toEqual({ kind: "stt" });
    expect(parsePipelineTargetId(LLM_TARGET_ID)).toEqual({ kind: "llm" });
    expect(parsePipelineTargetId(TTS_TARGET_ID)).toEqual({ kind: "tts" });
  });
  it("preset", () => {
    expect(parsePipelineTargetId(presetTargetId("lenin"))).toEqual({ kind: "preset", preset: "lenin" });
  });
  it("lang", () => {
    expect(parsePipelineTargetId(langTargetId("en"))).toEqual({ kind: "lang", language: "en" });
  });
  it("чужие id → null", () => {
    expect(parsePipelineTargetId("sup:mode:mixed")).toBeNull();
    expect(parsePipelineTargetId("tts:launch")).toBeNull();
    expect(parsePipelineTargetId("panel_0")).toBeNull();
  });
  it("пустой preset/lang → null", () => {
    expect(parsePipelineTargetId("vpl:preset:")).toBeNull();
    expect(parsePipelineTargetId("vpl:lang:")).toBeNull();
  });
});

describe("computePipelineLayout", () => {
  const layout = computePipelineLayout(512);

  it("6 чипов пресетов + 2 языка", () => {
    expect(layout.presetChips).toHaveLength(6);
    expect(layout.langButtons).toHaveLength(2);
  });

  it("все rect внутри канваса", () => {
    const rects = [
      layout.header,
      layout.chain,
      layout.stt,
      layout.llm,
      layout.tts,
      layout.sectionLabel,
      layout.loading,
      ...layout.presetChips.map((c) => c.rect),
      ...layout.langButtons.map((b) => b.rect)
    ];
    for (const r of rects) {
      expect(r.x).toBeGreaterThanOrEqual(0);
      expect(r.y).toBeGreaterThanOrEqual(0);
      expect(r.x + r.w).toBeLessThanOrEqual(512 + 0.001);
      expect(r.y + r.h).toBeLessThanOrEqual(528 + 0.001);
    }
  });

  it("чипы не пересекаются между собой", () => {
    const chips = layout.presetChips;
    for (let i = 0; i < chips.length; i++) {
      for (let j = i + 1; j < chips.length; j++) {
        const a = chips[i].rect;
        const b = chips[j].rect;
        const overlap = a.x < b.x + b.w && a.x + a.w > b.x && a.y < b.y + b.h && a.y + a.h > b.y;
        expect(overlap, `chips ${i} and ${j} overlap`).toBe(false);
      }
    }
  });

  it("id чипов соответствуют порядку PRESET_ORDER", () => {
    expect(layout.presetChips.map((c) => c.buttonId)).toEqual([
      "preset:technical",
      "preset:street",
      "preset:caveman",
      "preset:business",
      "preset:philosopher",
      "preset:lenin"
    ]);
  });
});

describe("hitTest", () => {
  const layout = computePipelineLayout(512);

  function center(rect: { x: number; y: number; w: number; h: number }): [number, number] {
    return [rect.x + rect.w / 2, rect.y + rect.h / 2];
  }

  it("тумблеры STT / LLM / TTS", () => {
    const [sx, sy] = center(layout.stt);
    expect(hitTest(sx / 512, sy / 528, layout, 512, 528)).toBe(STT_TARGET_ID);
    const [lx, ly] = center(layout.llm);
    expect(hitTest(lx / 512, ly / 528, layout, 512, 528)).toBe(LLM_TARGET_ID);
    const [tx, ty] = center(layout.tts);
    expect(hitTest(tx / 512, ty / 528, layout, 512, 528)).toBe(TTS_TARGET_ID);
  });

  it("чип пресета", () => {
    const chip = layout.presetChips[5]; // lenin
    const [x, y] = center(chip.rect);
    expect(hitTest(x / 512, y / 528, layout, 512, 528)).toBe(chip.id);
  });

  it("кнопка языка", () => {
    const b = layout.langButtons[1]; // en
    const [x, y] = center(b.rect);
    expect(hitTest(x / 512, y / 528, layout, 512, 528)).toBe(b.id);
  });

  it("вне кнопок → null", () => {
    // header не кликабелен.
    const [x, y] = center(layout.header);
    expect(hitTest(x / 512, y / 528, layout, 512, 528)).toBeNull();
    // loading-строка тоже.
    const [lx, ly] = center(layout.loading);
    expect(hitTest(lx / 512, ly / 528, layout, 512, 528)).toBeNull();
  });

  it("невалидные координаты → null", () => {
    expect(hitTest(Number.NaN, 0.5, layout, 512, 528)).toBeNull();
    expect(hitTest(0.5, 0.5, layout, 0, 528)).toBeNull();
    expect(hitTest(2, 0.5, layout, 512, 528)).toBeNull();
  });
});

describe("геометрия панели", () => {
  it("панель справа (+105°) и не в нуле", () => {
    expect(VOICE_PIPELINE_ANGLE_DEG).toBeGreaterThan(0);
    expect(VOICE_PIPELINE_RADIUS_M).toBeGreaterThan(0);
    expect(VOICE_PIPELINE_Y_M).toBeGreaterThan(0);
    expect(VOICE_PIPELINE_W_M).toBeGreaterThan(0);
    expect(VOICE_PIPELINE_H_M).toBeGreaterThan(0);
  });
});

describe("типы языков", () => {
  it("LANG id нормализуется в id цели", () => {
    expect(langTargetId("ru" as VoiceLanguage)).toBe("vpl:lang:ru");
  });
});
