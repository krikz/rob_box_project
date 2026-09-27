// voice_pipeline_panel: pure-логика (без Three.js).
// Формат-разбор, геометрия раскладки и hit-test — детерминированные
// функции; render/WebGL не тестируем (jsdom не даёт WebGL, как в
// supervisor_panel.test.ts и status_hud.test.ts).

import { describe, it, expect } from "vitest";
import {
  CANVAS_H,
  CANVAS_W,
  FALLBACK_PRESETS,
  LANG_LABELS,
  LANG_ORDER,
  LLM_TARGET_ID,
  PRESET_ORDER,
  STYLE_OFF_TARGET_ID,
  PIPELINE_DRAG_TARGET_ID,
  STT_TARGET_ID,
  TTS_TARGET_ID,
  computePipelineLayout,
  hitTest,
  langTargetId,
  parsePipelineTargetId,
  pipelineCanvasHeight,
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
  it("drag-ручка не является кнопкой", () => {
    expect(PIPELINE_DRAG_TARGET_ID).toBe("vpl:drag");
    expect(parsePipelineTargetId(PIPELINE_DRAG_TARGET_ID)).toBeNull();
  });
});

describe("FALLBACK_PRESETS", () => {
  it("7 пресетов с русскими именами", () => {
    expect(FALLBACK_PRESETS).toHaveLength(7);
    expect(FALLBACK_PRESETS.map((p) => p.id)).toEqual([...PRESET_ORDER]);
    expect(FALLBACK_PRESETS.find((p) => p.id === "lenin")?.name).toBe("Ленин");
  });

  it("«Перевод» идёт первым: чаще всего нужен именно он", () => {
    expect(PRESET_ORDER[0]).toBe("translate");
    expect(FALLBACK_PRESETS[0]).toEqual({ id: "translate", name: "Перевод" });
  });
});

describe("языки вывода", () => {
  it("ru/en/fr/de/zh/hi — как в voice_presets.yaml", () => {
    expect([...LANG_ORDER]).toEqual(["ru", "en", "fr", "de", "zh", "hi"]);
  });
  it("у каждого языка есть подпись кнопки", () => {
    for (const lang of LANG_ORDER) {
      expect(LANG_LABELS[lang], `нет подписи для ${lang}`).toBeTruthy();
    }
  });
});

describe("computePipelineLayout", () => {
  const layout = computePipelineLayout(CANVAS_W);

  it("7 чипов пресетов + 6 языков", () => {
    expect(layout.presetChips).toHaveLength(7);
    expect(layout.langButtons).toHaveLength(6);
  });

  it("высота канваса считается из раскладки, а не задана числом", () => {
    expect(CANVAS_H).toBe(pipelineCanvasHeight(layout));
  });

  it("все rect внутри канваса", () => {
    const rects = [
      layout.header,
      layout.progress,
      layout.stt,
      layout.llm,
      layout.tts,
      layout.sectionLabel,
      layout.noStyle.rect,
      layout.loading,
      ...layout.presetChips.map((c) => c.rect),
      ...layout.langButtons.map((b) => b.rect)
    ];
    for (const r of rects) {
      expect(r.x).toBeGreaterThanOrEqual(0);
      expect(r.y).toBeGreaterThanOrEqual(0);
      expect(r.x + r.w).toBeLessThanOrEqual(CANVAS_W + 0.001);
      expect(r.y + r.h).toBeLessThanOrEqual(CANVAS_H + 0.001);
    }
  });

  it("кнопки не наезжают друг на друга", () => {
    const boxes = [
      layout.stt,
      layout.llm,
      layout.tts,
      layout.noStyle.rect,
      ...layout.presetChips.map((c) => c.rect),
      ...layout.langButtons.map((b) => b.rect)
    ];
    for (let i = 0; i < boxes.length; i++) {
      for (let j = i + 1; j < boxes.length; j++) {
        const a = boxes[i];
        const b = boxes[j];
        const overlap = a.x < b.x + b.w && a.x + a.w > b.x && a.y < b.y + b.h && a.y + a.h > b.y;
        expect(overlap, `buttons ${i} and ${j} overlap`).toBe(false);
      }
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
    expect(layout.presetChips.map((c) => c.buttonId)).toEqual(
      PRESET_ORDER.map((p) => `preset:${p}`)
    );
  });
});

describe("hitTest", () => {
  const layout = computePipelineLayout(CANVAS_W);

  function center(rect: { x: number; y: number; w: number; h: number }): [number, number] {
    return [rect.x + rect.w / 2, rect.y + rect.h / 2];
  }

  it("тумблеры STT / LLM / TTS", () => {
    const [sx, sy] = center(layout.stt);
    expect(hitTest(sx / CANVAS_W, sy / CANVAS_H, layout, CANVAS_W, CANVAS_H)).toBe(STT_TARGET_ID);
    const [lx, ly] = center(layout.llm);
    expect(hitTest(lx / CANVAS_W, ly / CANVAS_H, layout, CANVAS_W, CANVAS_H)).toBe(LLM_TARGET_ID);
    const [tx, ty] = center(layout.tts);
    expect(hitTest(tx / CANVAS_W, ty / CANVAS_H, layout, CANVAS_W, CANVAS_H)).toBe(TTS_TARGET_ID);
  });

  it("чип пресета", () => {
    const chip = layout.presetChips[5]; // lenin
    const [x, y] = center(chip.rect);
    expect(hitTest(x / CANVAS_W, y / CANVAS_H, layout, CANVAS_W, CANVAS_H)).toBe(chip.id);
  });

  it("кнопка «Без стиля»", () => {
    const [x, y] = center(layout.noStyle.rect);
    expect(hitTest(x / CANVAS_W, y / CANVAS_H, layout, CANVAS_W, CANVAS_H)).toBe(
      STYLE_OFF_TARGET_ID
    );
    expect(parsePipelineTargetId(STYLE_OFF_TARGET_ID)).toEqual({ kind: "style_off" });
  });

  it("кнопка языка", () => {
    const b = layout.langButtons[4]; // zh — второй ряд
    const [x, y] = center(b.rect);
    expect(hitTest(x / CANVAS_W, y / CANVAS_H, layout, CANVAS_W, CANVAS_H)).toBe(b.id);
  });

  it("вне кнопок → null", () => {
    // header не кликабелен.
    const [x, y] = center(layout.header);
    expect(hitTest(x / CANVAS_W, y / CANVAS_H, layout, CANVAS_W, CANVAS_H)).toBeNull();
    // loading-строка тоже.
    const [lx, ly] = center(layout.loading);
    expect(hitTest(lx / CANVAS_W, ly / CANVAS_H, layout, CANVAS_W, CANVAS_H)).toBeNull();
  });

  it("невалидные координаты → null", () => {
    expect(hitTest(Number.NaN, 0.5, layout, CANVAS_W, CANVAS_H)).toBeNull();
    expect(hitTest(0.5, 0.5, layout, 0, CANVAS_H)).toBeNull();
    expect(hitTest(2, 0.5, layout, CANVAS_W, CANVAS_H)).toBeNull();
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
