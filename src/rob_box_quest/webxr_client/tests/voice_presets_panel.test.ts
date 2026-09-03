import { describe, it, expect, vi, beforeEach, afterEach } from "vitest";
import {
  createVoicePresetsPanel,
  renderHud,
  type VoicePresetsPanel
} from "../src/ui/voice_presets_panel";
import type { VoicePresetInfo, VoiceLanguage, VoicePreset } from "../src/wire/messages";

describe("renderHud", () => {
  // AV-28 §P7: HUD-метка должна быть стабильной и читабельной с
  // расстояния. Тест контракта, не рендеринга — это pure-функция.
  it("returns ST:-- when no preset selected", () => {
    expect(renderHud(null, null)).toBe("ST:--");
  });

  it("renders ST:PRESET when only preset provided", () => {
    // ADR-0027 §3.4.1: префикс "ST:" обязателен, чтобы не путать стиль
    // речи (AV-28) с voice_id (AV-27).
    expect(renderHud("lenin", null)).toBe("ST:LENIN");
    expect(renderHud("technical", null)).toBe("ST:TECHNICAL");
  });

  it("renders ST:PRESET@LANG when both provided", () => {
    expect(renderHud("lenin", "ru")).toBe("ST:LENIN@RU");
    expect(renderHud("philosopher", "en")).toBe("ST:PHILOSOPHER@EN");
  });

  it("is case-insensitive (input uppercased)", () => {
    // VoicePreset допускает legacy ("Lenin") + AV-28 IDs ("lenin");
    // берём legacy-капитализацию, чтобы проверить и uppercase.
    expect(renderHud("Lenin" as VoicePreset, "ru")).toBe("ST:LENIN@RU");
  });

  it("ignores preset but renders language when preset is null", () => {
    // (текущий контракт: без preset нет HUD-метки; язык показываем
    // только при наличии пресета — иначе оператор не знает, к чему
    // относится @RU).
    expect(renderHud(null, "ru")).toBe("ST:--");
  });
});

const PRESETS: VoicePresetInfo[] = [
  { id: "technical", name: "Технический" },
  { id: "street", name: "По понятиям" },
  { id: "caveman", name: "Пещерный" }
];

describe("createVoicePresetsPanel", () => {
  let parent: HTMLElement;
  let panel: VoicePresetsPanel;

  beforeEach(() => {
    parent = document.createElement("div");
    document.body.appendChild(parent);
  });

  afterEach(() => {
    panel?.dispose();
    parent.remove();
  });

  it("renders role=group + aria-label and presets list", () => {
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      currentPreset: "technical",
      languages: ["ru", "en"],
      currentLanguage: "ru"
    });
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    expect(root).not.toBeNull();
    expect(root.getAttribute("role")).toBe("group");
    expect(root.getAttribute("aria-label")).toBeTruthy();
    // 3 кнопки-чипа + 2 кнопки языка = 5 кнопок.
    const chips = root.querySelectorAll(".voice-presets-panel__chip");
    expect(chips.length).toBe(3);
    const langBtns = root.querySelectorAll(".voice-presets-panel__lang-btn");
    expect(langBtns.length).toBe(2);
  });

  it("currentPreset/aria-pressed = current value", () => {
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      currentPreset: "street",
      currentLanguage: "ru"
    });
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const chips = Array.from(
      root.querySelectorAll(".voice-presets-panel__chip")
    ) as HTMLButtonElement[];
    const pressed = chips.filter((c) => c.getAttribute("aria-pressed") === "true");
    expect(pressed.length).toBe(1);
    expect(pressed[0].getAttribute("data-preset")).toBe("street");
  });

  it("currentLanguage/aria-checked = current value", () => {
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      currentLanguage: "en"
    });
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const langBtns = Array.from(
      root.querySelectorAll(".voice-presets-panel__lang-btn")
    ) as HTMLButtonElement[];
    const checked = langBtns.filter(
      (c) => c.getAttribute("aria-checked") === "true"
    );
    expect(checked.length).toBe(1);
    expect(checked[0].getAttribute("data-language")).toBe("en");
  });

  it("click on chip fires onPresetChange and updates aria-pressed", () => {
    const onPresetChange = vi.fn();
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      currentPreset: "technical",
      onPresetChange
    });
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const cavemanBtn = root.querySelector(
      '[data-preset="caveman"]'
    ) as HTMLButtonElement;
    cavemanBtn.click();
    expect(onPresetChange).toHaveBeenCalledWith("caveman");
    expect(cavemanBtn.getAttribute("aria-pressed")).toBe("true");
    // Старая подсветка снята.
    const technicalBtn = root.querySelector(
      '[data-preset="technical"]'
    ) as HTMLButtonElement;
    expect(technicalBtn.getAttribute("aria-pressed")).toBe("false");
  });

  it("click on language fires onLanguageChange and updates aria-checked", () => {
    const onLanguageChange = vi.fn();
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      currentLanguage: "ru",
      onLanguageChange
    });
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const enBtn = root.querySelector(
      '[data-language="en"]'
    ) as HTMLButtonElement;
    enBtn.click();
    expect(onLanguageChange).toHaveBeenCalledWith("en");
    expect(enBtn.getAttribute("aria-checked")).toBe("true");
    const ruBtn = root.querySelector(
      '[data-language="ru"]'
    ) as HTMLButtonElement;
    expect(ruBtn.getAttribute("aria-checked")).toBe("false");
  });

  it("click on already-selected chip does NOT fire callback", () => {
    const onPresetChange = vi.fn();
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      currentPreset: "technical",
      onPresetChange
    });
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const technicalBtn = root.querySelector(
      '[data-preset="technical"]'
    ) as HTMLButtonElement;
    technicalBtn.click();
    expect(onPresetChange).not.toHaveBeenCalled();
  });

  it("setPresets replaces list and clears stale currentPreset", () => {
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      currentPreset: "lenin" // нет в списке
    });
    panel.setPresets([
      { id: "technical", name: "Технический" },
      { id: "street", name: "По понятиям" }
    ]);
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const chips = root.querySelectorAll(".voice-presets-panel__chip");
    expect(chips.length).toBe(2);
    const pressed = Array.from(chips).filter(
      (c) => c.getAttribute("aria-pressed") === "true"
    );
    expect(pressed.length).toBe(0);
  });

  it("setLanguages replaces list", () => {
    panel = createVoicePresetsPanel(parent, {
      languages: ["ru", "en"],
      currentLanguage: "ru"
    });
    panel.setLanguages(["ru"] as VoiceLanguage[]);
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const langBtns = root.querySelectorAll(".voice-presets-panel__lang-btn");
    expect(langBtns.length).toBe(1);
    expect(langBtns[0].getAttribute("data-language")).toBe("ru");
  });

  it("setLoading(true) disables all buttons and shows loading text", () => {
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      currentLanguage: "ru"
    });
    panel.setLoading(true);
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const chips = Array.from(
      root.querySelectorAll(".voice-presets-panel__chip")
    ) as HTMLButtonElement[];
    for (const c of chips) expect(c.disabled).toBe(true);
    const loadingEl = root.querySelector(
      ".voice-presets-panel__loading"
    ) as HTMLElement;
    expect(loadingEl.hidden).toBe(false);
    expect(panel.isLoading).toBe(true);
  });

  it("setLoading(false) re-enables buttons", () => {
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      loading: true
    });
    panel.setLoading(false);
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const chips = Array.from(
      root.querySelectorAll(".voice-presets-panel__chip")
    ) as HTMLButtonElement[];
    for (const c of chips) expect(c.disabled).toBe(false);
    expect(panel.isLoading).toBe(false);
  });

  it("click while loading does NOT fire callback", () => {
    const onPresetChange = vi.fn();
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      currentPreset: "technical",
      loading: true,
      onPresetChange
    });
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const cavemanBtn = root.querySelector(
      '[data-preset="caveman"]'
    ) as HTMLButtonElement;
    // click() на disabled button не сработает в jsdom, поэтому проверим,
    // что disabled=true, и попробуем .dispatchEvent.
    expect(cavemanBtn.disabled).toBe(true);
    cavemanBtn.dispatchEvent(new Event("click"));
    expect(onPresetChange).not.toHaveBeenCalled();
  });

  it("setCurrentPreset updates aria-pressed without firing callback", () => {
    const onPresetChange = vi.fn();
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      currentPreset: "technical",
      onPresetChange
    });
    panel.setCurrentPreset("street");
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const streetBtn = root.querySelector(
      '[data-preset="street"]'
    ) as HTMLButtonElement;
    expect(streetBtn.getAttribute("aria-pressed")).toBe("true");
    expect(onPresetChange).not.toHaveBeenCalled();
  });

  it("HUD element starts with ST:-- and updates on chip click", () => {
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      currentPreset: null,
      currentLanguage: null
    });
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const hud = root.querySelector(
      '[data-testid="voice-presets-hud"]'
    ) as HTMLElement;
    expect(hud).toBeTruthy();
    expect(hud.textContent).toBe("ST:--");
    // Кликаем по чипу → HUD должен обновиться оптимистично
    // (префикс ST: обязателен по ADR-0027 §3.4.1).
    const cavemanBtn = root.querySelector(
      '[data-preset="caveman"]'
    ) as HTMLButtonElement;
    cavemanBtn.click();
    expect(hud.textContent).toBe("ST:CAVEMAN");
  });

  it("HUD includes language after setCurrentLanguage", () => {
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      languages: ["ru", "en"],
      currentPreset: "lenin", // намеренно не в PRESETS — HUD рисуется отдельно
      currentLanguage: "ru"
    });
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const hud = root.querySelector(
      '[data-testid="voice-presets-hud"]'
    ) as HTMLElement;
    expect(hud.textContent).toBe("ST:LENIN@RU");
    panel.setCurrentLanguage("en");
    expect(hud.textContent).toBe("ST:LENIN@EN");
  });

  it("setCurrentLanguage updates aria-checked without firing callback", () => {
    const onLanguageChange = vi.fn();
    panel = createVoicePresetsPanel(parent, {
      currentLanguage: "ru",
      onLanguageChange
    });
    panel.setCurrentLanguage("en");
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const enBtn = root.querySelector(
      '[data-language="en"]'
    ) as HTMLButtonElement;
    expect(enBtn.getAttribute("aria-checked")).toBe("true");
    expect(onLanguageChange).not.toHaveBeenCalled();
  });

  it("setVisible(true|false) toggles hidden attribute and isVisible", () => {
    panel = createVoicePresetsPanel(parent, { presets: PRESETS });
    expect(panel.isVisible).toBe(true);
    panel.setVisible(false);
    expect(panel.isVisible).toBe(false);
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    expect(root.hidden).toBe(true);
    panel.setVisible(true);
    expect(panel.isVisible).toBe(true);
    expect(root.hidden).toBe(false);
  });

  it("onChange fires on visibility transitions", () => {
    panel = createVoicePresetsPanel(parent, { presets: PRESETS });
    const cb = vi.fn();
    panel.onChange(cb);
    panel.setVisible(false);
    expect(cb).toHaveBeenCalledWith(false);
    panel.setVisible(true);
    expect(cb).toHaveBeenCalledWith(true);
  });

  it("listener exception does not break UI", () => {
    const onPresetChange = vi.fn(() => {
      throw new Error("boom");
    });
    const warnSpy = vi.spyOn(console, "warn").mockImplementation(() => {});
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS,
      currentPreset: "technical",
      onPresetChange
    });
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const cavemanBtn = root.querySelector(
      '[data-preset="caveman"]'
    ) as HTMLButtonElement;
    cavemanBtn.click();
    expect(onPresetChange).toHaveBeenCalled();
    warnSpy.mockRestore();
  });

  it("dispose removes element and makes API no-op", () => {
    panel = createVoicePresetsPanel(parent, { presets: PRESETS });
    panel.dispose();
    expect(parent.querySelector("[data-voice-presets-panel]")).toBeNull();
    expect(panel.isDisposed).toBe(true);
    // Повторный dispose / set* — no-op без падений.
    expect(() => panel.dispose()).not.toThrow();
    expect(() => panel.setPresets([])).not.toThrow();
  });
});
