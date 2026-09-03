// Integration-тест: VoicePresetsPanel ↔ Connection-like wire layer.
// Не дёргает bootstrap() (Three.js / canvas), а напрямую проверяет
// контракт клик → set_voice и server event → UI sync.
//
// Покрывает AV-28 §P7 acceptance:
//   - выбор сохраняется в mode_manager.currentPreset
//   - событие уходит на сервер (правильный cmd: set_voice)
//   - серверный voice_presets event обновляет список и снимает loading
//   - серверный voice_set_ack подтверждает выбор
//   - серверный voice_set_nack откатывает UI
//
// Используем FakeConnection — минимальный stub, записывающий send() и
// позволяющий вручную эмитить JSON_EVENT'ы в listener.
import { describe, it, expect, beforeEach } from "vitest";
import { createVoicePresetsPanel, type VoicePresetsPanel } from "../src/ui/voice_presets_panel";
import { createModeManager, type ClientModeManager } from "../src/ui/mode_manager";
import type {
  JsonCmd,
  JsonEvent,
  VoicePresetId,
  VoicePresetInfo,
  VoiceLanguage
} from "../src/wire/messages";

class FakeConnection {
  public sent: JsonCmd[] = [];
  private listeners = new Set<(ev: JsonEvent) => void>();
  public connected = true;

  send(cmd: JsonCmd): void {
    if (!this.connected) throw new Error("not connected");
    this.sent.push(cmd);
  }

  /** Подписка на JSON_EVENT'ы (повторяет ConnectionListeners.onJsonEvent). */
  onJsonEvent(cb: (ev: JsonEvent) => void): () => void {
    this.listeners.add(cb);
    return () => this.listeners.delete(cb);
  }

  /** Эмулировать серверный event. */
  emit(ev: JsonEvent): void {
    for (const cb of this.listeners) cb(ev);
  }
}

const PRESETS_FALLBACK: VoicePresetInfo[] = [
  { id: "technical", name: "Технический" },
  { id: "street", name: "По понятиям" },
  { id: "caveman", name: "Пещерный" },
  { id: "business", name: "Деловой" },
  { id: "philosopher", name: "Философ" },
  { id: "lenin", name: "Ленин" }
];

const PRESETS_FROM_SERVER: VoicePresetInfo[] = [
  { id: "technical", name: "Tech (EN)" },
  { id: "lenin", name: "Lenin (EN)" }
];

describe("Voice presets UI ↔ Connection wiring (AV-28 §P7)", () => {
  let parent: HTMLElement;
  let conn: FakeConnection;
  let panel: VoicePresetsPanel;
  let mm: ClientModeManager;

  beforeEach(() => {
    parent = document.createElement("div");
    document.body.appendChild(parent);
    conn = new FakeConnection();
    mm = createModeManager(undefined, { preset: "technical", language: "ru" });
    panel = createVoicePresetsPanel(parent, {
      presets: PRESETS_FALLBACK,
      languages: ["ru", "en"],
      currentPreset: (mm.snapshot().currentPreset ?? "technical") as VoicePresetId,
      currentLanguage: mm.snapshot().currentLanguage ?? "ru",
      // До ответа сервера loading=true — клики игнорируются.
      loading: true,
      onPresetChange: (preset) => {
        if (!conn.connected) return;
        try {
          conn.send({
            cmd: "set_voice",
            ts_ms: 1234,
            voice_id: mm.snapshot().currentVoice ?? "",
            preset,
            language: mm.snapshot().currentLanguage ?? "ru"
          });
        } catch {
          // Откат UI при ошибке.
          panel.setCurrentPreset(mm.snapshot().currentPreset);
        }
      },
      onLanguageChange: (language) => {
        if (!conn.connected) return;
        try {
          conn.send({
            cmd: "set_voice",
            ts_ms: 1234,
            voice_id: mm.snapshot().currentVoice ?? "",
            preset: mm.snapshot().currentPreset ?? "technical",
            language
          });
        } catch {
          panel.setCurrentLanguage(mm.snapshot().currentLanguage);
        }
      }
    });
    // Снимаем loading — клики должны работать (имитируем, что сервер
    // уже ответил — конкретные тесты ниже ещё дополнительно эмитят
    // voice_presets чтобы проверить реакцию UI на смену дефолтов).
    panel.setLoading(false);
    // Подписка main.ts: server event → обновление UI + mode_manager.
    conn.onJsonEvent((ev) => {
      const e = ev as { type?: string } & Record<string, unknown>;
      if (e.type === "voice_presets") {
        const p = e as unknown as {
          presets?: VoicePresetInfo[];
          languages?: VoiceLanguage[];
          default_preset?: VoicePresetId;
          default_language?: VoiceLanguage;
        };
        if (Array.isArray(p.presets) && p.presets.length > 0) {
          panel.setPresets(p.presets);
        }
        if (Array.isArray(p.languages) && p.languages.length > 0) {
          panel.setLanguages(p.languages);
        }
        if (p.default_preset) {
          panel.setCurrentPreset(p.default_preset);
          mm.setCurrentPreset(p.default_preset);
        }
        if (p.default_language) {
          panel.setCurrentLanguage(p.default_language);
          mm.setCurrentLanguage(p.default_language);
        }
        panel.setLoading(false);
      } else if (e.type === "voice_set_ack") {
        const ack = e as {
          voice_id?: string;
          preset?: VoicePresetId;
          language?: VoiceLanguage;
        };
        if (ack.voice_id) mm.setCurrentVoice(ack.voice_id);
        if (ack.preset) mm.setCurrentPreset(ack.preset);
        if (ack.language) mm.setCurrentLanguage(ack.language);
      } else if (e.type === "voice_set_nack") {
        panel.setCurrentPreset(mm.snapshot().currentPreset);
        panel.setCurrentLanguage(mm.snapshot().currentLanguage);
      }
    });
  });

  it("defaults to technical + ru before server reply", () => {
    // Уникальный setup: loading=true ещё не снимали.
    document.body.removeChild(parent);
    parent = document.createElement("div");
    document.body.appendChild(parent);
    const mmLocal = createModeManager(undefined, { preset: "technical", language: "ru" });
    const panelLocal = createVoicePresetsPanel(parent, {
      presets: PRESETS_FALLBACK,
      languages: ["ru", "en"],
      currentPreset: (mmLocal.snapshot().currentPreset ?? "technical") as VoicePresetId,
      currentLanguage: mmLocal.snapshot().currentLanguage ?? "ru",
      loading: true
    });
    expect(mmLocal.snapshot().currentPreset).toBe("technical");
    expect(mmLocal.snapshot().currentLanguage).toBe("ru");
    expect(panelLocal.isLoading).toBe(true);
    panelLocal.dispose();
  });

  it("click on 'lenin' chip sends set_voice cmd", () => {
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const leninBtn = root.querySelector('[data-preset="lenin"]') as HTMLButtonElement;
    leninBtn.click();
    expect(conn.sent.length).toBe(1);
    const cmd = conn.sent[0] as {
      cmd: string;
      preset: VoicePresetId;
      language: VoiceLanguage;
    };
    expect(cmd.cmd).toBe("set_voice");
    expect(cmd.preset).toBe("lenin");
    expect(cmd.language).toBe("ru");
  });

  it("click on 'en' language toggle sends set_voice cmd", () => {
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const enBtn = root.querySelector('[data-language="en"]') as HTMLButtonElement;
    enBtn.click();
    expect(conn.sent.length).toBe(1);
    const cmd = conn.sent[0] as {
      cmd: string;
      preset: VoicePresetId;
      language: VoiceLanguage;
    };
    expect(cmd.cmd).toBe("set_voice");
    expect(cmd.preset).toBe("technical"); // дефолт
    expect(cmd.language).toBe("en");
  });

  it("click on already-selected chip does NOT send cmd", () => {
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const techBtn = root.querySelector('[data-preset="technical"]') as HTMLButtonElement;
    techBtn.click();
    expect(conn.sent.length).toBe(0);
  });

  it("server voice_presets event replaces list, applies defaults, clears loading", () => {
    conn.emit({
      type: "voice_presets",
      presets: PRESETS_FROM_SERVER,
      languages: ["ru", "en"],
      default_preset: "lenin",
      default_language: "en",
      ts_ms: 1000
    });
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const chips = root.querySelectorAll(".voice-presets-panel__chip");
    expect(chips.length).toBe(2); // server list replaces fallback
    expect(panel.isLoading).toBe(false);
    // Серверные дефолты применились.
    expect(mm.snapshot().currentPreset).toBe("lenin");
    expect(mm.snapshot().currentLanguage).toBe("en");
    const leninBtn = root.querySelector('[data-preset="lenin"]') as HTMLButtonElement;
    expect(leninBtn.getAttribute("aria-pressed")).toBe("true");
    const enBtn = root.querySelector('[data-language="en"]') as HTMLButtonElement;
    expect(enBtn.getAttribute("aria-checked")).toBe("true");
  });

  it("server voice_set_ack updates mode_manager (with voice_id)", () => {
    conn.emit({
      type: "voice_set_ack",
      voice_id: "alena-001",
      preset: "caveman",
      language: "en",
      ts_ms: 2000
    });
    expect(mm.snapshot().currentVoice).toBe("alena-001");
    expect(mm.snapshot().currentPreset).toBe("caveman");
    expect(mm.snapshot().currentLanguage).toBe("en");
  });

  it("server voice_set_nack rolls UI back to last good snapshot", () => {
    // Имитируем выбор → сервер отказал.
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const cavemanBtn = root.querySelector('[data-preset="caveman"]') as HTMLButtonElement;
    cavemanBtn.click();
    // Panel подсветил caveman, но в mm ещё technical (там синхронизация
    // только через ACK). Эмулируем NACK: сервер говорит «не сменил».
    conn.emit({
      type: "voice_set_nack",
      voice_id: "",
      preset: "caveman",
      language: "ru",
      reason: "voice 'caveman' not configured for current TTS provider",
      ts_ms: 3000
    });
    // После NACK панель откатилась на текущее значение mm (technical).
    expect(root.querySelector('[data-preset="technical"]')?.getAttribute("aria-pressed")).toBe(
      "true"
    );
    expect(root.querySelector('[data-preset="caveman"]')?.getAttribute("aria-pressed")).toBe(
      "false"
    );
  });

  it("click sends set_voice with updated snapshot after server applies default", () => {
    // Сервер сменил дефолт на lenin/en → клик по technical должен прислать
    // preset=technical, language=en.
    conn.emit({
      type: "voice_presets",
      presets: PRESETS_FROM_SERVER,
      languages: ["ru", "en"],
      default_preset: "lenin",
      default_language: "en",
      ts_ms: 1000
    });
    conn.sent.length = 0; // сбрасываем запись
    const root = parent.querySelector("[data-voice-presets-panel]") as HTMLElement;
    const techBtn = root.querySelector('[data-preset="technical"]') as HTMLButtonElement;
    techBtn.click();
    expect(conn.sent.length).toBe(1);
    const cmd = conn.sent[0] as {
      cmd: string;
      preset: VoicePresetId;
      language: VoiceLanguage;
    };
    expect(cmd.preset).toBe("technical");
    expect(cmd.language).toBe("en");
  });
});
