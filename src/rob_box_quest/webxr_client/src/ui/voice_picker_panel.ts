// voice_picker_panel — UI для выбора TTS-голоса и пресета (Phase 2 §4.1–§4.5).
//
// Использует lil-gui как floating panel справа-сверху.
// Список голосов берётся через callback `getVoices()` (обычно из
// `JSON_EVENT{type: "voice_list"}`), preview отправляется через
// `onPreviewVoice({voice_id, text, request_id})`, apply — через
// `onSetVoice({voice_id, preset})`. Текущий голос (`currentVoice`)
// отображается в HUD-overlay (элемент `voiceIndicatorEl`) и обновляется
// по `setCurrentVoice({voice_id, preset})`.

import GUI from "lil-gui";
import type { VoiceInfo, VoicePreset } from "../wire/messages";
import { makePreviewRequestId, type PreviewState } from "../audio/preview_player";

export interface VoicePickerHandle {
  destroy(): void;
  setVoices(voices: VoiceInfo[]): void;
  setCurrentVoice(voice_id: string | null, preset: VoicePreset | null): void;
  setConnectionOnline(online: boolean): void;
  setPreviewState(state: PreviewState): void;
  refresh(): void;
  /** Тест-хелпер: прочитать текущее состояние preview (последний setPreviewState). */
  _peekPreviewState(): PreviewState;
}

export interface VoicePickerOptions {
  /** Получить список голосов (например, последний полученный voice_list). */
  getVoices(): VoiceInfo[];
  /** Получить текущий голос (или null, если не установлен). */
  getCurrentVoice(): { voice_id: string; preset: VoicePreset } | null;
  /** Подключение к серверу установлено? */
  isOnline(): boolean;
  /** HUD-элемент, в котором показываем "Voice: <id> (<preset>)" или "Voice: unknown". */
  voiceIndicatorEl: HTMLElement;
  /** Кнопка Apply → отправить серверу set_voice. */
  onSetVoice(args: { voice_id: string; preset: VoicePreset }): void;
  /** Кнопка Preview → запросить синтез фразы. */
  onPreviewVoice(args: { voice_id: string; text: string; request_id: string }): void;
  /** Кнопка Request Voices → отправить list_voices. */
  onRequestVoices(): void;
  /** Стандартная фраза для preview (можно переопределить через UI). */
  defaultPreviewText?: string;
  /** Дефолтный пресет. */
  defaultPreset?: VoicePreset;
}

const ALL_PRESETS: VoicePreset[] = ["standard", "friendly", "authoritative", "whisper"];

export function createVoicePicker(opts: VoicePickerOptions): VoicePickerHandle {
  const gui = new GUI({ title: "rob_box_quest / voice_settings", width: 320 });
  const state = {
    selectedVoice: "",
    preset: opts.defaultPreset ?? "standard",
    previewText: opts.defaultPreviewText ?? "Привет, я робот. Какой голос вам нравится?",
    requestVoices: (): void => opts.onRequestVoices(),
    preview: (): void => previewSelected(),
    apply: (): void => applySelected()
  };
  let voices: VoiceInfo[] = [];
  let previewState: PreviewState = "idle";

  const folderConn = gui.addFolder("Voice (server)");
  folderConn.add(state, "requestVoices").name("↻ Request voices");
  const statusCtrl = folderConn
    .add({ status: voiceIndicatorText(opts.getCurrentVoice(), opts.isOnline()) }, "status")
    .name("Status")
    .disable();
  folderConn.open();

  const folderPick = gui.addFolder("Pick voice");
  const voiceCtrl = folderPick
    .add(state, "selectedVoice", voicesList())
    .name("Voice")
    .onChange(() => refreshApplyState());
  const presetCtrl = folderPick
    .add(state, "preset", ALL_PRESETS)
    .name("Preset")
    .onChange(() => refreshApplyState());
  folderPick.add(state, "previewText").name("Preview text");
  const previewCtrl = folderPick.add(state, "preview").name("▶ Preview");
  const applyCtrl = folderPick.add(state, "apply").name("✔ Apply");
  folderPick.open();

  function setPreviewStateImpl(next: PreviewState): void {
    previewState = next;
    if (previewCtrl) {
      const label =
        next === "playing"
          ? "⏸ Playing…"
          : next === "error"
            ? "✗ Error"
            : next === "stopped"
              ? "▶ Preview (stopped)"
              : "▶ Preview";
      previewCtrl.name(label);
    }
  }

  function voicesList(): string[] {
    return voices.length > 0 ? voices.map((v) => v.voice_id) : [];
  }

  function previewSelected(): void {
    if (!state.selectedVoice) return;
    if (!opts.isOnline()) {
      setPreviewStateImpl("error");
      return;
    }
    opts.onPreviewVoice({
      voice_id: state.selectedVoice,
      text: state.previewText,
      request_id: makePreviewRequestId()
    });
  }

  function applySelected(): void {
    if (!state.selectedVoice) return;
    opts.onSetVoice({
      voice_id: state.selectedVoice,
      preset: state.preset as VoicePreset
    });
  }

  function refreshApplyState(): void {
    const v = state.selectedVoice;
    const canApply = !!v && opts.isOnline();
    if (previewCtrl) previewCtrl.disable(!canApply);
    if (applyCtrl) applyCtrl.disable(!canApply);
  }

  function voiceIndicatorText(
    current: { voice_id: string; preset: VoicePreset } | null,
    online: boolean
  ): string {
    if (!online) return "Voice: unknown (offline)";
    if (!current) return "Voice: <not set>";
    return `Voice: ${current.voice_id} (${current.preset})`;
  }

  function updateIndicator(): void {
    const txt = voiceIndicatorText(opts.getCurrentVoice(), opts.isOnline());
    opts.voiceIndicatorEl.textContent = txt;
    opts.voiceIndicatorEl.classList.toggle("voice-indicator--offline", !opts.isOnline());
    statusCtrl.object = { status: txt };
    // statusCtrl.object не сработает (read-only), обновим DOM напрямую:
    const el = (statusCtrl.domElement as HTMLElement).closest(".controller");
    if (el) {
      (el as HTMLElement).style.color = opts.isOnline() ? "var(--accent)" : "var(--danger)";
    }
  }

  // Initial indicator state — иначе DOM останется пустым до первого setConnectionOnline/setCurrentVoice.
  updateIndicator();

  return {
    destroy(): void {
      gui.destroy();
    },
    setVoices(next): void {
      voices = next;
      voiceCtrl.options(voicesList());
      if (!state.selectedVoice && voices.length > 0) {
        const cur = opts.getCurrentVoice();
        const match = cur ? voices.findIndex((v) => v.voice_id === cur.voice_id) : -1;
        state.selectedVoice = match >= 0 ? voices[match].voice_id : voices[0].voice_id;
        voiceCtrl.updateDisplay();
      } else if (state.selectedVoice && !voices.find((v) => v.voice_id === state.selectedVoice)) {
        // Текущий выбор больше не доступен → сбросить.
        state.selectedVoice = voices[0]?.voice_id ?? "";
        voiceCtrl.updateDisplay();
      }
      refreshApplyState();
    },
    setCurrentVoice(voice_id, preset): void {
      if (voice_id) {
        state.selectedVoice = voice_id;
        voiceCtrl.updateDisplay();
      }
      if (preset) {
        state.preset = preset;
        presetCtrl.updateDisplay();
      }
      updateIndicator();
      refreshApplyState();
    },
    setConnectionOnline(online): void {
      // При (re)connect заново запрашиваем voices и обновляем indicator.
      // При disconnect — disable preview/apply (refreshApplyState) и пометить offline.
      if (online) {
        opts.onRequestVoices();
      }
      refreshApplyState();
      updateIndicator();
    },
    setPreviewState(next): void {
      setPreviewStateImpl(next);
    },
    refresh: refreshApplyState,
    _peekPreviewState: (): PreviewState => previewState
  };
}
