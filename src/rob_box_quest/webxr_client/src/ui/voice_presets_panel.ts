// Voice presets panel — выбор стиля речи (AV-28 §P7) и языка.
//
// Контракт (см. wire/messages.ts):
//   - сервер присылает JSON_EVENT{type:"voice_presets",
//     presets:[{id,name}...], languages:["ru","en"], default_preset,
//     default_language}; UI вызывает `panel.setPresets(...)`,
//     `panel.setLanguages(...)`, `panel.setCurrentPreset(...)`,
//     `panel.setCurrentLanguage(...)`.
//   - при клике пользователя панель дёргает onPresetChange /
//     onLanguageChange; bootstrap в main.ts шлёт
//     `cmd:"set_voice", voice_id, preset, language` через Connection.
//
// Дизайн — в стиле существующих overlay'ов (error/help):
//   - position: fixed, нижняя-левая четверть экрана (body говорит
//     "левом грипе Meta Quest"; в DOM-overlay через WebXR это проецируется
//     примерно туда же — окончательное 3D-позиционирование вынесено в
//     отдельную задачу, см. TODO в комментарии).
//   - ARIA: role="group", aria-label, кнопки пресетов с aria-pressed;
//     тумблер языка — radiogroup с aria-checked.
//   - Никакой зависимости от Three.js — тестируется как обычный DOM.
//
// Использование:
//   const panel = createVoicePresetsPanel(parent, {
//     presets: [{id:"technical", name:"Технический"}, …],
//     languages: ["ru","en"],
//     currentPreset: "technical",
//     currentLanguage: "ru",
//     onPresetChange: (id) => …,
//     onLanguageChange: (lang) => …,
//   });
//   panel.setPresets([…]);  // обновить список после server push
//   panel.dispose();

import type {
  VoiceLanguage,
  VoicePreset,
  VoicePresetId,
  VoicePresetInfo
} from "../wire/messages";

export interface VoicePresetsPanelOptions {
  /** Начальный список пресетов (дефолт, до ответа сервера). */
  presets?: VoicePresetInfo[];
  /** Начальный список языков (дефолт, до ответа сервера). */
  languages?: VoiceLanguage[];
  /** Текущий выбранный пресет (AV-28 ID из voice_presets.yaml). */
  currentPreset?: VoicePresetId | null;
  /** Текущий выбранный язык. */
  currentLanguage?: VoiceLanguage | null;
  /** Колбэк при клике пользователя на пресет. */
  onPresetChange?: (preset: VoicePresetId) => void;
  /** Колбэк при клике пользователя на язык. */
  onLanguageChange?: (language: VoiceLanguage) => void;
  /**
   * Если true — панель показывает loading-state (список ещё не пришёл).
   * Кнопки заблокированы, видна надпись "Загрузка…".
   */
  loading?: boolean;
}

export interface VoicePresetsPanel {
  /** Полная замена списка пресетов (сервер прислал голоса). */
  setPresets(presets: VoicePresetInfo[]): void;
  /** Полная замена списка языков. */
  setLanguages(languages: VoiceLanguage[]): void;
  /**
   * Подсветить выбранный пресет (без эмита callback'а).
   * Принимает любой VoicePreset (legacy + AV-28) — для совместимости
   * с сервером, который может прислать старые ID через voice_set_ack.
   * Если переданный пресет не в списке — подсветка сбрасывается в null.
   */
  setCurrentPreset(preset: VoicePreset | null): void;
  /** Подсветить выбранный язык (без эмита callback'а). */
  setCurrentLanguage(language: VoiceLanguage | null): void;
  /** Включить/выключить loading-state. */
  setLoading(loading: boolean): void;
  /** Видна ли панель. */
  setVisible(visible: boolean): void;
  /** Подписка на изменения видимости. */
  onChange(cb: (visible: boolean) => void): () => void;
  readonly isVisible: boolean;
  readonly isDisposed: boolean;
  readonly isLoading: boolean;
  dispose(): void;
}

const LANG_LABELS: Record<VoiceLanguage, string> = {
  ru: "RU",
  en: "EN"
};

export function createVoicePresetsPanel(
  parent: HTMLElement,
  options: VoicePresetsPanelOptions = {}
): VoicePresetsPanel {
  const root = document.createElement("section");
  root.className = "voice-presets-panel";
  root.setAttribute("data-voice-presets-panel", "");
  root.setAttribute("role", "group");
  root.setAttribute("aria-label", "Голос: стиль и язык");

  const heading = document.createElement("div");
  heading.className = "voice-presets-panel__heading";
  heading.textContent = "Голос";

  // ----- Список пресетов -------------------------------------------------
  const presetsLabel = document.createElement("div");
  presetsLabel.className = "voice-presets-panel__subheading";
  presetsLabel.textContent = "Стиль речи";

  const presetsGroup = document.createElement("div");
  presetsGroup.className = "voice-presets-panel__chips";
  presetsGroup.setAttribute("role", "group");
  presetsGroup.setAttribute("aria-label", "Пресеты стиля речи");

  // ----- Тумблер языка ---------------------------------------------------
  const langLabel = document.createElement("div");
  langLabel.className = "voice-presets-panel__subheading";
  langLabel.textContent = "Язык";

  const langGroup = document.createElement("div");
  langGroup.className = "voice-presets-panel__lang";
  langGroup.setAttribute("role", "radiogroup");
  langGroup.setAttribute("aria-label", "Язык голоса");

  // ----- Loading-индикатор ----------------------------------------------
  const loadingEl = document.createElement("div");
  loadingEl.className = "voice-presets-panel__loading";
  loadingEl.textContent = "Загрузка пресетов…";
  loadingEl.hidden = !(options.loading ?? false);

  // ----- Сборка DOM ------------------------------------------------------
  root.appendChild(heading);
  root.appendChild(presetsLabel);
  root.appendChild(presetsGroup);
  root.appendChild(langLabel);
  root.appendChild(langGroup);
  root.appendChild(loadingEl);
  parent.appendChild(root);

  // Mutable state -------------------------------------------------------
  let presets: VoicePresetInfo[] = options.presets ? [...options.presets] : [];
  let languages: VoiceLanguage[] = options.languages ? [...options.languages] : ["ru", "en"];
  // currentPreset — широкого типа: клик шлёт VoicePresetId (AV-28),
  // сервер через voice_set_ack может прислать любой VoicePreset (legacy).
  // При рендере кнопок фильтруем — legacy-пресеты не имеют UI-кнопки.
  let currentPreset: VoicePreset | null = options.currentPreset ?? null;
  let currentLanguage: VoiceLanguage | null = options.currentLanguage ?? null;
  let loading = options.loading ?? false;
  let visible = true;
  let disposed = false;
  const listeners = new Set<(v: boolean) => void>();

  function emitVisibility(): void {
    for (const cb of listeners) cb(visible);
  }

  function rebuildPresets(): void {
    presetsGroup.replaceChildren();
    for (const p of presets) {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "voice-presets-panel__chip";
      btn.setAttribute("data-preset", p.id);
      btn.setAttribute("aria-pressed", currentPreset === p.id ? "true" : "false");
      btn.textContent = p.name;
      btn.disabled = loading;
      btn.addEventListener("click", () => {
        if (loading || disposed) return;
        if (currentPreset === p.id) return;
        const prev = currentPreset;
        currentPreset = p.id;
        // Переподсветить кнопки без эмита — это уже синхронизировано
        // с mode_manager через callback в main.ts.
        for (const child of Array.from(presetsGroup.children) as HTMLButtonElement[]) {
          child.setAttribute(
            "aria-pressed",
            child.getAttribute("data-preset") === p.id ? "true" : "false"
          );
        }
        // Если выбранный пресет реально сменился — дёргаем callback.
        if (prev !== p.id) {
          try {
            options.onPresetChange?.(p.id);
          } catch (err) {
            // eslint-disable-next-line no-console
            console.warn("[voice-presets-panel] onPresetChange threw:", err);
          }
        }
      });
      presetsGroup.appendChild(btn);
    }
  }

  function rebuildLanguages(): void {
    langGroup.replaceChildren();
    for (const lang of languages) {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "voice-presets-panel__lang-btn";
      btn.setAttribute("role", "radio");
      btn.setAttribute("data-language", lang);
      btn.setAttribute("aria-checked", currentLanguage === lang ? "true" : "false");
      btn.textContent = LANG_LABELS[lang] ?? lang.toUpperCase();
      btn.disabled = loading;
      btn.addEventListener("click", () => {
        if (loading || disposed) return;
        if (currentLanguage === lang) return;
        const prev = currentLanguage;
        currentLanguage = lang;
        for (const child of Array.from(langGroup.children) as HTMLButtonElement[]) {
          child.setAttribute(
            "aria-checked",
            child.getAttribute("data-language") === lang ? "true" : "false"
          );
        }
        if (prev !== lang) {
          try {
            options.onLanguageChange?.(lang);
          } catch (err) {
            // eslint-disable-next-line no-console
            console.warn("[voice-presets-panel] onLanguageChange threw:", err);
          }
        }
      });
      langGroup.appendChild(btn);
    }
  }

  function refreshDisabled(): void {
    for (const child of Array.from(presetsGroup.children) as HTMLButtonElement[]) {
      child.disabled = loading;
    }
    for (const child of Array.from(langGroup.children) as HTMLButtonElement[]) {
      child.disabled = loading;
    }
    loadingEl.hidden = !loading;
  }

  rebuildPresets();
  rebuildLanguages();
  refreshDisabled();

  function setPresets(next: VoicePresetInfo[]): void {
    if (disposed) return;
    presets = [...next];
    // Если текущий пресет отсутствует в новом списке — сбрасываем в null,
    // чтобы UI не показывал подсветку несуществующего. Legacy-пресеты
    // (standard/friendly/...) не имеют UI-кнопки — это валидно.
    if (currentPreset && !presets.some((p) => p.id === currentPreset)) {
      currentPreset = null;
    }
    rebuildPresets();
  }

  function setLanguages(next: VoiceLanguage[]): void {
    if (disposed) return;
    languages = [...next];
    if (currentLanguage && !languages.includes(currentLanguage)) {
      currentLanguage = null;
    }
    rebuildLanguages();
  }

  function setCurrentPreset(preset: VoicePreset | null): void {
    if (disposed) return;
    if (currentPreset === preset) return;
    currentPreset = preset;
    // Если переданный пресет не из текущего списка (legacy или просто
    // неизвестный ID) — просто не подсвечиваем ничего, currentPreset
    // оставляем чтобы вернуть его обратно при следующем sync от сервера.
    for (const child of Array.from(presetsGroup.children) as HTMLButtonElement[]) {
      child.setAttribute(
        "aria-pressed",
        child.getAttribute("data-preset") === preset ? "true" : "false"
      );
    }
  }

  function setCurrentLanguage(language: VoiceLanguage | null): void {
    if (disposed) return;
    if (currentLanguage === language) return;
    currentLanguage = language;
    for (const child of Array.from(langGroup.children) as HTMLButtonElement[]) {
      child.setAttribute(
        "aria-checked",
        child.getAttribute("data-language") === language ? "true" : "false"
      );
    }
  }

  function setLoading(next: boolean): void {
    if (disposed) return;
    loading = next;
    refreshDisabled();
  }

  function setVisible(next: boolean): void {
    if (disposed) return;
    if (visible === next) return;
    visible = next;
    root.hidden = !next;
    emitVisibility();
  }

  function onChange(cb: (v: boolean) => void): () => void {
    listeners.add(cb);
    return () => {
      listeners.delete(cb);
    };
  }

  function dispose(): void {
    if (disposed) return;
    disposed = true;
    listeners.clear();
    if (root.parentNode === parent) {
      parent.removeChild(root);
    }
  }

  return {
    setPresets,
    setLanguages,
    setCurrentPreset,
    setCurrentLanguage,
    setLoading,
    setVisible,
    onChange,
    get isVisible(): boolean {
      return visible && !disposed;
    },
    get isDisposed(): boolean {
      return disposed;
    },
    get isLoading(): boolean {
      return loading && !disposed;
    },
    dispose
  };
}
