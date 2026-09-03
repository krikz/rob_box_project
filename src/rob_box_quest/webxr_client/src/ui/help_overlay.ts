// Help overlay: список горячих клавиш Captain Bridge.
// По умолчанию скрыт; тогглится клавишей H (работает только когда
// PIN-overlay закрыт — иначе H печаталась бы в PIN-инпуте).
//
// Дизайн: центрированная карточка со списком hotkey → action. Закрывается
// по H, Escape, клику вне карточки. Никаких зависимостей от Three.js —
// чистая DOM-логика.
//
// Использование:
//   const help = createHelpOverlay(parentEl);
//   help.toggle();          // показать/скрыть
//   help.show();
//   help.hide();
//   help.dispose();
//
// API:
export interface HelpOverlay {
  show(): void;
  hide(): void;
  toggle(): void;
  /** true если overlay сейчас видим. */
  readonly isVisible: boolean;
  /** Подписаться на toggle-события (для интеграции с другими overlay). */
  onChange(cb: (visible: boolean) => void): () => void;
  /** Разрушить overlay. */
  dispose(): void;
}

export interface HotkeyEntry {
  /** Комбинация клавиш, отображаемая пользователю (например "H"). */
  key: string;
  /** Описание действия. */
  description: string;
  /** Опциональная категория (Desktop / WebXR / Global). */
  category?: "Desktop" | "WebXR" | "Global";
}

export const DEFAULT_HOTKEYS: ReadonlyArray<HotkeyEntry> = Object.freeze([
  { key: "W A S D", description: "Движение робота (desktop fallback)", category: "Desktop" },
  { key: "Space", description: "Boost (×1.5, desktop)", category: "Desktop" },
  { key: "E", description: "Emergency stop (desktop)", category: "Desktop" },
  { key: "M", description: "Показать / скрыть 3D-панель режима аватара", category: "Desktop" },
  { key: "R", description: "Сброс раскладки панелей к default", category: "Desktop" },
  { key: "L", description: "Left stick forward/back/strafe", category: "WebXR" },
  { key: "R stick click", description: "Arm / disarm teleop (toggle)", category: "WebXR" },
  { key: "L grip", description: "Voice mode: passthrough (рация)", category: "WebXR" },
  { key: "R grip", description: "Voice mode: robot_voice (STT→LLM→TTS)", category: "WebXR" },
  { key: "B / Y", description: "Emergency stop (controller)", category: "WebXR" },
  { key: "H", description: "Показать / скрыть эту подсказку", category: "Global" },
  { key: "Esc", description: "Закрыть overlay / exit VR", category: "Global" }
]);

export function createHelpOverlay(
  parent: HTMLElement,
  hotkeys: ReadonlyArray<HotkeyEntry> = DEFAULT_HOTKEYS
): HelpOverlay {
  const root = document.createElement("div");
  root.className = "help-overlay help-overlay--hidden";
  root.setAttribute("data-help-overlay", "");
  root.setAttribute("role", "dialog");
  root.setAttribute("aria-modal", "true");
  root.setAttribute("aria-labelledby", "help-overlay-title");

  const card = document.createElement("div");
  card.className = "help-overlay__card";

  const title = document.createElement("h2");
  title.id = "help-overlay-title";
  title.className = "help-overlay__title";
  title.textContent = "Captain Bridge — горячие клавиши";

  const hint = document.createElement("p");
  hint.className = "help-overlay__hint";
  hint.textContent = "Нажмите H или Esc чтобы закрыть.";

  const table = document.createElement("dl");
  table.className = "help-overlay__hotkeys";

  // Группируем по category (Global последний, чтобы он шёл в конце).
  const groups = new Map<string, HotkeyEntry[]>();
  for (const h of hotkeys) {
    const cat = h.category ?? "Global";
    if (!groups.has(cat)) groups.set(cat, []);
    groups.get(cat)!.push(h);
  }
  const order: Array<"Desktop" | "WebXR" | "Global"> = ["Desktop", "WebXR", "Global"];
  for (const cat of order) {
    const items = groups.get(cat);
    if (!items || items.length === 0) continue;
    const groupHeader = document.createElement("dt");
    groupHeader.className = "help-overlay__group";
    groupHeader.textContent = cat;
    table.appendChild(groupHeader);
    for (const h of items) {
      const term = document.createElement("dt");
      term.className = "help-overlay__term";
      const kbd = document.createElement("kbd");
      kbd.className = "help-overlay__key";
      kbd.textContent = h.key;
      term.appendChild(kbd);

      const desc = document.createElement("dd");
      desc.className = "help-overlay__desc";
      desc.textContent = h.description;

      table.appendChild(term);
      table.appendChild(desc);
    }
  }

  card.appendChild(title);
  card.appendChild(hint);
  card.appendChild(table);
  root.appendChild(card);
  parent.appendChild(root);

  let visible = false;
  let disposed = false;
  const listeners = new Set<(v: boolean) => void>();

  function emit(): void {
    for (const cb of listeners) cb(visible);
  }

  function show(): void {
    if (disposed) return;
    if (visible) return;
    root.classList.remove("help-overlay--hidden");
    visible = true;
    emit();
  }

  function hide(): void {
    if (disposed) return;
    if (!visible) return;
    root.classList.add("help-overlay--hidden");
    visible = false;
    emit();
  }

  function toggle(): void {
    if (visible) hide();
    else show();
  }

  function onKeydown(ev: KeyboardEvent): void {
    // Игнорируем, если фокус в input/textarea (PIN, future chat).
    const target = ev.target as HTMLElement | null;
    const tag = target?.tagName?.toLowerCase();
    if (tag === "input" || tag === "textarea" || target?.isContentEditable) {
      return;
    }
    if (ev.key === "h" || ev.key === "H") {
      ev.preventDefault();
      toggle();
    } else if (ev.key === "Escape" && visible) {
      ev.preventDefault();
      hide();
    }
  }

  function onBackdropClick(ev: MouseEvent): void {
    // Закрываем по клику вне карточки.
    if (ev.target === root) {
      hide();
    }
  }

  document.addEventListener("keydown", onKeydown);
  root.addEventListener("click", onBackdropClick);

  function onChange(cb: (v: boolean) => void): () => void {
    listeners.add(cb);
    return () => {
      listeners.delete(cb);
    };
  }

  function dispose(): void {
    if (disposed) return;
    disposed = true;
    document.removeEventListener("keydown", onKeydown);
    root.removeEventListener("click", onBackdropClick);
    listeners.clear();
    if (root.parentNode === parent) {
      parent.removeChild(root);
    }
  }

  return {
    show,
    hide,
    toggle,
    onChange,
    dispose,
    get isVisible(): boolean {
      return visible && !disposed;
    }
  };
}
