// HUD overlay для отображения текущего Captain Mode (Phase 2 §3.1).
//
// Текст в правом верхнем углу, рядом с lil-gui. Цвет/лейбл меняются при
// переходе. Поддерживает warning-режим (deadman около release).
//
// Не зависит от Three.js. Импортируется из main.ts, не тестируется в
// изоляции — это чисто визуальный компонент.

import type { CaptainMode, ModeChangeEvent } from "./mode_manager";

export interface ModeHudOptions {
  /** Куда монтировать overlay. По умолчанию body. */
  parent?: HTMLElement;
  /** Начальный режим (если ModeManager ещё не подписан). */
  initial?: CaptainMode;
}

export interface ModeHudHandle {
  /** Подписаться на изменения ModeManager. Возвращает unsubscriber. */
  attachModeManager(mgr: { getMode(): CaptainMode; subscribe(l: (e: ModeChangeEvent) => void): () => void }): () => void;
  /** Прямо установить режим (для тестов / init). */
  setMode(mode: CaptainMode): void;
  /** Показать / скрыть warning "Release B to stop!" (deadman близок к release). */
  setWarning(text: string | null): void;
  destroy(): void;
}

const MODE_META: Record<CaptainMode, { label: string; color: string }> = {
  explore: { label: "EXPLORE", color: "#7aa2f7" },
  teleop: { label: "TELEOP", color: "#2ec27e" },
  voice: { label: "VOICE", color: "#e0af68" },
  mixed: { label: "MIXED", color: "#bb9af7" }
};

export function createModeHud(opts: ModeHudOptions = {}): ModeHudHandle {
  const root = document.createElement("div");
  root.className = "captain-mode-hud";
  root.setAttribute("data-mode", opts.initial ?? "explore");
  root.style.position = "fixed";
  root.style.top = "12px";
  root.style.right = "12px";
  root.style.padding = "6px 10px";
  root.style.borderRadius = "6px";
  root.style.fontFamily = "monospace, ui-monospace, Menlo, Consolas";
  root.style.fontSize = "13px";
  root.style.fontWeight = "700";
  root.style.letterSpacing = "0.04em";
  root.style.zIndex = "1000";
  root.style.pointerEvents = "none";
  root.style.boxShadow = "0 2px 6px rgba(0,0,0,0.4)";
  root.style.userSelect = "none";

  const labelEl = document.createElement("span");
  labelEl.className = "captain-mode-hud__label";
  root.appendChild(labelEl);

  const warningEl = document.createElement("div");
  warningEl.className = "captain-mode-hud__warning";
  warningEl.style.position = "absolute";
  warningEl.style.top = "100%";
  warningEl.style.right = "0";
  warningEl.style.marginTop = "4px";
  warningEl.style.padding = "4px 8px";
  warningEl.style.background = "rgba(255, 80, 80, 0.85)";
  warningEl.style.color = "#fff";
  warningEl.style.borderRadius = "4px";
  warningEl.style.fontSize = "11px";
  warningEl.style.fontWeight = "600";
  warningEl.style.display = "none";
  root.appendChild(warningEl);

  const parent = opts.parent ?? document.body;
  parent.appendChild(root);

  function applyMode(mode: CaptainMode): void {
    const meta = MODE_META[mode];
    root.setAttribute("data-mode", mode);
    labelEl.textContent = `MODE: ${meta.label}`;
    root.style.background = "rgba(10, 13, 17, 0.85)";
    root.style.color = meta.color;
    root.style.border = `1px solid ${meta.color}`;
  }

  function setWarning(text: string | null): void {
    if (text == null) {
      warningEl.style.display = "none";
      warningEl.textContent = "";
    } else {
      warningEl.style.display = "block";
      warningEl.textContent = text;
    }
  }

  applyMode(opts.initial ?? "explore");

  return {
    attachModeManager(mgr): () => void {
      applyMode(mgr.getMode());
      return mgr.subscribe((ev) => applyMode(ev.next));
    },
    setMode(mode): void {
      applyMode(mode);
    },
    setWarning,
    destroy(): void {
      root.remove();
    }
  };
}