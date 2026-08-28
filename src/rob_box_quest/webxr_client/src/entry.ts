// HTML-entry: bootstrap после загрузки DOM.

import { bootstrap } from "./main";

function findEl<T extends HTMLElement>(id: string): T {
  const el = document.getElementById(id);
  if (!el) throw new Error(`element #${id} not found`);
  return el as T;
}

window.addEventListener("DOMContentLoaded", () => {
  const canvas = findEl<HTMLCanvasElement>("scene");
  const pinOverlay = findEl<HTMLDivElement>("pin-overlay");
  const pinForm = findEl<HTMLFormElement>("pin-form");
  const pinInput = findEl<HTMLInputElement>("pin-input");
  const pinError = findEl<HTMLParagraphElement>("pin-error");
  const statusEl = findEl<HTMLDivElement>("status");
  const modeHud = findEl<HTMLDivElement>("mode-hud");
  const resetLayoutBtn = findEl<HTMLButtonElement>("reset-layout-btn");

  bootstrap({
    canvas,
    pinOverlay,
    pinForm,
    pinInput,
    pinError,
    statusEl,
    modeHud,
    resetLayoutBtn,
    pin: ""
  });
});