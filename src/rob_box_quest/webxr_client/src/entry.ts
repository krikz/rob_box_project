// HTML-entry: bootstrap после загрузки DOM.

import { bootstrap } from "./main";
import { isSimRequested, simLatencyMs } from "./dev/sim_flag";

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
  // body используется для монтирования overlay'ов (loading/error/help),
  // чтобы они перекрывали и canvas, и HUD. help-toggle опционален
  // — если кнопку убрали из DOM, мост всё равно работает (H по-прежнему
  // тогглит overlay).
  const body = document.body;
  const helpToggle = document.getElementById("help-toggle");

  const common = { canvas, pinOverlay, pinForm, pinInput, pinError, statusEl, body, helpToggle };

  // `?sim=1` — симулятор мостика без шлема и робота (#3149). Код мока
  // грузится отдельным чанком только по флагу; без флага путь прежний.
  if (isSimRequested(location.search)) {
    void import("./dev/sim_entry").then(({ startSim }) =>
      startSim(common, { latencyMs: simLatencyMs(location.search) })
    );
    return;
  }

  bootstrap({ ...common, pin: "" });
});