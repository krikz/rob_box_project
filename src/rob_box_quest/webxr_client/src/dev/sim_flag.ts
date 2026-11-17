// Флаг симулятора мостика (#3149). Отдельный крошечный модуль, чтобы
// entry.ts мог проверить `?sim=1`, не затягивая весь src/dev/ в основной
// бандл: сам симулятор грузится динамическим import() только по флагу.

/** `?sim=1` / `?sim=true` / `?sim` — включить симулятор. */
export function isSimRequested(search: string): boolean {
  const params = new URLSearchParams(search);
  if (!params.has("sim")) return false;
  const v = (params.get("sim") ?? "").toLowerCase();
  return v === "" || v === "1" || v === "true" || v === "yes";
}

/** `?sim_latency=40` — задержка мок-сети в одну сторону, мс (0..2000). */
export function simLatencyMs(search: string): number {
  const raw = Number(new URLSearchParams(search).get("sim_latency"));
  return Number.isFinite(raw) ? Math.min(2000, Math.max(0, Math.round(raw))) : 0;
}
