// Entry point для Phase 2.2 telemetry demo / minimal client.
//
// Phase 2.2 фокус — только telemetry reporter. Полный Captain Bridge UI
// (video panels, lidar overlay, teleop) добавляется в Phase 2.3+ карточках.
// Этот entry нужен чтобы можно было запустить `npm run dev`, увидеть
// console-вывод FPS/GPU/VRAM и убедиться, что WSS шлёт TELEMETRY_PERF.
//
// Использование:
//   import { bootstrap } from "./main";
//   bootstrap({ url: "wss://10.1.1.11:8443/quest", pin: "123456" });
//
// Query string:
//   ?telemetry=off   — opt-out (см. ADR-0032 §3.5).

import { PerfReporter, isTelemetryOptOut, type PerfSceneContext } from "./perf/reporter";
import { TelemetryConnection } from "./wire/connection";

export interface BootstrapOpts {
  url: string;
  pin: string;
  /** Текущая XR session (если null — desktop mode). */
  xrSession?: XRSession | null;
  /** Scene context (Three.js renderer для GPU timer / VRAM / XR framebuffer). */
  scene?: PerfSceneContext;
  /** Лог в console (default: console.log). */
  log?: (line: string) => void;
}

/**
 * Полный bootstrap: создаёт TelemetryConnection + PerfReporter.
 *
 * При ?telemetry=off — no-op (только лог). Это покрывает dev-сессии.
 */
export function bootstrap(opts: BootstrapOpts): {
  reporter: PerfReporter | null;
  connection: TelemetryConnection | null;
  dispose: () => void;
} {
  const log = opts.log ?? ((line: string) => console.log(line));

  // Opt-out.
  if (typeof location !== "undefined" && isTelemetryOptOut(location.search)) {
    log("[telemetry] disabled via ?telemetry=off");
    return { reporter: null, connection: null, dispose: () => {} };
  }

  const conn = new TelemetryConnection({
    url: opts.url,
    pin: opts.pin,
    clientVersion: "0.2.0",
    subprotocol: "robbox-quest-v1",
    onLatencyChange: (ms) => log(`[telemetry] RTT=${ms !== null ? ms.toFixed(1) : "?"}ms`),
    onStateChange: (s) => log(`[telemetry] state=${s}`),
  });
  conn.connect();

  const scene: PerfSceneContext = {
    xrSession: opts.xrSession ?? null,
    renderer: opts.scene?.renderer ?? null,
  };

  const reporter = new PerfReporter(
    {
      source: opts.xrSession ? "webxr" : "desktop",
      emit: (payload) => {
        const ok = conn.sendTelemetry(payload);
        if (!ok) {
          // Сервер недоступен — копим в console для отладки.
          log(`[telemetry] payload queued: ${JSON.stringify(payload)}`);
        }
      },
      log: (line) => log(line),
      getLatencyMs: () => conn.getLatencyMs(),
      getQuestMetrics: () => null, // Phase 2.3+: интеграция с QuestMetrics JS API.
    },
    scene
  );
  reporter.start(opts.xrSession ?? null);

  return {
    reporter,
    connection: conn,
    dispose: () => {
      reporter.dispose();
      conn.close();
    },
  };
}

// Side-effect: при прямой загрузке скрипта в HTML — автозапуск с дефолтами.
// Полная интеграция с Three.js scene будет в Phase 2.3+.
if (typeof window !== "undefined") {
  (window as unknown as { __questBootstrap?: typeof bootstrap }).__questBootstrap = bootstrap;
}