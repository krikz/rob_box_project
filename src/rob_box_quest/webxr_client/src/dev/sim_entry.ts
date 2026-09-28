// Вход в симулятор мостика (#3149): `index.html?sim=1` (или `npm run dev:sim`).
//
// Поднимает мок-робота в этой же вкладке, отдаёт bootstrap'у его WebSocket-
// конструктор (штатный шов Connection) и сам «вводит» PIN — оверлей
// исчезает, как только мок ответит WELCOME. Дальше мостик работает ровно
// тем же кодом, что с живым роботом.

import { bootstrap } from "../main";
import { createMockWebSocketCtor, MockRobot } from "./mock_robot";
import { createCanvasCameraRenderer } from "./sim_camera";

/** PIN, который симулятор вводит сам. Мок принимает любой. */
export const SIM_PIN = "000000";

type BootstrapOptions = Parameters<typeof bootstrap>[0];

export interface SimHandle {
  robot: MockRobot;
  dispose(): void;
}

/** Бейдж «SIM» в DOM-оверлее: оператор не должен спутать мок с роботом. */
export function mountSimBadge(parent: HTMLElement): HTMLElement {
  const badge = document.createElement("div");
  badge.className = "sim-badge";
  badge.setAttribute("data-sim-badge", "");
  badge.title = "Симулятор мостика: данные идут от мок-робота в браузере, не от настоящего робота";
  badge.textContent = "SIM · мок-робот";
  parent.appendChild(badge);
  return badge;
}

export function startSim(
  opts: Omit<BootstrapOptions, "WebSocketCtor" | "url" | "pin">,
  simOpts: { latencyMs?: number } = {}
): SimHandle {
  const cameraRenderer = createCanvasCameraRenderer();
  if (!cameraRenderer) {
    // Без canvas видеопанели останутся пустыми — говорим об этом в консоли.
    console.warn("[sim] canvas недоступен — камеры симулятора выключены");
  }
  const robot = new MockRobot({ cameraRenderer, latencyMs: simOpts.latencyMs ?? 0 });
  robot.start();
  const handle = bootstrap({
    ...opts,
    url: "sim://mock-robot/quest",
    pin: "",
    WebSocketCtor: createMockWebSocketCtor(robot)
  });
  const badge = mountSimBadge(opts.statusEl.parentElement ?? opts.body);
  document.title = `SIM · ${document.title}`;
  // Для отладки из консоли: window.__robBoxSim.robot.state и т.п.
  (window as unknown as { __robBoxSim?: unknown }).__robBoxSim = { robot };
  // Сабмит через форму — тот же путь, что у человека (валидация PIN,
  // openConnection, попытка авто-входа в VR).
  opts.pinInput.value = SIM_PIN;
  opts.pinForm.requestSubmit();
  return {
    robot,
    dispose(): void {
      handle.dispose();
      robot.stop();
      badge.remove();
    }
  };
}
