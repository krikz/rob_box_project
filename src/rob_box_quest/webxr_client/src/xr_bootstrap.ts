// xr_bootstrap — тонкая обёртка над WebXR Device API.
//
// Зачем: main.ts не должен падать если navigator.xr нет (desktop Chrome /
// обычный браузер в Quest — там есть, в десктопной dev-сессии — нет), а
// также чтобы можно было подменить navigator.xr в unit-тестах.
//
// Дизайн Phase 1.5 §2, §6:
//   - Phase 1: immersive-vr (Quest standalone headset).
//   - Phase 2: immersive-ar (passthrough).
//   - requiredFeatures: ['local-floor'] — пользователь стоит, есть физический пол.
//
// Что НЕ покрываем:
//   - pollXrInput — это xr_teleop.ts.
//   - cleanup listeners внутри session.end кроме самого факта обнуления —
//     этим занимается вызывающий код (main.ts ставит свои listener'ы через bindSession).

export type XrMode = "immersive-vr" | "immersive-ar" | "inline";

/** Структура, которую нативный API ожидает от navigator.xr (минимально). */
export interface FakeNavigatorXr {
  isSessionSupported(mode: XrMode): Promise<boolean>;
  requestSession(mode: XrMode, options?: XRSessionInit): Promise<XRSession>;
}

export interface XrBootstrap {
  /** true, если navigator.xr есть и устройство поддерживает этот режим. */
  isSupported(mode: XrMode): Promise<boolean>;
  /** Запрашивает сессию; пробрасывает requiredFeatures: ['local-floor'] + optionalFeatures ['hand-tracking']. */
  requestSession(mode: XrMode): Promise<XRSession>;
  /** Помечает сессию как активной; вызывающий код получит session в XR-сцене. */
  bindSession(session: XRSession): void;
  /** Текущая привязанная сессия (или null). */
  getSession(): XRSession | null;
  /** Корректно завершает сессию (вызывает session.end). */
  endSession(): Promise<void>;
  /** true после bindSession, false после session.end / endSession. */
  isActive(): boolean;
}

export function createXrBootstrap(): XrBootstrap {
  let activeSession: XRSession | null = null;

  function getXr(): FakeNavigatorXr | null {
    const nav = (globalThis as unknown as { navigator?: { xr?: FakeNavigatorXr } }).navigator;
    return nav?.xr ?? null;
  }

  async function isSupported(mode: XrMode): Promise<boolean> {
    const xr = getXr();
    if (!xr || typeof xr.isSessionSupported !== "function") return false;
    try {
      return await xr.isSessionSupported(mode);
    } catch {
      // Некоторые браузеры на isSessionSupported кидают SecurityError
      // без user-activation — для UI-кнопки это нормально, считаем unsupported.
      return false;
    }
  }

  async function requestSession(mode: XrMode): Promise<XRSession> {
    const xr = getXr();
    if (!xr || typeof xr.requestSession !== "function") {
      throw new Error("WebXR not available");
    }
    // local-floor нужен для физического пола (Quest standalone — всегда есть).
    // hand-tracking — optional, если браузер/устройство не поддерживает,
    // XR controller fallback всё равно работает (дизайн §3.5).
    const session = await xr.requestSession(mode, {
      requiredFeatures: ["local-floor"],
      optionalFeatures: ["hand-tracking"]
    });
    return session;
  }

  function bindSession(session: XRSession): void {
    activeSession = session;
    session.addEventListener("end", () => {
      if (activeSession === session) activeSession = null;
    });
  }

  function getSession(): XRSession | null {
    return activeSession;
  }

  async function endSession(): Promise<void> {
    const s = activeSession;
    if (!s) return;
    try {
      await s.end();
    } catch {
      // ignore — сессия и так завершится через 'end' event.
    }
    if (activeSession === s) activeSession = null;
  }

  function isActive(): boolean {
    return activeSession !== null;
  }

  return { isSupported, requestSession, bindSession, getSession, endSession, isActive };
}
