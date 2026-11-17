// xr_bootstrap: тонкий слой над WebXR Device API.
// Изолируем navigator.xr, чтобы:
//   1) main.ts мог безопасно спросить "есть ли XR?" без try/catch на каждом вызове,
//   2) тесты могли подсунуть FakeNavigatorXr.
// Дизайн Phase 1.5 §2, §6: immersive-vr в Phase 1, immersive-ar — Phase 2.
//
// Что НЕ покрываем:
//   - сам requestSession внутри (поведение нативного API) — мы только
//     пробрасываем аргументы и пробрасываем ошибки.
//   - pollXrInput — это xr_teleop.test.ts (см. teleop_fsm.test.ts как
//     ближайший аналог).

import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import {
  createXrBootstrap,
  type XrBootstrap,
  type FakeNavigatorXr
} from "../src/xr_bootstrap";

class FakeXRSession {
  static instances: FakeXRSession[] = [];
  ended = false;
  listeners = new Map<string, Array<() => void>>();
  inputSources: unknown[] = [];
  constructor(public mode: string) {
    FakeXRSession.instances.push(this);
  }
  addEventListener(name: string, fn: () => void): void {
    const arr = this.listeners.get(name) ?? [];
    arr.push(fn);
    this.listeners.set(name, arr);
  }
  removeEventListener(name: string, fn: () => void): void {
    const arr = this.listeners.get(name);
    if (!arr) return;
    const i = arr.indexOf(fn);
    if (i >= 0) arr.splice(i, 1);
  }
  /** Test helper: fire 'end' event. */
  fireEnd(): void {
    this.ended = true;
    for (const fn of this.listeners.get("end") ?? []) fn();
  }
  async end(): Promise<void> {
    this.fireEnd();
  }
}

function installFakeNavigator(supported: Record<string, boolean>): FakeNavigatorXr {
  const fake: FakeNavigatorXr = {
    isSessionSupported: vi.fn(async (mode: string) => supported[mode] ?? false),
    // Тип XRSession — structural duck-typed DOM-интерфейс с 20+ свойствами.
    // Для unit-тестов достаточно FakeXRSession через unknown.
    requestSession: vi.fn(async (mode: string) => new FakeXRSession(mode) as unknown as XRSession)
  };
  (globalThis as unknown as { navigator: { xr: FakeNavigatorXr } }).navigator = { xr: fake };
  return fake;
}

describe("xr_bootstrap", () => {
  let savedNavigator: unknown;

  beforeEach(() => {
    savedNavigator = (globalThis as unknown as { navigator?: unknown }).navigator;
    FakeXRSession.instances = [];
  });

  afterEach(() => {
    if (savedNavigator !== undefined) {
      (globalThis as unknown as { navigator?: unknown }).navigator = savedNavigator;
    } else {
      delete (globalThis as unknown as { navigator?: unknown }).navigator;
    }
    vi.restoreAllMocks();
  });

  it("isSupported('immersive-vr') returns true when navigator.xr reports true", async () => {
    installFakeNavigator({ "immersive-vr": true, "immersive-ar": false });
    const xr: XrBootstrap = createXrBootstrap();
    expect(await xr.isSupported("immersive-vr")).toBe(true);
  });

  it("isSupported('immersive-ar') returns false when navigator.xr reports false", async () => {
    installFakeNavigator({ "immersive-vr": true, "immersive-ar": false });
    const xr: XrBootstrap = createXrBootstrap();
    expect(await xr.isSupported("immersive-ar")).toBe(false);
  });

  it("isSupported returns false when navigator.xr is missing (desktop Chrome)", async () => {
    (globalThis as unknown as { navigator: Record<string, unknown> }).navigator = {};
    const xr: XrBootstrap = createXrBootstrap();
    expect(await xr.isSupported("immersive-vr")).toBe(false);
    expect(await xr.isSupported("immersive-ar")).toBe(false);
  });

  it("requestSession calls navigator.xr.requestSession with required local-floor", async () => {
    const fake = installFakeNavigator({ "immersive-vr": true });
    const xr: XrBootstrap = createXrBootstrap();
    const session = await xr.requestSession("immersive-vr");
    expect(fake.requestSession).toHaveBeenCalledTimes(1);
    const call = (fake.requestSession as unknown as { mock: { calls: unknown[][] } }).mock.calls[0];
    expect(call[0]).toBe("immersive-vr");
    const opts = call[1] as { requiredFeatures: string[] };
    expect(opts.requiredFeatures).toContain("local-floor");
    expect(session).toBeInstanceOf(FakeXRSession);
  });

  it("tracks active session: enter() → active true; session.end → active false", async () => {
    installFakeNavigator({ "immersive-vr": true });
    const xr: XrBootstrap = createXrBootstrap();
    expect(xr.isActive()).toBe(false);

    const session = (await xr.requestSession("immersive-vr")) as unknown as FakeXRSession;
    xr.bindSession(session as unknown as XRSession);
    expect(xr.isActive()).toBe(true);

    session.fireEnd();
    expect(xr.isActive()).toBe(false);
  });

  it("endSession() calls session.end and clears active state", async () => {
    installFakeNavigator({ "immersive-vr": true });
    const xr: XrBootstrap = createXrBootstrap();
    const session = (await xr.requestSession("immersive-vr")) as unknown as FakeXRSession;
    xr.bindSession(session as unknown as XRSession);
    const endSpy = vi.spyOn(session, "end");
    await xr.endSession();
    expect(endSpy).toHaveBeenCalledTimes(1);
    expect(xr.isActive()).toBe(false);
  });
});
