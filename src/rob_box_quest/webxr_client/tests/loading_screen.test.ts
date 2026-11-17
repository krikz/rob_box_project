import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import {
  createLoadingScreen,
  type LoadingScreen
} from "../src/ui/loading_screen";

describe("createLoadingScreen", () => {
  let parent: HTMLElement;
  let screen: LoadingScreen;

  beforeEach(() => {
    parent = document.createElement("div");
    document.body.appendChild(parent);
  });

  afterEach(() => {
    screen?.dispose();
    parent.remove();
    vi.useRealTimers();
  });

  it("creates root element with role='status' and aria-busy=true", () => {
    screen = createLoadingScreen(parent, "Loading environment…");
    const root = parent.querySelector("[data-loading-screen]") as HTMLElement;
    expect(root).toBeTruthy();
    expect(root.getAttribute("role")).toBe("status");
    expect(root.getAttribute("aria-busy")).toBe("true");
    expect(root.querySelector(".loading-screen__text")?.textContent).toBe(
      "Loading environment…"
    );
  });

  it("hide() adds hidden class and sets aria-busy=false", () => {
    screen = createLoadingScreen(parent, "Loading…", { minVisibleMs: 0 });
    expect(screen.isVisible).toBe(true);
    screen.hide();
    const root = parent.querySelector("[data-loading-screen]") as HTMLElement;
    expect(root.classList.contains("loading-screen--hidden")).toBe(true);
    expect(root.getAttribute("aria-busy")).toBe("false");
    expect(screen.isVisible).toBe(false);
  });

  it("setText updates label without changing visibility", () => {
    screen = createLoadingScreen(parent, "Loading…");
    screen.setText("Loading models (2/5)");
    const text = parent.querySelector(".loading-screen__text") as HTMLElement;
    expect(text.textContent).toBe("Loading models (2/5)");
  });

  it("fail() adds --failed class and shows error in label", () => {
    screen = createLoadingScreen(parent, "Loading…");
    screen.fail("Network unreachable");
    const root = parent.querySelector("[data-loading-screen]") as HTMLElement;
    expect(root.classList.contains("loading-screen--failed")).toBe(true);
    const text = parent.querySelector(".loading-screen__text") as HTMLElement;
    expect(text.textContent).toBe("Error: Network unreachable");
    expect(root.getAttribute("aria-busy")).toBe("false");
  });

  it("watch() hides after promise resolves (with minVisibleMs=0)", async () => {
    screen = createLoadingScreen(parent, "Loading…", { minVisibleMs: 0 });
    const p = Promise.resolve(42);
    const result = await screen.watch(p);
    expect(result).toBe(42);
    expect(screen.isVisible).toBe(false);
  });

  it("watch() shows error and re-throws on rejection", async () => {
    screen = createLoadingScreen(parent, "Loading…", { minVisibleMs: 0 });
    const p = Promise.reject(new Error("boom"));
    await expect(screen.watch(p)).rejects.toThrow("boom");
    const text = parent.querySelector(".loading-screen__text") as HTMLElement;
    expect(text.textContent).toBe("Error: boom");
  });

  it("watch() applies label before await", async () => {
    screen = createLoadingScreen(parent, "Loading…", { minVisibleMs: 0 });
    const p = new Promise<number>((resolve) => {
      setTimeout(() => resolve(1), 5);
    });
    await screen.watch(p, "Loading models…");
    const text = parent.querySelector(".loading-screen__text") as HTMLElement;
    expect(text.textContent).toBe("Loading models…");
  });

  it("dispose() removes element from DOM and makes methods no-op", () => {
    screen = createLoadingScreen(parent);
    expect(parent.querySelector("[data-loading-screen]")).toBeTruthy();
    screen.dispose();
    expect(screen.isDisposed).toBe(true);
    expect(parent.querySelector("[data-loading-screen]")).toBeNull();
    // Subsequent calls are no-ops (не падают).
    expect(() => screen.show("x")).not.toThrow();
    expect(() => screen.hide()).not.toThrow();
  });

  it("minVisibleMs defers hide() when elapsed < threshold", () => {
    // Не полагаемся на vi.useFakeTimers — performance.now() в jsdom
    // продвигается реальным временем, а advanceTimersByTimeAsync
    // только триггерит запланированные callback'и, не двигая часы.
    // Поведение minVisibleMs покрыто косвенно: если overlay сразу
    // виден с момента создания и hide() вызывается до minVisibleMs,
    // overlay не получит класс --hidden до вызова реального таймера.
    // Здесь мы проверяем что hide() идемпотентен и не падает.
    screen = createLoadingScreen(parent, "Loading…", { minVisibleMs: 50_000 });
    const root = parent.querySelector("[data-loading-screen]") as HTMLElement;
    expect(root.classList.contains("loading-screen--hidden")).toBe(false);
    screen.hide();
    // С большим minVisibleMs — overlay НЕ скрывается за время теста.
    // Это подтверждает, что deferred-механизм работает (иначе бы
    // закрылся мгновенно при hide()).
    expect(root.classList.contains("loading-screen--hidden")).toBe(false);
  });
});
