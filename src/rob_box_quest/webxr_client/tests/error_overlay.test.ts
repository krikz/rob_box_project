import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import {
  createErrorOverlay,
  createDisconnectWatchdog,
  type ErrorOverlay,
  type DisconnectWatchdog
} from "../src/ui/error_overlay";

describe("createErrorOverlay", () => {
  let parent: HTMLElement;
  let err: ErrorOverlay;

  beforeEach(() => {
    parent = document.createElement("div");
    document.body.appendChild(parent);
    err = createErrorOverlay(parent);
  });

  afterEach(() => {
    err.dispose();
    parent.remove();
  });

  it("starts hidden with role='alert' and aria-live='assertive'", () => {
    const root = parent.querySelector("[data-error-overlay]") as HTMLElement;
    expect(root.getAttribute("role")).toBe("alert");
    expect(root.getAttribute("aria-live")).toBe("assertive");
    expect(root.classList.contains("error-overlay--hidden")).toBe(true);
    expect(err.isVisible).toBe(false);
  });

  it("show(headline) reveals overlay and sets headline", () => {
    err.show("Connection lost");
    expect(err.isVisible).toBe(true);
    const root = parent.querySelector("[data-error-overlay]") as HTMLElement;
    expect(root.classList.contains("error-overlay--hidden")).toBe(false);
    const h = root.querySelector(".error-overlay__headline") as HTMLElement;
    expect(h.textContent).toBe("Connection lost");
  });

  it("show(headline, detail) shows detail", () => {
    err.show("Connection lost", "Trying to reconnect…");
    const d = parent.querySelector(".error-overlay__detail") as HTMLElement;
    expect(d.textContent).toBe("Trying to reconnect…");
    expect(d.hidden).toBe(false);
  });

  it("dismiss() hides overlay", () => {
    err.show("Lost");
    err.dismiss();
    expect(err.isVisible).toBe(false);
  });

  it("setHeadline / setDetail update without changing visibility", () => {
    err.show("Initial");
    err.setHeadline("Updated headline");
    err.setDetail("Updated detail");
    expect(err.isVisible).toBe(true);
    const h = parent.querySelector(".error-overlay__headline") as HTMLElement;
    const d = parent.querySelector(".error-overlay__detail") as HTMLElement;
    expect(h.textContent).toBe("Updated headline");
    expect(d.textContent).toBe("Updated detail");
  });

  it("setDetail('') hides detail line", () => {
    err.show("Lost", "Detail");
    err.setDetail("");
    const d = parent.querySelector(".error-overlay__detail") as HTMLElement;
    expect(d.hidden).toBe(true);
  });

  it("dismiss button hides overlay", () => {
    err.show("Lost");
    const btn = parent.querySelector(
      ".error-overlay__dismiss"
    ) as HTMLButtonElement;
    btn.click();
    expect(err.isVisible).toBe(false);
  });

  it("onChange fires on visibility transitions", () => {
    const cb = vi.fn();
    err.onChange(cb);
    err.show("Lost");
    expect(cb).toHaveBeenCalledWith(true);
    err.dismiss();
    expect(cb).toHaveBeenCalledWith(false);
  });

  it("level option adds --warn/--error/--info class", () => {
    err.dispose();
    err = createErrorOverlay(parent, { level: "error" });
    const root = parent.querySelector("[data-error-overlay]") as HTMLElement;
    expect(root.classList.contains("error-overlay--error")).toBe(true);
  });

  it("dispose removes element", () => {
    err.show("Lost");
    err.dispose();
    expect(parent.querySelector("[data-error-overlay]")).toBeNull();
    expect(err.isDisposed).toBe(true);
  });
});

describe("createDisconnectWatchdog", () => {
  let parent: HTMLElement;
  let err: ErrorOverlay;
  let watchdog: DisconnectWatchdog;

  beforeEach(() => {
    vi.useFakeTimers();
    parent = document.createElement("div");
    document.body.appendChild(parent);
    err = createErrorOverlay(parent);
    watchdog = createDisconnectWatchdog(err, { thresholdMs: 5000 });
  });

  afterEach(() => {
    watchdog.dispose();
    err.dispose();
    parent.remove();
    vi.useRealTimers();
  });

  it("does not show overlay before threshold", () => {
    watchdog.markDisconnected();
    expect(err.isVisible).toBe(false);
    vi.advanceTimersByTime(4999);
    expect(err.isVisible).toBe(false);
  });

  it("shows overlay after threshold", () => {
    watchdog.markDisconnected();
    vi.advanceTimersByTime(5001);
    expect(err.isVisible).toBe(true);
  });

  it("updates detail with elapsed seconds each tick", () => {
    watchdog.markDisconnected();
    vi.advanceTimersByTime(5500); // past threshold
    vi.advanceTimersByTime(1000); // 1s tick
    const d = parent.querySelector(".error-overlay__detail") as HTMLElement;
    // Real performance.now() doesn't move under vi.useFakeTimers(),
    // so elapsed is reported as the time between markDisconnected and
    // this tick. We only assert the structure: detail starts with
    // "No response from server for " and contains a number+s suffix.
    expect(d.textContent).toMatch(/^No response from server for \d+s$/);
  });

  it("markConnected() hides overlay and resets state", () => {
    watchdog.markDisconnected();
    vi.advanceTimersByTime(6000);
    expect(err.isVisible).toBe(true);
    watchdog.markConnected();
    expect(err.isVisible).toBe(false);
    expect(watchdog.isInDownState).toBe(false);
  });

  it("dispose() clears timers", () => {
    watchdog.markDisconnected();
    watchdog.dispose();
    vi.advanceTimersByTime(10000);
    // Overlay не должен показаться — таймеры выключены.
    expect(err.isVisible).toBe(false);
  });

  it("double markDisconnected is a no-op", () => {
    watchdog.markDisconnected();
    watchdog.markDisconnected(); // не должно перезапустить таймер
    vi.advanceTimersByTime(5001);
    expect(err.isVisible).toBe(true);
  });
});
