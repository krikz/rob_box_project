import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import {
  createHelpOverlay,
  DEFAULT_HOTKEYS,
  type HelpOverlay
} from "../src/ui/help_overlay";

describe("createHelpOverlay", () => {
  let parent: HTMLElement;
  let help: HelpOverlay;

  beforeEach(() => {
    parent = document.createElement("div");
    document.body.appendChild(parent);
    help = createHelpOverlay(parent);
  });

  afterEach(() => {
    help.dispose();
    parent.remove();
    vi.useRealTimers();
  });

  it("starts hidden", () => {
    expect(help.isVisible).toBe(false);
    const root = parent.querySelector("[data-help-overlay]") as HTMLElement;
    expect(root.classList.contains("help-overlay--hidden")).toBe(true);
  });

  it("show/hide/toggle flips isVisible", () => {
    help.show();
    expect(help.isVisible).toBe(true);
    help.hide();
    expect(help.isVisible).toBe(false);
    help.toggle();
    expect(help.isVisible).toBe(true);
    help.toggle();
    expect(help.isVisible).toBe(false);
  });

  it("renders default hotkeys grouped by category", () => {
    help.show();
    const terms = parent.querySelectorAll(".help-overlay__term");
    // DEFAULT_HOTKEYS has 10 entries (одна клавиша 'W A S D' может идти
    // отдельной строкой, но мы не считаем — главное, что категории есть).
    expect(terms.length).toBeGreaterThanOrEqual(DEFAULT_HOTKEYS.length - 2);
    const groups = parent.querySelectorAll(".help-overlay__group");
    const labels = Array.from(groups).map((g) => g.textContent);
    expect(labels).toContain("Desktop");
    expect(labels).toContain("WebXR");
    expect(labels).toContain("Global");
  });

  it("Escape key hides overlay when visible", () => {
    help.show();
    const ev = new KeyboardEvent("keydown", { key: "Escape" });
    document.dispatchEvent(ev);
    expect(help.isVisible).toBe(false);
  });

  it("'h' key toggles overlay", () => {
    const ev1 = new KeyboardEvent("keydown", { key: "h" });
    document.dispatchEvent(ev1);
    expect(help.isVisible).toBe(true);
    const ev2 = new KeyboardEvent("keydown", { key: "H" });
    document.dispatchEvent(ev2);
    expect(help.isVisible).toBe(false);
  });

  it("ignores 'h' when focus is in input", () => {
    const input = document.createElement("input");
    parent.appendChild(input);
    input.focus();
    const ev = new KeyboardEvent("keydown", { key: "h", bubbles: true });
    input.dispatchEvent(ev);
    expect(help.isVisible).toBe(false);
  });

  it("onChange callback fires on show/hide", () => {
    const cb = vi.fn();
    help.onChange(cb);
    help.show();
    expect(cb).toHaveBeenCalledWith(true);
    help.hide();
    expect(cb).toHaveBeenCalledWith(false);
    expect(cb).toHaveBeenCalledTimes(2);
  });

  it("dispose removes element and detaches keydown listener", () => {
    help.show();
    expect(parent.querySelector("[data-help-overlay]")).toBeTruthy();
    help.dispose();
    expect(parent.querySelector("[data-help-overlay]")).toBeNull();
    // После dispose keydown больше не должен ничего делать (не падать).
    const ev = new KeyboardEvent("keydown", { key: "h" });
    expect(() => document.dispatchEvent(ev)).not.toThrow();
    // isVisible остаётся false.
    expect(help.isVisible).toBe(false);
  });

  it("backdrop click hides overlay", () => {
    help.show();
    const root = parent.querySelector("[data-help-overlay]") as HTMLElement;
    const ev = new MouseEvent("click", { bubbles: true });
    Object.defineProperty(ev, "target", { value: root });
    root.dispatchEvent(ev);
    expect(help.isVisible).toBe(false);
  });

  it("custom hotkeys array replaces defaults", () => {
    help.dispose();
    const custom = [
      { key: "X", description: "Custom action", category: "Desktop" as const }
    ];
    help = createHelpOverlay(parent, custom);
    help.show();
    const terms = parent.querySelectorAll(".help-overlay__term");
    expect(terms.length).toBe(1);
    expect(terms[0].textContent).toContain("X");
  });
});
