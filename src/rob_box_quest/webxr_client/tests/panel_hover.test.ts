import { describe, expect, it } from "vitest";
import { createPanelHover } from "../src/scene/panel_hover";

describe("createPanelHover", () => {
  it("starts with no hovered panel", () => {
    const h = createPanelHover();
    expect(h.getHovered()).toBe(null);
  });

  it("setHovered(null) when already null — no events", () => {
    const entered: string[] = [];
    const exited: string[] = [];
    const h = createPanelHover({
      subscriber: {
        onHoverEnter: (id) => entered.push(id),
        onHoverExit: (id) => exited.push(id)
      }
    });
    expect(h.setHovered(null)).toEqual([]);
    expect(entered).toEqual([]);
    expect(exited).toEqual([]);
  });

  it("setHovered(A) from null → enter A", () => {
    const entered: string[] = [];
    const exited: string[] = [];
    const h = createPanelHover({
      subscriber: {
        onHoverEnter: (id) => entered.push(id),
        onHoverExit: (id) => exited.push(id)
      }
    });
    const changed = h.setHovered("p1");
    expect(changed).toEqual(["p1"]);
    expect(entered).toEqual(["p1"]);
    expect(exited).toEqual([]);
    expect(h.getHovered()).toBe("p1");
  });

  it("setHovered(A) when already A — no-op", () => {
    const entered: string[] = [];
    const h = createPanelHover({
      subscriber: { onHoverEnter: (id) => entered.push(id) }
    });
    h.setHovered("p1");
    expect(h.setHovered("p1")).toEqual([]);
    expect(entered).toEqual(["p1"]); // только один раз
  });

  it("setHovered(null) from A → exit A", () => {
    const exited: string[] = [];
    const h = createPanelHover({
      subscriber: { onHoverExit: (id) => exited.push(id) }
    });
    h.setHovered("p1");
    const changed = h.setHovered(null);
    expect(changed).toEqual(["p1"]);
    expect(exited).toEqual(["p1"]);
    expect(h.getHovered()).toBe(null);
  });

  it("setHovered(B) from A → exit A + enter B", () => {
    const events: string[] = [];
    const h = createPanelHover({
      subscriber: {
        onHoverEnter: (id) => events.push(`enter:${id}`),
        onHoverExit: (id) => events.push(`exit:${id}`)
      }
    });
    h.setHovered("p1");
    h.setHovered("p2");
    expect(events).toEqual(["enter:p1", "exit:p1", "enter:p2"]);
    expect(h.getHovered()).toBe("p2");
  });

  it("subscribe adds additional listener", () => {
    const entered: string[] = [];
    const h = createPanelHover();
    h.subscribe({ onHoverEnter: (id) => entered.push(id) });
    h.setHovered("p1");
    expect(entered).toEqual(["p1"]);
  });

  it("subscribe returns unsubscriber", () => {
    const entered: string[] = [];
    const h = createPanelHover();
    const unsub = h.subscribe({ onHoverEnter: (id) => entered.push(id) });
    h.setHovered("p1");
    unsub();
    h.setHovered("p2");
    expect(entered).toEqual(["p1"]);
  });

  it("reset() exits current and clears", () => {
    const exited: string[] = [];
    const h = createPanelHover({
      subscriber: { onHoverExit: (id) => exited.push(id) }
    });
    h.setHovered("p1");
    h.reset();
    expect(exited).toEqual(["p1"]);
    expect(h.getHovered()).toBe(null);
  });

  it("reset() with no hover — no-op", () => {
    const exited: string[] = [];
    const h = createPanelHover({
      subscriber: { onHoverExit: (id) => exited.push(id) }
    });
    h.reset();
    expect(exited).toEqual([]);
  });

  it("listener throw does not break subsequent emit", () => {
    const seen: string[] = [];
    const h = createPanelHover({
      subscriber: {
        onHoverEnter(_id) {
          throw new Error("boom");
        }
      }
    });
    h.subscribe({ onHoverEnter: (id) => seen.push(id) });
    h.setHovered("p1");
    expect(seen).toEqual(["p1"]);
  });
});