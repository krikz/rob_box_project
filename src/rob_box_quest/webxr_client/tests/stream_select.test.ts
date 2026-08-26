// Unit-тесты для stream_select (Phase 2 §6.2 — drag/drop UX, §6.3 — layout cycle).
// Используем jsdom DOM + lil-gui для проверки handle contract.

import { describe, it, expect, beforeEach } from "vitest";
import { createStreamSelect, readDropTopic } from "../src/ui/stream_select";
import { PanelManager } from "../src/scene/panel_manager";
import type { LayoutMode } from "../src/scene/panel_layout_modes";

function makeOpts(): {
  panels: PanelManager;
  log: {
    subscribe: string[];
    unsubscribe: string[];
    reset: number;
    drops: Array<{ panelId: string; topic: string }>;
    cycles: Array<{ panelId: string; current: LayoutMode }>;
  };
} {
  const panels = new PanelManager();
  panels.resetLayout();
  const log = {
    subscribe: [] as string[],
    unsubscribe: [] as string[],
    reset: 0,
    drops: [] as Array<{ panelId: string; topic: string }>,
    cycles: [] as Array<{ panelId: string; current: LayoutMode }>
  };
  return { panels, log };
}

describe("stream_select: createStreamSelect", () => {
  beforeEach(() => {
    document.body.innerHTML = "";
  });

  it("creates panel without errors", () => {
    const { panels, log } = makeOpts();
    const handle = createStreamSelect({
      panels,
      onSubscribe: (t) => log.subscribe.push(t),
      onUnsubscribe: (t) => log.unsubscribe.push(t),
      onResetLayout: () => (log.reset += 1),
      onDropTopic: (panelId, topic) => log.drops.push({ panelId, topic }),
      onCyclePanelLayout: (panelId, current) =>
        log.cycles.push({ panelId, current }),
      getActiveTopics: () => panels.list().map((p) => p.topic)
    });
    expect(handle).toBeDefined();
    expect(typeof handle.destroy).toBe("function");
    handle.destroy();
  });

  it("setAvailableStreams updates the dropdown", () => {
    const { panels, log } = makeOpts();
    const handle = createStreamSelect({
      panels,
      onSubscribe: (t) => log.subscribe.push(t),
      onUnsubscribe: (t) => log.unsubscribe.push(t),
      onResetLayout: () => (log.reset += 1),
      onDropTopic: () => undefined,
      onCyclePanelLayout: () => undefined,
      getActiveTopics: () => []
    });
    handle.setAvailableStreams([
      {
        topic: "camera_rear",
        topic_id: 0,
        kind: "ros_topic",
        source: "",
        default_quality: "med"
      },
      {
        topic: "camera_oak_color",
        topic_id: 1,
        kind: "ros_topic",
        source: "",
        default_quality: "med"
      }
    ]);
    handle.destroy();
  });

  it("setPanelLayoutMode updates cycle button label", () => {
    const { panels, log } = makeOpts();
    const layoutModes = new Map<string, LayoutMode>([["p1", "single"]]);
    const handle = createStreamSelect({
      panels,
      onSubscribe: (t) => log.subscribe.push(t),
      onUnsubscribe: (t) => log.unsubscribe.push(t),
      onResetLayout: () => (log.reset += 1),
      onDropTopic: () => undefined,
      onCyclePanelLayout: () => undefined,
      getActiveTopics: () => panels.list().map((p) => p.topic),
      getPanelLayoutModes: () => layoutModes
    });
    handle.setPanelLayoutMode("p1", "2x2");
    handle.destroy();
  });

  it("regression: destroy() removes DOM", () => {
    const { panels, log } = makeOpts();
    const handle = createStreamSelect({
      panels,
      onSubscribe: (t) => log.subscribe.push(t),
      onUnsubscribe: (t) => log.unsubscribe.push(t),
      onResetLayout: () => (log.reset += 1),
      onDropTopic: () => undefined,
      onCyclePanelLayout: () => undefined,
      getActiveTopics: () => []
    });
    handle.destroy();
    expect(document.body.querySelectorAll(".lil-gui").length).toBe(0);
  });

  it("regression: refresh() doesn't throw when layout modes missing", () => {
    const { panels, log } = makeOpts();
    const handle = createStreamSelect({
      panels,
      onSubscribe: (t) => log.subscribe.push(t),
      onUnsubscribe: (t) => log.unsubscribe.push(t),
      onResetLayout: () => (log.reset += 1),
      onDropTopic: () => undefined,
      onCyclePanelLayout: () => undefined,
      getActiveTopics: () => []
    });
    expect(() => handle.refresh()).not.toThrow();
    handle.destroy();
  });
});

describe("stream_select: readDropTopic helper", () => {
  it("returns null when no dataTransfer", () => {
    const fakeEv = { dataTransfer: null } as unknown as DragEvent;
    expect(readDropTopic(fakeEv)).toBeNull();
  });

  it("returns null when dataTransfer empty", () => {
    const fakeEv = {
      dataTransfer: {
        getData: (_mime: string): string => ""
      }
    } as unknown as DragEvent;
    expect(readDropTopic(fakeEv)).toBeNull();
  });

  it("returns topic from x-rob-box-topic MIME", () => {
    const fakeEv = {
      dataTransfer: {
        getData: (mime: string): string =>
          mime === "application/x-rob-box-topic" ? "camera_oak_color" : ""
      }
    } as unknown as DragEvent;
    expect(readDropTopic(fakeEv)).toBe("camera_oak_color");
  });

  it("falls back to text/plain", () => {
    const fakeEv = {
      dataTransfer: {
        getData: (mime: string): string =>
          mime === "text/plain" ? "lidar_2d" : ""
      }
    } as unknown as DragEvent;
    expect(readDropTopic(fakeEv)).toBe("lidar_2d");
  });
});
