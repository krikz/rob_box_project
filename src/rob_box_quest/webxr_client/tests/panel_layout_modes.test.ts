// Unit-тесты для panel_layout_modes: чистая логика layout (Canvas API не нужен,
// потому что CompositeDrawer создаёт canvas через document.createElement —
// jsdom это поддерживает).

import { describe, it, expect, beforeEach } from "vitest";
import {
  getSlots,
  maxSlotsForMode,
  cycleLayout,
  CompositeDrawer,
  type LayoutMode
} from "../src/scene/panel_layout_modes";

// jsdom не имеет canvas npm-пакета — предоставим заглушку 2D-контекста.
class FakeCanvasContext {
  fillStyle = "#000000";
  font = "10px sans-serif";
  fillRectCalls = 0;
  fillTextCalls = 0;
  drawImageCalls = 0;
  fillRect(_x: number, _y: number, _w: number, _h: number): void {
    this.fillRectCalls += 1;
  }
  fillText(_text: string, _x: number, _y: number): void {
    this.fillTextCalls += 1;
  }
  drawImage(
    _img: HTMLImageElement,
    _sx: number,
    _sy: number,
    _sw: number,
    _sh: number,
    _dx: number,
    _dy: number,
    _dw: number,
    _dh: number
  ): void {
    this.drawImageCalls += 1;
  }
}

beforeEach(() => {
  HTMLCanvasElement.prototype.getContext = ((): FakeCanvasContext | null =>
    new FakeCanvasContext()) as unknown as typeof HTMLCanvasElement.prototype.getContext;
  // jsdom не реализует URL.createObjectURL/revokeObjectURL — заглушка.
  if (typeof URL.createObjectURL !== "function") {
    URL.createObjectURL = (): string => "blob:fake";
  }
  if (typeof URL.revokeObjectURL !== "function") {
    URL.revokeObjectURL = (): void => undefined;
  }
});

describe("panel_layout_modes: getSlots / maxSlotsForMode", () => {
  it("single → 1 slot, full area", () => {
    expect(maxSlotsForMode("single")).toBe(1);
    expect(getSlots("single")).toEqual([{ x: 0, y: 0, width: 1, height: 1 }]);
  });

  it("split-2h → 2 slots, horizontal halves", () => {
    expect(maxSlotsForMode("split-2h")).toBe(2);
    expect(getSlots("split-2h")).toEqual([
      { x: 0, y: 0, width: 0.5, height: 1 },
      { x: 0.5, y: 0, width: 0.5, height: 1 }
    ]);
  });

  it("split-2v → 2 slots, vertical halves", () => {
    expect(maxSlotsForMode("split-2v")).toBe(2);
    expect(getSlots("split-2v")).toEqual([
      { x: 0, y: 0, width: 1, height: 0.5 },
      { x: 0, y: 0.5, width: 1, height: 0.5 }
    ]);
  });

  it("2x2 → 4 slots in grid", () => {
    expect(maxSlotsForMode("2x2")).toBe(4);
    expect(getSlots("2x2")).toHaveLength(4);
    expect(getSlots("2x2")[2]).toEqual({ x: 0, y: 0.5, width: 0.5, height: 0.5 });
  });

  it("pip → 1 big + 1 small bottom-right", () => {
    expect(maxSlotsForMode("pip")).toBe(2);
    const slots = getSlots("pip");
    expect(slots[1].x).toBeGreaterThan(0.5);
    expect(slots[1].y).toBeGreaterThan(0.5);
    expect(slots[1].width).toBeLessThan(0.5);
  });
});

describe("panel_layout_modes: cycleLayout", () => {
  it("cycles single → split-2h → split-2v → 2x2 → pip → single", () => {
    expect(cycleLayout("single")).toBe("split-2h");
    expect(cycleLayout("split-2h")).toBe("split-2v");
    expect(cycleLayout("split-2v")).toBe("2x2");
    expect(cycleLayout("2x2")).toBe("pip");
    expect(cycleLayout("pip")).toBe("single");
  });

  it("skips excluded modes", () => {
    expect(cycleLayout("single", ["split-2h"])).toBe("split-2v");
    expect(cycleLayout("split-2v", ["single", "2x2"])).toBe("pip");
  });

  it("returns first of filtered if current was excluded", () => {
    const result = cycleLayout("single", ["single"]);
    expect(result).toBe("split-2h");
  });
});

describe("CompositeDrawer", () => {
  function makeJpegBytes(width: number, height: number, color: [number, number, number]): Uint8Array {
    // Минимальный валидный JPEG: SOI + APP0 + SOF0 + SOS + EOI, чтобы Image-onload
    // сработал предсказуемо в jsdom (jsdom не делает реальный декод JPEG, но
    // для теста нам достаточно проверить, что drawImage вызывается).
    // Возвращаем массив с правильным JPEG-magic, но тесту не нужен реальный
    // декод — мы тестируем контракт, не image decoder.
    const bytes = new Uint8Array([
      0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 0x4a, 0x46, 0x49, 0x46, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01,
      0x00, 0x01, 0x00, 0x00, 0xff, 0xdb, 0x00, 0x43, 0x00,
      // Заглушка-квантизация
      ...new Array(64).fill(0x10),
      0xff, 0xc0, 0x00, 0x0b, 0x08,
      // SOF0: height/width
      (height >> 8) & 0xff, height & 0xff, (width >> 8) & 0xff, width & 0xff,
      0x01, 0x01, 0x11, 0x00,
      0xff, 0xc4, 0x00, 0x1f, 0x00,
      ...new Array(31).fill(0x00),
      0xff, 0xda, 0x00, 0x08, 0x01, 0x01, 0x00, 0x00, 0x3f, 0x00,
      0xfe, 0xfe, 0xfe, 0xfe, // payload (dummy)
      0xff, 0xd9
    ]);
    void color;
    void width;
    void height;
    return bytes;
  }

  it("constructor: creates canvas and starts with single mode", () => {
    const cd = new CompositeDrawer(640, 360);
    expect(cd.getCanvas().width).toBe(640);
    expect(cd.getCanvas().height).toBe(360);
    expect(cd.getMode()).toBe("single");
    expect(cd.getTopics()).toEqual([]);
  });

  it("setLayout: switches mode, slices topics to max slots, clears stale", () => {
    const cd = new CompositeDrawer(640, 360);
    cd.setLayout("split-2h", ["camera_rear", "camera_oak_color", "lidar_2d", "lidar_3d"]);
    expect(cd.getMode()).toBe("split-2h");
    expect(cd.getTopics()).toEqual(["camera_rear", "camera_oak_color"]);
    cd.setLayout("2x2", ["a", "b", "c", "d"]);
    expect(cd.getTopics()).toEqual(["a", "b", "c", "d"]);
    cd.setLayout("single", ["only"]);
    expect(cd.getTopics()).toEqual(["only"]);
  });

  it("setLayout: from 2x2 back to single → drops extras", () => {
    const cd = new CompositeDrawer(640, 360);
    cd.setLayout("2x2", ["a", "b", "c", "d"]);
    cd.setLayout("single", ["only"]);
    expect(cd.getTopics()).toEqual(["only"]);
  });

  it("ingestJpeg on out-of-range slot returns false", () => {
    const cd = new CompositeDrawer(640, 360);
    cd.setLayout("single", ["a"]);
    const jpeg = makeJpegBytes(2, 2, [255, 0, 0]);
    expect(cd.ingestJpeg(5, jpeg)).toBe(false);
  });

  it("clearSlot invalidates image", () => {
    const cd = new CompositeDrawer(640, 360);
    cd.setLayout("single", ["a"]);
    cd.clearSlot(0);
    const stats = cd.getStats();
    expect(stats.frameCount).toBe(0);
    expect(stats.droppedCount).toBe(0);
  });

  it("composite callback fires after setLayout", () => {
    const cd = new CompositeDrawer(640, 360);
    let compositeCalls = 0;
    cd.setOnCompositeUpdated(() => (compositeCalls += 1));
    cd.setLayout("split-2h", ["a", "b"]);
    expect(compositeCalls).toBe(1);
    cd.setLayout("2x2", ["a", "b", "c", "d"]);
    expect(compositeCalls).toBe(2);
  });

  it("setLayout triggers redraw (composite callback)", () => {
    const cd = new CompositeDrawer(640, 360);
    let compositeCalls = 0;
    cd.setOnCompositeUpdated(() => (compositeCalls += 1));
    cd.setLayout("2x2", ["a", "b", "c", "d"]);
    cd.setLayout("pip", ["a", "b"]);
    expect(compositeCalls).toBe(2);
  });

  it("clearSlot then setLayout triggers redraw", () => {
    const cd = new CompositeDrawer(640, 360);
    let compositeCalls = 0;
    cd.setOnCompositeUpdated(() => (compositeCalls += 1));
    cd.setLayout("2x2", ["a", "b", "c", "d"]);
    const baseline = compositeCalls;
    cd.clearSlot(2);
    expect(compositeCalls).toBe(baseline + 1);
  });

  it("regression: ingestJpeg drop-oldest when previous not decoded", () => {
    const cd = new CompositeDrawer(640, 360);
    cd.setLayout("single", ["a"]);
    // В jsdom Image.complete всегда false после src=, пока не сработает onload.
    // Дёргаем дважды подряд → первый должен приняться, второй отброшен.
    const jpeg = makeJpegBytes(2, 2, [255, 0, 0]);
    expect(cd.ingestJpeg(0, jpeg)).toBe(true);
    expect(cd.ingestJpeg(0, jpeg)).toBe(false);
    expect(cd.getStats().droppedCount).toBe(1);
  });

  it("regression: cycleLayout never returns excluded mode", () => {
    const all: LayoutMode[] = ["single", "split-2h", "split-2v", "2x2", "pip"];
    for (const start of all) {
      let cur = start;
      for (let i = 0; i < all.length; i += 1) {
        cur = cycleLayout(cur, ["pip"]);
        expect(cur).not.toBe("pip");
      }
    }
  });
});
