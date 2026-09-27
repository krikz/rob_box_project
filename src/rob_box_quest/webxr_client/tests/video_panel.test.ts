// Тесты тракта заливки видеокадров (issue #2144 — мигание левого глаза).
//
// Что здесь проверяется и почему именно это:
//   * кадр доезжает до текстуры как ImageBitmap, а не через 2D-canvas —
//     это и есть фикс (старый путь делал `new Image()` + `drawImage` +
//     `texImage2D` из HTMLCanvasElement на каждый кадр внутри
//     `renderer.render()`);
//   * предыдущий битмап закрывается после замены — иначе течёт GPU-память;
//   * drop-oldest переехал с `img.complete` на флаг декодирования;
//   * без `createImageBitmap` код не падает, а уходит на <img>-фолбэк.
//
// jsdom не даёт ни WebGL, ни настоящего декодера картинок, поэтому
// `createImageBitmap` подменяем управляемым deferred'ом: так виден и
// момент «декод ещё идёт» (drop-oldest), и момент «кадр готов».

import { describe, it, expect, beforeAll, beforeEach, afterEach, vi } from "vitest";
import * as THREE from "three";
import { VideoPanel } from "../src/scene/video_panel";
import type { PanelState } from "../src/scene/panel_manager";

const STATE: PanelState = {
  id: "main_screen",
  topic: "/oak/rgb/image_raw/compressed",
  position: { x: 0, y: 1.5, z: -3.9 },
  facing: { x: 0, z: 1 },
  size: { width: 4.8, height: 2.7 },
  selected: false
};

/** Заглушка ImageBitmap: важны width/height и close() (утечка/не утечка). */
interface FakeBitmap {
  width: number;
  height: number;
  closed: boolean;
  close(): void;
}

function makeBitmap(width = 64, height = 36): FakeBitmap {
  const bmp: FakeBitmap = {
    width,
    height,
    closed: false,
    close() {
      bmp.closed = true;
    }
  };
  return bmp;
}

type Deferred = {
  resolve: (bmp: FakeBitmap) => void;
  reject: (err: unknown) => void;
};

/** Очередь незавершённых декодов — по одному на вызов createImageBitmap. */
let pending: Deferred[] = [];
let createBitmapSpy: ReturnType<typeof vi.fn>;
let lastOptions: ImageBitmapOptions | undefined;

const g = globalThis as unknown as {
  createImageBitmap?: unknown;
  URL: typeof URL;
};

const JPEG = new Uint8Array([0xff, 0xd8, 0xff, 0xdb, 0x00, 0x01]);

/** Прокрутить микрозадачи: decode → .then → presentFrame. */
async function flush(): Promise<void> {
  for (let i = 0; i < 4; i += 1) await Promise.resolve();
}

function textureOf(panel: VideoPanel): THREE.Texture {
  const mat = panel.mesh.material as THREE.MeshBasicMaterial;
  const map = mat.map;
  if (!map) throw new Error("panel material has no map");
  return map;
}

let savedCreateImageBitmap: unknown;

/**
 * jsdom не реализует 2D-контекст — ставим заглушку глобально (как в
 * tars1_text_panel.test.ts / tars2_metrics_panel.test.ts). Заглушка нужна
 * панелям с подписью; быстрый путь `getContext` вообще не зовёт, и
 * наличие заглушки ему не мешает — зато на старой реализации тесты
 * падают на содержательной проверке, а не на брошенном конструкторе.
 */
const drawImage = vi.fn();

beforeAll(() => {
  const stubCtx = {
    fillStyle: "",
    font: "",
    textBaseline: "",
    fillRect: () => {},
    fillText: () => {},
    clearRect: () => {},
    measureText: (text: string) => ({ width: text.length * 7 }),
    drawImage
  } as unknown as CanvasRenderingContext2D;
  HTMLCanvasElement.prototype.getContext = function () {
    return stubCtx;
  } as unknown as typeof HTMLCanvasElement.prototype.getContext;
});

beforeEach(() => {
  pending = [];
  lastOptions = undefined;
  drawImage.mockClear();
  savedCreateImageBitmap = g.createImageBitmap;
  createBitmapSpy = vi.fn((_blob: Blob, options?: ImageBitmapOptions) => {
    lastOptions = options;
    return new Promise<FakeBitmap>((resolve, reject) => {
      pending.push({ resolve, reject });
    });
  });
  g.createImageBitmap = createBitmapSpy;
});

afterEach(() => {
  g.createImageBitmap = savedCreateImageBitmap;
  vi.restoreAllMocks();
});

describe("VideoPanel.ingestJpeg — быстрый путь (showLabel: false)", () => {
  let panel: VideoPanel;

  beforeEach(() => {
    panel = new VideoPanel(STATE, { showLabel: false, canvasWidth: 1280, canvasHeight: 720 });
  });

  afterEach(() => {
    panel.dispose();
  });

  it("до первого кадра в текстуре лежит заглушка, а не большой canvas", () => {
    const image = textureOf(panel).image as HTMLCanvasElement;
    expect(image).toBeInstanceOf(HTMLCanvasElement);
    // Быстрый путь не рисует в canvas — держать 1280×720 незачем.
    expect(image.width).toBe(2);
    expect(image.height).toBe(2);
  });

  it("декодирует через createImageBitmap, а не через <img> + object URL", () => {
    const createObjectURL = vi.fn(() => "blob:stub");
    g.URL.createObjectURL = createObjectURL as unknown as typeof URL.createObjectURL;

    expect(panel.ingestJpeg(JPEG)).toBe(true);

    expect(createBitmapSpy).toHaveBeenCalledTimes(1);
    expect(createObjectURL).not.toHaveBeenCalled();
    // Переворот заказан у декодера: three.js игнорирует flipY для ImageBitmap.
    expect(lastOptions).toEqual({ imageOrientation: "flipY" });
  });

  it("заливает непустой ImageBitmap прямо в текстуру (без 2D-canvas)", async () => {
    const texture = textureOf(panel);
    const versionBefore = texture.version;

    panel.ingestJpeg(JPEG);
    const bitmap = makeBitmap(1920, 1080);
    pending[0].resolve(bitmap);
    await flush();

    expect(texture.image).toBe(bitmap);
    expect((texture.image as FakeBitmap).width).toBeGreaterThan(0);
    expect((texture.image as FakeBitmap).height).toBeGreaterThan(0);
    // needsUpdate у THREE.Texture — сеттер, читаем эффект через version.
    expect(texture.version).toBeGreaterThan(versionBefore);
    // flipY выключен: битмап уже перевёрнут декодером.
    expect(texture.flipY).toBe(false);
  });

  it("закрывает предыдущий ImageBitmap после замены (нет утечки)", async () => {
    panel.ingestJpeg(JPEG);
    const first = makeBitmap();
    pending[0].resolve(first);
    await flush();

    panel.ingestJpeg(JPEG);
    const second = makeBitmap();
    pending[1].resolve(second);
    await flush();

    expect(first.closed).toBe(true);
    // Текущий кадр закрывать нельзя — он в текстуре.
    expect(second.closed).toBe(false);
    expect(textureOf(panel).image).toBe(second);
  });

  it("dispose закрывает последний битмап", async () => {
    panel.ingestJpeg(JPEG);
    const bitmap = makeBitmap();
    pending[0].resolve(bitmap);
    await flush();

    panel.dispose();
    expect(bitmap.closed).toBe(true);
  });

  it("кадр, доехавший после dispose, закрывается и не трогает текстуру", async () => {
    panel.ingestJpeg(JPEG);
    panel.dispose();
    const bitmap = makeBitmap();
    pending[0].resolve(bitmap);
    await flush();

    expect(bitmap.closed).toBe(true);
    expect(textureOf(panel).image).not.toBe(bitmap);
  });
});

describe("VideoPanel.ingestJpeg — drop-oldest и статистика", () => {
  let panel: VideoPanel;

  beforeEach(() => {
    panel = new VideoPanel(STATE, { showLabel: false });
  });

  afterEach(() => {
    panel.dispose();
  });

  it("пока предыдущий кадр декодируется — новый дропается", async () => {
    expect(panel.ingestJpeg(JPEG)).toBe(true);
    expect(panel.ingestJpeg(JPEG)).toBe(false);
    expect(panel.ingestJpeg(JPEG)).toBe(false);

    expect(panel.getStats()).toEqual({ frameCount: 3, droppedCount: 2 });
    expect(createBitmapSpy).toHaveBeenCalledTimes(1);

    // Декод завершился — следующий кадр снова принимается.
    pending[0].resolve(makeBitmap());
    await flush();
    expect(panel.ingestJpeg(JPEG)).toBe(true);
    expect(panel.getStats()).toEqual({ frameCount: 4, droppedCount: 2 });
  });

  it("провал декодирования считается дропом и не блокирует поток", async () => {
    panel.ingestJpeg(JPEG);
    pending[0].reject(new Error("broken jpeg"));
    await flush();

    expect(panel.getStats()).toEqual({ frameCount: 1, droppedCount: 1 });
    expect(panel.ingestJpeg(JPEG)).toBe(true);
  });
});

describe("VideoPanel.ingestJpeg — панель с подписью (showLabel: true)", () => {
  let panel: VideoPanel;

  beforeEach(() => {
    panel = new VideoPanel(STATE, { showLabel: true, canvasWidth: 640, canvasHeight: 360 });
  });

  afterEach(() => {
    panel.dispose();
  });

  it("битмап блитуется в canvas, закрывается, текстура остаётся canvas-овой", async () => {
    const texture = textureOf(panel);
    // Битмап под метку декодируется без переворота — иначе canvas-путь
    // (flipY = true) показал бы кадр вверх ногами.
    panel.ingestJpeg(JPEG);
    expect(lastOptions).toEqual({ imageOrientation: "none" });

    const bitmap = makeBitmap();
    pending[0].resolve(bitmap);
    await flush();

    expect(drawImage).toHaveBeenCalledWith(bitmap, 0, 0, 640, 360);
    expect(bitmap.closed).toBe(true);
    expect(texture.image).toBeInstanceOf(HTMLCanvasElement);
    expect((texture.image as HTMLCanvasElement).width).toBe(640);
  });

  it("setLabel по-прежнему рисует подпись", () => {
    const texture = textureOf(panel);
    const versionBefore = texture.version;
    panel.setLabel("/ceiling/image_raw/compressed");
    expect(texture.version).toBeGreaterThan(versionBefore);
  });
});

describe("VideoPanel.ingestJpeg — фолбэк без createImageBitmap", () => {
  it("не падает и уходит на <img> + object URL", () => {
    delete g.createImageBitmap;
    const createObjectURL = vi.fn(() => "blob:fallback");
    const revokeObjectURL = vi.fn();
    g.URL.createObjectURL = createObjectURL as unknown as typeof URL.createObjectURL;
    g.URL.revokeObjectURL = revokeObjectURL as unknown as typeof URL.revokeObjectURL;

    const panel = new VideoPanel(STATE, { showLabel: false });
    expect(() => panel.ingestJpeg(JPEG)).not.toThrow();
    expect(panel.getStats()).toEqual({ frameCount: 1, droppedCount: 0 });
    expect(createObjectURL).toHaveBeenCalledTimes(1);
    panel.dispose();
  });

  it("не падает, если createImageBitmap бросает синхронно", () => {
    g.createImageBitmap = vi.fn(() => {
      throw new TypeError("options not supported");
    });
    const createObjectURL = vi.fn(() => "blob:fallback");
    g.URL.createObjectURL = createObjectURL as unknown as typeof URL.createObjectURL;

    const panel = new VideoPanel(STATE, { showLabel: false });
    expect(() => panel.ingestJpeg(JPEG)).not.toThrow();
    expect(createObjectURL).toHaveBeenCalledTimes(1);
    panel.dispose();
  });
});
