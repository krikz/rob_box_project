// panel_layout_modes — компоновщик multi-stream panel через Canvas 2D.
//
// Phase 2 §6.3: Layout modes для одной панели — single / split-2h / split-2v / 2x2 / pip.
// Видео рисуется на одном 2D canvas, который превращается в THREE.CanvasTexture.
//
// Реализация: композитный Canvas держит несколько sub-canvas (ImageBitmap-like),
// каждый из которых обновляется независимо от входящих JPEG. При каждом
// update одного из sub-canvas делается композитная отрисовка всего layout.

export type LayoutMode = "single" | "split-2h" | "split-2v" | "2x2" | "pip";

/**
 * Координаты sub-region в пикселях canvas.
 * - x,y — левый-верхний угол
 * - width, height — размеры
 */
export interface SubRegion {
  x: number;
  y: number;
  width: number;
  height: number;
}

/**
 * Координаты slot'ов для layout mode.
 * Каждый slot описывает, где на canvas должна быть отрисовка одного стрима.
 */
export type SlotMap = SubRegion[];

/**
 * Возвращает slot map для layout mode и числа активных тем (1..4).
 * Если активных тем меньше, чем слотов — лишние слоты игнорируются.
 * Если больше — лишние темы игнорируются (композитный canvas переполняется).
 */
export function getSlots(mode: LayoutMode): SlotMap {
  switch (mode) {
    case "single":
      return [{ x: 0, y: 0, width: 1, height: 1 }];
    case "split-2h":
      // 2 потока горизонтально: левая и правая половины.
      return [
        { x: 0, y: 0, width: 0.5, height: 1 },
        { x: 0.5, y: 0, width: 0.5, height: 1 }
      ];
    case "split-2v":
      // 2 потока вертикально: верхняя и нижняя половины.
      return [
        { x: 0, y: 0, width: 1, height: 0.5 },
        { x: 0, y: 0.5, width: 1, height: 0.5 }
      ];
    case "2x2":
      // 4 потока сеткой 2×2.
      return [
        { x: 0, y: 0, width: 0.5, height: 0.5 },
        { x: 0.5, y: 0, width: 0.5, height: 0.5 },
        { x: 0, y: 0.5, width: 0.5, height: 0.5 },
        { x: 0.5, y: 0.5, width: 0.5, height: 0.5 }
      ];
    case "pip":
      // 1 большой + 1 маленький в правом нижнем углу.
      return [
        { x: 0, y: 0, width: 1, height: 1 },
        { x: 0.7, y: 0.7, width: 0.28, height: 0.28 }
      ];
  }
}

/**
 * Максимальное число тем (слотов) для каждого mode.
 */
export function maxSlotsForMode(mode: LayoutMode): number {
  return getSlots(mode).length;
}

/**
 * Следующий layout в цикле: single → split-2h → split-2v → 2x2 → pip → single.
 * Если `exclude` не пуст, пропускает указанный mode.
 */
export function cycleLayout(current: LayoutMode, exclude: LayoutMode[] = []): LayoutMode {
  const order: LayoutMode[] = ["single", "split-2h", "split-2v", "2x2", "pip"];
  const filtered = exclude.length > 0 ? order.filter((m) => !exclude.includes(m)) : order;
  const idx = filtered.indexOf(current);
  if (idx === -1) return filtered[0];
  return filtered[(idx + 1) % filtered.length];
}

/**
 * Композитный drawer — управляет набором sub-image для каждого slot.
 *
 * Использование:
 *   const cd = new CompositeDrawer(640, 360);
 *   cd.setLayout("split-2h", ["camera_rear", "camera_oak_color"]);
 *   cd.updateSlot(0, jpegBytes);  // → перерисовывает композит
 *   cd.getCanvas() → HTMLCanvasElement
 *
 * Slot → topic маппинг:
 *   - slot 0 ↔ topic[0], slot 1 ↔ topic[1], ...
 *   - Если темы переставлены (setLayout вызвали снова), старые slot'ы очищаются.
 */
export class CompositeDrawer {
  private readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private mode: LayoutMode = "single";
  private topics: string[] = [];
  /** slot → текущая картинка (последняя успешно декодированная). */
  private images: (HTMLImageElement | null)[] = [];
  /** slot → ожидающая декодирования картинка (для drop-oldest). */
  private pending: (HTMLImageElement | null)[] = [];
  private frameCount = 0;
  private droppedCount = 0;
  /** callback когда композит реально перерисован (для THREE.CanvasTexture.needsUpdate). */
  private onCompositeUpdated: (() => void) | null = null;

  constructor(width: number, height: number) {
    this.canvas = document.createElement("canvas");
    this.canvas.width = width;
    this.canvas.height = height;
    const ctx = this.canvas.getContext("2d", { alpha: false });
    if (!ctx) {
      throw new Error("CompositeDrawer: failed to acquire 2D context");
    }
    this.ctx = ctx;
    this.ctx.fillStyle = "#000";
    this.ctx.fillRect(0, 0, width, height);
  }

  getCanvas(): HTMLCanvasElement {
    return this.canvas;
  }

  getMode(): LayoutMode {
    return this.mode;
  }

  getTopics(): string[] {
    return [...this.topics];
  }

  getStats(): { frameCount: number; droppedCount: number } {
    return { frameCount: this.frameCount, droppedCount: this.droppedCount };
  }

  setOnCompositeUpdated(cb: (() => void) | null): void {
    this.onCompositeUpdated = cb;
  }

  /**
   * Сменить layout и список тем. Если число тем больше слотов — лишние игнорируются.
   * Если меньше — оставшиеся слоты очищаются.
   */
  setLayout(mode: LayoutMode, topics: string[]): void {
    this.mode = mode;
    const max = maxSlotsForMode(mode);
    this.topics = topics.slice(0, max);
    // Изменяем размер image-массивов под новое число слотов.
    const oldImages = this.images;
    const oldPending = this.pending;
    this.images = new Array(max).fill(null);
    this.pending = new Array(max).fill(null);
    for (let i = 0; i < max && i < oldImages.length; i += 1) {
      this.images[i] = oldImages[i];
      this.pending[i] = oldPending[i];
    }
    // Очищаем canvas и перерисуем с новым layout.
    this.redrawComposite();
  }

  /**
   * Подставить JPEG в slot. Возвращает false, если кадр дропнут (предыдущий
   * ещё не декодирован). После декодирования — композит автоматически
   * перерисовывается.
   */
  ingestJpeg(slotIndex: number, jpeg: Uint8Array): boolean {
    if (slotIndex < 0 || slotIndex >= this.images.length) return false;
    this.frameCount += 1;
    // Drop-oldest: если ещё не декодирован предыдущий — пропускаем.
    if (this.pending[slotIndex] && !this.pending[slotIndex]!.complete) {
      this.droppedCount += 1;
      return false;
    }
    const img = new Image();
    const blob = new Blob([jpeg as BlobPart], { type: "image/jpeg" });
    img.onload = (): void => {
      this.images[slotIndex] = img;
      this.pending[slotIndex] = null;
      this.redrawComposite();
    };
    img.onerror = (): void => {
      this.droppedCount += 1;
      this.pending[slotIndex] = null;
    };
    this.pending[slotIndex] = img;
    img.src = URL.createObjectURL(blob);
    return true;
  }

  /**
   * Очистить slot (например, когда топик отписан).
   */
  clearSlot(slotIndex: number): void {
    if (slotIndex < 0 || slotIndex >= this.images.length) return;
    this.images[slotIndex] = null;
    this.pending[slotIndex] = null;
    this.redrawComposite();
  }

  /**
   * Полная очистка.
   */
  clear(): void {
    for (let i = 0; i < this.images.length; i += 1) {
      this.images[i] = null;
      this.pending[i] = null;
    }
    this.redrawComposite();
  }

  private redrawComposite(): void {
    const w = this.canvas.width;
    const h = this.canvas.height;
    this.ctx.fillStyle = "#000";
    this.ctx.fillRect(0, 0, w, h);
    const slots = getSlots(this.mode);
    for (let i = 0; i < slots.length; i += 1) {
      const slot = slots[i];
      const px = Math.round(slot.x * w);
      const py = Math.round(slot.y * h);
      const pw = Math.round(slot.width * w);
      const ph = Math.round(slot.height * h);
      const img = this.images[i];
      if (img) {
        // cover-семантика: масштабируем с сохранением пропорций, центрируем.
        this.drawCover(img, px, py, pw, ph);
      } else {
        // Пустой slot: чёрный квадрат с подписью топика (если есть).
        this.ctx.fillStyle = "#000";
        this.ctx.fillRect(px, py, pw, ph);
        if (this.topics[i]) {
          this.ctx.fillStyle = "#666";
          this.ctx.font = "11px monospace";
          this.ctx.fillText(this.topics[i], px + 4, py + 14);
        }
      }
    }
    // Подпись layout mode в углу.
    this.ctx.fillStyle = "rgba(0,0,0,0.5)";
    this.ctx.fillRect(0, h - 18, 110, 18);
    this.ctx.fillStyle = "#fff";
    this.ctx.font = "11px monospace";
    this.ctx.fillText(`layout: ${this.mode}`, 4, h - 5);
    this.onCompositeUpdated?.();
  }

  private drawCover(
    img: HTMLImageElement,
    dx: number,
    dy: number,
    dw: number,
    dh: number
  ): void {
    const ratio = img.width / img.height;
    const dstRatio = dw / dh;
    let sx = 0;
    let sy = 0;
    let sw = img.width;
    let sh = img.height;
    if (ratio > dstRatio) {
      // src шире — режем по ширине.
      sw = Math.round(img.height * dstRatio);
      sx = Math.round((img.width - sw) / 2);
    } else {
      sh = Math.round(img.width / dstRatio);
      sy = Math.round((img.height - sh) / 2);
    }
    this.ctx.drawImage(img, sx, sy, sw, sh, dx, dy, dw, dh);
  }
}
