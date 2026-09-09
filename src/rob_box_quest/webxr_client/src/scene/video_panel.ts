// Video panel — PlaneGeometry + текстура, обновляется из JPEG payload.
//
// Архитектура (issue #2144): JPEG декодируется через `createImageBitmap()`
// вне главного потока, а готовый `ImageBitmap` кладётся в текстуру как
// есть (`texture.image = bitmap`), без промежуточного 2D-canvas.
//
// Почему не как было. Старый путь на каждый входящий кадр делал три
// дорогие вещи на главном потоке:
//   1. `new Image()` + `URL.createObjectURL` — синхронный декод JPEG;
//   2. `ctx.drawImage` — CPU-блит с масштабированием в canvas 1280×720
//      (главный экран) и 960×540 (потолочная камера);
//   3. `CanvasTexture.needsUpdate` → полный `texImage2D` из
//      HTMLCanvasElement, ≈3.7 МБ + ≈2 МБ на кадр, синхронно ВНУТРИ
//      `renderer.render()`.
// XR-кадр не укладывался в дедлайн композитора; three.js в стерео рисует
// оба глаза последовательными вьюпортами в один буфер, и при обрыве один
// глаз оставался с clearColor — оператор видел мигание левого глаза.
// Бисект 2026-09-08 (`docker stop oak-d ceiling-camera` → мерцание
// пропало, камеры вернули) закрепил причину именно за этим трактом.
//
// Механизм — СТОИМОСТЬ заливки, а не «аплоад попал между глазами»: JS
// однопоточный, колбэк WebSocket не может прервать `render()`. Поэтому
// заливку надо было удешевить, а не отложить в XR-цикл.
//
// Canvas остался только там, где поверх видео рисуется подпись
// (`showLabel: true` — панели PanelManager): туда декодированный битмап
// блитуется, как раньше, зато декод всё равно ушёл с главного потока.
// Главный экран и потолочная камера идут быстрым путём (`showLabel:
// false`, см. `captain_bridge.ts`).
//
// Drop-oldest сохранён: пока предыдущий кадр декодируется, новый
// пропускается с инкрементом `droppedCount`.

import * as THREE from "three";
import type { PanelState } from "./panel_manager";

/** Что вернул декодер: быстрый путь — ImageBitmap, фолбэк — <img>. */
type DecodedFrame = ImageBitmap | HTMLImageElement;

/**
 * ImageBitmap отличается от HTMLImageElement наличием `close()`.
 * Проверяем по утке, а не через `instanceof ImageBitmap`: конструктора
 * может не быть (старый браузер, jsdom в тестах), и тогда `instanceof`
 * молча увёл бы нас в неверную ветку.
 */
function asBitmap(frame: DecodedFrame): ImageBitmap | null {
  const maybe = frame as ImageBitmap;
  return typeof maybe.close === "function" ? maybe : null;
}

export interface VideoPanelOptions {
  /** Рисовать debug-подпись topic в углу панели (default true). */
  showLabel?: boolean;
  /**
   * Разрешение внутреннего canvas (default 640×360). Работает только при
   * `showLabel: true` — быстрый путь берёт разрешение прямо из JPEG.
   */
  canvasWidth?: number;
  canvasHeight?: number;
}

export class VideoPanel {
  readonly mesh: THREE.Mesh;
  /** Canvas-композит. null на быстром пути (без подписи). */
  private canvas: HTMLCanvasElement | null = null;
  private ctx: CanvasRenderingContext2D | null = null;
  private texture: THREE.Texture;
  /** Битмап, который сейчас лежит в текстуре (быстрый путь). */
  private currentBitmap: ImageBitmap | null = null;
  /** Кадр в процессе декодирования — основа drop-oldest. */
  private decoding = false;
  private disposed = false;
  private state: PanelState;
  private frameCount = 0;
  private droppedCount = 0;
  private readonly showLabel: boolean;
  private readonly bitmapOptions: ImageBitmapOptions;

  constructor(state: PanelState, opts: VideoPanelOptions = {}) {
    this.state = state;
    this.showLabel = opts.showLabel ?? true;
    const canvasWidth = opts.canvasWidth ?? 640;
    const canvasHeight = opts.canvasHeight ?? 360;
    // three.js игнорирует `texture.flipY` для ImageBitmap (см. доку
    // ImageBitmapLoader), поэтому переворот заказываем у декодера, а
    // сам flipY на быстром пути выключаем — так кадр не перевернётся
    // дважды там, где UNPACK_FLIP_Y всё-таки применяется. Canvas-путь
    // остаётся с обычным flipY = true, и битмап туда нужен неперевёрнутый.
    this.bitmapOptions = { imageOrientation: this.showLabel ? "none" : "flipY" };

    const initial = document.createElement("canvas");
    if (this.showLabel) {
      initial.width = canvasWidth;
      initial.height = canvasHeight;
      const ctx = initial.getContext("2d", { alpha: false });
      if (!ctx) {
        throw new Error("VideoPanel: failed to acquire 2D context");
      }
      this.canvas = initial;
      this.ctx = ctx;
      ctx.fillStyle = "#000";
      ctx.fillRect(0, 0, initial.width, initial.height);
    } else {
      // Заглушка до первого кадра: чёрный 2×2 вместо canvas на 1280×720,
      // который на быстром пути никто не рисует. Держим именно canvas,
      // а не пустую текстуру, — иначе three.js ругается «no image data».
      initial.width = 2;
      initial.height = 2;
    }

    this.texture = new THREE.CanvasTexture(initial);
    this.texture.minFilter = THREE.LinearFilter;
    this.texture.magFilter = THREE.LinearFilter;
    this.texture.colorSpace = THREE.SRGBColorSpace;

    const geom = new THREE.PlaneGeometry(state.size.width, state.size.height);
    const mat = new THREE.MeshBasicMaterial({
      map: this.texture,
      side: THREE.DoubleSide,
      toneMapped: false
    });
    this.mesh = new THREE.Mesh(geom, mat);
    this.applyTransform();
  }

  /** topic текущего стрима панели. */
  get topic(): string {
    return this.state.topic;
  }

  /** Обновить состояние панели (позиция / размер / facing). */
  setState(s: PanelState): void {
    this.state = s;
    this.applyTransform();
    const geom = this.mesh.geometry as THREE.PlaneGeometry;
    geom.dispose();
    const newGeom = new THREE.PlaneGeometry(s.size.width, s.size.height);
    this.mesh.geometry = newGeom;
    geom.dispose();
  }

  /**
   * Подсветка при наведении луча (interaction/pointer). MeshBasicMaterial
   * умножает map на color, поэтому «подсветка» — это лёгкий голубой тон,
   * а выбранная панель светится сильнее.
   */
  setHighlight(state: "none" | "hover" | "selected"): void {
    const mat = this.mesh.material as THREE.MeshBasicMaterial;
    if (state === "hover") mat.color.setHex(0xbfe6ff);
    else if (state === "selected") mat.color.setHex(0x8fd4ff);
    else mat.color.setHex(0xffffff);
  }

  /** Подпись с topic в углу панели (для UI/отладки). */
  setLabel(text: string): void {
    if (!this.showLabel || !this.ctx) return;
    this.drawLabel(text);
    this.texture.needsUpdate = true;
  }

  /** Подставить JPEG-байты. Возвращает false, если кадр дропнут (GPU занят). */
  ingestJpeg(jpeg: Uint8Array): boolean {
    this.frameCount += 1;
    // Drop-oldest: предыдущий кадр ещё декодируется — этот пропускаем.
    if (this.decoding) {
      this.droppedCount += 1;
      return false;
    }
    this.decoding = true;
    const blob = new Blob([jpeg as BlobPart], { type: "image/jpeg" });
    this.decodeFrame(blob).then(
      (frame) => {
        this.decoding = false;
        this.presentFrame(frame);
      },
      () => {
        this.decoding = false;
        this.droppedCount += 1;
      }
    );
    return true;
  }

  getStats(): { frameCount: number; droppedCount: number } {
    return { frameCount: this.frameCount, droppedCount: this.droppedCount };
  }

  dispose(): void {
    this.disposed = true;
    (this.mesh.geometry as THREE.BufferGeometry).dispose();
    (this.mesh.material as THREE.Material).dispose();
    this.texture.dispose();
    if (this.currentBitmap) {
      this.currentBitmap.close();
      this.currentBitmap = null;
    }
  }

  /** Декод JPEG вне главного потока; фолбэк — <img>, если API нет. */
  private decodeFrame(blob: Blob): Promise<DecodedFrame> {
    if (typeof createImageBitmap !== "function") {
      return this.decodeViaImageElement(blob);
    }
    try {
      return createImageBitmap(blob, this.bitmapOptions);
    } catch {
      // Реализация без второго аргумента / без поддержки Blob.
      return this.decodeViaImageElement(blob);
    }
  }

  /** Старый путь: медленно (декод на главном потоке), зато везде есть. */
  private decodeViaImageElement(blob: Blob): Promise<DecodedFrame> {
    return new Promise<DecodedFrame>((resolve, reject) => {
      const url = URL.createObjectURL(blob);
      const img = new Image();
      img.onload = () => {
        URL.revokeObjectURL(url);
        resolve(img);
      };
      img.onerror = () => {
        URL.revokeObjectURL(url);
        reject(new Error("VideoPanel: JPEG decode failed"));
      };
      img.src = url;
    });
  }

  /** Декодированный кадр → текстура. */
  private presentFrame(frame: DecodedFrame): void {
    if (this.disposed) {
      asBitmap(frame)?.close();
      return;
    }
    if (this.ctx && this.canvas) {
      // Панель с подписью: метка ложится поверх видео в том же canvas.
      this.ctx.drawImage(frame, 0, 0, this.canvas.width, this.canvas.height);
      this.drawLabel(this.state.topic);
      this.texture.needsUpdate = true;
      // Пиксели уже скопированы в canvas — битмап больше не нужен.
      asBitmap(frame)?.close();
      return;
    }
    // Быстрый путь: кадр становится источником текстуры как есть.
    const previous = this.currentBitmap;
    const bitmap = asBitmap(frame);
    this.currentBitmap = bitmap;
    this.texture.flipY = bitmap === null;
    this.texture.image = frame;
    this.texture.needsUpdate = true;
    // Предыдущий кадр освобождаем ПОСЛЕ замены, иначе течёт GPU-память.
    if (previous && previous !== bitmap) previous.close();
  }

  private applyTransform(): void {
    this.mesh.position.set(this.state.position.x, this.state.position.y, this.state.position.z);
    // Смотрит на пользователя: rotateY по углу между facing и +Z.
    const angle = Math.atan2(this.state.facing.x, this.state.facing.z);
    this.mesh.rotation.y = angle;
  }

  private drawLabel(text: string): void {
    this.ctx!.fillStyle = "#000";
    this.ctx!.fillRect(0, 0, 200, 24);
    this.ctx!.fillStyle = "#fff";
    this.ctx!.font = "12px monospace";
    this.ctx!.fillText(text, 6, 16);
  }
}
