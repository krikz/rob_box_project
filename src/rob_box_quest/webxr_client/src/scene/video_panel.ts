// Video panel — PlaneGeometry + CanvasTexture, обновляется из JPEG payload.
//
// Архитектура (дизайн §3): <img> декодирует JPEG, затем texture.image = img
// и needsUpdate=true. Drop-oldest если GPU занят (флаг ready не успевает
// выставиться → пропускаем кадр).

import * as THREE from "three";
import type { PanelState } from "./panel_manager";

export class VideoPanel {
  readonly mesh: THREE.Mesh;
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private texture: THREE.CanvasTexture;
  private currentImage: HTMLImageElement | null = null;
  private state: PanelState;
  private frameCount = 0;
  private droppedCount = 0;

  constructor(state: PanelState) {
    this.state = state;
    // Внутренний canvas 640×360 — типичный JPEG с камеры робота,
    // CanvasTexture сама масштабируется на Plane.
    this.canvas = document.createElement("canvas");
    this.canvas.width = 640;
    this.canvas.height = 360;
    const ctx = this.canvas.getContext("2d", { alpha: false });
    if (!ctx) {
      throw new Error("VideoPanel: failed to acquire 2D context");
    }
    this.ctx = ctx;
    this.ctx.fillStyle = "#000";
    this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);

    this.texture = new THREE.CanvasTexture(this.canvas);
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

  /** Подпись с topic в углу панели (для UI/отладки). */
  setLabel(text: string): void {
    this.drawLabel(text);
    this.texture.needsUpdate = true;
  }

  /** Подставить JPEG-байты. Возвращает false, если кадр дропнут (GPU занят). */
  ingestJpeg(jpeg: Uint8Array): boolean {
    this.frameCount += 1;
    // Drop-oldest: если текущий image ещё не декодирован — пропускаем.
    if (this.currentImage && !this.currentImage.complete) {
      this.droppedCount += 1;
      return false;
    }
    const img = new Image();
    img.onload = () => {
      if (this.currentImage) {
        URL.revokeObjectURL(this.currentImage.src);
      }
      this.currentImage = img;
      this.ctx!.drawImage(img, 0, 0, this.canvas.width, this.canvas.height);
      this.drawLabel(this.state.topic);
      this.texture.needsUpdate = true;
    };
    img.onerror = () => {
      this.droppedCount += 1;
    };
    const blob = new Blob([jpeg as BlobPart], { type: "image/jpeg" });
    img.src = URL.createObjectURL(blob);
    return true;
  }

  getStats(): { frameCount: number; droppedCount: number } {
    return { frameCount: this.frameCount, droppedCount: this.droppedCount };
  }

  dispose(): void {
    (this.mesh.geometry as THREE.BufferGeometry).dispose();
    (this.mesh.material as THREE.Material).dispose();
    this.texture.dispose();
    if (this.currentImage) {
      URL.revokeObjectURL(this.currentImage.src);
    }
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