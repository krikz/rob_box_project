// Всплывающее меню (выбор стрима, TTS picker) перехватывает луч:
// клик по строке уходит в `menu:<topic>`, а не в панель под меню.
//
// Регрессия: меню всплывает в плоскости панели (та же дистанция, тот же
// поворот), а панели TARS — 4.8×2.7 м, так что строки меню целиком
// лежат внутри панели. Raycaster сортирует хиты по дистанции, панель
// оказывалась не дальше строки — клик «прокликивался» в панель.

import { describe, it, expect, vi } from "vitest";
import * as THREE from "three";
import { PointerSystem } from "../src/interaction/pointer";

const CENTER = { x: 0, y: 1.6, z: 0 };

function plane(w: number, h: number, x: number, y: number, z: number): THREE.Mesh {
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial());
  mesh.position.set(x, y, z);
  mesh.updateMatrixWorld(true);
  return mesh;
}

const ray = (pressed: boolean, dx = 0) => ({
  origin: CENTER,
  direction: { x: dx, y: 0, z: -1 },
  pressed
});

function click(sys: PointerSystem, dx = 0): void {
  sys.update(ray(false, dx));
  sys.update(ray(true, dx));
  sys.update(ray(false, dx));
}

function setup(menuZ: number) {
  const onSelect = vi.fn();
  const sys = new PointerSystem({ center: CENTER, handlers: { onSelect } });
  // Большая панель на z=-2 и строка меню поверх неё.
  sys.addTarget({ id: "p1", object: plane(4.8, 2.7, 0, 1.6, -2), draggable: true });
  sys.addTarget({
    id: "menu:camera_rear",
    object: plane(1.1, 0.16, 0, 1.6, menuZ),
    draggable: false,
    modal: true
  });
  return { sys, onSelect };
}

describe("PointerSystem — модальное меню", () => {
  it("меню в той же плоскости, что панель, получает клик", () => {
    const { sys, onSelect } = setup(-2);
    click(sys);
    expect(onSelect).toHaveBeenCalledWith("menu:camera_rear");
  });

  it("меню дальше панели всё равно получает клик", () => {
    const { sys, onSelect } = setup(-2.05);
    click(sys);
    expect(onSelect).toHaveBeenCalledWith("menu:camera_rear");
  });

  it("мимо строки меню луч по-прежнему попадает в панель", () => {
    const { sys, onSelect } = setup(-2);
    click(sys, 0.3);
    expect(onSelect).toHaveBeenCalledWith("p1");
  });

  it("скрытое меню луч не ловит", () => {
    const onSelect = vi.fn();
    const sys = new PointerSystem({ center: CENTER, handlers: { onSelect } });
    sys.addTarget({ id: "p1", object: plane(4.8, 2.7, 0, 1.6, -2) });
    const row = plane(1.1, 0.16, 0, 1.6, -1.9);
    row.visible = false;
    sys.addTarget({ id: "menu:camera_rear", object: row, modal: true });
    click(sys);
    expect(onSelect).toHaveBeenCalledWith("p1");
  });
});
