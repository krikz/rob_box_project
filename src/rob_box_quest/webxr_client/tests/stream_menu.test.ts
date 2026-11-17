// Меню выбора стрима: разбор id целей указателя.
//
// Само меню рисуется на canvas, которого в jsdom нет (см. комментарий в
// bridge_environment.test.ts), поэтому здесь — только чистая часть:
// отображение id цели указателя обратно в topic. Именно на ней держится
// маршрутизация клика в captain_bridge (`menu:` → смена стрима панели,
// всё остальное → выбор панели).

import { describe, it, expect } from "vitest";
import { MENU_TARGET_PREFIX, topicFromTargetId } from "../src/scene/stream_menu";

describe("topicFromTargetId", () => {
  it("extracts the topic from a menu target id", () => {
    expect(topicFromTargetId(`${MENU_TARGET_PREFIX}camera_ceiling`)).toBe("camera_ceiling");
  });

  it("returns null for a panel id, so panel clicks stay panel clicks", () => {
    expect(topicFromTargetId("p1")).toBeNull();
  });

  it("does not mistake a topic containing the prefix elsewhere", () => {
    expect(topicFromTargetId("camera_menu:x")).toBeNull();
  });

  it("handles an empty topic without throwing", () => {
    expect(topicFromTargetId(MENU_TARGET_PREFIX)).toBe("");
  });
});
