// AV-27 / issue #1919: путь отрисовки 3D-меню TTS picker'а
// (src/scene/tts_picker_menu.ts).
//
// jsdom не реализует canvas 2D (см. bridge_environment.test.ts), поэтому
// подменяем `getContext("2d")` записывающим фейком: он собирает все
// `fillText`, и мы проверяем, что на плашках реально нарисовано —
// в частности точный текст пустого состояния из карточки
// («Provider does not expose a voice list»), который ЛЕГКО потерять при
// рефакторинге, и что мёртвые кнопки не регистрируются как цели указателя.

import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import * as THREE from "three";
import { createTtsPickerMenu, MAX_VISIBLE_ROWS } from "../src/scene/tts_picker_menu";
import {
  APPLY_TARGET_ID,
  CLOSE_TARGET_ID,
  EMPTY_VOICES_TEXT,
  INITIAL_TTS_PICKER_STATE,
  LAUNCH_TARGET_ID,
  STOP_TARGET_ID,
  previewTargetId,
  selectTargetId,
  ttsPickerReducer,
  type TtsPickerAction,
  type TtsPickerState
} from "../src/state/tts_picker_state";
import type { VoiceInfo } from "../src/wire/messages";

/** Все строки, нарисованные через fillText с момента последнего сброса. */
let drawn: string[] = [];
let originalGetContext: HTMLCanvasElement["getContext"];

function fakeContext(): CanvasRenderingContext2D {
  return {
    clearRect: () => undefined,
    fillRect: () => undefined,
    fillText: (text: string) => {
      drawn.push(text);
    },
    set fillStyle(_v: unknown) {},
    get fillStyle() {
      return "#000";
    },
    set font(_v: unknown) {},
    get font() {
      return "10px monospace";
    },
    set textBaseline(_v: unknown) {},
    get textBaseline() {
      return "middle" as CanvasTextBaseline;
    },
    set textAlign(_v: unknown) {},
    get textAlign() {
      return "left" as CanvasTextAlign;
    }
  } as unknown as CanvasRenderingContext2D;
}

beforeEach(() => {
  drawn = [];
  originalGetContext = HTMLCanvasElement.prototype.getContext;
  HTMLCanvasElement.prototype.getContext = vi.fn((kind: string) =>
    kind === "2d" ? fakeContext() : null
  ) as unknown as HTMLCanvasElement["getContext"];
});

afterEach(() => {
  HTMLCanvasElement.prototype.getContext = originalGetContext;
});

const VOICES: VoiceInfo[] = [
  { voice_id: "alena", display_name: "Алёна", language: "ru-RU", gender: "female", provider: "yandex" },
  { voice_id: "filipp", display_name: "Филипп", language: "ru-RU", gender: "male", provider: "yandex" }
];

function stateOf(actions: TtsPickerAction[]): TtsPickerState {
  return actions.reduce((st, a) => ttsPickerReducer(st, a), INITIAL_TTS_PICKER_STATE);
}

describe("tts_picker_menu — пустое состояние", () => {
  it("рисует ДОСЛОВНЫЙ текст из карточки, когда voices: []", () => {
    const menu = createTtsPickerMenu();
    drawn = [];
    menu.render(stateOf([{ kind: "open" }, { kind: "voice_list", voices: [] }]));
    expect(drawn).toContain(EMPTY_VOICES_TEXT);
    expect(drawn).toContain("Provider does not expose a voice list");
    menu.dispose();
  });

  it("не рисует ни одного voice_id, когда список пуст (ничего не выдумываем)", () => {
    const menu = createTtsPickerMenu();
    drawn = [];
    menu.render(stateOf([{ kind: "open" }, { kind: "voice_list", voices: [] }]));
    expect(drawn.some((t) => t.includes("alena") || t.includes("Алёна"))).toBe(false);
    menu.dispose();
  });

  it("в loading рисует заглушку загрузки, а не пустое состояние", () => {
    const menu = createTtsPickerMenu();
    drawn = [];
    menu.render(stateOf([{ kind: "open" }]));
    expect(drawn.some((t) => t.includes("loading voices"))).toBe(true);
    expect(drawn).not.toContain(EMPTY_VOICES_TEXT);
    menu.dispose();
  });
});

describe("tts_picker_menu — наполненный список", () => {
  const ready = () =>
    stateOf([
      { kind: "open" },
      { kind: "voice_list", voices: VOICES, activeVoice: "alena", activeProvider: "yandex" }
    ]);

  it("рисует строки голосов, метку ACTIVE и кнопки", () => {
    const menu = createTtsPickerMenu();
    drawn = [];
    menu.render(ready());
    expect(drawn).toContain("Алёна · ru-RU");
    expect(drawn).toContain("Филипп · ru-RU");
    expect(drawn).toContain("ACTIVE");
    expect(drawn).toContain("APPLY");
    expect(drawn).toContain("STOP");
    expect(drawn).toContain("CLOSE");
    expect(drawn.filter((t) => t === "▶ PREVIEW")).toHaveLength(2);
    menu.dispose();
  });

  it("header печатает провайдера и активный голос", () => {
    const menu = createTtsPickerMenu();
    drawn = [];
    menu.render(ready());
    expect(drawn).toContain("TTS VOICES · yandex · now: alena");
    menu.dispose();
  });

  it("во время apply рисует APPLYING в header", () => {
    const menu = createTtsPickerMenu();
    const st = ttsPickerReducer(ready(), { kind: "apply_sent", voiceId: "filipp" });
    drawn = [];
    menu.render(st);
    expect(drawn).toContain("APPLYING filipp…");
    menu.dispose();
  });

  it("список длиннее MAX_VISIBLE_ROWS обрезается (меню не выше человека)", () => {
    const many: VoiceInfo[] = Array.from({ length: MAX_VISIBLE_ROWS + 4 }, (_, i) => ({
      voice_id: `v${i}`,
      display_name: `V${i}`,
      language: "ru-RU",
      gender: "neutral" as const
    }));
    const menu = createTtsPickerMenu();
    menu.show(new THREE.Vector3(0, 1, -1), 0);
    menu.render(stateOf([{ kind: "open" }, { kind: "voice_list", voices: many }]));
    const rowTargets = menu.targets().filter((t) => t.id.startsWith("tts:voice:"));
    expect(rowTargets).toHaveLength(MAX_VISIBLE_ROWS);
    menu.dispose();
  });
});

describe("tts_picker_menu — цели указателя", () => {
  const ready = () =>
    stateOf([
      { kind: "open" },
      { kind: "voice_list", voices: VOICES, activeVoice: "alena", activeProvider: "yandex" }
    ]);

  it("скрытое меню не отдаёт ни одной цели (невидимое не кликается)", () => {
    const menu = createTtsPickerMenu();
    menu.render(ready());
    expect(menu.isVisible()).toBe(false);
    expect(menu.targets()).toEqual([]);
    menu.dispose();
  });

  it("вкладка VOICE — отдельная постоянная цель", () => {
    const menu = createTtsPickerMenu();
    expect(menu.launchTarget().id).toBe(LAUNCH_TARGET_ID);
    expect(menu.launchObject.visible).toBe(true);
    menu.dispose();
  });

  it("открытое меню отдаёт строки, PREVIEW и CLOSE; мёртвый APPLY/STOP — нет", () => {
    const menu = createTtsPickerMenu();
    menu.show(new THREE.Vector3(0, 1, -1), 0);
    menu.render(ready());
    const ids = menu.targets().map((t) => t.id);
    expect(ids).toContain(selectTargetId("alena"));
    expect(ids).toContain(previewTargetId("filipp"));
    expect(ids).toContain(CLOSE_TARGET_ID);
    // Ничего не выбрано и preview не идёт → эти кнопки луч ловить не должны.
    expect(ids).not.toContain(APPLY_TARGET_ID);
    expect(ids).not.toContain(STOP_TARGET_ID);
    menu.dispose();
  });

  it("APPLY становится целью после выбора голоса", () => {
    const menu = createTtsPickerMenu();
    menu.show(new THREE.Vector3(0, 1, -1), 0);
    menu.render(ttsPickerReducer(ready(), { kind: "select", voiceId: "filipp" }));
    expect(menu.targets().map((t) => t.id)).toContain(APPLY_TARGET_ID);
    menu.dispose();
  });

  it("STOP становится целью, пока идёт preview, и пропадает после done", () => {
    const menu = createTtsPickerMenu();
    menu.show(new THREE.Vector3(0, 1, -1), 0);
    const running = ttsPickerReducer(ready(), {
      kind: "preview_sent",
      requestId: "r1",
      voiceId: "alena"
    });
    menu.render(running);
    expect(menu.targets().map((t) => t.id)).toContain(STOP_TARGET_ID);
    menu.render(ttsPickerReducer(running, { kind: "preview_done", requestId: "r1" }));
    expect(menu.targets().map((t) => t.id)).not.toContain(STOP_TARGET_ID);
    menu.dispose();
  });

  it("hide снимает все цели меню, вкладка остаётся", () => {
    const menu = createTtsPickerMenu();
    menu.show(new THREE.Vector3(0, 1, -1), 0);
    menu.render(ready());
    expect(menu.targets().length).toBeGreaterThan(0);
    menu.hide();
    expect(menu.targets()).toEqual([]);
    expect(menu.launchTarget().id).toBe(LAUNCH_TARGET_ID);
    menu.dispose();
  });
});

describe("tts_picker_menu — слой глубины (как у stream_menu)", () => {
  it("группа и плашки имеют renderOrder 20 и depthTest=false", () => {
    const menu = createTtsPickerMenu();
    expect(menu.object.renderOrder).toBe(20);
    const meshes = menu.object.children.filter((c) => (c as THREE.Mesh).isMesh) as THREE.Mesh[];
    expect(meshes.length).toBeGreaterThan(0);
    for (const m of meshes) {
      expect(m.renderOrder).toBe(20);
      expect((m.material as THREE.MeshBasicMaterial).depthTest).toBe(false);
    }
    menu.dispose();
  });

  it("show/hide переключают видимость группы", () => {
    const menu = createTtsPickerMenu();
    expect(menu.isVisible()).toBe(false);
    menu.show(new THREE.Vector3(1, 2, 3), Math.PI / 2);
    expect(menu.isVisible()).toBe(true);
    // Меню встаёт над точкой привязки, как stream_menu (+0.5 по Y).
    expect(menu.object.position.y).toBeCloseTo(2.5);
    expect(menu.object.rotation.y).toBeCloseTo(Math.PI / 2);
    menu.hide();
    expect(menu.isVisible()).toBe(false);
    menu.dispose();
  });
});
