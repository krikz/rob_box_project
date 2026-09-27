// Тесты alert_toast: чистая логика форматирования + DOM-менеджер стека.

import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import {
  alertText,
  createAlertToast,
  type RobotAlert,
  type AlertToast
} from "../src/scene/alert_toast";

// Тестовый helper: все autoHide держим маленькими, чтобы не висеть 8 с в тестах.
const FAST = { autoHideMs: 50, maxVisible: 3 } as const;

const baseAlert = (over: Partial<RobotAlert> = {}): RobotAlert => ({
  code: "BATTERY_LOW",
  active: true,
  level: "warn",
  args: { pct: 15 },
  ts_ms: 1_700_000_000_000,
  ...over
});

// --- alertText (чистая функция) --------------------------------------------

describe("alertText", () => {
  it("BATTERY_LOW с pct подставляет проценты", () => {
    expect(alertText({ code: "BATTERY_LOW", args: { pct: 12 } })).toBe(
      "Батарея 12% — пора на зарядку"
    );
  });

  it("BATTERY_LOW без pct показывает generic-сообщение", () => {
    expect(alertText({ code: "BATTERY_LOW", args: {} })).toBe(
      "Батарея разряжена — пора на зарядку"
    );
  });

  it("WIFI_WEAK не требует args", () => {
    expect(alertText({ code: "WIFI_WEAK" })).toBe(
      "Слабый Wi-Fi — связь с роботом нестабильна"
    );
  });

  it("ROBOT_STUCK подставляет компоненты команды", () => {
    expect(
      alertText({ code: "ROBOT_STUCK", args: { cmd_linear: 0.5, cmd_angular: 0.0 } })
    ).toBe("Робот не едет (команда 0.50 м/с, 0.00 рад/с) — возможно застрял");
  });

  it("ROBOT_STUCK без args — generic", () => {
    expect(alertText({ code: "ROBOT_STUCK" })).toBe(
      "Робот не едет — возможно застрял"
    );
  });

  it("неизвестный код показывается как есть (anti-silent-failure)", () => {
    expect(alertText({ code: "BATTERY_CRITICAL" })).toBe("BATTERY_CRITICAL");
  });
});

// --- createAlertToast (DOM-стек) -------------------------------------------

describe("createAlertToast — базовое поведение", () => {
  let parent: HTMLElement;
  let toast: AlertToast;

  beforeEach(() => {
    parent = document.createElement("div");
    document.body.appendChild(parent);
    toast = createAlertToast({ parent, ...FAST });
  });

  afterEach(() => {
    toast.dispose();
    parent.remove();
  });

  it("при создании стек пуст и скрыт", () => {
    expect(toast.activeCount).toBe(0);
    expect(toast.worstActive).toBeNull();
    const stack = parent.querySelector("[data-alert-toast-stack]") as HTMLElement;
    expect(stack.classList.contains("alert-toast-stack--hidden")).toBe(true);
  });

  it("active:true добавляет toast в стек и показывает", () => {
    toast.ingest(baseAlert({ code: "BATTERY_LOW", args: { pct: 12 } }));
    expect(toast.activeCount).toBe(1);
    const stack = parent.querySelector("[data-alert-toast-stack]") as HTMLElement;
    expect(stack.classList.contains("alert-toast-stack--hidden")).toBe(false);
    const node = stack.querySelector('[data-alert-code="BATTERY_LOW"]') as HTMLElement;
    expect(node).not.toBeNull();
    expect(node.textContent).toContain("Батарея 12%");
    expect(node.classList.contains("alert-toast--warn")).toBe(true);
  });

  it("active:false убирает toast", () => {
    toast.ingest(baseAlert({ code: "BATTERY_LOW", active: true }));
    toast.ingest(baseAlert({ code: "BATTERY_LOW", active: false, level: "info" }));
    expect(toast.activeCount).toBe(0);
    expect(toast.worstActive).toBeNull();
  });

  it("несколько алёртов сосуществуют", () => {
    toast.ingest(baseAlert({ code: "BATTERY_LOW" }));
    toast.ingest(baseAlert({ code: "WIFI_WEAK", args: {} }));
    expect(toast.activeCount).toBe(2);
  });

  it("один и тот же код пересоздаёт toast (новый таймер)", () => {
    toast.ingest(baseAlert({ code: "BATTERY_LOW", ts_ms: 1 }));
    toast.ingest(baseAlert({ code: "BATTERY_LOW", ts_ms: 2 }));
    expect(toast.activeCount).toBe(1);
  });
});

describe("createAlertToast — стек ≤ 3", () => {
  let parent: HTMLElement;
  let toast: AlertToast;

  beforeEach(() => {
    parent = document.createElement("div");
    document.body.appendChild(parent);
    toast = createAlertToast({ parent, ...FAST, maxVisible: 3 });
  });

  afterEach(() => {
    toast.dispose();
    parent.remove();
  });

  it("вытесняет менее важный алёрт, если стек заполнен", () => {
    toast.ingest(baseAlert({ code: "WIFI_WEAK", level: "warn" }));
    toast.ingest(baseAlert({ code: "BATTERY_LOW", level: "warn" }));
    toast.ingest(baseAlert({ code: "EXTRA_1", level: "warn" }));
    expect(toast.activeCount).toBe(3);

    // Новый critical → должен вытеснить самый старый warn.
    toast.ingest(baseAlert({ code: "ROBOT_STUCK", level: "error" }));
    expect(toast.activeCount).toBe(3);
    expect(toast.worstActive?.code).toBe("ROBOT_STUCK");
  });
});

describe("createAlertToast — worstActive", () => {
  let parent: HTMLElement;
  let toast: AlertToast;

  beforeEach(() => {
    parent = document.createElement("div");
    document.body.appendChild(parent);
    toast = createAlertToast({ parent, ...FAST });
  });

  afterEach(() => {
    toast.dispose();
    parent.remove();
  });

  it("error > warn > info по приоритету", () => {
    toast.ingest(baseAlert({ code: "WIFI_WEAK", level: "warn" }));
    expect(toast.worstActive?.code).toBe("WIFI_WEAK");
    toast.ingest(baseAlert({ code: "ROBOT_STUCK", level: "error" }));
    expect(toast.worstActive?.code).toBe("ROBOT_STUCK");
    // Снимаем error → warn снова на вершине.
    toast.ingest(baseAlert({ code: "ROBOT_STUCK", active: false, level: "info" }));
    expect(toast.worstActive?.code).toBe("WIFI_WEAK");
  });
});

describe("createAlertToast — autoHide", () => {
  beforeEach(() => vi.useFakeTimers());
  afterEach(() => vi.useRealTimers());

  it("toast исчезает после autoHideMs + tick (1000 мс интервал проверки)", () => {
    const parent = document.createElement("div");
    document.body.appendChild(parent);
    const toast = createAlertToast({ parent, autoHideMs: 100, maxVisible: 3 });
    try {
      toast.ingest(baseAlert({ code: "BATTERY_LOW" }));
      expect(toast.activeCount).toBe(1);
      vi.advanceTimersByTime(150); // прошло autoHideMs
      vi.advanceTimersByTime(1000); // +tick проверка
      expect(toast.activeCount).toBe(0);
    } finally {
      toast.dispose();
      parent.remove();
    }
  });
});

describe("createAlertToast — errorOverlay интеграция", () => {
  let parent: HTMLElement;
  let dismissCalls: number;
  let showCalls: string[];
  let toast: AlertToast;
  let overlayApi: {
    show: (text: string, detail?: string) => void;
    dismiss: () => void;
    readonly isVisible: boolean;
  };

  beforeEach(() => {
    parent = document.createElement("div");
    document.body.appendChild(parent);
    dismissCalls = 0;
    showCalls = [];
    let visible = false;
    overlayApi = {
      show: (headline: string) => {
        showCalls.push(headline);
        visible = true;
      },
      dismiss: () => {
        dismissCalls += 1;
        visible = false;
      },
      get isVisible(): boolean {
        return visible;
      }
    };
    toast = createAlertToast({
      parent,
      ...FAST,
      errorOverlay: overlayApi as never
    });
  });

  afterEach(() => {
    toast.dispose();
    parent.remove();
  });

  it("error алёрт вызывает overlay.show", () => {
    toast.ingest(baseAlert({ code: "ROBOT_STUCK", level: "error" }));
    expect(showCalls.length).toBe(1);
    expect(showCalls[0]).toContain("Робот не едет");
  });

  it("warn алёрт НЕ вызывает overlay.show", () => {
    toast.ingest(baseAlert({ code: "BATTERY_LOW", level: "warn" }));
    expect(showCalls.length).toBe(0);
  });

  it("снятие error алёрта вызывает overlay.dismiss", () => {
    toast.ingest(baseAlert({ code: "ROBOT_STUCK", level: "error" }));
    toast.ingest(baseAlert({ code: "ROBOT_STUCK", active: false, level: "info" }));
    expect(dismissCalls).toBe(1);
  });
});

describe("createAlertToast — clear()", () => {
  let parent: HTMLElement;
  let toast: AlertToast;

  beforeEach(() => {
    parent = document.createElement("div");
    document.body.appendChild(parent);
    toast = createAlertToast({ parent, ...FAST });
  });

  afterEach(() => {
    toast.dispose();
    parent.remove();
  });

  it("удаляет все активные toast'ы", () => {
    toast.ingest(baseAlert({ code: "BATTERY_LOW" }));
    toast.ingest(baseAlert({ code: "WIFI_WEAK" }));
    expect(toast.activeCount).toBe(2);
    toast.clear();
    expect(toast.activeCount).toBe(0);
  });
});

describe("createAlertToast — dispose()", () => {
  it("удаляет корневой элемент и все toast'ы", () => {
    const parent = document.createElement("div");
    document.body.appendChild(parent);
    const toast = createAlertToast({ parent, ...FAST });
    toast.ingest(baseAlert({ code: "BATTERY_LOW" }));
    toast.dispose();
    expect(parent.querySelector("[data-alert-toast-stack]")).toBeNull();
    parent.remove();
  });

  it("повторный dispose — no-op", () => {
    const parent = document.createElement("div");
    document.body.appendChild(parent);
    const toast = createAlertToast({ parent, ...FAST });
    toast.dispose();
    expect(() => toast.dispose()).not.toThrow();
    parent.remove();
  });
});
