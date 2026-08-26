// stream_select UI: lil-gui справа-сверху. Add Panel / Close / Layout reset /
// Connection status / Switch stream. Phase 1.5: +XR Mode (Enter/Exit VR).

import GUI from "lil-gui";
import type { PanelManager } from "../scene/panel_manager";
import type { StreamMeta } from "../wire/messages";
import type { XrBootstrap } from "../xr_bootstrap";

export interface StreamSelectHandle {
  destroy(): void;
  setConnectionStatus(text: string, cls: "connected" | "connecting" | "lost"): void;
  setAvailableStreams(streams: StreamMeta[]): void;
  /** Обновить статус XR-сессии в UI. */
  setXrSessionState(state: "not-in-vr" | "in-vr" | string): void;
  refresh(): void;
}

export interface StreamSelectOptions {
  panels: PanelManager;
  onSubscribe(topic: string): void;
  onUnsubscribe(topic: string): void;
  onResetLayout(): void;
  /** Полный сброс (storage + reset). Опционально — иначе работает как onResetLayout. */
  onResetToDefault?: () => void;
  getActiveTopics(): string[];
  xr: XrBootstrap;
  onEnterVr(): void;
  onExitVr(): void;
}

export function createStreamSelect(opts: StreamSelectOptions): StreamSelectHandle {
  const gui = new GUI({ title: "rob_box_quest / stream_select", width: 280 });
  const state = {
    status: "CONNECTING…",
    statusClass: "connecting" as "connected" | "connecting" | "lost",
    addStream: "camera_rear",
    addPanel: () => addPanel(),
    resetLayout: () => {
      opts.onResetLayout();
      refresh();
    }
  };

  // XR-Mode состояние (Phase 1.5, design §4).
  // xrStatus: "unsupported" | "supported-not-in-vr" | "in-vr" | "failed:<msg>"
  const xrState = {
    status: "checking…",
    enter: () => opts.onEnterVr(),
    exit: () => opts.onExitVr()
  };
  let xrEnterCtrl: ReturnType<GUI["add"]> | null = null;
  let xrExitCtrl: ReturnType<GUI["add"]> | null = null;

  // Создаём папку заранее; visibility/кнопки зависят от поддержки.
  const folderXr = gui.addFolder("XR Mode (Quest)");
  const xrStatusCtrl = folderXr.add(xrState, "status").name("Status").disable();
  folderXr.close();

  // Асинхронная проверка поддержки XR (некоторые браузеры зависают).
  void opts.xr.isSupported("immersive-vr").then((ok) => {
    if (ok) {
      xrState.status = "supported, not in VR";
      xrStatusCtrl.updateDisplay();
      folderXr.open();
      // Кнопка входа: lil-gui хранит controller; добавляем один раз.
      if (!xrEnterCtrl) {
        xrEnterCtrl = folderXr.add(xrState, "enter").name("▶ Enter VR");
      }
    } else {
      xrState.status = "unsupported";
      xrStatusCtrl.updateDisplay();
      folderXr.close();
    }
  });

  let availableStreams: StreamMeta[] = [];

  const folderStatus = gui.addFolder("Connection");
  const statusCtrl = folderStatus.add(state, "status").name("Status").disable();
  folderStatus.open();

  const folderPanels = gui.addFolder("Panels");
  const addCtrl = folderPanels.add(state, "addStream", availableTopicsOrEmpty()).name("Add Stream");
  folderPanels.add(state, "addPanel").name("Add Panel");
  folderPanels.add(state, "resetLayout").name("Reset Layout");
  // Кнопка полного сброса: если есть onResetToDefault — используем её,
  // иначе fallback на onResetLayout (на случай если потребитель не подключил
  // persistence).
  void folderPanels.add(
    {
      reset: () => {
        if (opts.onResetToDefault) {
          opts.onResetToDefault();
        } else {
          opts.onResetLayout();
        }
        refresh();
      }
    },
    "reset"
  ).name("Reset to Default");
  folderPanels.open();

  // Контролы для каждой panel: switch stream + close.
  const panelCtrls: Array<{ id: string; switchCtrl: { updateDisplay(): void; destroy(): void }; closeCtrl: { destroy(): void } }> = [];

  function availableTopicsOrEmpty(): string[] {
    return availableStreams.length
      ? availableStreams.map((s) => s.topic)
      : ["camera_rear", "camera_oak_color", "camera_oak_depth", "camera_ceiling"];
  }

  function addPanel(): void {
    const topic = state.addStream;
    opts.panels.createPanel(topic);
    opts.onSubscribe(topic);
    refresh();
  }

  function refresh(): void {
    // Очистить старые controls для panels.
    for (const c of panelCtrls) {
      c.switchCtrl.destroy();
      c.closeCtrl.destroy();
    }
    panelCtrls.length = 0;
    addCtrl.options(availableTopicsOrEmpty());
    addCtrl.updateDisplay();

    for (const p of opts.panels.list()) {
      const proxy = { topic: p.topic };
      const switchCtrl = folderPanels
        .add(proxy, "topic", availableTopicsOrEmpty())
        .name(`Switch [${p.id}]`)
        .onChange((v: string) => {
          opts.panels.switchStream(p.id, v);
          opts.onUnsubscribe(p.topic);
          opts.onSubscribe(v);
          refresh();
        });
      const closeBtn = { close: () => closePanel(p.id, p.topic) };
      const closeCtrl = folderPanels.add(closeBtn, "close").name(`✕ Close [${p.id}]`);
      panelCtrls.push({
        id: p.id,
        switchCtrl: switchCtrl as { updateDisplay(): void; destroy(): void },
        closeCtrl: closeCtrl as { destroy(): void }
      });
    }
  }

  function closePanel(id: string, topic: string): void {
    opts.panels.close(id);
    opts.onUnsubscribe(topic);
    refresh();
  }

  return {
    destroy(): void {
      gui.destroy();
    },
    setConnectionStatus(text, cls): void {
      state.status = text;
      state.statusClass = cls;
      statusCtrl.updateDisplay();
      // цвет обновим через DOM (lil-gui не умеет менять класс напрямую).
      const el = (statusCtrl.domElement as HTMLElement).closest(".controller");
      if (el) {
        (el as HTMLElement).style.color =
          cls === "connected" ? "var(--accent)" : cls === "lost" ? "var(--danger)" : "var(--warn)";
      }
    },
    setAvailableStreams(streams): void {
      availableStreams = streams;
      addCtrl.options(availableTopicsOrEmpty());
      addCtrl.updateDisplay();
    },
    setXrSessionState(s): void {
      // Перерисуем XR-блок: если в VR — кнопка Exit, иначе Enter (или unsupported).
      xrState.status = s;
      xrStatusCtrl.updateDisplay();
      if (s === "in-vr") {
        if (xrEnterCtrl) {
          xrEnterCtrl.destroy();
          xrEnterCtrl = null;
        }
        if (!xrExitCtrl) {
          xrExitCtrl = folderXr.add(xrState, "exit").name("■ Exit VR");
        }
        folderXr.open();
      } else if (s === "not-in-vr") {
        if (xrExitCtrl) {
          xrExitCtrl.destroy();
          xrExitCtrl = null;
        }
        // Кнопка Enter уже могла быть добавлена на старте.
        if (!xrEnterCtrl) {
          xrEnterCtrl = folderXr.add(xrState, "enter").name("▶ Enter VR");
        }
      } else {
        // failed:<msg> — просто показываем статус, кнопки оставляем как есть.
        if (xrEnterCtrl) {
          xrEnterCtrl.destroy();
          xrEnterCtrl = null;
        }
        if (xrExitCtrl) {
          xrExitCtrl.destroy();
          xrExitCtrl = null;
        }
      }
    },
    refresh
  };
}