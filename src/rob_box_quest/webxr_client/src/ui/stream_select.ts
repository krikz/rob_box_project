// stream_select UI: lil-gui справа-сверху. Add Panel / Close / Layout reset /
// Connection status / Switch stream. Phase 1.5: +XR Mode (Enter/Exit VR).
// Phase 2 §6.2: каждая stream-item draggable (HTML5 drag-and-drop API) — drop на
//               3D panel переназначает топик (panel.onDropTopic callback).
// Phase 2 §6.3: cycle button для каждой panel — single → split-2h → split-2v →
//               2x2 → pip → single (через callbacks panel.cycleLayoutMode).
//
// HTML5 drag API: dragstart ставит topic в dataTransfer, drop на panel читает
// через opts.onDropTopic(panelId, topic). Сам panel реализует raycast в scene
// (captain_bridge.ts) — этот модуль только триггерит callback.

import GUI from "lil-gui";
import type { PanelManager } from "../scene/panel_manager";
import type { LayoutMode } from "../scene/panel_layout_modes";
import type { StreamMeta } from "../wire/messages";
import type { XrBootstrap } from "../xr_bootstrap";

export interface StreamSelectHandle {
  destroy(): void;
  setConnectionStatus(text: string, cls: "connected" | "connecting" | "lost"): void;
  setAvailableStreams(streams: StreamMeta[]): void;
  /** Обновить layout-mode конкретной panel (UI mirror server-side layout). */
  setPanelLayoutMode(panelId: string, mode: LayoutMode): void;
  /** Обновить статус XR-сессии в UI. */
  setXrSessionState(state: "not-in-vr" | "in-vr" | string): void;
  refresh(): void;
}

export interface StreamSelectOptions {
  panels: PanelManager;
  onSubscribe(topic: string): void;
  onUnsubscribe(topic: string): void;
  onResetLayout(): void;
  /** Когда drag-from-gui item дропают на panel: panelId, topic. */
  onDropTopic(panelId: string, topic: string): void;
  /** Когда cycle button нажат — UI просит следующий layout для panel. */
  onCyclePanelLayout(panelId: string, currentMode: LayoutMode): void;
  getActiveTopics(): string[];
  /** Опционально: текущие layout modes по panelId (для UI отображения). */
  getPanelLayoutModes?(): Map<string, LayoutMode>;
  xr: XrBootstrap;
  onEnterVr(): void;
  onExitVr(): void;
}

const DRAG_MIME = "application/x-rob-box-topic";

export function createStreamSelect(opts: StreamSelectOptions): StreamSelectHandle {
  const gui = new GUI({ title: "rob_box_quest / stream_select", width: 280 });
  const state = {
    status: "CONNECTING…",
    statusClass: "connecting" as "connected" | "connecting" | "lost",
    addStream: "camera_rear",
    addPanel: (): void => addPanel(),
    resetLayout: (): void => {
      opts.onResetLayout();
      refresh();
    }
  };

  // XR-Mode состояние (Phase 1.5, design §4).
  // xrStatus: "unsupported" | "supported-not-in-vr" | "in-vr" | "failed:<msg>"
  const xrState = {
    status: "checking…",
    enter: (): void => opts.onEnterVr(),
    exit: (): void => opts.onExitVr()
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
  folderPanels.open();

  // Контролы для каждой panel: switch stream + close + cycle layout.
  const panelCtrls: Array<{
    id: string;
    switchCtrl: { updateDisplay(): void; destroy(): void };
    closeCtrl: { destroy(): void };
    cycleCtrl: { name(n: string): void; destroy(): void } | null;
  }> = [];

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
      c.cycleCtrl?.destroy();
    }
    panelCtrls.length = 0;
    addCtrl.options(availableTopicsOrEmpty());
    addCtrl.updateDisplay();

    const layoutModes = opts.getPanelLayoutModes?.() ?? new Map<string, LayoutMode>();

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
      const closeBtn = { close: (): void => closePanel(p.id, p.topic) };
      const closeCtrl = folderPanels.add(closeBtn, "close").name(`✕ Close [${p.id}]`);

      // Cycle layout button.
      const cycleBtn = { cycle: (): void => cyclePanelLayout(p.id) };
      const currentMode = layoutModes.get(p.id) ?? "single";
      const cycleCtrl = folderPanels.add(cycleBtn, "cycle").name(`▦ ${currentMode} [${p.id}]`);

      // Drag-source для stream-item: HTML5 dragstart → dataTransfer.setData.
      // Назначаем draggable=true и обработчик dragstart через addEventListener
      // на DOM-элементе контрола.
      attachDragSource(switchCtrl, p.topic);

      panelCtrls.push({
        id: p.id,
        switchCtrl: switchCtrl as { updateDisplay(): void; destroy(): void },
        closeCtrl: closeCtrl as { destroy(): void },
        cycleCtrl: cycleCtrl as { name(n: string): void; destroy(): void }
      });
    }
  }

  function closePanel(id: string, topic: string): void {
    opts.panels.close(id);
    opts.onUnsubscribe(topic);
    refresh();
  }

  function cyclePanelLayout(panelId: string): void {
    const current = opts.getPanelLayoutModes?.().get(panelId) ?? "single";
    opts.onCyclePanelLayout(panelId, current);
  }

  function attachDragSource(
    ctrl: { domElement: HTMLElement },
    topic: string
  ): void {
    const el = ctrl.domElement as HTMLElement;
    // Для lil-gui select-dropdown: draggable вешаем на сам controller (родитель).
    const draggable =
      (el.closest(".controller") as HTMLElement | null) ?? el;
    draggable.setAttribute("draggable", "true");
    draggable.addEventListener("dragstart", (ev: Event) => {
      const dragEv = ev as DragEvent;
      dragEv.dataTransfer?.setData(DRAG_MIME, topic);
      dragEv.dataTransfer?.setData("text/plain", topic);
      if (dragEv.dataTransfer) dragEv.dataTransfer.effectAllowed = "move";
    });
  }

  return {
    destroy(): void {
      gui.destroy();
    },
    setConnectionStatus(text, cls): void {
      state.status = text;
      state.statusClass = cls;
      statusCtrl.updateDisplay();
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
    setPanelLayoutMode(panelId, mode): void {
      const c = panelCtrls.find((x) => x.id === panelId);
      if (c?.cycleCtrl) c.cycleCtrl.name(`▦ ${mode} [${panelId}]`);
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

/**
 * Вспомогательный хелпер: extract topic из drop-event, если MIME правильный.
 * Используется scene-side обработчиком drop на 3D panel.
 */
export function readDropTopic(ev: DragEvent): string | null {
  if (!ev.dataTransfer) return null;
  return ev.dataTransfer.getData(DRAG_MIME) || ev.dataTransfer.getData("text/plain") || null;
}
