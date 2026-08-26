// Phase 2 §2.5: lil-gui контролы для управления panels.
//
// Дополняет существующий stream_select.ts: новая папка "Layout" с:
//   - Add Panel (dropdown topics = DEFAULT_VIDEO_TOPICS + lidar_2d/3d)
//   - Reset Layout (вызывает onResetLayout)
//   - Reset to Default (onResetToDefault — очищает storage + reset)
//   - для каждой panel: Switch [id], Opacity (cycle), ✕ Close [id]
//
// Сделано отдельным модулем, чтобы stream_select.ts не разрастался.

import GUI from "lil-gui";
import {
  PanelManager,
  DEFAULT_VIDEO_TOPICS,
  OPACITY_LEVELS
} from "../scene/panel_manager";

export interface PanelLayoutGuiOptions {
  panels: PanelManager;
  /** Дополнительные topics, которых нет в DEFAULT_VIDEO_TOPICS (lidar_2d/3d). */
  extraTopics?: string[];
  /** Колбэк при add panel: подписаться на topic. */
  onAdd(topic: string): void;
  /** Колбэк при close panel: отписаться от topic. */
  onRemove(topic: string): void;
  /** Колбэк при switch stream: отписаться от старого, подписаться на новый. */
  onSwitch?(oldTopic: string, newTopic: string): void;
  /** Сброс layout (без очистки storage). */
  onResetLayout(): void;
  /** Полный сброс: очистить storage + reset. */
  onResetToDefault(): void;
}

export interface PanelLayoutGuiHandle {
  destroy(): void;
  /** Вызвать, когда список panels изменился (создание/удаление). */
  refresh(): void;
}

function allTopics(extra: string[]): string[] {
  return [...DEFAULT_VIDEO_TOPICS, ...extra];
}

function formatOpacity(v: number): string {
  return `${Math.round(v * 100)}%`;
}

export function createPanelLayoutGui(opts: PanelLayoutGuiOptions): PanelLayoutGuiHandle {
  // Не создаём свой GUI, а подключаемся к существующему через контроллер.
  // Для простоты делаем отдельный GUI (компактный, сворачиваемый).
  const gui = new GUI({ title: "Layout", width: 260 });

  const topics = allTopics(opts.extraTopics ?? ["lidar_2d", "lidar_3d"]);

  const state = {
    addTopic: topics[0],
    addPanel: (): void => {
      const t = state.addTopic;
      opts.panels.createPanel(t);
      opts.onAdd(t);
      refresh();
    },
    resetLayout: (): void => {
      opts.onResetLayout();
      refresh();
    },
    resetToDefault: (): void => {
      opts.onResetToDefault();
      refresh();
    }
  };

  const folderLayout = gui.addFolder("Layout");
  folderLayout.add(state, "addTopic", topics).name("Add topic");
  folderLayout.add(state, "addPanel").name("+ Add Panel");
  folderLayout.add(state, "resetLayout").name("Reset Layout");
  folderLayout.add(state, "resetToDefault").name("Reset to Default");
  folderLayout.open();

  const folderPanels = gui.addFolder("Panels");
  folderPanels.open();

  // Контролы для каждой panel: switch + opacity + close.
  // Сохраняем чтобы удалять при refresh.
  const panelCtrls: Array<{ destroyAll(): void }> = [];

  function refresh(): void {
    for (const c of panelCtrls) c.destroyAll();
    panelCtrls.length = 0;
    for (const p of opts.panels.list()) {
      const proxy = {
        topic: p.topic,
        opacityLabel: formatOpacity(p.opacity),
        switchTopic: (v: string): void => {
          if (v === p.topic) return;
          opts.onSwitch?.(p.topic, v);
          opts.panels.switchStream(p.id, v);
          refresh();
        },
        cycleOpacity: (): void => {
          opts.panels.cycleOpacity(p.id);
          refresh();
        },
        close: (): void => {
          opts.panels.close(p.id);
          opts.onRemove(p.topic);
          refresh();
        }
      };
      const switchCtrl = folderPanels
        .add(proxy, "topic", topics)
        .name(`Switch [${p.id}]`)
        .onChange((v: string) => proxy.switchTopic(v));
      const opacityCtrl = folderPanels
        .add(proxy, "opacityLabel")
        .name(`Opacity [${p.id}]`)
        .disable();
      const opacityBtn = folderPanels
        .add(proxy, "cycleOpacity")
        .name(`Cycle opacity [${p.id}]`);
      const closeCtrl = folderPanels.add(proxy, "close").name(`✕ Close [${p.id}]`);
      panelCtrls.push({
        destroyAll(): void {
          switchCtrl.destroy();
          opacityCtrl.destroy();
          opacityBtn.destroy();
          closeCtrl.destroy();
        }
      });
      // Тихий no-op чтобы OPACITY_LEVELS не попал в unused (используется в JSDoc).
      void OPACITY_LEVELS;
    }
  }

  refresh();

  return {
    destroy(): void {
      for (const c of panelCtrls) c.destroyAll();
      gui.destroy();
    },
    refresh
  };
}
