// TTS picker: 3D-меню выбора голоса на слое указателя (AV-27 / issue #1919).
//
// Визуально и по механике — тот же приём, что `scene/stream_menu.ts`:
// каждая интерактивная зона — отдельный маленький меш с canvas-текстурой,
// зарегистрированный в PointerSystem как цель `tts:*`. Попадание считает
// сам raycaster, UV-координаты разбирать не нужно.
//
// Почему не DOM: в immersive-vr DOM не виден вообще (см. комментарий в
// stream_menu.ts), а голос оператор выбирает именно в VR.
//
// Состав меню (сверху вниз):
//   [header]                 — фаза + активный провайдер/голос
//   [row + PREVIEW] × N      — строки голосов; строка = select, кнопка = preview
//   [placeholder]            — в loading/empty вместо строк
//   [◀] [N–M из K] [▶]       — листание, если голосов больше страницы
//   [APPLY] [STOP] [CLOSE]   — панель кнопок
//   [footer]                 — прогресс preview / ошибка / подсказка
//
// Всё состояние живёт в `state/tts_picker_state.ts` (чистый редьюсер);
// здесь только отрисовка и геометрия — как status_hud поверх formatStatusLines.

import * as THREE from "three";
import {
  APPLY_TARGET_ID,
  CLOSE_TARGET_ID,
  EMPTY_VOICES_TEXT,
  LAUNCH_TARGET_ID,
  NEXT_PAGE_TARGET_ID,
  PREV_PAGE_TARGET_ID,
  STOP_TARGET_ID,
  canApply,
  canStopPreview,
  previewTargetId,
  selectTargetId,
  ttsFooterText,
  ttsHeaderText,
  ttsPageInfo,
  ttsRowViews,
  type TtsPageInfo,
  type TtsPickerState
} from "../state/tts_picker_state";

/** Ширина всего меню в метрах (стоит рядом с панелью, как stream_menu). */
const MENU_W = 1.3;
const ROW_H = 0.16;
const ROW_GAP = 0.02;
/** Доля ширины строки, отданная кнопке PREVIEW. */
const PREVIEW_FRAC = 0.22;
const HEADER_H = 0.14;
const FOOTER_H = 0.12;
const BUTTON_H = 0.14;
/** Сколько строк рисуем максимум: дальше меню выше человека. */
export const MAX_VISIBLE_ROWS = 8;
/** Высота строки листания (◀ / счётчик / ▶). */
const PAGER_H = 0.12;
/** Доля ширины под каждую стрелку листания. */
const PAGER_ARROW_FRAC = 0.2;

const TEX_ROW_W = 512;
const TEX_ROW_H = 72;

const COLOR_BG = "rgba(10, 13, 17, 0.88)";
const COLOR_BG_ACTIVE = "rgba(46, 194, 126, 0.85)";
const COLOR_BG_SELECTED = "rgba(46, 194, 126, 0.30)";
const COLOR_BG_DISABLED = "rgba(10, 13, 17, 0.55)";
const COLOR_ACCENT = "#2ec27e";
const COLOR_TEXT = "#d6dde5";
const COLOR_TEXT_DIM = "#8b949e";
const COLOR_TEXT_INVERT = "#0a0d11";
const COLOR_WARN = "#e5a50a";
const COLOR_BAD = "#e01b24";

export interface TtsPickerMenuHandle {
  readonly object: THREE.Group;
  /**
   * Постоянная «вкладка» VOICE: маленькая плашка, всегда видимая в сцене.
   * По клику мостик открывает/закрывает меню. Отдельный объект, потому что
   * `object` целиком скрывается вместе с меню.
   */
  readonly launchObject: THREE.Group;
  /** Цель указателя для вкладки VOICE (регистрируется один раз, навсегда). */
  launchTarget(): { id: string; object: THREE.Object3D };
  /** Цели для PointerSystem: только те, что сейчас видимы и активны. */
  targets(): Array<{ id: string; object: THREE.Object3D }>;
  /** Перерисовать под новое состояние (вызывать на каждое изменение стора). */
  render(state: TtsPickerState): void;
  /** Показать меню рядом с точкой (та же поза, что у stream_menu). */
  show(position: THREE.Vector3, facingAngleY: number): void;
  hide(): void;
  isVisible(): boolean;
  dispose(): void;
}

interface Tile {
  mesh: THREE.Mesh;
  canvas: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;
  texture: THREE.CanvasTexture;
  width: number;
  height: number;
}

function createTile(widthM: number, heightM: number, texW: number, texH: number): Tile {
  const canvas = document.createElement("canvas");
  canvas.width = texW;
  canvas.height = texH;
  const ctx = canvas.getContext("2d");
  if (!ctx) throw new Error("tts_picker_menu: failed to acquire 2D context");
  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(widthM, heightM),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthTest: false })
  );
  // Тот же renderOrder, что у stream_menu: меню поверх видео-панелей.
  mesh.renderOrder = 20;
  return { mesh, canvas, ctx, texture, width: texW, height: texH };
}

function fillPlate(tile: Tile, bg: string, accent: string | null): void {
  const { ctx, width, height } = tile;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = bg;
  ctx.fillRect(0, 0, width, height);
  if (accent) {
    ctx.fillStyle = accent;
    ctx.fillRect(0, 0, 8, height);
  }
}

function drawText(
  tile: Tile,
  text: string,
  opts: { color: string; font: string; x?: number; y?: number; align?: CanvasTextAlign }
): void {
  const { ctx, width, height } = tile;
  ctx.fillStyle = opts.color;
  ctx.font = opts.font;
  ctx.textBaseline = "middle";
  ctx.textAlign = opts.align ?? "left";
  const x = opts.x ?? 24;
  const y = opts.y ?? height / 2;
  ctx.fillText(text, opts.align === "center" ? width / 2 : x, y, width - x - 12);
}

export function createTtsPickerMenu(): TtsPickerMenuHandle {
  const group = new THREE.Group();
  group.visible = false;
  group.renderOrder = 20;

  const rowW = MENU_W * (1 - PREVIEW_FRAC);
  const previewW = MENU_W * PREVIEW_FRAC - ROW_GAP;
  const buttonW = (MENU_W - 2 * ROW_GAP) / 3;

  const header = createTile(MENU_W, HEADER_H, TEX_ROW_W, 64);
  const placeholder = createTile(MENU_W, ROW_H, TEX_ROW_W, TEX_ROW_H);
  const footer = createTile(MENU_W, FOOTER_H, TEX_ROW_W, 56);
  const pagerPrev = createTile(MENU_W * PAGER_ARROW_FRAC, PAGER_H, 128, 56);
  const pagerNext = createTile(MENU_W * PAGER_ARROW_FRAC, PAGER_H, 128, 56);
  const pagerLabel = createTile(MENU_W * (1 - 2 * PAGER_ARROW_FRAC) - 2 * ROW_GAP, PAGER_H, 320, 56);
  const applyBtn = createTile(buttonW, BUTTON_H, 256, 64);
  const stopBtn = createTile(buttonW, BUTTON_H, 256, 64);
  const closeBtn = createTile(buttonW, BUTTON_H, 256, 64);

  const rowTiles: Tile[] = [];
  const previewTiles: Tile[] = [];
  for (let i = 0; i < MAX_VISIBLE_ROWS; i++) {
    rowTiles.push(createTile(rowW, ROW_H, TEX_ROW_W, TEX_ROW_H));
    previewTiles.push(createTile(previewW, ROW_H, 192, TEX_ROW_H));
  }

  const allTiles = [
    header,
    placeholder,
    footer,
    pagerPrev,
    pagerNext,
    pagerLabel,
    applyBtn,
    stopBtn,
    closeBtn,
    ...rowTiles,
    ...previewTiles
  ];
  for (const t of allTiles) group.add(t.mesh);

  // Вкладка VOICE: постоянная плашка на слое указателя (в VR клавиатуры
  // нет, поэтому точка входа в меню должна быть кликабельным объектом).
  const launchGroup = new THREE.Group();
  launchGroup.renderOrder = 20;
  const launchTile = createTile(0.34, 0.12, 256, 64);
  launchGroup.add(launchTile.mesh);
  function drawLaunch(open: boolean): void {
    fillPlate(launchTile, open ? COLOR_BG_ACTIVE : COLOR_BG, null);
    drawText(launchTile, open ? "▼ VOICE" : "▶ VOICE", {
      color: open ? COLOR_TEXT_INVERT : COLOR_ACCENT,
      font: "bold 24px monospace",
      align: "center"
    });
    launchTile.texture.needsUpdate = true;
  }
  drawLaunch(false);

  // Текущий снимок состояния — targets() отдаёт только активные цели.
  let visibleRows: ReturnType<typeof ttsRowViews> = [];
  let applyEnabled = false;
  let stopEnabled = false;
  let pagerVisible = false;
  let prevEnabled = false;
  let nextEnabled = false;

  /** Раскладка: header сверху, строки вниз, листание, кнопки и footer. */
  function layout(rowCount: number, showPager: boolean): void {
    let y = 0;
    header.mesh.position.set(0, y, 0);
    y -= HEADER_H / 2 + ROW_GAP;

    if (rowCount === 0) {
      // loading / empty — одна плашка-заглушка вместо строк.
      placeholder.mesh.visible = true;
      placeholder.mesh.position.set(0, y - ROW_H / 2, 0);
      y -= ROW_H + ROW_GAP;
    } else {
      placeholder.mesh.visible = false;
    }

    for (let i = 0; i < MAX_VISIBLE_ROWS; i++) {
      const shown = i < rowCount;
      rowTiles[i].mesh.visible = shown;
      previewTiles[i].mesh.visible = shown;
      if (!shown) continue;
      const rowY = y - ROW_H / 2;
      // Строка слева, кнопка PREVIEW справа — обе на одной высоте.
      rowTiles[i].mesh.position.set(-(MENU_W - rowW) / 2, rowY, 0);
      previewTiles[i].mesh.position.set((MENU_W - previewW) / 2, rowY, 0);
      y -= ROW_H + ROW_GAP;
    }

    // Строка листания — сразу под списком: она относится к нему, а не к
    // кнопкам применения.
    pagerPrev.mesh.visible = showPager;
    pagerNext.mesh.visible = showPager;
    pagerLabel.mesh.visible = showPager;
    if (showPager) {
      const pagerY = y - PAGER_H / 2;
      const arrowW = MENU_W * PAGER_ARROW_FRAC;
      pagerPrev.mesh.position.set(-(MENU_W - arrowW) / 2, pagerY, 0);
      pagerNext.mesh.position.set((MENU_W - arrowW) / 2, pagerY, 0);
      pagerLabel.mesh.position.set(0, pagerY, 0);
      y -= PAGER_H + ROW_GAP;
    }

    const btnY = y - BUTTON_H / 2;
    applyBtn.mesh.position.set(-(buttonW + ROW_GAP), btnY, 0);
    stopBtn.mesh.position.set(0, btnY, 0);
    closeBtn.mesh.position.set(buttonW + ROW_GAP, btnY, 0);
    y -= BUTTON_H + ROW_GAP;

    footer.mesh.position.set(0, y - FOOTER_H / 2, 0);
  }

  function drawHeader(state: TtsPickerState): void {
    fillPlate(header, COLOR_BG, COLOR_ACCENT);
    drawText(header, ttsHeaderText(state), {
      color: state.applyingVoiceId !== null ? COLOR_WARN : COLOR_TEXT,
      font: "bold 26px monospace"
    });
  }

  function drawPlaceholder(state: TtsPickerState): void {
    fillPlate(placeholder, COLOR_BG, COLOR_TEXT_DIM);
    if (state.phase === "empty") {
      // Явный честный текст из карточки — голоса НЕ выдумываем.
      drawText(placeholder, EMPTY_VOICES_TEXT, { color: COLOR_WARN, font: "bold 24px monospace" });
      return;
    }
    // loading — спиннер-заглушка в стиле stream_menu (текст, не анимация:
    // перерисовка текстуры на каждый кадр стоит дороже, чем польза).
    drawText(placeholder, "◐ loading voices…", { color: COLOR_TEXT_DIM, font: "bold 24px monospace" });
  }

  function drawRow(tile: Tile, row: ReturnType<typeof ttsRowViews>[number], locked: boolean): void {
    const bg = row.current ? COLOR_BG_ACTIVE : row.selected ? COLOR_BG_SELECTED : locked ? COLOR_BG_DISABLED : COLOR_BG;
    fillPlate(tile, bg, row.current ? COLOR_TEXT_INVERT : COLOR_ACCENT);
    const titleColor = row.current ? COLOR_TEXT_INVERT : COLOR_TEXT;
    const subColor = row.current ? COLOR_TEXT_INVERT : COLOR_TEXT_DIM;
    drawText(tile, row.title, { color: titleColor, font: "bold 26px monospace", y: tile.height * 0.36 });
    if (row.subtitle) {
      drawText(tile, row.subtitle, { color: subColor, font: "20px monospace", y: tile.height * 0.72 });
    }
    if (row.current) {
      drawText(tile, "ACTIVE", {
        color: COLOR_TEXT_INVERT,
        font: "bold 18px monospace",
        align: "right",
        x: tile.width - 14
      });
    }
  }

  function drawPreviewButton(tile: Tile, previewing: boolean, locked: boolean): void {
    fillPlate(tile, previewing ? COLOR_BG_ACTIVE : locked ? COLOR_BG_DISABLED : COLOR_BG, null);
    drawText(tile, previewing ? "▮ PLAY" : "▶ PREVIEW", {
      color: previewing ? COLOR_TEXT_INVERT : locked ? COLOR_TEXT_DIM : COLOR_ACCENT,
      font: "bold 22px monospace",
      align: "center"
    });
  }

  function drawButton(tile: Tile, label: string, enabled: boolean, hot = false): void {
    fillPlate(tile, hot ? COLOR_BG_ACTIVE : enabled ? COLOR_BG : COLOR_BG_DISABLED, null);
    drawText(tile, label, {
      color: hot ? COLOR_TEXT_INVERT : enabled ? COLOR_ACCENT : COLOR_TEXT_DIM,
      font: "bold 24px monospace",
      align: "center"
    });
  }

  /**
   * Строка листания. Рисуется только когда голосов больше страницы —
   * бесполезные стрелки на списке из трёх голосов только ловили бы луч.
   */
  function drawPager(info: TtsPageInfo, locked: boolean): void {
    const arrow = (tile: Tile, label: string, enabled: boolean) => {
      fillPlate(tile, enabled ? COLOR_BG : COLOR_BG_DISABLED, null);
      drawText(tile, label, {
        color: enabled ? COLOR_ACCENT : COLOR_TEXT_DIM,
        font: "bold 26px monospace",
        align: "center"
      });
    };
    arrow(pagerPrev, "◀", info.hasPrev && !locked);
    arrow(pagerNext, "▶", info.hasNext && !locked);
    fillPlate(pagerLabel, COLOR_BG, null);
    drawText(pagerLabel, `${info.from}–${info.to} из ${info.total}`, {
      color: COLOR_TEXT_DIM,
      font: "bold 22px monospace",
      align: "center"
    });
  }

  function drawFooter(state: TtsPickerState): void {
    const { text, level } = ttsFooterText(state);
    fillPlate(footer, COLOR_BG, level === "bad" ? COLOR_BAD : level === "warn" ? COLOR_WARN : COLOR_ACCENT);
    drawText(footer, text, {
      color: level === "bad" ? COLOR_BAD : level === "warn" ? COLOR_WARN : COLOR_TEXT_DIM,
      font: "22px monospace"
    });
  }

  function render(state: TtsPickerState): void {
    const rows = ttsRowViews(state).slice(0, MAX_VISIBLE_ROWS);
    visibleRows = rows;
    // Во время apply UI залочен: строки/preview гасим, чтобы оператор не
    // отправил вторую команду до ответа сервера.
    const locked = state.applyingVoiceId !== null;
    applyEnabled = canApply(state);
    stopEnabled = canStopPreview(state);

    const pageInfo = ttsPageInfo(state);
    pagerVisible = state.phase === "ready" && pageInfo.pages > 1;
    prevEnabled = pagerVisible && pageInfo.hasPrev && !locked;
    nextEnabled = pagerVisible && pageInfo.hasNext && !locked;

    layout(rows.length, pagerVisible);
    if (pagerVisible) drawPager(pageInfo, locked);
    drawHeader(state);
    if (rows.length === 0) drawPlaceholder(state);
    for (let i = 0; i < rows.length; i++) {
      drawRow(rowTiles[i], rows[i], locked);
      drawPreviewButton(previewTiles[i], rows[i].previewing, locked);
    }
    drawButton(applyBtn, "APPLY", applyEnabled);
    drawButton(stopBtn, "STOP", stopEnabled, stopEnabled);
    drawButton(closeBtn, "CLOSE", true);
    drawFooter(state);
    for (const t of allTiles) t.texture.needsUpdate = true;
  }

  function targets(): Array<{ id: string; object: THREE.Object3D }> {
    const out: Array<{ id: string; object: THREE.Object3D }> = [];
    if (!group.visible) return out;
    for (let i = 0; i < visibleRows.length; i++) {
      out.push({ id: selectTargetId(visibleRows[i].voiceId), object: rowTiles[i].mesh });
      out.push({ id: previewTargetId(visibleRows[i].voiceId), object: previewTiles[i].mesh });
    }
    // APPLY/STOP регистрируем только когда они реально что-то делают:
    // мёртвая кнопка, которая ловит луч, — обман оператора.
    // Стрелки регистрируем только пока они реально листают: мёртвая
    // стрелка, ловящая луч, — тот же обман, что мёртвый APPLY.
    if (prevEnabled) out.push({ id: PREV_PAGE_TARGET_ID, object: pagerPrev.mesh });
    if (nextEnabled) out.push({ id: NEXT_PAGE_TARGET_ID, object: pagerNext.mesh });
    if (applyEnabled) out.push({ id: APPLY_TARGET_ID, object: applyBtn.mesh });
    if (stopEnabled) out.push({ id: STOP_TARGET_ID, object: stopBtn.mesh });
    out.push({ id: CLOSE_TARGET_ID, object: closeBtn.mesh });
    return out;
  }

  return {
    object: group,
    launchObject: launchGroup,
    launchTarget(): { id: string; object: THREE.Object3D } {
      return { id: LAUNCH_TARGET_ID, object: launchTile.mesh };
    },
    targets,
    render,
    show(position: THREE.Vector3, facingAngleY: number): void {
      // Меню встаёт над точкой привязки и повёрнуто так же, как панель —
      // ровно как stream_menu.show (тот же слой глубины).
      group.position.set(position.x, position.y + 0.5, position.z);
      group.rotation.y = facingAngleY;
      group.visible = true;
      drawLaunch(true);
    },
    hide(): void {
      group.visible = false;
      drawLaunch(false);
    },
    isVisible(): boolean {
      return group.visible;
    },
    dispose(): void {
      for (const t of [...allTiles, launchTile]) {
        t.mesh.geometry.dispose();
        (t.mesh.material as THREE.Material).dispose();
        t.texture.dispose();
      }
    }
  };
}
