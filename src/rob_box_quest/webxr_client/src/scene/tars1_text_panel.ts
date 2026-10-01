// TARS 1 text panel — Captain Bridge (issue #2113, quest #2112).
//
// Side panel по бокам от FRONT CAM, лицом к оператору. На нём TARS показывает
// то, что говорит (speech-to-text echo / LLM response) — параллельно TTS,
// идущему в /avatar/tts/audio. Источник — топик `/tars1/text`, на который
// зеркалит tts_node при приходе `/avatar/tts/request` (mirror-only, чтобы
// не было «TARS говорит одно, экран показывает другое»).
//
// Архитектура (как VideoPanel, но без JPEG):
//   PlaneGeometry + CanvasTexture. Текст рендерится в `<canvas>` шрифтом
//   monospace, dark theme, с авто-прокруткой (кольцевой буфер последних
//   N строк). При приходе новой порции старые строки уходят вверх.
//
// API (минимальный, под wire):
//   append(text: string)        — дописать текст (стрим: чанки могут быть
//                                  дробными; склеиваем в линии по \n)
//   clear()                      — стереть
//   setStreaming(true|false)    — визуальный индикатор «TARS ещё говорит»
//
// Компонент чист от Three.js-DOM-логики высокого уровня: он не знает про
// WSS, топики, ROS — это ответственность main.ts (та же модель, что у
// остальных UI-панелей, см. captain_bridge.ts). Здесь — только геометрия,
// canvas-рендер и базовые стили.

import * as THREE from "three";
import { tarsActivityView, type TarsActivity } from "../state/tars_activity";
import { TARS_SEGMENTS, tarsFigurePose } from "../state/tars_figure";
import {
  CONSOLE_STYLES,
  ConsoleBuffer,
  type ConsoleLineKind
} from "../state/tars_console_lines";
import {
  INFO_DASH,
  buildInfoRows,
  computeTars1Layout,
  type InfoRow,
  type Rect,
  type Tars1Info
} from "../state/tars1_layout";

export interface Tars1TextPanelOptions {
  /** Ширина canvas в пикселях (default 1280 — 16:9 как у основного экрана). */
  canvasWidth?: number;
  /** Высота canvas в пикселях (default 720 — 16:9). */
  canvasHeight?: number;
  /** Сколько последних строк хранить в кольцевом буфере (default 32). */
  maxLines?: number;
  /** Размер шрифта в пикселях (default 22). */
  fontSize?: number;
  /**
   * Скорость «печати» новых реплик, символов/с. 0 (default) — без эффекта,
   * append применяется сразу (так работают юнит-тесты). Мост включает ~90.
   */
  typewriterCps?: number;
}

/**
 * Handle для интеграции со сценой. `mesh` добавляется в THREE.Scene,
 * `dispose()` снимает ресурсы при tear-down.
 */
export interface Tars1TextPanelHandle {
  readonly mesh: THREE.Mesh;
  append(text: string): void;
  /**
   * Целая строка консоли с видом (#3253 Ш4): `operator` (фраза оператора,
   * `you> `, циан) или `event` (короткое событие). Появляется сразу, без
   * typewriter; недопечатанный хвост реплики ТАРС дописывается мгновенно,
   * чтобы порядок строк не перепутался. kind="tars" == append(text).
   */
  appendLine(kind: ConsoleLineKind, text: string): void;
  clear(): void;
  setStreaming(streaming: boolean): void;
  /** Состояние ТАРС для строки статуса на консоли (СЛУШАЕТ/ДУМАЕТ/ГОВОРИТ). */
  setActivity(activity: TarsActivity): void;
  /**
   * Служебная информация для правой колонки рамки (связь, floor, PTT, …).
   * Незаданные/пустые поля рисуются прочерком «—». Перерисовка — по общему
   * бюджету tick(), не на каждый вызов.
   */
  setInfo(info: Tars1Info): void;
  /**
   * Шаг анимации (печать, курсор, статус). Зовётся каждый кадр; canvas
   * перерисовывается только при изменении: 4 Гц в покое («дыхание»), ~11 Гц (≤ 12) пока ТАРС
   * не idle или печатает.
   */
  tick(nowMs: number): void;
  getStats(): { lineCount: number; streaming: boolean; pending: number };
  dispose(): void;
}

/**
 * Создаёт панель TARS 1 — текстовое полотно с прокруткой.
 *
 * Корень компонента — `THREE.Mesh` (Plane + MeshBasicMaterial с map=canvas).
 * Mesh лежит на нулевой позиции; перенос в сцену делает вызывающий код
 * (см. captain_bridge.ts: `scene.add(tars1Panel.mesh)`).
 */
export function createTars1TextPanel(
  opts: Tars1TextPanelOptions = {}
): Tars1TextPanelHandle {
  const canvasWidth = opts.canvasWidth ?? 1280;
  const canvasHeight = opts.canvasHeight ?? 720;
  const maxLines = opts.maxLines ?? 32;
  const fontSize = opts.fontSize ?? 22;
  const typewriterCps = opts.typewriterCps ?? 0;

  const canvas = document.createElement("canvas");
  canvas.width = canvasWidth;
  canvas.height = canvasHeight;
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    throw new Error("Tars1TextPanel: failed to acquire 2D context");
  }

  const texture = new THREE.CanvasTexture(canvas);
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;
  texture.colorSpace = THREE.SRGBColorSpace;

  // Соотношение сторон canvas и плоскости — 16:9 (ADR-0074 §4.0, вариант E:
  // все три экрана Captain Bridge должны быть одного aspect ratio, как
  // основной 4.8 × 2.7). Геометрия — ЕДИНИЧНЫЙ план (1×1): фактический
  // размер в метрах задаётся снаружи через mesh.scale.set(width, height, 1)
  // (см. captain_bridge.ts: TARS_PANEL_SIZE). ВАЖНО: если тут поставить
  // не-единичный размер (было 1.6×0.9 до bugfix #2142-B), итоговый мировой
  // размер меша станет geometry-size × scale, а не scale — двойное
  // масштабирование. Ровно это раздувало панель до 4.8×1.52 м вместо
  // заявленных 3.0×1.69 м и гнало её в главный экран (issue #2142-B,
  // раскопано nightly-review-fix: bug существовал с самого fa5854fd, стал
  // заметнее после ресайза W=1.6→3.0 в e17e5bca). Aspect ratio 16:9 теперь
  // держит сам TARS_PANEL_SIZE (width, width*9/16) в captain_bridge.ts —
  // геометрии он не касается.
  const geometry = new THREE.PlaneGeometry(1, 1);
  const material = new THREE.MeshBasicMaterial({
    map: texture,
    side: THREE.DoubleSide,
    toneMapped: false
  });
  const mesh = new THREE.Mesh(geometry, material);

  // Кольцевой буфер строк. Никогда не пустой — минимум одна пустая строка,
  // чтобы setStreaming индикатор рисовался корректно даже до первого
  // append.
  const buffer = new ConsoleBuffer(maxLines);
  let streaming = false;

  // Хакерская палитра: зелёный/циановый фосфор на почти чёрном.
  const BG = "#020a07";
  const FG = "#39ff88";
  const FG_DIM = "#1f9c55";
  const CYAN = "#33e0ff";

  const layout = computeTars1Layout(canvasWidth, canvasHeight);
  let info: Tars1Info = {};
  let infoKey = "{}";
  let activity: TarsActivity = "idle";
  let lastNowMs = 0;
  let lastFrameKey = "";
  let lastTickMs = 0;
  // Хвост печати: символы, ещё не «выведенные» на консоль.
  let pending = "";

  function render(nowMs: number = lastNowMs): void {
    const c = ctx!;
    c.fillStyle = BG;
    c.fillRect(0, 0, canvasWidth, canvasHeight);

    const view = tarsActivityView(activity, nowMs);
    const small = Math.round(fontSize * 0.7);
    const pad = layout.console.x;

    // ── Рамка `╭─ TARS ─╮` ────────────────────────────────────────────
    const f = layout.frame;
    const T = 2; // толщина линии, px
    const R = 16; // радиус скругления
    const titleFont = `bold ${Math.round(small * 1.15)}px monospace`;
    c.font = titleFont;
    c.textBaseline = "top";
    const title = " TARS ";
    const titleW = c.measureText(title).width;
    const titleX = f.x + R + 14;
    c.fillStyle = FG_DIM;
    c.globalAlpha = 0.75;
    // верхняя линия — с разрывом под заголовок
    c.fillRect(f.x + R, f.y, titleX - f.x - R, T);
    c.fillRect(titleX + titleW, f.y, f.x + f.w - R - titleX - titleW, T);
    c.fillRect(f.x + R, f.y + f.h - T, f.w - R * 2, T);
    c.fillRect(f.x, f.y + R, T, f.h - R * 2);
    c.fillRect(f.x + f.w - T, f.y + R, T, f.h - R * 2);
    drawCorner(c, f.x, f.y, R, T, 1, 1);
    drawCorner(c, f.x + f.w, f.y, R, T, -1, 1);
    drawCorner(c, f.x, f.y + f.h, R, T, 1, -1);
    drawCorner(c, f.x + f.w, f.y + f.h, R, T, -1, -1);
    // вертикальная линия между колонками
    c.fillRect(layout.dividerX, f.y + T, 1, f.h - T * 2);
    // горизонтальная линия между «служебное» и «контекст»
    c.fillRect(layout.right.x - 4, layout.rightDividerY, layout.right.w + 8, 1);
    c.globalAlpha = 1;
    c.fillStyle = CYAN;
    c.shadowColor = CYAN;
    c.shadowBlur = 6;
    c.fillText(title, titleX, f.y - Math.round(small * 0.6));
    c.shadowBlur = 0;

    // ── Левая колонка: персонаж + подпись состояния ──────────────────
    const L = layout.left;
    const labelFont = Math.round(small * 1.3);
    const labelH = labelFont + 10;
    const figH = Math.round((L.h - labelH - 6) * 0.8);
    const segW = Math.max(6, Math.round(Math.min(L.w / 9, figH / 4)));
    const gap = Math.round(segW * 0.55);
    const totalW = TARS_SEGMENTS * segW + (TARS_SEGMENTS - 1) * gap;
    const figX = L.x + Math.round((L.w - totalW) / 2);
    const figCy = L.y + Math.round((L.h - labelH - 6) / 2);
    const pose = tarsFigurePose(activity, nowMs);
    c.fillStyle = view.color;
    c.shadowColor = view.color;
    for (let i = 0; i < pose.length; i += 1) {
      const s = pose[i];
      const sh = Math.max(4, Math.round(figH * s.h));
      const sx = Math.round(figX + i * (segW + gap) + s.dx * segW);
      const sy = Math.round(figCy - sh / 2 + s.dy * figH);
      c.globalAlpha = 0.2 + 0.8 * s.glow;
      c.shadowBlur = 4 + Math.round(12 * s.glow);
      c.fillRect(sx, sy, segW, sh);
    }
    c.shadowBlur = 0;
    c.globalAlpha = 1;
    // Подпись состояния цветом из tarsActivityView.
    const badge = `${view.label}${view.glyph ? " " + view.glyph : ""}`;
    c.font = `bold ${labelFont}px monospace`;
    const badgeW = c.measureText(badge).width;
    c.fillStyle = view.color;
    c.globalAlpha = view.pulse;
    c.shadowColor = view.color;
    c.shadowBlur = 8;
    c.fillText(badge, L.x + Math.round((L.w - badgeW) / 2), L.y + L.h - labelFont - 2);
    c.shadowBlur = 0;
    c.globalAlpha = 1;

    // ── Правая колонка: служебное / контекст ────────────────────────
    const rows = buildInfoRows(info);
    const rowFont = Math.round(small * 1.05);
    const rowH = rowFont + 5;
    const drawBlock = (r: Rect, head: string, items: InfoRow[]): void => {
      c.font = `bold ${rowFont}px monospace`;
      c.fillStyle = CYAN;
      c.fillText(head, r.x, r.y);
      c.font = `${rowFont}px monospace`;
      const labelW = c.measureText("реплика ").width + 6;
      for (let i = 0; i < items.length; i += 1) {
        const y = r.y + (i + 1) * rowH + 2;
        if (y + rowFont > r.y + r.h + 2) break;
        c.fillStyle = FG_DIM;
        c.fillText(items[i].label, r.x, y);
        const dash = items[i].value === INFO_DASH;
        c.fillStyle = dash ? FG_DIM : FG;
        c.globalAlpha = dash ? 0.6 : 1;
        c.fillText(items[i].value, r.x + labelW, y);
        c.globalAlpha = 1;
      }
    };
    drawBlock(layout.service, "СЛУЖЕБНОЕ", rows.service);
    drawBlock(layout.context, "КОНТЕКСТ", rows.context);

    // ── Консоль: нижняя часть canvas, под рамкой ─────────────────────
    const cons = layout.console;
    c.font = `${fontSize}px monospace`;
    const lineHeight = Math.round(fontSize * 1.25);
    const startY = cons.y + 4;
    // Каждая логическая строка → wrap; первая визуальная строка получает
    // префикс своего вида («> » / «you> » / «· »), продолжения — отступ.
    const crows: { text: string; first: boolean; kind: ConsoleLineKind; prefixW: number }[] = [];
    for (const row of buffer.lines) {
      const style = CONSOLE_STYLES[row.kind];
      const pw = c.measureText(style.prefix).width;
      const wrapped = wrapLine(row.text, c, cons.w - pw);
      wrapped.forEach((t, k) =>
        crows.push({ text: t, first: k === 0, kind: row.kind, prefixW: pw })
      );
    }
    const capacity = Math.max(1, Math.floor((cons.y + cons.h - startY) / lineHeight));
    const tail = crows.slice(-capacity);
    c.shadowBlur = 4;
    for (let i = 0; i < tail.length; i += 1) {
      const y = startY + i * lineHeight;
      const style = CONSOLE_STYLES[tail[i].kind];
      c.shadowColor = style.textColor;
      if (tail[i].first) {
        c.fillStyle = style.prefixColor;
        c.fillText(style.prefix, pad, y);
      }
      c.fillStyle = style.textColor;
      c.fillText(tail[i].text, pad + tail[i].prefixW, y);
    }
    c.shadowBlur = 0;

    // Курсор: за последней строкой; при печати горит постоянно, иначе мигает.
    const typing = pending.length > 0;
    const blinkOn = typing || Math.floor(nowMs / 530) % 2 === 0;
    if (blinkOn) {
      const lastRow = tail.length > 0 ? tail[tail.length - 1] : null;
      const cy = startY + Math.max(0, tail.length - 1) * lineHeight;
      const cx =
        pad +
        (lastRow ? lastRow.prefixW : c.measureText(CONSOLE_STYLES.tars.prefix).width) +
        c.measureText(lastRow ? lastRow.text : "").width +
        2;
      c.fillStyle = FG;
      c.fillRect(cx, cy + 2, Math.round(fontSize * 0.55), lineHeight - 6);
    }

    // Сканлайны: тонкие тёмные полосы через 4 px — дёшево (~180 rect).
    c.fillStyle = "rgba(0,0,0,0.22)";
    for (let y = 1; y < canvasHeight; y += 4) c.fillRect(0, y, canvasWidth, 1);

    texture.needsUpdate = true;
  }

  function tick(nowMs: number): void {
    lastNowMs = nowMs;
    const dt = lastTickMs > 0 ? Math.max(0, nowMs - lastTickMs) : 0;
    lastTickMs = nowMs;
    let dirty = false;
    if (pending.length > 0) {
      // Догоняем при большом хвосте, чтобы длинная реплика не печаталась минуту.
      const boost = 1 + pending.length / 200;
      const n = Math.max(1, Math.round((typewriterCps * dt * boost) / 1000));
      commit(pending.slice(0, n));
      pending = pending.slice(n);
      dirty = true;
    }
    // Кадр анимации: ~11 Гц (≤ 12) пока что-то живое, иначе 4 Гц («дыхание» + курсор).
    const busy = activity !== "idle" || pending.length > 0 || streaming;
    const key = busy ? `f${Math.floor(nowMs / 90)}` : `i${Math.floor(nowMs / 250)}`;
    if (dirty || key !== lastFrameKey) {
      lastFrameKey = key;
      render(nowMs);
    }
  }

  function setActivity(next: TarsActivity): void {
    if (next === activity) return;
    activity = next;
    lastFrameKey = "";
    render();
  }

  function setInfo(next: Tars1Info): void {
    // Без перерисовки: данные подхватит ближайший кадр tick().
    const k = JSON.stringify(next);
    if (k === infoKey) return;
    infoKey = k;
    info = { ...next };
  }

  function append(text: string): void {
    if (!text) return;
    if (typewriterCps > 0) {
      pending += text;
      return;
    }
    commit(text);
  }

  function commit(text: string): void {
    buffer.appendTars(text);
    if (typewriterCps <= 0) render();
  }

  function appendLine(kind: ConsoleLineKind, text: string): void {
    const clean = text.replace(/\s+/g, " ").trim();
    if (!clean) return;
    if (kind === "tars") {
      append(clean);
      return;
    }
    // Фраза оператора — сразу: недопечатанный хвост ТАРС дописываем мгновенно.
    if (pending.length > 0) {
      buffer.appendTars(pending);
      pending = "";
    }
    buffer.pushLine(kind, clean);
    render();
  }

  function clear(): void {
    buffer.clear();
    pending = "";
    streaming = false;
    render();
  }

  function setStreaming(value: boolean): void {
    if (streaming === value) return;
    streaming = value;
    render();
  }

  function getStats(): { lineCount: number; streaming: boolean; pending: number } {
    return { lineCount: buffer.length, streaming, pending: pending.length };
  }

  function dispose(): void {
    (mesh.geometry as THREE.BufferGeometry).dispose();
    (mesh.material as THREE.Material).dispose();
    texture.dispose();
  }

  // Первый рендер — пустая панель с заголовком.
  render();

  return {
    mesh,
    append,
    appendLine,
    clear,
    setStreaming,
    setActivity,
    setInfo,
    tick,
    getStats,
    dispose
  };
}

// ───────────────────────── helpers ─────────────────────────

/**
 * Скруглённый угол рамки из прямоугольников (только fillRect — без путей).
 * (cx, cy) — угол рамки, (sx, sy) — направление внутрь: ±1.
 */
function drawCorner(
  c: CanvasRenderingContext2D,
  cx: number,
  cy: number,
  r: number,
  t: number,
  sx: number,
  sy: number
): void {
  for (let i = 0; i < r; i += 1) {
    const a = r - Math.sqrt(r * r - (r - i) * (r - i));
    const b = r - Math.sqrt(r * r - (r - i - 1) * (r - i - 1));
    const h = Math.max(t, Math.ceil(b - a));
    const x = sx > 0 ? cx + i : cx - i - 1;
    const y = sy > 0 ? cy + a : cy - a - h;
    c.fillRect(x, y, 1, h);
  }
}

/**
 * Разбивает одну строку на массив строк, каждая из которых влезает в
 * `maxWidth` пикселей при текущем шрифте. Простая word-wrap: режем по
 * ближайшему пробелу, если слово длиннее maxWidth — режем посимвольно
 * (на случай URL или хеша).
 */
function wrapLine(line: string, ctx: CanvasRenderingContext2D, maxWidth: number): string[] {
  if (!line) return [""];
  const out: string[] = [];
  let current = "";
  // Сначала пробуем word-wrap по словам.
  const words = line.split(" ");
  for (const word of words) {
    const tentative = current ? `${current} ${word}` : word;
    if (ctx.measureText(tentative).width <= maxWidth) {
      current = tentative;
      continue;
    }
    // current не пуст — сбрасываем и пробуем поместить word целиком.
    if (current) {
      out.push(current);
      current = "";
    }
    if (ctx.measureText(word).width <= maxWidth) {
      current = word;
      continue;
    }
    // Слово длиннее maxWidth — режем посимвольно.
    let buf = "";
    for (const ch of word) {
      if (ctx.measureText(buf + ch).width > maxWidth && buf) {
        out.push(buf);
        buf = ch;
      } else {
        buf += ch;
      }
    }
    if (buf) current = buf;
  }
  if (current) out.push(current);
  return out.length > 0 ? out : [""];
}