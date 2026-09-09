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

export interface Tars1TextPanelOptions {
  /** Ширина canvas в пикселях (default 1280 — 16:9 как у основного экрана). */
  canvasWidth?: number;
  /** Высота canvas в пикселях (default 720 — 16:9). */
  canvasHeight?: number;
  /** Сколько последних строк хранить в кольцевом буфере (default 32). */
  maxLines?: number;
  /** Размер шрифта в пикселях (default 22). */
  fontSize?: number;
}

/**
 * Handle для интеграции со сценой. `mesh` добавляется в THREE.Scene,
 * `dispose()` снимает ресурсы при tear-down.
 */
export interface Tars1TextPanelHandle {
  readonly mesh: THREE.Mesh;
  append(text: string): void;
  clear(): void;
  setStreaming(streaming: boolean): void;
  getStats(): { lineCount: number; streaming: boolean };
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
  const lines: string[] = [""];
  // Частичный буфер для незакрытых переводом строки чанков: append может
  // прийти как кусок «Привет, |как дела?», и до \n мы держим хвост здесь.
  let partial = "";
  let streaming = false;

  function render(): void {
    // Подложка (тёмная «sci-fi» палитра — в Captain Bridge это ночная смена).
    ctx!.fillStyle = "#0a0d11";
    ctx!.fillRect(0, 0, canvasWidth, canvasHeight);

    // Рамка-индикатор стрима: сверху, тонкая полоса.
    if (streaming) {
      ctx!.fillStyle = "#2ec27e";
      ctx!.fillRect(0, 0, canvasWidth, 4);
    } else {
      ctx!.fillStyle = "#444a52";
      ctx!.fillRect(0, 0, canvasWidth, 4);
    }

    // Заголовок канала.
    ctx!.fillStyle = "#8fd4ff";
    ctx!.font = `bold ${Math.round(fontSize * 0.7)}px monospace`;
    ctx!.textBaseline = "top";
    ctx!.fillText("TARS 1 ▸ ", 8, 8);

    // Основной текст — monospace, белый, перенос по строкам буфера.
    ctx!.fillStyle = "#e6edf3";
    ctx!.font = `${fontSize}px monospace`;
    const lineHeight = Math.round(fontSize * 1.25);
    const startY = 8 + Math.round(fontSize * 1.0);
    const maxWidth = canvasWidth - 16;
    // visibleLines хранит уже разбитые по wrap'у строки (для длинных
    // реплик LLM, не влезающих в ширину канвы).
    const visibleLines: string[] = [];
    for (const raw of lines) {
      visibleLines.push(...wrapLine(raw, ctx!, maxWidth));
    }
    // Рисуем только хвост, который помещается: самая свежая строка снизу.
    const capacity = Math.floor((canvasHeight - startY - 8) / lineHeight);
    const tail = visibleLines.slice(-capacity);
    for (let i = 0; i < tail.length; i += 1) {
      ctx!.fillText(tail[i] ?? "", 8, startY + i * lineHeight);
    }

    texture.needsUpdate = true;
  }

  function append(text: string): void {
    if (!text) return;
    partial += text;
    // Разделяем по \n: всё, что до последнего \n, идёт в буфер строк,
    // хвост после последнего \n остаётся в `partial` для следующего чанка.
    const parts = partial.split("\n");
    partial = parts.pop() ?? "";
    for (const p of parts) {
      lines.push(p);
    }
    // Текущая «незавершённая» строка рисуется как последний элемент lines:
    // для этого мы НЕ пушим partial до перевода строки — но тогда оператор
    // не видит стримящийся текст. Решение: держим отдельную «активную»
    // строку как последний элемент lines, обновляем её на каждый append.
    // Для этого выносим partial в lines[-1], не дожидаясь \n.
    if (lines.length === 0) lines.push("");
    // Убираем «виртуальную» пустую строку из capacity-расчёта.
    // Если предыдущая строка была пустой (initial) — заменяем её, иначе
    // добавляем.
    if (lines.length === 1 && lines[0] === "" && partial) {
      lines[0] = partial;
      partial = "";
    } else if (partial) {
      lines[lines.length - 1] = partial;
    }
    // Обрезаем старые строки (кольцевой буфер).
    while (lines.length > maxLines) lines.shift();
    render();
  }

  function clear(): void {
    lines.length = 0;
    lines.push("");
    partial = "";
    streaming = false;
    render();
  }

  function setStreaming(value: boolean): void {
    if (streaming === value) return;
    streaming = value;
    render();
  }

  function getStats(): { lineCount: number; streaming: boolean } {
    return { lineCount: lines.length, streaming };
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
    clear,
    setStreaming,
    getStats,
    dispose
  };
}

// ───────────────────────── helpers ─────────────────────────

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