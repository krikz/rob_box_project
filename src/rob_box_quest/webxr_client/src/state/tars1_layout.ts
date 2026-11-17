// Раскладка и тексты экрана ТАРС 1 (issue #3253, Ш1): рамка с двумя колонками
// сверху, консоль снизу. Чистые функции без canvas — тестируются без DOM.

export interface Rect {
  x: number;
  y: number;
  w: number;
  h: number;
}

export interface Tars1Layout {
  /** Внешняя рамка `╭─ TARS ─╮`. */
  frame: Rect;
  /** X вертикальной линии между колонками. */
  dividerX: number;
  /** Левая колонка: персонаж + подпись состояния. */
  left: Rect;
  /** Правая колонка целиком (служебное + контекст). */
  right: Rect;
  /** Y горизонтального разделителя между «СЛУЖЕБНОЕ» и «КОНТЕКСТ». */
  rightDividerY: number;
  service: Rect;
  context: Rect;
  /** Консоль под рамкой. */
  console: Rect;
}

/** Раскладка canvas w×h. Консоль всегда ниже рамки, колонки не пересекаются. */
export function computeTars1Layout(w: number, h: number): Tars1Layout {
  const pad = Math.round(w * 0.01);
  const inset = Math.round(w * 0.012);
  const frame: Rect = { x: pad, y: pad + 4, w: w - pad * 2, h: Math.round(h * 0.44) };
  const dividerX = frame.x + Math.round(frame.w * 0.36);
  const innerTop = frame.y + inset + 8;
  const innerH = frame.h - inset * 2 - 8;
  const left: Rect = { x: frame.x + inset, y: innerTop, w: dividerX - frame.x - inset * 2, h: innerH };
  const right: Rect = {
    x: dividerX + inset,
    y: innerTop,
    w: frame.x + frame.w - dividerX - inset * 2,
    h: innerH
  };
  // «Служебное» крупнее «контекста»: в нём 7 строк против 3.
  const rightDividerY = right.y + Math.round(right.h * 0.64);
  const service: Rect = { x: right.x, y: right.y, w: right.w, h: rightDividerY - right.y - 4 };
  const context: Rect = {
    x: right.x,
    y: rightDividerY + 4,
    w: right.w,
    h: right.y + right.h - rightDividerY - 4
  };
  const consoleY = frame.y + frame.h + Math.round(h * 0.015);
  const cons: Rect = { x: pad, y: consoleY, w: w - pad * 2, h: h - consoleY - pad };
  return { frame, dividerX, left, right, rightDividerY, service, context, console: cons };
}

/** Прочерк для полей, которых у клиента нет (ADR-0018: ничего не выдумываем). */
export const INFO_DASH = "—";

/** Что клиент знает про ТАРС; всё необязательное, отсутствие = прочерк. */
export interface Tars1Info {
  /** Состояние связи WSS (CONNECTED / RECONNECTING… / …). */
  link?: string | null;
  /** Кто держит floor: «teleop my · voice free». */
  floor?: string | null;
  /** Режим аватара из supervisor (mixed / voice_only / …). */
  mode?: string | null;
  /** PTT: none / radio / robot_voice. */
  ptt?: string | null;
  /** Время последней реплики ТАРС (чч:мм:сс). */
  lastReplyAt?: string | null;
  // Поля ниже в Ш1 клиент не получает (появятся в Ш3) → прочерк.
  llm?: string | null;
  tts?: string | null;
  wake?: string | null;
  topic?: string | null;
  nearby?: string | null;
}

export interface InfoRow {
  label: string;
  value: string;
}

export interface InfoRows {
  service: InfoRow[];
  context: InfoRow[];
}

const val = (v: string | null | undefined): string => (v && v.trim() ? v.trim() : INFO_DASH);

/** Строки правой колонки. Пустое/неизвестное значение → «—». */
export function buildInfoRows(info: Tars1Info): InfoRows {
  return {
    service: [
      { label: "связь", value: val(info.link) },
      { label: "floor", value: val(info.floor) },
      { label: "режим", value: val(info.mode) },
      { label: "ptt", value: val(info.ptt) },
      { label: "llm", value: val(info.llm) },
      { label: "tts", value: val(info.tts) },
      { label: "wake", value: val(info.wake) }
    ],
    context: [
      { label: "реплика", value: val(info.lastReplyAt) },
      { label: "тема", value: val(info.topic) },
      { label: "рядом", value: val(info.nearby) }
    ]
  };
}

/** Локальное время чч:мм:сс для «время последней реплики». */
export function formatClock(ms: number): string {
  const d = new Date(ms);
  const p = (n: number): string => String(n).padStart(2, "0");
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
}
