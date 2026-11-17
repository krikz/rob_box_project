// Консоль ТАРС 1 в обе стороны (issue #3253, Ш4): виды строк и буфер.
//
// Чистая логика без canvas/DOM: панель рисует то, что лежит в буфере, и берёт
// префикс/цвет из CONSOLE_STYLES. Три вида строк:
//   tars     — реплики ТАРС (`> `, зелёный), приходят кусками, печатаются
//              typewriter'ом; чанк без \n дописывается в открытую строку;
//   operator — распознанная фраза оператора (`you> `, циан), появляется сразу;
//   event    — короткое служебное событие (`· tool: …`, янтарный);
//   reply    — ПОЛНЫЙ текст ответа ТАРС (`≡ `, белый), многострочный: вслух
//              говорится коротко, весь текст — на экране (#3296).
//
// Источник operator/event — серверное событие `tars_console`
// (quest_node: /avatar/stt/result и /avatar/command_result, core/tars_console.py).

export type ConsoleLineKind = "tars" | "operator" | "event" | "reply";

export interface ConsoleLine {
  kind: ConsoleLineKind;
  text: string;
  /** Продолжение многострочного блока: без префикса, с отступом (#3296). */
  cont?: boolean;
}

/** Сколько строк ответа показываем; остальное — видимый маркер «… ещё N». */
export const REPLY_MAX_LINES = 14;

export interface ConsoleStyle {
  prefix: string;
  prefixColor: string;
  textColor: string;
}

export const CONSOLE_STYLES: Record<ConsoleLineKind, ConsoleStyle> = {
  tars: { prefix: "> ", prefixColor: "#1f9c55", textColor: "#39ff88" },
  operator: { prefix: "you> ", prefixColor: "#1b8ea6", textColor: "#33e0ff" },
  event: { prefix: "· ", prefixColor: "#8a6a1f", textColor: "#ffc94d" },
  reply: { prefix: "≡ ", prefixColor: "#7a8a8a", textColor: "#e6f2f2" }
};

/**
 * Markdown ответа → простой текст для консоли: убираем `**`, `__`, `` ` ``,
 * заголовки `#`; переводы строк и маркеры списков остаются.
 */
export function plainReplyText(text: string): string[] {
  const lines = text
    .replace(/\r\n?/g, "\n")
    .split("\n")
    .map((l) =>
      l
        .replace(/\*\*|__|`/g, "")
        .replace(/^\s{0,3}#{1,6}\s+/, "")
        .replace(/^(\s*)[-*]\s+/, "$1• ")
        .replace(/[ \t]+/g, " ")
        .trimEnd()
    );
  // Схлопываем повторные пустые строки и срезаем пустые края.
  const out: string[] = [];
  for (const l of lines) {
    if (l === "" && (out.length === 0 || out[out.length - 1] === "")) continue;
    out.push(l);
  }
  while (out.length > 0 && out[out.length - 1] === "") out.pop();
  if (out.length <= REPLY_MAX_LINES) return out;
  const hidden = out.length - REPLY_MAX_LINES;
  return [...out.slice(0, REPLY_MAX_LINES), `… ещё ${hidden} стр. (полный текст — в логе супервизора)`];
}

/** Событие моста `tars_console` → строка консоли; чужое/битое → null. */
export function parseConsoleEvent(event: unknown): ConsoleLine | null {
  if (typeof event !== "object" || event === null) return null;
  const e = event as Record<string, unknown>;
  if (e.type !== "tars_console") return null;
  if (e.kind !== "operator" && e.kind !== "event" && e.kind !== "reply") return null;
  if (typeof e.text !== "string") return null;
  if (e.kind === "reply") {
    const lines = plainReplyText(e.text);
    return lines.length > 0 ? { kind: "reply", text: lines.join("\n") } : null;
  }
  const text = e.text.replace(/\s+/g, " ").trim();
  if (!text) return null;
  return { kind: e.kind, text };
}

/**
 * Кольцевой буфер строк консоли. Реплики ТАРС приходят потоком (`appendTars`),
 * фразы оператора и события — целыми строками (`pushLine`).
 */
export class ConsoleBuffer {
  private rows: ConsoleLine[] = [{ kind: "tars", text: "" }];
  // Последняя строка — «открытая» реплика ТАРС, в которую дописывается поток.
  private open = true;

  constructor(private readonly maxLines: number) {}

  get lines(): readonly ConsoleLine[] {
    return this.rows;
  }

  get length(): number {
    return this.rows.length;
  }

  appendTars(text: string): void {
    if (!text) return;
    const parts = text.split("\n");
    parts.forEach((part, i) => {
      if (i > 0) this.open = false;
      if (part) this.writeTars(part);
    });
    this.trim();
  }

  pushLine(kind: ConsoleLineKind, text: string): void {
    if (!text) return;
    const last = this.rows[this.rows.length - 1];
    // Пустая «стартовая» строка не должна оставаться дыркой перед первой фразой.
    if (this.open && last && last.kind === "tars" && last.text === "") this.rows.pop();
    this.open = false;
    this.rows.push({ kind, text });
    this.trim();
  }

  /** Многострочный блок (reply): первая строка с префиксом, остальные — отступом. */
  pushBlock(kind: ConsoleLineKind, text: string): void {
    text.split("\n").forEach((line, i) => {
      if (i === 0) {
        this.pushLine(kind, line || " ");
      } else {
        this.rows.push({ kind, text: line, cont: true });
      }
    });
    this.trim();
  }

  clear(): void {
    this.rows = [{ kind: "tars", text: "" }];
    this.open = true;
  }

  private writeTars(part: string): void {
    const last = this.rows[this.rows.length - 1];
    if (this.open && last && last.kind === "tars") {
      last.text += part;
    } else {
      this.rows.push({ kind: "tars", text: part });
    }
    this.open = true;
  }

  private trim(): void {
    while (this.rows.length > this.maxLines) this.rows.shift();
  }
}
