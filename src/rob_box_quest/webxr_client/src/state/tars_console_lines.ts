// Консоль ТАРС 1 в обе стороны (issue #3253, Ш4): виды строк и буфер.
//
// Чистая логика без canvas/DOM: панель рисует то, что лежит в буфере, и берёт
// префикс/цвет из CONSOLE_STYLES. Три вида строк:
//   tars     — реплики ТАРС (`> `, зелёный), приходят кусками, печатаются
//              typewriter'ом; чанк без \n дописывается в открытую строку;
//   operator — распознанная фраза оператора (`you> `, циан), появляется сразу;
//   event    — короткое служебное событие (`· tool: …`, янтарный).
//
// Источник operator/event — серверное событие `tars_console`
// (quest_node: /avatar/stt/result и /avatar/command_result, core/tars_console.py).

export type ConsoleLineKind = "tars" | "operator" | "event";

export interface ConsoleLine {
  kind: ConsoleLineKind;
  text: string;
}

export interface ConsoleStyle {
  prefix: string;
  prefixColor: string;
  textColor: string;
}

export const CONSOLE_STYLES: Record<ConsoleLineKind, ConsoleStyle> = {
  tars: { prefix: "> ", prefixColor: "#1f9c55", textColor: "#39ff88" },
  operator: { prefix: "you> ", prefixColor: "#1b8ea6", textColor: "#33e0ff" },
  event: { prefix: "· ", prefixColor: "#8a6a1f", textColor: "#ffc94d" }
};

/** Событие моста `tars_console` → строка консоли; чужое/битое → null. */
export function parseConsoleEvent(event: unknown): ConsoleLine | null {
  if (typeof event !== "object" || event === null) return null;
  const e = event as Record<string, unknown>;
  if (e.type !== "tars_console") return null;
  if (e.kind !== "operator" && e.kind !== "event") return null;
  if (typeof e.text !== "string") return null;
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
