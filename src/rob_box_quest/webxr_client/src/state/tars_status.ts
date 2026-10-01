// Разбор события tars_status (issue #3253, Ш3): служебная информация ТАРС 1
// от quest_node. Чистая функция — тестируется без DOM.
//
// Сервер шлёт только поля, для которых есть источник (llm/wake/tts/topic/
// nearby); нет значения — нет ключа, панель рисует прочерк.

import type { Tars1Info } from "./tars1_layout";

export type TarsStatusFields = Pick<Tars1Info, "llm" | "tts" | "wake" | "topic" | "nearby">;

const FIELDS: (keyof TarsStatusFields)[] = ["llm", "tts", "wake", "topic", "nearby"];

/**
 * Вытащить поля из события. Не-строки и пустые строки отбрасываются (→ прочерк),
 * чужие ключи игнорируются. Не tars_status → null.
 */
export function parseTarsStatus(event: unknown): TarsStatusFields | null {
  if (typeof event !== "object" || event === null) return null;
  const e = event as Record<string, unknown>;
  if (e.type !== "tars_status") return null;
  const out: TarsStatusFields = {};
  for (const k of FIELDS) {
    const v = e[k];
    if (typeof v === "string" && v.trim()) out[k] = v.trim();
  }
  return out;
}
