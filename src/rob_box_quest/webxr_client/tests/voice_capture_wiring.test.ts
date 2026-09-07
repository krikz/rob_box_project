// Регрессия issue #1992: wake-микрофон шлема (stream_id=2) никогда не
// отправлял ни байта аудио, хотя весь остальной тракт (сервер → мост →
// /audio/quest_wake → stt_node) был подключён и подтверждён на роботе.
//
// Причина жила в main.ts (не в voice_capture.ts — там gate-логика уже
// была полностью протестирована в voice_capture.test.ts): единственный
// вызов `voiceCapture.start()` во всём файле находился внутри
// applyVoicePtt() и срабатывал только на зажатие грипа (PTT), а именно в
// этот момент wake принудительно подавлялся `setWakeGate({suppressed:
// true})`. При отпускании грипа вызывался `voiceCapture.stop()`, который
// рвёт общий getUserMedia-стрим целиком (ptt и wake делят один захват).
// Итог: `capturing === true` и `!wakeGate.suppressed` никогда не были
// истинны одновременно → push() в voice_capture.ts никогда не пропускал
// wake-чанк ни при каких обстоятельствах — ни до первого грипа (capture
// не запущен), ни во время грипа (wake подавлен), ни после отпускания
// (capture остановлен).
//
// main.ts — 1683-строчный bootstrap() с three.js/WebXR/Connection и без
// тестовой обвязки (см. tests/ — ни одного main.test.ts; тестируются
// только извлечённые модули). Поднимать здесь полный DOM+WebGL harness
// ради одной проверки — за рамками этого фикса. Вместо этого — контрактный
// тест на исходник: он не исполняет код, а проверяет ИМЕННО тот инвариант,
// который был нарушен, тем же методом, что уже принят в этом проекте для
// ADR DoD (см. docs/adr/0054-operator-agent-step-7b-eventbus-bridge.md
// §3: `git grep -i reflex ... | wc -l`). Слабее полного behavioural-теста,
// но ловит именно класс регрессии, который случился, и это лучше, чем
// ничего для файла без другой обвязки.

import { describe, it, expect } from "vitest";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const MAIN_TS_PATH = join(here, "..", "src", "main.ts");
const source = readFileSync(MAIN_TS_PATH, "utf-8");

/**
 * Вырезать тело `{ ... }`, начинающееся сразу после первого совпадения
 * `openPattern` (которое обязано заканчиваться на `{`), балансировкой
 * скобок. Не парсер TS — но main.ts не содержит `{`/`}` внутри строк на
 * релевантных участках (проверено), так что достаточно для контрактного
 * теста.
 */
function extractBracedBlock(src: string, openPattern: RegExp): string {
  const m = openPattern.exec(src);
  if (!m) {
    throw new Error(`pattern not found in main.ts: ${openPattern}`);
  }
  const start = m.index + m[0].length;
  let depth = 1;
  let i = start;
  for (; i < src.length && depth > 0; i++) {
    if (src[i] === "{") depth++;
    else if (src[i] === "}") depth--;
  }
  if (depth !== 0) {
    throw new Error(`unbalanced braces while extracting block for: ${openPattern}`);
  }
  return src.slice(start, i - 1);
}

describe("main.ts voice capture wiring (issue #1992 regression)", () => {
  it("starts the shared mic capture unconditionally, not gated behind grip/PTT", () => {
    const createIdx = source.indexOf("const voiceCapture = createVoiceCapture(");
    const applyVoicePttIdx = source.indexOf("function applyVoicePtt(");
    expect(createIdx).toBeGreaterThan(-1);
    expect(applyVoicePttIdx).toBeGreaterThan(createIdx);

    // Всё, что вызывается на voiceCapture МЕЖДУ созданием объекта и
    // определением applyVoicePtt, исполняется один раз при boot —
    // независимо от того, зажат ли когда-либо грип. Именно сюда
    // переехал реальный voiceCapture.start().
    const bootRegion = source.slice(createIdx, applyVoicePttIdx);
    expect(bootRegion).toMatch(/voiceCapture\.start\(\)/);
  });

  it("does not stop the shared capture when the grip is released (that would kill wake too)", () => {
    // Тело if (next === "none") { ... } внутри applyVoicePtt — ветка
    // "грип отпущен". voiceCapture.stop() здесь и был баг: рвёт ОБЩИЙ
    // getUserMedia-стрим (ptt+wake делят один захват), убивая wake ровно
    // в момент, когда следующая строка снимает с него suppression.
    const applyVoicePttBody = extractBracedBlock(
      source,
      /function applyVoicePtt\([^)]*\)\s*:\s*void\s*\{/
    );
    const noneBranch = extractBracedBlock(applyVoicePttBody, /if\s*\(next === "none"\)\s*\{/);

    expect(noneBranch).not.toMatch(/voiceCapture\.stop\(\)/);
    // Отпускание грипа обязано только гасить ptt-канал и снимать
    // wake-suppression — не рвать сам захват.
    expect(noneBranch).toMatch(/voiceCapture\.setPttEnabled\(false\)/);
    expect(noneBranch).toMatch(/voiceCapture\.setWakeGate\(\{\s*suppressed:\s*false\s*\}\)/);
  });

  it("still releases the mic on session dispose (no permanent hot-mic leak)", () => {
    // stop() должен остаться — но только на явном teardown сессии
    // (dispose()), не на каждое отпускание грипа.
    const disposeBody = extractBracedBlock(source, /dispose\(\)\s*:\s*void\s*\{/);
    expect(disposeBody).toMatch(/voiceCapture\.stop\(\)/);
  });

  it("surfaces getUserMedia/AudioWorklet failures instead of failing silently (hypothesis #4)", () => {
    // Раньше onError не был подключён вовсе: если getUserMedia/addModule
    // бросали (permission denied, no mic), voice_capture.ts ловил ошибку
    // в opts.onError?.(...) — и при отсутствии onError она просто
    // терялась. Ни одной строки нигде, даже при открытой консоли шлема.
    const createVoiceCaptureCall = extractBracedBlock(
      source,
      /createVoiceCapture\(\{/
    );
    expect(createVoiceCaptureCall).toMatch(/onError\s*:/);
  });
});
