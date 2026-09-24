# GATE-1: acceptance.json — дизайн (t_ba114e5c)

## Что это

ADR-0022 §4.1 GATE-1 — обязательный acceptance.json (next to scenario.json) с
`expected_tool_calls` + `must_not_call`. Если задан `scenario.json` И
`acceptance.json` отсутствует → **FAIL** (gating активен).

## Acceptance criteria (из kanban t_ba114e5c)

- [ ] `e2e_voice_test.sh` принимает `--acceptance <path>` или ищет
      `acceptance.json` рядом с `scenario.json`
- [ ] Парсинг: `expected_tool_calls: list[str]` + `must_not_call: list[str]`
- [ ] Validate: если фактические tool_calls НЕ содержат все из `expected`
      OR содержат любой из `must_not_call` → FAIL с объяснением
- [ ] Write `acceptance.json` в artifacts: фактические calls + verdict
      (PASS/FAIL)
- [ ] Если `acceptance.json` отсутствует И `scenario.json` задан → FAIL
- [ ] Юнит-тест: мок acceptance с `expected=["generate_music"]`,
      `must_not_call=["execute_music_code"]` → e2e FAIL если вызван Renardo
- [ ] Live: PR #1398 + acceptance.json → e2e проверяет generate_music вызван

## Что НЕ меняется

- `check_acceptance()` (per-step) — остаётся, это legacy/расширенный контракт
  (issue #1396: `expected_keywords`, `response_max_ms`).
- ADR-0022 GATE-1 — **top-level aggregate** layer поверх per-step.
- `--text` single-shot — НЕ требует acceptance.json (smoke-test остаётся
  для fast iteration).

## Что НЕ делаем (out-of-scope)

- Live прогон на роботе — это e2e-process после PR.
- `agent-flow-merge-gate.sh` / `agent-flow-e2e-process.sh` — GATE-1 валидация
  там отдельная карточка.

## Контракт acceptance.json

```json
{
  "name": "music_library_suite_v1",
  "expected_tool_calls": ["generate_music", "gen_list_library"],
  "must_not_call": ["execute_music_code"],
  "_comment": "GATE-1 contract: aggregate across all steps"
}
```

- `expected_tool_calls` — КАЖДЫЙ из них должен быть вызван хотя бы
  в одном шаге (AND, не OR — один acceptance.json покрывает все новые
  tools; если хотя бы один не сработал, e2e-done не ставится,
  ADR-0022 §5.3).
- `must_not_call` — НИ ОДНОГО из них не должно быть вызвано ни в одном шаге.
- Допустимые дополнительные поля (forward-compat): `expected_keywords`,
  `response_max_ms`, `must_call_for_renardo_request` (negative control).

## Архитектура изменений

```
e2e_voice_test.sh
  ├── parse args: --acceptance <path>  (NEW)
  ├── parse args: SCENARIO_FILE set + acceptance missing → FAIL GATE-1  (NEW)
  ├── per-step acceptance (legacy, unchanged)
  └── GATE-1 aggregate check (NEW)
        ├── собирает tool_call facts из docker logs за весь прогон
        ├── пишет $OUT_DIR/acceptance.json с verdict
        └── если expected отсутствует OR must_not_call сработал → PASS=0
```

## Artifacts

`$OUT_DIR/acceptance.json` — расширенный формат:

```json
{
  "gate": "GATE-1",
  "expected_tool_calls": ["generate_music"],
  "must_not_call": ["execute_music_code"],
  "actual_tool_calls": ["generate_music", "set_dj_mode"],
  "found_expected": ["generate_music"],
  "missing_expected": [],
  "forbidden_called": [],
  "pass": true,
  "reason": "all checks passed"
}
```

## Per-step: регистрация голоса и вердикт шага (issue #2846)

Живой прогон 35875477264 (акт 2): шаги `n201…n202c` получили `E2E_STEP … OK`,
а `speaker_id_node` отклонял каждую регистрацию (`no_utterance_context`).
Per-step acceptance проверял только, что LLM **вызвала** `register_speaker`.
Тул — fire-and-forget публикация в топик: он «выполнен успешно» и тогда, когда
профиль не заведён.

### Правило «регистрация принята»

Для шага, у которого `register_speaker` есть в `expected_tool_calls` или
`discovery_tools`, `check_acceptance` дополнительно требует **исход**
регистрации в логе `voice-assistant` за окно шага
(`registration_failures()` в `.github/workflows/scripts/e2e_tool_match.py`):

| в логе шага | вердикт |
|---|---|
| `✅ Speaker '<имя>' registered (id=…)` (speaker_id_node) | принята |
| `⚠️ Speaker '<имя>' (id=…) — голос похож на уже известного …` (ADR-0127, отдельный профиль) | принята |
| `register_request for '<имя>' has no utterance_id` / `Регистрация '<имя>' отклонена: no_utterance_context` | **FAIL** |
| `register_request for '<имя>': no embedding for utterance=…` / `… отклонена: utterance_not_found` | **FAIL** |
| `Registration of '<имя>' rejected` / `Регистрация '<имя>' отклонена — реплика …` (too_short) | **FAIL** |
| `🔗 Speaker '<имя>' merged into existing profile` | **FAIL** (склейка, run 35667281570) |
| ничего из этого (ack так и не пришёл) | **FAIL** — «не подтверждена» |

Ack ноды может лечь в лог чуть позже начала проверки (нода ждёт эмбеддинг до
1.5с), поэтому `check_acceptance` дочитывает лог до
`E2E_REGISTRATION_ACK_WAIT_SEC` секунд (по умолчанию 6), пока исхода нет.
Отказ считается FAIL, даже если позже в том же окне есть принятие.

Почему авто-правило, а не только новое поле: все шаги сценариев, где
ожидается `register_speaker` (акты 2/2b/2c, `dialogue_gap_probe`), и так
перечисляют его в `expected_tool_calls`. Сценарий, требующий вызова тула,
требует и его результата — поле, которое надо не забыть поставить, забывают
(именно так #2846 и прожил). Явное поле оставлено как ручка:

```json
"acceptance": {
  "expected_tool_calls": ["register_speaker"],
  "require_registration_accepted": false
}
```

`require_registration_accepted` (bool, необязательное) перекрывает
авто-правило в обе стороны: `false` — не требовать исхода (например, шаг
проверяет именно честный отказ), `true` — требовать и без
`register_speaker` в списках. Не-булево значение — ошибка схемы, шаг FAIL.
Склейка профиля проверяется и у шагов с `must_not_call: ["register_speaker"]`
— как и раньше, когда это делал отдельный bash-блок scenario-цикла.

В `acceptance.json` шага добавлены поля `registration_expected`,
`registration` (`accepted`/`merged`/`rejected`) и `registration_failures`.

### Одна строка вердикта на шаг

Две строки `ACCEPTANCE[n201…]: ❌ …` и `ACCEPTANCE[n201…]: ✅ all checks passed`
в том прогоне — это **две попытки** ретрая (`retry_acceptance: 1` у n201), а
не две проверки одного и того же. Ретрай штатный (LLM недетерминирован,
`04d4ba2f6`): решает последняя попытка. Теперь:

- при `retry_acceptance > 0` строка помечена попыткой:
  `ACCEPTANCE[n201…] попытка 1/2: ❌ …`;
- склейка профиля больше не печатает отдельное `❌` после `✅` той же
  попытки — она внутри того же вердикта;
- итог шага — одна строка `STEP <label>: итог — ✅ OK с попытки 2/2; прежние
  попытки провалились: …` или `STEP <label>: итог — ❌ FAIL, …`;
- OK после проваленной попытки уходит в `E2E_STEP <label> OK after_retry=N`
  (статус по-прежнему `OK`, сводка считает его как OK — детали видны).