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

- `expected_tool_calls` — хотя бы ОДИН из них должен быть вызван хотя бы
  в одном шаге (OR, не AND — каждый tool — отдельная фича).
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