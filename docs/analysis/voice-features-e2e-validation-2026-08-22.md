# Voice Features — E2E Validation Plan (2026-08-22)

| Поле | Значение |
|------|----------|
| Автор | dev (по запросу Шифу) |
| Цель | Зафиксировать большие голосовые фичи за последние ~2 недели, определить как их проверять e2e, и завести валидационную задачу через процесс |
| Связано | `.github/e2e/scenarios/voice_core_suite_v1.json` + `voice_core_acceptance_v1.json`, issue (создаётся следом) |

## 1. Большие голосовые фичи (08.06–08.22)

| # | Фича | Источник (issue/PR/commit) | Как проверять e2e | Покрытие |
|---|------|----------------------------|-------------------|----------|
| 1 | MiniMax AI-генерация музыки (`generate_music`) | #1358, PR #1398, `4761ae3c` | «сгенерируй …» → `generate_music`, НЕ `execute_music_code` | ✅ `music_library_suite_v1.json` |
| 2 | Музыкальная библиотека (6 `gen_*` tools: list/search/save/delete/get_track_info) | #1358, PR #1398 | «что в библиотеке?» → `gen_list_library` и т.д. | ✅ `music_library_suite_v1.json` |
| 3 | Renardo vs `gen_*` namespace split | #1371/#1372, `69adcc92` | «сыграй renardo бит» → `execute_music_code` (negative control) | ✅ `music_library_suite_v1.json` |
| 4 | Воспроизведение сгенерированного трека через sound_node + LLM-решаемый stop | PR #1398, `5b05d9c9`, `36e500d5` | «сыграй последний трек» → `gen_play_from_library` | ⚠️ частично (нужен сохранённый трек; в suite не входит) |
| 5 | Выбор голоса / `set_voice` + персистентный голос | #1219, `742a507e` | «говори голосом Алены» → `set_voice`/`alena` | ✅ `voice_selection_suite_v1.json` |
| 6 | Speech backlog — накопление речи без wake-слова | `7e8d8c20`, `b5b00666` | фраза БЕЗ wake-слова → потом «кто я» | 🔬 unit-тесты (`test_speech_backlog_accumulator.py`) — не voice-e2e (нет реакции на no-wake фразу) |
| 7 | New-session reset («сбрось всё») | `f7756178` | «сбрось всё» → `🧹 [new-session] session reset` + детерм. подтверждение | 🆕 `voice_core_suite_v1.json` |
| 8 | Command-intent gate (команды движения/статуса не идут в LLM) | #1279, `3871bbab` | «где ты» → `LLM dispatch skipped` (STATUS, без движения) | 🆕 `voice_core_suite_v1.json` |
| 9 | DJ mode guardrails / music guard (chop/spack, stop-commands) | `bcee38a6`, `1579d56b`, `4d090792` | «стоп музыку» → `stop_music` | 🆕 `voice_core_suite_v1.json` (только stop; DJ-сет требует музыку) |
| 10 | Barge-in: отмена старой темы при новой фразе | #1280 | «длинная команда» → «дважды два» → `Cancel: new STT input` | ✅ `1280_barge_in_abort_old_topic.json` |
| 11 | Wake words sync (13 вариантов) | #1252, `5e1f5bdf` | альтернативный wake word «Робокс» | 🆕 `voice_core_suite_v1.json` (`ww01_roboks_wake`) |
| 12 | LLM skip reason SSoT | #1409, `8b8bf1aa` | — (внутренний guard) | 🔬 unit-тесты |

## 2. Стратегия e2e-проверки

Три класса:

- **✅ Уже покрыто** существующими сценариями — `music_library_suite_v1.json`, `voice_selection_suite_v1.json`, `1280_barge_in_abort_old_topic.json`.
- **🆕 Новый сценарий** `voice_core_suite_v1.json` — фичи #7/#8/#9/#11, которые voice-e2e-тестируемы и дают полный цикл:
  - `cc01_status_gate` — «Робот, где ты» → command-intent gate (STATUS, без движения). Маркеры сверены: `command_parser.py` STATUS regex `(где|куда)\s+(ты|робот)`, лог `🎯 [issue 1279] … LLM dispatch skipped`, feedback `Я нахожусь в стартовой позиции` → TTS.
  - `ns01_reset_session` — «Робот, сбрось всё» → `🧹 [new-session] session reset` + детерминированное подтверждение («Начинаю новую сессию…»).
  - `ww01_roboks_wake` — «Робокс, как дела» → полный цикл на альтернативном wake-слове.
  - `dj01_stop_music` — «Робот, стоп музыку» → `stop_music` (music-stop override доходит до LLM).
- **🔬 Только unit-тесты** — speech backlog (#6) и SSoT (#12): не voice-e2e-тестируемы, т.к. требуют отсутствия реакции или это внутренние инварианты.

## 3. GATE-1 acceptance (voice_core_acceptance_v1.json)

- `expected_tool_calls: ["stop_music"]` (AND-семантика: единственный ожидаемый tool — остальные шаги намеренно обходят LLM).
- `must_not_call: []` (глобально ничего не запрещаем — per-step ограничения живут в шагах).

## 4. Риски / что подтвердить live-прогоном

- `dj01_stop_music`: вызов `stop_music` зависит от LLM — если робот «решит» просто ответить текстом, шаг (и GATE-1) честно зафейлится. Это и есть валидируемый сценарий.
- `cc01_status_gate`: маркер и feedback сверены с кодом, но финальный полный цикл (STT→feedback→TTS) подтверждается live-прогоном.
- Полный прогон требует живого робота (10.1.1.21) + квоты MiniMax/Yandex.

## 5. Как это пойдёт по процессу

Issue с метками `hermes` + `needs-e2e` + `## e2e`-блоком (`scenario_file`/`acceptance_file`) → triage → kanban-карточка → worker → e2e-process → вердикт. Найденные баги заводятся отдельными issues по `bug`-шаблону.
