# Voice Features — E2E Validation Plan (2026-08-22)

| Поле | Значение |
|------|----------|
| Автор | dev (по запросу Шифу) |
| Цель | Зафиксировать большие голосовые фичи за последние ~2 недели, определить как их проверять e2e, и завести валидационную задачу через процесс |
| Связано | `.github/e2e/scenarios/voice_core_suite_v1.json` + `voice_core_acceptance_v1.json`, issue #1506 |

## 1. Большие голосовые фичи (08.06–08.22)

| # | Фича | Источник (issue/PR/commit) | Как проверять e2e | Покрытие |
|---|------|----------------------------|-------------------|----------|
| 1 | MiniMax AI-генерация музыки (`generate_music`) | #1358, PR #1398, `4761ae3c` | «сгенерируй …» → `generate_music` | ⏸️ ИСКЛЮЧЕНО — MiniMax Music API недоступен |
| 2 | Музыкальная библиотека (`gen_*`: list/search/save/get_track_info) | #1358, PR #1398 | «что в библиотеке?» → `gen_list_library` | ⏸️ удаление (`gen_delete_from_library`) не проверяем пока |
| 3 | Renardo vs `gen_*` namespace split | #1371/#1372, `69adcc92` | «сыграй renardo бит» → `execute_music_code` | 🆕 `voice_core_suite_v1.json` (`dj01_start_renardo`) |
| 4 | Воспроизведение сгенерированного трека через sound_node | PR #1398, `5b05d9c9` | «сыграй последний трек» → `gen_play_from_library` | ⏸️ ИСКЛЮЧЕНО — нужен сохранённый трек (пост-условие generate_music) |
| 5 | Выбор голоса / `set_voice` + персистентный голос | #1219, `742a507e` | «говори голосом Алены» → `set_voice`/`alena` | ✅ `voice_selection_suite_v1.json` |
| 6 | Speech backlog — накопление речи без wake-слова | `7e8d8c20`, `b5b00666` | фраза БЕЗ wake-слова → потом «кто я» | 🔬 unit-тесты (`test_speech_backlog_accumulator.py`) — не voice-e2e (нет реакции на no-wake фразу) |
| 7 | New-session reset («сбрось всё») | `f7756178` | «сбрось всё» → `🧹 [new-session] session reset` + детерм. подтверждение | 🆕 `voice_core_suite_v1.json` |
| 8 | Command-intent gate (команды движения/статуса не идут в LLM) | #1279, `3871bbab` | «где ты» → `LLM dispatch skipped` (STATUS, без движения) | 🆕 `voice_core_suite_v1.json` |
| 9 | Music guard / stop-commands | `bcee38a6`, `1579d56b` | «сыграй renardo бит» → `execute_music_code`, затем «стоп музыку» → `stop_music` | 🆕 `voice_core_suite_v1.json` (`dj01_start_renardo` → `dj02_stop_music`) |
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
  - `dj01_start_renardo` — «Робот, сыграй renardo бит» → `execute_music_code` (музыка реально играет, MiniMax не нужен).
  - `dj02_stop_music` — «Робот, стоп музыку» → `stop_music` (музыка играет → стоп отрабатывает честно).
- **🔬 Только unit-тесты** — speech backlog (#6) и SSoT (#12): не voice-e2e-тестируемы, т.к. требуют отсутствия реакции или это внутренние инварианты.

## 3. GATE-1 acceptance (voice_core_acceptance_v1.json)

- `expected_tool_calls: ["execute_music_code", "stop_music"]` (AND-семантика: цепочка music start→stop; остальные шаги намеренно обходят LLM).
- `must_not_call: []` (глобально ничего не запрещаем — per-step ограничения живут в шагах).

## 4. Риски / что подтвердить live-прогоном

- `dj02_stop_music`: вызов `stop_music` зависит от LLM, но музыка реально играет после `dj01_start_renardo` — если робот «решит» просто ответить текстом, шаг (и GATE-1) честно зафейлится. Это и есть валидируемый сценарий.
- `cc01_status_gate`: маркер и feedback сверены с кодом, но финальный полный цикл (STT→feedback→TTS) подтверждается live-прогоном.
- Полный прогон требует живого робота (10.1.1.21) + LLM (minimax-m3) + Yandex TTS. MiniMax Music API не нужен (генерация исключена).

## 5. Как это пойдёт по процессу

Issue с метками `hermes` + `needs-e2e` + `## e2e`-блоком (`scenario_file`/`acceptance_file`) → triage → kanban-карточка → worker → e2e-process → вердикт. Найденные баги заводятся отдельными issues по `bug`-шаблону.
