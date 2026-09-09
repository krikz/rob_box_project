# Хендофф — живой e2e на реальном роботе, 2026-08-29

> **Кто писал:** Claude Sonnet 5, сессия с прямым SSH-доступом к билд-машине
> (10.1.1.249, Tailscale-хост `ros2-katana-gf66-11ud`) и роботу (10.1.1.21).
> **Честность (ADR-0018):** всё ниже — либо raw-вывод реальных workflow-ранов
> (ссылки), либо прямая цитата логов робота. Ничего не помечено «работает»
> без прогона.

---

## 1. Что было целью сессии

1. Проверить S8 (e2e) из `docs/plans/2026-08-28-scheduler-segments-merge-plan.md`
   (issue #968) на реальном железе — предыдущий воркер не мог: «нет ни ROS 2,
   ни стенда» (см. `docs/plans/2026-08-28-scheduler-KICKOFF.txt`).
2. Дальше — прогнать по спеку другие сьюты `.github/e2e/scenarios/`, часть
   из которых параллельно готовил Opus.

---

## 2. Состояние билд-машины (Катана) — важно для следующего прогона

- **Аудио было физически сломано** в начале сессии: что-то было воткнуто в
  headphone-jack ноутбука, PulseAudio продолжал считать активным порт
  `analog-output-speaker`, реальный звук уходил в джек, где ALSA-микшер
  `Headphone` был замьючен на 0% — итог: полная тишина при видимо-успешном
  `paplay`/`pactl`.
- **Временный software-обход** (не переживает `pactl` reset/reboot):
  ```bash
  pactl set-sink-port alsa_output.pci-0000_00_1f.3.analog-stereo analog-output-headphones
  amixer sset Headphone 100% unmute
  ```
- **Пользователь физически подключил внешние колонки** к Катане и это
  наконец заработало «по-честному» — реальный e2e с 2026-08-29 11:20 UTC
  и позже уже шёл через нормальный звук, без обходов.
- Если в следующий раз опять «e2e синтезирует команду, `paplay` без ошибок,
  но робот молчит 0 строк в логах 10+ минут» — **первым делом проверяй звук
  на Катане** (`pactl list sinks` → `Active Port`, `amixer sget Speaker`),
  а не сценарий/харнесс. На это ушло ~2 часа сессии.
- Также был найден (и не является причиной) конфликт PipeWire/PulseAudio на
  этой машине — оба процесса живут одновременно, PipeWire оказался
  побочным, не главной причиной, но стоит иметь в виду.

---

## 3. Главная находка — issue #1734 (ИСПРАВЛЕНО)

**Баг:** `stt_node.py:1012-1017` безусловно публиковал `STOP` в
`/voice/tts/control` на любое новое STT с wake-словом, **независимо** от
`dialogue_node.barge_in_policy`. План #968 (S1-S7) правил только
`dialogue_node.py` — этот код-путь (issue #993, старше #968) остался
незамеченным. Из-за этого MERGE ("правка на лету без замолкания",
`SCHEDULER_DESIGN.md` §2.5) физически не мог работать даже с
`barge_in_policy=classify`.

**Найдено** живым прогоном `968_merge_song_komar_i_enot` на реальном
роботе — GATE-1 (`must_not_call: ["STOP command received"]`) падал, хотя
STT честно принимал обе фразы.

**Заведено:** [issue #1734](https://github.com/krikz/rob_box_project/issues/1734).

**Исправлено** другим воркером (W2-8), commit `2da2b122`: `dialogue_node`
публикует `barge_in_policy` на latched-топик
`/voice/dialogue/barge_in_policy` (TRANSIENT_LOCAL), `stt_node`
подписывается и при `classify` откладывает `STOP`, отдавая решение
`dialogue_node`/`quick_decide`.

**Проверено мной живьём** после билда+деплоя коммита `2da2b122`:
- Runs: [build 33249830794](https://github.com/krikz/rob_box_project/actions/runs/33249830794) → [deploy 33250142611](https://github.com/krikz/rob_box_project/actions/runs/33250142611) → [e2e 33250361541](https://github.com/krikz/rob_box_project/actions/runs/33250361541) — **E2E_VERDICT PASS**, GATE-1 ✅.
- Механизм подтверждён: `ros2 param set /dialogue_node barge_in_policy classify` →
  `ros2 topic echo /voice/dialogue/barge_in_policy --once` вернул `data: classify`
  **без рестарта контейнера**.
- В логе робота вместо старой пары строк — ровно одна:
  ```
  [stt_node-6] 🎯 [issue 1734] Wake word detected: "..." → STOP TTS отложен
  (barge_in_policy=classify, решает dialogue_node/quick_decide)
  ```
  и **нигде** нет `STOP command received`.

### ⚠️ Важный недоделанный хвост — читать перед тем как закрывать #968

Даже с зелёным GATE-1 **настоящего MERGE через сегменты нет**. В логе
`dialogue_node` на вторую фразу («и ещё про енота»):
```
LLM INPUT: 'и еще про енота'
process_input returned: spoken='Енот на реке! Енот!...' tools=[] error=None
```
Это **новая история с нуля** про енота, а не продолжение куплета про
комара. Ни `task_delta`, ни `[SEGMENT PLAN]`, ни `task.updated` в логах
нет вообще.

**Причина** (уже задокументирована в BRIEF/design, не новая находка): в S2-S6
плана `TaskScheduler` получил `group_id`/`seg_idx`/`update()`, но четыре
хука интеграции —
`set_group_boundary`, `set_frozen_touch_hook`, `set_llm_continue_hook`,
`set_eta_provider` — **ни к чему не подключены**. Без блока `[SEGMENT PLAN]`
в системном контексте LLM физически не видит, что можно править, и просто
отвечает с нуля.

**Что дальше по #968:** следующая карточка — подключить эти хуки (S5 плана,
`tool_executor.py:223`, рядом с `active_tasks_block`) и повторно прогнать
`968_merge_song_komar_i_enot` с проверкой `task.updated` в
`/harness/task_events`. Сценарий и acceptance уже готовы:
`.github/e2e/scenarios/968_merge_song_komar_i_enot.json` +
`968_merge_song_acceptance.json`.

---

## 4. Побочная находка — issue #1735 (ИСПРАВЛЕНО)

Харнесс `e2e_voice_wake_gate.sh` не пушился на билд-машину вместе с
`e2e_voice_test.sh`/`e2e_voice_lib.sh` → `WAKE_GATE_CLEARED` не
выставлялся → шаги с `expect=backlog` (аккумулятор без wake-слова, ds01/ds02
в `voice_core_suite_v1.json`) ложно падали как `FAIL no_accept`, хотя
реально аккумулятор работал штатно. Исправлено в `742d3eae`, проверено:
после фикса `ds01_speaker_A_tea` / `ds02_speaker_B_coffee` дают честный
`OK backlog`.

---

## 5. Прогнанные сьюты и их реальный статус

| Сьюта | Run | Статус | Комментарий |
|---|---|---|---|
| `voice_core_suite_v1.json` | [33248910201](https://github.com/krikz/rob_box_project/actions/runs/33248910201) | 8/10 OK | `mv03`, `dj02` падают — уже помечены `flaky-known` в самом файле (LLM иногда отвечает без вызова тула). Не новая находка. |
| `968_merge_song_komar_i_enot.json` | [33250361541](https://github.com/krikz/rob_box_project/actions/runs/33250361541) | GATE-1 PASS, но MERGE не настоящий | См. §3. |
| `dialogue_tools_coverage_v1.json` | [33251879328](https://github.com/krikz/rob_box_project/actions/runs/33251879328) (чистый повтор после сброса сессии) | 7/16 OK | См. §6 — новые находки. |
| `dialogue_gap_probe_v1.json` | — | **не прогонялся** | Закоммичен, готов. Диагностический, ожидаемо частично красный by design. |
| `dialogue_motion_coverage_v1.json` | — | **не прогонялся, не закоммичен** | 🚨 Двигает робота и переписывает карту. Лежит локально в рабочей копии (не в git) — `docker/vision/test/...` нет, файл в `.github/e2e/scenarios/dialogue_motion_coverage_v1.json` и `..._acceptance_v1.json`. Требует физического присутствия человека у кнопки останова, 2м свободного пространства, бэкап карты. **Не гонять без явного подтверждения условий безопасности.** |
| `music_library_suite_v1.json`, `full_instrument_suite_v1.json`, `voice_selection_suite_v1.json` | — | не прогонялись в этой сессии | Уже существовали до сессии, не тронуты. |

### 5.1 Важный урок про session state между сьютами

Первый прогон `dialogue_tools_coverage_v1` дал 15/16 FAIL с массовыми
ответами `spoken='Молчу, жду команды.'` — робот отвечал так, будто ждёт
подтверждения от **предыдущего** контекста (сессия не была сброшена между
предыдущим `968_merge_song` прогоном и этим). После
`ros2`/голосовой команды сброса сессии («Робот, сбрось всё») повторный
прогон дал куда более честную картину (7 OK / 9 FAIL по существу).

**Вывод для следующего воркера:** между независимыми e2e-сьютами **обязательно
сбрасывать диалоговую сессию** (шаг `ns01_reset_session` в начале
`voice_core_suite_v1` для этого и существует — но у отдельно запускаемых
сьют типа `dialogue_tools_coverage_v1` такого шага нет, добавь вручную или
как первый шаг).

---

## 6. Новые находки из `dialogue_tools_coverage_v1` (чистый прогон)

7 из 16 прошли (`tc01`, `tc02`, `tc07`, `tc08`, `tc10`, `tc11`, `tc14`).
9 упали. Разобрано по каждому:

| Шаг | Тул | Причина FAIL | Статус |
|---|---|---|---|
| `tc03_faq_search` | `faq_search` | **Не баг.** Event-режим (FAQ мероприятия) намеренно отключён с июня 2026 (commit `67e20164`, «Robot is no longer at the open day event»). Acceptance уже это учитывает (`expected_tool_calls: []`). | Ожидаемо, не трогать. |
| `tc04_perception_context` | `get_perception_context` | Тул **полностью реализован и подключён живьём**: проверил `ros2 topic info /perception/context_update -v` — есть реальный паблишер `context_aggregator` (питается от `oak-d`) и подписчик `mcp_server`. LLM просто **не вызвал тул**, хотя его прямо попросили. | Настоящая находка — промпт не учит LLM когда звать этот тул. |
| `tc05_sound_info` | `get_sound_info` | LLM не вызвал тул. Не проверял инфраструктуру отдельно (по аналогии с tc04 вероятно тоже промпт). | Нужна отдельная проверка + вероятно тот же класс бага. |
| `tc06_music_state_idle` | `get_music_state` | LLM не только не вызвал `get_music_state`, но и вызвал **запрещённый** `execute_music_code`. | Похоже на путаницу intent — стоит завести отдельно. |
| `tc09_list_tracks` | `list_tracks` | LLM не вызвал тул. | Как tc04/tc05. |
| `tc12_delete_track` | `delete_track` | LLM не вызвал тул. | Как tc04/tc05. |
| `tc13_estimate_tts_duration` | `estimate_tts_duration` | ACCEPTANCE (JSON) прошёл ✅, но свободный `patterns`-чек (`PATTERN_MISS: estimate_tts_duration`) не совпал → шаг всё равно помечен `FAIL`. **Не обязательно баг фичи** — возможно баг самого сценария/паттерна. | Нужно перепроверить сам YAML/JSON сценария, не код робота. |
| `tc15_save_then_delete_waypoint` | `save_waypoint` | LLM не вызвал тул. | Как tc04/tc05. |
| `tc16_delete_waypoint` | `delete_waypoint` | LLM не вызвал тул. | Как tc04/tc05. |

**Паттерн:** это тот же класс проблемы, что уже задокументирован в
`voice_core_suite_v1` для `dj02_stop_music` («voice-cycle OK but tool
skipped... LLM сделал verbal-only answer») — целый пласт из ~34 зарегистрированных
тулов (`mcp_server` стартует со списком 51 тула — см. лог
`✅ Подписан на /perception/context_update` + список тулов), которые
реально существуют и подключены, но **мастер-промпт не enforce'ит их
вызов** для команд, которые не входят в «частые» сценарии. Стоит завести
отдельный issue по мотивам этой сьюты (или несколько — по одному на явную
находку, `tc06` отдельно из-за forbidden-call).

---

## 7. Незакоммиченное / оставленное как есть

- `.github/e2e/scenarios/dialogue_motion_coverage_v1.json` +
  `dialogue_motion_coverage_acceptance_v1.json` — **не в git**, лежат в
  рабочей копии. Не коммить и не гоняй без физической проверки безопасности
  (см. §5, таблица).
- `docs/e2e/dialogue-coverage-map-2026-08-29.md` — тоже untracked, написан
  Opus, на него ссылается `dialogue_gap_probe_v1.json`. Не мой файл, не
  трогал.
- Разные `docs/plans/2026-08-28-*` / `2026-08-29-*` (KICKOFF, BRIEF, wave2)
  — untracked с начала сессии, не мои, не трогал.

---

## 8. Текущее live-состояние робота (может протухнуть!)

- На момент конца сессии `barge_in_policy` на реальном `/dialogue_node`
  выставлен **вручную через `ros2 param set`** в `classify` — это НЕ
  персистентный конфиг (`docker/vision/config/voice_assistant/dialogue_node.yaml:39`
  всё ещё `"replace"`, так и должно быть до зелёного S8 целиком — S8 Task 8.3).
  **Переживёт до следующего рестарта контейнера `voice-assistant`**, потом
  сам вернётся на `replace`. Если нужно продолжить тестировать MERGE —
  проверь текущее значение (`ros2 param get /dialogue_node barge_in_policy`)
  перед прогоном.
- Деплой на роботе сейчас = commit `2da2b122` (staging/dev тег), включает
  фикс #1734.

---

## 9. Рекомендуемый порядок для следующей итерации

1. Проверить/восстановить `barge_in_policy=classify` (§8), если ещё не
   сброшен рестартом.
2. Прогнать `dialogue_gap_probe_v1.json` (готов, закоммичен) — ожидаемо
   частично красный, ценность в сыром логе по известным дырам спека.
3. Решить с пользователем безопасность для `dialogue_motion_coverage_v1.json`
   (физическое пространство, человек у кнопки останова, бэкап карты) —
   и либо прогнать, либо оставить на потом.
4. Завести issue(ы) по находкам §6 (tool-calling enforcement для
   `get_perception_context`/`get_sound_info`/`get_music_state`/`list_tracks`/
   `delete_track`/`save_waypoint`/`delete_waypoint` — либо один общий, либо
   разбить, `tc06` forbidden-call отдельно).
5. Проверить `tc13_estimate_tts_duration` — баг сценария или баг фичи
   (§6, отдельная строка).
6. Следующая карточка по #968: подключить 4 хука планировщика (S5/S9 плана)
   и повторно прогнать `968_merge_song_komar_i_enot` с проверкой
   `task.updated`.
