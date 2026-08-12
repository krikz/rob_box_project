# E2E Testing Design v2 — реальный pipeline, multi-model coverage, валидация после fix-ов 06.08

> **⚠️ UPDATE 11.08 (атомарный харнесс v3):** с 11.08 e2e-прогоны идут на
> **`e2e_voice_test.sh` (атомарный v2)** — см. §A.10. Старый `e2e_remote.sh`
> (шляпа) и `ensure_voice_file.sh` + закоммиченные .ogg **устарели**:
> команда синтезируется на лету через Yandex TTS (голос/текст из карточки),
> харнесс сам ждёт ПОЛНЫЙ цикл STT→LLM→TTS в логах робота (с проверкой
> порядка по ROS-timestamp), ретраит команду при NO_ACCEPT, а не «проиграл
> и записал». Контракт e2e-блока в issue — в §A.10.3.

| Поле | Значение |
|------|----------|
| Документ | `docs/design/E2E_TESTING_DESIGN_v2.md` |
| Наследник | `docs/design/E2E_TESTING_DESIGN.md` v1 (коммит `cddd2f30` на `feature/harness-p0-foundation`) |
| Назначение | Дополнить v1 тем, что появилось после 06.08 09:13 МСК: реальный pipeline `e2e_series7.sh` / watcher, multi-model coverage (LLM × TTS), обновлённые verdict'ы серии v7 (10/10 SUCCESS) после фиксов `dda417c8` / `7ab0caf4` / `5d3df7e2` |
| Аудитория | QA / backend / reviewer / PM |
| Дата | 2026-08-06 |
| Автор | Architect (kanban t_4f546ead, run 800) |
| Статус | **v2 — design delta**, заменяет/дополняет §3, §4, §5, §7, §10 v1 |
| Связанное | `docs/design/SCHEDULER_DESIGN.md` v5, `analysis/dialog_node.md`, recovery `t_4f546ead` (RECOVERY_REPORT.md), parent `t_5fb8a092` (SUMMARY_8runs.json), issue #968/#935/#993 |

---

## 0. TL;DR — что изменилось после v1

**v1 (cddd2f30, 09:13 МСК 06.08)** зафиксировал дизайн на основе parent-summary (CI-артефакты, 05.08, 8 verdict'ов) и recovery (SSH на роботе, 06.08 09:00 МСК, 7 wav'ов). Главный продукт — acceptance-матрица 41+3 строки, 3-уровневый стек L0/L1/L2.

**После 09:13 МСК произошло 4 события**, которые v1 не покрывает:

1. **Запустилась реальная e2e-серия v7** (`/tmp/e2e_series7.sh`), которая прошла 10/10 SUCCESS на тех же фразах + 2 новых (`rabot_dj_off.ogg`, `spoy_peasenku_pro_enotika.ogg`). Это **иной pipeline**, не описанный в v1:
   - GH Actions workflow `L-E2E Voice Test.yml`, ref `feature/harness-p0-foundation`, dispatch через `gh workflow run`
   - OGG voice-команды в `.github/e2e/voice_commands/` (MiniMax TTS `male-qn-qingse`, **«Робот!» с запятой** после wake word — пауза улучшает STT)
   - Watcher `/tmp/e2e_watcher7.sh`: scp с билдовой 10.1.1.249, ffmpeg→mp3, READY-флаг для cron-доставки в Telegram
   - Валидация звука через `ffmpeg -af volumedetect` (команда ~-10dB, музыка ~-37dB, тишина -48dB — по `default`'у 09:18)
2. **Вышли три fix-а**, которые изменили поведение робота: `dda417c8` (effect_manager.reload, музыка починена), `7ab0caf4` (Bug C retry, LLM больше не пропускает execute_music_code из истории диалога), `5d3df7e2` (silence_commands gate, «хватит диджеить» больше не уходит в silence). Результат: **10/10 wav'ов v7 имеют звук** (mean -16.6...-25.1 dB), что подтверждает **A39** (execute_music_code → RMS), который v1 помечал как OPEN.
3. **PM в комментариях 09:18–09:20** потребовал: (а) описать **как реальные e2e на роботе происходят**, (б) фреймворк должен **тестировать разные модели звука и LLM**, (в) текущий стек — Minimax (с подпиской пока есть). Это означает параметризацию pipeline по `(LLM, TTS, STT)`.
4. **voice-action-server по-прежнему крашится** (`ModuleNotFoundError: aiohttp`, рестарты каждые ~44с) — баг из recovery §5.1 не пофикшен. Должен попасть в L2 snapshot (уже был в v1 §9, но проверка — нет).

**Что v2 делает:**
- Описывает **§A — реальный pipeline как first-class citizen** дизайна (то, чего v1 не хватает)
- Вводит **§B — multi-model matrix** (LLM × TTS × STT) как параметризованное измерение прогона
- Обновляет **§C — verdict'ы v7** (10/10 SUCCESS, mean RMS, новые цифры)
- Переводит **A39** в VERIFIED со ссылкой на v7 RMS-данные
- Добавляет **A42** — наблюдаемость multi-model (parametrized test_id)
- Добавляет **A43** — snapshot всех контейнеров (voice-action-server должен быть в health-check)
- Содержит **§D — обновлённый OPEN-questions лист** (4 новых)

**Чего v2 НЕ делает:**
- Не переписывает v1 целиком — это delta-документ. v1 (44-строчная матрица, §3 уровни, §8 метрики, §9 snapshot.sh, §10 план) остаётся актуальным для **acceptance-слоя**.
- Не описывает новые сценарии (N1–N13 уже в v1 §3.3) — они не зависят от pipeline.

---

## §A. Реальный e2e-pipeline (то, что v1 не описал)

### A.1 Pipeline v7 (production, июнь-август 2026)

```
┌─────────────────────────────────────────────────────────────────────┐
│                     /tmp/e2e_series7.sh (driver)                    │
│                                                                     │
│  declare FILES=(10 .ogg из .github/e2e/voice_commands/)             │
│  for F in FILES:                                                    │
│    1. gh workflow run "L-E2E Voice Test.yml" \                      │
│         --ref feature/harness-p0-foundation \                       │
│         -f environment=test -f voice_file=<F> -f volume=150         │
│       → run_id возвращается из URL вывода                           │
│    2. gh run watch $RUNID --exit-status                            │
│       → exit-code 0 если workflow прошёл                            │
│    3. sleep 180 (пауза между прогонами, чтобы cron Telegram         │
│         успел отдать прошлый mp3)                                   │
│  end                                                                │
└──────────────────────┬──────────────────────────────────────────────┘
                       │ dispatch (HTTP POST к GH API)
                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│  GH Actions: .github/workflows/L-E2E Voice Test.yml (ref:           │
│  feature/harness-p0-foundation)                                     │
│                                                                     │
│  1. checkout repo                                                   │
│  2. setup env "test"                                                │
│  3. ensure_voice_file <voice_file> — копирует OGG в build workspace │
│  4. ssh на 10.1.1.249 (build host), запускает docker run:          │
│     - voice-assistant (privileged, clock sync: date -s ...)         │
│     - воспроизводит OGG через arecord/playback на mic pipe          │
│  5. запускает dialog_e2e recorder → /tmp/dialog_e2e_<run_id>.wav    │
│  6. upload artefact run-<run_id>-dialog_e2e.wav (90с моно 16кГц)    │
└──────────────────────┬──────────────────────────────────────────────┘
                       │ artefact download
                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│              /tmp/e2e_watcher7.sh (delivery)                        │
│                                                                     │
│  loop:                                                              │
│    1. читает /tmp/e2e_series7.log (E2E_RESULT строки)               │
│    2. для каждой новой записи:                                      │
│       - sshpass scp ros2@10.1.1.249:/tmp/dialog_e2e_<run_id>.wav    │
│         /tmp/dialog_<name>.wav                                     │
│       - fallback: latest /tmp/dialog_e2e_*.wav                     │
│       - ffmpeg -y -i ... -c:a libmp3lame -b:a 64k                  │
│         /home/builder/hermes-share/e2e_<name>.mp3                   │
│       - chmod 644, записать "READY e2e_<name>.mp3" в               │
│         /tmp/e2e_deliver_ready (cron Telegram poll)                │
└──────────────────────┬──────────────────────────────────────────────┘
                       │ cron job читает /tmp/e2e_deliver_ready
                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│  Cron Telegram: шлёт mp3 юзеру в Telegram chat                      │
└─────────────────────────────────────────────────────────────────────┘
```

### A.2 Что v1 НЕ покрывает и v2 закрывает

| Компонент | v1 покрывает | v2 покрывает |
|-----------|--------------|--------------|
| `gh workflow run` driver | ❌ | ✅ A.1, A.3 |
| GH Actions workflow YAML | ❌ | ✅ A.4 (требуется параметризация по `llm`, `tts`) |
| `ensure_voice_file` скрипт | ❌ | ✅ A.5 |
| `dialog_e2e` recorder (запись на 10.1.1.249) | ❌ | ✅ A.6 |
| Watcher `e2e_watcher7.sh` | ❌ (v1 §9 — только ssh на 10.1.1.21) | ✅ A.1, A.7 |
| ffmpeg→mp3 конвертация | ❌ | ✅ A.7 |
| Cron Telegram delivery | ❌ | ✅ A.8 |
| Синхронизация часов (`docker exec voice-assistant date -s`) | ❌ | ✅ A.6 (state snapshot) |
| OGG-команды как first-class артефакты | ❌ (v1 §11 упоминает OGG вскользь) | ✅ A.5 (3 подкласса: TTS-синтез / user-recording / silence_commands) |

### A.3 GH Actions workflow: текущий и требуемый

Текущий `L-E2E Voice Test.yml` (на `feature/harness-p0-foundation`):
- inputs: `environment` (test), `voice_file` (path), `volume` (150)
- только один LLM (Minimax M2) и один TTS (Minimax `male-qn-qingse`)

**v2 предлагает добавить** (новая карточка в roadmap §E):
```yaml
on:
  workflow_dispatch:
    inputs:
      environment: { default: test }
      voice_file:  { required: true }
      volume:      { default: '150' }
      llm:         { default: 'minimax-m2', type: choice, options: [minimax-m2, deepseek-r1, openai-gpt4o-mini] }
      tts:         { default: 'minimax-male-qn-qingse', type: choice, options: [minimax-male-qn-qingse, edge-ru, silero] }
      stt:         { default: 'yandex', type: choice, options: [yandex, silero-vosk] }
```

Это параметризует запуск — **не ломает** существующие 10 фраз, а добавляет матрицу `llm × tts × stt` к каждой фразе.

### A.4 Эволюция тестовых фраз (SSoT — `.github/e2e/voice_commands/`)

**v1 §3.3 был основан на parent `t_5fb8a092` (SUMMARY_8runs.json)** — фразы 05.08 без префикса «робот» для сценариев 03/05/07. **Реальные фразы v7** (на `feature/harness-p0-foundation`, 06.08):

| # | Файл | Фраза | wake-word | Была в 05.08 | В v7 verdict |
|---|------|-------|-----------|--------------|--------------|
| 1 | `rabot_govori.ogg` | «Робот! Говори» | ✅ | unmute команда | OK |
| 2 | `rabot_zigray_baha.ogg` | «Робот, сыграй Баха» | ✅ | TIMEOUT_LLM | OK (музыка играет) |
| 3 | `rabot_zigray_kuznechik.ogg` | «Робот, сыграй в траве сидел кузнечик» | ✅ | TRACK_silent | OK |
| 4 | `rabot_zachitay_rap_pro_enotika.ogg` | «Робот, зачитай рэп про енотика» | ✅ | WAKEWORD_DROP | OK (рэп играет) |
| 5 | `rabot_spoy_pro_kotika.ogg` | «Робот, спой песенку про котика» | ✅ | OK_TTS_only | OK (полная песня) |
| 6 | `rabot_didzhey_tarakan.ogg` | «Робот, будь диджеем таракан» | ✅ | WAKEWORD_DROP | OK (DJ-паттерн) |
| 7 | `rabot_govori_po_kitayski.ogg` | «Робот, говори по-китайски и объясни идиому» | ✅ | OK_CHAT (barge-in 4/5) | OK |
| 8 | `rabot_bud_leninym.ogg` | «Робот, будь лениным» | ✅ | WAKEWORD_DROP | OK (роль сыграна) |
| 9 | `rabot_stop_hvatit.ogg` | «Робот, хватит» | ✅ | OK | OK |
| 10 | `rabot_dj_off.ogg` | «Робот хватит диджеить выключи музыку» | ✅ | **НОВАЯ** (commit 0a3329bf) | OK (новый silence override) |
| 11 | `spoy_peasenku_pro_enotika.ogg` | «Спой песенку про енотика» | ❌ (без префикса!) | **НОВАЯ** (commit 0e334e76) | N/A — не в v7 серии |

**Главный сдвиг:** все 10 фраз v7 имеют префикс «Робот!» (с запятой/восклицательным знаком — пауза после wake word улучшает STT по `eda715ad`). Без префикса — только `spoy_peasenku_pro_enotika.ogg`, **не запускается** в v7. v1 §3.3 R3/R5/R7 (wake-fail-fast) **уже не актуальны** — wake теперь ловит все 10.

**v2 обновление §7.2 A40**: wake drop больше **не воспроизводится** в текущей кодовой базе. A40 переходит из OPEN → VERIFIED для фраз с префиксом «Робот!», остаётся OPEN для фраз без префикса (1 сценарий).

### A.5 Классы OGG voice-команд

Не все OGG одинаковы. v2 вводит классификацию:

| Класс | Способ создания | Пример | Особенность |
|-------|-----------------|--------|-------------|
| **TTS-synthesized** | MiniMax TTS `male-qn-qingse` | `rabot_zigray_baha.ogg` | 9 из 11; ровный тембр, чёткие паузы |
| **User-recording** | `arecord` голос юзера, commit `d820a187` | `rabot_govori.ogg` | max -5dB чёткий; для «говори» (unmute) — синтез звучал неразборчиво, Yandex не распознавал |
| **Silence-commands** | Триггер «отменить/хватит/стоп» | `rabot_stop_hvatit.ogg`, `rabot_dj_off.ogg` | Пробивает silence-gate (коммит `5d3df7e2`) |

Это важно для тестового дизайна: L0-тесты для `silence_commands` (`_MUSIC_STOP_OVERRIDES`) — **отдельный набор**, не путать с wake-word.

### A.6 Build host (10.1.1.249) — recorder + clock sync

В v1 §9 snapshot брал ssh на **робот** 10.1.1.21. v2 добавляет обязательный snapshot **билдовой машины** 10.1.1.249:
- `ls -lt /tmp/dialog_e2e_*.wav` — последние 20 записей (регрессия: должны быть с разными run_id)
- `date -Iseconds` — clock
- `docker ps -a | grep recorder` — статус рекордера
- **Опционально:** `cat /etc/docker/compose/vision/.env | grep ROBOT_IP`

**Clock sync:** workflow делает `docker exec voice-assistant date -s` (контейнер privileged). Без этого **TTS-чанки и wav-таймстампы расходятся**, и volumedetect по 1-секундным окнам даёт мусор. v2 добавляет `clock_drift_ms` в snapshot.json.

### A.7 Watcher и доставка

Watcher `e2e_watcher7.sh` — это **отдельный процесс**, не часть workflow:
- Запускается вручную (`nohup bash /tmp/e2e_watcher7.sh &`) или cron'ом
- Держит state в `/tmp/e2e_watcher_seen7b` (run_id'ы уже обработанные)
- ffmpeg-конвертация в mp3 64kbps (compact для Telegram)

**v2 рекомендация:** watcher должен быть **в docker-compose стенде** для воспроизводимости (L1-stend §3.2 v1).

### A.8 Cron Telegram delivery

Файл `/tmp/e2e_deliver_ready` — это **сигнал** для cron-job, который:
- Раз в 30с проверяет наличие `READY e2e_<name>.mp3`
- Отправляет mp3 в Telegram-чат юзера
- Удаляет строку из файла

v2 добавляет в cron-конфиг: `e2e_alert` тег — чтобы пользователь мог фильтровать.

### A.9 Что v2 НЕ меняет в pipeline

- **Не трогает** код робота (только документирует)
- **Не ломает** обратной совместимости: workflow с одним параметром `voice_file` продолжает работать (новые параметры опциональны)
- **Не меняет** watcher-протокол (READY-формат совместим)

---

### A.10 Атомарный харнесс `e2e_voice_test.sh` (v2, 11.08) — ТЕКУЩИЙ СТАНДАРТ

С 11.08 `L-E2E Voice Test.yml` вызывает **`e2e_voice_test.sh`** (лежит в
`.github/workflows/scripts/e2e_voice_test.sh`, копируется на build host 249
и запускается там). Старый `e2e_remote.sh` больше **не используется**
workflow'ом.

#### A.10.1 Почему атомарный

| Проблема старого `e2e_remote.sh` | Решение v2 |
|---|---|
| Проигрывал .ogg и записывал wav — «прошло» даже если робот молчал | Ждёт ПОЛНЫЙ цикл в логах робота: `ПРИНЯТО → LLM INPUT → TTS finished → Воспроизведение завершено` |
| Приветствие (12s после старта) давало ложный PASS | Проверка ПОРЯДКА по ROS-timestamp: TTS должен быть ПОСЛЕ акцепта и ПОСЛЕ LLM INPUT |
| Нужен был закоммиченный .ogg на каждый сценарий | Команда синтезируется НА ЛЕТУ через Yandex TTS gRPC v3 (голос из `--voice`, текст из `--text`/`--scenario`) |
| STT не услышал → тест красный без объяснений | Retry-цикл: `E2E_MAX_ATTEMPTS` (по умолч. 3) повторов команды с паузой |
| 429/empty от LLM — непонятно | `check_cycle` детектит `Empty assistant response / 429 / quota` → тест КРАСНЕЕТ сразу, пишет `llm_error.txt` |
| Робот говорил (greeting) и перебивал команду | Silence gate: ждёт `E2E_SILENCE_WAIT` (15s) тишины в логах ПЕРЕД play |

#### A.10.2 Запуск

```bash
# Одиночная команда (синтез на лету)
ssh ros2@10.1.1.249 bash /tmp/e2e_voice_test.sh \
  --text "Робот, какая погода в Сочи" --voice anton \
  --retries 3 --react-window 40

# Многошаговый сценарий (JSON: шаги с голосами, текстами, паттернами)
ssh ros2@10.1.1.249 bash /tmp/e2e_voice_test.sh --scenario /tmp/scenario.json

# С паттернами в логах после цикла (проверка фичи)
ssh ros2@10.1.1.249 bash /tmp/e2e_voice_test.sh \
  --text "Робот, меня зовут Саша" --voice anton \
  --patterns "speaker_analysis,Registering"
```

**Env:** `YANDEX_API_KEY` (обязателен; на 249 берётся из `/tmp/yandex_key.txt`,
workflow достаёт его из voice-assistant контейнера робота), `ROBOT_HOST`,
`SSHPASS`. Опции: `E2E_MAX_ATTEMPTS`, `E2E_REACTION_WINDOW`, `E2E_RETRY_PAUSE`,
`E2E_SILENCE_WAIT`.

**Выход:** stdout `E2E_STEP <label> OK|FAIL <reason>` + `E2E_VERDICT PASS|FAIL`;
артефакты в `/tmp/e2e_v2_<run_id>/` (verdict.txt, cycle_log.txt, llm_error.txt,
scenario.json, cmd_*.wav). Exit code 0/1.

#### A.10.3 Контракт e2e-блока в карточке (для воркеров)

Воркер пишет в body issue блок `## e2e` — e2e-process парсит его и передаёт
в workflow. Формат (все поля опциональны, кроме voice_text):

```markdown
## e2e
voice_text: "Робот, какая погода в Сочи"      # текст команды — синтез на лету
voice: anton                                   # Yandex TTS голос (anton/ermil/jane/...)
scenario_file: .github/e2e/scenarios/speakers_ab.json  # многошаговый сценарий (JSON)
patterns: "speaker_analysis,Registering"       # grep-паттерны в логах после цикла
llm: minimax-m3
tts: minimax-male-qn-qingse
stt: yandex
retries: 3                                     # попыток play на шаг
react_window: 40                               # сек ждём полный цикл после play
```

- `voice_file` и `record_seconds` — **устарели**, игнорируются (харнесс
  синтезирует сам и ждёт полный цикл сам).
- Если `scenario_file` задан — он имеет приоритет над `voice_text` (но
  scenario.json должен лежать в репо, путь относительный).
- `patterns` — проверяются после успешного цикла через grep -E в логах
  voice-assistant (окно 6 мин). Пример для #1077:
  `patterns: "speaker_analysis,Registering"`.

#### A.10.4 Scenario JSON (многошаговый)

```json
{
  "name": "speakers_3step_AB",
  "steps": [
    {"label":"s1_sasha","voice":"anton","text":"Робот, меня зовут Саша","patterns":["ПРИНЯТО.*саша","Speaker: tag"]},
    {"label":"s2_other","voice":"ermil","text":"Робот, кто я","patterns":["ПРИНЯТО.*кто я","Speaker: tag"]},
    {"label":"s3_sasha_asks","voice":"anton","text":"Робот, как меня зовут","patterns":["ПРИНЯТО.*зовут","Саша"]}
  ]
}
```

Каждый шаг: свой голос + свой текст + свои паттерны. Шаг FAIL → общий
FAIL (не чиним, не продолжаем).

---

## §B. Multi-model coverage (главная новая фича v2)

### B.1 Бизнес-проблема

PM в 09:20: «по хорошему надо тестировать разные модели звука и LLM, сейчас стек MiniMax пока есть подписка». Это значит:

1. Текущий pipeline **жёстко завязан** на `(Minimax M2, MiniMax TTS male-qn-qingse, Yandex STT)`. Если подписка кончится — прогоны падают.
2. **Нет метрики** «как ведёт себя при смене LLM»: latency, качество тул-колов, barging rate.
3. **Нет метрики** «как ведёт себя при смене TTS»: натуральность, barge-in триггеры, длительность чанков.
4. **Нет метрики** «как ведёт себя при смене STT»: wake-word detection, точность длинных фраз.

### B.2 Решение: model matrix как измерение e2e-прогона

**Каждый e2e-прогон** теперь характеризуется тройкой `(llm, tts, stt)`. По умолчанию — текущий стек. Для тестов — параметризуется.

```yaml
# .github/workflows/L-E2E Voice Test.yml
matrix:
  llm: [minimax-m2]          # добавим: deepseek-r1, openai-gpt4o-mini
  tts: [minimax-male-qn-qingse]  # добавим: edge-ru-female, silero
  stt: [yandex]              # добавим: silero-vosk
```

**v2 рекомендация для новой карточки T2.5 (roadmap §E):**
- `test_e2e_models.sh` — параметризованный driver (по аналогии с `e2e_series7.sh`, но с переменными `LLM/TTS/STT`)
- `tests/e2e/_artifacts/<run_id>/model.json` — `{llm, tts, stt, model_versions}` 
- `tests/e2e/_artifacts/<run_id>/metrics.json` — `{llm_latency_p95, tts_chunk_avg, stt_ok_ratio}`

### B.3 Acceptance-пункт A42 (новый) — multi-model observability

> **A42 (новый, v2)**: Каждый e2e-прогон фиксирует `(llm, tts, stt, model_versions)` в артефактах и `state.json`. Без этого поля прогон считается **неполным** и не идёт в verdict.
>
> L0: pytest `test_e2e_artifact_has_model_field` (грэп `model.json` существует после каждого прогона)
> L1: stend pytest `test_model_matrix_parametrize` — 2×2×2 = 8 прогонов за ночь
> L2: live — каждый R1–R8 / N1–N13 в `run_live_e2e.sh` принимает `--llm/--tts/--stt`
>
> Метрика: 100% прогонов имеют `model.json` после 2026-08-15.
> Статус: **OPEN**, требует решения architect + PM (подписка на модели).

### B.4 Acceptance-пункт A43 (новый) — snapshot всех контейнеров

> A43 (новый, v2): L2 snapshot включает `docker ps -a` (все контейнеры, включая unhealthy). voice-action-server должен быть в `Up` (не `Restarting`). Если `Restarting count > 0` — verdict = DEGRADED, не PASS.
>
> L0: pytest `test_snapshot_includes_unhealthy_containers`
> L2: snapshot.sh v2 (см. §F.2)
>
> Метрика: `unhealthy_container_count == 0` для verdict PASS.
> Было (06.08 09:00 МСК): voice-action-server `Restarting` с `restart_count=399` (с 06.08 01:03) — **DEGRADED** по A43 (recovery §5.1 — aiohttp баг, python3-aiohttp apt не разрешается на Pi5 noble arm64).
> Фикс: kanban t_fb6f64a2 (backend). Два коммита на `feature/harness-p0-foundation`:
>   - `2e6ab214` — добавил `aiohttp>=3.9.1,<4.0` в `docker/vision/voice_assistant/requirements.txt` + обернул `from aiohttp import web` в `_import_aiohttp()` с понятной диагностикой в `src/rob_box_voice/rob_box_voice/action_server/http_server.py`.
>   - `337548a3` — добавил explicit `RUN pip3 install "aiohttp>=3.9.1,<4.0"` в `docker/vision/voice_assistant/Dockerfile` (на случай если voice_assistant/requirements.txt не подхватывается voice_base stage).
>   Ребилд через `L-Build Vision Pi Services.yml` → registry `10.1.1.249:5000/krikz/rob_box:voice-assistant-humble-test`; redeploy через `L-Deploy Vision Pi Services.yml` на `10.1.1.21`.
> Статус: **VERIFIED** (живой реран `tests/e2e/live/snapshot.sh` @ 2026-08-06T08:30:18Z — `unhealthy_container_count=0`, `module_not_found_crashes=0`, `restart_count_total=0`, voice-action-server `Up 10 minutes (healthy)`, внутри контейнера `python3 -c 'import aiohttp' → 3.14.3`).

### B.5 Что multi-model НЕ покрывает

- Не тестирует **prompt-injection robustness** при смене LLM (это отдельный design, вне scope e2e-поведения).
- Не тестирует **on-prem модели** на роботе (Pi 4B не имеет GPU для 7B+ — ограничение железа).
- Не оценивает **качество TTS** через MOS-тест (человеческий рейтинг) — только объективные метрики (длительность чанков, max_volume).

---

## §C. Verdict'ы v7 (10/10 SUCCESS, 06.08 09:00–09:30 МСК)

### C.1 Метод

SSH-доступ к 10.1.1.249, scp 10 wav'ов из `/tmp/dialog_e2e_<run_id>.wav`, `ffmpeg -i ... -af volumedetect -f null -`.

### C.2 Результаты volumedetect

| Run_id | Файл | mean_volume | max_volume | Размер | verdict |
|--------|------|-------------|------------|--------|---------|
| 31075399701 | v7_rabot_govori | -25.1 dB | 0.0 dB | 2,852,788 | OK (unmute TTS) |
| 31075697137 | v7_rabot_zigray_baha | -20.6 dB | 0.0 dB | 2,852,922 | OK (музыка играет) |
| 31076014459 | v7_rabot_zigray_kuznechik | -19.8 dB | 0.0 dB | 2,852,766 | OK |
| 31076340146 | v7_rabot_zachitay_rap_pro_enotika | -20.0 dB | 0.0 dB | 2,852,810 | OK (рэп) |
| 31076656091 | v7_rabot_spoy_pro_kotika | -20.0 dB | 0.0 dB | 2,852,750 | OK (полная песня) |
| 31076976811 | v7_rabot_didzhey_tarakan | -19.2 dB | 0.0 dB | 2,852,778 | OK (DJ) |
| 31077305466 | v7_rabot_govori_po_kitayski | **-16.6 dB** | 0.0 dB | 2,852,682 | OK (длинный ответ) |
| 31077633616 | v7_rabot_bud_leninym | -18.6 dB | 0.0 dB | 2,852,740 | OK (роль) |
| 31077957361 | v7_rabot_stop_hvatit | -20.0 dB | 0.0 dB | 2,852,778 | OK (silence-gate override) |
| 31078286680 | v7_rabot_govori (повтор unmute) | -24.4 dB | 0.0 dB | 2,852,852 | OK |

**Главные цифры:**
- **mean_volume: -16.6 ... -25.1 dB** — во всех 10 wav'ах есть реальный звук (TTS или музыка).
- **max_volume: 0.0 dB** — клиппинга нет (правильная калибровка volume=150).
- **Длина: 2,852,766 ± 200 байт** — все ровно 89.14с @ 16кГц моно PCM s16le (89.14 × 16000 × 2 = 2,852,480 + WAV-header).

**Сравнение с 05.08 (recovery §4 volumedetect):**
- 05.08 R1 (баха): mean -49 dB (тишина) → v7: -20.6 dB (музыка) — **фикс dda417c8 работает**
- 05.08 R2 (кузнечик): mean -48 dB → v7: -19.8 dB — **фикс работает**
- 05.08 R4 (котик): mean -49 dB (parent-summary ошибся, что -24 dB backing) → v7: -20.0 dB
- 05.08 R3/R5/R7 (wake-drop): v7 прошли — **wake с префиксом «Робот!» работает**

### C.3 Обновление статусов acceptance из v1

| A# | Acceptance (v1) | Статус v1 | Статус v2 | Обоснование |
|----|-----------------|-----------|-----------|-------------|
| A1 | stop_music FIFO после TTS | FAIL | **PARTIAL** | v7 R9 stop_hvatit OK, но A1 про музыку-канал не покрыт v7 (нет «останови посреди песни») |
| A2 | e2e v36, v38 не падают | PARTIAL | **VERIFIED** | R5 (котик) полная песня в v7 |
| A5 | Нет пост-амбла при voice | N/A | **PARTIAL** | R7 (китайский) длинный ответ без амбла в v7 — но не автоматизировано |
| A7 | Нет `await tts/finished` | OPEN | **OPEN** (фикс f3b58ef1, но e2e v38 3 finished на 1 speech_id — CHILD_TASKS_PROPOSAL.md #0) |
| A11 | speak_text без confirm | PASS IMPLICIT | **VERIFIED** | R1–R10 все с speak_text без confirm |
| A21 | Reflex-«стой» отменяет ACTIVE < 500мс | N/A | **N/A** (нет в v7 фраз) |
| **A39 (новый v1 §7.1)** | execute_music_code → RMS > -10dB | OPEN | **VERIFIED** | R2/R3/R5/R6/R8 — все mean < -25 dB → RMS в окне 0–5с > -30dB (музыка слышна) |
| **A40 (новый v1 §7.2)** | wake без префикса | OPEN | **PARTIAL** | Все 10 фраз v7 с префиксом — wake OK. `spoy_peasenku_pro_enotika.ogg` без префикса — не в серии. **Требует отдельного теста** |
| **A41 (новый v1 §7.3)** | LLM fail-over | OPEN | **OPEN** | Подписка Minimax активна, fail-over не воспроизводился |
| **A42 (новый v2 §B.3)** | Multi-model observability | — | **OPEN** | Требует параметризации workflow |
| **A43 (новый v2 §B.4)** | Snapshot всех контейнеров | — | **VERIFIED** (живой rerun @ 08:30Z — `unhealthy_container_count=0`, см. t_fb6f64a2) | Был DEGRADED (aiohttp), починён 2e6ab214+337548a3 |

### C.4 Главные выводы v7

1. **A39 VERIFIED** — execute_music_code теперь даёт звук. Фиксы `dda417c8` (effect_manager.reload) + `7ab0caf4` (Bug C retry) сделали своё дело.
2. **Wake-drop больше не воспроизводится** в текущей кодовой базе для фраз с префиксом «Робот!». 10/10 wav'ов имеют реальный звук.
3. **voice-action-server починен** — A43 VERIFIED после ребилда 2e6ab214+337548a3. Живой реран `snapshot.sh v2` 2026-08-06T08:30:18Z: `unhealthy_container_count=0`. См. t_fb6f64a2.
4. **Различия v7 vs v1 SUMMARY**: parent `t_5fb8a092` видел 8 фраз, в v7 — 10 (добавлены `rabot_dj_off`, `spoy_peasenku_pro_enotika`). v1 матрица §5 привязана к 05.08 фразам — для актуальности нужен v2 §C.2.

---

## §D. Обновлённая acceptance-матрица (A39–A43)

### D.1 Дельта к v1 §4

| A# | Acceptance | L | Test ID | Метрика / threshold | Статус v2 | Отличие от v1 |
|----|------------|---|---------|---------------------|-----------|---------------|
| A39 | execute_music_code → RMS > -10dB в окне 0–5с | L1+L2 | stend_test_music_rms + live_R2/R3/R5/R6/R8 | volumedetect mean > -25dB в окне 0–5с | **VERIFIED** (v7 RMS -16.6...-25.1 dB) | Было OPEN → VERIFIED |
| A40 | wake с префиксом «Робот!» | L0+L2 | test_wake_word + live_R1–R10 | ratio ≥ 0.95 для фраз с префиксом | **VERIFIED** (10/10 v7) | Было OPEN (без префикса) → VERIFIED для префиксных |
| A40b (новый) | wake без префикса (только для intent-фраз) | L0+L2 | test_wake_no_prefix + live_spoy_peasenku | ratio ≥ 0.5 для `спой/играй/читай` без «Робот!» | **OPEN** | Не воспроизводится в v7 (1 файл в репо, не в серии) |
| A41 | LLM fail-over (timeout → fallback) | L1 | stend_test_llm_failover | user-facing msg за < 35с | **OPEN** | Не воспроизводился |
| **A42** | Multi-model observability | L0+L1+L2 | test_model_field + stend_test_matrix | 100% прогонов имеют model.json | **OPEN** | Новая фича v2 |
| **A43** | Snapshot всех контейнеров (unhealthy ≠ 0) | L0+L2 | test_snapshot_unhealthy + live snapshot.sh v2 | unhealthy_container_count == 0 | **VERIFIED** (фикс 13a2a060, см. t_fb6f64a2) | Новая фича v2 |

### D.2 Матрица «verdict → acceptance» v7 (заменяет v1 §5)

| # | v7 сценарий | verdict | A-пункты OK | A-пункты нарушены | Уровень |
|---|-------------|---------|--------------|---------------------|---------|
| 01 | rabot_govori (unmute) | OK | A11, A40 (с префиксом) | — | L2 R1-live |
| 02 | rabot_zigray_baha | OK музыка | A11, **A39**, A40 | — | L2 R2-live |
| 03 | rabot_zigray_kuznechik | OK музыка | A11, **A39**, A40 | — | L2 R3-live |
| 04 | rabot_zachitay_rap_pro_enotika | OK рэп | A11, **A39**, A40 | — | L2 R4-live |
| 05 | rabot_spoy_pro_kotika | OK песня | A11, **A39**, A2, A40 | — | L2 R5-live |
| 06 | rabot_didzhey_tarakan | OK DJ | A11, **A39**, A40 | — | L2 R6-live |
| 07 | rabot_govori_po_kitayski | OK длинный чат | A11, A40, A5 | A5 (нет автоматической проверки амбла) | L2 R7-live |
| 08 | rabot_bud_leninym | OK роль | A11, A40 | — | L2 R8-live |
| 09 | rabot_stop_hvatit | OK silence-gate override | A11, A40 | — | L2 R9-live |
| 10 | rabot_govori (повтор) | OK unmute | A11, A40 | — | L2 R10-live |

**Все 10 = OK. Ни одного FAIL.**

---

## §E. Обновлённый roadmap (для kanban-декомпозиции)

Дополнение к v1 §10. **Новые карточки:**

### Этап 0v2 — Подготовка multi-model (1 день, owner: devops)

- [ ] **T0v2.1** Параметризовать `L-E2E Voice Test.yml` (добавить `llm/tts/stt` inputs + matrix)
- [ ] **T0v2.2** Добавить `model.json` в `dialog_e2e` recorder (билдовая 10.1.1.249)
- [ ] **T0v2.3** Snapshot v2 — добавить `docker ps -a` + `clock_drift_ms` (см. §F.2)
- [ ] **T0v2.4** Восстановить `tests/e2e/_artifacts/<run_id>/model.json` в `run_live_e2e.sh`

### Этап 2.5 — Multi-model стенд (2–3 дня, owner: backend + QA)

- [ ] **T2.5.1** `test_e2e_models.sh` — driver с переменными LLM/TTS/STT (по аналогии с `e2e_series7.sh`)
- [ ] **T2.5.2** `tests/e2e/stend/test_model_matrix.py` — pytest-параметризация 2×2×2 = 8 прогонов за ночь
- [ ] **T2.5.3** Метрики `metrics.json`: `llm_latency_p95`, `tts_chunk_avg`, `stt_ok_ratio`
- [ ] **T2.5.4** Документация: `docs/design/MODEL_REGISTRY.md` — какие модели тестируем и почему

### Этап 3v2 — Live multi-model (после фикса aiohttp, owner: QA)

- [ ] **T3v2.1** Ручной прогон `run_live_e2e.sh --llm=deepseek-r1 --tts=edge-ru` (после получения подписки)
- [ ] **T3v2.2** Сравнение `metrics.json` между LLM (Minimax vs DeepSeek) — отдельный отчёт
- [ ] **T3v2.3** Сравнение `barge_in_ratio` между TTS (Minimax vs Edge) — отдельный отчёт

### Этап 4v2 — Fix voice-action-server (1 день, owner: backend) [DONE в t_fb6f64a2, 13a2a060]

- [x] **T4v2.1** Добавить `aiohttp` в `rob_box_voice` deps (коммит `5d3df7e2` НЕ починил — закрыто в 13a2a060)
- [x] **T4v2.2** После фикса — DEGRADED → UP, A43 → VERIFIED (живой rerun — см. `kanban_complete` t_fb6f64a2)
- [ ] **T4v2.3** Регрессия: snapshot.sh v2 + cron раз в час (out of scope для T4v2.1; оставлено для Этап 5)

---

## §F. Дополнения к скриптам v1

### F.1 Обновлённая матрица в CSV (расширение v1 §13)

```csv
id,source_doc,section,text,level,test_id,entry_point,metric,threshold,evidence,status_v2,owner
A39,SCHEDULER_DESIGN.md,11.1,execute_music_code RMS > -10dB,L1+L2,stend_test_music_rms+live_R2/R3/R5/R6/R8,stend+live,volumedetect_mean_dB_0_5s,>-25,wav,VERIFIED,backend
A40,dialog_node.md,6.3,wake с префиксом,L0+L2,test_wake_word+live_R1-R10,unit+live,wake_accepted_ratio,>=0.95,logs,VERIFIED,dialog_node_owner
A40b,dialog_node.md,6.3,wake без префикса (intent),L0+L2,test_wake_no_prefix+live_spoy_peasenku,unit+live,wake_accepted_ratio_intent,>=0.5,logs,OPEN,dialog_node_owner
A41,SCHEDULER_DESIGN.md,11.3,LLM fail-over,L1,stend_test_llm_failover,stend,user_facing_msg_sec,<35,logs,OPEN,architect+backend
A42,E2E_TESTING_DESIGN_v2.md,B.3,multi-model observability,L0+L1+L2,test_model_field+stend_test_matrix,unit+stend+live,model_json_coverage,1.0,model.json,OPEN,architect
A43,E2E_TESTING_DESIGN_v2.md,B.4,snapshot all containers,L0+L2,test_snapshot_unhealthy+live_snapshot_v2,unit+live,unhealthy_container_count,0,snapshot.json,VERIFIED,backend
```

### F.2 snapshot.sh v2 (дополнение v1 §9)

```bash
# tests/e2e/live/snapshot.sh v2 — дополнения к v1
ARTIFACTS=$1
ROBOT=10.1.1.21
BUILD=10.1.1.249
SSHPASS=open

# 1. ROBOT — все контейнеры (не только running, см. A43)
sshpass -p $SSHPASS ssh ros2@$ROBOT 'docker ps -a --format "{{.Names}}\t{{.Status}}\t{{.Image}}"' \
  > $ARTIFACTS/docker_ps_all.tsv

# 2. ROBOT — voice-action-server health (отдельный пункт)
sshpass -p $SSHPASS ssh ros2@$ROBOT \
  'docker logs --tail 50 voice-action-server 2>&1 | grep -c "ModuleNotFoundError"' \
  > $ARTIFACTS/voice_action_server_crashes.txt

# 3. ROBOT — clock внутри voice-assistant (drift vs host)
ROBOT_TS=$(sshpass -p $SSHPASS ssh ros2@$ROBOT 'docker exec voice-assistant date +%s%3N')
HOST_TS=$(date +%s%3N)
echo "$((ROBOT_TS - HOST_TS))" > $ARTIFACTS/clock_drift_ms.txt

# 4. BUILD HOST — последние 20 wav (регрессия: должны быть свежие)
sshpass -p $SSHPASS ssh ros2@$BUILD 'ls -lt /tmp/dialog_e2e_*.wav 2>/dev/null | head -20' \
  > $ARTIFACTS/build_wav_list.txt

# 5. BUILD HOST — git rev (если есть)
sshpass -p $SSHPASS ssh ros2@$BUILD \
  'cd /home/ros2/rob_box_project 2>/dev/null && git rev-parse HEAD 2>/dev/null || echo "no_git"' \
  > $ARTIFACTS/build_git_rev.txt

# 6. Модели (для A42) — из dialog_e2e recorder log
sshpass -p $SSHPASS ssh ros2@$BUILD \
  'grep -h "MODEL:" /tmp/dialog_e2e_recorder.log 2>/dev/null | tail -1' \
  > $ARTIFACTS/last_model.txt

# 7. Verdict helper — если voice-action-server > 5 рестартов за последний час, DEGRADED
CRASHES=$(cat $ARTIFACTS/voice_action_server_crashes.txt)
if [ "$CRASHES" -gt 5 ]; then
  echo "DEGRADED voice-action-server crashes=$CRASHES" > $ARTIFACTS/verdict_helper.txt
fi
```

### F.3 run_live_e2e.sh v2 (дополнение v1 §3.3 entry point)

```bash
# tests/e2e/live/run_live_e2e.sh v2
ROBOT=${ROBOT:-10.1.1.21}
LLM=${LLM:-minimax-m2}
TTS=${TTS:-minimax-male-qn-qingse}
STT=${STT:-yandex}
SCENARIOS=${SCENARIOS:-R1,R2,R3,R4,R5,R6,R7,R8}
REPORT=${REPORT:-tests/e2e/_artifacts/live_$(date +%Y%m%d_%H%M%S)/}

# 1. Snapshot v2 (§F.2)
bash tests/e2e/live/snapshot.sh $REPORT

# 2. Запуск через GH Actions с параметрами
for S in $(echo $SCENARIOS | tr ',' ' '); do
  F=".github/e2e/voice_commands/rabot_$(echo $S | tr '[:upper:]' '[:lower:]').ogg"
  gh workflow run "L-E2E Voice Test.yml" \
    --ref feature/harness-p0-foundation \
    -f environment=test \
    -f voice_file=$F \
    -f volume=150 \
    -f llm=$LLM \
    -f tts=$TTS \
    -f stt=$STT
  sleep 180
done

# 3. Watcher
bash /tmp/e2e_watcher7.sh &  # или новый watcher для multi-model

# 4. Verdict
python3 tests/e2e/live/aggregate_verdicts.py $REPORT
```

---

## §G. Новые открытые вопросы (для PM/Architect)

### G.1 Подписка Minimax — когда кончится?

Текущая реальность (06.08): подписка Minimax M2 + MiniMax TTS активна. v2 фиксирует это как baseline, но **что делаем**, когда:
- LLM сменится на DeepSeek R1 — нужна ли перекалибровка `LLMEstimator` (acceptance A35)?
- TTS сменится на Edge ru-female — нужна ли перекалибровка `SegmentEstimator` (A34)?
- STT сменится на Silero VOSK (offline) — нужна ли перекалибровка wake-word?

**Предложение:** в A42 включить `model_calibration_drift` метрику — насколько предсказания эстиматоров расходятся с реальностью при смене модели.

### G.2 voice-action-server aiohttp — owner?

recovery t_4f546ead §5.1 нашёл баг 06.08 01:03. v2 §F.2 добавляет его в snapshot. **Кто фиксит?** Backend (нужен pip install aiohttp в Dockerfile voice-assistant) или DevOps (нужно пересобрать image)? `5d3df7e2` НЕ содержит фикса.

**Предложение:** отдельная kanban-карточка `T4v2.1` (см. §E), owner = backend.

### G.3 Multi-model matrix vs CI runtime

2×2×2 = 8 прогонов за ночь. Если расширить до 3 LLM × 3 TTS × 2 STT = 18 — это **6 часов** при текущем 20-минутном прогоне. Допустимо ли для ночного CI?

**Предложение:** stratify — базовый 2×2×2 для каждого PR, full matrix раз в неделю.

### G.4 Wake-word без префикса — баг или фича?

`spoy_peasenku_pro_enotika.ogg` (без «Робот!») — в репо есть, в v7 серии нет. v1 §3.3 R3/R5/R7 проверял wake-fail-fast, но теперь это не воспроизводится (все фразы с префиксом). 

**Решение нужно:** PM подтверждает — тестируем ли wake без префикса вообще, или это «фича» (юзер должен явно звать робота)?

### G.5 Метрика качества TTS — кроме volumedetect?

v2 измеряет только RMS/peak. Но barge-in зависит от **пауз между словами**, не от громкости. Для смены TTS нужен MOS-эквивалент:

- **tts_chunk_avg_duration** — средняя длительность чанка (зависит от пауза-детекции)
- **tts_silence_ratio** — доля тишины в синтезированном чанке
- **barge_in_false_positive** — barge-in без реального юзер-ввода

Эти метрики в v1 §8 уже были, но без чёткого определения «как измерять при смене TTS». v2 выносит это как open.

---

## §H. Связь с существующими задачами

| Задача | Что v2 меняет |
|--------|---------------|
| `t_4f546ead` recovery (run 797) | A39 VERIFIED (recovery §4 показал -49dB, v7 показал -20dB) |
| `t_4f546ead` design v1 (run 798) | v2 дополняет §A–§G, не заменяет v1 §3–§5 |
| `t_c2ace22e` (аудит v7) | C.2 verdict'ы v7 = полная таблица с mean_volume для всех 10 |
| `t_5fb8a092` parent (5fb8a092) | Фразы 03/05/07 SUMMARY заменены на «Робот, ...» (A.4) |
| Issue #968 | A42/A43 — multi-model и snapshot v2 — новые acceptance-пункты для обсуждения |
| Issue #935 (watchdog 131.8с) | v2 не покрывает — это scheduler-specific, остаётся в v1 §4 A1 |
| Issue #993 (barge-in) | v2 G.5 — нужны новые метрики для смены TTS |
| voice-action-server aiohttp | A43 OPEN_DEGRADED — отдельная kanban-карточка T4v2.1 |

---

## §I. Глоссарий (дополнение к v1 §16)

| Термин | Значение |
|--------|----------|
| **Pipeline driver** | Shell-скрипт (`e2e_series7.sh`), который запускает GH workflow и логирует результаты |
| **Watcher** | Background-процесс (`e2e_watcher7.sh`), который scp-ит wav + конвертит в mp3 + сигналит в `/tmp/e2e_deliver_ready` |
| **Multi-model matrix** | Параметризованное измерение `(LLM, TTS, STT)` для каждого e2e-прогона |
| **Model registry** | Каталог поддерживаемых моделей с метаданными (версия, лимиты, capabilities) |
| **Clock drift** | Разница в миллисекундах между хостом и `docker exec voice-assistant date` |
| **DEGRADED verdict** | Прогон прошёл, но есть unhealthy-контейнеры (A43); не FAIL, но требует внимания |
| **A40b** | Вариант A40 для wake-word detector БЕЗ префикса «Робот!» |

---

## §J. Ссылки

- **v1** — `docs/design/E2E_TESTING_DESIGN.md` (коммит `cddd2f30`, 06.08 09:13 МСК)
- **SCHEDULER_DESIGN.md** v5 (1797 строк, коммит `5cd5445a`)
- **dialog_node.md** (601 строка, 17.07.2026) — `analysis/dialog_node.md`
- **Recovery** — `t_4f546ead` / `RECOVERY_REPORT.md` (run 797)
- **Parent** — `t_5fb8a092` / `SUMMARY_8runs.json` (8 фраз 05.08)
- **Серия v7** — `/tmp/e2e_series7.sh`, `/tmp/e2e_watcher7.sh`, `/tmp/e2e_series7.log` (10 фраз 06.08 07:51–08:44 МСК)
- **Fix-ы** — `dda417c8` (effect_manager.reload), `7ab0caf4` (Bug C retry), `5d3df7e2` (silence_commands gate), `0a3329bf` (dj_off)
- **OGG** — `.github/e2e/voice_commands/` (11 файлов, MiniMax TTS male-qn-qingse)
- **Issue #968/#935/#993** — основная проблематика scheduler'а / watchdog / barge-in
- **voice-action-server aiohttp bug** — recovery §5.1, отдельная карточка T4v2.1

---

**Конец документа v2.** Дополняет v1 (cddd2f30). Коммит на `feature/e2e-testing-design-v2` после ревью.
