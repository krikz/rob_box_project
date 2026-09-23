# Workflow scripts

Скрипты, вызываемые GitHub Actions workflow'ами проекта (`L: E2E Voice Test`,
`L: Deploy and Verify`, build-воркфлоу) или девопс-операциями вручную. Копии
этих файлов на build host (10.1.1.249) появляются либо через `scp` из шага
workflow, либо через ручной `rsync`/`scp` от дежурного инженера.

Помечено:

- 🟢 **active** — текущий стандарт, используется в workflow.
- 🟡 **deprecated** — старый путь, в репо для истории; workflow больше не вызывает.
- 🔧 **ops** — операционный скрипт, вызывается девопсом вручную, не в workflow.

## E2E voice test

### `e2e_voice_test.sh` — 🟢 активный, атомарный (v2)

Единственный e2e-харнесс, который вызывает `L: E2E Voice Test.yml`. Синтезирует
голосовую команду на лету, играет её, ждёт **полный цикл**
`ПРИНЯТО → LLM INPUT → TTS finished → Воспроизведение завершено` в логах
робота, ретраит команду при NO_ACCEPT, детектит LLM 429 как красный. Поддерживает
сценарии из JSON, паттерны в логах, выход `E2E_VERDICT PASS|FAIL`.

Запуск:

```bash
ssh ros2@10.1.1.249 bash /tmp/e2e_voice_test.sh \
  --text "Робот, привет как дела" --voice anton --retries 3 --react-window 40

ssh ros2@10.1.1.249 bash /tmp/e2e_voice_test.sh --scenario /tmp/scenario.json
```

Env: `ROBOT_HOST`, `SSHPASS`; ключи TTS — по выбранному провайдеру (см. ниже).
Подробности — `docs/design/E2E_TESTING_DESIGN_v2.md` §A.10.

#### Выбор TTS-провайдера (`--tts-provider`)

Речь про провайдера, которым **билд-машина озвучивает команду в колонку**, а не
про `tts_node` на роботе. Раньше харнесс умел только Yandex: когда доступ к
папке Yandex Cloud отвалился (`PERMISSION_DENIED`), каждый шаг каждого прогона
падал `FAIL synth`, и робота при этом никто не спрашивал (run 35533542706).

| Значение  | Чем синтезирует            | Ключ             |
|-----------|----------------------------|------------------|
| `auto` ⭐ | проба по очереди, первый живой | по ситуации  |
| `yandex`  | SpeechKit v3 (gRPC)        | `YANDEX_API_KEY` |
| `minimax` | T2A v2 (HTTP)              | `MINIMAX_API_KEY`|
| `silero`  | локально, torch на 249     | не нужен         |

```bash
# прибить провайдера явно (никакой подмены — упадёт именно на нём)
ssh ros2@10.1.1.249 bash /tmp/e2e_voice_test.sh --scenario /tmp/scenario.json \
  --tts-provider silero
```

`auto` (дефолт) один раз за прогон проходит `E2E_TTS_PROVIDER_ORDER`
(по умолчанию `yandex,minimax,silero`) и берёт первого, кто синтезирует пробную
фразу; проба одна на прогон, а не на шаг. Явно заданный провайдер **не
подменяется** живым соседом — иначе прогон был бы зелёным, проверив не то, что
просили. Silero локальный и без ключей, поэтому «облака легли» больше не равно
«e2e красный».

Голоса сценариев названы по-яндексовски (`anton`/`ermil`/`zahar`/`filipp`);
`map_tts_voice` в `e2e_voice_lib.sh` переводит их в каталог выбранного
провайдера, сохраняя **различимость** — иначе диаризация в
`night_marathon` act2/act3 проверяла бы один голос вместо четырёх.

Выбор виден в логе (`E2E_TTS_PROVIDER <name> <auto|explicit>`) и лежит в
артефакте `tts_provider.json`. Из workflow — input `tts_provider`; из
agent-flow — env `E2E_TTS_PROVIDER`. Прочие ручки: `E2E_TTS_PROVIDER_ORDER`,
`MINIMAX_TTS_MODEL`, `E2E_SILERO_MODEL`, `E2E_SILERO_SAMPLE_RATE`.

#### Что прогон оставляет после себя

Харнесс пишет в `OUT_DIR` (`/tmp/e2e_v2_<run_id>`):

| файл | что внутри |
|---|---|
| `verdict.txt` | `PASS`/`FAIL` — вердикт, бинарный по ADR-0015 |
| `steps.jsonl` | по строке на шаг: `label`, `status`, `detail`, время |
| `summary.json` | сводка: шаги N/M, GATE-1, RMS/тишина, сверка с golden |
| `acceptance.json` | GATE-1: ожидаемые/вызванные тулы, ключевые слова (per-step — ещё и исход `register_speaker`, см. `.github/e2e/docs/GATE-1-DESIGN.md`, issue #2846) |
| `audio_metrics.json` | RMS/peak/silence по записи |
| `baseline_diff.json` | сверка записи с golden + `keyword_match_pct` |
| `transcript.json` | что просили сказать vs что распознал STT |
| `recording.wav`, `cmd_*.wav` | запись микрофона и синтезированные команды |

`summary.json` — то, что рендерится в GitHub Step Summary шагом
**E2E quality summary**. Вердикт при этом остаётся бинарным: счётчик
«9 из 11 OK» — доказательство, а не новая шкала, FAIL от него не теплеет.

Три места, где эти цифры раньше терялись, и почему их важно не сломать обратно:

1. **Замеры шли раньше, чем появлялся файл.** `audio_metrics`/`baseline_diff`
   читают `recording.wav`, а создавал его `stop_recording`, висевший только на
   `trap ... EXIT`. В каждом прогоне (включая зелёные) оба артефакта содержали
   `{"error":"recording.wav not found"}`. Теперь `stop_recording` вызывается
   явно перед замерами; trap остался страховкой.
2. **`transcript.json` был невалидным JSON** — `"expected"` подставлялся без
   кавычек. Его читает `e2e_baseline_diff.py` под `except: pass`, поэтому
   `keyword_match_pct` молча не считался. Сборка JSON ушла в `json.dumps`.
3. **На FAIL сводка скипалась.** У шага без `if:` действует неявное
   `if: success()`, а `Verdict from atomic harness` падает без
   `continue-on-error` — всё, что ниже, отменялось ровно на тех прогонах, где
   отчёт и нужен. Шаги отчётности теперь под `if: always()`.

Артефактов workflow'а — семь, и они не пересекаются: `e2e-voice-recording`
(все wav), `e2e-voice-artifacts` (полный бандл логов и json), `e2e-voice-logs`,
`e2e-voice-model`, `e2e-voice-timing`, `e2e-voice-diff` и условный
`e2e-acceptance`. Раньше их было одиннадцать: `transcript`/`audio-metrics`/
`baseline-diff`/`acceptance` дублировали файлы из общего бандла побайтово, а
`harness-artifacts` тянул каталог целиком и уносил вторую копию всех wav
(+26 МБ на прогон). Guard на это — `tests/unit/e2e_scripts/
test_issue_1429_no_recording_wav_dupe.py`.

### `e2e_remote.sh` — 🟡 deprecated (шляпа)

Старый e2e-харнесс (проигрывает .ogg и пишет wav; не проверяет полный цикл
робота; даёт ложные PASS из-за приветствия). **Не используется workflow'ом**
с 11.08 и заменён атомарным. Оставлен в репо как reference для истории и для
отладки редких регрессий, если атомарный харнесс споткнётся. **НЕ запускай
напрямую на 249 без понимания рисков (ложные PASS).**

### `ensure_voice_file.sh` — 🟡 deprecated

Старый way обеспечения voice-файла: три фолбэка (repo → build host →
генерация Yandex TTS на роботе через `v3/rpc`-стиль). **Не нужен атомарному
харнессу**: `e2e_voice_test.sh` синтезирует команду на лету сам. Оставлен
для legacy-сценариев, которые ещё пользуются `voice_file` input.

### `e2e_timing.py` — 🟢 активный, телеметрия

Извлекает метрики скорости ответа из лога voice-assistant:

- `T_accept` — STT latency (Получена фраза → ПРИНЯТО)
- `T_llm` — LLM вызов (LLM INPUT → первый Синтез)
- `T_tts` — синтез (Синтез → TTS finished)
- `T_total` — акцепт → ответ

Запуск: `python3 e2e_timing.py /tmp/voice_e2e_<run_id>.log`. Зовётся из
шага "E2E timing metrics" в `L: E2E Voice Test.yml`.

### `yts.py` — 🟢 активный, переиспользуемый

Синтез речи через **Yandex Cloud TTS gRPC v3** (контракт как у `tts_node.py`,
нота `text: str`-форма для старого proto). Используется для отладочных
прогонов с одной командой. В атомарном харнесс синтез встроен, но этот
скрипт пригодится для ad-hoc "проверить как звучит голос X с фразой Y".

Только Yandex: выбор провайдера живёт в самом харнессе (`--tts-provider`), а
этот скрипт остаётся однопровайдерным пробником — если нужен MiniMax/Silero,
зови харнесс.

Запуск:

```bash
ssh ros2@10.1.1.249 "export YANDEX_API_KEY=\$(cat /tmp/yandex_key.txt); \
  python3 /tmp/yts.py 'Робот, привет меня зовут Саша' anton /tmp/cmd.wav"
```

Контракт proto: см. https://cloud.yandex.ru/docs/speechkit/tts/api-ref/grpc/tts_service
(версия — более старый билд yandex-cloud python, `text=text` без обёртки
`tts_pb2.Text` — это **намеренно** для совместимости с build host,
обновится в рамках #FIXME — см. issue skill `synthesis-tts-chain-debugging`).

## Ops scripts

### `restart_runners.sh` — 🔧 ops

Перезапускает билд-раннеры (GitHub Actions self-hosted на build host).
Использовался при симптоме «диск-фулл на раннере» (warning в логах деплоя:
`Free space left: 0 MB`) — обычно `docker run --rm` с примонтированным
workspace чистит кэш, дальше `docker compose up -d` поднимает runner'ы
свежими. **Вызывается девопсом вручную**, не из workflow.

```bash
ssh ros2@10.1.1.249 'bash /tmp/restart_runners.sh'
```

Контекст возникновения: round-42 e2e упал на «no free space», см.
issue #1084 (фикс в работе devops-карточкой t_db72f69f).

## Версионирование

- **Синхронизация на 249:** workflow делает `sshpass scp` нужного скрипта в
  начале каждого прогона (`/tmp/<script>`) — поэтому изменения в этой папке
  попадают на build host **автоматически**, без ручных правок.
- **legacy-файлы (e2e_remote.sh, ensure_voice_file.sh)** на 249 можно
  держать как резерв для дебага; основной поток идёт через атомарный
  харнесс.

## Исторические/отладочные (НЕ в репо)

На 249 в `/tmp` исторически копятся **отладочные скрипты** конкретных сценариев
— `play_scenario_v3.sh`, `play_scenario_v4.sh`, `synth_v4.sh`,
`e2e_remote_t7e.sh`, и прочие. Они не параметризованы, привязаны к
конкретным тестам 06.08-10.08, в репо не идут. Периодически чистятся
batch-ом (см. канбан-карточку «build host scripts GC», когда создастся).
