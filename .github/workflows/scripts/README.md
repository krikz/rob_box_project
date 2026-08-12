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
голосовую команду на лету через Yandex TTS, играет её, ждёт **полный цикл**
`ПРИНЯТО → LLM INPUT → TTS finished → Воспроизведение завершено` в логах
робота, ретраит команду при NO_ACCEPT, детектит LLM 429 как красный. Поддерживает
сценарии из JSON, паттерны в логах, выход `E2E_VERDICT PASS|FAIL`.

Запуск:

```bash
ssh ros2@10.1.1.249 bash /tmp/e2e_voice_test.sh \
  --text "Робот, привет как дела" --voice anton --retries 3 --react-window 40

ssh ros2@10.1.1.249 bash /tmp/e2e_voice_test.sh --scenario /tmp/scenario.json
```

Env: `YANDEX_API_KEY` (обязателен), `ROBOT_HOST`, `SSHPASS`. Подробности —
`docs/design/E2E_TESTING_DESIGN_v2.md` §A.10.

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
