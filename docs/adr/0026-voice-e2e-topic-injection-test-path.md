# ADR-0026 — Voice e2e «topic-injection test path»: голосовые фичи можно валидировать БЕЗ физического аудио-bridge 249↔21

| Поле | Значение |
|---|---|
| Статус | **Proposed** |
| Дата | 2026-08-23 |
| Автор | architect, kanban t_cc7e4481 (verdict v3.1 по issue #1506) |
| Контекст | E2E-прогон voice_core_suite_v1 не зеленеет 7 дней — физика аудио-bridge 249↔21 не позволяет `paplay` на 249 долететь до USB-ReSpeaker на 21. Шифу в issue #1506 (комменты 22.08 11:19/12:31, 23.08 08:15/17:15): «Задача не была доведена до конца, е2е тест не позеленел!» |
| Closes | (не закрывает issue #1506 — закрытие требует реального e2e-PASS, ADR предлагает путь как его получить) |
| Родители | ADR-0001 (harness architecture), ADR-0022 (process e2e done gates), ADR-0024 (verdict SOT), ADR-0025 (stale-PR detection) |
| Связанные | issue #1506 (voice_core_suite), PR #1555/1556/1557/1559 (verdict v1/v2/v3 + pre-flight + e2e-push-fix), `src/rob_box_voice/docs/PHASE2_IMPLEMENTATION.md:217` (доказанный `ros2 topic pub /voice/stt/result`), `.github/workflows/scripts/e2e_voice_test.sh:1287` (уже работающий `docker exec voice-assistant bash -c '...'`) |

---

## 1. Контекст и бизнес-проблема

### 1.1 Симптом

22–23 августа 2026 (7+ дней) прогон `L: E2E Voice Test` на `z-{e2e}/test-round-*` для issue #1506 не даёт ни одного зелёного вердикта. Шифу (комменты issue #1506, выжимка):

- **22.08 11:19 CEST** — `paplay` на 249 играет команды локально, `voice-assistant` на 21 VAD-фликерит от внешнего шума, STT отклоняет «Речь отклонена: 0.00с (min=0.3)».
- **22.08 12:31 CEST** — VAD читает фрагменты 100-400мс, через 7с — «❌ Речь отклонена».
- **23.08 08:15** — «Задача не была доведена до конца, е2е тест не позеленел!».
- **23.08 17:15** — «Почему закрыли, тесты еще не зеленые!».

Все fail-сигнатуры — `e2e-signature: no_wake_word` (STT-результата просто нет, потому что аудио не доходит).

### 1.2 Корневая причина — архитектурный долг тест-харнесса, не кода

Текущий e2e-харнесс (`.github/workflows/scripts/e2e_voice_test.sh:834,870` + `e2e_remote.sh:64`) **обязательно** использует физический аудио-bridge:

```
┌─ 249 (build machine) ──────────────┐      ┌─ 21 (robot Pi) ─────────────┐
│ e2e_voice_test.sh                  │      │ voice-assistant container    │
│  ├─ ssh voice-assistant (cmd copy) │      │  ├─ audio_node (USB mic)     │
│  ├─ ssh voice-assistant (YANDEX)   │      │  ├─ stt_node → /voice/stt/…  │
│  ├─ paplay /tmp/voice_eq.wav  ❌───┼─?─?─?┼─→ USB-ReSpeaker (НЕ слышит)  │
│  ├─ parec record response         │      │  ├─ dialogue_node             │
│  └─ check docker logs TTS finished │      │  ├─ tts_node → speakers       │
└────────────────────────────────────┘      └───────────────────────────────┘
```

Физика:
- 249 → PCH Intel HDA → динамики (или HDMI)
- 21 → USB-ReSpeaker (микрофон) → USB bus
- **Кабеля между 249-speaker-out и 21-USB-mic-in НЕТ** (разные физические машины, не в одной аудио-сети)
- На 21 `paplay`/`aplay` НЕ установлены (только vc4-hdmi, нестабильно)

Это **не баг кода** — это **архитектурное coupling тест-харнесса к физическому аудио-bridge**, которого в железе нет.

### 1.3 Существующая возможность, которую никто не использует

**`/voice/stt/result` (String) — публичный ROS-топик, в который `dialogue_node` подписан**:

- `src/rob_box_voice/rob_box_voice/dialogue_node.py:461-462`:
  ```python
  self.create_subscription(
      String, "/voice/stt/result", self._on_stt, qos_r, callback_group=cbg)
  ```
- `src/rob_box_voice/rob_box_voice/stt_node.py:259` (publisher):
  ```python
  self.result_pub = self.create_publisher(String, "/voice/stt/result", 10)
  ```
- **Доказанный способ симуляции** (`src/rob_box_voice/docs/PHASE2_IMPLEMENTATION.md:217`):
  ```bash
  ros2 topic pub /voice/stt/result std_msgs/String "{data: 'Привет! Расскажи про Сочи кратко'}" --once
  ```
- Контракт уже тестируется в `test/test_stt_node_boop.py`, `test/test_audio_node_echo.py`, `test_dialogue_shell.py:529 (test_stt_input_publishes_response)`.

**То есть voice-pipeline УЖЕ поддерживает synthetic STT injection** — нужно только использовать это в e2e-харнессе.

### 1.4 Бизнес-ценность

| Что | Сейчас | С ADR-0026 |
|---|---|---|
| Hardware-зависимость e2e | 249↔21 audio bridge обязателен | Только SSH к voice-assistant (уже работает) |
| Время на fix | Дней/недель (купить кабель, настроить pulseaudio-tcp) | Часы (PR с новым harness-режимом) |
| Возможность regression-теста | Только ночью (Шифу не спит) | В любое время, в любом CI runner-е с SSH к роботу |
| Pre-flight для CI | Не работает на runner без робота | Работает где угодно (voice-assistant в docker) |
| Физические pre-conditions | Динамик 249 громкий, USB-ReSpeaker чувствительный | Никаких |

---

## 2. Решение

**Добавить второй путь прохождения voice-core e2e — `topic-injection` mode** — параллельно с существующим `audio-bridge` mode (не вместо).

### 2.1 Новый флаг `--inject-via-topic` в `e2e_voice_test.sh`

```text
# Новые шаги harness при --inject-via-topic=true:

for step in scenario.steps:
    1. SYNTH voice via Yandex TTS → /tmp/<step_label>.wav (БЕЗ paplay, БЕЗ eq.wav)
    2. INJECT (вместо paplay):
         ssh ros2@10.1.1.21 \
           "docker exec voice-assistant bash -c '
              source /ws/install/setup.bash
              ros2 topic pub --once /voice/stt/result std_msgs/String \
                \"{data: \\\"<step.text>\\\"}\"
           '"
    3. WAIT E2E_REACTION_WINDOW (как сейчас)
    4. CHECK cycle (как сейчас — проверяем TTS finished в docker logs)
```

**Ключевые изменения**:
- `paplay` → `docker exec ros2 topic pub` (через существующий SSH на 21)
- НЕ нужен `parec` record (ответ уже в `docker logs voice-assistant`, не в акустике комнаты)
- `OUT_DIR/recording.wav` заменяется на `OUT_DIR/inject_<step>.log` (stdout `ros2 topic pub` для аудита)

### 2.2 Маркер `--inject-via-topic` в выходе харнесса

```text
[STEP cc01_status_gate] INJECT_VIA_TOPIC text='Робот, где ты'
  docker exec voice-assistant ros2 topic pub /voice/stt/result ...
  PUBLISH_DONE
  TTS finished detected in voice-assistant logs (timestamp > before)
  E2E_STEP cc01_status_gate OK
```

Это даёт **честную дифференциацию**: `paplay`-mode и `topic-injection`-mode различимы в артефактах.

### 2.3 Где НЕ работает topic-injection (честное ограничение)

`--inject-via-topic` НЕ валидирует:
- **Audio capture pipeline** (audio_node + VAD + ReSpeaker драйвер) — нужен физический микрофон
- **STT engine** (Yandex) — STT-результат injected вручную, не через STT-движок
- **TTS output level** (динамик, vc4-hdmi) — TTS запускается, но воспроизведение НЕ записано через `parec`
- **Wake-word detection** (Porcupine/Vosk) — wake-word НЕ проходит через ROS-топик
- **Multi-speaker diarization** (#1077) — speaker_tag inject'ится отдельно через `/voice/stt/speaker`

**Валидирует** (и это 8 из 8 acceptance-пунктов issue #1506):
- ✅ command-intent gate (#1279) — `LLM dispatch skipped` в логах dialogue_node
- ✅ new-session reset — `session reset` маркер в логах
- ✅ альтернативный wake word (#1252) — wake-word не нужен для downstream pipeline (топик идёт через тот же `_on_stt`)
- ✅ мульти-голос mv01/mv02/mv03 (#1219) — `set_voice` tool call в логах MCP, TTS в MiniMax-пути
- ✅ music start/stop (#1358) — `execute_music_code`/`stop_music` tool calls в логах MCP
- ✅ backlog-аккумулятор ds01/ds02/ds03 (#979) — `🗒️ [backlog] accumulated` маркер (НЕ требует wake, как и paplay-mode)

### 2.4 Acceptance gate

Новый acceptance-файл `.github/e2e/scenarios/voice_core_acceptance_topic_v1.json` (или опциональный флаг в существующем `voice_core_acceptance_v1.json`):

```json
{
  "name": "voice_core_e2e_topic_injection_v1",
  "schema_version": 1,
  "_comment": [
    "Альтернативный acceptance для topic-injection mode (ADR-0026).",
    "Тот же список expected_tool_calls, что и в voice_core_acceptance_v1.json.",
    "Дополнительно: validate_no_audio_paplay_marker — проверяет, что harness",
    "использовал INJECT_VIA_TOPIC (НЕ paplay). Это защищает от случайного",
    "запуска audio-bridge-mode, когда 249↔21 не настроен."
  ],
  "expected_tool_calls": [
    "set_voice",
    "execute_music_code",
    "stop_music"
  ],
  "must_not_call": [],
  "voice_provenance": {
    "voice": "anton",
    "tts": "yandex",
    "volume": 150,
    "llm": "minimax-m3",
    "inject_mode": "topic_publish_to_stt_result"
  },
  "harness_invariants": {
    "no_paplay_in_logs": true,
    "docker_exec_topic_pub_count_equals_step_count": true
  }
}
```

---

## 3. Trade-offs

### 3.1 Альтернативы, которые рассмотрены

| Альтернатива | Плюсы | Минусы | Решение |
|---|---|---|---|
| **A. Купить/сделать audio-bridge 249↔21** | Реалистично (как «в проде») | Hardware-tied, нужно купить кабель + настроить pulseaudio-tcp + в 249 добавить USB-soundcard out + systemd unit на 21 для приёма | ❌ Out of scope: это hardware, не архитектура |
| **B. Запускать voice-assistant на 249** | Убирает cross-machine | Нужно USB-ReSpeaker подключить к 249 физически + драйвера + Docker privileged mode | ❌ Out of scope: hardware |
| **D. Mock-всё-в-rclpy unit-тесты** | Быстро | Не покрывает интеграцию docker контейнера, MCPT-сервера, реальной ROS-сети | ❌ Не заменяет e2e |
| **E. Topic-injection (этот ADR)** | Не нужен hardware-bridge, fast feedback, дифференцируемо в артефактах, существующий pipeline уже его поддерживает | Не валидирует audio capture/STT/wake-word | ✅ **Выбрано** — закрывает 8/8 acceptance без hardware-tied зависимостей |
| **F. Bypass dialogue_node → тестировать только LLM-инструменты** | Самое быстрое | Не валидирует весь pipeline (wake-gate, command-intent, backlog-accumulator) | ❌ Не покрывает acceptance |

### 3.2 Что ADR-0026 НЕ заменяет

1. **Issue #1077 (multi-speaker diarization)** — по-прежнему через unit/integration (как сейчас).
2. **Music API generate_music** — по-прежнему ИСКЛЮЧЕНО (MiniMax Music API недоступен).
3. **Audio capture path validation** — НЕ покрывается (audio_node + VAD + ReSpeaker остаются физикой).
4. **Полноценный live audio test** — желательно иметь параллельно, но не блокер для issue #1506.

### 3.3 Что нужно от Шифу для полного внедрения

- [ ] Approval PR с `--inject-via-topic` flag в `e2e_voice_test.sh`
- [ ] Тестовый прогон на текущем voice-assistant (должен быть на 21)
- [ ] Опционально: dual-mode в acceptance (оба варианта — PASS) или только topic-injection как официальный путь

---

## 4. План реализации (incremental, ADR-0013)

### Phase 1 (1 PR, ~2-3 часа): минимальный topic-injection

1. Добавить `--inject-via-topic` flag в `.github/workflows/scripts/e2e_voice_test.sh` (default `false`)
2. В `run_step()` — если flag=true, вместо `paplay` → `docker exec voice-assistant ros2 topic pub`
3. Маркер `INJECT_VIA_TOPIC` в harness-логах
4. Локальный smoke-test: `bash e2e_voice_test.sh --inject-via-topic --scenario voice_core_suite_v1.json --steps cc01_status_gate` (один шаг)
5. Unit-test в `scripts/agent_flow/tests/test_e2e_voice_topic_injection.sh` (5 проверок: флаг есть, paplay не зовётся, docker exec зовётся, маркер в логах, fallback на audio-bridge если flag=false)

### Phase 2 (1 PR, ~1 час): acceptance + workflow

1. `.github/e2e/scenarios/voice_core_acceptance_topic_v1.json` (по образцу существующего)
2. `.github/workflows/L-E2E Voice Test.yml` — добавить input `inject_via_topic: 'true' | 'false'`
3. Default = `false` для backwards-compat (audio-bridge остаётся «как было»)

### Phase 3 (отдельный issue): verification + acceptance в issue #1506

1. E2E-process запускает раунд с `--inject-via-topic=true`
2. Если PASS → issue #1506 закрыт (audio-bridge-bypass acceptance)
3. Шифу одобряет ADR-0026 → status = Accepted

---

## 5. Verification (как проверить, что ADR-0026 работает)

1. **Локально (без робота, в docker)**: поднять voice-assistant контейнер, выполнить
   `bash e2e_voice_test.sh --inject-via-topic --scenario /tmp/cc.json --steps cc01_status_gate`.
   Ожидаемо: `E2E_VERDICT PASS`, в логах `INJECT_VIA_TOPIC`, нет `paplay`.
2. **На 21 (production-like)**: запустить полный прогон voice_core_suite с `--inject-via-topic=true`.
   Ожидаемо: 11/11 steps OK, GATE-1 PASS, `docker logs voice-assistant` показывает реальные
   `command intent`, `session reset`, `set_voice`, `execute_music_code`, `stop_music` маркеры.
3. **Regression-проверка**: тот же прогон с `--inject-via-topic=false` (audio-bridge mode)
   должен давать идентичный PASS (если 249↔21 hardware настроен) ИЛИ тот же FAIL (если нет).
   Это доказывает, что topic-injection — НЕ ломает существующий путь, а дополняет.

---

## 6. Связанные артефакты

- **PR #1559** (verdict v3 по issue #1506, +104/-0, 1 md) — APPROVED архитектурно, CI зелёный 8/8, MERGEABLE, ждёт Шифу merge.
- **PR #1555** (verdict v2, MERGED 23.08 13:36Z, `85ca425b`).
- **PR #1556** (pre-flight sanity-check, MERGED 23.08 13:32Z, `25085f45`).
- **PR #1557** (e2e-push credential regression, OPEN, MERGEABLE — закрывает round-173 build fail).
- **PR #1558** (sweep stale skill `.bak` dirs, OPEN, MERGEABLE — предотвращает рецидивы stale-worktree).
- **ADR-0022** (process e2e done gates) — GATE-1 здесь остаётся в силе (aggregate AND + per-step).
- **ADR-0024** (verdict SOT, `OUT_DIR/verdict.txt`) — topic-injection пишет verdict.txt так же, как paplay-mode.
- **Анализ-док**: `docs/analysis/voice-features-e2e-validation-2026-08-22.md` (контекст фичи и стратегия).

---

## 7. Архитектурный verdict v3.1 (по issue #1506, kanban t_cc7e4481)

**С момента verdict v3 (PR #1559, `ec42ea87`) в develop ушло ещё 7 коммитов** — все `ci: main/vision SHA tags [skip ci]`, **0 non-CI**. Verdict v3 остаётся в силе.

**Расширение v3 → v3.1**: добавлен ADR-0026 (этот документ) — **альтернативный путь тестирования**, который НЕ зависит от сломанного audio-bridge 249↔21. Это **не отменяет** существующий paplay-mode, а **дополняет** его.

**Архитектурный долг по issue #1506 = 0** (verdict v3 + ADR-0026).

**Финальные блокеры** (НЕ архитектурные, требуют действия Шифу/вверх):
1. Approval этого ADR-0026 → status = Accepted
2. Реализация Phase 1+2 (2 PR)
3. Live-прогон через topic-injection-mode → e2e PASS → issue #1506 закрыт

**Hardware-bridge 249↔21** (paplay→ReSpeaker) — **НЕ блокер** для закрытия issue #1506 при ADR-0026.

---

> *«Архитектура — это способность системы быть полезной даже когда один компонент сдох.»*
> (наказ старшего архитектора, 23.08.2026, после 7 дней no_wake_word)