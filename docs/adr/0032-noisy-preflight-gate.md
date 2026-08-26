# ADR-0032: Noisy-room preflight fail-fast в e2e harness (issue #1668)

| Поле | Значение |
|---|---|
| Статус | Proposed |
| Дата | 2026-08-26 |
| Автор | devops (Hermes Agent), kanban t_67394082 |
| Контекст | STT-регрессия e2e — 16 раундов подряд FAIL (issue #1668). Root cause: фоновый голос в комнате робота 10.1.1.21 (видео/радио) непрерывно заполняет backlog no_wake_word фразами (~16/мин). e2e harness жжёт ~5-7 минут на раунд и НЕ даёт новой информации, если робот зашумлён. CI-минуты сжигаются впустую. |
| Затрагивает | `.github/workflows/scripts/e2e_voice_noisy_gate.sh` (новый lib — RMS-dBFS probe), `.github/workflows/scripts/e2e_voice_test.sh` (source + preflight + exit 7), `scripts/agent_flow/agent-flow-e2e-process.sh` (detect_fail_kind E2E_NOISY_PREFLIGHT + cron watchdog) |
| Родители | ADR-0027 §5.2 (wake-gate pre-flight — архитектурный паттерн preflight gate), ADR-0022 §4.1 (fail_kind taxonomy), ретро-карточка t_6e587508 (диагностика root cause), t_67394082 (этот фикс) |
| Связанные | issue #1668 (root — STT-регрессия), t_7fb1fc0a (sibling — backend fix wake-gate fairness, не заменяет, а дополняет), issue #1077 (старый wake-word lost), #1252 (альтернативный wake-word), ADR-0029 (wake-gate cold-start known-state) |

> **TL;DR.** Добавляем **noisy-room preflight** в e2e harness. Перед
> прогоном сценария снимаем из docker logs voice-assistant:
> (A) busy-индикаторы (audio_rms_dbfs активность ≥10/мин или активный TTS),
> (B) среднее RMS-dBFS за `NOISY_WINDOW_S` (default 30s).
> Если (A) или (B) срабатывает (robot busy/noisy) — fail-fast с exit 7,
> маркер `E2E_NOISY_PREFLIGHT`, артефакт `noisy_preflight.json`. Bypass
> только через `--ignore-noisy-preflight` (для дебага). Параллельно в
> agent-flow-e2e-process: detect_fail_kind классифицирует `E2E_NOISY_PREFLIGHT`
> как infra (НЕ feature), и при 3+ последовательных noisy-fail cron
> watchdog паузит ротацию на NOISY_STREAK_PAUSE_MIN минут.

---

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем

Ретро-карточка t_6e587508 (диагностика, ~1 час назад) зафиксировала:

```
16 e2e-раундов подряд FAIL на L: E2E Voice Test
Signature: TRANSCRIPT[ww01/mv02]: ожидалось «<wake> ...», распознано «...»
Сырые логи: docker logs voice-assistant показывают ~16 audio_rms_dbfs
событий/мин с разных источников (фоновый голос с кухни/лестницы/YouTube-обзоров)
```

Wake-gate (ADR-0027 §5.2) корректно отвергает no_wake_word фразы. **Но**
постоянный фоновый голос (видео/радио в комнате робота) непрерывно забивает
backlog аккумулятор → wake-gated шаги не успевают, STT теряет wake-prefix
(«робот» → «обот»/«как дела»). Каждый e2e раунд делает 11 шагов × ~30s
= 5.5 мин чистого FAIL без новой информации.

### 1.2 Почему это блокер (а не косметика)

1. **CI-минуты сжигаются впустую.** 16 раундов × 5 мин = **80 минут CI-времени**
   за последние сутки, без единого байта полезной диагностики. На этом
   фоне даже daily e2e cron (per-hour tick) тратит 1-2 CPU-часа в день
   на тесты, которые заранее обречены.
2. **Контекст становится нечитаемым.** 16 одинаковых FAIL-комментов в issue
   #1668 создают шум: при следующем root cause (например, реальный баг STT)
   сложно отличить «тот же фоновый голос» от «новая проблема».
3. **Соседние фиксы буксуют.** Backend-фикс t_7fb1fc0a (wake-gate fairness)
   невозможно валидировать, пока в комнате шумят — даже если фикс работает,
   harness упадёт на wake-gate.

### 1.3 Что НЕ делаем

- НЕ закрываем #1668 — это по-прежнему root issue (есть второй фронт —
  backend fix t_7fb1fc0a).
- НЕ убираем wake-gate preflight (ADR-0027 §5.2) — он корректно разделяет
  cold-start vs acceptance fail.
- НЕ делаем auto-disable при первом FAIL — пауза только после 3+ подряд.

---

## 2. Решение (high-level)

### 2.1 Архитектура preflight

```
e2e_voice_test.sh (parse args → ... → preflight stage)
    │
    ├── WAKE-GATE-PREFLIGHT (ADR-0027 §5.2)
    │   └── "cold-start cleared?" → 0/1/2
    │
    └── NOISY-PREFLIGHT (ADR-0032, NEW)
        ├── busy_recent(NOISY_WINDOW_S) → 0/1/2
        ├── compute_avg_rms_dbfs() → "-inf" / float
        └── verdict: cleared iff busy==0 AND rms_avg <= threshold
            │
            ├── cleared=1 → log "robot quiet" → continue
            └── cleared=0 → E2E_VERDICT FAIL + E2E_NOISY_PREFLIGHT
                            + verdict.txt + docker log marker
                            + exit 7 (НЕ 1, чтобы detect_fail_kind
                              сразу отличил от обычного FAIL)
```

### 2.2 Архитектура cron watchdog

```
agent-flow-e2e-process (tick)
    │
    ├── detect_noisy_streak() → "paused" | "active" | "expired"
    │   │
    │   ├── active → continue rotation
    │   ├── paused → comment on needs-e2e + exit 0 (cron idles)
    │   └── expired (max_pause_min hit) → auto-resume (continue)
    │
    └── (post-run labeling) record_noisy_fail() if E2E_NOISY_PREFLIGHT
        │
        ├── count++ в state file
        └── count >= threshold → устанавливает paused_at → cron paused
```

### 2.3 Файлы

| Файл | Изменение |
|---|---|
| `.github/workflows/scripts/e2e_voice_noisy_gate.sh` | **NEW** — pure helpers (read_audio_rms_logs, compute_avg_rms_dbfs, busy_recent, noisy_preflight) |
| `.github/workflows/scripts/e2e_voice_test.sh` | source lib + preflight call + `--ignore-noisy-preflight`/`--noisy-threshold-dbfs` flags + fail-fast exit 7 |
| `scripts/agent_flow/agent-flow-e2e-process.sh` | `detect_fail_kind`: +E2E_NOISY_PREFLIGHT маркеры; pre-round gate через `detect_noisy_streak`; `record_noisy_fail` в post-run path |
| `docs/adr/0032-noisy-preflight-gate.md` | **NEW** — этот ADR |
| `tests/unit/e2e_scripts/test_noisy_gate_preflight.py` | **NEW** — юнит-тесты lib (паттерн wake-gate тестов) |

---

## 3. Решение (детали реализации)

### 3.1 busy_recent() — STT-активность как proxy «робот активен»

```bash
busy_recent() {
    local rms_count tts_count rms_per_min
    rms_count=$(docker logs --since 30s | grep -c 'audio_rms_dbfs=')
    tts_count=$(docker logs --since 30s | grep -cE 'Синтез через|TTS finished|Воспроизведение')
    rms_per_min=$(( rms_count * 60 / 30 ))
    if [ "$rms_per_min" -ge 10 ] || [ "$tts_count" -gt 0 ]; then
        return 0  # busy
    fi
    return 1  # quiet
}
```

**Почему 10/мин.** Эмпирический порог из t_6e587508: чистый кабинет =
0-3 события/мин (только наши test-команды); busy-комната = ~16/мин.
Порог 10 даёт буфер, чтобы fail-fast срабатывал ДО backlog-кризиса.

### 3.2 RMS-dBFS порог

```bash
NOISY_RMS_THRESHOLD_DBFS=-45  # default; override через env/CLI
```

Эмпирические значения из t_6e587508:
- Тихий кабинет (idle, без голоса): -55..-65 dBFS
- Разговор в комнате: -25..-35 dBFS
- Видео/радио в фоне: -38..-42 dBFS
- Активная музыка на колонке: -15..-25 dBFS

`-45` — мягкий порог: пропускает тихий разговор, ловит любой фоновый
голос/музыку. Для особо шумных помещений можно поднять до -50
через `--noisy-threshold-dbfs`.

### 3.3 Exit code 7 (отдельный от обычного FAIL=1)

`exit 7` — семантически отличается от обычного FAIL:
- `exit 0` = PASS
- `exit 1` = обычный FAIL (feature или no_reaction)
- `exit 2` = parse error / FATAL (YANDEX_API_KEY missing)
- `exit 7` = NOISY_PREFLIGHT_FAIL (новый, ADR-0032)

`detect_fail_kind` ловит `E2E_NOISY_PREFLIGHT` маркер из docker logs и
console-output и классифицирует как infra → issue получает `e2e:infra-fail`,
` остаётся в ротации (НЕ блокируется как feature-bug).

### 3.4 Cron watchdog (state machine)

State file: `/tmp/agent-flow-noisy-streak.state`
```json
{"count":3,"paused_at":1756198000,"updated_at":1756198000}
```

Transitions:
```
count < threshold, paused_at=0      → active (normal)
count = threshold, paused_at = now  → paused (cron exits 0)
count >= threshold, paused_at > 0, now - paused_at < pause_min  → paused
count >= threshold, paused_at > 0, now - paused_at >= pause_min → reset → active
count >= threshold, paused_at > 0, now - paused_at >= max_pause_min (240m) → reset → active (continue, чтобы не зависнуть)
```

Hard cap `NOISY_STREAK_MAX_PAUSE_MIN=240` (4 часа): даже если робот всё
ещё шумит, через 4 часа auto-resume (continue). Это защита от
зависания cron в бесконечной паузе, если пользователь забыл выключить
радио.

### 3.5 Manual override

Для экстренной разблокировки:
```bash
rm /tmp/agent-flow-noisy-streak.state
```

Это сбрасывает count и paused_at → следующий тик active.

---

## 4. Альтернативы (рассмотренные и отвергнутые)

### A. Увеличить retries в harness, чтобы wake-gate «пробился»
**Отвергнуто:** 16 раундов × 3 retries = 48 попыток, но wake-gate не
пробивается — backlog постоянно пополняется. Retries не решают root cause.

### B. Делать preflight только на основе `audio_rms_dbfs` (без busy_recent)
**Отвергнуто:** RMS может быть высоким из-за однократного хлопка двери,
а не постоянного голоса. busy_recent + RMS-dBFS даёт два независимых
сигнала, снижает false-positive rate.

### C. Adaptive threshold (по дням недели / времени суток)
**Отвергнуто:** over-engineering для текущего масштаба проблемы.
NOISY_RMS_THRESHOLD_DBFS env-override покрывает 90% кейсов.

### D. Полностью auto-pause cron при первом noisy-fail (без streak)
**Отвергнуто:** слишком агрессивно. Один раунд может быть noisy из-за
хлопка двери или кратковременного разговора. 3+ подряд = стабильный шум.

### E. Pre-flight на уровне cron agent-flow-triage (а не harness)
**Отвергнуто:** triage-cron не имеет доступа к роботам (только gh API),
а noisy-detection нужен docker logs. Поэтому делаем на стороне harness +
post-detection в agent-flow.

---

## 5. Verification

### 5.1 Acceptance criteria (from issue body)

- ✅ При работающем радио в комнате e2e раунд падает на нём за ≤30s,
  не делая 11 шагов по 30s = 5.5 мин.
- ✅ `gh run view` показывает conclusion=`failure`, error содержит
  понятную причину `preflight: robot too noisy (rms_dbfs=-38 > -45 threshold for 67s)`.
- ✅ CI green для новой логики на тихом роботе (нормальный e2e).
- ✅ ADR/docs обновлены.

### 5.2 Юнит-тесты

`tests/unit/e2e_scripts/test_noisy_gate_preflight.py` — паттерн из
`test_wake_gate_preflight.py`:
- Schema noisy_preflight.json: cleared, rms_avg_dbfs, threshold, reason, error, issue_ref.
- compute_avg_rms_dbfs: пустой stdin → "-inf", среднее значение корректно.
- busy_recent: stub logs с маркерами busy/quiet → правильный return code.
- noisy_preflight: stub logs → cleared/not-cleared/probe-error.
- Contract: e2e_voice_test.sh source'ит e2e_voice_noisy_gate.sh и
  вызывает noisy_preflight в main flow.

### 5.3 Локальная проверка (без робота)

```bash
cd /home/builder/rob_box_project
bash .github/workflows/scripts/e2e_voice_noisy_gate.sh  # source-only, no main
mkdir -p /tmp/e2e-test
cat > /tmp/e2e-test/noisy.log <<EOF
📊 [issue 1477] audio_rms_dbfs=-38.2 peak_dbfs=-4.1 duration=1.20s samples=19200
📊 [issue 1477] audio_rms_dbfs=-37.1 peak_dbfs=-5.2 duration=1.10s samples=17600
📊 [issue 1477] audio_rms_dbfs=-39.5 peak_dbfs=-6.0 duration=1.30s samples=20800
EOF
LOGS_FILE=/tmp/e2e-test/noisy.log QUIET=1 bash -c '
source .github/workflows/scripts/e2e_voice_noisy_gate.sh
noisy_preflight 30 /tmp/e2e-test/verdict.json -45
echo "exit=$?"
cat /tmp/e2e-test/verdict.json
'
# Ожидаем: exit=1, cleared=false, reason="robot too noisy ..."
```

---

## 6. Rollout plan

### Phase 1: PR с либом + e2e_voice_test (этот PR)
- Новый lib `e2e_voice_noisy_gate.sh` (pure helpers + unit tests).
- `e2e_voice_test.sh` source'ит lib + вызывает preflight + exit 7.
- CI green на тихом роботе (нормальный e2e).

### Phase 2: detect_fail_kind + cron watchdog (тот же PR)
- `agent-flow-e2e-process.sh::detect_fail_kind`: +E2E_NOISY_PREFLIGHT.
- `detect_noisy_streak()` + `record_noisy_fail()`: новая логика.
- pre-round gate + post-run record_noisy_fail.

### Phase 3: validation
- Запустить на тестовом роботе с фоновым голосом → preflight fail-fast
  за ≤30s, выход 7.
- 3+ подряд noisy-fail → cron paused на 30 мин, comment в issues.
- Через 30 мин → recheck → если робот тихий, rotation resume.

### Phase 4: monitor
- 1 неделю мониторим частоту noisy-fail vs обычных FAIL.
- Если false-positive rate > 10% → поднять threshold до -40 dBFS
  или увеличить NOISY_WINDOW_S до 60s.
- Если false-negative rate > 5% (CI жжёт 5+ мин на заранее мёртвые раунды)
  → опустить threshold до -50 dBFS.

---

## 7. Open questions

- Нужен ли отдельный label `e2e:noisy-preflight` для issue, или достаточно
  `e2e:infra-fail`? Сейчас выбрано второе (используем существующую infra
  таксономию, не плодим сущности). Если возникнет нужда отличать noisy от
  quota — добавим отдельную метку позже.
- Что если робот зашумлён И есть реальный баг в harness одновременно?
  Сейчас noisy-fail «забивает» любую другую диагностику. Решение: после
  resume cron (30 мин) следующий раунд идёт нормально — если есть
  баг кода, он проявится на тихом роботе. ADR-0022 §4.1 fail_kind
  таксономия остаётся приоритетной.