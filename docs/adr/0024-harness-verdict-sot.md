# ADR-0024: `OUT_DIR/verdict.txt` — single source of truth для harness verdict (pull, не tee-race)

| Поле | Значение |
|---|---|
| Статус | Accepted |
| Дата | 2026-08-22 |
| Автор | devops (Hermes Agent) |
| Контекст | kanban t_b7d7656e — `harness-output-missing` (round-167 scp race, run #32533566491) |
| Затрагивает | `.github/workflows/L-E2E Voice Test.yml` (`Collect e2e artifacts from build machine` + `Validate e2e result`), `.github/workflows/scripts/e2e_voice_test.sh` (уже пишет verdict.txt) |
| Родители | ADR-0015 (e2e verdict SOT, stdout), ADR-0022 (GATE-1 done-gates) |
| Связанные | PR #1517, verdict-fail-streak-2026-08-22.md (t_fb037ed1) §W4 + §S3, issue #1353 (GITHUB_RUN_ID env для OUT_DIR), issue #1478 (rsync вместо trailing-dot scp) |

## 1. Контекст и бизнес-проблема

Workflow `L: E2E Voice Test` валидирует результат прогона харнесса в шаге `Validate e2e result (real robot response)`. Источник истины — `/tmp/e2e_verdict.log` (локальный файл на runner-е, в который шагом ранее через `tee` записывается stdout харнесса, полученный через `ssh ... cat /tmp/e2e_atomic_out.log`).

В round-167 (run #32533566491, 22:39:48Z) и потенциально других e2e-прогонах этот канал **потерял данные**: `VERDICT_LOG=/tmp/e2e_verdict.log` оказался пустым, и валидатор упал в `harness-output-missing`, хотя:

- харнесс **успешно** записал `recording.wav` (1 979 406 bytes) в `/tmp/e2e_v2_20260822_013527/recording.wav` на билд-машине (10.1.1.249)
- харнесс **успешно** записал `verdict.txt` в тот же каталог (логика строк `e2e_voice_test.sh:1054`/`1100`: `echo "PASS" > "$OUT_DIR/verdict.txt"` / `echo "FAIL" > "$OUT_DIR/verdict.txt"`)
- **но** `verdict.txt` остался на 249, и **никто его не стянул на runner** для использования валидатором

Корень проблемы — **race между двумя каналами передачи verdict**:

1. **stdout-канал**: `ssh ... ${CMD} 2>&1 | tee /tmp/e2e_atomic_out.log` на runner-е → cat → `tee /tmp/e2e_verdict.log`. Любой сбой в SSH (флап сети, permission denied на root-owned /tmp) **обнуляет содержимое** на runner-е, даже если харнесс на 249 давно закончил и положил verdict.txt.
2. **Файл-канал**: `verdict.txt` в `/tmp/e2e_v2_<RID>/` на 249. Записывается `ensure_outdir` после того, как харнесс принял финальное решение (PASS/FAIL). Это **single source of truth внутри харнесса**, но workflow его не использовал.

Дополнительная боль (W1 архитектора t_fb037ed1): первый шаг `Collect e2e artifacts from build machine` использовал **наивный** `ls -1dt /tmp/e2e_v2_* | head -1` по mtime — тянул **самый свежий** каталог, а не каталог текущего RID. На билд-машине могут быть stale legacy `e2e_v2_*` от старых прогонов (08-12/08-15), и mtime-эвристика ошибалась, подменяя нужный каталог чужим.

## 2. Инвариант (как должно быть)

```text
harness verdict (PASS|FAIL) для RID  <=>  содержимое /tmp/e2e_v2_<RID>/verdict.txt на 249
                                       И совпадает с E2E_VERDICT PASS|FAIL в stdout харнесса

workflow Validate e2e result        <=  читает VERDICT_LOG (stdout-tee)
                                       + fallback scp-ит /tmp/e2e_v2_<RID>/verdict.txt с 249
                                       и при пустом VERDICT_LOG использует verdict.txt как SoT

Collect e2e artifacts               <=  всегда сначала /tmp/e2e_v2_<RID> (primary, по RID)
                                       иначе fallback по mtime (с пометкой stale)
```

verdict.txt — **канонический канал** для итогового PASS/FAIL (пишется после `ensure_outdir`, не зависит от tee/ssh-флапа). stdout харнесса — **вторичный канал** для гранулярных маркеров (`E2E_REACTION_OK`, `E2E_FEATURE_FAIL`, `E2E_NO_REACTION`); валидатор должен использовать **оба** и приоритизировать verdict.txt при пустом stdout-tee.

## 3. Решение

### 3.1. Харнесс (`e2e_voice_test.sh`) — уже пишет verdict.txt

Никаких изменений не требуется: `e2e_voice_test.sh:1054` (`echo "PASS" > "$OUT_DIR/verdict.txt"`) и `:1100` (`echo "FAIL" > "$OUT_DIR/verdict.txt"`). Это и есть single source of truth.

### 3.2. Workflow `L-E2E Voice Test.yml` — два минимальных изменения

**a) `Collect e2e artifacts from build machine` (первый экземпляр, идёт ДО Validate)** — заменить `ls -1dt ... | head -1` (наивный mtime) на **primary + fallback**:

```bash
PRIMARY_DIR="/tmp/e2e_v2_${RID}"
if sshpass -e ssh ... "[ -d '$PRIMARY_DIR' ]" 2>/dev/null; then
  REMOTE_OUT_DIR="$PRIMARY_DIR"     # канон. каталог текущего RID
else
  REMOTE_OUT_DIR="$(sshpass -e ssh ... "ls -1dt /tmp/e2e_v2_* | head -1")"   # stale fallback
fi
```

**b) `Validate e2e result` — VERDICT_PULL блок перед цепочкой grep'ов**:

```bash
VERDICT_TXT_LOCAL="/tmp/e2e_verdict_${RID}.txt"
if sshpass -e scp ... "ros2@10.1.1.249:/tmp/e2e_v2_${RID}/verdict.txt" "$VERDICT_TXT_LOCAL"; then
  if [ -s "$VERDICT_TXT_LOCAL" ]; then
    # Подмешиваем в VERDICT_LOG, чтобы дальнейшая цепочка grep'ов
    # использовала оба канала (а не падала в harness-output-missing).
    if [ -s "$VERDICT_LOG" ]; then
      printf '\n--- VERDICT_PULL (ADR-0024 fallback, %s bytes) ---\n' "$(stat -c %s $VERDICT_TXT_LOCAL)" >> "$VERDICT_LOG"
      cat "$VERDICT_TXT_LOCAL" >> "$VERDICT_LOG"
    else
      cp "$VERDICT_TXT_LOCAL" "$VERDICT_LOG"     # VERDICT_LOG потерян → verdict.txt это SoT
    fi
  fi
fi
```

Логика: если `VERDICT_LOG` уже есть — добавляем verdict.txt в хвост как audit-trail; если потерян (tee-race) — копируем целиком. **Никаких изменений в цепочке `if ! -s VERDICT_LOG; then FAIL=harness-output-missing`** — fallback выше срабатывает раньше и подменяет VERDICT_LOG на verdict.txt при потере.

### 3.3. Что НЕ делаем

- **Не дублируем verdict.txt локально** в отдельную логику — просто подмешиваем в существующий VERDICT_LOG, чтобы остальная цепочка `grep -q E2E_REACTION_OK` / `grep -q E2E_VERDICT PASS` работала без изменений.
- **Не делаем retry-loop на sshpass scp** — если 249 недоступна (сеть лежит), то и робот тоже недоступен, и harness просто не выполнился; `harness-output-missing` остаётся валидным FAIL-маркером в этом крайнем случае.
- **Не трогаем `E2E_VERDICT PASS/FAIL` в stdout** — это второй канал, его хватает (round-178 показал, что stdout-канал работает; round-167 — что race в tee бывает).
- **Не пишем отдельный helper-скрипт** в `scripts/e2e_voice_lib.sh` — VERDICT_PULL это чисто workflow-логика, ~25 строк bash, инлайнить проще чем поддерживать общий API.

## 4. Альтернативы, которые отвергли

### A. Сделать retry-loop вокруг `tee /tmp/e2e_verdict.log`
Симптоматично — решает только tee-race, не решает первичную проблему (verdict.txt — канон, а stdout-tee — производное). И retry-loop увеличит время валидации на 10-30s в worst case.

### B. Полностью убрать stdout-канал и читать только verdict.txt
Радикально, но ломает гранулярную диагностику (E2E_REACTION_OK vs E2E_FEATURE_FAIL — это разные причины FAIL, нужны для ретро). VERDICT_TXT содержит только "PASS"/"FAIL", без причин.

### C. Парсить `e2e_atomic_out.log` после его загрузки артефактом
Слишком поздно — `Collect e2e artifacts from build machine` (полный) стоит ПОСЛЕ `Validate e2e result`. Сейчас архитектура шагов не позволяет.

## 5. Обратная совместимость

- Старые харнессы (без `verdict.txt` в OUT_DIR): VERDICT_PULL-блок сделает scp, получит пустой файл или ошибку, ничего не подмешает, фоллбэк на старый stdout-канал остаётся. **Никаких регрессий.**
- Новые харнессы (с verdict.txt): при живом stdout — две записи (stdout + verdict.txt) для надёжности; при потерянном stdout — verdict.txt спасает.
- Второй `Collect e2e artifacts from build machine` (после Validate) уже использует primary-by-RID — задокументировано как reference, изменения не требует.

## 6. Verification (как проверить, что фикс работает)

1. Запустить 3 прогона `L: E2E Voice Test` подряд с дефолтным сценарием.
2. В каждом логе шага `Validate e2e result` найти строку вида `VERDICT_PULL: /tmp/e2e_verdict_<RID>.txt (3 bytes) → harness SoT verdict`.
3. `gh run list --workflow "L: E2E Voice Test" --limit 10 --json conclusion` — 3 SUCCESS подряд, **fail-rate по `harness-output-missing` = 0**.
4. Симулировать tee-race: добавить в `Validate e2e result` `rm -f $VERDICT_LOG && touch $VERDICT_LOG` сразу после tee (одноразовый debug-флаг), убедиться что verdict.txt-fallback спасает прогон от ложного FAIL.

## 7. Связанные риски и follow-up'ы

- **(R1)** Если билд-машина 249 поменяется (новая host, новый путь OUT_DIR) — фикс сломается. Mitigation: переменная `ROBOT_BUILD_HOST` в шапке job'а (пока hardcoded `10.1.1.249`). Тривиальный refactor.
- **(R2)** Если харнесс перестанет писать verdict.txt (рефакторинг ensure_outdir) — VERDICT_PULL молча вернёт пустой файл, и валидатор упадёт обратно в harness-output-missing. **Защита:** smoke-test в CI, который дёргает `e2e_voice_test.sh` с дефолтным сценарием и проверяет наличие `verdict.txt`. Не входит в этот ADR (отдельная карточка).
- **(R3)** ADR-0022 §4.1 pinит acceptance.json как обязательный для GATE-1. verdict.txt — **отдельный** канал, не путать: это прошёл/не прошёл общий цикл харнесса, а acceptance.json — прошёл/не прошёл per-step patterns + tool calls. Они дополняют, не пересекаются.

## 8. Acceptance для "done"

- [ ] PR открыт, CI зелёный
- [ ] В workflow-логе видна строка `VERDICT_PULL: /tmp/e2e_verdict_<RID>.txt (NN bytes)` для каждого прогона
- [ ] `gh run list --workflow "L: E2E Voice Test" --limit 10 --json conclusion` — 0 FAIL по `harness-output-missing`
- [ ] Симуляция tee-race (R1 verification step 4) проходит — verdict.txt-fallback спасает прогон
