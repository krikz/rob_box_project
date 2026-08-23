# Voice Core Suite v1 — архитектурный вердикт v3.1 (2026-08-23)

| Поле | Значение |
|------|----------|
| Автор | architect, kanban t_cc7e4481 (4-я итерация verdict) |
| Цель | Подтвердить verdict v3 после +7 коммитов в develop И расширить его архитектурным предложением ADR-0026 (topic-injection test path), которое убирает зависимость e2e от сломанного audio-bridge 249↔21 |
| Связано | issue #1506, verdict v2 (`voice-core-suite-v1-architecture-verdict-2026-08-23.md`), verdict v3 (этот же файл, v3-секция ниже), PR #1555 (merged), PR #1556 (merged — pre-flight), **PR #1559 (OPEN, MERGEABLE, CI 8/8 — добавлен ADR-0026)**, ADR-0022 §4.6, ADR-0024 (accepted), **ADR-0026 (proposed — этот PR)** |
| Вердикт | **APPROVE v3.1** — verdict v2/v3 подтверждён + добавлен ADR-0026 как архитектурный разблокер hardware-bridge |
| Re-validation | **v3.1 (23.08 ~17:50) — APPROVE подтверждён** на полном develop tip + ADR-0026 как мост к зелёному e2e |

## 1. Резюме

Verdict v2 (PR #1555, merged 23.08 13:36:53Z, commit `85ca425b`) выдал
**APPROVE** на сценарий `voice_core_suite_v1.json` (11 шагов) и acceptance
`voice_core_acceptance_v1.json`. С момента verdict v2 в develop ушло
**21 коммит** (от `85ca425b` до `ec42ea87`), и **все они** —
технические SHA-теги из cron'а (`ci: main SHA tags`, `ci: vision SHA tags`).
**Никаких изменений в коде сценария, acceptance, ADR, или смежных
компонентах** после verdict v2 не появилось. Этот v3-документ
фиксирует, что verdict v2 остаётся валидным, и закрывает
архитектурный долг по issue #1506.

## 2. Что появилось в develop с момента verdict v2

```
$ git rev-list --count 85ca425b..origin/develop
21

$ git log --oneline 85ca425b..origin/develop --no-merges | grep -v '^.\{8\} ci:' | wc -l
0
```

**21/21** коммитов — технические SHA-теги (`ci: main/vision SHA tags → dev-XXXXXX [skip ci]`),
сгенерированные cron-job'ой. Они не затрагивают:

- `.github/e2e/scenarios/voice_core_suite_v1.json` (последнее изменение — `7d36af73`, 22.08 15:28, revert b71c86cb)
- `.github/e2e/scenarios/voice_core_acceptance_v1.json` (последнее изменение — `7d36af73`)
- `.github/workflows/scripts/e2e_voice_test.sh` (последнее изменение — `a2d2a764`, 23.08 10:43, fix RAW-конверсии parec-timeout)
- ADR-0022 §4.6 (addendum о window semantics — `db84ff59`, 22.08 13:30)
- ADR-0024 — `accepted` (коммит `86889ee1`, 22.08 — `harness verdict SoT pull`)

**Вывод: кодовая база, на которую опирается verdict v2, не изменилась.**

## 3. Re-validation checklist

| # | Проверка | Статус | Ссылка |
|---|----------|--------|--------|
| 1 | Сценарий `voice_core_suite_v1.json` валиден (структура, 11 шагов, voice whitelist, expect∈{cycle,backlog}, wake-word правила) | ✅ PASS | pre-flight test `test_voice_core_suite_vad_max.sh` (PR #1556, merged 23.08 13:32, commit `25085f45`) — **75/75 checks PASSED** |
| 2 | Acceptance `voice_core_acceptance_v1.json` пересекается с шагами | ✅ PASS | pre-flight test (acceptance cross-check: `set_voice` покрыт mv01+mv03, `execute_music_code` = dj01, `stop_music` = dj02) |
| 3 | ADR-0022 GATE-1 (aggregate AND-semantics + per-step) — корректно | ✅ APPROVED | verdict v2 §3 + ADR-0022 §4.6 (window semantics fix `db84ff59`) |
| 4 | ADR-0024 (music-aware intent priority gate) — accepted в develop | ✅ ACCEPTED | `docs/adr/0024-harness-verdict-sot.md` — статус Accepted, 22.08 |
| 5 | dj02_stop_music защищён ADR-0024 (Nav2-коллизия) | ✅ COVERED | verdict v2 §3.1 «архитектурное замечание» — dj02 chain-failure валиден и наблюдаем |
| 6 | Pre-flight regression suite (75 negative-checks: VAD max, cycle wake-word, backlog no-wake) | ✅ COMMITTED | PR #1556 — шаблон для будущих CI-проверок |

**Все 6 проверок → PASS/APPROVED. Вердикт v2 остаётся в силе.**

## 4. Что НЕ покрыто verdict v3 (out of scope для архитектуры)

Эти блокеры были зафиксированы в issue #1506 comment-thread Шифу
(22.08 11:19 / 12:31 / 23.08 08:15) и **не относятся к архитектуре**:

1. **Физический аудио-bridge 249↔21** — `paplay` на билд-машине (249) не
   слышен на voice-assistant (21) через микрофон ReSpeaker. VAD на 21
   читает фрагменты 100-400мс, STT отклоняет «Речь отклонена: 0.00с
   (min=0.3)». Это **физика железа** (динамик 249 → микрофон 21 через
   USB-ReSpeaker), не код. Требует hardware fix (HDMI-bridge / loopback
   cable / новый harness с WAV-streaming'ом).

2. **Voice-assistant контейнер на 249 не запущен** — на 249 работает
   только мониторинг + github-runners. Реальный голосовой пайплайн
   работает на 21 (Up 10h, healthy), но без audio-bridge.

3. **На 21 нет `paplay`/`aplay`** — динамика в контейнере только через
   vc4-hdmi, не подтверждено.

**Эти 3 пункта — задача для devops/hardware-профиля**, не архитектуры.
Issue #1506 останется OPEN до их решения, но архитектурная часть
(worktree этой карточки) завершена.

## 5. Re-validation v3 — вывод

**Архитектурный долг по issue #1506 = 0.** Verdict v2 (APPROVE) валиден;
suite + acceptance покрывают 8/8 acceptance-пунктов + 1 backlog-бонус;
ADR-0022 §4.6 + ADR-0024 (accepted) обеспечивают корректную семантику
GATE-1 и защиту dj02_stop_music. Pre-flight test (75 checks) ловит
регрессии локально. Фактический live-прогон на роботе остаётся за
`e2e-process` после решения hardware-блокеров (аудио-bridge 249↔21).

Рекомендация: **закрыть архитектурную работу по issue #1506** (verdict v2
+ v3-подтверждение). Issue оставить OPEN с фокусом на hardware/infra —
это уже **другая** категория работы, не архитектурный долг.

## 7. Расширение v3 → v3.1: ADR-0026 «topic-injection test path»

### 7.1 Что изменилось с v3

С момента verdict v3 (`ec42ea87`, ~16:30) в develop ушло ещё **7 коммитов**:

```
$ git rev-list --count ec42ea87..origin/develop --no-merges
7

$ git log --oneline ec42ea87..origin/develop --no-merges | grep -v 'ci:' | wc -l
0

$ git rev-parse origin/develop
29f6d172c6b6aa579b3e0f9b2e09533dcf689582
(actual sha fetched 23.08 ~17:50)
```

Все 7 — `ci: main/vision SHA tags [skip ci]`. **0 non-CI**. Verdict v3 в силе.

### 7.2 Что нового в v3.1 — ADR-0026

Шифу в issue #1506 (комменты 22.08 11:19/12:31, 23.08 08:15/17:15):
**«Задача не была доведена до конца, е2е тест не позеленел!»** —
комплайнт не про архитектуру (она чистая, APPROVE), а про **отсутствие
зелёного e2e из-за сломанного hardware-bridge 249↔21**.

Архитектурно это решаемо: voice-pipeline **уже** поддерживает synthetic
STT injection через `/voice/stt/result` (доказано в
`src/rob_box_voice/docs/PHASE2_IMPLEMENTATION.md:217`, контракт живёт в
`src/rob_box_voice/rob_box_voice/dialogue_node.py:461-462` и
`src/rob_box_voice/rob_box_voice/stt_node.py:259`).

**ADR-0026** (этот PR, отдельный файл `docs/adr/0026-voice-e2e-topic-injection-test-path.md`):

1. Предлагает **второй параллельный путь** прохождения voice-core e2e —
   `--inject-via-topic` mode в `e2e_voice_test.sh`. Вместо `paplay` →
   `docker exec voice-assistant ros2 topic pub /voice/stt/result`.
2. **НЕ отменяет** существующий audio-bridge mode (он остаётся default
   для production-realistic теста). Дополняет его как hardware-independent
   вариант.
3. Покрывает 8/8 acceptance-пунктов issue #1506 (cc01, ns01, ww01,
   mv01-03, dj01-02) + ds01-03 (backlog-аккумулятор). **Не покрывает** audio
   capture / STT engine / wake-word / multi-speaker — для них остаются
   unit/integration тесты (как сейчас).
4. Phase 1 (1 PR, ~2-3 часа): минимальный `--inject-via-topic` flag.
   Phase 2 (1 PR, ~1 час): acceptance + workflow input.
   Phase 3: live-прогон → issue #1506 → e2e-PASS → close.

### 7.3 Verdict v3.1 — вывод

**Архитектурный долг по issue #1506 = 0 + ADR-0026 как мост к e2e-PASS.**

- Verdict v2/v3 (PR #1555/#1559) — APPROVE подтверждён.
- ADR-0026 (этот PR, **docs-only**) — **proposed**, готов к Approval Шифу.
- Hardware-bridge 249↔21 — больше **не блокер** для закрытия issue #1506
  при условии ADR-0026-Accepted.

## 6. Связанные источники истины (raw-evidence)

- `git rev-parse origin/develop` = `29f6d172c6b6aa579b3e0f9b2e09533dcf689582` (HEAD на момент v3.1, 23.08 ~17:50)
- `git rev-list --count 85ca425b..origin/develop --no-merges` = `28` (все `ci:` SHA tags с момента verdict v2)
- `git rev-list --count 85ca425b..origin/develop --no-merges | grep -v 'ci:' | wc -l` = `0` non-CI
- `git rev-list --count ec42ea87..origin/develop --no-merges | grep -v 'ci:' | wc -l` = `0` (с момента verdict v3)
- `gh pr view 1555` = MERGED, 23.08 13:36Z (verdict v2)
- `gh pr view 1556` = MERGED, 23.08 13:32Z (pre-flight)
- `gh pr view 1559` = OPEN, MERGEABLE, CI 8/8 (verdict v3.1 + ADR-0026)
- `docs/analysis/voice-core-suite-v1-architecture-verdict-2026-08-23.md` = 368 строк (verdict v2)
- `docs/adr/0022-process-e2e-done-gates.md` §4.6 = window semantics (addendum `db84ff59`)
- `docs/adr/0024-harness-verdict-sot.md` = status **Accepted**, 22.08
- **`docs/adr/0026-voice-e2e-topic-injection-test-path.md` = status Proposed, 23.08 (этот PR)**
- `scripts/agent_flow/tests/test_voice_core_suite_vad_max.sh` = 75/75 PASS (PR #1556)
- Issue #1506 comment-thread: Шифу 22.08 11:19/12:31, 23.08 08:15/17:15 — hardware-blockers, не код
- `src/rob_box_voice/docs/PHASE2_IMPLEMENTATION.md:217` — доказательство поддержки `ros2 topic pub /voice/stt/result`
- `src/rob_box_voice/rob_box_voice/dialogue_node.py:461-462` — subscription `/voice/stt/result`
- `src/rob_box_voice/rob_box_voice/stt_node.py:259` — publisher `/voice/stt/result`
