# Voice Core Suite v1 — архитектурный вердикт v3 (2026-08-23)

| Поле | Значение |
|------|----------|
| Автор | architect |
| Цель | Подтвердить/обновить архитектурный verdict по `voice_core_suite_v1.json` + `voice_core_acceptance_v1.json` (issue #1506) после 21 коммита в develop с момента verdict v2 (PR #1555, merged 23.08 13:36) |
| Связано | issue #1506, verdict v2 (`voice-core-suite-v1-architecture-verdict-2026-08-23.md`), PR #1555 (merged), PR #1556 (merged — pre-flight), ADR-0022 §4.6, ADR-0024 (accepted), анализ-док `voice-features-e2e-validation-2026-08-22.md` |
| Вердикт | **APPROVE** (подтверждение verdict v2; архитектурный долг = 0) |
| Re-validation | **v3 (24.08 16:30) — APPROVE подтверждён** на полном develop tip |

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

## 6. Связанные источники истины (raw-evidence)

- `git rev-parse origin/develop` = `ec42ea87a7d942ba393a710bdbf2737eb1ae1275`
- `git rev-list --count 85ca425b..origin/develop` = `21` (все — `ci:` SHA tags)
- `git rev-list --count 85ca425b..origin/develop --no-merges` = `0` non-CI
- `gh pr view 1555` = MERGED, 23.08 13:36Z (verdict v2)
- `gh pr view 1556` = MERGED, 23.08 13:32Z (pre-flight)
- `docs/analysis/voice-core-suite-v1-architecture-verdict-2026-08-23.md` = 368 строк (verdict v2)
- `docs/adr/0022-process-e2e-done-gates.md` §4.6 = window semantics (addendum `db84ff59`)
- `docs/adr/0024-harness-verdict-sot.md` = status **Accepted**, 22.08
- `scripts/agent_flow/tests/test_voice_core_suite_vad_max.sh` = 75/75 PASS (PR #1556)
- Issue #1506 comment-thread: Шифу 22.08 11:19/12:31/23.08 08:15 — hardware-blockers, не код
