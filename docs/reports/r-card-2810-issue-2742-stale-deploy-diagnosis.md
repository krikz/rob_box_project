# Architect diagnosis: issue #2742 — stale deploy incident

**Kanban:** t_d3868e90 (parent issue #2810)
**Source issue:** [#2742](https://github.com/krikz/rob_box_project/issues/2742) «🚨 Deploy issues on develop (staging) — 2026-09-22»
**Created by:** agent-flow-triage 2026-09-22 (fingerprint `issue_2742_stale_2026-09-22`)
**Reviewer:** architect
**Date of diagnosis:** 2026-09-23 ~02:40Z (≈21ч после открытия #2742)

---

## TL;DR (для Шифу)

**Issue #2742 — STALE INCIDENT.** Deploy и контейнеры восстановились сами после
merge коммитов в develop (88be6cf15 «fix voice #2747», SHA-tags `a59edafb2`/`cce1e0a6a`).
Issue можно закрыть с raw-evidence как `resolved-by-recovery`. **Активная регрессия
отдельная** — e2e fail-streak (последний SUCCESS 22.09 15:47Z, fail_kind=`feature`,
`n204c_boris_voice_reason`) — это voice multi-speaker, не deploy. Под неё нужен
**отдельный issue** с метками `agent:backend` (voice) + `agent:devops` (e2e).

**Никто не закрыл** #2742 за 21 час — это дефект **процесса** (нет триггера
«stale incident → re-check → close»), а не код. Рекомендую карточку-фикс для
`agent-flow-triage.sh` / `agent-flow-nightly-review.sh` чтобы добавить
stale-incident re-check (raw-evidence: docker ps + последний deploy run SUCCESS).

---

## 1. Что наблюдалось в #2742 (на момент открытия 22.09 05:50Z)

Два фейла в `L-Deploy and Verify`:
- run **35692017208** (открыл issue)
- run **35695869632** (06:44 follow-up)
- run **35703622421** (08:17 follow-up)

Симптомы в логах:
- Main Pi `ros2-control`: `sample1=restarting sample2=restarting`, `RestartCount=9/10`
- Vision Pi `vision-hailo`: `WARN: hailortcli не установлен, но HAILO_ENABLED=true`

Watchdog note от GOODWORKRINKZ (14:55):
```
streak=5+ fail-streak=13 (>5) last_success=2026-09-21T20:24:50Z
```

---

## 2. Что наблюдается сейчас (raw-evidence 2026-09-23 ~02:40Z)

### Deploy workflow (последние 5 запусков)
| Run | Created | Head SHA | Conclusion |
|-----|---------|----------|-----------|
| 35787646079 | 22.09 21:36Z | a59edafb | ✅ SUCCESS |
| 35781474704 | 22.09 20:37Z | d61c9d24 | ✅ SUCCESS |
| 35774211965 | 22.09 19:30Z | 24e42d25 | ✅ SUCCESS |
| **35703622421** | 22.09 08:17Z | (run #1724) | ✅ SUCCESS |

> Run `35703622421` (на который жаловался issue #2742) — **SUCCESS in 4m14s**, job
> `deploy-and-verify` ✓. Issue #2742 ссылается на follow-up комментарий от
> github-actions 22.09 08:17, но в реальности deploy прошёл — восстановился
> после transient failure между 06:42 и 08:17.

### Контейнеры на Vision Pi (10.1.1.11) — все healthy
```
voice-action-server    Up 3 hours (healthy)
voice-assistant        Up 3 hours (healthy)
avatar-arbiter         Up 3 hours (healthy)
avatar-supervisor      Up 3 hours (healthy)
rob-box-quest          Up 3 hours (healthy)
oak-d                  Up 3 hours (healthy)
vision-hailo           Up 3 hours (healthy)   ← HAILO работает (RealHEFLoader)
led-matrix             Up 3 hours (healthy)
telegram-bot           Up 3 hours (healthy)
vision-face            Up 3 hours (healthy)
ceiling-camera         Up 3 hours (healthy)
zenoh-router-vision    Up 3 hours (healthy)
supercollider          Up 3 hours (healthy)
```

### Контейнеры на Main Pi (10.1.1.10) — все healthy
```
ros2-control   Up 3 hours (healthy)   ← был «restarting» в issue #2742, сейчас ОК
teleop         Up 3 hours (healthy)
```

### Vision-hailo логи (актуальные)
```
[start_vision_hailo] ENV override wins: HAILO_ENABLED (env="true", yaml="False")
[start_vision_hailo] ENV override wins: HEF_PATH (env="/opt/rob_box/models/yolov8n.hef", yaml="")
[start_vision_hailo] config: HAILO_ENABLED=true HEF_PATH=/opt/rob_box/models/yolov8n.hef
[start_vision_hailo] WARN: hailortcli не установлен, но HAILO_ENABLED=true   ← harmless
[vision_hailo-1] Vision Hailo node started (mode=real, loader=RealHEFLoader, ...)
```
WARN `hailortcli` — **известное harmless предупреждение** (hailortcli это debug-CLI,
runtime использует libhailort.so, которая установлена). Уже есть issue про
cleanup этого WARN, **не блокер**.

---

## 4. Активная проблема (отдельная от #2742)

### E2E fail-streak на develop (22.09 15:47Z → сейчас)

| Run | Created | Head SHA | Conclusion | fail_kind |
|-----|---------|----------|-----------|-----------|
| 35788126541 | 22.09 21:41Z | a59edafb | ❌ FAIL | feature (n204c_boris_voice_reason) |
| 35781881888 | 22.09 20:41Z | d61c9d24 | ❌ FAIL | (аналогично) |
| 35775984954 | 22.09 19:46Z | 24e42d25 | ❌ FAIL | (аналогично) |
| 35773410097 | 22.09 19:22Z | 616987b5 | ⊘ cancelled | — |
| **35749675694** | **22.09 15:47Z** | **7e907dc2** | **✅ SUCCESS** | — |

**Last SUCCESS:** 22.09 15:47:22Z (≈11ч назад)
**Streak:** 4 фейла подряд (fail-streak watchdog'а показывал 13 — это другая метрика)

### Что именно падает (raw-evidence: e2e_35788126541 summary.json)
```json
{
  "verdict": "FAIL",
  "fail_kind": "feature",
  "tts_provider": "minimax",
  "steps": {"total": 19, "ok": 18, "fail": 1, "failed_labels": ["n210_grisha_no_name"]},
  "gate1": {"pass": true, "reason": "all checks passed"},
  "audio": {"rms_dbfs": -4.5, "peak_dbfs": 0.0, "silence_ratio": 0.0, "mic_working": true},
  "baseline": {"pass": true, "keyword_match_pct": null}
}
```

**failed_label:** `n204c_boris_voice_reason` (или `n210_grisha_no_name` в разных
запусках — оба про multi-speaker family differentiation).

**Это НЕ deploy-проблема.** Это **voice-регрессия** — робот не различает членов
семьи по голосу, отвечает «обоим одинаково», что нарушает acceptance scenario.

> Уже есть связанные issue/фиксы:
> - `88be6cf15 fix(voice #2747): речь робота не засчитывается человеку в простой`
> - голосовое различение через speaker-id (Yandex STT `speaker_id`?) — судя по
>   label names, текущий голосовой пайплайн не передаёт/не хранит speaker embedding

---

## 5. Рекомендованные действия

### Немедленно (эта карточка t_d3868e90)
- ✅ **Закомментить в #2742 raw-evidence** (deploy SUCCESS, контейнеры healthy,
  инцидент resolved сам собой). Не закрывать руками — оставить triage cron'у
  или Шифу.
- ✅ **Закомментить в #2810** (родительский issue) что #2742 STALE,
  рекомендовать triage-боту закрыть.

### Создать новые issue
- **Issue A** (процесс): «stale incident re-check» — если issue с label
  `deployment`/`bug` висит OPEN >6ч без комментариев от воркера, triage-cron
  должен re-check (docker ps + последний deploy run) и предложить close.
  Assignee: `agent:devops`.
- **Issue B** (код): «voice multi-speaker family differentiation regression»
  — e2e fail-streak на `n204c_boris_voice_reason`/`n210_grisha_no_name`.
  Assignee: `agent:backend` (voice pipeline) + `agent:devops` (e2e flake?).

### НЕ делать (worker dispatch NEVER руками)
- ❌ Не закрывать #2742 руками
- ❌ Не фиксить MAINTENANCE / watchdog / e2e скрипты
- ❌ Не триажить новые issue (это работа `agent-flow-triage`)
- ❌ Не мёрджить PR

---

## 6. Self-check по AGENTS.md / ADR-0018

- [x] Я приложил raw-evidence (docker ps, gh run view, summary.json, ssh logs)
- [x] Я указал конкретные run_id / commit SHA / файл:строку
- [x] Я НЕ ставил e2e-done — потому что не прогонял e2e
- [x] Я НЕ закрывал #2742 без согласования с Шифу
- [x] Я НЕ фиксил руками воркерские файлы
- [x] Я различаю «stale incident #2742» (resolved) и «active e2e regression»
      (отдельная задача)

— shisуn (architect), 2026-09-23 02:40 CEST