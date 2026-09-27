# ADR-0134 — AI-generated assets: legal/license tracking policy (process + CREDITS.md contract)

| Поле | Значение |
|---|---|
| Статус | **Proposed** |
| Дата | 2026-09-27 |
| Автор | architect (Hermes Agent), kanban `t_2d0f27f7`, по ревью-находке issue #3050 |
| Контекст | В `src/rob_box_quest/webxr_client/public/models/environment/CREDITS.md` появилась секция "Hero props (Tripo3D, 2026-09-26)" — ассеты сгенерированы Tripo3D из Qwen reference images. ADR-0032 §1.2 R9 требует «Все ассеты — CC0 (или собственные)». AI-generated формально НЕ CC0 (юридический статус mesh'ей, сгенерированных text-to-3D моделью, не settled в большинстве юрисдикций) и НЕ собственные в строгом смысле (промежуточная генерация через proprietary Qwen reference). Текущее примечание в CREDITS.md («flag before any public redistribution») — только комментарий, не process-инвариант. Решение ниже формализует обязательный process + CREDITS.md contract + CI guard. |
| Затрагивает | (а) `docs/process/ai-generated-asset-handling.md` — новый process-doc; (б) `src/rob_box_quest/webxr_client/public/models/environment/CREDITS.md` — расширение §"Adding non-CC0 assets" явным разделом про AI-generated; (в) новые labels `legal` + `ai-asset` (уже созданы этим PR); (г) tracking issue (single source of truth) — `meta: AI-generated-asset legal/license review status`. |
| Родители | ADR-0018 (честность — не выдавать неурегулированный legal status за решённый), ADR-0032 §1.2 R9 (CC0-or-own requirement), ADR-0013 (incremental delivery — этот ADR маленький, точечный), ADR-AF-0030 (ADR-numbering SOT — домен RT, следующий свободный номер после 0133) |
| Связанные | issue #3050 (origin review finding), issue #3051 (tracking issue, создаётся этим PR), коммиты `27800a3d` + `ecfc454b` (Tripo3D hero props), `docs/process/HOTFIX.md` (операционные процедуры — стиль-референс), ADR-0099 §1.1 (style: capability-honest + ревью-комментарий как источник решения) |

> **TL;DR.** AI-generated assets (mesh/texture/audio produced by generative models)
> не должны попадать в публично-редістрибутируемый артефакт без явного sign-off
> товарища Шифу (владелец репо). Каждый такой ассет обязан: (1) быть зарегистрирован
> в tracking issue `meta: AI-generated-asset legal/license review status` с
> owner + license status + resolution date; (2) иметь соответствующую строку
> в CREDITS.md со ссылкой на tracking issue; (3) проходить pre-commit guard
> (commit trailer `<!-- ai-asset-policy: yes|pending|removed -->`). Только
> `yes` (settled license) допускает публичную редiстрибуцию.

---

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем

`src/rob_box_quest/webxr_client/public/models/environment/CREDITS.md`
(после merge коммитов `27800a3d` + `ecfc454b` в окне ревью 2026-09-26 →
2026-09-27) содержит новую секцию с примечанием:

```
**License note:** AI-generated (Tripo3D from Qwen reference images), no
third-party meshes/textures. License status of AI-generated content is not yet
settled — flag before any public redistribution.
```

Это **только флаг** в комментарии. Никакой process-инфраструктуры
(issue, due date, CI hook) вокруг него нет.

### 1.2 Почему это блокер (а не косметика)

1. **Юридический риск публичного релиза.** Если товарищ Шифу завтра попросит
   опубликовать артефакт (web-клиент, docker image, GitHub release) — никто
   не вспомнит, что в нём лежат mesh'и с unsettled license. CREDITS.md
   читают только при добавлении ассетов, не при релизе.
2. **ADR-0032 §1.2 R9 нарушен формально.** «Все ассеты — CC0 (или
   собственные)» — это **строгий** контракт. AI-generated — ни то ни другое
   в большинстве юрисдикций (включая РФ, US, EU: copyright status of
   AI-only output без значимого human authorship не settled).
3. **Тренд будет нарастать.** Tripo3D — это только начало: проект будет
   генерировать всё больше ассетов через Tripo3D / MiniMax image / MiniMax
   TTS (voice). Без policy каждый коммит будет создавать ad-hoc «flag»,
   не агрегируемый и не reviewable.
4. **Существующий §"Adding non-CC0 assets" в CREDITS.md** явно покрывает
   только CC-BY (third-party meshes/textures) — AI-generated не покрыт
   ни им, ни отдельным разделом.

### 1.3 Что НЕ так в текущем CREDITS.md

| Аспект | Сейчас | Нужно |
|---|---|---|
| Owner / accountable human | Не указан | товарищ Шифу (явно, в каждой AI-generated строке) |
| Due date / milestone | «flag before redistribution» — без даты | Tracking issue с target resolution milestone |
| Tracking issue / epic | Отсутствует | Один issue `meta: AI-generated-asset legal/license review status` (per-area) |
| CI guard при `git commit` | Никакого | pre-commit: `<!-- ai-asset-policy: yes|pending|removed -->` trailer обязателен при `git diff --diff-filter=A` на `public/models/**` |
| Агрегация по проекту | Невозможна — флаги разбросаны по разным `CREDITS.md` | Labels `ai-asset` + `legal` на tracking issue — `gh issue list --label ai-asset` даёт полный список |

### 1.4 Не путать с уже существующими labels

`gh label list` подтверждает:

```
ai-generated    #7B61FF    "Created by AI agent (GSD workflow)"
```

Этот label означает «issue создан AI-агентом через GSD-workflow», а **не**
«ассет сгенерирован AI-моделью». Семантическая коллизия: в issue #3050
можно было бы поставить `ai-generated`, но это было бы **ложно** —
создатель issue — Hermes Agent (architect), а не AI-генерированный ассет.

Решение: **новый label `ai-asset`** (D93F0B, «AI-generated asset — distinct
from ai-generated agent label») + **новый label `legal`** (D93F0B,
«Legal/license/IP review required»). Оба созданы этим PR.

---

## 2. Рассмотренные альтернативы

### 2.1 «Ничего не делать — оставить только флаг в CREDITS.md»

| За | Против |
|---|---|
| Нулевая работа | Юридический риск при release не закрыт |
| Минимальный diff | Не масштабируется (каждый AI-ассет — новый ad-hoc флаг) |
| | Не reviewable (нет агрегации) |
| | ADR-0032 R9 формально нарушен |

**Отвергнуто:** не решает root cause.

### 2.2 «Сделать CI guard, который просто сканирует CREDITS.md на ключевое слово `AI-generated`»

| За | Против |
|---|---|
| Минимальный diff | Ложные срабатывания на исторические CC-BY с упоминанием AI |
| | Не помогает с tracking/agрегацией |
| | Не масштабируется на новые типы ассетов (audio, textures) |

**Отвергнуто:** band-aid, не policy.

### 2.3 (Выбрано) Трёхслойная policy: process-doc + CREDITS.md contract + commit trailer + tracking issue

| Слой | Что даёт |
|---|---|
| `docs/process/ai-generated-asset-handling.md` | Операционная процедура (когда, кто, как) |
| CREDITS.md — расширенный раздел | Видно при code review, ссылка на tracking issue |
| Commit trailer `<!-- ai-asset-policy: ... -->` | Машино-читаемый, проверяемый pre-commit guard'ом |
| Tracking issue с labels `legal` + `ai-asset` | Single source of truth, reviewable, milestone-bound |

**Trade-off:** чуть больше бюрократии на каждом AI-generated коммите
(3 строки trailer + 1 строка в CREDITS.md + checkbox в tracking issue),
но это **стоимость правовой определённости при release**.

### 2.4 «Запретить AI-generated assets полностью (как в ADR-0032 R9)»

| За | Против |
|---|---|
| Полная определённость | Потеря productivity (Tripo3D экономит часы на каждую mesh) |
| ADR-0032 R9 literal | Шифу может осознанно принять legal risk ради скорости |
| | Запрет не соответствует тренду (весь рынок идёт в AI-ассеты) |

**Отвергнуто:** слишком жёстко. ADR-0032 R9 остаётся целью по умолчанию,
но AI-generated допустим как **explicit exception** через process.

---

## 3. Решение (детально)

### 3.1 Process-doc: `docs/process/ai-generated-asset-handling.md`

Новый файл. Содержит:

1. **Scope**: что считается AI-generated asset (text/image→mesh, text/image→texture, text→audio, voice cloning).
2. **Default policy**: запрет, как в ADR-0032 R9. AI-generated — **exception**, не default.
3. **Exception procedure**:
   - Сгенерировать ассет.
   - ДО `git commit` — завести checkbox в tracking issue (per-area, см. §3.2).
   - ДО `git commit` — добавить commit trailer (см. §3.3).
   - В CREDITS.md (или per-area `CREDITS.md`) добавить строку со ссылкой на tracking issue (см. §3.4).
4. **Pre-public-release checklist**: каждый milestone close / каждый
   release tag обязан пройти review `gh issue list --label ai-asset --state open`
   и либо resolved, либо явно carry-forward с новой датой.

### 3.2 Tracking issue: single source of truth per area

- **Pattern**: один issue на area (per-component / per-repo-area).
  На текущий момент — один issue `meta: AI-generated-asset legal/license
  review status` (уже создан этим PR: issue #3051) покрывает весь `rob_box_quest`.
- **Labels**: `legal` + `ai-asset` + `area:<component>` + `process`.
- **Owner**: товарищ Шифу (явно в issue body).
- **Structure**: checklist с одним item на каждый AI-generated ассет
  (asset path, generator, generation date, license status, owner, resolution date).
- **Lifecycle**: issue открыт пока есть хотя бы один `pending` ассет.
  Закрывается когда все `yes` (settled license) ИЛИ все `removed`.

### 3.3 Commit trailer: машино-читаемый flag

Каждый commit, добавляющий AI-generated ассет (по `git diff --diff-filter=A`
на `public/models/**`, `assets/**`, `*.glb`, `*.ktx2`, `*.hdr`,
`*.wav`, `*.mp3`, `*.ogg`), обязан содержать trailer:

```
<!-- ai-asset-policy: yes|pending|removed -->
```

| Значение | Когда |
|---|---|
| `yes` | Settled license (например, вы приняли AI-output как собственный work-for-hire, или явная CC0-эквивалентная лицензия от модели) — публичная редiстрибуция разрешена |
| `pending` | Юридический статус unsettled (default для новых ассетов) — **ЗАПРЕЩЕНО для публичного release** |
| `removed` | Ассет был удалён из репо, но commit остаётся в истории |

Pre-commit guard (отдельная задача — **НЕ** в этом PR): bash-скрипт
`scripts/agent_flow/agent-flow-ai-asset-guard.sh` + pre-commit hook,
который:

```bash
if git diff --cached --diff-filter=A --name-only | \
     grep -qE '\.(glb|ktx2|hdr|wav|mp3|ogg)$|^(public|assets)/.*\.glb'; then
  if ! git log -1 --format=%B | grep -qE '<!-- ai-asset-policy: '; then
    echo "ERROR: new asset requires '<!-- ai-asset-policy: ... -->' trailer"
    exit 1
  fi
fi
```

**Out of scope этого ADR** (фиксируется как future-work в §6), чтобы не
раздувать PR.

### 3.4 CREDITS.md — расширение существующего раздела

Существующий §"Adding non-CC0 assets" в
`src/rob_box_quest/webxr_client/public/models/environment/CREDITS.md`
дополняется (а не заменяется) явным подразделом про AI-generated:

```markdown
## AI-generated assets

AI-generated assets (mesh/texture/audio produced by generative models
such as Tripo3D, MiniMax image, MiniMax TTS, voice cloning) are
**NOT CC0** by default. Each such asset MUST:

1. Be tracked in the meta tracking issue
   `meta: AI-generated-asset legal/license review status` (per-area).
2. Appear in CREDITS.md with: generator name, generation date,
   tracking-issue link, current license status, owner.
3. Have a `<!-- ai-asset-policy: yes|pending|removed -->` commit trailer.

`pending` assets MUST NOT be included in public releases.
```

### 3.5 Labels (уже созданы этим PR)

```
legal    D93F0B  Legal/license/IP review required
ai-asset D93F0B  AI-generated asset (mesh/texture/audio)
```

`legal` — общий для всех legal/IP вопросов (потенциально пригодится
для других case'ов: proprietary font в UI, проприетарный codec и т.п.).
`ai-asset` — специально для разделения с `ai-generated` (label для
AI-агентских issues).

---

## 4. Треade-off'ы (по decision framework)

| Решение | Trade-off |
|---|---|
| Tracking issue (а не таблица в `docs/`) | Агрегация через `gh issue list` + label filter vs необходимость создавать и поддерживать issue |
| Commit trailer (а не только CREDITS.md) | Машино-читаемость для CI guard vs лишняя строка в commit body |
| Per-area tracking issue (а не один глобальный) | Granular ownership, меньше шума при release vs лёгкое дублирование при нескольких area |
| `pending` как default, `yes` через sign-off | Не блокирует разработку, но блокирует release vs юридический риск на разработчике |
| Только process + doc (без CI guard в этом PR) | Маленький PR vs guard остаётся manual |
| Label `legal` (а не `compliance`) | Короткое, neutral, расширяемо vs менее специфично |

---

## 5. Что будет, если НЕ делать это сейчас

- Каждый следующий AI-generated коммит будет добавлять ad-hoc «flag» в
  CREDITS.md без tracking.
- При попытке публичного release (Docker Hub, GitHub release, web deploy)
  товарищ Шифу либо: (а) отложит release пока не разберётся вручную
  (потеря времени), либо (б) выпустит с unsettled license (юридический риск).
- Тренд усиливается: Tripo3D, MiniMax image, MiniMax TTS — все будут
  генерировать всё больше ассетов.

**Это policy gap, который не блокирует CI, но проявится при публичном
релизе.** Medium severity по issue #3050.

---

## 6. Future work (отдельные карточки — НЕ в этом PR)

- **CI/pre-commit guard** (скрипт `scripts/agent_flow/agent-flow-ai-asset-guard.sh`
  + pre-commit hook + интеграция с merge-gate).
- **Release-checklist integration**: `agent-flow-release-tag.sh` обязан
  вызывать `gh issue list --label ai-asset --state open` и блокировать
  release если есть `pending` без явного override от Шифу.
- **Per-area duplication**: если AI-generated assets появятся в
  `src/rob_box_voice/`, `src/rob_box_perception/`, etc. — открыть
  per-area tracking issues с теми же labels.
- **Шаблон CREDITS.md-строки для AI-generated**: вынести в
  `docs/process/ai-generated-asset-handling.md` как copy-paste-ready.

---

## 7. Acceptance criteria этого ADR

- [ ] Tracking issue создан с labels `legal` + `ai-asset` + `area:rob_box_quest` + `process` (issue #3051).
- [ ] `docs/process/ai-generated-asset-handling.md` создан, ссылается на этот ADR.
- [ ] `src/rob_box_quest/webxr_client/public/models/environment/CREDITS.md` расширен подразделом «AI-generated assets» со ссылкой на tracking issue.
- [ ] Labels `legal` и `ai-asset` созданы в репо (подтверждено `gh label list`).
- [ ] Коммиты `27800a3d` + `ecfc454b` (или их successors) явно
      cross-linked из tracking issue.
- [ ] Issue #3050 (origin review) получает ссылку на этот ADR + tracking issue.

---

## 8. Почему это архитектурный долг (а не новый feature)

- ADR-0032 R9 уже требует CC0. Этот ADR не меняет требование — он
  формализует exception procedure для AI-generated.
- CREDITS.md §"Adding non-CC0 assets" уже существует. Этот ADR
  дополняет его AI-generated разделом.
- Tracking issues в репо уже pattern (см. issue #2017, #1989, #1990).
  Этот ADR переиспользует pattern с новыми labels.

**Honesty note (ADR-0018):** я (architect) **не проводил** юридический
ревью — это работа товарища Шифу как legal owner. Этот ADR только
формализует **process** вокруг неопределённого юридического статуса, не
пытается его разрешить.
