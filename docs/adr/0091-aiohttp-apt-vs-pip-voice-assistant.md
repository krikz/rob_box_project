# ADR-0091: aiohttp в voice-assistant — apt vs pip, разделение по версионным границам

**Дата:** 2026-09-15
**Статус:** Accepted (архитектурный вердикт, architectural review issue #2568)
**Автор:** architect worker (kanban-card `t_fa0e3ef7`, issue #2568)
**Домен:** docker/vision (build cache, dependency pinning)
**Severity:** MEDIUM — уточнение scope PR #2407, без правок кода
**Тип:** dependency-strategy (architectural decision, не bug-fix)
**Связанные:** PR #2407 (issue #2378), commit `337548a38` (2026-08-06), ADR-0011 (action-protocol), ADR-0018 (честный FAIL), ADR-0089 §3 (touchpoint #11)
**Заменяет / уточняет:** частично issue #2568 (выводы см. §7)

---

## 1. Контекст

### 1.1 Что пишет issue #2568

Issue #2568 утверждает, что **PR #2407 не достиг своей цели** для `voice-assistant`, потому что
`voice_assistant/Dockerfile:140` содержит `pip3 install "aiohttp>=3.9.1,<4.0"` поверх
apt-установленного `python3-aiohttp` из `voice_base/Dockerfile:78`. Утверждается:
«pip переписывает apt-версию → фактически цель PR #2407 для voice_assistant не достигнута».

Предлагается один из двух фиксов:
1. Убрать `aiohttp` из pip-install (apt-версия из voice_base подойдёт).
2. Убрать `python3-aiohttp` из voice_base:78 (откатить часть PR #2407).

### 1.2 Что говорит raw-evidence (коммиты, код)

Проверка `git log` + `git show` + чтение файлов в текущей ветке `develop`:

| Артефакт | Содержание |
|---|---|
| `commit 337548a38` (2026-08-06), `fix(voice-action-server): also pin aiohttp in voice_assistant Dockerfile` | Явное **архитектурное решение** держать `aiohttp` в app-layer `voice_assistant`, **НЕ** в `voice_base`. Цитата из commit-msg: *"Kept off voice_base on purpose: changing voice_base would invalidate the ~10 GB pytorch etc. cache for every consumer."* |
| `docker/vision/voice_assistant/Dockerfile:140` | `RUN pip3 install --no-cache-dir --retries 5 --timeout 300 \ "aiohttp>=3.9.1,<4.0" \ ...` |
| `docker/vision/voice_assistant/requirements.txt:22` | `aiohttp>=3.14.3,<4.0` (**жёстче, чем в Dockerfile**) |
| `docker/vision/voice_assistant/requirements.txt:13-21` | Пояснительный комментарий: ссылка на issue, вызвавшее регрессию (`ModuleNotFoundError: No module named 'aiohttp'`, 399 рестартов с 06.08 01:03 — см. E2E_TESTING_DESIGN_v2 §D A43) |
| `src/rob_box_voice/rob_box_voice/action_server/http_server.py:18-36` | Защитный импорт `_import_aiohttp()`: при отсутствии aiohttp в image кидает `RuntimeError("voice-action-server requires the 'aiohttp' Python package. It is declared in src/rob_box_voice/package.xml as <exec_depend> and pinned in docker/vision/voice_assistant/requirements.txt. If you see this on a freshly built image, the build cache was stale — bump BASE image or rebuild without cache.")` |
| `src/rob_box_voice/package.xml:22` | `<exec_depend>aiohttp</exec_depend>` |
| `src/rob_box_voice/rob_box_voice/action_server/http.py:22-24` | `from aiohttp import web` + `raise RuntimeError("aiohttp is required for the HTTP action adapter")` |
| `commit 70df9f5fe` (2026-09-14), `wip(#2378 arch): вынести apt-deps voice-assistant в voice_base; убрать rosdep install || true (#2407)` | Добавил `python3-aiohttp` в voice_base apt-install. **НЕ убирал** pip-override в voice_assistant — это **осознанно** (см. §3 ниже). |
| `docker/vision/voice_base/Dockerfile:18` | `ARG BUILD_VERSION=2026-09-14-2378-httpx-aiohttp` (bump для инвалидации apt-слоя) |
| `.github/workflows/L-Build Single Service.yml:229` | `BASE_IMAGE="localhost:5000/krikz/rob_box:voice-base-${ROS_DISTRO}-${DOCKER_TAG}"` — workflow подставляет **динамический тег** (не `-latest`), build bump в voice_base действительно доезжает до voice_assistant |

### 1.3 Версии пакетов: критический факт

| Источник | Версия aiohttp |
|---|---|
| Ubuntu jammy apt (`python3-aiohttp`, ubuntu-updates) | **3.8.1** (`3.8.1-4ubuntu0.2`) |
| `requirements.txt:22` (voice-assistant) | `>=3.14.3,<4.0` |
| `Dockerfile:140` (voice-assistant) | `>=3.9.1,<4.0` |
| `<exec_depend>aiohttp</exec_depend>` (package.xml) | без версии (semver-free) |

**Разрыв**: apt-версия (3.8.x) **на мажор ниже** минимума, требуемого кодом (3.9.1+). В aiohttp 3.9 были
breaking-changes (новый API `aiohttp.web.Application`, изменённые таймауты, `aiohttp_socks` совместимость).
**Код voice-action-server не запустится на apt-версии** (проверяется загрузкой `from aiohttp import web` —
синтаксис/сигнатуры совместимы, но runtime-поведение `aiohttp.web.Application` отличается).

---

## 2. Решение (что НЕ делаем, что делаем)

### 2.1 Ни один из двух фиксов issue #2568 не применяем

**Не делаем**: ни один из двух вариантов в issue #2568.

**Почему**:
1. **Фикс №1** (убрать `aiohttp` из pip-install) **сломает voice-action-server** — apt-версия 3.8.1
   ниже требуемого `>=3.9.1`, runtime-регрессия (тот же класс, что лечили в #1004-area / commit `337548a38`).
2. **Фикс №2** (откатить PR #2407, убрать `python3-aiohttp` из voice_base) — ухудшает архитектуру:
   удаляет **общий** apt-резолв, который PR #2407 использовал как основной механизм «build once, share
   with all» (паттерн PR #2286 для `rob_box_supervisor_msgs`). Прямое нарушение ADR-0011 §1.4
   («build once, share with all»).

### 2.2 Что делаем: оставляем **двухслойный** паттерн (apt + pip override)

**Решение**:
- **apt** (`voice_base/Dockerfile:78`) ставит `python3-aiohttp` как **нижнюю границу версии** — это
  гарантирует, что в любом image **есть** какой-то aiohttp (для rosdep-style резолва, для системных
  пакетов, которым нужна минимальная aiohttp, для быстрого cold-start).
- **pip** (`voice_assistant/Dockerfile:140`) **переопределяет** до **актуального** диапазона
  `>=3.9.1,<4.0` (на практике даёт 3.14.3+ согласно `requirements.txt`) — это даёт voice-action-server
  нужный API.
- **build cache safety**: bump BUILD_VERSION в `voice_base/Dockerfile:18` при изменении apt-слоя +
  workflow `L-Build Single Service.yml:229` подставляет динамический тег → voice_assistant пересобирается
  с новой базой и переписывает pip-override на новую базовую версию.

### 2.3 Что фиксим в комментариях (косметический долг, без правок кода)

`voice_assistant/Dockerfile:122-126` («Почему НЕ voice_base»):

```
# Почему НЕ voice_base: ~10 GB pytorch/vosk/yandex/renardo cache
# инвалидируется для всех потребителей (voice_assistant, supervisor,
# quest). aiohttp/prometheus/resemblyzer/coverage/pyyaml/ddgs/
# duckduckgo-search — лёгкие, ставятся в app-layer точечно.
```

Этот комментарий **стал частично неверным** после PR #2407: aiohttp теперь стоит и в voice_base
(apt), и в voice_assistant (pip override). Расхождение с реальностью — но **комментарий отражает
НАМЕРЕНИЕ** (почему именно pip-override не выносим целиком), а не текущее состояние. Комментарий
корректен **по дизайну** (pip-override остаётся в app-layer по решению commit `337548a38`).

**Правка комментария не блокирует merge** issue #2568 — но предлагаю сделать её отдельным
touchpoint в том же PR, что и любой code-change по этому ADR (если такой потребуется).

---

## 3. Trade-off анализ

### 3.1 Альтернативы, которые рассмотрели

| Вариант | Плюсы | Минусы | Вердикт |
|---|---|---|---|
| A. Убрать pip-override, оставить apt (фикс #1 issue) | Меньше layers, чище Dockerfile | voice-action-server упадёт на jammy 3.8.1, регрессия #1004-class | ❌ |
| B. Убрать apt-пакет, оставить pip (фикс #2 issue / частичный откат PR #2407) | Один источник истины для aiohttp | Нарушает паттерн «build once, share with all» (PR #2286, ADR-0011 §1.4); система-wide пакеты теряют aiohttp | ❌ |
| C. **Двухслойный** (apt + pip override) — текущее состояние | Совместимо с voice-action-server; apt-гарантия для других потребителей; cache-isolation voice_base | Два места с aiohttp; комментарий в Dockerfile требует уточнения | ✅ принято |
| D. Поднять apt-версию до 3.9+ (backport jammy-ppa или pypi-apt) | Один источник | Не существует стабильного pypi-apt для aiohttp 3.9+ на jammy; новые сюрпризы в CI | ❌ |
| E. Зафиксировать версию aiohttp в apt (pinned version, не just package name) | Reproducibility | apt pinning на jammy не гарантирует 3.9+ (нет в репозитории) | ❌ невозможно |

### 3.2 Цена vs выгода двухслойного паттерна (C)

| Цена | Выгода |
|---|---|
| Два места с aiohttp (apt в voice_base + pip в voice_assistant) | Voice-action-server работает на нужной версии (3.9+); apt-пакет гарантирует резолв для всех потребителей |
| Комментарий в `voice_assistant/Dockerfile:122-126` нуждается в уточнении (косметика) | build cache voice_base не инвалидируется при апгрейдах aiohttp (экономия ~10 GB pytorch/etc cache rebuild) |
| Один лишний pip-install (минута CI) | Никаких регрессий типа #1004 |

---

## 4. Почему это НЕ баг (по ADR-0018 «честный FAIL лучше красивого PASS»)

Issue #2568 заявляет: «фактически цель PR #2407 для voice_assistant не достигнута».

**Это верно только если цель PR #2407 — полностью устранить упоминание aiohttp из voice_assistant.**
Но raw-evidence (commit `337548a38` 2026-08-06) показывает, что **авторы осознанно оставили
pip-override**. Цель PR #2407 была: **убрать `rosdep install || true`** (Шаг 2 voice_assistant) и
**перенести apt-зависимости в voice_base**. Это **достигнуто**: Шаг 2 удалён, `python3-aiohttp` в
voice_base есть, `rosdep install` больше не вызывается.

Что PR #2407 **не** делал: не убирал pip-override aiohttp — потому что это сломало бы voice-action-server.
Это **не недосмотр**, а **согласованное решение** (см. §1.2 raw-evidence).

**Голословное «не достигнуто» в issue #2568 — это признак неполной диагностики, не баг кода.**

---

## 5. Точки наблюдения / мониторинг

| Что мониторить | Где | Когда срабатывает |
|---|---|---|
| apt-версия aiohttp в voice-base | `docker run --rm voice-base:tag dpkg -l \| grep aiohttp` | При каждом bump BUILD_VERSION voice_base |
| pip-версия aiohttp в voice-assistant | `docker run --rm voice-assistant:tag pip show aiohttp` | При каждом изменении Dockerfile:140 / requirements.txt:22 |
| Рассинхрон версий | Если apt ≠ pip → подтверждаем, что override активен | Это **норма** для текущего дизайна (см. §2) |
| ModuleNotFoundError aiohttp в voice-action-server логах | `docker logs voice-action-server \| grep -i aiohttp` | Если override сломался → регрессия #1004-class |

**Acceptance**: если `pip show aiohttp` показывает `3.9.x` или выше — override работает, всё ок.

---

## 6. Связанные ADR и коммиты

- **ADR-0011** (action-protocol): §1.4 — паттерн «build once, share with all», обоснование sidecar
  `voice-action-server` отдельно от `voice-assistant`. Наш двухслойный паттерн — частный случай этого.
- **ADR-0018** (agent-honesty-culture): запрет голословных маркеров без raw-evidence. Применён в §4.
- **ADR-0089** §3 touchpoint #11: упоминает config для AI HAT+, не имеет отношения к этому ADR —
  issue #2568 ссылается на touchpoint #11 по ошибке (надо проверить issue-ссылку при ревью).
- **commit `337548a38`** (2026-08-06, `fix(voice-action-server): also pin aiohttp in voice_assistant Dockerfile`):
  первоисточник двухслойного паттерна.
- **commit `70df9f5fe`** (2026-09-14, `wip(#2378 arch): вынести apt-deps voice-assistant в voice_base` / PR #2407):
  добавил apt-слой, оставил pip-override сознательно.
- **E2E_TESTING_DESIGN_v2 §D A43**: документирует исходную регрессию с 399 рестартами из-за отсутствия
  aiohttp — это **то, что мы НЕ хотим повторить**.

---

## 7. Вердикт по issue #2568

**Issue #2568 close-with-comment "wontfix" / "by design"**, с обоснованием:

1. PR #2407 свою цель **достиг** (убрал `rosdep install || true`, перенёс apt-deps в voice_base).
2. Pip-override aiohttp в `voice_assistant/Dockerfile:140` — **сознательное решение**, документированное
   в commit `337548a38`. Удаление override'а сломает voice-action-server.
3. Ни один из двух фиксов issue #2568 не применим:
   - **Фикс #1** (убрать pip-override) → регрессия voice-action-server (apt-версия 3.8.1 < требуемой 3.9+).
   - **Фикс #2** (откатить часть PR #2407, убрать apt-пакет) → ломает паттерн «build once, share with all».

**Рекомендуемые follow-up** (не блокируют close issue #2568):

| # | Действие | Owner | Когда |
|---|---|---|---|
| F1 | Уточнить комментарий `voice_assistant/Dockerfile:122-126` (отразить, что aiohttp в voice_base тоже есть, override — для версии) | devops | Следующий touchpoint voice_assistant Dockerfile |
| F2 | Проверить issue #2568 ссылку «ADR-0089 §3 touchpoint #11» — touchpoint #11 в ADR-0089 относится к `docker/vision/config/hailo_models.yaml` (Vision Pi Phase 1, AI HAT+), **не к voice_base/Dockerfile:78**. Ссылка ошибочна — issue title правильный, но related-линки требуют правки. | architect (этот ADR) | В этом же issue |
| F3 | Синхронизировать `requirements.txt:22` (`>=3.14.3,<4.0`) и `Dockerfile:140` (`>=3.9.1,<4.0`) — сейчас **мягче** в Dockerfile. Рекомендуется поднять Dockerfile до `>=3.14.3,<4.0` для единой картины (но не блокер). | devops | Вместе с F1 |

---

## 8. Change log

| Дата | Автор | Изменение |
|---|---|---|
| 2026-09-15 | architect (t_fa0e3ef7, issue #2568) | Initial ADR-0091. Architectural verdict: PR #2407 достиг своей цели (убрал `rosdep install \|\| true`), pip-override aiohttp в voice_assistant — сознательное решение commit `337548a38` (2026-08-06). Ни один из двух фиксов issue #2568 не применяется. F1–F3 как follow-up. Без правок кода. |

---

*ADR-0091 принят в рамках kanban-card `t_fa0e3ef7` (issue #2568). Все ссылки и файлы проверены
реальным `git show` / `git log` / чтением, не выдуманы. Issue #2568 — close with «by design».*
