# ADR-0057: ADR-namespace collision guard как hard gate в G-Lint Code

| Поле | Значение |
|---|---|
| Статус | Proposed |
| Дата | 2026-09-07 |
| Автор | devops (Hermes Agent), issue #2072 / ретро audit operator-agent-хендоффа |
| Контекст | `docs/adr/` на `origin/develop` — 13 номеров заняты более чем одним файлом (0052×5, 0055×3, 0027×3, 0028×2, 0030×2 и т.д.). Guard `validate_adr_namespace.sh` написан и покрыт 10/10 регресс-тестами, но в CI **не подключён** — запускался только вручную и post-factum в `agent-flow-merge-gate.sh` (откуда его можно обойти rebase'ом или service-action merge'ом, как показывают свежие коллизии `0052-mcp-slice-guard-on-transport.md` (PR #2064, a35c5088) и `0054-operator-agent-step-7b-eventbus-bridge.md` (PR #2048, 6fe87180)) |
| Затрагивает | `.github/workflows/G-Lint Code.yml` (новый шаг в `python-lint`), `scripts/agent_flow/validate_adr_namespace.sh` (без изменений — только регистрация в CI), `docs/adr/0030` (ссылка как на родительское решение) |
| Родители | ADR-AF-0030 §2.4 (pre-merge guard), ADR-0018 (честный FAIL), ADR-0013 (incremental delivery), ADR-0021 (hard gate pattern через `cc_budget.py`) |
| Связанные | issue #2072 (эта ретро), #1984 (прецедент «скрипт написан, но не в CI»), #2069 (соседний сбой merge-gate), PR `#2064`, `#2048`, `#2049`, `#2042` (свежие bypass'ы merge-gate) |

## 1. Контекст

### 1.1 Что наблюдаем

На `origin/develop` (2026-09-07) — **13 уникальных номеров**, каждый из которых занят 2-5 файлами:

| номер | файлов | slug'и |
|---|---|---|
| **0052** | **5** | `decomposed-children-wake-up-watchdog`, `fan-out-dedup-by-file-overlap`, `issue-auto-close-after-merge`, `mcp-slice-guard-on-transport`, `wake-words-ssot-code-with-yaml-operator-override` |
| **0055** | 3 | `operator-tts-headset-channel`, `operator-tts-headset-channel-impl-plan`, `voice-memory-db-unify-with-harness` |
| 0027 | 3 | (исторические — ретро 25.08) |
| 0028, 0030, 0032, 0054 | 2 | (включая `0030-adr-numbering-sot.md` соседствует с `0030-e2e-stale-branch-guard.md` — **ADR-AF-0030 нарушен сам собой**) |
| 0009, 0013, 0016, 0021, 0024, 0026 | 2-3 | — |

Ссылка «ADR-0052» перестала означать что-либо: 4 из 5 файлов под этим номером — про agent-flow (watchdog, dedup, auto-close, wake-words) и **ни один** из них не про срезы (`slice`), кроме пятого (`mcp-slice-guard-on-transport.md`). Живой пример — `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py:270`:

```
# Issue #1998 §6.2 — sender → slice → tool allowlist (ADR-0052).
```

Под 0052 лежат пять документов, и читатель кода обязан угадывать.

### 1.2 Почему guard не сработал (хотя есть)

1. **`scripts/agent_flow/validate_adr_namespace.sh`** — **локальный pre-PR скрипт** (10/10 регресс-тестов, merge commit `08386a896`). Воркер обязан вызвать его сам, ничего не заставляет. Проверил локально на текущей ветке — `validate_adr_namespace: clean (нет новых ADR-файлов в origin/develop...HEAD)`.
2. **`check_adr_number_collision()` в `agent-flow-merge-gate.sh:2599`** — вызывается **ровно один раз**, из ветки на строке 4613, **внутри** пути присвоения `needs-e2e`. PR, попавший в develop иным маршрутом (rebase, service action, прямой merge через `G-Auto-merge Feature to Develop`), guard **не проходит вовсе**.

Четыре свежие коллизии зашли именно так:

| файл | принёсший коммит | маршрут |
|---|---|---|
| `0052-mcp-slice-guard-on-transport.md` | `a35c5088` (PR #2064) | «devops service action #2061» |
| `0054-operator-agent-step-7b-eventbus-bridge.md` | `6fe87180` (PR #2048) | «clean rebase» |
| `0055-voice-memory-db-unify-with-harness.md` | `a81e7b36` (PR #2049) | обычный merge |
| `0055-operator-tts-headset-channel-impl-plan.md` | `c0dace5a` (PR #2042) | обычный merge |

3. **В `.github/` ссылок на `validate_adr_namespace` нет.** То есть в CI проверки **нет вообще** — единственный документированный путь guard'а работает только если воркер прочитал `AGENTS.md` или `scripts/agent_flow/README.md` и сам запустил.

### 1.3 Гипотеза (root cause)

Параллельные ветки воркеров выбирают ADR-номера, не сверяясь с `origin/develop`. ADR-AF-0030 §2.2 требует `git fetch origin develop` + ручной проверки через `git ls-tree`, но это **неформализованная** процедура, которая ломается на:
- агентах, которые коммитят «из HEAD» (не знают, что в develop за это время появились соседи),
- маршрутах merge'а, минующих `merge-gate.sh`,
- срочных правках (hotfix ветки), где экономия минуты важнее review-процесса.

Локальный скрипт `validate_adr_namespace.sh` решает только первый случай и не подключён к обязательному pipeline.

## 2. Принятое решение

### 2.1 Подключить `validate_adr_namespace.sh` в `G-Lint Code` как hard gate

В `.github/workflows/G-Lint Code.yml`, в job `python-lint` (рядом с уже существующим `cc_budget.py` — **тот же паттерн**):

```yaml
# ADR-AF-0030 §2.4 / issue #2072: ADR-namespace collision guard.
# Hard gate (no continue-on-error) — PR с новым docs/adr/NNNN-*.md
# на занятом номере должен фейлить lint ещё до merge-gate.
- name: ADR-namespace collision guard (ADR-AF-0030)
  run: |
    git fetch --no-tags origin develop
    bash scripts/agent_flow/validate_adr_namespace.sh --ref origin/develop
```

**Почему `python-lint`, а не отдельный job**: cc_budget уже там как hard gate (ADR-0021 R1, issue #1984 — прецедент «скрипт написан, но не в CI»), `lint-summary` агрегирует `python-lint.result` в общий отчёт.

**Почему `git fetch --no-tags` перед запуском**: `actions/checkout@v7` без `fetch-depth: 0` отдаёт shallow clone; `validate_adr_namespace.sh` ходит в `origin/develop` через `git ls-tree -r`, для чего ref должен существовать локально.

**Hard gate** (без `continue-on-error`) — иначе это та же ловушка, что сейчас: «скрипт написан, но warn-only».

### 2.2 Что в существующем скрипте уже покрыто (regression test 10/10)

Проверил локально перед коммитом:

```
PASS [A: negative-collision] (rc=1, next-free-detected)
PASS [B: positive-fresh] (rc=0, clean)
PASS [C: modify-existing-not-collision] (rc=0)
PASS [D: no-new-adrs] (rc=0)
PASS [E: missing-ref] (rc=2)
PASS [F: empty-baseline] (rc=0)
PASS [G: pipefail-no-silent-exit] (rc=0)
PASS [H: --help] (rc=0)
PASS [I: multi-collision] (rc=1)
PASS [J: --strict-on-clean] (rc=0)
=== validate_adr_namespace test summary ===
PASS: 10
FAIL: 0
ALL OK
```

Дополнительный dry-run в worktree карточки #2072:

```
$ bash scripts/agent_flow/validate_adr_namespace.sh --ref origin/develop
validate_adr_namespace: clean (нет новых ADR-файлов в origin/develop...HEAD).
rc=0
```

И negative dry-run (фиктивный `0052-dry-run-test-collide.md`):

```
  52 (занято в origin/develop: decomposed-children-wake-up-watchdog,fan-out-dedup-by-file-overlap,
                                issue-auto-close-after-merge,mcp-slice-guard-on-transport,
                                wake-words-ssot-code-with-yaml-operator-override;
     новый в PR: dry-run-test-collide)

ERROR: ADR namespace collision detected.
  Новый ADR-файл(ы) в PR используют номер(а), уже занятые в origin/develop:
  52

  Next free slot: 0057
rc=1
```

— всё корректно: exit 1, список slug'ов baseline, slug из нового PR, `Next free slot: 0057` (по `ls-tree | tail -1 + 1`).

### 2.3 Что **не** делаем в этом PR

- **Не разрешаем 13 существующих коллизий.** Это требует решения владельца по схеме: сквозная перенумерация или доменные неймспейсы (`ADR-AF-0052` для agent-flow против `ADR-0052` для рантайма). ADR-AF-0030 §«Затрагивает» явно оставил этот выбор за Шифу — до сих пор не сделан. Это отдельная карточка.
- **Не правим существующие ADR-файлы** — только регистрируем CI-шаг.
- **Не трогаем `agent-flow-merge-gate.sh`** — там уже есть `check_adr_number_collision()`, и после этого ADR merge-gate становится вторым рубежом (хорошо — defence-in-depth), а не единственным.
- **Не вводим ADR-bot / external counter** — overkill (см. ADR-AF-0030 §3).

## 3. Альтернативы, которые мы отвергли

| Альтернатива | Почему отвергли |
|---|---|
| Только в merge-gate, не в CI | Уже там есть (`check_adr_number_collision`) — но 4 свежие коллизии зашли мимо merge-gate. CI обязан быть первым рубежом. |
| External ADR-counter bot / PR-bot | Overkill для проекта. Guard в `G-Lint Code` дешевле (1 шаг в существующем workflow). |
| Переписать все существующие ADR сразу (через этот PR) | Нарушает ADR-0013 (incremental delivery, ≤300 строк) и требует решения Шифу по §2.6 ADR-AF-0030. Отдельная карточка. |
| Soft-warning в PR-comment (continue-on-error) | Ровно то, что есть сейчас — неэффективно. Воркеры читают `gh pr checks`, но не `gh pr view --comments` (если CI не красный). |
| Отдельный job `adr-namespace-collision` | Дублирует `python-lint.result` агрегацию. cc_budget уже там — последовательность. |

## 4. Trade-offs

| Что получаем | Чем платим |
|---|---|
| Guard в CI: PR с коллизией падает на lint ДО merge-gate | +1 шаг в `python-lint` (≈5 сек) + `git fetch origin develop` на runner (~2-3 сек на cold cache) |
| Обход через rebase/service-action больше не работает (CI ловит раньше) | Ломает существующие скрипты, которые merge'ат мимо merge-gate — придётся чинить; но это и есть цель |
| Defense-in-depth: CI (G-Lint) + merge-gate (`check_adr_number_collision`) | Дублирование проверки — дёшево (скрипт запускается дважды, но <1 сек total) |
| Защита от регрессии: новый ADR-номер, не проверенный через `git fetch`, сразу красный PR | Воркеры должны помнить `git fetch` перед созданием ADR (медленнее на ~1-2 с) |

## 5. План внедрения

| # | Действие | Кто | Acceptance |
|---|---|---|---|
| 1 | `.github/workflows/G-Lint Code.yml` — добавлен шаг `ADR-namespace collision guard (ADR-AF-0030)` в `python-lint`, hard gate | devops (эта карточка) | PR открыт, base=develop, CI зелёный |
| 2 | Локальная проверка: dry-run с фиктивным 0052-* → exit 1; с 0057-* → exit 0 | devops | raw-вывод в комментарии PR |
| 3 | Этот ADR смержен в `develop` | architect (после Шифу merge) | PR смержен, статус → Accepted |
| 4 | Cleanup 13 существующих коллизий по решению Шифу | devops (после Шифу) | отдельная карточка, **не** эта |
| 5 | ADR-AF-0030 перевести из `Proposed` в `Accepted` после успешного rollout guard'а | (авто при merge ADR-0057) | статус в frontmatter обновлён |

## 6. Что **не** делаем

- Не авто-merge этого ADR — Шифу мержит сам (правило §2.5 ADR-AF-0030: ручной коммит запрещён, manual merge — за Шифу).
- Не разрешаем существующие 13 коллизий в этом PR — отдельная карточка, требует §2.6 ADR-AF-0030.
- Не вводим ADR-bot — overkill.

## 8. Ссылки

- ADR-AF-0030 §2.4 (pre-merge guard), §2.5 (запрет ручного коммита), §2.6 (cleanup коллизий)
- ADR-0021 R1 (hard gate через cc_budget — паттерн для этой карточки)
- ADR-0018 (честный FAIL лучше красивого PASS)
- ADR-0013 (incremental delivery — почему cleanup отдельной карточкой)
- issue #2072 (эта ретро)
- issue #1984 (прецедент «скрипт написан, но не в CI → не работает»)
- issue #2069 (соседний сбой merge-gate)
- `scripts/agent_flow/validate_adr_namespace.sh` — реализация guard'а
- `scripts/agent_flow/tests/test_validate_adr_namespace.sh` — 10/10 регресс-тестов
- PR #2064 (service-action обход), #2048 (clean-rebase обход), #2049, #2042 (обычные merge'и) — примеры bypass'ов, которые этот guard закрывает