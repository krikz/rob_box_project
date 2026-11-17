# AF-0069 — merge-gate ADR-collision INFLIGHT-check (защита от ADR-number race)

**Дата:** 2026-09-15
**Автор:** devops worker (ретро-карточка t_3f086e23)
**Контекст:** `scripts/agent_flow/agent-flow-merge-gate.sh`, функция `check_adr_number_collision` (L3260-L3450)
**Статус:** accepted (в этом PR, ожидает ревью Шифу)
**Связанные тикеты:** #2582 (issue), PR #2582 (branch `z-{agent}/2582-...`)
**Связанные ADR:** ADR-0057 (adr-namespace-collision-guard-in-ci — *тот, что не сработал*),
                    ADR-AF-0030 (ADR numbering SOT), ADR-AF-0065 (merge-gate scan-all-prs race-guard),
                    ADR-0018 (agent-honesty-culture)
**Связанные ретро:** t_3f086e23 (эта карточка), t_a8e82f2d (предшественник по race-class),
                      t_42741511 (13.08 race-guard общий)

## Контекст

2026-09-15 в `origin/develop` одновременно оказались **три разных ADR с
номером 0101**, влитых параллельно за несколько часов:

| Файл                                           | Заголовок                                                                                              | PR    | Merge-time   |
| ---------------------------------------------- | ------------------------------------------------------------------------------------------------------ | ----- | ------------ |
| `0101-robot-id-compose-default-pattern.md`     | «ROBOT_ID inline-default в docker-compose» (architect verdict #2569)                                   | #2572 | 13:31:41Z    |
| `0101-occasion-unified-turn-entry.md`          | «Повод (Occasion): единый шов "можно ли заговорить"» (ADR-0101 «Повод»)                                | #2575 | 13:37:41Z    |
| `0101-perception-gaze-seam.md`                 | «Взгляд: единый шов источника кадра для vision_hailo_node» (ADR-0101 «Взгляд»)                         | #2578 | 13:52:44Z    |

Все три ссылаются на себя как «ADR-0101», и **заголовки влитых коммитов
уже увековечили эту неразрешимость**:

- `3913e5f7 feat(perception): Взгляд — единый шов источника кадра (issue #2531, ADR-0101) (#2578)`
- `e7c9abc6 adr(#2536): ADR-0101 «Повод» — единый шов «можно ли заговорить» (#2575)`
- `9485018f [architect] #2569 — ADR-0101 verdict: ROBOT_ID inline default pattern (#2572)`

То есть ссылка «ADR-0101» в истории коммитов **теперь неразрешима** —
она указывает на три разных решения. Глобальная ADR-коллизия — это
именно то, от чего ADR-0057 обещал защитить.

## Root cause

ADR-0057 (`adr-namespace-collision-guard-in-ci.md`) был заведён ровно
под этот класс дефектов. Guard реализован в **двух местах**:

1. **CI каждого PR** (`scripts/ci/validate_adr_namespace.sh` L120-L152,
   ADR-AF-0030 Phase 2) — линтит NNNN в новых ADR-файлах PR.
2. **Pre-merge на merge-gate** (`agent-flow-merge-gate.sh`, функция
   `check_adr_number_collision`, L3260+) — проверяет, что NNNN из
   новых файлов PR не конфликтует с файлами **в `origin/develop`**.

Здесь guard **не сработал**, и причина **структурная**:

Каждый PR по отдельности брал следующий свободный номер от своей базы
(`0100` был максимумом на момент форка всех трёх веток), проходил
проверку против **своего `origin/develop` на момент создания** — и
проходил чисто. Коллизия возникала **в момент merge**, когда сосед
уже влил свой `0101`. Pre-merge проверка **не видит того, что
произойдёт после merge соседа**.

Это **тот же класс гонки**, что ADR-AF-0065 описывает для
spawn-карточек merge-gate: «проверка на pre-merge состоянии не видит
того, что произойдёт после merge соседа». И race-condition #2459 (race
при параллельных PR на одну issue) — то же семейство.

## Решение

### §1. INFLIGHT-check в `check_adr_number_collision`

Добавить второй контур проверки поверх существующего develop-контура:
собрать все открытые PR (через `gh pr list --state open --limit 100
--json number,files`), отфильтровать **по номеру PR** (не по path) self,
и для каждого NNNN из `pr_new_adrs` проверять — не приносит ли
**другой открытый PR** такой же NNNN с **другим slug'ом**.

```bash
# Дополнение к develop-контуру (~L3290):
inflight_adrs="$(gh pr list --state open --limit 100 --json number,files \
    | python3 -c '
import json, re, sys
arr = json.load(sys.stdin)
adr_re = re.compile(r"^docs/adr/(0[0-9]{3})-.*\.md$")
self_pr = sys.argv[1]
seen = set(); out = []
for pr in arr:
    if str(pr.get("number", "")) == self_pr: continue
    for f in pr.get("files", []) or []:
        p = f.get("path") if isinstance(f, dict) else None
        if not adr_re.match(p or ""): continue
        leaf = p[len("docs/adr/"):]
        if leaf in seen: continue
        seen.add(leaf); out.append(leaf)
print("\n".join(out))
' -- "$pr_number")"
```

**Ключевые свойства:**

- Self-фильтр по **номеру PR**, не по path — потому что два PR могут
  легитимно трогать один и тот же NNNN-slug при rename (это отдельный
  сценарий, см. §3.5 ниже).
- **Без сложного `--jq`** — apply_jq в `mock_env.sh` не поддерживает
  `select|as`. Inline python проще, стабильнее, и переиспользует
  уже-используемый в guard'е паттерн парсинга.
- **Без gh / без auth** — fail-open (`return 0`) + явное логирование.
  Коллизия никуда не денется — её поймает следующий тик ИЛИ
  `validate_adr_namespace.sh` в CI самого PR (defence in depth: 2
  контура — CI-линтер per-PR + merge-gate INFLIGHT-check).
- **Лимит 100 PR** — на develop в rob_box_project сейчас ~10 открытых
  PR, лимит с большим запасом.

### §2. Формат collision-detail

Развить существующий формат `(develop clashes: slug1, slug2)` в
**двухсекционный** — для каждого NNNN, где есть коллизия, выводим
**обе** секции, если есть и develop, и inflight:

```
NNNN (develop clashes: dev-slug.md), NNNN (inflight clashes: inf-slug.md)
```

Тестируется в `test_L_inflight_and_develop_collision` (J..O).

### §3. Регресс-тесты (kейсы ADR-AF-0068 §3)

В `scripts/agent_flow/tests/test_merge_gate_adr_collision.sh` добавлены
тесты **J..O** (покрывают §1):

| Кейс | Сценарий                                                                          | Ожидание            |
| ---- | --------------------------------------------------------------------------------- | ------------------- |
| J    | PR-A приносит 0033-foo.md; inflight PR-B приносит 0033-other-adr.md                | **REJECT** (inflight) — главный сценарий #2582 |
| K    | PR-A приносит 0033-foo.md; inflight есть, но без ADR                              | PASS                |
| L    | PR-A приносит 0033-foo.md; develop содержит 0033-in-develop.md; inflight PR-B приносит 0033-other-adr.md | **REJECT** (оба: develop AND inflight) |
| M    | PR-A переименовывает 0033-foo.md → 0034-bar.md; inflight PR-B тоже трогает 0033-foo.md (PR-A и PR-B правят один файл) | PASS (self-overlap не коллизия) |
| N    | PR-A приносит 0033-foo.md; inflight содержит **сам** PR-A (номер тот же) | PASS (self PR отфильтрован) |
| O    | inflight пуст (нет открытых PR); develop содержит 0033-in-develop.md              | **REJECT** (develop-проверка работает как раньше) |

**Итого 15/15 тестов зелёные** (A..O, ADR-0057 regression + AF-0068
inflight).

### §4. ADR-0057 amendment — задним числом

Дополнить ADR-0057 §X «Почему pre-merge проверки недостаточны»:

> Race-condition: pre-merge guard проверяет NNNN против `origin/develop`
> **на момент тика**, но не видит параллельные PR в полёте. Реальный
> пример: issue #2582 — три PR (#2572/0101-robot-id, #2575/0101-occasion,
> #2578/0101-perception) прошли каждый свой guard чисто и легли в
> develop под одним номером 0101. Фикс — ADR-AF-0069 (INFLIGHT-check).

### §5. ADR-AF-0030 amendment — учёт параллельных PR

Дополнить §2.4 правилом: «При выборе NNNN для нового ADR проверять не
только `git ls-tree origin/develop`, но и открытые PR (через
`gh pr list --state open --json number,files`); если NNNN уже занят
**другим** PR'ом с другим slug'ом — это коллизия, даже если develop
чист.»

### §6. Объяснение, почему fail-open без gh — допустимо

Без gh auth / без gh в PATH guard fail-open (return 0) и не блокирует
PR. Это **намеренно**, потому что:

1. В CI у merge-gate gh **всегда** есть и авторизован (его не пустят).
2. В крайнем случае (нет gh) **CI-линтер per-PR**
   `validate_adr_namespace.sh` всё равно поймает коллизию в собственном
   PR — потому что PR проверяется **против `origin/develop` на момент
   коммита в PR**, не на момент merge соседа. Но это **race** —
   коллизия может проскочить, если оба PR одновременно в полёте.
3. Двухконтурная защита (CI-линтер + merge-gate INFLIGHT) — defence in
   depth. Потеря одного контура не означает полную потерю защиты.

ADR-0018 «Честный FAIL лучше красивого PASS»: **fail-open не
скрывает проблему** — guard явно логирует
«INFLIGHT-check: gh недоступен/не авторизован — fail-open». Если
такое логирование появляется в merge-gate журнале — это сигнал для
оператора.

## Альтернативы (рассмотрены и отклонены)

1. **«Блокировать через gh API: получить все PR с docs/adr/* файлами и
   пройтись по их base_ref'ам».** Сложнее, медленнее (N API-вызовов),
   reuses ту же информацию (open PRs), ничего принципиально не даёт.
2. **«Сделать GitHub webhook → отдельный process, который обновляет
   «ADR numbers in flight» в KV-сторе».** Overengineering — ADR-процесс
   не настолько частый, чтобы ради него поднимать инфраструктуру.
3. **«Запретить параллельные PR с новыми ADR через triage-процесс».**
   Это политическое решение, а не технический guard — оно не масштабируется,
   и три #2572/#2575/#2578 были открыты **до того**, как triage понял,
   что они все берут 0101. Технический guard — единственное надёжное решение.

## Acceptance criteria

- [x] Кейс J: PR-A и inflight-PR-B с одинаковым NNNN, разными slug'ами → REJECT
- [x] Кейс K: inflight чист → PASS
- [x] Кейс L: develop + inflight одновременно → REJECT (обе секции в логе)
- [x] Кейс M: self-overlap (PR-A и inflight-PR-B трогают один файл) → PASS
- [x] Кейс N: self PR в inflight → отфильтрован
- [x] Кейс O: inflight пуст, develop collision → REJECT
- [x] 15/15 тестов зелёные
- [x] ADR-0057 §X дополнен «почему pre-merge недостаточно»
- [x] ADR-AF-0030 §2.4 дополнен «учитывай параллельные PR»
- [x] Issue #2582 обновлён (ADR-0103/0104 фиксация + новая защита)

## Roll-out

Этот PR уже включает все правки (merge-gate INFLIGHT-check +
регресс-тесты + ADR-0057/AF-0030 amendment). После merge в develop
guard заработает на следующем тике merge-gate (5 минут).

**Defence in depth:** для уже влитых трёх 0101 нужен отдельный
re-numbering (в том же PR — renumber 0101-occasion → 0103,
0101-perception → 0104, 0101-robot-id остаётся 0101). См. issue #2582.