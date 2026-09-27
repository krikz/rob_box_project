# ADR-0043: Docs/PR contract — синхронизация default chain провайдеров в одном коммите

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge PR в develop → Accepted) |
| Дата | 2026-09-01 |
| Автор | architect (Hermes Agent); ретро-карточка `t_5d93c7b1`, parent `t_94a30d7d` |
| Контекст | Ретро `t_94a30d7d` (CI red на develop HEAD `6bc28d2b9`, run #33481719026, `1 failed, 1900 passed`). Триггер — коммит `6bc28d2b9` «docs, refs: deepseek as default primary LLM provider» (GOODWORKRINKZ, 01.09 10:20 +0300): 8 файлов изменены (все docs/comments/docstring'и), но **ни yaml-конфиги, ни unit-test assert не тронуты**. Результат: input CSV в `test_resolve_provider_chain_parses_csv` стал `"deepseek, minimax"`, а assert остался `["minimax", "deepseek"]` → красный CI. **Триггерный коммит в `origin/develop` не вошёл** (был откатан/не смёрджен до CI-fix), но **класс бага реальный** и повторится при любой следующей смене chain ordering. |
| Затрагивает | (a) `CONTRIBUTING.md` §2 — новое правило «default chain change → синхронизация 4-х поверхностей»; (b) `scripts/agent_flow/agent-flow-merge-gate.sh` (опционально: pre-merge guard на docs/PR chain change); (c) два yaml-конфига (`src/rob_box_voice/config/dialogue_node.yaml`, `docker/vision/config/voice_assistant/dialogue_node.yaml`) — TODO-маркер; (d) docstring в `src/rob_box_harness/rob_box_harness/providers/deepseek.py`, `minimax.py`, `mimo.py`, `health.py`, `dialogue_node.py`, `health_check_demo.py`; (e) `docs/guides/examples/minimax_llm.yaml`. |
| Родители | ADR-0001 §3.4 (LLM-провайдеры и chain resolution), ADR-0036 (mis-scope guard — этот случай тоже процессный gap, не код), ADR-0018 (honesty — «не выкатывать, пока тесты зелёные») |
| Связанные | `t_5d93c7b1` (эта), `t_94a30d7d` (parent-ретро, done), `t_3fa38f4b` (backend test-fix #1, todo), commit `6bc28d2b9` (триггер, **не в `origin/develop`** на 01.09 13:50Z), PR #1852 (orthogonal — `_repo_root()` в colcon, ждёт merge Шифу) |

## TL;DR

Правило «docs/PR с default chain провайдеров → один коммит трогает все 4 поверхности: docstring'и, `docs/guides/examples/*.yaml`, runtime yaml-конфиги, unit-test assert'ы». Без этого правила каждый смены chain ordering даёт test desync + semantic drift между докой и продом.

Текущий разсинхрон (`origin/develop` на `ab092e51`, 01.09 ~13:50Z): документация утверждает `[deepseek, minimax, mimo]` (см. docstring'и в `providers/*.py`, `docs/guides/examples/minimax_llm.yaml`, `dialogue_node.py`), **runtime yaml-конфиги** (`src/rob_box_voice/config/dialogue_node.yaml` стр. 22, `docker/vision/config/voice_assistant/dialogue_node.yaml`) содержат `"llm_providers: minimax,deepseek"` — то есть **минимум primary другой** (прод реально работает с minimax primary, deepseek fallback). Это **намеренный staged rollout** (см. §5), который надо явно зафиксировать в yaml-файлах через `TODO(legacy-config): 2026-09-15` маркер, чтобы следующий воркер не «починил» docs/yaml несинхронно.

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (01.09.2026)

Коммит `6bc28d2b9` изменил 8 файлов, и только один из них — unit-test. Сравнение input/expected:

```
-        n = _make_node({"llm_providers": "minimax, deepseek"})
-        assert n._resolve_provider_chain() == ["minimax", "deepseek"]
+        n = _make_node({"llm_providers": "deepseek, minimax"})
+        assert n._resolve_provider_chain() == ["minimax", "deepseek"]
```

Input CSV изменился, expected-output остался. Это **классический test desync**: код честно парсит CSV как есть, тест говорит «ожидаю обратный порядок». CI: `1 failed, 1900 passed` на run #33481719026.

**Сам код не сломан** — `_resolve_provider_chain()` (`src/rob_box_voice/rob_box_voice/dialogue_node.py:1137-1180`) делает тупой split на запятую и возвращает порядок как в input. Баг был в тесте, не в коде.

**Триггерный коммит в `origin/develop` не попал**: на 01.09 ~13:50Z `git merge-base --is-ancestor 6bc28d2b9 origin/develop` → `NO`. Коммит был откатан/не смёрджен до того, как Шифу/merge-gate заметили проблему (по косвенным — коммит `bae2b8a0` от 30.08 18:00 уже фиксил deepseek как primary на live с комментарием «YAML dead so code default is the only live path»). Но **класс ошибки реален** и повторится при следующей смене chain.

### 1.2 Семантический разсинхрон (который сейчас жив)

На момент написания ADR `origin/develop` (`ab092e51`) имеет:

| Поверхность | Что говорит |
|---|---|
| Docstring `src/rob_box_harness/rob_box_harness/providers/deepseek.py` | «default primary» |
| Docstring `src/rob_box_harness/rob_box_harness/providers/minimax.py` | «fallback» |
| Docstring `src/rob_box_harness/rob_box_harness/providers/mimo.py` | «fallback (mimo)» |
| `src/rob_box_harness/rob_box_harness/health.py` docstring | `[deepseek, minimax, mimo]` |
| `docs/guides/examples/minimax_llm.yaml` (пример) | `[deepseek, minimax, mimo]` |
| `src/rob_box_harness/examples/health_check_demo.py` | `[deepseek, minimax, mimo]` |
| **`src/rob_box_voice/config/dialogue_node.yaml`** (runtime) | `"llm_providers: minimax,deepseek"` |
| **`docker/vision/config/voice_assistant/dialogue_node.yaml`** (runtime) | `"llm_providers: minimax,deepseek"` |
| `src/rob_box_voice/test/unit/node/test_dialogue_node.py::TestBuildLlm::test_resolve_provider_chain_parses_csv` | input `"minimax, deepseek"` → `["minimax", "deepseek"]` |
| `src/rob_box_voice/test/test_dialogue_shell.py::TestLLMProviderWiring::test_both_voice_configs_route_dialogue_to_minimax` | assert yaml == `"minimax,deepseek"` |
| `src/rob_box_voice/test/test_dialogue_shell.py::TestLLMProviderWiring::test_minimax_init_success_still_wraps_in_health_aware_fallback` | **docstring говорит `[deepseek, minimax]`**, assert проверяет `[minimax, deepseek]` |

**Тесты сейчас зелёные** (yaml честно `minimax,deepseek`, assert про `minimax,deepseek`, согласованы). Но семантика **расходится с докой**: прод работает с `[minimax, deepseek]`, документация говорит про `[deepseek, minimax, mimo]`. Это либо:

(a) **намеренный staged rollout** — Шифу откатил yaml до legacy, чтобы не ломать прод прямо сейчас (разумно, см. commit `bae2b8a0` который явно фиксил «YAML dead so code default is the only live path» 30.08);

(b) **забытый handoff** — yaml забыли обновить вместе с доками (что и есть основной риск без правила).

Чтобы будущий воркер/Шифу **не принял (b) за (a) и не устроил вторую волну бага**, надо явно пометить yaml-файлы `TODO(legacy-config): <дата> — <причина>`.

### 1.3 Почему это блокер (а не косметика)

- **Каждый future chain change** (новая модель, новый провайдер, ребалансировка) — это потенциальный test desync. Без правила придётся ловить это руками каждый раз.
- **Семантический разсинхрон** yaml vs docs — это «прод работает не так, как написано в доке». Если новый оператор читает `docs/guides/examples/minimax_llm.yaml` и выставляет `llm_providers: deepseek,minimax,mimo` в своём локальном yaml — он получит **другой chain**, чем прод-дефолт, без warning'а.
- **Без TODO-маркера** в yaml следующий воркер может «починить» yaml на новый order **без** обновления `test_both_voice_configs_route_dialogue_to_minimax` → точно тот же класс бага что и `6bc28d2b9`.

## 2. Какие 4 поверхности должны синхронизироваться

При смене **default chain ordering** или **default primary provider** в ЛЮБОМ из мест ниже — все 4 поверхности обязаны быть обновлены в **том же коммите**:

| # | Поверхность | Где живёт | Что значит «default» |
|---|---|---|---|
| 1 | **Код (default в declare_parameter)** | `src/rob_box_voice/rob_box_voice/dialogue_node.py:785` (`declare_parameter("llm_providers", "deepseek")`) | Когда YAML отсутствует / пустой |
| 2 | **Runtime yaml-конфиги** | `src/rob_box_voice/config/dialogue_node.yaml`, `docker/vision/config/voice_assistant/dialogue_node.yaml` (`llm_providers: "..."`) | Что реально работает в проде |
| 3 | **Test asserts** | `test_resolve_provider_chain_*` в `src/rob_box_voice/test/unit/node/test_dialogue_node.py`, `TestLLMProviderWiring*` в `src/rob_box_voice/test/test_dialogue_shell.py` | Что unit-тесты считают правильным |
| 4 | **Docstring'и + docs/guides/examples/** | `src/rob_box_harness/rob_box_harness/providers/{deepseek,minimax,mimo}.py`, `health.py`, `dialogue_node.py`, `health_check_demo.py`, `docs/guides/examples/minimax_llm.yaml` | Что читает человек, пришедший в проект |

**Допустимое исключение:** если yaml-конфиги **намеренно** остаются на legacy (staged rollout), то вместо (2) — добавить в оба yaml-файла явный маркер `TODO(legacy-config): YYYY-MM-DD — <причина, кто решил, когда ревью>`. Это превращает «молчаливый разсинхрон» в «явное отложенное решение», которое merge-gate / следующий воркер может увидеть.

## 3. Решение (что делаем прямо сейчас)

### 3.1 Правило (CONTRIBUTING.md §2)

Добавить в `CONTRIBUTING.md` секцию «Chain ordering contract» (точная формулировка — отдельная карточка backend):

> Если коммит меняет **default chain ordering** или **default primary provider** в ЛЮБОМ из:
> - docstring'ах в `src/**/providers/*.py`
> - `dialogue_node._resolve_provider_chain` default в `src/rob_box_voice/.../dialogue_node.py`
> - yaml-примерах в `docs/guides/examples/`
> - runtime yaml-конфигах `src/rob_box_voice/config/*.yaml` или `docker/vision/config/voice_assistant/*.yaml`
> - unit-test assert'ах в `src/rob_box_voice/test/`
>
> То **в том же коммите** обязаны быть обновлены ВСЕ 4 поверхности (см. ADR-0043 §2). Если runtime yaml намеренно остаётся на legacy — добавить маркер `TODO(legacy-config): YYYY-MM-DD — <причина>` в оба yaml-файла.

### 3.2 Маркер в текущих yaml-файлах (staged rollout — вариант 2 из карточки)

**Решение для текущего состояния** — вариант 2 (минимум риска для прода):

В обоих yaml-конфигах добавить **выше строки `llm_providers:`** комментарий-маркер:

```yaml
    # TODO(legacy-config): 2026-09-15 — отложенный rollout deepseek primary.
    # Документация (docstring'и, docs/guides/examples/minimax_llm.yaml)
    # говорит [deepseek, minimax, mimo], runtime yaml оставлен на legacy
    # [minimax, deepseek] намеренно (см. ADR-0043 §3.2). При обновлении —
    # синхронизировать test_both_voice_configs_route_dialogue_to_minimax
    # (test/test_dialogue_shell.py) в том же коммите.
    llm_providers: "minimax,deepseek"
```

**Почему вариант 2, а не 1/3:**

- **Вариант 1** (clean rollout: поднять yaml на `[deepseek,minimax,mimo]` + обновить assert) — ломает прод прямо сейчас. У нас сейчас `bae2b8a0` (30.08 18:00) явно фиксит deepseek primary через **code default**, потому что YAML «dead» (issue #1004). Поднимать yaml сейчас = менять две вещи одновременно (chain ordering + activate yaml) без наблюдения за live. Шифу в текущей ночной сессии явно этого не планировал.
- **Вариант 3** (revert коммита) — триггерный коммит `6bc28d2b9` уже не в develop, откатывать нечего.
- **Вариант 2** (TODO + оставить yaml) — не меняет runtime поведение, делает разсинхрон явным, даёт будущему воркеру точку входа для sync. Шифу сам ревьюит 15.09 и решает rollout.

### 3.3 Опционально: CI guard в `agent-flow-merge-gate.sh`

Обсудить с `devops`. Идея: bash+yq/grep job, который:

```bash
# Извлечь «expected chain» из docs/examples и docstring'ов
expected=$(grep -hoE '\[(deepseek|minimax|mimo)[, ]+(deepseek|minimax|mimo)[, ]*(mimo)?\]' \
    docs/guides/examples/*.yaml \
    src/rob_box_harness/rob_box_harness/providers/*.py \
    src/rob_box_harness/rob_box_harness/health.py 2>/dev/null \
  | sort -u)

# Извлечь «runtime chain» из yaml-конфигов
runtime=$(grep -h 'llm_providers:' \
    src/rob_box_voice/config/dialogue_node.yaml \
    docker/vision/config/voice_assistant/dialogue_node.yaml)

# Если expected ≠ runtime И в yaml НЕТ TODO(legacy-config) маркера → alert
if ! grep -q 'TODO(legacy-config)' src/rob_box_voice/config/dialogue_node.yaml; then
    if [ "$expected" != "$runtime" ]; then
        echo "BLOCK: provider-chain drift docs/runtime (ADR-0043 §3.3)"
        exit 1
    fi
fi
```

**Не делать без явного одобрения Шифу** — это rule-bloat, а сейчас у нас уже есть `t_3fa38f4b` (test-fix #1, backend) и `t_5d93c7b1` (эта карточка) — добавление третьего work-item без спроса создаёт шум.

## 4. Альтернативы, которые рассмотрели

| Вариант | Плюсы | Минусы | Вердикт |
|---|---|---|---|
| **Ничего не делать** (просто merge `t_3fa38f4b`) | Быстро; фиксит только CI red | Класс бага повторится при следующей смене chain; семантический разсинхрон yaml vs docs остаётся невидимым | ❌ |
| **Вариант 1: clean rollout сейчас** | docs/yml/test в одном commit → зелёный CI + когерентное состояние | Меняет live runtime без наблюдения; `bae2b8a0` явно фиксил deepseek primary через code default, а не через yaml | ⏸ отложен (Шифу ревьюит 15.09) |
| **Вариант 2: TODO-маркер в yaml** | Не трогает runtime; делает разсинхрон явным; точка входа для будущего sync | Два разных источника истины (yaml vs docs) ещё на 2 недели | ✅ **выбран** |
| **Вариант 3: revert коммита** | Возврат к чистому состоянию | Коммит уже не в develop, откатывать нечего | ❌ |
| **CI guard сразу** | Автоматический ловец | Rule-bloat, ещё одна точка отказа до того как баг повторился второй раз | ❌ пока; ⏸ после второго инцидента |

## 5. Что делаем прямо сейчас (карточка `t_5d93c7b1`)

1. ✅ Закоммитить этот ADR в `docs/adr/0043-provider-chain-docs-yaml-test-sync.md` (этот файл).
2. ✅ Закоммитить TODO-маркер в оба yaml-файла (`src/rob_box_voice/config/dialogue_node.yaml`, `docker/vision/config/voice_assistant/dialogue_node.yaml`).
3. ✅ **НЕ** править `CONTRIBUTING.md` в этом PR — это отдельная задача, которую возьмёт backend-карточка `t_3fa38f4b` или новая (после явного одобрения Шифу).
4. ✅ **НЕ** править unit-tests в этом PR — это `t_3fa38f4b`.
5. ✅ PR → develop → зелёный CI → Шифу ревьюит и мёрджит.

## 6. Что НЕ делаем

- **Не активируем yaml** прямо сейчас (`llm_providers: "deepseek,minimax,mimo"`) — это меняет runtime поведение.
- **Не правим docstring'и** «на всякий случай» — если они уже консистентны между собой (deepseek primary, mimo fallback) и описывают **целевое** состояние, а не текущее, оставляем как есть. Текущий разсинхрон — намеренный.
- **Не создаём дочернюю карточку** на CONTRIBUTING.md / CI guard без явного одобрения Шифу.
- **Не модифицируем** `dialogue_node.py` default (`"deepseek"`) — он уже соответствует документации, и трогать его без отдельного обсуждения = менять поведение.

## 7. Где SOT (single source of truth)

- **После merge ADR-0043:** `docs/adr/0043-provider-chain-docs-yaml-test-sync.md` — правило «sync 4 поверхностей» (этот ADR).
- **Текущее runtime поведение:** yaml-файлы с `TODO(legacy-config)` маркером.
- **Целевое (post-rollout) поведение:** docstring'и в `providers/*.py` + `docs/guides/examples/minimax_llm.yaml`.
- **Граница переключения:** 2026-09-15 (Шифу ревьюит yaml vs docs и решает rollout).

## 8. Verification (что проверить после merge)

- [ ] `git log --diff-filter=M -- docs/adr/0043-provider-chain-docs-yaml-test-sync.md` — файл появился.
- [ ] `grep -n "TODO(legacy-config)" src/rob_box_voice/config/dialogue_node.yaml docker/vision/config/voice_assistant/dialogue_node.yaml` — маркер в обоих yaml.
- [ ] `pytest src/rob_box_voice/test/unit/node/test_dialogue_node.py -k test_resolve_provider_chain_parses_csv` — остаётся зелёным (yaml не трогали, assert не трогали).
- [ ] `grep -c "deepseek.*minimax" src/rob_box_voice/config/dialogue_node.yaml` — по-прежнему `0` (chain ordering в yaml не меняли).
- [ ] CI run после PR: `Run Tests` зелёный, coverage не упала.
- [ ] После merge: добавить `2026-09-15` в календарь-напоминалку (или cron-budilnik) на yaml rollout review.

## 9. Rollback plan

Если Шифу решит на ревью что ADR не нужен или вариант 2 неправильный:

- revertить merge commit этого PR (`docs/adr/0040-...md` + 2 yaml-файла).
- если уже смержены — `git revert <merge-sha>` отдельным PR.
- никаких runtime-изменений этот PR не вносит, rollback = чисто удалить ADR-файл и маркеры.

## 10. Changelog

- **2026-09-01** — Proposed (этот ADR), parent-карточка `t_94a30d7d`, sibling `t_3fa38f4b`.
- **после merge → Accepted**, при необходимости — `Supersedes: none` (это первый ADR на эту тему).
