# Статус расследования #2003 (operator-agent 13, pregenerate) — 2026-09-07

> Спутник `pregenerate-in-tts-node-contract.md`. Фиксирует **реальное**
> состояние зависимостей и обнаруженные ветки, чтобы следующий PM
> не переоткрывал дизайн-спор заново.

## 0. Что говорит карточка

- Issue #2003 «[operator-agent 13] Спекулятивная генерация (pregenerate в
  tts_node)» — **заблокирован** зависимостью #1996 (7a, приоритетная
  очередь в tts_node).
- Предыдущий воркер (architect, попытка в этом же worktree) уже
  зафиксировал дизайн-контракт в
  [`pregenerate-in-tts-node-contract.md`](./pregenerate-in-tts-node-contract.md)
  (коммит `a1b823ce2`).

## 1. Что реально на `origin/develop` HEAD = `0cc0d242`

Проверено `git ls-tree origin/develop docs/plans/ | grep operator-agent-architecture-handoff`
и `grep -c "priority" src/rob_box_voice/rob_box_voice/tts_node.py` —
2026-09-07 12:58 UTC.

| Артефакт | Статус |
|---|---|
| `docs/plans/2026-09-05-operator-agent-architecture-handoff.md` | **ЕСТЬ** (blob `4db74d84…`, добавлен коммитом `d0b898e3`) |
| `docs/architecture/target-operator-agent-and-dialogue.md` | **ОТСУТСТВУЕТ** (ADR ссылается на него как SOT — ссылка битая) |
| `priority` в `tts_node.py` | **0 вхождений** — 7a на develop не влит |
| `pregenerate`/`pre_gen` в `tts_node.py` | **0 вхождений** в логике (есть только в комментарии `config/tts_node.yaml:8`) |
| `scheduler/pregen/` (5 модулей) | **ОТСУТСТВУЕТ** на develop |
| DoD-тесты `test_prefetch_quality_rejects_below_threshold` и т. п. | **ОТСУТСТВУЮТ** на develop |

⇒ Карточка действительно заблокирована по сути. Никакого «продвижения»
по сравнению с моментом, когда architect писал контракт, не произошло.

## 2. Что реально лежит в чужих ветках (НЕ влито в develop)

`git log --all --grep "pregenerate\|priority\|tts_node" -i` на 2026-09-07
12:58 UTC выдал несколько артефактов. **Ниже — статус, не предложение
что с ними делать.**

### 2.1 Ветка `z-{agent}/1996-operator-agent-07a-tts-node-priority-voi`
- `git branch -a --contains 09b782db` → ровно эта ветка и её origin.
- Коммит `09b782db5 wip(07a): tts_node priority queue + cascaded slot
  re-numbering`. Коммит-сообщение: «Closes #1996 (operator-agent step
  7a — priority field в /voice/tts/request)».
- Содержимое: `priority` поле в JSON `/voice/tts/request`,
  priority-aware insertion в `_submit_synthesis`, `_play_order_cond`,
  `_pending_seqs`, 6 тестов в `test_tts_priority_queue.py` — все зелёные.
- **Не влит в develop.** PR в GitHub из ветки не открыт (проверено по
  логу `agent-flow` — нет референса на PR #).

⇒ #1996 написано, но не доведено до develop. Это блокер #2003.

### 2.2 Ветка `wt/t_df57cb7a`
- `git branch -a --contains 0c8f172e6` → ровно эта ветка.
- Коммит `0c8f172e6 merge: speculative pregenerate implementation + DoD
  tests (from wt/t_f9fdc0e9-rb1)` — **merge-коммит**, вкатывает:
  - `scheduler/pregen/{__init__,decision,estimator,pre_gen,quality,speculative_executor}.py`
    (≈ 1400 LOC);
  - `tts_node.py` +572 строки (`pregenerate`, `claim_pregen`,
    `cancel_pregen` — wire-up);
  - 7 файлов тестов в `test/unit/pregen/` (DoD-именованные), в т. ч.
    `test_speculative_path_latency.py` (374 строки — замер метрики).
- Поверх ещё 3 коммита: `wip(test): speculative pregenerate DoD-named
  tests (#2003)`, `wip(adr-0056): scripts/tts_bench/chunk_latency_bench.py`,
  `docs(adr-0056): README для scripts/tts_bench/`.
- **Не влит в develop.** PR в GitHub не открыт (или был открыт и закрыт
  без merge — нет лога в `agent-flow`).
- **Зависит от `z-{agent}/1996-operator-agent-07a-tts-node-priority-voi`**:
  `git log --oneline origin/wt/t_df57cb7a ^origin/develop` показывает
  7a как **не** предка `wt/t_df57cb7a` — то есть в этой ветке 7a
  отсутствует. Реализация pregenerate рассчитана на новое поле
  `priority`, которого в `tts_node.py` этой ветки тоже нет.

⇒ Готовой реализации #2003 «в один шаг от develop» нет — нужно
сначала влить 7a.

### 2.3 Ветка `z-{agent}/2003-operator-agent-13-pregenerate-tts-node` (наша)
- `git log --oneline HEAD..origin/develop` = 14 коммитов, мой HEAD
  (`a1b823ce2`) отстаёт.
- В рабочей копии **нет** файла `2026-09-05-operator-agent-architecture-handoff.md`
  (хотя в develop HEAD он есть; не влит в эту ветку).
- В рабочей копии **нет** `target-operator-agent-and-dialogue.md`
  (его и в develop HEAD нет).

## 3. Что НЕ сделано (anti-honesty-FAIL)

Этот документ **не** утверждает, что:

- Реализация pregenerate на wt/t_df57cb7a корректна — не проверял,
  не запускал, только прочитал commit-message.
- 6 тестов в `test_tts_priority_queue.py` зелёные в текущем окружении —
  этот worktree старый, не синхронизирован с develop, запускать pytest
  не имело смысла.
- 7a действительно закрывает #1996 — только commit-message так говорит;
  до проверки merge-критериев нельзя ставить `e2e-done`.

Все эти проверки — работа следующего кодерского воркера (см. §4).

## 4. Что делать дальше — это не кодерская задача PM

PM не должен ни открывать PR от чужой ветки, ни «кодить» поверх
блокера. Следующий шаг — **запрос Шифу** на ручное решение (всё это
находится за пределами прав PM-агента):

1. **Влить или отклонить `z-{agent}/1996-operator-agent-07a-tts-node-priority-voi`**
   в develop. Это снимает блокер #2003.
2. **После 1** — влить или отклонить `wt/t_df57cb7a`. Там уже готовый
   DoD-комплект, но он зависит от 7a.
3. **Альтернатива** — оставить issue #2003 как есть (заблокированным)
   и отметить, что реализация уже существует в `wt/t_df57cb7a`.

Если Шифу выбирает вариант «влить обе ветки» — следующая карточка для
кодера: rebase `wt/t_df57cb7a` на develop (после 7a-merge), прогнать
тесты, открыть PR с заголовком «Closes #2003».

## 5. Сырые ссылки

```
$ git log --oneline HEAD..origin/develop | head -14
0cc0d242 fix(test): voice_presets fallback fixture uses map-with-label for languages (#2051)
c0dace5a docs(adr-0055): operator-agent step 5b — /avatar/tts/audio → headset (deliver_audio(stream)) (#2042)
…
e2b5556d [AV-24] Telegram: /avatar — карточка состояния супервизора…

$ git branch -a --contains 09b782db
z-{agent}/1996-operator-agent-07a-tts-node-priority-voi
remotes/origin/z-{agent}/1996-operator-agent-07a-tts-node-priority-voi

$ git branch -a --contains 0c8f172e6
+ wt/t_df57cb7a
remotes/origin/wt/t_df57cb7a

$ git log --oneline origin/wt/t_df57cb7a ^origin/develop
2d226882 docs(adr-0056): README для scripts/tts_bench/ — порядок запуска baseline vs speculative
2d7452b1 wip(adr-0056): scripts/tts_bench/chunk_latency_bench.py + stats unit tests
0c8f172e merge: speculative pregenerate implementation + DoD tests (from wt/t_f9fdc0e9-rb1)

$ grep -c "priority" src/rob_box_voice/rob_box_voice/tts_node.py
0

$ git ls-tree origin/develop docs/plans/ | grep operator-agent-architecture-handoff
100644 blob 4db74d84711c8abcf5a5e619cc3e63ccc209e560 docs/plans/2026-09-05-operator-agent-architecture-handoff.md

$ git ls-tree origin/develop | grep "target-operator-agent-and-dialogue"
(пусто)
```
