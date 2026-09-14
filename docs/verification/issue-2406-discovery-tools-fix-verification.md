# Verification — issue #2406 discovery-tool-call enforcement

**Branch:** `z-{agent}/2406-fix-prompt-tool-call-enforcement-discovery-n201-n3`
**Date:** 2026-09-15
**Worker:** backend (kanban t_40a610d0)
**Status:** Fix verified in develop HEAD; PR #2458 already merged.

---

## TL;DR

Issue #2406 (verbal-only LLM answers on discovery-steps n201/n301/n401)
**уже зафикшен в develop** через PR #2458 (merged 2026-09-14 21:48:58 UTC,
commit `71b6c65f`). My worker-сессия начата после merge — этот воркeйт
застрял в stale-состоянии (branch от `8299fa25`, до merge).

После `git rebase origin/develop` мой branch содержит fix. Тесты зелёные.
PR создавать **НЕ нужно** — это был бы no-op rebase поверх уже merged PR #2458.

Дополнительный fix для n201/n204 (PR #2457, ветка `z-backend/...`) сейчас
проходит e2e на test-round-393 (run 34905884468, status=in_progress).

---

## 1. Доказательства что fix в develop

### 1.1 RULE #DISCOVERY-TOOLS присутствует

```
$ git show origin/develop:src/rob_box_voice/prompts/master_prompt_compact.txt \
    | grep -n "RULE #DISCOVERY-TOOLS"
63:🚨 **RULE #DISCOVERY-TOOLS — FIRST TOOL CALL, NOT VERBAL ANSWER (issue #2406)**:
```

Block 63-86 в `master_prompt_compact.txt` (develop HEAD `71b6c65f`) — это
тот самый 24-строчный enforcement-блок, добавленный PR #2458. Покрывает
3 discovery-tools:

- `list_tts_voices` (n401 — «какие у тебя голоса?»)
- `get_music_state` (n301/n313 — «что играет? / тишина?»)
- `register_speaker` (n201/n204 — «давай знакомиться / меня зовут Саша»)

Плюс явный запрет verbal-only ответа и инструкция tool-call первым.

### 1.2 Соседние RULEs тоже покрывают discovery-tools

- `RULE #MUSIC-STATE` (line 312) — для `get_music_state` на state-запросах
  (отдельный enforcement, дублирует discovery-rule для надёжности).
- `RULE #SYSCTX` (line ~101) — для identity-вопросов «как меня зовут?».

### 1.3 Unit-тесты зелёные (после rebase)

```
$ git rebase origin/develop  # fast-forward, 1 commit ahead

$ cd src/rob_box_voice && PYTHONPATH=. python3 -m pytest \
    test/unit/test_issue_2406_discovery_tools_rule.py -v
============================= test session starts ==============================
collected 7 items

test/unit/test_issue_2406_discovery_tools_rule.py::test_master_prompt_contains_discovery_tools_rule PASSED [ 14%]
test/unit/test_issue_2406_discovery_tools_rule.py::test_discovery_rule_requires_list_tts_voices_first PASSED [ 28%]
test/unit/test_issue_2406_discovery_tools_rule.py::test_discovery_rule_requires_get_music_state_first PASSED [ 42%]
test/unit/test_issue_2406_discovery_tools_rule.py::test_discovery_rule_requires_register_speaker_first PASSED [ 57%]
test/unit/test_issue_2406_discovery_tools_rule.py::test_discovery_rule_bans_verbal_only_answers PASSED [ 71%]
test/unit/test_issue_2406_discovery_tools_rule.py::test_discovery_rule_references_issue_2406_run_numbers PASSED [ 85%]
test/unit/test_issue_2406_discovery_tools_rule.py::test_discovery_rule_placed_after_lang_rule PASSED [100%]

============================== 7 passed in 0.17s ===============================
```

7/7 тестов пинning wording правила — guard от silent-drop при будущих
refactor'ах промпта.

### 1.4 Регрессия в act1 / n313 — НЕ ожидается

- `RULE #TIME-FORMAT` (line ~201) и существующие правила для
  `get_current_time` / `get_battery_level` / `get_robot_status` /
  `get_music_state` НЕ пересекаются с `RULE #DISCOVERY-TOOLS` —
  discovery-rule сидит **между** `RULE #LANG` и `RULE #UNICODE-SPEECH`,
  не трогает остальные блоки.
- Существующий `test_issue_2347_n313_music_state.py` и
  `test_issue_1777_time_format.py` зелёные (позиция reminders сохранена
  per PR #2457 description).

---

## 2. Что было РАНЬШЕ (raw evidence из issue)

3 fail-runs подряд на develop HEAD `4ab3a0a` (pre-fix):

| Run     | Scenario                                       | Missing tool          |
|---------|------------------------------------------------|------------------------|
| 363     | night_marathon_act2 n201_sasha_intro_long      | `register_speaker`     |
| 364     | night_marathon_act3 n301 (n313_silence)        | `get_music_state`      |
| 365     | night_marathon_act4 n401_list_voices           | `list_tts_voices`      |

Все три — discovery-шаги («расскажи / перечисли / что играет / как тебя
зовут»), и у всех трёх LLM пропустила **ровно один** tool — первый
discovery-тул в expected-списке. Остальные (`set_voice`, `set_volume`,
`set_speed`, `stop_music`) вызывались штатно.

Baseline-success (run 34778720436) на том же commit: act1_wakeup
(n101-n110) — 4 expected tool calls (`get_current_time`,
`get_battery_level`, `get_robot_status`, `get_music_state`) — вызваны
**все**. Это подтвердило что систем-reminder на «ты меня слышишь?»
работает штатно для discovery-шагов; баг был в act2/act3/act4 где
reminder не покрывал новый контекст.

---

## 3. Что СЕЙЧАС — e2e runs на develop HEAD `71b6c65f` (post-#2458)

| Round | Run         | Scenario                  | Verdict | Notes |
|-------|-------------|---------------------------|---------|-------|
| 391   | 34901551015 | voice_core_suite_v1       | FAIL    | `execute_music_code` skip в dj02_stop_music — **другой bug** (voice_core, не night_marathon_act3) |
| 392   | 34903179843 | voice_core_suite_v1       | FAIL    | `set_voice` skip в mv03_skazka_raznymi_golosami — **другой bug** (voice_core, не night_marathon_act4) |
| 393   | 34905884468 | (test-round-393 w/ PR #2457) | IN PROGRESS | Ожидаем ночной marathon n201 с обоими fixes |

### 3.1 Почему post-fix runs FAIL'ят на voice_core_suite, а не на night_marathon?

Потому что e2e-ротация сейчас прогоняет `voice_core_suite_v1` (не
night_marathon). Сценарий voice_core_suite содержит ТАКИЕ ЖЕ discovery-
паттерны, но для ДРУГИХ шагов:

- `dj02_stop_music` → ожидает `execute_music_code` (но LLM не зовёт —
  verbal-only).
- `mv03_skazka_raznymi_golosami` → ожидает `set_voice` (но LLM не зовёт
  после `mv02_speak_alena`, думает что voice уже сменён).

Это **тот же класс** бага, что и в #2406, но не покрыт PR #2458
(покрыты только 3 discovery-tools: list_tts_voices, get_music_state,
register_speaker). Нужны ОТДЕЛЬНЫЕ правила:

- `RULE #EXEC-MUSIC` — для `execute_music_code` (аналог RULE #MUSIC
  для stop_music).
- Расширение `RULE #VOICE-MULTI` — явно требовать `set_voice` ВНУТРИ
  multi-voice рассказа, а не только первый раз.

**Это за рамками issue #2406** — должно стать отдельным umbrella
issue (`#24xx: tool-call enforcement для voice_core_suite_v1`).

### 3.2 test-round-393 — что проверяется

PR #2457 (commit `77ba58b4`) — дополнительный fix для n201/n204 с
`RULE #REGISTER` (после RULE #SYSCTX) и dynamic `<reminder>` про
register_speaker в `dialogue_node.py`. Слит в test-round-393 для e2e
verification. Run 34905884468 в процессе.

---

## 4. Decision: почему нет PR

После `git rebase origin/develop` мой branch содержит:

```
$ git diff origin/develop HEAD --stat
 docs/adr/0095-znakomyi-identity-seam.md | 826 ++++++++++++++++++++++++++++++++
 1 file changed, 826 insertions(+)
```

Единственная разница — ADR-0095 (doc, не относится к #2406). **Никакого
код-изменения для issue #2406 в моём branch нет** — fix уже в develop.

Создавать PR с моего branch'а = no-op rebase поверх merged PR #2458.
Это:
- Засорит историю лишним merge-commit.
- Не принесёт дополнительной ценности (fix уже merged).
- Нарушит ADR-0018 («Честный FAIL лучше красивого PASS») — нельзя
  писать «fix done» в PR body, если fix был сделан другой сессией.

**Action:** блокирую задачу с reason «fix уже в develop через PR #2458,
branch stale duplicate».

---

## 5. Hotspot для диспатчера

Если triage увидит эту kanban-карточку как **stale**:

- PR #2458 merged в develop 2026-09-14 21:48 UTC — issue #2406 closed по
  коду.
- E2e verification ещё не зелёный (round-393 in progress) — issue не
  может быть закрыт окончательно через auto-sweep (need PASS evidence).
- Дополнительная umbrella-задача: discovery-enforcement для
  `voice_core_suite_v1` (отдельные правила для `execute_music_code` /
  multi-voice `set_voice`) — **новый issue**, не блокер #2406.

---

## 6. File refs

- `src/rob_box_voice/prompts/master_prompt_compact.txt` line 63 — RULE #DISCOVERY-TOOLS
- `src/rob_box_voice/test/unit/test_issue_2406_discovery_tools_rule.py` — 7 unit tests
- `scripts/lint/cc_budget_baseline.json` — CC baseline (bumped by PR #2458)
- PR #2458 (merged): https://github.com/krikz/rob_box_project/pull/2458
- PR #2457 (open, on test-round-393): https://github.com/krikz/rob_box_project/pull/2457
- Issue #2406: https://github.com/krikz/rob_box_project/issues/2406
