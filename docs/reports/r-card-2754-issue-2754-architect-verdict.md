# Architect verdict: issue #2754 — bug(e2e/voice): незнакомец опознаётся как известный диктор (0.73–0.82), у MiniMax нет четырёх различимых голосов

**Kanban:** t_1fdd7990
**Source issue:** [#2754](https://github.com/krikz/rob_box_project/issues/2754)
**Verdict:** **READY TO CLOSE** — реализация ADR-0134 уже в develop (raw-evidence ниже)
**Reviewer:** architect
**Date:** 2026-10-03

---

## TL;DR (для Шифу)

**Issue #2754 закрывается реализацией ADR-0134**, которая уже в develop. Все четыре
acceptance-criteria ADR-0134 §5 выполнены, unit-тесты зелёные. Дыра в проверке
закрыта, регрессия n210_grisha_no_name больше не пройдёт незамеченной.

**Никакого нового кода эта карточка не вносит.** Это **архитектурная верификация**:
карточка пришла ко мне (architect) с уже готовым ADR и уже влитой реализацией — мне
остаётся подтвердить соответствие «as built ↔ as designed» и рекомендовать close.

**Что нужно от тебя (товарищ Шифу):**
- Закрыть #2754 (raw-evidence в §3) как `resolved-by-feature`.
- (опционально) Если хочешь — добавить в #2754 финальный комментарий-резолюцию
  со ссылкой на ADR-0134 + #3020/#3023/#3070 (шаблон в §6 ниже).

---

## 1. Что показывал issue #2754 (контекст)

**Симптом (run 35699257202, шаг `n210_grisha_no_name`):**
- TTS-голос `zahar` (дядя Гриша) намеренно не называет имя.
- `speaker_id_node` всё равно публикует `Speaker: 'Борис' confidence=0.816`.
- Шаг зелёный, потому что проверяется только `must_not_call: register_speaker`.
- Дыра: «identify выдал Бориса» — никто не ассертит.

**Корневая причина (issue #2754 / Шифу в body):**
- У MiniMax нет 4 различимых русских голосов. Худшая пара в матрице
  28 пар = 0.684 (`evidence/tts-voice-distinctness-2026-09-22/minimax_final.json`).
- Микрофонный канал поднимает косинус: 0.684 в файлах → 0.73–0.82 на роботе.
- Боевая пара Саша/Ночной-инженер = **0.929** (speaker_db_admin.py list,
  22.09.2026) — вчетверо меньший зазор между голосами (0.071), чем до порога (0.209).
- **Порогом не чинится** (#2747): 0.72 одновременно высок для живого и низок для синтетики.

**Главный вывод:** акт 2 (требующий различить 4 персонажа подряд по голосу) при
`E2E_TTS_PROVIDER=minimax` красный по построению. Это **свойство тестовых данных**,
а не баг кода. Лечить надо не порог и не провайдера, а **дыру в проверке**.

---

## 2. Решение ADR-0134 — что было спроектировано (raw-evidence)

ADR-0134 (PR #3020, смержен): `docs/adr/0134-e2e-voice-distinctness-contract.md`.

**B (принято)** — инвариант на шаге: `must_not_identify_as: list[str]` в `acceptance`
шага. Парсер ищет в лог-стриме шага финальный вердикт `Speaker: '<NAME>'` (с эмодзи
или без) и красит шаг красным, если `NAME in must_not_identify_as`. Семантика:
«незнакомец не опознан как известный диктор».

**C (принято)** — provider-aware guard `_validate_voice_distinctness()` в
`scripts/e2e/gen_night_marathon.py`. Helper поднимает
`evidence/tts-voice-distinctness-*/<provider>_voices.json` и fail-fast при генерации,
если голоса не разводятся. Для greenfield-провайдеров (файла замера нет) — warning.

**A (отклонено)** — разнести персонажей по разным TTS-провайдерам. KISS / YAGNI:
это лечит свойство, которое тесту не нужно (см. ADR-0134 §3.1).

**D (отклонено)** — отказаться от MiniMax в пользу Yandex. Противоречит ADR-0009
(мульти-провайдерность) и требует рабочий yandex-ключ (сейчас `PERMISSION_DENIED`).

---

## 3. Что реализовано (raw-evidence: SHA, файлы, тесты)

### 3.1 PR-ы, смерженные в develop

| PR | SHA | Title | Что делает |
|---|---|---|---|
| #3020 | `39031bf7` | docs(adr): 0134 — e2e voice distinctness contract | Сам ADR |
| #3023 | `c82e46a9` | test(ADR-0134 #2754): helpers + unit-tests for must_not_identify_as + voice distinctness guard | Unit-тесты |
| #3070 | `9ba5d0b6` | [ADR-0134] feat(e2e): enforce must_not_identify_as for unknown speakers | Реализация |

Все три смержены (PR #3070 = 2026-09-27 12:44Z). `git log --grep="0134"` в develop:

```
9ba5d0b6 [ADR-0134] feat(e2e): enforce must_not_identify_as for unknown speakers (#3070)
c82e46a9 test(ADR-0134 #2754): helpers + unit-tests for must_not_identify_as + voice distinctness guard (#3023)
39031bf7 docs(adr): 0134 — e2e voice distinctness contract (issue #2754) (#3020)
```

### 3.2 Acceptance-критерий #1 — `n210_grisha_no_name.must_not_identify_as == ["Борис", "Саша"]`

```json
// .github/e2e/scenarios/night/night_marathon_act2_acquaintance_v1.json
{
  "label": "n210_grisha_no_name",
  "voice": "zahar",
  "acceptance": {
    "must_not_call": ["register_speaker"],
    "must_not_say": ["Борис", "Саш", "Спартак", "пицц"],
    "must_not_identify_as": ["Борис", "Саша"],
    ...
  }
}
```

Подтверждено `python3 -c 'import json; ...'` (см. §5). ✅

### 3.3 Acceptance-критерий #2 — парсер `identify_failures` подключён

`.github/workflows/scripts/e2e_voice_test.sh`:
- L2341: `from e2e_tool_match import keyword_hit, robot_speech, identify_failures`
- L2365: `identification_failures = identify_failures(acc, logs)`
- L2508-2509:
  ```python
  if identification_failures:
      failures.extend(identification_failures)
  ```
- L2538-2539: `must_not_identify_as` и `identify_failures` пробрасываются в acceptance.json.

`.github/workflows/scripts/e2e_tool_match.py:540-547` — якорь парсера:
```python
_IDENTIFY_VERDICT_RE = re.compile(
    r"(?:👤\s*)?Speaker:\s*['\"]([^'\"]+)['\"]"
)
```

Якорь устойчив к старому формату без эмодзи (см. ADR-0134 §2.1 / §6 риск #2). ✅

### 3.4 Acceptance-критерий #3 — unit-тесты зелёные

Локальный прогон `python3 -m pytest tests/unit/e2e_scripts/ tests/unit/test_gen_night_marathon_voice_guard.py --no-cov`:

```
========================= 313 passed, 9 skipped, 2 warnings in 29.34s ==========================
```

Покрытие по #2754:
- `test_e2e_voice_test_must_not_identify_as.py` — **10 passed**:
  ```
  TestMustNotIdentifyAsBlocksKnownMatch::test_must_not_identify_as_blocks_known_match PASSED
  TestMustNotIdentifyAsPassesForUnknown::test_must_not_identify_as_passes_for_unknown PASSED
  TestMustNotIdentifyAsPassesForUnknown::test_candidates_diagnostik_does_not_trigger PASSED
  TestMultipleNamesBlock::test_multiple_names_block PASSED
  TestAnchorRobustToOldFormat::test_old_format_without_emoji_also_blocked PASSED
  TestEmptyForbiddenListIsZeroEffect::test_no_field_returns_empty PASSED
  TestEmptyForbiddenListIsZeroEffect::test_empty_list_returns_empty PASSED
  TestSchemaErrorsAreReported::test_non_list_field_is_schema_error PASSED
  TestSchemaErrorsAreReported::test_non_string_element_is_partially_checked PASSED
  TestNoVerdictIsNotFailure::test_no_verdict_at_all_is_zero_effect PASSED
  ```
- `test_gen_night_marathon_voice_guard.py` — **6 passed**:
  ```
  TestFourIdenticalVoicesRaise::test_four_identical_voices_raises PASSED
  TestYandexGreenfieldWarnsNoData::test_yandex_warns_when_no_data PASSED
  TestThresholdOverride::test_threshold_override_takes_effect PASSED
  TestNoFalsePositive::test_no_falsely_passing_pair PASSED
  TestAsymmetricKeysSupported::test_only_one_side_of_pair_key_is_enough PASSED
  TestMissingPairInEvidenceIsSkipped::test_unmeasured_pair_is_not_violation PASSED
  ```

9 skipped — `yaml`/`golden/<slug>.wav` (не связано с #2754). ✅

### 3.5 Acceptance-критерий #4 — greenfield-провайдер не ломает CI

`TestYandexGreenfieldWarnsNoData::test_yandex_warns_when_no_data PASSED` подтверждает:
при отсутствии `evidence/.../yandex_voices.json` (текущее состояние, ключ робота
`PERMISSION_DENIED`) guard уходит в warning, не error. То есть **текущий CI
(`provider=yandex`, файл замера отсутствует) работает без изменений**. ✅

### 3.6 Acceptance-критерий #5 — minimax guard срабатывает

Логика `_validate_voice_distinctness()` (ADR-0134 §2.2 + реализация в
`gen_night_marathon.py:211-359`) при наличии `minimax_voices.json` поднимет
`SystemExit(2)` на act 2 (потому что act 2 заточен под yandex, и смена провайдера
на minimax без переписывания сценария должна быть видна красным на этапе генерации).
Это и есть то, чего мы хотим — **видимый красный** вместо тихого false-green.
Подтверждено unit-тестом `test_four_identical_voices_raises`. ✅

---

## 4. Acceptance-критерии ADR-0134 §5 — статус

- [x] ADR-0134 смержен. ✅ (PR #3020)
- [x] PR с реализацией прошёл `Python Code Quality` и unit-тесты зелёные. ✅
      (PR #3070 + #3023, unit-тесты в §3.4).
- [x] `gen_night_marathon.py` для `provider=yandex` без файла замера — только warning. ✅
      (`test_yandex_warns_when_no_data`).
- [x] `gen_night_marathon.py --provider=minimax` с подложенным замером fail-fast
      на act 2. ✅ (`test_four_identical_voices_raises` + ADR-0134 §2.2).
- [x] Регенерированный `n210_grisha_no_name.acceptance.must_not_identify_as == ["Борис", "Саша"]`. ✅
- [ ] Issue #2754 получает комментарий-резолюцию + close. **← Шифу.**
- [ ] Закрытие #2754 после мержа Шифу (по правилу «merge только владелец»).
      Реализация уже в main (`dd93728e chore: release develop to main (2026-09-29) (#3194)`)
      и в develop (`3941f848 chore: release develop to main (#3098)`), правило выполнено.

**Все автоматические критерии выполнены.** Остался человеческий шаг — close issue.

---

## 5. Reproducible raw-evidence (для Шифу — можешь прогнать сам)

```bash
# 1. Acceptance-критерий #1
python3 -c "import json; d=json.load(open('.github/e2e/scenarios/night/night_marathon_act2_acquaintance_v1.json')); s=[x for x in d['steps'] if x['label']=='n210_grisha_no_name'][0]; print(s['acceptance'])"

# 2. Acceptance-критерий #3 — все тесты
python3 -m pytest tests/unit/e2e_scripts/test_e2e_voice_test_must_not_identify_as.py \
                  tests/unit/test_gen_night_marathon_voice_guard.py --no-cov -v

# 3. PR-ы
gh pr view 3020 --json state,mergedAt
gh pr view 3023 --json state,mergedAt
gh pr view 3070 --json state,mergedAt

# 4. Issue статус
gh issue view 2754 --json state,closedAt,closedByPullRequestsReferences
```

---

## 6. Шаблон финального комментария для #2754 (если Шифу попросит)

```markdown
Резолюция: реализовано в ADR-0134 (#3020) + PR-ы #3023 (тесты) и #3070
(реализация), все вмержены в develop и в main (через release PR #3098 / #3194).

Дыра в проверке `n210_grisha_no_name` закрыта инвариантом `must_not_identify_as`
в acceptance шага — парсер `identify_failures` (e2e_tool_match.py:550) ловит
финальный вердикт `Speaker: '<NAME>'` (с эмодзи или без) и красит шаг красным,
если NAME входит в список запрещённых. Для n210 это `["Борис", "Саша"]`.

Корневая причина (свойство тестовых данных, не баг кода) — у MiniMax нет
4 различимых русских голосов. Лечить порогом или подменой провайдера
неправильно (см. ADR-0134 §3), правильно — ассертить семантику шага
(«незнакомец не опознан»), а не различимость голосов.

Полный архитектурный разбор: docs/reports/r-card-2754-issue-2754-architect-verdict.md.
Unit-тесты: 313 passed, 9 skipped (yaml/golden w/o fixtures — не связано с #2754).
Closes #2754.
```

---

## 7. Trade-offs / известные ограничения (наследуются из ADR-0134)

1. **Guard слишком строгий для greenfield-провайдеров:** нет файла замера →
   warning. Снятие: `python3 scripts/e2e/measure_tts_voice_distinctness.py --provider X`
   + коммит `evidence/.../<X>_voices.json`. Для Yandex сейчас это упёрто в
   `PERMISSION_DENIED` от ключа робота.
2. **Якорь лога `Speaker: '<NAME>'`** может эволюционировать. ADR-0134 §6
   описывает смягчение — парсер уже устойчив к старому формату без эмодзи.
3. **`must_not_identify_as` не различает «обознался Сашей» vs «обознался Борисом»**
   в терминах репорта — оба попадают в одну строку диагностики. Это правильно:
   обе ошибки одинаково wrong.
4. **Не покрывает гонку `register_or_merge`** (если Гриша «прибьётся» к Борису
   как имя) — это отдельный класс бага, ADR-0127 закрыт юнит-тестами.
5. **Расход на регенерацию:** только act-2 JSON (не все 12), diff минимизирован
   (см. PR #3070 body — «регенерирует только act-2 scenario JSON»).

---

> «Честный FAIL лучше красивого PASS» (ADR-0018). Этот документ — **верификация
> готовности к close**, а не «я починил». Реализация сделана в PR #3070 до того,
> как карточка t_1fdd7990 попала на доску.
