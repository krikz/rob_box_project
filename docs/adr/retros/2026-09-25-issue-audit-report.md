# Issue audit 2026-09-25 (chore #3016)

## Summary

Полный аудит всех 63 открытых issue в `krikz/rob_box_project` (включая саму #3016).

| Категория | Кол-во |
|-----------|--------|
| Всего открыто до аудита | 63 (исключая #3016 = 62) |
| Закрыто с evidence (round 1) | 45 |
| Дозакрыто с evidence (round 2 — body-only references) | 3 |
| **Итого закрыто** | **48** |
| Оставлено открытыми (помечены audit-комментарием) | 14 |
| Issue #3016 (текущая задача) | 1 |

## Критерии закрытия

Issue закрыт **только** если выполнены все 3 условия:
1. В `git log origin/develop` найден коммит, у которого subject **ИЛИ** body содержит явную ссылку на этот issue (`#N` или `issue #N`).
2. Этот коммит имеет conventional prefix (`fix/feat/refactor/...`), не `wip`/`report`/`docs` чисто диагностического характера.
3. `git merge-base --is-ancestor <sha> origin/develop` → 0 (коммит реально в develop).

PR-номер извлечён из subject (последний `(#N)`), проверен через `gh pr view`.

## Метод

1. `gh issue list --state open` → 63 issues
2. Для каждого: `git log origin/develop --all --grep=#N` (subject match)
3. Для найденных: проверить commit body на `issue #N`/`#N`
4. Для каждого candidate: `git merge-base --is-ancestor` → да/нет
5. Для candidates в develop: `gh pr view <PR> --json mergedAt,title,url`
6. Compose closing comment + `gh issue close --reason completed`
7. Для оставшихся 14: оставить audit-комментарий с пояснением

## Закрытые issues (48)

Round 1 (45):
- #2989, #2980, #2979, #2978, #2977, #2971, #2970, #2969, #2968, #2967,
  #2966, #2965, #2964, #2963, #2959, #2955, #2949, #2943, #2942, #2941,
  #2939, #2934, #2932, #2931, #2926, #2925, #2924, #2908, #2907, #2863,
  #2859, #2856, #2841, #2837, #2829, #2824, #2817, #2771, #2767, #2766,
  #2751, #2747, #2676, #2625, #2556

Round 2 (3) — найдены через body-only references:
- #2962 (коммит 53e6da06e «fix(music/compose): ручной lead_octave» body="issue #2962")
- #2961 (коммит 2e3cc5425 «fix(music/key): тай-брейк» body="issue #2961")
- #2960 (коммит 85e1f521f «fix(music/rtttl_compose): детектор затакта» body="issue #2960")

## Оставлены открытыми (14)

Причины:
- **NO_MATCH** (4): #2956, #2930, #2826, #2754 — нет ни одного коммита с упоминанием
- **WIP_OR_REPORT_ONLY** (6): #3014, #3013, #3005, #3004, #3000, #2995 — только wip/report коммиты (диагностика)
- **FIX_IN_FEATURE_BRANCH** (4): #3008, #2999, #2997, #2805 — есть fix-коммит, но в feature-ветке (не влит в develop)

## Скрипты аудита

Все скрипты лежат в `/tmp/` (не в репо, чтобы не мусорить):
- `/tmp/analyze_open_issues.py` — базовый список
- `/tmp/find_commits_for_issues.py` — git log grep
- `/tmp/classify_commits.py` — категоризация commit
- `/tmp/verify_in_develop.py` — merge-base проверка
- `/tmp/final_selection.py` — финальный фильтр
- `/tmp/get_pr_v2.py` — извлечение PR через gh
- `/tmp/prepare_close_v2.py` — генерация close-комментариев
- `/tmp/deep_audit.py` — round 2 (body-only references)
- `/tmp/prepare_keep_open_comments.py` — audit-комментарии для оставшихся

## Доказательства

Каждое закрытое issue получило:
- sha (12 hex)
- subject коммита
- PR номер + URL
- merged_at timestamp

Каждый оставшийся issue получил:
- список связанных коммитов с классификацией (WIP/REPORT/POSSIBLE_FIX/OTHER)
- статус «в develop» или «в feature-ветке <name>»
- причину «почему не закрыто»

## Что НЕ делалось

- Не правился код
- Не удалялись issues
- Не менялись title/body/issues
- Не ставились метки (чтобы не зацепить agent-flow cron)

## Verification

`gh issue list --state open --limit 200` после всех операций → 15 issues
(1 — сама #3016, 14 — оставлены открытыми с audit-комментарием).

Воркер не модифицировал ни одного файла в репо (working tree clean).