# Test-suite budget guideline (ADR-0048)

## Зачем

tester в Hermes — это профиль для **генерации test-suite**, а не smoke-тестов. Исторически у него было `agent.max_turns=30` (минимум для smoke). С ростом test-suite задач (10+ полей dataclass, parametrization, edge cases) 30 turns стало не хватать → 5+ timed_out подряд на одних и тех же карточках.

После ADR-0048 (2026-09-02) профиль `tester` поднят до **60 turns**. Этот документ — короткая памятка, чтобы не возвращаться к проблеме.

## Бюджеты по типу задачи

| Тип задачи (assignee=tester) | Реалистичный расход | Бюджет в профиле | OK? |
|---|---|---|---|
| Один smoke unit-test (1 файл, 1 test) | 5-10 turns | 60 | ✓ |
| **Test-suite generation** (10+ полей dataclass, parametrization) | **50-70 turns** | 60 | ✓ (впритык) |
| Test-fix (нашёл регрессию, дописать 1-2 теста) | 15-25 turns | 60 | ✓ |
| E2E test (Playwright/Selenium, mock-server) | 30-50 turns | 60 | ✓ (если без sdlc-review skill) |

Если ты запускаешь test-suite карточку **с явной parametrize** или **10+ полей value-object** — планируй 50-70 turns. Если профиль показывает `30/30` или даже `60/60` — это **сигнал**, что задача слишком большая для одного воркера, и надо декомпозировать (см. §Декомпозиция).

## Когда карточка tester требует `goal_max_turns` override

По умолчанию `task.goal_max_turns=NULL` → goal-loop не запускается (`goal_mode=0` на tester). Это **нормально**, потому что tester работает в обычном agent-loop с `max_iterations=60`.

Override имеет смысл **только если** ты явно включаешь `goal_mode=1` в карточке (пока таких карточек на доске 0). Если включаешь — ставь `goal_max_turns=45-60`, не больше.

## Декомпозиция больших test-suite задач

Если понимаешь, что задача займёт 80+ turns:

1. **Не поднимай `max_turns` ещё выше.** 80+ turns = воркер потеряет контекст на полпути.
2. **Декомпозируй по test-class.** Пример для `TTSSettings`:
   - `t_<id>-ttssettings-voice-model-lang` — базовые поля (1 файл, 5-10 тестов).
   - `t_<id>-ttssettings-prosody` — speed/volume/pitch (1 файл, 5-10 тестов).
   - `t_<id>-ttssettings-format-sample` — format/sample_rate (1 файл, 3-5 тестов).
   - `t_<id>-ttssettings-pronunciation-dict` — MiniMax-специфика (1 файл, 3-5 тестов).
3. **Связать через `parents=[root_id]`** — финальная карточка собирает PR.

## Диагностика «30/30» или «60/60»

Если опять видишь `Iteration budget exhausted`:

```bash
# 1. Проверить конфиг
grep max_turns ~/.hermes/profiles/tester/config.yaml
# Ожидаем: max_turns: 60

# 2. Проверить историю на доске
sqlite3 ~/.hermes/kanban/boards/robbox/kanban.db \
  "SELECT t.id, t.title, COUNT(*) FROM task_events e
   JOIN tasks t ON t.id=e.task_id
   WHERE t.assignee='tester' AND e.kind='timed_out'
   GROUP BY t.id ORDER BY COUNT(*) DESC LIMIT 5"
# Если несколько карточек с >3 timed_out — это уже **системная** проблема
# (например, skill загружает слишком много, или value-object слишком большой).

# 3. Проверить, что MAINTENANCE-файл не активен
ls -la ~/.hermes/MAINTENANCE 2>&1
# Если есть — воркер может выходить раньше.
```

## Связанные

- ADR-0048 — основной документ, утверждает `tester.max_turns=60`.
- `t_c401ecaa` — карточка-источник аномалии (5 timed_out на TTSSettings, issue #1780).
- `agent/turn_finalizer.py:193` — где формируется сообщение «Iteration budget exhausted».
- `hermes_cli/config.py:3382` — `_normalize_max_turns_config` (нормализация legacy `max_turns` → `agent.max_turns`).
