# ADR-0052: `wake_words.yaml` — override operator namespace, а не SSoT; SSoT в коде

| Поле | Значение |
|---|---|
| Статус | Accepted (kanban t_e1d6d151 / issue #2022) |
| Дата | 2026-09-07 |
| Автор | architect |
| Контекст | Ревью компонента `src/rob_box_voice` за 2026-09-06 — формально `docker/vision/config/wake_words.yaml` объявлен «SSoT», но фактически достижим только в docker-compose vision, а секция `personality` дублирует `DEFAULT_WAKE_WORDS` байт-в-байт. |
| Затрагивает | `docker/vision/config/wake_words.yaml` (источник-правды изменяется), `src/rob_box_voice/setup.py` (data_files добавляется), `src/rob_box_voice/rob_box_voice/core/dialogue_text.py` (докстринг `resolve_wake_word_namespaces`), `src/rob_box_voice/test/unit/core/test_wake_words_config.py` (переформулировать проверку), комментарии в `src/rob_box_voice/config/*.yaml` и `docker/vision/config/voice_assistant/*.yaml` |
| Родители | ADR-0028 (avatar supervisor — wake-роутер по namespace §3), ADR-0004 (per-node ROS2 config — формат) |
| Связанные | issue #2022 (это ревью), issue #1990 / PR #2011 (что привело к текущему состоянию), test `test_wake_words_are_declared_exactly_once` (доктрина «список — в коде, не в YAML»), test `test_wake_word_sync` (#1252), handoff `docs/plans/2026-09-05-operator-agent-architecture-handoff.md` §7.3 |

> **TL;DR.** Вейк-слова личности (`personality`) — **SSoT в коде**
> (`rob_box_voice.core.dialogue_text.DEFAULT_WAKE_WORDS`). YAML
> `config/wake_words.yaml` — **deploy-time override ТОЛЬКО для operator namespace**
> (и других будущих не-личностных namespace). Секция `personality:` в YAML —
> **запрещена** (как запрещены и сами `wake_words:` в per-node YAML по
> `test_wake_words_are_declared_exactly_once`). Это устраняет байт-в-байт
> дублирование (источник регрессий #1252/#1734) и делает YAML достижимым из
> любого окружения.

---

## 1. Контекст и бизнес-проблема

PR #2011 (issue #1990, merged 2026-09-06) ввёл wake-роутер в `stt_node.py` и
файл `docker/vision/config/wake_words.yaml` как «единственный источник истины
(SSoT) вейк-слов» (целевая §7.3). Файл содержит два namespace:

```yaml
personality:
  - роб бокс
  - роббокс
  ...
operator:
  - тарс
  - tars
```

Внутри пакета `src/rob_box_voice`:

- `setup.py:21-23` ставит в share только `config/*.yaml` самого пакета.
  `wake_words.yaml` физически лежит в `docker/vision/config/` — другой корень,
  не копируется в образ ни при `colcon build`, ни bare-metal.
- `src/rob_box_voice/config/{stt_node,dialogue_node}.yaml` имеют
  `wake_words_file: ""` — осмысленно пусто. Нода читает кодовые фолбеки.
- `core/dialogue_text.py::resolve_wake_word_namespaces` при `path=None`
  возвращает `(DEFAULT_WAKE_WORDS, DEFAULT_OPERATOR_WAKE_WORDS)`.
- `core/dialogue_text.py::DEFAULT_WAKE_WORDS` — байт-в-байт копия секции
  `personality` из YAML. Это поддерживается
  `test_personality_matches_code_defaults_byte_for_byte`.

Где YAML достижим:

| Окружение | YAML доступен? | Что работает |
|---|---|---|
| docker-compose vision (`./config:/config:ro`) | ✅ `/config/wake_words.yaml` | YAML override |
| main-контейнеры (zenoh/oak-d/led/ceiling/...) | ❌ volume не монтирует YAML | n/a — wake-нод не запускают |
| colcon build → pytest | ❌ | кодовый фолбек |
| bare-metal ROS2 | ❌ | кодовый фолбек |
| CI без docker | ❌ | кодовый фолбек |
| dev-env без файла | ❌ (явно `wake_words_file: ""`) | кодовый фолбек |

Итог: **«SSoT» существует ровно в одном деплой-контуре** (vision). В остальных
шести — фолбек. Доктрина «список — в коде» (закреплённая в
`test_wake_words_are_declared_exactly_once` от #1252/#1734) уже сложилась,
но PR #2011 её нарушил для namespace `personality`, обосновав это «namespace
routing нужен в проде».

Правка wake-слова «только в YAML» сегодня:

- в docker vision: подействует ✅
- в colcon build / bare-metal / тестах / CI: **тихо не подействует** ⚠️ — ровно
  тот класс расхождения #1252, только теперь между деплой-контурами.

---

## 2. Инвариант (как должно быть)

```text
rob_box_voice.core.dialogue_text.DEFAULT_WAKE_WORDS         <= SSoT personality
rob_box_voice.core.dialogue_text.DEFAULT_OPERATOR_WAKE_WORDS <= SSoT operator (fallback)
docker/vision/config/wake_words.yaml                         <= override operator ONLY
src/rob_box_voice/config/wake_words.yaml                     <= в share, override operator
```

- В коде — **единственный источник истины** для `personality` (порядок списка
  важен: regex leftmost-first).
- В YAML — **только override для operator namespace** (и будущих namespace,
  не относящихся к личности). `personality` секция в YAML **запрещена**.
- Нода: `personality` всегда берётся из кода. `operator` — из YAML при наличии
  файла, иначе из кода.
- YAML достижим из **любого** окружения: монтируется docker-volume в vision,
  попадает в share через `setup.py` для colcon/bare-metal/CI.

---

## 3. Решение

### 3.1. `core/dialogue_text.py` — `personality_fallback` остаётся `DEFAULT_WAKE_WORDS`, но override теперь только operator

Текущий контракт `resolve_wake_word_namespaces`:

```python
loaded = load_wake_word_namespaces(path)
personality = loaded.get("personality") or list(personality_fallback)
operator    = loaded.get("operator") or list(operator_fallback)
```

Остаётся, но **семантика `personality` секции в YAML меняется**: она
**игнорируется** (или — эквивалентно — файл, в котором она объявлена,
отвергается загрузчиком). Минимальный безопасный вариант — оставить загрузку
на уровне документа, но явно задокументировать: «override operator only».

Рекомендуемый код (владелец примет):

```python
def resolve_wake_word_namespaces(
    path: str | os.PathLike[str] | None,
    personality_fallback: Sequence[str] = DEFAULT_WAKE_WORDS,
    operator_fallback:    Sequence[str] = DEFAULT_OPERATOR_WAKE_WORDS,
) -> tuple[list[str], list[str]]:
    """personality — ВСЕГДА из personality_fallback (SSoT в коде).
    operator — из YAML, если файл есть и namespace непуст; иначе fallback.

    Секция ``personality:`` в YAML ЗАПРЕЩЕНА (ADR-0052): если она объявлена,
    выводим WARNING в лог ноды и игнорируем — порядок списка критичен
    (regex leftmost-first), и редактировать его безопаснее в коде, чем в YAML,
    который рассинхронизируется между контурами (issue #2022).
    """
    loaded = load_wake_word_namespaces(path)
    if "personality" in loaded:
        # WARNING через rob_box_core.logging — personality в YAML запрещена.
        _log.warning(
            "wake_words.yaml содержит секцию `personality:` — игнорируется "
            "(ADR-0052: SSoT — rob_box_voice.core.dialogue_text.DEFAULT_WAKE_WORDS)"
        )
    operator = loaded.get("operator") or list(operator_fallback)
    personality = list(personality_fallback)
    return personality, operator
```

### 3.2. `setup.py` — добавить `config/wake_words.yaml` в data_files пакета

```python
(os.path.join('share', package_name, 'config'),
    glob('config/*.yaml') + glob('config/*.json')),
```

`config/wake_words.yaml` (см. 3.3) уже будет матчиться `glob('config/*.yaml')`
после переноса.

### 3.3. `src/rob_box_voice/config/wake_words.yaml` — перенести файл, убрать `personality`

Новый путь: `src/rob_box_voice/config/wake_words.yaml` (внутри пакета → попадает
в share через setup.py). docker-volume `docker/vision/config/wake_words.yaml`
остаётся как **опциональный override** для production (на случай, если в проде
нужно пополнить operator namespace без пересборки).

Содержимое обоих файлов:

```yaml
# wake_words.yaml — override operator namespace для wake-роутера (ADR-0052).
# Секция `personality:` ЗАПРЕЩЕНА: SSoT личности —
# rob_box_voice.core.dialogue_text.DEFAULT_WAKE_WORDS
# (issue #2022, ADR-0052). Правка списка личности — в коде, не здесь.
#
# `operator:` — STT-искажения «ТАРС», наполняем по логам e2e (целевая §14.1).
# Без файла нода использует DEFAULT_OPERATOR_WAKE_WORDS = ("тарс", "tars").
#
# Путь до файла в docker-compose vision: монтируется ./config:/config:ro,
# параметр wake_words_file ноды указывает на /config/wake_words.yaml.
# Путь в colcon/bare-metal: $(ros2 pkg prefix rob_box_voice)/share/rob_box_voice/config/wake_words.yaml
operator:
  - тарс
  - tars
```

### 3.4. Тесты — `test_wake_words_config.py` переформулировать

Заменяем `test_personality_matches_code_defaults_byte_for_byte` на:

- `test_wake_words_yaml_must_not_declare_personality` — файл (и src-share, и
  docker-vision) **не должен** содержать ключ `personality:` верхнего уровня.
- `test_wake_words_yaml_operator_matches_code_default` — если `operator:` в
  YAML объявлен, он должен совпадать с `DEFAULT_OPERATOR_WAKE_WORDS` **или**
  быть его надмножеством (с сохранением порядка «существующие первые,
  расширения после»).
- `test_resolve_ignores_personality_section_in_yaml` — если YAML содержит
  `personality:`, `resolve_wake_word_namespaces` возвращает
  `(DEFAULT_WAKE_WORDS, ...)` — то есть personality берётся из кода.

### 3.5. Комментарии в per-node YAML — обновить формулировку

`src/rob_box_voice/config/stt_node.yaml:16-18` и
`docker/vision/config/voice_assistant/stt_node.yaml:13-18`:

```yaml
# wake_words намеренно НЕ здесь: список личности —
# rob_box_voice.core.dialogue_text.DEFAULT_WAKE_WORDS (SSoT, ADR-0052).
# Секция `personality:` в wake_words.yaml запрещена.
# wake_words_file указывает на override operator namespace (SSoT = код).
```

### 3.6. Dockerfile и docker-compose — без изменений

`./config:/config:ro` (docker/vision/docker-compose.yaml:252) уже
монтирует нужный путь. Docker-сборка через `COPY src/rob_box_voice/config` →
install/share → в ноду попадает fallback-файл из пакета. Если в
`/config/wake_words.yaml` есть файл — он перекрывает share-версию (это
ожидаемо: продовый override всегда приоритетнее пакета).

---

## 4. Trade-offs

- **Дублирование → нет.** `personality` больше не хранится в двух местах
  байт-в-байт. Источник регрессий #1252/#1734 закрыт структурно.
- **Достижимость YAML.** Файл попадает в share через `setup.py` →
  `colcon build` → `pytest`, bare-metal, CI. Docker-volume по-прежнему
  монтирует override.
- **Ответственность за правку.** Personality-список теперь правят в коде.
  Плюс: порядок виден в IDE, защищён type-checker'ом, тестом
  `test_node_defaults_use_the_canonical_wake_words` уже есть. Минус:
  правка требует PR + rebuild → deploy (а не только правки YAML в
  `/config/wake_words.yaml` на работающем роботе). Это сознательное
  ограничение: порядок списка — security-relevant.
- **Расширение operator namespace** в проде без пересборки — **сохраняется**:
  именно для этого YAML остался override-слоем для operator.
- **Backward-incompatible** для любого внешнего инструмента, который
  правит `personality` в YAML (если такие есть — не зафиксировано в коде).
  Деплой-инструкция должна явно сказать «personality в YAML игнорируется».

## 5. Альтернативы (рассмотренные и отклонённые)

- **Перенести YAML в `src/rob_box_voice/config/` без удаления `personality`.**
  Делает YAML достижимым из любого окружения, но **не устраняет дублирование**
  и источник расхождений #1252. Частичный фикс, отклонён.
- **Полностью удалить `wake_words.yaml`, оставить только `DEFAULT_*` в коде.**
  Ломает контракт wake-роутера для operator namespace: добавить STT-искажение
  «ТАРС» в проде можно только через пересборку (CI/deploy). Это противоречит
  §14.1 целевой архитектуры «наполняем по логам e2e». Отклонён.
- **Сделать `personality` YAML ключом `personality_override` (явный override)**
  с предупреждением «использовать только для аварийного rollback». Сохраняет
  сценарий «упал новый список — откатился по YAML», но вводит второй путь
  правки personality. Прямо противоречит доктрине «список — в коде».
  Отклонён.
- **Принять текущее состояние как есть (issue #2022 закрыть wontfix).**
  Означает признать «SSoT» двусмысленным термином и жить с расхождением
  между контурами. Прямо противоречит ADR-0018 (честный FAIL) и истории
  #1252/#1734. Отклонён.

---

## 6. План реализации (для `developer`)

1. `git mv docker/vision/config/wake_words.yaml src/rob_box_voice/config/wake_words.yaml`.
2. Удалить секцию `personality:` из обоих файлов (src-share + docker-vision);
   в docker-версии оставить `operator:` без изменений.
3. `setup.py` уже включает `glob('config/*.yaml')` — проверить, что
   `config/wake_words.yaml` попадает в share (после `colcon build`).
4. `core/dialogue_text.py::resolve_wake_word_namespaces` — добавить WARNING
   при `personality in loaded`, всегда возвращать `list(personality_fallback)`
   для personality.
5. `test/unit/core/test_wake_words_config.py` — заменить
   `test_personality_matches_code_defaults_byte_for_byte` на три теста из §3.4.
6. Комментарии в `src/rob_box_voice/config/{stt,dialogue}_node.yaml` и
   `docker/vision/config/voice_assistant/{stt,dialogue}_node.yaml` — обновить
   формулировку (см. §3.5).
7. README/deploy-doc: одна строка «personality в YAML игнорируется — править
   в `rob_box_voice.core.dialogue_text.DEFAULT_WAKE_WORDS`».

## 7. Definition of Done

- [ ] `find . -name wake_words.yaml -not -path './.git/*'` показывает **два**
      файла: `src/rob_box_voice/config/wake_words.yaml` (для share/colcon/bare-metal)
      **и** `docker/vision/config/wake_words.yaml` (для docker-volume override).
- [ ] Ни один из них **не содержит** ключа `personality:` (raw: `grep -L '^personality:' ...`).
- [ ] `colcon build --packages-select rob_box_voice` →
      `install/rob_box_voice/share/rob_box_voice/config/wake_words.yaml` существует.
- [ ] `pytest src/rob_box_voice/test/unit -q -k wake_words` зелёный:
      новые тесты из §3.4 проходят, `test_wake_word_sync` /
      `test_wake_words_are_declared_exactly_once` не сломаны.
- [ ] `grep -rn 'personality' docker/vision/config/wake_words.yaml src/rob_box_voice/config/wake_words.yaml` → пусто.
- [ ] В issue #2022 — комментарий со ссылкой на этот ADR.

---

## 8. Послесловие: где править, если завтра найдут новый STT-вариант

| Тип правки | Куда |
|---|---|
| Новый STT-вариант «роббокс» | `core/dialogue_text.py::DEFAULT_WAKE_WORDS` + тест `test_wake_word_sync::CANONICAL_WAKE_WORDS` |
| Новый STT-вариант «ТАРС» | `docker/vision/config/wake_words.yaml::operator` (override, **порядок важен**: новые после базовых) |
| Новый namespace (не личность) | YAML (как `operator`) + расширение `resolve_wake_word_namespaces` |
| Изменить wake-word gateway (regex, \b) | код `core/dialogue_text.py` (ни в коем случае не YAML — это контракт матчинга) |
