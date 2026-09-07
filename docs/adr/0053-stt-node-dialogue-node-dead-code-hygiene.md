# ADR-0053: Гигиена мёртвого кода после рефакторинга wake-роутинга (PR #2011, issue #2024)

| Поле | Значение |
|---|---|
| Статус | Proposed |
| Дата | 2026-09-07 |
| Автор | architect по карточке kanban `t_25eef7c1` (issue #2024 / #1990) |
| Контекст | После PR #2011 (wake-роутер в stt_node + SSoT wake-слов) в `src/rob_box_voice` осталось 5 однотипных «следов LLM-воркера»: дубль реализации, дубль импорта, противоречащие docstring'и, тесты мёртвого кода, бесполезный аргумент. CI их не ловит: фолбек под `pragma: no cover`, docstring'и — комментарии, тесты мёртвого кода зелёные по определению. Цель — зафиксировать минимальный набор правок и порядок их выполнения. |
| Затрагивает | `src/rob_box_voice/rob_box_voice/stt_node.py` (находки 1, 2), `src/rob_box_voice/rob_box_voice/dialogue_node.py` (3, 5), `src/rob_box_voice/test/unit/node/test_quest_stt_source.py` (4) |
| Родители | ADR-0051 §4 (явно перечисляет `voice_input_mode`, `_on_quest_stt`, `_publish_avatar_command_from_quest`, `_formalize_with_llm` и подписку `/voice/stt/quest` как удаляемое на шаге 6 миграции), ADR-0027 §3.4 (маршрутизация Quest-STT — контракт остаётся, физический путь мигрирует), ADR-0018 (культура честности: «никаких голословных PASS», «сын за отца не отвечает, но мёртвое удалять обязан»), ADR-0013 (incremental delivery, маленькие PR) |
| Связанные | issue #2024 (эта карточка), issue #1990 (PR #2011), `docs/architecture/target-operator-agent-and-dialogue.md` §7.3 «Что уходит», `docs/adr/0051-supervisor-operator-agent-arbiter-split.md` шаг 6 |

> **TL;DR.** Не проектируем ничего нового — только убираем то, что PR #2011
> оставил в коде по инерции. Правки механические, каждая ≤30 строк diff, можно
> идти одним PR (общий контекст — коммит `6e016325`) или разбить на два
> (функциональный + гигиена). Рекомендую один PR: правки слишком мелкие,
> плодить коммиты ради них — анти-pattern (ADR-0013).

---

## 1. Контекст и бизнес-проблема

PR #2011 (merge `6e016325`, issue #1990) сделал важную вещь: убрал
`/voice/stt/quest` из `dialogue_node` и перенёс маршрутизацию wake в `stt_node`
(SSoT `config/wake_words.yaml`). Но в процессе осталось пять артефактов
«недоделанной миграции», которые в совокупности — реальный риск расхождения
поведения и сигнал вопиющего самообмана:

| # | Файл:строка | Класс проблемы | Реальный риск |
|---|---|---|---|
| 1 | `stt_node.py:95-103` | дубль `strip_wake_word` в `except ImportError` | **расхождение семантики** (regex vs substring) |
| 2 | `stt_node.py:493, 553` | локальный `import time` при module-level импорте | мёртвые строки, оба стиля в одном файле |
| 3 | `dialogue_node.py:2774-2778, 2780-2789` | docstring описывает удалённый топик | вводит читателя в заблуждение |
| 4 | `test_quest_stt_source.py` (111 строк) | тесты мёртвого маршрута `_on_quest_stt` | ложный PASS, скрывает реальное состояние |
| 5 | `dialogue_node.py:96, 362-367` | `operator_fallback` передаётся и выбрасывается через `[0]` | мёртвый аргумент + бесполезный импорт |

CI пропускает всё: находка 1 — под `pragma: no cover`, находки 3-5 — не код,
а текст/тесты мёртвых путей (по определению зелёные). Это ровно та ситуация,
про которую ADR-0018: «PASS, который ничего не значит» хуже FAIL.

---

## 2. Решение

Один PR, пять логически независимых коммитов в одной ветке (squash на merge
опционально, ADR-0013 не запрещает серию мелких коммитов в PR). Каждое изменение
ниже — отдельный коммит с conventional-префиксом `chore`/`fix`/`docs`/`test`.

### 2.1 Дубль `strip_wake_word` в `stt_node.py` (находка 1)

**Файл:строка:** `src/rob_box_voice/rob_box_voice/stt_node.py:95-103`

**Проблема.** В `except ImportError` для импорта из `rob_box_voice.core.dialogue_text`
определён локальный `strip_wake_word`, который ищет подстроку через `in` +
`find`/`sorted(key=len)`. Канон в `dialogue_text.py:112-138` — regex по границам
слов `\b(...)\b` с `re.escape` и съедением пунктуации. Семантика **расходится**:

- Канон: «денчик ой фу робот меня зовут» → «денчик ой фу меня зовут» (regex \b не пропустит «робот» внутри «робот-меня», если бы такое было; в данном случае граница слова пропустит, потом вырежет целиком).
- Фолбек: вырежет первое вхождение longest-first, но **не проверит границу слова** — «робототехника» потеряет «робот», останется «техника».

В текущем проде фолбек недостижим (комментарий: `pragma: no cover — модуль всегда есть в нашем пакете`). Но это **та же логическая ошибка, что #1252 для списков wake-слов** — воркер не нашёл канон и написал своё. Плюс вторая копия алгоритма.

**Решение.** Удалить ветку `# pragma: no cover` полностью. `try/except ImportError`
оставить с минимальным фолбеком-сигналом (например, `from None` или
раннее оповещение в логе). Если `_HAS_WAKE_WORDS_AVAILABLE is False` — нода
падает на старте с понятной ошибкой, а не молча подменяет алгоритм.

**Конкретный код (после правки):**

```python
try:
    from rob_box_voice.core.dialogue_text import (
        DEFAULT_OPERATOR_WAKE_WORDS,
        DEFAULT_WAKE_WORDS,
        has_wake_word,
        resolve_wake_word_namespaces,
        strip_wake_word,
    )
except ImportError as exc:  # реально не должно случиться — это наш пакет
    raise RuntimeError(
        "rob_box_voice.core.dialogue_text недоступен — "
        "проверь colcon build / PYTHONPATH"
    ) from exc
```

Все 4 функции-фолбека (`:90-108`) удаляются вместе с константами `DEFAULT_WAKE_WORDS = ()` и т.д. Импорт `time` тоже уйдёт в этом коммите (см. 2.2).

**Альтернатива, которую НЕ берём.** Оставить фолбек, но сделать его делегатом
канона (вызов через importlib.util cached). Отвергнуто: усложнение ради
недостижимой ветки. KISS.

### 2.2 Лишние `import time` (находка 2)

**Файл:строки:** `stt_node.py:493` (`tts_state_callback`), `stt_node.py:553`
(`_process_audio`).

**Проблема.** `import time` на строке 20 (module-level) уже есть. Внутри функций
повторный импорт мёртв. При этом `_route_wake_result` на `:742` использует
module-level `time` — обе конвенции в одном файле.

**Решение.** Удалить оба локальных `import time`. Это чистый `refactor`:
поведение не меняется, только синтаксис.

**Альтернатива.** Вынести `time.monotonic()` в обёртку `_now()` и тестировать
через `monkeypatch`. Отвергнуто для этого PR — другой scope (тестируемость
callbacks), отдельная задача.

### 2.3 Устаревшие docstring'и в `dialogue_node.py` (находка 3)

**Файл:строки:** `dialogue_node.py:2774-2778` (ADR-0027 §3.4 про `/voice/stt/quest`),
`dialogue_node.py:2780-2789` (W3-1 ссылается на `_on_quest_stt` как на живой путь).

**Проблема.** Подписка `/voice/stt/quest` удалена на строке 594 («подписка на
/voice/stt/quest УДАЛЕНА»), `from_quest=True` теперь недостижим из ROS
(единственный публикатор пропал). Но комментарий через 2000 строк продолжает
описывать старый путь как рабочий. Это и есть «код лжёт о коде».

**Решение.** Переписать оба комментария как описание **текущего** поведения
(через `_on_stt` от `stt_node` → `/voice/stt/result`). Сослаться на тот же
ADR-0027 §3.4, но описать новый маршрут:

```python
# ADR-0027 §3.4 — Quest robot-voice (PTT): результат теперь приходит через
# общий /voice/stt/result с уже снятым wake-словом (см. stt_node:
# /audio/quest_in → /avatar/ptt/result, /audio/quest_wake → /avatar/stt/result
# после wake-роутинга в _process_audio, issue #1990). Параметр from_quest
# сохранён в сигнатуре _on_stt для совместимости с юнит-тестами и
# ADR-0051 §4 (шаг 6 миграции выпилит его вместе с voice_input_mode).
is_quest: bool = from_quest
```

Строки 2780-2789 — аналогично: убрать «приходит из _on_quest_stt при
voice_input_mode=quest_stt», оставить суть (W3-1 глушит только ReSpeaker,
Telegram и Quest-пути продолжают работать).

**Что НЕ делаем.** Не удаляем `from_quest=True` параметр — это отдельный
шаг по ADR-0051 §4 (вместе с выпиливанием `voice_input_mode`). Текущий PR —
про docstring'и, не про API.

### 2.4 Мёртвый тест `test_quest_stt_source.py` (находка 4)

**Файл:** `src/rob_box_voice/test/unit/node/test_quest_stt_source.py` (111 строк, 5 тестов).

**Проблема.** Все 5 тестов вызывают `_on_quest_stt`, подписка на который удалена.
Тесты зелёные, потому что и тестируемый код, и тесты — мёртвые. Это и есть
«красивый PASS, который ничего не значит» (ADR-0018).

**Решение.** **Удалить весь файл.** Не «починить тесты под новое поведение» —
нового поведения нет, `/voice/stt/quest` мёртв, и ADR-0051 §4 явно относит
его (и связанные `_on_quest_stt`, `_publish_avatar_command_from_quest`,
`_formalize_with_llm`) к удаляемому на шаге 6. Тащить 111 строк через шаг 6
нет смысла — это будет анти-pattern «оставим на потом».

**Альтернатива, которую НЕ берём.** Переписать тесты под `from_quest=True`
через `_on_stt`. Отвергнуто: фиксирует мёртвый параметр как живую часть
контракта и замедляет шаг 6 миграции. Плюс текущий PR не тестирует новый
путь (для нового пути нужен wake-роутер в `stt_node`, у него свои тесты —
`test_wake_word_sync` и т.д.).

**Комментарий в PR.** «Файл удалён: тестирует мёртвый маршрут. Замена при
шаге 6 ADR-0051 (новые тесты wake-роутинга stt_node уже есть, см.
`test_wake_word_sync`).»

### 2.5 Бесполезный `operator_fallback` в `dialogue_node.py` (находка 5)

**Файл:строки:** `dialogue_node.py:96` (импорт), `dialogue_node.py:362-367` (вызов).

**Проблема.** `resolve_wake_word_namespaces(...)` возвращает кортеж
`(personality_list, operator_list)`. `dialogue_node` берёт `[0]` и **выбрасывает**
operator. `DEFAULT_OPERATOR_WAKE_WORDS` импортируется только ради этого
мёртвого аргумента. `grep operator_wake_words` по `dialogue_node.py` — пусто.
Поведение не сломaно (operator — прерогатива `stt_node`), но код врёт о
намерении: «я, типа, тоже использую operator-namespace».

**Решение.** Два варианта:

(a) **Прокинуть operator в `self._operator_wake_words`** — но тогда его
  надо где-то **использовать** (например, для логов/метрик). Сейчас
  негде — `dialogue_node` не работает с микрофоном шлема после миграции. **Не берём (a)**.

(c) **Убрать аргумент `operator_fallback`** из вызова и убрать импорт
  `DEFAULT_OPERATOR_WAKE_WORDS`. Если `resolve_wake_word_namespaces`
  когда-нибудь понадобится с operator в `dialogue_node` — добавим явно,
  с использованием. Сейчас это YAGNI. **Берём (c).**

**Конкретный код (после правки):**

```python
from rob_box_voice.core.dialogue_text import (
    DEFAULT_WAKE_WORDS,
    has_wake_word,
    is_silence_command,
    is_unsilence_command,
    resolve_wake_word_namespaces,
    strip_wake_word,
)
# DEFAULT_OPERATOR_WAKE_WORDS удалён — dialogue_node работает только с
# ReSpeaker (operator-namespace обрабатывает stt_node, ADR-0051 §4).

self._wake_words: List[str] = list(
    resolve_wake_word_namespaces(
        str(self.get_parameter("wake_words_file").value or ""),
        personality_fallback=list(self.get_parameter("wake_words").value),
    )[0]
)
```

Сигнатура `resolve_wake_word_namespaces` уже имеет `operator_fallback` со
дефолтом `DEFAULT_OPERATOR_WAKE_WORDS` (`dialogue_text.py:183`) — вызов
без третьего аргумента остаётся валидным. Никакой обратной несовместимости.

**Что НЕ делаем.** Не удаляем `operator_fallback` параметр из самой
`resolve_wake_word_namespaces` — он нужен `stt_node` (там operator
реально используется в `_process_audio`).

---

## 3. Структура PR

Один PR, 5 коммитов в ветке `z-{developer}/<id>-stt-dialogue-dead-code-hygiene`
(base = `develop`). Коммиты:

```
chore(voice): удалить мёртвый except ImportError в stt_node (находка 1)
refactor(voice): убрать локальные `import time` в stt_node (находка 2)
docs(voice): обновить устаревшие docstring'и в _on_stt (находка 3)
test(voice): удалить test_quest_stt_source.py — тесты мёртвого маршрута (находка 4)
refactor(voice): убрать operator_fallback из resolve_wake_word_namespaces в dialogue_node (находка 5)
```

`chore` для первого, потому что **поведение не меняется** — падать на
отсутствующем пакете это «улучшение fail-loudness», не фича. `refactor` для
2 и 5 (поведение не меняется). `docs` для 3 (комментарии). `test` для 4
(удаление теста).

**Если reviewer против пяти коммитов:** squash в один `chore(voice): гигиена
после wake-роутинга (#2024)`. Оба варианта в рамках ADR-0013.

---

## 4. Инварианты и trade-off

### 4.1 Что НЕ делаем в этом PR (явные out-of-scope)

1. **Не выпиливаем `voice_input_mode`, `_on_quest_stt`, `_publish_avatar_command_from_quest`, `_formalize_with_llm`.** Это шаг 6 ADR-0051, отдельная задача. Здесь только гигиена.
2. **Не удаляем `from_quest=True` параметр `_on_stt`.** Тот же шаг 6.
3. **Не удаляем `operator_fallback` параметр `resolve_wake_word_namespaces`.** Он нужен `stt_node`. Удаляем только аргумент в **вызове** в `dialogue_node`.
5. **Не трогаем `config/wake_words.yaml`.** SSoT на месте.
6. **Не трогаем `test_wake_word_sync` и прочие живые тесты.** Они покрывают канон.

### 4.2 Trade-offs

| Решение | Плюс | Минус |
|---|---|---|
| один PR на 5 находок | меньше overhead, единый контекст | один коммит = много несвязанных правок (anti-pattern «scope creep») |
| удалить `test_quest_stt_source.py` целиком | честный FAIL лучше красивого PASS | 111 строк покрытия исчезают; замена — тесты wake-роутинга, уже есть |
| raise вместо фолбека в `except ImportError` | fail-loud, невозможно молча подменить алгоритм | падение при сломанной установке (но пакет наш, не должно случиться) |
| не выпиливать `from_quest` | ADR-0051 §4 делает это в шаге 6 | параметр-зомби живёт дольше |

### 4.3 Почему не «ничего не делать»

- **Находка 1** — реальная ловушка. Если кто-то однажды удалит пакет
  `rob_box_voice` (нереально, но) или соберёт ros-overlay (`colcon build
  --merge-install` с другим PYTHONPATH) — фолбек сработает и **молча**
  подменит семантику. Дубль алгоритма — это тот же класс, что #1252.
- **Находки 2, 3, 5** — гигиена, но это и есть та самая «мёртвая строкa,
  которая сбивает читателя». Читатель тратит время на то, чтобы понять
  «почему это здесь», и перестаёт доверять коду.
- **Находка 4** — главный аргумент. Тест зелёный **по определению**, потому
  что тестирует мёртвое. Если оставить — в следующий раз, когда кто-то
  будет искать «а покрыто ли это тестами?», он увидит зелёный PASS и
  успокоится. Это ровно то, против чего ADR-0018.

---

## 5. План валидации (для developer-а, который возьмёт карточку)

После PR, до `kanban complete`:

1. `pytest src/rob_box_voice/test/unit/node/test_wake_word_sync.py -v`
   — должен остаться зелёным (не задет).
2. `pytest src/rob_box_voice/test/unit/node/ -v` — НЕ должен упасть на
   удалённом файле (если pytest собирает по glob, проверить). Если есть
   ссылка на `test_quest_stt_source` где-то ещё — почистить.
3. `python -c "from rob_box_voice.stt_node import strip_wake_word; print(strip_wake_word('робот, привет', ['робот']))"`
   — должно вывести «привет» (канон из dialogue_text).
4. `grep -n "import time" src/rob_box_voice/rob_box_voice/stt_node.py`
   — должно показать ТОЛЬКО строку 20 (module-level).
5. `grep -n "DEFAULT_OPERATOR_WAKE_WORDS\|operator_wake_words" src/rob_box_voice/rob_box_voice/dialogue_node.py`
   — должно быть пусто.
6. CI на PR (develop-base): ожидаемо зелёный.

Если CI падает — это не regression от наших правок, а pre-existing
проблема. Разбираться через `pr-reviewer` или `devops`.

---

## 6. Связанные карточки и handoff

- **Issue #2024** — родительская (эта карточка).
- **Issue #1990** — PR #2011, откуда весь мусор.
- **Шаг 6 ADR-0051** — следующая (после merge этого). Отдельная карточка,
  не эта.
- **e2e-тест**: не нужен. Все находки — статический код, не голос.
  Голосовой wake-роутинг покрыт существующими `test_wake_word_sync` +
  e2e-процессом (если уже настроен под #1990).

---

> *«Мёртвое удалять обязан. Иначе оно начинает жить своей жизнью и
> врать всем, кто его читает.»*
> (наказ товарища Шифу, 18.08.2026, ADR-0018)