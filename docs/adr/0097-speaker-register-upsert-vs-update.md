# ADR-0097: `SpeakerDatabase.register()` — UPSERT-семантика для нового профиля, UPDATE для существующего

- **Status:** proposed (review-вердикт от architect; реализация — child-карта `t_<next>` для backend)
- **Date:** 2026-09-15
- **Issue:** [#2469](https://github.com/krikz/rob_box_project/issues/2469)
- **Review window:** `src/rob_box_voice`, 2026-09-14
- **Source card:** `t_3802c39a`
- **Affected file:** `src/rob_box_voice/rob_box_voice/utils/speaker_embeddings.py:434–443`
- **Related:** issue #1787 (эпитеты), issue #2348 / #1101 (валидация имени), commit `12f4d22c`, PR #2370

---

## 1. Контекст и корневая причина

`SpeakerDatabase.register(name, embedding, speaker_id=None)` — низкоуровневый API
голосовой биометрии. На входе два сценария:

| `speaker_id` | Что должно происходить |
|---|---|
| `None` | Создать **новый** профиль (uuid + INSERT) |
| явно передан | Дописать ещё один эмбеддинг в **существующий** профиль (UPDATE-семантика: обновить имя, не трогая остальное) |

Текущий код (строки 432–443):

```python
if speaker_id is None:
    speaker_id = str(uuid.uuid4())
    self._conn.execute(
        "INSERT OR IGNORE INTO speakers (speaker_id, name, created_at) VALUES (?, ?, ?)",
        (speaker_id, clean_name, now),
    )
else:
    # Update name in case it changed
    self._conn.execute(
        "INSERT OR REPLACE INTO speakers (speaker_id, name, created_at) VALUES (?, ?, ?)",
        (speaker_id, clean_name, now),
    )
```

В обеих ветках используется «upsert», но с разной семантикой:

- `INSERT OR IGNORE` для нового — корректно (новый uuid не может конфликтовать, но формально это защита от гонки).
- **`INSERT OR REPLACE` для существующего — НЕкорректно.** В SQLite это эквивалент
  `DELETE` старой строки + `INSERT` новой. Все колонки, не упомянутые в
  `VALUES (...)`, получают значение по умолчанию (`NULL`/`0`/etc.).

После миграции `_migrate_epithet_columns` (issue #1787) в таблице появились 5
nullable-колонок:

```python
_EPITHET_COLUMNS = (
    ("epithet",            "TEXT"),
    ("epithet_history",    "TEXT"),
    ("tags",               "TEXT"),
    ("sentiment_score",    "REAL"),
    ("last_epithet_review","REAL"),
)
```

Каждый вызов `register(..., speaker_id=<existing>)` (типичный путь — через
`register_or_merge()`, который на повторной фразе того же голоса матчит уже
существующий `speaker_id` и дописывает эмбеддинг) **молча обнуляет** все 5
колонок:

- `epithet` (текущая кличка) — теряется
- `epithet_history` (JSON-история кличек) — **катастрофически**, теряется аудит-трейл
- `tags` (CSV тем речи) — теряются
- `sentiment_score` — теряется
- `last_epithet_review` — теряется (а это таймер антидребезга пересмотра эпитета)

Дополнительно `created_at` тоже перезаписывается на `now`, что ломает
семантику («время создания профиля» ≠ «время последней регистрации эмбеддинга»).

**Severity: medium.** Данные теряются в тихом режиме: ни лог-сообщения, ни
исключения, ни мониторинг. Эпитет мог быть выдан LLM минуту назад — а
следующая реплика того же спикера его обнулит. Для issue #1787 это фундамент
функционала (разноценки тёзок).

### 1.1. Почему баг прошёл незамеченным

1. Миграция `_migrate_epithet_columns` (issue #1787) добавляла колонки в
   уже существующую таблицу. Тестовая БД тестов `test_speaker_embeddings.py`
   создаётся заново через `_CREATE_SQL` (без эпитет-колонок — они появятся
   только после миграции, которая тоже сработает, но `register()` обнулит
   их в первом же вызове — тестам это без разницы).
2. Тесты `register()` проверяют happy-path и валидацию имени (issue #2348),
   но **не проверяют сохранность метаданных профиля** при повторной
   регистрации. Такого теста нет в `test_speaker_embeddings.py` и в
   `test_epithets.py`.
3. `register_or_merge()`-тесты тоже проверяют только `speaker_id` и
   `reused`-флаг, не `get_speaker_profile(sid)['epithet']` после повторной
   регистрации.

## 2. Решение

**Заменить `INSERT OR REPLACE` на idempotent UPDATE с условным INSERT-ом
новой строки через `ON CONFLICT DO NOTHING`.** SQLite поддерживает
upsert-форму с явным `ON CONFLICT(...) DO NOTHING`, которая НЕ перезаписывает
строку, а лишь говорит «если есть конфликт по PK — пропусти».

### 2.1. Целевой код

```python
if speaker_id is None:
    speaker_id = str(uuid.uuid4())
    self._conn.execute(
        "INSERT INTO speakers (speaker_id, name, created_at) VALUES (?, ?, ?) "
        "ON CONFLICT(speaker_id) DO NOTHING",
        (speaker_id, clean_name, now),
    )
else:
    # Update name in case it changed; preserve epithet/tags/etc.
    self._conn.execute(
        "UPDATE speakers SET name=? WHERE speaker_id=?",
        (clean_name, speaker_id),
    )
```

### 2.2. Что НЕ делаем в этом фиксе

| Колонка | Поведение в фиксе | Почему |
|---|---|---|
| `name` | UPDATE на повторной регистрации | документированный rename-путь (`register_or_merge` → `register(speaker_id=...)` для уже опознанного голоса) |
| `created_at` | **НЕ трогаем** | семантика «время создания профиля», а не «время последнего апдейта». Для аудита «когда этот человек впервые появился в системе». |
| `epithet`/`epithet_history`/`tags`/`sentiment_score`/`last_epithet_review` | **НЕ трогаем** | это метаданные профиля, управляются отдельным API (`set_epithet`, `update_speaker_stats`); `register()` о них не знает и не должен знать |

### 2.3. Альтернативы, рассмотренные и отклонённые

**A. Добавить колонку `updated_at REAL NOT NULL DEFAULT 0` в `_CREATE_SQL` + миграцию.**
- *За:* отслеживание «когда последний раз регистрировали эмбеддинг» полезно для
  оператора (найти забытых спикеров).
- *Против:* это новый API-контракт, который не запрашивался. `created_at`
  сейчас нигде не используется как «когда создан профиль vs когда дописан
  эмбеддинг» — нет ни одной сигнатуры, которая бы на это полагалась.
  Добавлять колонку «на всякий случай» = менять схему по принципу YAGNI.
- **Решение:** отложить в отдельный тикет, если появится реальный
  потребитель. В этом фиксе — НЕ делать.

**B. Заменить `INSERT OR REPLACE` на `INSERT ... ON CONFLICT DO UPDATE SET name=?`.**
- *За:* одна SQL-операция, идемпотентна.
- *Против:* поведение `name=` перезапишет `name` даже в случае, когда мы
  добавляем ЭМБЕДДИНГ в существующий профиль с тем же именем (это OK), но
  смешивает две разные семантики в одной конструкции. Труднее читать,
  труднее регрессировать.
- *Против:* семантически эквивалентно варианту A с `updated_at`, но без
  колонки. Не даёт преимуществ над чистым UPDATE.
- **Решение:** отклонено в пользу явного UPDATE для существующего профиля +
  INSERT для нового (легче аудировать).

**C. Заменить `INSERT OR REPLACE` на чистый `UPDATE` без INSERT-fallback.**
- *За:* самый явный код.
- *Против:* теряется симметрия с веткой `speaker_id is None`. Если кто-то
  однажды передаст `speaker_id` для ещё-не-существующего профиля (например,
  в тестах или в миграционных скриптах), UPDATE ничего не сделает и
  `register()` ругнётся на FK при попытке вставить эмбеддинг в
  `embeddings(speaker_id REFERENCES speakers)`. UPDATE не создаёт строку.
- *Против:* менее устойчиво к поломкам upstream-вызовов.
- **Решение:** отклонено. Но **в нашем конкретном коде это безопасно** —
  `register_or_merge()` вызывает `register(..., speaker_id=match.speaker_id)`
  только после успешного `identify()` (т.е. строка гарантированно есть).
  Оставляем ветку с `ON CONFLICT DO NOTHING` для нового профиля, чтобы
  сохранить идемпотентность и устойчивость к гонкам.

## 3. Trade-offs

| Решение | Плюс | Минус |
|---|---|---|
| `UPDATE speakers SET name=? WHERE speaker_id=?` (целевое) | Явная семантика «обновление», 0 риск перезаписи других колонок, читается однозначно | Две SQL-операции в худшем случае (но в нашем случае — одна для `speaker_id!=None`) |
| `INSERT ... ON CONFLICT DO NOTHING` для нового (целевое) | Идемпотентность, гонкоустойчивость | Чуть менее привычный синтаксис, чем `INSERT OR IGNORE` |
| Миграция схемы | — | Не нужна: `_CREATE_SQL` уже корректен, миграция `_migrate_epithet_columns` уже работает |

**Что получит проект:**

1. Данные эпитета/тегов/валентности перестают теряться при повторной регистрации.
2. `created_at` остаётся «временем первого появления» — корректная семантика для аудита.
3. Тесты ловят регрессию: добавленный test на «register с явным speaker_id не обнуляет epithet» будет зелёным.

**Что проект НЕ получит:**

- Колонку `updated_at`. Не запрашивалась, не нужна прямо сейчас.
- Миграцию существующих повреждённых БД (где эпитет уже затёрт). Это отдельная задача (см. §6).

## 4. Совместимость и обратная совместимость

- **API наружу:** без изменений. `register(name, embedding, speaker_id=None)`
  возвращает тот же `speaker_id`, поведение для нового профиля идентично,
  для существующего — фиксится баг.
- **DB schema:** без изменений. Никаких миграций не требуется — фикс
  только в семантике SQL.
- **Backward compat для существующих БД на роботе:**
  - До фикса: эпитет мог быть стёрт предыдущими вызовами register.
  - После фикса: эпитет перестаёт стираться, но уже стёртые — не
    восстанавливаются автоматически.
  - Если эпитеты реально были потеряны в проде — Шифу/оператору надо
    перезапустить epithet-cycle (см. §6, child-задача).
- **Тесты:** существующие тесты остаются зелёными (поведение `register()` для
  нового профиля не меняется). Новый test добавляется — см. §5.

## 5. Acceptance criteria для child-карты backend

1. **Фикс SQL:** `register()` в `speaker_embeddings.py` использует UPDATE
   для существующего `speaker_id` и `INSERT ... ON CONFLICT DO NOTHING` для
   нового. Поведение для `created_at`, `epithet`, `epithet_history`,
   `tags`, `sentiment_score`, `last_epithet_review` — не меняется при
   повторной регистрации.

2. **Регрессионный тест (юнит, `test_speaker_embeddings.py`):**
   ```python
   def test_register_with_explicit_id_preserves_epithet_and_metadata(self, db):
       """Issue #2469 — register() не должен стирать эпитет/теги/историю
       при повторной регистрации в существующий профиль."""
       sid = db.register("Иван", _random_embedding(1))
       db.set_epithet(sid, "Гроссмейстер", reason="llm_assigned")
       db.update_speaker_stats(sid, tags=["шахматы"], sentiment_score=0.3)

       # Повторная регистрация того же голоса (типичный путь через
       # register_or_merge на 2-й фразе).
       db.register("Иван", _random_embedding(2), speaker_id=sid)

       profile = db.get_speaker_profile(sid)
       assert profile["epithet"] == "Гроссмейстер"
       assert profile["tags"] == ["шахматы"]
       assert profile["sentiment_score"] == 0.3
       assert profile["epithet_history"]  # не пустой, не стёрт
       # created_at — НЕ перезаписан
       # (assert на конкретное значение — отдельный sub-test)
   ```

3. **Регрессионный тест на `created_at`:**
   ```python
   def test_register_with_explicit_id_does_not_overwrite_created_at(self, db):
       sid = db.register("Иван", _random_embedding(1))
       before = db.get_speaker_profile(sid)["created_at"]
       time.sleep(0.05)  # гарантируем различимые timestamp'ы
       db.register("Иван", _random_embedding(2), speaker_id=sid)
       after = db.get_speaker_profile(sid)["created_at"]
       assert before == pytest.approx(after, abs=1e-3)
   ```

4. **Все существующие тесты** `test_speaker_embeddings.py` +
   `test_epithets.py` + `test_speaker_id_node.py` (если есть) +
   `test_identity_seam.py` (если есть) — зелёные.

5. **Поведение `register_or_merge()` не меняется** по
   контракту (он по-прежнему возвращает `(speaker_id, reused)`). Но после
   фикса — эпитет перестаёт стираться при merge-пути. Это и есть цель
   фикса.

6. **Лог-сообщение `Registered speaker ...`** — без изменений (для оператора
   не нужна новая «warn о перезаписи эпитета» — её теперь и нет).

7. **Diff минимальный:** только строки 432–443 в `speaker_embeddings.py` +
   новые test-кейсы в `test_speaker_embeddings.py`. Никаких изменений
   схемы, миграций, других модулей.

## 6. Что НЕ входит в этот ADR (out of scope)

- **Восстановление уже затёртых эпитетов в прод-БД** (`/data/speakers.db`
  на роботе). Это отдельный issue/таск — нужен либо ручной re-assign
  через `dialogue.epithets_node`, либо одноразовая миграция, читающая
  последний доступный эпитет из `MemoryStore` (если он там есть) или из
  LLM-лога. **Не блокер для merge фикса** — после фикса потери
  прекращаются, реставрация делается отдельно.
- **Добавление `updated_at` колонки.** См. §2.3.A.
- **Рефакторинг `register_or_merge()`** в сторону чистого `register()`-без-
  rename. Текущий контракт (повторная регистрация обновляет `name`) —
  документирован и используется `speaker_id_node`.

## 7. Решение (резюме)

**Принять вариант «UPDATE для существующего, ON CONFLICT DO NOTHING для
нового».** Минимальный diff, явная семантика, ноль миграций, починка
тихого data loss.

Child-карта для backend (assignee=backend, parent=t_3802c39a) — отдельный
тикет с телом из §5.
