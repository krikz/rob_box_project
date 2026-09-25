# ADR-0135: Лицо↔голос — мост идентичности в шве «Знакомый» (закрывает #3024)

| Поле | Значение |
|---|---|
| Статус | **Proposed** (дизайн к ревью товарища Шифу; реализация — отдельная карточка, после ревью) |
| Дата | 2026-09-25 |
| Автор | architect (Hermes Agent), kanban `t_ee583f20` |
| Решает | issue #3024 — «Робот поздоровался по имени (лицо), а через 26 сек спрашивает „Дэнчик, это ты?" — лицо и голос не сшиты» |
| Заменяет | — |
| Родители | ADR-0089 §R6, ADR-0102 §3 (повод), **ADR-0105** («Встреча»), **ADR-0106** («Знакомый»), **ADR-0123** §6 (лицевые моды и arbitration) |
| Связанные | issue #3024, #2599 (PR-C лицо, влит), #2809/#2888 (tentative-спикер), #2771/#2828 (ак-гейт), ADR-0131 (один источник истины спикера на utterance), ADR-0069 (ADR-коллизия inflight-check) |

> **TL;DR.** Лицо (Vision Pi) и голос (Main Pi) сегодня дают **разные стабильные UUID** одного человека: `person_id=4ff0ddc5` (ArcFace-эмбеддинг, `FaceStore`) и `biometric_uuid=c9e981cb` (resemblyzer-эмбеддинг, `SpeakerDatabase`). Оба — стабильные между сессиями, но оба ключа пишут свои `name`/`epithet`/`last_seen` отдельно, и **моста между ними нет**. Поэтому `_handle_meeting` (Vision) через 26 сек обнуляется голосовым `_handle_tentative_speaker`, который не знает, что лицо уже подтвердило личность. Решение: расширяем шов «Знакомый» (ADR-0106) третьим адаптером — VisionFace, и вводим явный сигнал `face-to-voice hint` через тот же `IdentitySeam`, по которому ходит голос. Мост не сшивает UUID'ы «один-к-одному» (это решит Phase 2 / ADR-0123 §6), а решает **только** приёмку #3024: «лицо уже подтвердило → голос не переспрашивает». Деградация к текущему поведению — дефолт, поведение включается флагом `face_voice_hint_enabled: bool` в `dialogue_node.yaml`.

---

## 1. Контекст и бизнес-проблема

### 1.1 Сценарий-заказчик (доказательная база — issue #3024)

`Vision Pi 10.1.1.21, 2026-09-25 ~14:00 MSK`:

1. `14:00:15` — Vision (`vision_face` → `meeting_node`): `person_id=4ff0ddc5 sim=0.812 new=False встреч=27 name='Дэнчик'`. Robot здоровается по имени → `"Дэнчик! Давно тебя не видел. Добрый день."`.
2. `14:00:40` — пользователь отвечает `"привет"`.
3. `14:00:41` — Voice (`speaker_id_node` → `/voice/speaker/result`): `best='Дэнчик'(c9e981cb) score=0.791`, **полоса `single` (`[0.72, 0.80)`)** → имя подавлено (`name=None epithet='Незнакомец' (name suppressed: single, issue #2809)`).
4. `14:00:42` — `dialogue_node` ставит `[Speaker:tentative]`, `<name>unknown</name>` в `system_context`, и по #2888 задаёт встречный вопрос: `"Дэнчик, это ты?"`.

**UX-дефект.** Робот уже сказал «Добрый день, Дэнчик» 26 сек назад. Сейчас же — спрашивает «Дэнчик, это ты?». Это разрушает доверие (робот делает вид, что не помнит, что только что подтвердил идентичность) и конфликтует с ADR-0096 §6 «нет противоречия каналов».

### 1.2 Корневая причина (архитектурный разбор)

**Лицо и голос — два независимых стейта идентичности**, между ними:
- **Нет общего ключа.** Лицо пишет `Acquaintance.id = person_id (4ff0ddc5)` в `FaceStore` (`/data/faces/`). Голос пишет `Acquaintance.id = biometric_uuid (c9e981cb)` в `SpeakerDatabase` (`speakers.db`). Оба — UUIDv4, оба стабильные.
- **Нет моста.** `IdentitySeam.merge(src_id, dst_id)` существует (ADR-0106 §3.2), но у него нет операций `link_face_to_speaker`, `note_face_seen`, и нет колбэка на `/perception/face` в `dialogue_node`. Лицо никогда не доходит до шва (см. ADR-0123 §6 — arbitration «голос+лицо» явно отложена «на отдельную карточку»).
- **Нет состояния «мы только что видели».** `_handle_meeting` (Vision) **не пишет** ничего в общий кеш «текущий собеседник, как опознанный лицом» — только здоровается и забывает. Поэтому `_handle_tentative_speaker` (Voice), приходя с 26-секундным опозданием, вынужден строить идентичность **только из голоса** — и при score=0.791 спрашивать.

### 1.3 Что НЕ предлагаем

- **Не предлагаем «слить person_id и biometric_uuid в один UUID».** Это технически неверно: ArcFace и resemblyzer оперируют в **разных пространствах эмбеддингов**, и физически разные люди дают разные пары. Система, привязывающая их вручную, ошибается при первой же смене причёски (см. ADR-0123 §6 п.1).
- **Не предлагаем убрать переспрос #2809 в принципе.** `#2809/#2888` фиксируют корректное правило для **голосовой**-only сессии (холодный старт, без лица), когда голос — единственный канал. Вопрос **не должен** задаваться, только если есть свежий внешний сигнал «этот человек уже подтверждён».
- **Не предлагаем немедленно реализовать полную arbitration ADR-0123 §6** (merge лицо+голос через embedding similarity). Это нетривиальный ML-контур (Phase 2), явно отложен владельцем в ADR-0123 §9 п.3 как отдельная карточка. Здесь закрываем **только** #3024 — UX-симптом «голос переспрашивает, когда лицо уже подтвердило».

---

## 2. Решение

### 2.1 Расширение шва «Знакомый» третьим сигналом `FaceSignal`

В `src/rob_box_harness/rob_box_harness/identity/seam.py` (ADR-0106 §3.2) — три существующих адаптера становятся **внешним контрактом сигнала**:

```
Signal = VoiceSignal | FaceSignal   # pseudo-type; реально — два dataclass'а
```

```python
# FaceSignal: что приходит из vision_face_node через /perception/face (или
# эквивалентный топик — см. §5.2). Структурно-типизирован, без зависимости
# от rob_box_perception в шов.
@dataclass(frozen=True)
class FaceSignal:
    person_id: str       # UUID из FaceStore
    name: Optional[str]  # имя, если FaceStore его уже знает
    similarity: float    # 0..1, score ArcFace-матча
    is_new: bool         # новая запись (created_at == сейчас) или старая
    source_camera: str
    captured_at: float   # unix-time; для freshness-окна
```

### 2.2 Операция `note_face_seen(signal, *, now=None) -> FaceObservation`

В `IdentitySeam` добавляется **без побочного эффекта записи** в долговременный стор. Это **наблюдение**, не «обновление профиля» — оно нужно в `dialogue_node` (и любому другому потребителю) как подсказка «лицо только что видело этого человека с уверенностью X».

```python
@dataclass(frozen=True)
class FaceObservation:
    person_id: str
    name: Optional[str]
    similarity: float
    captured_at: float
    is_new: bool
    confidence_band: str   # "high" | "tentative" | "low"; см. §2.4

    @property
    def age_sec(self, *, now=None) -> float:
        return (now or time.time()) - self.captured_at

    def is_recent(self, *, window_sec=30, now=None) -> bool:
        # Дефолтное freshness-окно — 30 сек (см. §2.4). Конфигурируется через
        # YAML `dialogue_node.yaml::face_voice_hint.window_sec`.
        return self.age_sec(now=now) <= window_sec
```

Семантика — **кольцевой буфер in-memory, per-acquaintance**, последние N наблюдений:
- `N = 8` (достаточно, чтобы перекрыть многорепликовый диалог в 60-90 сек);
- ключ = `person_id` (а не voice-biometric_uuid — потому что человек-человек ещё не сшиты, см. §1.3);
- TTL записи = `window_sec` (дефолт 30 сек); за пределами TTL — запись считается протухшей и помечается в `is_recent() = False`.

**Критерий «не писать профиль».** Запись в шов — это **наблюдение**, она **не меняет** `Acquaintance.name`/`epithet`/`last_seen`. Задача операции — дать `dialogue_node` факт «лицо только что увидело этого кандидата», не более. Это важно для деградации к ADR-0106 §3.4 (источник `name` остаётся голос или прямое именование через #2925, не лицо через мост — пока владелец не сделает полную arbitration).

### 2.3 Подписка `dialogue_node` на `/perception/face`

В `dialogue_node._setup_subscriptions` (там, где сейчас создаются подписки на `/voice/speaker/result`, `~929-1000` строки — см. ADR-0131 §3.1 про общую картину) добавляется:

```python
# Issue #3024 / ADR-0135: bridge face → voice.
# Колбэк дешёвый: только кладёт в in-memory кольцевой буфер IdentitySeam.
self.create_subscription(
    String, "/perception/face/meeting",
    self._on_face_meeting, qos_r,
)
```

Колбэк `_on_face_meeting(msg)`:
1. Парсит payload как JSON (`{"person_id": "...", "name": "...", "similarity": 0.81, ...}`).
2. Преобразует в `FaceSignal` через dataclass-конструктор.
3. Вызывает `self._identity.note_face_seen(signal)`.
4. Логирует уровня DEBUG (не INFO — на 5 событий/сек у одного человека, см. ADR-0123 §3, лог уровня INFO утопит `docker logs voice-assistant`).

### 2.4 Использование hint в `_handle_tentative_speaker`

Перед блоком переспроса (т.е. до `state["asked"] = True` в `dialogue_node._handle_tentative_speaker`, ~line 3927) добавляется **короткий** look-up:

```python
# Issue #3024 / ADR-0135: face уже увидело человека с именем N — голос
# не должен задавать «<Имя>, это ты?» в этом окне. Селект только по
# сигналу, который мы сами положили в шов.
face_obs = (
    self._identity.recent_face_observation(full_sid)
    if full_sid else None
)
if (
    face_obs is not None
    and face_obs.is_recent(window_sec=self._face_voice_hint_window)
    and face_obs.confidence_band == "high"
    and (tentative_name is None or face_obs.name == tentative_name)
):
    # Лицо уже подтвердило — фиксируем голос как confirmation, не
    # как registration. Идём в confirmation path тем же способом,
    # как если бы человек словесно ответил «да» (#2809).
    state["asked"] = True
    state["confirmed"] = True
    state["name"] = tentative_name or face_obs.name
    self.get_logger().info(
        f"👤 [issue #3024 ADR-0135] voice tentative suppressed by recent "
        f"face hint ({face_obs.name}, age={face_obs.age_sec:.1f}s, "
        f"sim={face_obs.similarity:.3f}); confirming as {state['name']!r}"
    )
    return self._confirm_tentative_speaker(
        full_sid, state["name"], user_input, utterance_id,
    )
```

**Полосы уверенности** (`confidence_band`) пороги — дефолты в конфиге:

| Полоса | Диапазон `similarity` | Поведение |
|---|---|---|
| `high` | `>= 0.78` | Hint отменяет голосовой переспрос #2809 |
| `tentative` | `[0.65, 0.78)` | Hint присутствует в логе и `system_context`, но **не** отменяет переспрос |
| `low` | `< 0.65` | Hint игнорируется (стаб-фильтр зеркалит ADR-0089 §2.2) |

Дефолт `0.78` взят из ADR-0123 §6 «`face_identify_threshold` калибруется на реальных данных», но для hint достаточно менее строгого порога: hint не выдаёт имени, он только снимает чужой переспрос, который и так при низкой уверенности задан быть не должен.

### 2.5 Деградация к текущему поведению

- **`/perception/face/meeting` недоступен** (Vision Pi не поднят, VisionEvent'ы только stub'ы) → `note_face_seen` никем не зовётся, `recent_face_observation()` возвращает `None`, поведение = текущее.
- **`similarity < 0.65`** → `confidence_band="low"`, hint игнорируется, поведение = текущее.
- **`face_voice_hint_enabled: false`** (дефолт `true`, см. §2.6) → подписка не создаётся, поведение = текущее.

Таким образом **самый медленный rollout** — включить флаг, дождаться стабильного `docker logs vision-face` на неделю, потом открыть подписку.

### 2.6 Конфигурация

`docker/vision/config/voice_assistant/dialogue_node.yaml` (и зеркало в `src/rob_box_voice/config/`):

```yaml
face_voice_hint:
  enabled: true                       # feature flag, дефолт true (см. §2.5)
  window_sec: 30                      # freshness окно; см. §2.2
  similarity_high_threshold: 0.78     # граница полос "high" | "tentative"
  similarity_low_threshold: 0.65      # граница полос "tentative" | "low" (стаб-зеркало)
  buffer_capacity: 8                  # per-person ring (см. §2.2)
```

Все четыре значения синхронизируются через `test_yaml_param_consistency.py` уже после первой реализации (см. ADR-0106 §0 — расширение проверкой **значений**).

---

## 3. Что остаётся сделать (следующие инкременты для #3024 → PR)

1. **Расширение `IdentitySeam`** операцией `note_face_seen` + `recent_face_observation` (in-memory ring в `base.py`). Тесты — в `src/rob_box_harness/test/test_identity_seam_face_hint.py`. ~0.5 дня.
2. **Колбэк `_on_face_meeting` в `dialogue_node`** + регистрация подписки через `YAML::face_voice_hint`. Тест интеграционный — мокаем `/perception/face/meeting` сообщением и проверяем, что `_handle_tentative_speaker` идёт в confirmation path вместо переспроса. ~0.5 дня.
3. **Расширение `test_yaml_param_consistency.py`** проверкой значений четырёх новых YAML-ключей (защита от регрессии ADR-0106 §2.1 «калибровка порогов не доехала»). ~0.25 дня.
4. **Acceptance-тест (юнит) на #3024-сценарий:**
   - `dialogue_node` получает `/voice/speaker/result` с `name='Дэнчик', score=0.79, full_sid='c9e981cb'`;
   - затем получает `/perception/face/meeting` с `name='Дэнчик', person_id='4ff0ddc5', similarity=0.81` (в пределах `window_sec`);
   - **в течение 30 сек** приходит голосовая реплика → подтверждение **без** переспроса «Дэнчик, это ты?».

   Эта пара: тест должен **красным** проходить на текущем коде и **зелёным** — после реализации.
5. **(опционально)** unit-тест «лицо вне окна`window_sec` → голос переспрашивает как раньше» — регрессионная защита от #3024 false-fix.
6. **`docs/architecture/identity-seam.md`** (новый файл, ADR-style) — обновить список адаптеров с двух (голос) до трёх (голос, лицо). Без этого новый контрибьютор не найдёт шов.

Минимум для PR — пункты 1, 2, 3, 4. Пункты 5 и 6 — в том же PR или сразу за ним (по объёму ≤ 300 строк, правило ADR-0013 соблюсти).

---

## 4. Чего этот ADR НЕ делает

| Тема | Почему НЕ здесь |
|---|---|
| Полная arbitration лицо+голос через embedding similarity | Это ADR-0123 §6, Phase 2, явно отложенный владельцем (см. ADR-0123 §9 п.3) |
| Замена голосового переспроса в принципе | #2809/#2888 фиксируют корректное правило для voice-only сессии; этот ADR их не отменяет, а сужает |
| Хранение лиц (FaceStore policy / режимы privacy) | ADR-0123 §5 |
| Удаление защит #2809 от `must_not_say` в `contested` | `n210` логика остаётся без изменений — hint молчит, если voice пришёл с `contested` (см. §2.4 условие `tentative_name is None or ...`) |
| Сшивка person_id ↔ biometric_uuid в один UUID | Технически неверно (разные embedding-пространства, см. §1.3) |

---

## 5. Альтернативы (отвергнутые)

### 5.A — «Лицо пишет имя прямо в `<system_context>`»

**Идея.** Vision шлёт `name` в `system_context` напрямую, голос его читает как есть.

**Почему плохо.** Обходит шов идентичности, плодит дубль — `Acquaintance.name` берётся из одного места, `<system_context>/<name>` — из другого. Любой будущий потребитель (MCP-тул `memory_save`, UI-логи) увидит рассогласование, если у лица и голоса разные имена для одного человека (ребёнок Дениса назвал не себя). ADR-0106 §1.1 явно зафиксировал корень проблемы — множественные несвязанные источники идентичности.

### 5.B — «Сшить person_id и biometric_uuid маппингом при первой коллизии»

**Идея.** Раз пришёл face c `4ff0ddc5` + voice c `c9e981cb` за последний час и оба имени `'Дэнчик'` → слить.

**Почему плохо.** Два разных embedding-пространства (ArcFace 512-d / resemblyzer 256-d), одинаковые имена у разных людей (тёзки), и физическое расхождение при смене причёски. ADR-0123 §6 фиксирует это как «открытый вопрос», не как задачу одного PR. Плюс ломает уже работающие тесты ADR-0106 §8 (acceptance-тест).

### 5.C — «Снизить порог переспроса до 0.7 (score=0.791 > 0.7)»

**Идея.** Расширить «confident»-полосу вниз — голос бы не задавал вопрос при 0.791.

**Почему плохо.** Это **выключает** защиту #2809 для всех остальных сценариев (шумный зал, чужой телефон с похожим голосом, двойник). Face-only-hint узкий и безопасен, потому что акт-источник (лицо) — нейронка с собственным калиброванным порогом.

### 5.D — «Подождать полную реализацию ADR-0123 §6»

**Идея.** Сделать всё разом — голос + лицо через единственный arbitration.

**Почему плохо.** Это Phase 2, владелец в ADR-0123 §9 явно отложил её как отдельную карточку. #3024 не требует arbitration — он требует **снять переспрос**, когда **внешний** сигнал (лицо) уже подтвердил. Hint **строго у́же**: он не пытается принять решение «кто этот человек», он только сообщает голосовому коду «уже подтверждено извне».

---

## 6. Последствия

| Плюсы | Минусы и риски |
|---|---|
| Робот перестаёт врать про незнакомца, если лицо уже поздоровалось | Новый failure mode: hint пришёл не от того человека (тёзка с похожим эмбеддингом) → голос ошибочно «подтверждает». Защита: `similarity_high_threshold = 0.78` + freshness-окно 30 сек |
| Минимальный PR (≤ 300 строк по ADR-0013), закрывает #3024, не ломает #2809/#2888 | Sub-symmetry: добавление `note_face_seen` ломает тех, кто полагался на «`IdentitySeam` — это только голос» (см. §6.1) |
| Деградация к текущему поведению в трёх режимах (§2.5) | False positives при калибровке (один ТП замер); потребуется shadow-логирование в первый месяц |
| ADR-0123 §6 становится **проще** — на момент, когда Phase 2 добавит реальную arbitration, кольцо `note_face_seen` уже есть, ему не нужно заводить отдельный механизм | Нужно обновление архитектурной диаграммы `docs/architecture/identity-seam.md` (см. §3 п.6) |

**§6.1 Что ломается у других потребителей `IdentitySeam`.** `mcp_server`, UI-логи, `tool_executor` — если кто-то из них полагался на «`MemoryIdentitySeam` — только про голос» (т.е. конкретно на список операций в ADR-0106 §3.2), расширение безопасно (добавлены новые методы, ничего не переименовано и не удалено). Проверка: в §3 п.1 добавить тест на сигнатуру класса — публичные методы только растут, ничего не удаляется.

---

## 7. Контракт верификации (raw-evidence, ADR-0018)

Шифу в issue / PR — приложить:

1. `pytest -v src/rob_box_harness/test/test_identity_seam_face_hint.py` — полный вывод, 8+ тестов зелёные.
2. `pytest -v src/rob_box_voice/test/unit/node/test_issue_3024_face_hint_suppresses_voice_recheck.py` — full PASS, проверяет сценарий из §3 п.4.
3. `pytest -v src/rob_box_voice/test/test_yaml_param_consistency.py` — расширение на 4 новых ключа, зелёный.
4. `docker logs vision-face | tail -50` — реальное сообщение `/perception/face/meeting` с payload (после e2e-process stage, см. AGENTS.md; я в этой карточке e2e на железо не запускаю — это работа e2e-процесса ПОСЛЕ merge).
5. `git diff develop -- docs/adr/0135-*.md src/.../identity/base.py src/.../dialogue_node.py docker/.../dialogue_node.yaml` — diff в PR.

**Без raw-вывода 1–3 Шифу не принимает.** ADR-0018 explicit.

---

## 8. Ссылки

- issue #3024 — баг «26 сек переспрос».
- ADR-0089 §8 — ранний privacy-дизайн, отменён ADR-0123.
- ADR-0102 §3 — «Повод» (случай `kind="meeting"` уже используется в `_handle_meeting`).
- **ADR-0106 §3.2** — `IdentitySeam` сигнатура и `VoiceSignal`. Этот ADR её расширяет, не переписывает.
- **ADR-0105 §3.1** — `EncounterSeam` дублирование состояния, открыто «до миграции потребителей». Этот ADR — одна из тех миграций.
- **ADR-0123 §6** — отложенная полная arbitration. Этот ADR её не делает, но расчищает мост.
- ADR-0131 — один источник истины спикера на utterance (на чём держится single-confirmation-путь #2809).
- ADR-0069 — ADR-коллизия inflight-check (для безопасности renumbering 0135 vs 0134 / 0136).
- test/unit/node/test_issue_2809_identity_confirmation_dialogue.py — регрессия: новая реализация должна оставить его зелёным.
