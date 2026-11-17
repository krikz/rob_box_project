# ADR-0139: Face→voice hint в шве «Знакомый» — свежее наблюдение лица снимает голосовой переспрос

> Текст реконструирован по коду, конфигу, тесту и сообщению коммита `eaa71ec9`
> (PR #3040, issue #3024, 28.09.2026) — см. «Почему номер 0139» и раздел
> «Восстановление текста» ниже. Отдельного ADR-документа в истории репозитория
> `git log --all` для этого контракта не найдено; PR #3040 ссылался на номер
> `ADR-0135`, который на деле занят другим, не связанным ADR (`0135-face-picker-quest.md`,
> лицевые команды на Quest, PR #3031/#3026). Этот файл закрывает разрыв.

| Поле | Значение |
|---|---|
| Статус | **Accepted** (реализовано PR #3040, issue #3024; текст восстановлен/реконструирован по коду 2026-09-28) |
| Дата | 2026-09-28 (дата реконструкции; код и тест — из PR #3040, слит в `develop` до `eaa71ec9`) |
| Автор | Claude Code (реконструкция по коду, тесту и коммиту), исходная реализация — воркер PR #3040 |
| Контекст | Issue #3024: «Робот поздоровался по имени (лицо), а через 26 сек спрашивает „Дэнчик, это ты?"». Лицо (Vision Pi) уже подтвердило имя через `/vision/hailo/events` → `_handle_meeting` (приветствие), но шов «Знакомый» (ADR-0106) не знал об этом наблюдении — голосовая ветка `_handle_tentative_speaker` (issue #2809/#2888) видела только голосовой скор (например 0.791, band `single`) и задавала переспрос заново. Нужен канал, которым лицо оставляет **подсказку**, а не факт, голосовому потребителю того же шва. |
| Затрагивает | `src/rob_box_harness/rob_box_harness/identity/base.py` (`FaceSignal`, `FaceObservation`, `IdentitySeam.configure_face_hint/note_face_seen/recent_face_observation[_by_name]`, `DEFAULT_FACE_HINT_*`); `src/rob_box_voice/rob_box_voice/dialogue_node.py` (`_declare_params` namespace `face_voice_hint.*`, `_on_vision_event`, `_configure_face_voice_hint`, `_face_hint_confirmation`, `_handle_tentative_speaker`); `src/rob_box_voice/config/dialogue_node.yaml` и `docker/vision/config/voice_assistant/dialogue_node.yaml` (секция `face_voice_hint`); `src/rob_box_voice/test/unit/node/test_issue_3024_face_hint_suppresses_voice_recheck.py` |
| Родители | ADR-0106 («Знакомый» — базовый контракт шва: `resolve`/`note_seen`/`since_last_seen`/`merge`, на который этот ADR добавляет параллельную, необязательную грань `note_face_seen`/`recent_face_observation*`), ADR-0018 (честный FAIL / `capability-honest` — деградация флагом, не молчаливый провал), ADR-0013 (инкрементальная поставка — hint как маленький аддитивный шаг перед ADR-0130 v2.0) |
| Связанные | issue #3024 (этот ADR — его фикс), PR #3040 (реализация), ADR-0123 §6 («Узнавание и слияние» — источник дефолта `high=0.78`: тот же порядок, что у голосового `identify`), ADR-0089 §2.2 (источник дефолта `low=0.65` как «стаб-зеркало» нижней полосы), ADR-0106 (шов «Знакомый», на котором hint живёт как ring-буфер), ADR-0130 §2.6 путь 2 «Два независимых канала согласны» (в v2.0 face→voice-совпадение по имени станет одним из путей `confirmed` трека; этот hint — его рабочий предшественник в 1.1, см. §2.5 ADR-0130 «Время улики» — здесь свежесть считается тем же принципом «наблюдение протухает по времени», но ещё без общего трекера) |

> **TL;DR.** `IdentitySeam` (ADR-0106) получает вторую, необязательную грань:
> `note_face_seen(FaceSignal)` кладёт наблюдение лица в in-memory кольцевой
> буфер per `person_id` (не в долговременный стор — это **наблюдение, а не
> факт**), `recent_face_observation_by_name(name)` читает самое свежее в
> пределах TTL-окна. Голосовая ветка `_handle_tentative_speaker` перед тем,
> как задать переспрос «Дэнчик, это ты?», проверяет: есть ли свежий
> face-hint с тем же именем и полосой уверенности `high` (similarity ≥
> 0.78)? Если да — сразу confirmation path, переспроса нет. Вся логика
> под флагом `face_voice_hint.enabled` (дефолт `true`): выключен — ноль
> побочных эффектов, поведение как до PR #3040.

---

## 1. Контекст: живой случай issue #3024

Гипотеза подтверждена по логам конкретного прогона (`t_ee583f20` в сообщении
коммита `eaa71ec9`):

1. Лицо (Vision Pi, Hailo) видит человека → `/vision/hailo/events` →
   `parse_meeting_marker` → `_handle_meeting` → робот здоровается по имени.
2. Через ~26 секунд голос того же человека распознаётся с умеренной
   уверенностью (`score=0.791`, band `single` — не `confident`), и
   голосовая ветка `_handle_tentative_speaker` (issue #2809/#2888) задаёт
   вопрос «Дэнчик, это ты?», потому что шов «Знакомый» не хранил
   информацию о том, что лицо *только что* подтвердило то же имя.

Проблема — не в порогах голосового распознавания (это отдельный вопрос
#2809), а в том, что два канала одного шва (ADR-0106) не обменивались
даже такой слабой уликой, как «пять секунд назад лицо с высокой
уверенностью видело человека с этим именем».

## 2. Решение

### 2.1 `FaceSignal` — структурно-типизированный сигнал, без зависимости на `rob_box_perception`

`FaceSignal` — `@dataclass(frozen=True)` с полями `person_id`, `name`,
`similarity`, `is_new`, `source_camera`, `captured_at`. Шов **не
импортирует** `rob_box_perception`: `dialogue_node._on_vision_event` уже
парсит `/vision/hailo/events` через `parse_meeting_marker` (тот же путь,
что кормит `_handle_meeting`) и сам собирает `FaceSignal` из
`MeetingMarker`. Структурная типизация — намеренно: `IdentitySeam` живёт
в `rob_box_harness`, у которого нет и не должно быть зависимости на
пакет восприятия.

### 2.2 Наблюдение — не обновление профиля

`note_face_seen` кладёт `FaceObservation` в **in-memory кольцевой буфер**
(`Dict[person_id, Deque[FaceObservation]]`, `maxlen=buffer_capacity`,
дефолт 8) и **не пишет** в `MemoryStore`/долговременный стор шва. Это
осознанная граница: лицо остаётся источником **подсказки**, а не
источником `name` для `Acquaintance` — пока не появится полноценная
арбитрация двух каналов (ADR-0123 §6 п.2 «арбитр» открыт; закрывается
как отдельный путь `confirmed` только в ADR-0130 §2.6 путь 2).

Контракт roundtrip: `note_face_seen(signal)` → `recent_face_observation(person_id, window_sec, now)`
возвращает самое свежее наблюдение в пределах окна или `None`, если буфер
пуст, ключа нет, либо последнее наблюдение старше `window_sec` (TTL).
Отдельно — `recent_face_observation_by_name(name, ...)`: голосовой
потребитель знает `tentative_name`, а не `person_id` (пространство
Vision), и сшивка по имени — единственная доступная связка до полной
arbitration ADR-0123 §6 / ADR-0130 §2.6.

Буфер `_face_observations` очищается от протухших `person_id` при каждом
новом `note_face_seen`/чтении (`_prune_face_observations`) — `deque(maxlen=...)`
сам по себе ограничивает только число наблюдений *внутри* одного
`person_id`, а не рост словаря по новым UUID.

### 2.3 Доставка — через существующий топик, без нового контракта Vision Pi

`_on_vision_event` (уже существующий колбэк `/vision/hailo/events`,
общий с `_handle_meeting`) дополнительно вызывает
`self._identity.note_face_seen(...)`, когда `parse_meeting_marker`
вернул маркер. Новый топик `/perception/face/meeting` **не заводится** —
контракт Vision Pi (§5.2) остаётся прежним. Колбэк горячий (~5
событий/сек на человека в кадре), поэтому `note_face_seen` — дешёвая
in-memory операция без сети и БД.

### 2.4 Полосы уверенности и условие подавления переспроса

```python
DEFAULT_FACE_HINT_HIGH = 0.78          # ADR-0123 §6 (голосовой identify)
DEFAULT_FACE_HINT_LOW = 0.65           # ADR-0089 §2.2 (стаб-зеркало low)
DEFAULT_FACE_HINT_WINDOW_SEC = 30.0
DEFAULT_FACE_HINT_BUFFER_CAPACITY = 8
```

`_classify_face_band(similarity)` возвращает `"high"` (`≥ 0.78`),
`"tentative"` (`≥ 0.65`), `"low"` (иначе). Голосовая ветка
`_face_hint_confirmation` подавляет переспрос **только** если
одновременно:

1. `face_voice_hint.enabled` — `True`;
2. переспрос по этому кандидату ещё не задавался (`not state.get("asked")`);
3. есть `tentative_name` (голос уже предполагает имя);
4. `recent_face_observation_by_name(tentative_name, window_sec=...)`
   вернул наблюдение (**не** `None`, то есть оно в пределах TTL-окна);
5. `face_obs.confidence_band == "high"` (полоса `tentative`/`low` —
   недостаточно, переспрос задаётся как раньше);
6. **имя совпадает** — сравнение идёт уже через `recent_face_observation_by_name(tentative_name, ...)`,
   то есть подсказка для *другого* имени просто не находится и не
   подавляет переспрос.

При выполнении условия: `state["asked"] = True`, `state["confirmed"] = True`,
`state["name"] = tentative_name`, вызывается `_confirm_tentative_speaker`
— переспрос «Дэнчик, это ты?» не звучит.

### 2.5 Флаг `enabled` — деградация к текущему поведению (`capability-honest`)

`face_voice_hint.enabled=False` (или параметр не задан — `getattr(..., False)`
для легаси unit-тестов, собирающих `DialogueNode` через `object.__new__`)
означает: `_on_vision_event` не зовёт `note_face_seen`, `_face_hint_confirmation`
возвращает `False` сразу, `_handle_tentative_speaker` работает как до
PR #3040. Падение самого `note_face_seen` (например, неожиданный тип в
`FaceSignal`) ловится `try/except` в `_on_vision_event` и логируется на
уровне `debug` — не должно ронять `_handle_meeting`. Это `capability-honest`
деградация (ADR-0018): явный флаг/лог, а не молчаливое поведение
«наполовину работает».

### 2.6 Конфигурация — `face_voice_hint.*` namespace-параметры

Дотированные ROS2-параметры (проверяются `test_yaml_param_consistency`
между `src/` и `docker/`-вариантами):

| Параметр | Дефолт | Смысл |
|---|---|---|
| `face_voice_hint.enabled` | `true` | §2.5 — общий выключатель |
| `face_voice_hint.window_sec` | `30.0` | TTL наблюдения (§2.2, §2.4) |
| `face_voice_hint.similarity_high_threshold` | `0.78` | полоса `high` (§2.4) |
| `face_voice_hint.similarity_low_threshold` | `0.65` | полоса `low` (§2.4) |
| `face_voice_hint.buffer_capacity` | `8` | размер кольцевого буфера на `person_id` (§2.2) |

`DialogueNode.__init__` читает эти параметры и вызывает
`_configure_face_voice_hint()` → `IdentitySeam.configure_face_hint(...)`
— вынесено в отдельный helper, чтобы `__init__` не выходил за
CC-бюджет (ADR-0021).

## 3. Acceptance-тест (issue #3024) — восемь кейсов

`test_issue_3024_face_hint_suppresses_voice_recheck.py`, красный до
реализации, зелёный после:

1. `FaceSignal`/`FaceObservation` — dataclass'ы существуют и
   импортируются из `IdentitySeam` (§2.1, §2.2).
2. `note_face_seen`/`recent_face_observation` — roundtrip, TTL = `window_sec`.
3. `confidence_band`: `high`/`tentative`/`low` по порогам 0.78/0.65 (§2.4).
4. Главный сценарий: свежий face-hint с тем же именем и band `high` →
   переспрос **не** задаётся, `state["confirmed"] = True`,
   `_confirm_tentative_speaker` вызван, результат — тег confirmation
   path (не `_tag_tentative`).
5. **Регрессия**: hint старше `window_sec` (31с при окне 30с) →
   `recent_face_observation` возвращает `None` — переспрос возможен,
   «false-fix» не проходит.
6. `face_voice_hint.enabled=false` → переспрос как раньше (§2.5):
   `asked=True`, `confirmed=None`, `_ask_tentative_identity` вызван,
   `_confirm_tentative_speaker` — нет.
7. `similarity < low_threshold` (band `low`) → hint игнорируется,
   переспрос задан (§2.4).
8. Hint для **другого** имени (тёзка/путаница в кадре) → переспрос
   задан, подавления нет (§2.4 условие совпадения имени).

Тест не поднимает ROS2 — `DialogueNode` собирается через `object.__new__`,
как в `test_issue_2809_*`.

## 4. Отвергнутые альтернативы

| Альтернатива | Почему нет |
|---|---|
| Писать face-hint сразу в `Acquaintance`/долговременный стор | Смешивает «наблюдение» и «факт» — лицо стало бы источником имени в обход голосовой верификации, до полной arbitration ADR-0123 §6 / ADR-0130 §2.6 |
| Новый топик `/perception/face/meeting` под hint | Дублирует уже существующий `/vision/hailo/events`; лишний контракт Vision Pi без необходимости (§2.3, §5.2) |
| Сшивка по `person_id` вместо имени | Voice не знает Vision-пространство `person_id` до полной arbitration (ADR-0123 §6 Phase 2); имя — единственный общий атрибут сейчас |
| Подавлять переспрос на любой полосе (включая `tentative`) | Полоса `tentative` (0.65–0.78) — то же качество, что и голосовой `single`; два слабых сигнала не должны молча складываться в подтверждение без явного правила arbitration |

## 5. Инварианты и проверка

1. `note_face_seen` — синхронная, in-memory, без сети/БД (§2.2, §2.3).
2. Наблюдение не переживает `window_sec` — `recent_face_observation*`
   не возвращает протухшее (§2.2, тест п.5).
3. `enabled=false` — ноль побочных эффектов относительно поведения до
   PR #3040 (§2.5, тест п.6).
4. Подавление переспроса возможно только при полосе `high` и совпадении
   имени (§2.4, тесты п.7–8).

### 5.2 Контракт Vision Pi не меняется

`/vision/hailo/events` остаётся единственным топиком, через который
Vision Pi сообщает о встречах лиц; `parse_meeting_marker` — единственный
парсер. Hint — потребитель уже существующего потока, не новый
производитель на стороне Vision Pi.

## 6. Trade-offs

- **Ещё один намespace параметров** (`face_voice_hint.*`) в и без того
  большом `dialogue_node.yaml` — оправдано тем, что калибровка (пороги,
  окно) может понадобиться без релиза кода.
- **In-memory буфер теряется при рестарте ноды** — осознанно: это
  наблюдение с TTL 30 секунд, а не факт памяти; переживать рестарт ему
  не нужно.
- **Сшивка по имени, а не по `person_id`** — временное решение до
  arbitration (ADR-0123 §6 / ADR-0130 §2.6); тёзки в кадре подавят
  переспрос неверно, но условие «то же имя и полоса high» уже сильно
  сужает ложные срабатывания, а при разночтении переспрос просто
  проходит как раньше (safe default — тест п.8).

### 6.1 Аддитивность

Реализация не удаляет и не меняет поведение существующих операций шва
(`resolve`/`note_seen`/`since_last_seen`/`merge` из ADR-0106) — только
добавляет параллельную, по умолчанию включённую, но полностью
опциональную грань (`configure_face_hint`/`note_face_seen`/
`recent_face_observation*`). Старые швы и узлы, ни разу не вызывавшие
`configure_face_hint`, продолжают работать: буфер пуст,
`recent_face_observation*` всегда возвращает `None`. Легаси unit-тесты,
собирающие `DialogueNode` без `__init__` (`test_issue_2809_*`,
`test_dialogue_node_imports`), не требуют правки моков — `getattr(...,
False)` покрывает отсутствующие атрибуты (§2.5).

## 7. Почему номер 0139

PR #3040 был написан со ссылками на «ADR-0135», но к моменту его слияния
номер 0135 уже был закреплён за другим ADR — `0135-face-picker-quest.md`
(лицевые команды на капитанском мостике Quest, PR #3031/#3026, слит
раньше). Коллизия номеров ADR внутри RT-домена запрещена ADR-AF-0030
(«ADR-numbering — SOT», issue #2076): номер уникален внутри своего
домена (`ADR-NNNN` — RT, `ADR-AF-NNNN` — agent-flow). `0139` — следующий
свободный номер RT-домена на момент реконструкции (`0136`…`0138` заняты
или зарезервированы); закреплён оркестратором задачи, а не выбран
автором этого файла. Все ссылки `ADR-0135` в коде/конфигах/тестах,
относящиеся к face-hint, заменены на `ADR-0139`; ссылки на настоящий
`0135-face-picker-quest.md` (Quest, `face_card.py`, `face_collage.py`,
`docs/design/2026-09-25-face-picker-quest.md`) не тронуты.
