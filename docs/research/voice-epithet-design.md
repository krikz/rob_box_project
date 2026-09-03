# Voice-epithet per speaker — research-отчёт (проблема «двух Денисов»)

**Дата:** 2026-08-31
**Назначение:** сводный ресёрч по схемам именования/идентификации спикеров
в persistent voice-agent'ах. Цель — выбрать подход к **эпитету** (внутренней
кличка голоса) для rob_box_voice, чтобы решить коллизию «два Деника с одним
именем». Сам **фикс не входит** в эту задачу — только дизайн-рекомендация
для отдельного PR.
**Контекст:** Kanban-задача `t_3107327b`, GitHub issue #1787.
Связанные: #1077 (speaker profiles), #1770 (memory_context без
speaker_id), #1774 (ADR про слои памяти).

**Исходники в репо:**
- `src/rob_box_voice/rob_box_voice/utils/speaker_embeddings.py` —
  `SpeakerDatabase`, d-vector (resemblyzer), 256-dim, `/data/speakers.db`,
  `IDENTIFY_THRESHOLD = 0.75`.
- `src/rob_box_voice/rob_box_voice/speaker_id_node.py` — ROS2-нода,
  привязывает `d-vector → name`.
- `src/rob_box_voice/rob_box_voice/dialogue_node.py:822` —
  `self.declare_parameter("speaker_db_path", "/data/speakers.db")`.
- `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/dialogue.py:893, 980` —
  известный баг «`name='Зовут'`» при ловушке фразы «робот меня зовут
  Денис говорю» (фикс в #1101, регрессия в
  `test/test_tools/test_dialogue_register_speaker.py`).

---

## 0. TL;DR

- **Проблема.** Робот запоминает спикера **по имени** («Денчик», «Саша»).
  Имя — публичное поле, которое юзер сам произносит. Два разных человека
  с именем «Денис» → одна и та же запись в `speakers.db`, перезапись
  эмбеддингов, «другой Денис теряет личность».
- **Решение.** Добавить параллельное поле `epithet` — **внутреннее имя,
  которое робот сам себе назначает**, опираясь на семантический кластер
  речи, эмоциональный окрас, типичные темы и контекст первого появления.
  Имя живёт публично, эпитет — приватно, для робота.
- **Подход.** **Гибрид: словарь-кандидаты (200–500 кличек) + LLM-валидатор**
  из контекста первых N реплик. Словарь ограничивает стиль и делает
  эпитет узнаваемым, LLM выбирает/меняет при накоплении новых сигналов.
- **Хранение.** Миграция `speakers.db`: `+epithet TEXT`,
  `+epithet_history TEXT (JSON)`, `+tags TEXT`, `+sentiment_score REAL`.
  Эпитет версионируется (`epithet_history`) — можно откатить и видеть
  эволюцию.
- **Триггер пересмотра.** Не каждый спич. Только при «новом
  отличительном» событии (новая доминирующая тема > N% диалога, явная
  смена тональности, контекст первого появления). Дефолт — заморозить
  эпитет через 30 дней после установки.
- **Самый близкий prior art.** arxiv **2604.25022** — «AFA: Identity-Aware
  Memory for Preventing Persona Confusion in Multi-User Dialogue»
  (апрель 2026). Это **ровно наша проблема**, но для общего
  multi-user-сценария, не для домашнего робота.
- **Рекомендация для rob_box:** гибрид «словарь + LLM-валидатор», v1 —
  словарь 200 кличек + правила, эпитет ставится при первом появлении,
  пересматривается не чаще раза в N дней. Реализация — отдельный PR после
  согласования дизайна.

---

## 1. Проблема в rob_box'е (конкретика)

### 1.1 Текущая схема идентификации

`SpeakerDatabase` (см. `speaker_embeddings.py:90-204`):

```sql
CREATE TABLE speakers (
    speaker_id  TEXT PRIMARY KEY,  -- UUID
    name        TEXT NOT NULL,     -- публичное имя («Денчик»)
    created_at  REAL NOT NULL
);
CREATE TABLE embeddings (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    speaker_id  TEXT NOT NULL REFERENCES speakers(speaker_id) ON DELETE CASCADE,
    embedding   BLOB NOT NULL,     -- 256-dim d-vector, numpy float32
    created_at  REAL NOT NULL
);
```

Идентификация:
1. STT → d-vector (`resemblyzer VoiceEncoder`, 256-dim).
2. `SpeakerDatabase.identify(embedding)` → cosine-similarity ко всем
   референсным эмбеддингам, берётся argmax.
3. Если `best_score ≥ 0.75` — `SpeakerMatch(speaker_id, name,
   confidence)`. Ниже — `None`, создаётся новый спикер с именем из
   контекста диалога («робот меня зовут …») или `null`.

### 1.2 Где ломается

**Сценарий «двух Денисов».**
- Денис #1 (хозяин) регистрируется: `name='Денчик'`, embedding #1.
- Приходит Денис #2 (тёзка, другой голос): d-vector #2 отличается →
  cosine с embedding #1 ≈ 0.3 (ниже 0.75) → создаётся новая запись
  `name='Денчик'` с другим `speaker_id`. Два «Денчика» в БД.
- Следующее появление Дениса #2: d-vector снова похож на свой
  embedding → корректно идентифицируется как «Денчик #2».
- Но если денис #2 однажды скажет «робот меня зовут Денис» — `name`
  перезаписывается (см. `rename_by_name` в коде), и Денис #1 теряет
  узнавание у робота.

**Сценарий «`name='Зовут'`».** Уже задокументирован в
`dialogue.py:911-998`: STT-ловушка на фразе «робот меня зовут Денис
говорю» парсится как `name='Зовут'`. Без эпитета эта строка остаётся
«правдой» в БД.

**Сценарий «нового спикера без имени».** Юзер молчит, не говорит
имени. `name=null`. Робот не знает, как обратиться при следующей
встрече. Сейчас робот вынужден спрашивать или дефолтить «Друг».

### 1.3 Что должна решить «эпитетная» система

1. **Коллизия имён.** Даже если оба «Дениса» — робот различает их
   внутренне («Денчик-шахматист» vs «Денчик-кодер»).
2. **Устойчивость к мусорным именам.** Если STT вернул «Зовут» —
   эпитет остаётся нормальным («Странник», «Тихий»).
3. **Холодный старт.** Юзер без имени сразу получает обращение в
   логах/контексте — пусть и не публичное.
4. **Эволюция.** Если спикер меняет темы/тональность — эпитет
   пересматривается, история сохраняется.

---

## 2. Что делают другие (5+ источников)

### 2.1 arxiv 2604.25022 — AFA: Identity-Aware Memory for Preventing Persona Confusion in Multi-User Dialogue (апр 2026)

**Самый релевантный источник — буквально наша проблема.**

- URL: https://arxiv.org/abs/2604.25022 (html: https://arxiv.org/html/2604.25022v1)
- Суть: модульный фреймворк Adaptive Friend Agent — комбинирует
  voice-based speaker identification (как у нас, d-vector/ECAPA-TDNN) с
  per-user memory stores. На одной shared-ноде живёт несколько юзеров,
  каждый со своей памятью.
- Ключевая метрика: **Persona Attribution Accuracy (PAA) > 60%** в
  multi-user-сценариях.
- Что берём для rob_box'а:
  - **Идентичность ≠ имя.** Имя — публичное поле, спикер-ID —
  внутреннее. То же самое, что мы называем «эпитет».
  - **Embedding → memory routing обязателен.** Никаких «по имени из
  контекста» в multi-user — только по эмбеддингу + эпитету в логах.
  - **PAA** можно взять как нашу метрику качества идентификации.

### 2.2 arxiv 2406.13960 — SPDA: Self-evolving Personalized Dialogue Agents (2024)

- URL: https://arxiv.org/html/2406.13960v1
- Суть: персонаж не preset, а эволюционирует в течение разговора,
  подстраиваясь под юзера. Непрерывное обучение на лету.
- Что берём:
  - **Эпитет = эволюционирующая сущность**, а не «навеки застывшее
  имя при первом появлении».
  - Но для rob_box v1 рекомендую **замедленную эволюцию** (раз в N
  дней, а не каждый спич) — иначе робот начнёт менять «клички» в
  диалоге, что юзеру будет звучать как баг.

### 2.3 arxiv 2406.05925 — Hello Again! LLM-powered Personalized Agent for Long-term Dialogue (2024)

- URL: https://arxiv.org/abs/2406.05925
- Суть: lifelong-агент, который помнит предпочтения юзера между сессиями
  (месяцы, годы). Персонализированная долговременная память.
- Что берём:
  - **Три уровня памяти:** raw events → summarised → consolidated facts.
  Эпитет попадает в верхний уровень (consolidated facts) — самое
  стабильное.
  - **Forget curve:** старые эпизоды забываются, важные остаются.
  Эпитет живёт «наверху» и не забывается никогда (пока спикер не
  перестанет приходить).

### 2.4 arxiv 2506.06254 — PersonaAgent (jun 2025)

- URL: https://arxiv.org/abs/2506.06254
- Суть: персона как «уникальный system prompt для каждого юзера».
  Цикл: insights из памяти → контроль действий → outcomes → refine
  память.
- Что берём:
  - **Эпитет можно хранить в system prompt** для каждого
  идентифицированного спикера — не только в БД.
  - **Безопасность:** когда персона используется для legitimation
  вредоносных запросов, нужны guardrails (отдельная тема, не блокируем
  голос, но в ADR отметить).

### 2.5 Stanford Smallville / Generative Agents (Park et al., 2023)

- URL: https://github.com/joonspk-research/generative_agents
- Суть: 25 агентов в симуляции, memory stream (raw events), retrieval,
  reflection, planning. Reflection создаёт «higher-level self-notions» —
  по сути, автоматические абстракции над памятью.
- Что берём:
  - **Reflection = прообраз эпитета.** Агент периодически
  рефлексирует: «последние 100 событий — я в основном обсуждаю
  шахматы → я Шахматист».
  - **Не делаем** в v1 полноценный Smallville — но триггер
  пересмотра эпитета можно сделать по тому же принципу (накопление
  salient memories → reflection → новый эпитет).

### 2.6 Soul Machines / Anam AI — avataric agent identity

- URL: https://zylos.ai/research/2026-04-10-ai-agent-persona-design-behavioral-consistency/
  (обзорное), конкретные доки — проприетарные.
- Суть: avataric agent identity отделяется от task context.
  Identity — на отдельном слое, user-specific adaptation — на другом.
- Что берём:
  - **Архитектурный принцип:** слой identity (сюда эпитет) ≠ слой
  task (сюда memory_context из #1770). Не смешивать.

### 2.7 SOUL.md / soul.py — Persistent Identity в OpenClaw

- URL: https://www.researchgate.net/publication/403790183_Persistent_Identity_in_AI_Agents
  (multi-anchor architecture, mar 2026).
- Суть: agent identity = структурированный markdown-документ
  (`SOUL.md`), загружается при старте, до любого user-взаимодействия.
  Разделение: identity files + memory logs.
- Что берём:
  - **Структура файла, а не плоский текст.** Эпитет — это пара
  полей (label + metadata), а не свободная строка в логе.
  - **Multi-anchor:** можно иметь несколько «якорей» идентичности
  (голос, имя, эпитет, поведенческие паттерны) — голос самый
  надёжный, имя — самое хрупкое.

### 2.8 NPC persistent identity (game industry)

- URL: https://mistscale.com/blog/persistent-memory-for-game-npcs (июль 2026),
  https://lineage2ai.com/npc-memory
- Суть: три слоя памяти NPC — raw logs, summarised, stable character
  facts. «Stable facts» не подменяются — NPC не противоречит своей
  backstory.
- Что берём:
  - **Stable layer = эпитет.** Не пересматривать эпитет «в режиме
  разговора» — только при явном событии.
  - **Конфликт-резолюция:** если новый эпитет противоречит старому
  (юзеру не нравится «Странник»), юзер должен иметь способ
  перебить: «робот, теперь я Кадет».

### 2.9 VoiceMem — Streaming Dual-Brain Memory for Real-Time Interaction (авг 2026)

- URL: https://arxiv.org/pdf/2608.26005
- Суть: «left brain» (память, факты) + «right brain» (эмоция + persona)
  в real-time диалоге. Минимальная latency.
- Что берём:
  - **Persona отделён от фактов.** Эпитет — в right brain, история
  разговоров — в left brain. Не смешивать.

---

## 3. Сравнительная таблица подходов

| Подход | Что делает | Pros | Cons | Для rob_box v1? |
|---|---|---|---|---|
| **A. Просто UUID, без эпитета** | Спикер = UUID, имени нет, в диалоге только «Друг» | Минимум кода, никаких коллизий | Юзер не узнаёт обращение; скучно | ❌ Хуже текущего |
| **B. Только embedding (без имени)** | Идентификация чисто по d-vector, имя не используется | 100% робаст к коллизиям имён | Нет человекочитаемой метки; в логах UUID; нельзя обратиться по имени | ⚠️ Частично (как fallback) |
| **C. Словарь-кандидаты (200–500 кличек)** | Из фиксированного списка выбирается «Гроссмейстер» по кластеру тем | Предсказуемо, дёшево (без LLM), короткий латент | Не учитывает уникальный стиль речи; топорно | ✅ Как **стартовый слой** |
| **D. LLM-generate epithet** | LLM сама придумывает кличку по первым N репликам | Максимально точная, контекстная | Дорого (токены на каждый новый голос); может выдавать странное; требует guardrails | ⚠️ Как **валидатор/замена**, не основа |
| **E. Hybrid: словарь + LLM-валидатор** | Словарь генерирует кандидата → LLM подтверждает/правит | Баланс предсказуемости и гибкости | Два шага, чуть сложнее пайплайн | ✅ **Рекомендую** |
| **F. Multi-anchor (voice + name + epithet + behavior)** | Каждый идентификатор — отдельный якорь | Устойчиво к сбоям каждого канала | Архитектурно тяжелее | 📌 v2 |
| **G. SOUL.md per-speaker** | Каждый спикер = markdown-файл с секциями | Прозрачно, отлаживаемо | Файловая система вместо БД; не масштабируется | ❌ Избыточно |
| **H. Reflection-based (Smallville-style)** | Эпитет = emergent-абстракция над памятью | Самый «живой» | Тяжёлый, медленный, непредсказуемый | 📌 v3 (исследование) |

---

## 4. Рекомендация для rob_box

### 4.1 Гибрид (подход E)

**Слой 1 — словарь-кандидаты (детерминированный).**
- 200–500 кличек, размеченных по 4 осям (из задачи):
  1. Семантический кластер речи: «джунгли», «город», «техно», «шахматы»,
     «классика», «космос», «фантастика», «детектив», «спорт», …
  2. Эмоциональный окрас: «тревожный», «спокойный», «игривый»,
     «сосредоточенный», …
  3. Стиль запросов: «крутой», «мистик», «классик», «гик», …
  4. Стоимость упоминания: «Кадет», «Барон», «Странник», «Магистр», …
- Словарь — JSON в `src/rob_box_voice/rob_box_voice/data/epithets.json`,
  рядом с тем, где лежат другие статические справочники (см. `sound_pack`).
- Выбор кандидата — правилами: «если > N% сообщений содержат
  ключевые слова кластера X → выбирай топ-3 эпитета из X».

**Слой 2 — LLM-валидатор (опциональный, по контексту).**
- Если кандидатов несколько → LLM получает 3 эпитета + 5 последних
  реплик спикера → возвращает один + короткое обоснование.
- Если валидатор недоступен (offline, латентность) → fallback на
  rule-based выбор из топ-3.

**Слой 3 — пересмотр (reflection).**
- Эпитет пересматривается **не чаще раза в N дней** (N=30 по
  умолчанию) **или** при «distinctive event»:
  - новая доминирующая тема (> 40% последних 50 реплик);
  - смена эмоционального окраса голоса (VAD drift > порога);
  - юзер явно просит переобозвать: «робот, теперь я Кадет».
- История эпитетов — `epithet_history TEXT (JSON array)`. Откат
  доступен.

### 4.2 Почему не только словарь (C) и не только LLM (D)

- **Только словарь** — топорно, не отражает уникальный стиль.
  Пример: двое «Денисов» оба любят шахматы → оба «Гроссмейстер».
  Каллизия вернётся.
- **Только LLM** — дорого (токены на каждого нового юзера), может
  выдавать странное («Весело-Угрюмый-Кентавр»). Без словаря
  предсказуемость = 0, нельзя построить guardrails.

Гибрид берёт **детерминированную рамку** (словарь) и **гибкую
валидацию** (LLM), при этом любой из слоёв может отключаться без
потери работоспособности.

### 4.3 Схема данных (миграция)

```sql
ALTER TABLE speakers ADD COLUMN epithet TEXT;
ALTER TABLE speakers ADD COLUMN epithet_history TEXT;  -- JSON, [{ts, epithet, reason}]
ALTER TABLE speakers ADD COLUMN tags TEXT;             -- CSV: «шахматы,техно,классика»
ALTER TABLE speakers ADD COLUMN sentiment_score REAL;  -- VAD-средний
```

Индекс по `epithet` — опционально (словарь маленький, lookup и так O(N)).

### 4.4 Пайплайн

```
[STT + d-vector]
    ↓
SpeakerDatabase.identify(embedding)
    ↓
[если new speaker]:
    register(embedding, name=from_context_or_null)
        ↓
    [первые N реплик собраны]:
        classify_topics(recent_messages) → tags
        choose_epithet(tags, sentiment_score) → epithet_candidate
        [если LLM-валидатор доступен]:
            validate_epithet(epithet_candidate, recent_messages) → epithet
        [иначе]:
            epithet = epithet_candidate
        speaker.epithet = epithet
        speaker.epithet_history.append({ts: now, epithet: None, epithet: epithet, reason: "first_seen"})
    ↓
[если recurring speaker + distinctive event]:
    re_evaluate_epithet(speaker, recent_messages)
        ↓
    speaker.epithet_history.append(...)
    speaker.epithet = new_epithet
```

### 4.5 Псевдокод (не для немедленной реализации)

```python
# src/rob_box_voice/rob_box_voice/utils/epithets.py

@dataclass
class EpithetCandidate:
    label: str
    source_cluster: str  # «шахматы», «техно», …
    confidence: float


@dataclass
class SpeakerProfile:
    speaker_id: str  # UUID
    name: Optional[str]
    epithet: Optional[str]
    epithet_history: list[tuple[datetime, Optional[str], str, str]]  # ts, old, new, reason
    tags: list[str]
    sentiment_score: float
    embedding: np.ndarray  # d-vector
    last_epithet_review: Optional[datetime]


# Словарь (JSON)
EPITHET_LEXICON = {
    "шахматы": ["Гроссмейстер", "Кадет", "Магистр"],
    "техно":   ["Электрик", "Инженер", "Кулибин"],
    "классика":["Академик", "Мэтр", "Классик"],
    "спорт":   ["Чемпион", "Атлет", "Спринтер"],
    "космос":  ["Космонавт", "Звездопроходец", "Астроном"],
    # ...
}


def choose_epithet(tags: list[str], sentiment: float) -> EpithetCandidate:
    """Rule-based выбор из словаря по топ-тегам."""
    counts: dict[str, int] = {}
    for tag in tags:
        counts[tag] = counts.get(tag, 0) + 1
    if not counts:
        return EpithetCandidate("Странник", "default", 0.5)
    top_tag = max(counts, key=counts.get)
    candidates = EPITHET_LEXICON.get(top_tag, ["Странник"])
    # детерминированный выбор через hash(speaker_id) для стабильности
    return EpithetCandidate(candidates[0], top_tag, counts[top_tag] / sum(counts.values()))


def validate_epithet(candidate: EpithetCandidate, recent_msgs: list[str]) -> str:
    """Опциональный LLM-валидатор. None если LLM недоступен → fallback."""
    if not _llm_available():
        return candidate.label
    prompt = (
        f"Кандидат: {candidate.label} (тема: {candidate.source_cluster})\n"
        f"Последние реплики спикера: {recent_msgs[:5]}\n"
        f"Подходит ли кличка? Если да — верни её, если лучше другую — предложи из "
        f"{EPITHET_LEXICON.get(candidate.source_cluster, [])}. "
        f"Не длиннее одного слова. Без объяснений."
    )
    return _llm_call(prompt).strip() or candidate.label


def maybe_re_evaluate(speaker: SpeakerProfile, recent_messages: list[str], now: datetime) -> None:
    """Пересмотр эпитета при distinctive event."""
    # Защита от частого пересмотра: не чаще раза в N дней
    if speaker.last_epithet_review and (now - speaker.last_epithet_review) < timedelta(days=30):
        return
    # Distinctive: новая доминирующая тема >40%
    new_tags = extract_tags(recent_messages[-50:])
    old_tags = set(speaker.tags)
    new_top = new_tags[0] if new_tags else None
    if new_top and new_top not in old_tags:
        new_candidate = choose_epithet(new_tags, speaker.sentiment_score)
        validated = validate_epithet(new_candidate, recent_messages)
        speaker.epithet_history.append((now, speaker.epithet, validated, f"new_topic:{new_top}"))
        speaker.epithet = validated
        speaker.tags = new_tags
        speaker.last_epithet_review = now
```

### 4.6 Метрики качества (для будущего PR)

- **PAA (Persona Attribution Accuracy)** — как в AFA-paper
  (arxiv 2604.25022). «Из 100 диалогов в multi-user-сценарии — сколько
  раз робот правильно подставил нужную память?»
- **Коллизии эпитетов** — сколько раз двум спикерам был назначен
  одинаковый эпитет. Цель: 0.
- **Стабильность эпитета** — медианное время жизни одного эпитета
  (в днях) до первого пересмотра. Цель: > 30 дней без явного
  запроса юзера.
- **Satisfaction proxy** — если юзер явно просит сменить эпитет
  («робот, я теперь Кадет»), логируем как failure: предыдущая
  система не угадала.

---

## 5. Открытые вопросы и риски

### 5.1 Латентность на «новом спикере»

Выбор эпитета требует N первых реплик. Варианты:
1. Ждать N (5–10) реплик — спикер какое-то время «без клички».
2. Сразу выбрать placeholder «Странник» по дефолту, обновить после N.
3. Назначать эпитет по **одной** первой реплике, если она содержит
   яркий сигнал («я люблю шахматы» → «Гроссмейстер»).

Рекомендую (2) для v1: пользователь видит кличку сразу, она может
поменяться позже — это нормально и показано как эволюция.

### 5.2 Что делать с юзером, который меняет имя каждый визит

«Робот, я теперь Кадет» → перебить эпитет? Или завести нового
спикера? AFA-paper не покрывает. Предлагаю: **это явный override**,
записываем в `epithet_history` с `reason="user_override"`. Не
создаём нового спикера — embedding тот же.

### 5.3 «Эпитет прозвучал в голосовом ответе»

Опасный сценарий: робот говорит «привет, Гроссмейстер» — а юзер
никогда не слышал этого слова. Решение: эпитет **никогда** не
озвучивается в TTS-ответах без явной команды юзера. Только в
логах, в memory_context для LLM, в debug-выводе.

### 5.4 Безопасность / персона и вредонос

Из arxiv 2506.06254: персона может использоваться для legitimation
вредоносных запросов («как Гроссмейстер, я имею право …»). Для rob_box
не блокер (мы не в multi-user-домене с критическими действиями), но
в ADR отметить: **эпитет не влияет на политику безопасности**.

### 5.5 Миграция существующих 42 спикеров

`ALTER TABLE ADD COLUMN` — null допустимо. Старые спикеры останутся
без эпитета, пока не накопят данные. Никакого backfill'а не нужно.

### 5.6 LLM-валидатор как зависимость

Если LLM недоступна (offline, rate-limit), epithet-generator должен
падать на rule-based слой **gracefully**. Не блокировать голос.
Это контракт ADR-0008 (capability-honest, no silent degradation).

---

## 6. План реализации (для будущего PR, не блокируем голос)

1. **Миграция БД** — отдельный PR, только schema. Тест:
   `ALTER TABLE speakers ADD COLUMN …` + rollback.
2. **Словарь `epithets.json`** — 200–500 кличек, разметка по 4 осям.
   PR с data + unit-тест на rule-based выбор.
3. **Tag-extractor** — NLTK / простой keyword-list. Отдельный PR.
4. **`SpeakerDatabase.extend()`** — добавление `epithet`, `tags`,
   `epithet_history`. API: `set_epithet(speaker_id, epithet, reason)`,
   `re_evaluate_epithet(speaker_id, messages)`. PR.
5. **LLM-валидатор** — отдельный модуль `epithet_validator.py`,
   тянет LLM-провайдер из существующей инфры. PR.
6. **Wiring в `speaker_id_node`** — при идентификации нового спикера
   вызывать `maybe_re_evaluate()`. PR.
7. **Тесты** — unit (rule-based выбор, edge-cases), integration
   (multi-user сценарий, как в AFA, но в домашнем контексте).
8. **Метрики** — PAA, коллизии, стабильность эпитета. Экспорт в
   существующий health-monitor / observability.

Оценка: 5–7 PR среднего размера (по правилам ADR-0013 — incremental).

---

## 7. Что **не** входит в этот research

- **Реализация.** Эта задача — только дизайн. Любой код
  реализации — отдельный kanban-карточка + отдельный PR.
- **Блокировка текущих голосовых фичей.** Никаких изменений в
  `dialogue_node`, `speaker_id_node`, MCP tools в этом PR.
- **Изменение схемы идентификации.** `d-vector` + `IDENTIFY_THRESHOLD`
  остаются как есть. Эпитет — параллельное поле.
- **GUI для юзера.** Никаких новых кнопок в UI. Эпитет — внутренняя
  метка для робота и разработчика.

---

## 8. Источники (резюме)

1. **arxiv 2604.25022** — AFA: Identity-Aware Memory for Preventing
   Persona Confusion in Multi-User Dialogue (апр 2026) —
   https://arxiv.org/abs/2604.25022
   *(ровно наша проблема, multi-user)*
2. **arxiv 2406.13960** — SPDA: Self-evolving Personalized Dialogue
   Agents (2024) — https://arxiv.org/html/2406.13960v1
   *(эволюция персоны во времени)*
3. **arxiv 2406.05925** — Hello Again! LLM-powered Personalized Agent
   for Long-term Dialogue (2024) —
   https://arxiv.org/abs/2406.05925
   *(три уровня памяти, долговременная)*
4. **arxiv 2506.06254** — PersonaAgent (jun 2025) —
   https://arxiv.org/abs/2506.06254
   *(персона как system prompt, цикл refine)*
5. **Stanford Smallville / Generative Agents** (Park et al., 2023) —
   https://github.com/joonspk-research/generative_agents
   *(reflection как прообраз эпитета)*
6. **arxiv 2608.26005** — VoiceMem: Streaming Dual-Brain Memory for
   Real-Time Interaction (авг 2026) — https://arxiv.org/pdf/2608.26005
   *(left/right brain — факты vs persona)*
7. **Multi-Anchor Persistent Identity** (soul.py, mar 2026) —
   https://www.researchgate.net/publication/403790183_Persistent_Identity_in_AI_Agents
   *(SOUL.md, multi-anchor)*
8. **AI Agent Persona Design and Behavioral Consistency** (Zylos,
   apr 2026) — https://zylos.ai/research/2026-04-10-ai-agent-persona-design-behavioral-consistency/
   *(identity отдельно от task)*
9. **Persistent NPC Memory for Games** (MistScale, июл 2026) —
   https://mistscale.com/blog/persistent-memory-for-game-npcs
   *(stable layer, conflict resolution)*
10. **Character.AI Memory Architecture** (konshus.ai, июл 2026) —
    https://konshus.ai/character-ai-memory-guide
    *(трёхслойная память в продукте, drifting context window)*

---

## 9. ADR / disposition

Это **research**-документ, не ADR. По результатам обсуждения с Шифу
— либо:
- превращаем в ADR (формат `NNNN-voice-epithet-design.md` в
  `docs/adr/`, по правилам ADR-0030 — следующий свободный номер),
- либо оставляем как `docs/research/` и архивируем после согласования
  дизайна.

Не блокируем ни одну текущую задачу. Реализация — после явного
«вперёд» от Шифу.

---

## 10. Что реализовано (v1, issue #1787)

Шифу дал «вперёд» на гибрид. Реализовано в `develop` одним PR — оба слоя,
как в §4.1.

### 10.1 Модули

| Файл | Что делает |
|---|---|
| `src/rob_box_voice/rob_box_voice/core/epithets.py` | Словарь (19 кластеров, 231 кличка), `extract_tags`, `score_sentiment`, `choose_epithet`, `build_llm_prompt`, `sanitize_llm_epithet`, правила пересмотра. Чистый stdlib |
| `utils/speaker_embeddings.py` | Миграция схемы (+`epithet`, +`epithet_history`, +`tags`, +`sentiment_score`, +`last_epithet_review`), `set_epithet`, `taken_epithets`, `get_speaker_profile`, `update_speaker_stats` |
| `speaker_id_node.py` | Владеет БД: копит окно из 50 реплик спикера, назначает кличку, публикует запрос к LLM, принимает и валидирует её ответ |
| `dialogue_node.py` | Публикует реплики опознанного спикера, зовёт LLM на выдумывание клички, отдаёт эпитет в `<user_profile>` |

### 10.2 Пайплайн (топики)

```
dialogue_node  --/voice/speaker/observe-->        speaker_id_node
                  {speaker_id, text}                    |
                                            слой 1: словарь → кличка в БД
                                                        |
dialogue_node  <--/voice/speaker/epithet_request--------+
      |            {speaker_id, fallback, cluster, hints, messages}
   слой 2: LLM придумывает одно слово
      |
      +--/voice/speaker/epithet-->  speaker_id_node → валидация → БД
```

### 10.3 Отличия от §4.5 (и почему)

1. **Порядок слоёв: сначала словарь пишется в БД, потом спрашиваем LLM.**
   В §4.5 LLM была валидатором на пути назначения. Так робот оставался бы
   без клички на время сетевого запроса — и совсем без неё при офлайне.
   Теперь словарный кандидат ложится в профиль сразу, а ответ LLM —
   отдельное событие, которое просто переименовывает. Отказ сети,
   таймаут или мусор в ответе не имеют последствий (ADR-0008).
2. **Уникальность обеспечивается кодом, а не моделью.** `choose_epithet`
   и валидатор LLM-ответа получают список уже занятых кличек. Коллизия
   «два Дениса-шахматиста → оба Гроссмейстеры» (§4.2 — главный аргумент
   за LLM) закрыта детерминированно. LLM осталась ради живости имени, а
   не ради корректности.
3. **Словарь — Python-константа, не `data/epithets.json`.** `data/` без
   `__init__.py` не попадает в `find_packages()`, потребовал бы строки в
   `data_files` и resolve пути на роботе. Выигрыша ноль, точка отказа
   лишняя.
4. **Модуль лежит в `core/`, а не в `utils/`.** `utils/__init__.py` тянет
   pyaudio через `audio_utils`, а `pyaudio` не объявлен в rosdep
   (ставится pip-ом) — импорт из `dialogue_node` мог бы уронить сборку.
   `core/` — ровно «pure Python без ROS», куда модуль и просится.
5. **Кличка из НОВОЙ темы при пересмотре.** Старая тема обычно ещё
   лидирует по упоминаниям в окне; без явной перестановки кандидатов
   пересмотр возвращал кличку из прежнего кластера, то есть ту же самую.

### 10.4 Границы v1

- Эмоциональный окрас (§4.1, ось 2) — **лексический**, не просодический:
  VAD/prosody не подключены, `sentiment_score` считается по словарю слов.
  Влияет только на выбор дефолтного пула у спикера без тем.
- `user_override` («робот, теперь я Кадет», §5.2) — константа причины
  есть, голосовой команды нет.
- Метрики §4.6 (PAA, стабильность) в health-monitor не экспортируются.
- Тесты: 49 unit-тестов на словарь, хранение и обвязку `speaker_id_node`.
  Путь `dialogue_node` (`_on_epithet_request` / `_generate_epithet`)
  тестами **не покрыт** — импорт `DialogueNode` требует ROS-окружения.
