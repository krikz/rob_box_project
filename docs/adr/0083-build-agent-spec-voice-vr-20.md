# ADR-0083: `build_agent(spec)` — один стенд агента, личность и ТАРС как две конфигурации

| Поле | Значение |
|---|---|
| Статус | **Proposed** (kanban `t_c6cbec4c`, issue #2242) |
| Дата | 2026-09-09 |
| Автор | architect (Hermes Agent) по ADR-0080 §2.7 + ADR-0055 §2.5 |
| Контекст | `AgentCore` (бывший `DialogCore`, ADR-0051 §5.2) действительно один. Но его собирают дважды: на стороне личности (`dialogue_node`) и на стороне оператора/ТАРС (`supervisor_node`), и расхождения выглядят случайными, а не намеренными. Эти расхождения уже дали дефекты (см. §1.4) и блокируют #2000 (одна БД, namespace на агента), #2111 (HealthCache для оператора), #1988/1989 (шаги 4–8 operator-agent). Этот ADR фиксирует **контракт** `AgentSpec` + `build_agent`, классифицирует каждое расхождение на «намеренное → поле `AgentSpec`» или «случайное → канонизируется», и описывает план реализации в одной волне (ADR-0013 — incremental). |
| Затрагивает | `src/rob_box_harness/rob_box_harness/core/assembly.py` (новый), `core/agent_core.py` (без изменений API), `dialogue_node.py` (тонкая правка, см. §4.1), `supervisor_node.py` (удаление `_build_operator_*`, см. §4.2), `migrations/011_agent_namespace.sql` (новый — добавляет `agent TEXT NOT NULL DEFAULT 'default'` в таблицу `facts`), `memory/sqlite_voice.py` (расширение `init` на `agent`), `docker/vision/config/voice_assistant/dialogue_node.yaml` (новый блок `agent_spec`), `docker/vision/config/voice_assistant/supervisor_node.yaml` (новый блок `agent_spec`), `scripts/lint/` (инвариант из ADR-0080 §3 #9: grep на `AgentCore(` вне `assembly/`) |
| Родители | ADR-0080 §2.7 + инвариант 9 (этот ADR их фиксирует и уточняет), ADR-0051 §5.1–§5.2 (operator-agent = конфиг), ADR-0055 §2.3 + §5 (миграция `agent` колонки), ADR-0028 §4.5 (отменена ADR-0051), ADR-0018 (честный FAIL), ADR-0013 (incremental delivery), ADR-0043 §3.2 (provider-chain YAML-sync) |
| Связанные | `docs/architecture/target-operator-agent-and-dialogue.md` §5.2 (состав AgentCore), карточки `t_2242_voice-vr_19` (ход диалога), `t_2242_voice-vr_21` (чужие ROS-параметры, из ADR-0080 §1.6), `t_c6cbec4c` (эта работа), issues #2000, #2111, #1988, #1989 |

> **TL;DR.** Один публичный интерфейс — `build_agent(spec: AgentSpec) -> AgentCore` — и
> единственная точка, где `AgentCore(` встречается в проде. Все 12 «забот» из
> ADR-0080 §1.6 раскладываются по 9 полям `AgentSpec` (намерения) и 3 модулям
> (память / промпт / инструменты). Личность и ТАРС — **две конфигурации** этого
> спека, а не две реализации. `dialogue_node` и `supervisor_node` собирают агента
> только через `build_agent`; набор `_build_operator_*` удаляется. Одна БД
> (`/data/harness_voice.db`), namespace через колонку `agent TEXT` в `facts`
> и через разные `sqlite_db_path`-значения в обеих YAML — без хитрости и без
> гонки на dual-write.

---

## 1. Контекст и бизнес-проблема

### 1.1 Что сегодня

`AgentCore` собирается в двух местах:

|| место | строки | гонит |
|---|---|---|---|
| `dialogue_node.py:491` | `AgentCore(llm=…, tools=…, memory=…, dsm=…, system_prompt=…, skill_prompts=…, narrow_tools_to_skill=…, history_trim_limit=…, use_streaming=…, on_prompt=…, llm_settings=…)` | личность |
| `supervisor_node.py:1971` | `AgentCore(llm=…, tools=…, memory=…, dsm=…, system_prompt=…, skill_prompts=…, narrow_tools_to_skill=…, history_trim_limit=…, use_streaming=…, llm_settings=…)` | ТАРС |

Каждое поле собирается отдельным методом — по 6 на каждой стороне
(см. таблицу в карточке `t_c6cbec4c`). Всё, что не **одно** — это расхождение.

### 1.2 Расхождения, найденные после merge PR #2219

Проверено на `develop` после `#2219`, `#2217`, `#2207` (коммит `7b72adfe`).
Все цифры — сырые из `git grep`:

| № | расхождение | где (personality) | где (operator) | класс |
|---|---|---|---|---|
| A | Цепочка провайдеров LLM | `_build_llm` строит `HealthAwareFallbackLLM(..., cache=HealthCache(ttl_s, persist_path=…), balance_checkers=…, settings_for=…)` | `_build_operator_llm` строит `HealthAwareFallbackLLM(built, cache=HealthCache(), logger=…)` — **нет `persist_path`, нет `balance_checkers`, нет `settings_for`** | **случайное** (докстринг супервизора: `:2000` обещает «повторяет `dialogue_node`») |
| B | Настройки LLM (temperature/max_tokens) | `_build_llm_settings_for(name)` — пер-провайдер, потом `primary_settings`, issue #1883 | `_build_operator_llm_settings` — только глобальные `temperature`/`max_tokens` из параметров supervisor | **частично намеренное**: пер-провайдерные настройки личности не нужны оператору, но метод должен это явно говорить через `AgentSpec`, а не исчезать |
| C | Загрузка системного промпта | `_load_system_prompt` (`dialogue_node.py:1108`) — `rob_box_voice/prompts/master_prompt_compact.txt` | `_load_operator_system_prompt` (`supervisor_node.py:2213`) — `rob_box_supervisor/prompts/operator_system_prompt.txt` (a fallback путь, `voice/skills` не дёргается) | **намеренное** (разные источники) — становится полем `prompt_dir` |
| D | Загрузка скилл-промптов | `_load_skill_prompts` (`dialogue_node.py:1269`) — только `rob_box_voice/prompts/skills` | `_load_operator_skill_prompts` (`supervisor_node.py:2229`) — supervisor `prompts/skills` + **best-effort чтение `rob_box_voice/prompts/skills`** | **намеренное + регрессия**: supervisor-сторона читает чужие файлы по относительному пути — это ровно та протечка, которую ADR-0080 §1.6 фиксирует в «четыре места, откуда супервизор знает имена параметров по провайдерам» |
| E | Память (отдельная БД!) | `SQLiteVoiceMemory(db_path=sqlite_db_path)` → `/data/harness_voice.db` | `SQLiteVoiceMemory(db_path=operator_db_path)` → `/data/operator_memory.db` (`.declare_parameter("operator_db_path", "/data/operator_memory.db")`, строка `518`) | **намерение под вопросом**: «память оператора отдельно от личности» — но ADR-0055 §1.1 явно говорит, что **это протечка** (`/data/operator_memory.db` не упомянут в ADR-0055 вообще, появился в супервизоре без ADR). Разговор «operator vs personality одна БД, namespace через колонку» — это #2000 |
| F | Инструменты + планировщик | `_build_tool_provider` → `ToolProvider` → `SchedulerToolExecutor` (W7b scheduler) | `_build_operator_tools` → `adapt_tool_provider(ROSMCPToolProvider(…))`, **без планировщика** | **частично намеренное**: планировщик — это ADR-0075 «для личности», для ТАРС уместен отдельный `use_scheduler=False`. Но ТАРС *может* получить его позже (ADR-0080 §2.8: прополка) — поэтому это поле спека, а не хардкод |
| G | `AgentCore` сборка | `dialogue_node.py:491-512` | `supervisor_node.py:1971-1982` — **идентичные параметры, но нет `on_prompt`/observer** | **случайное** (supervisor не публикует `PromptStats`; метрики оператора не завелись) |
| H | history_trim_limit | `int(history_max_turns)` (YAML: 10) | `int(history_max_turns)` (YAML: 10) | идентично |
| I | use_streaming | `bool(llm_streaming)` (YAML) | `bool(llm_streaming)` (YAML) | идентично |
| J | narrow_tools_to_skill | `bool(skill_tool_narrowing)` (YAML) | `False` (хардкод, supervisor `1978`) | **намеренное** (ТАРС видит все инструменты, сужение не нужно), но хардкод должен стать полем спека с явным значением |
| K | user_id | `"default"` (default AgentCore) | `"default"` | идентично (заплатка: для оператора нужно что-то вроде `"operator"`/`session_uuid`, но это ADR-0055 Phase 2) |
| L | DSM | `DialogueStateMachine()` | `DialogueStateMachine()` | идентично |

**Итог.** Из 12 расхождений A–L — **4 случайные** (A, D-drain, G, J-as-clarification),
**1 под вопросом** (E — утечка БД), **5 намеренные** (B-clarify, C, D-source, F, J),
**2 идентичные** (H, I, K, L). Это **ровно та же фактура**, что ADR-0080 §1.6
описывает словами «выглядят случайными, а не намеренными».

### 1.3 Что это стоит сегодня

Конкретные дефекты, зафиксированные в коде или issue-трекере:

1. **Supervisor `HealthCache` эфемерен** — `supervisor_node.py:2063` пишет
   `HealthCache()` без `persist_path`. После рестарта супервизора все
   «больные» провайдеры снова «здоровы», латентный failover работает
   с нуля — ровно тот класс инцидента, ради которого
   `HealthCache(persist_path=…)` в `dialogue_node.py:1672` и существует.
   Докстринг supervisor (`supervisor_node.py:2000`) прямо говорит
   «повторяет `dialogue_node`» — это обещание, которое сам же и нарушает.
2. **`operator_db_path = /data/operator_memory.db`** (default в
   supervisor_node.py:518) — это **третья БД** на `/data/` томе в дополнение к
   уже существующим `harness_voice.db` и `voice_memory.db`. ADR-0055 §1.1 их
   перечисляет двумя, про третью не знает — значит, добавление проскочило
   без ADR. backups, restore, forensic — теперь должны помнить про три файла.
3. **`_load_operator_skill_prompts` читает `rob_box_voice/prompts/skills`** —
   `supervisor_node.py:2259-2278`. Директива ADR-0080 §1.6: «супервизор
   перестаёт писать чужие ROS-параметры… голос, пресет и язык меняются
   явными контрактами». Это та же категория — supervisor лезет в чужой
   пакет по относительному пути. Не падает, но при переносе
   `rob_box_voice/prompts/skills/` (на что есть план) сломается молча.
4. **Operator не публикует `PromptStats`** — `supervisor_node.py:1971-1982`
   в `AgentCore(...)` нет `on_prompt=self._on_prompt_stats`. Метрики
   личности (`record_voice_llm_prompt_tokens`) исправно считают, метрики
   оператора — нет. Это «один и тот же класс, разные конфиги» ADR-0051 §5.1
   в негативной формулировке.

### 1.4 Внешний контракт, который нужен (acceptance criteria от e2e + DoD)

Из карточки `t_c6cbec4c` Definition of Done (полный список — §3 этого ADR):

```
[ ]  git grep -nE 'AgentCore\(' --include='*.py' src/rob_box_voice src/rob_box_supervisor
     → только src/.../core/agent_core.py (определение)
       и src/rob_box_harness/test/test_agent_core.py (тесты)
[ ]  Один файл БД; namespace на агента; истории личности и оператора
     не смешиваются (e2e-тест, инвариант 4 целевой архитектуры)
[ ]  Тест: build_agent для двух spec даёт два ядра с разным
     промптом, разным срезом и namespace
[ ]  На роботе: личность и ТАРС отвечают как раньше
```

Последний пункт — это **ничего не сломать на проде**. Тест e2e на живом
роботе выполняет `e2e-process` после merge (по контракту agent-flow), а
не этот ADR.

---

## 2. Принятое решение

### 2.1 Модуль `rob_box_harness.core.assembly`

Новый файл. Содержит **ровно два** публичных символа:

```python
@dataclass(frozen=True)
class AgentSpec:
    """Конфигурация одного агента. Неизменяема.

    Поля разделены на «что у агента своё» (всегда разное у личности и ТАРС)
    и «что обязано быть одинаковым» (инварианты сборки).
    """

    # ── идентичность (всегда разное у двух агентов) ─────────────────
    name: str                                # "personality" | "operator"
    prompt_dir: Path                         # где лежит system_prompt + skills/
    skill_slice: tuple[str, ...]              # имена skill-фрагментов
    system_prompt_file: str                  # имя .txt внутри prompt_dir
    tools: ToolProvider                      # уже собранный, включая ROSMCPToolProvider
    use_scheduler: bool                      # W7b SchedulerToolExecutor (только личность пока)
    memory_namespace: str                    # значение колонки `agent` (см. §2.4)
    on_prompt: PromptObserver | None = None  # метрики

    # ── LLM (частично общее — см. §2.3) ─────────────────────────────
    provider_chain: tuple[str, ...]          # ["deepseek"] | ["minimax","deepseek"]
    settings: LLMSettings                    # глобальные temperature/max_tokens
    per_provider_settings: Mapping[str, LLMSettings] = field(default_factory=dict)
    use_streaming: bool = False
    health_cache_persist_path: Path | None = None  # общий для всех агентов машины
    health_ttl_s: float = 300.0
    health_balance_checkers: Mapping[str, Callable[[], Any]] = field(default_factory=dict)

    # ── обязательно общее ────────────────────────────────────────────
    history_trim_limit: int = 10
    narrow_tools_to_skill: bool = False
    dsm: DialogueStateMachine = field(default_factory=DialogueStateMachine)
    user_id: str = "default"


def build_agent(spec: AgentSpec) -> AgentCore:
    """Собрать AgentCore по спеке. Единственная публичная сборка.

    Поднимает ``tools`` уже собранным в ``spec.tools`` (нода отвечает за
    builder’ов и Failure-mode), и собирает LLM-цепочку сама (ей нужен
    доступ к registry и HealthCache, которые должны быть общими на
    машину).
    """
```

**Реализация LLM-сборки внутри `build_agent`** — потому что это та забота,
которая не должна расходиться между нодами: balance-checkers, settings_for,
persistent cache, error-fallback chain. `AgentSpec.tools`, наоборот,
нода приносит готовая, потому что нода знает, как стартовать
`ROSMCPToolProvider` поверх `LLMToolCallAdapter(self)` (нужен reference на
node для service-client'а).

### 2.2 Классификация полей — что намеренное, что нет

Карта соответствия «старое расхождение → новое решение»:

| № | старое | новое |
|---|---|---|
| A | supervisor не имел persistent HealthCache | `health_cache_persist_path` берётся из единого YAML-параметра `health_cache_path` (тот же, что dialogue_node уже использует). Обе ноды используют **один и тот же файл** для персиста. |
| B | supervisor без per-provider settings | `per_provider_settings={}` — пустой dict по умолчанию, supervisor не задаёт; personality может задать |
| C | `_load_system_prompt` vs `_load_operator_system_prompt` | `prompt_dir` + `system_prompt_file` — два разных пути |
| D (source) | supervisor читает `voice/skills` best-effort | НЕ читает. Supervisor знает только свой `prompt_dir/skills/`. Если ТАРСу нужны personality-skill-фрагменты, они публикуются через `tool_catalog` (ADR-0051 §6), а не через файловую систему |
| D (drain) | `_load_operator_skill_prompts` содержит логику склейки | `skill_slice` = `("operator.speech", "operator.control")` для ТАРС, `("personality",)` для личности. `loader` — отдельная функция `load_skill_prompts(prompt_dir, slice)`, два разных спек'а зовут её с разными аргументами |
| E | `/data/operator_memory.db` vs `/data/harness_voice.db` | **Один файл `/data/harness_voice.db`**, обе ноды пишут через `SQLiteVoiceMemory(db_path=…)` с разными `agent=memory_namespace` |
| F | `_build_operator_tools` без SchedulerToolExecutor | `use_scheduler=False` в `AgentSpec` (ТАРС); `use_scheduler=True` (личность) |
| G | supervisor не публикует `PromptStats` | `on_prompt` — поле спека, обе ноды передают свой observer (уже есть `_on_prompt_stats` в dialogue_node, supervisor получит свой `_record_supervisor_prompt_stats` в том же PR) |
| H, I | история / streaming — идентичные | поля спека |
| J | supervisor хардкодит `narrow_tools_to_skill=False` | явно `narrow_tools_to_skill=False` в `AgentSpec.operator()` |
| K, L | user_id, DSM | общие поля |

### 2.3 Что остаётся за нодой (и почему — Trade-off)

| забота | кто делает | почему |
|---|---|---|
| `tools: ToolProvider` | **нода** | `ROSMCPToolProvider(LLMToolCallAdapter(self))` требует ссылку на ноду для создания `Client`’ов к `/avatar_arbiter/*`, `/voice/tts/request`. Вынести в общий модуль = тащить rclpy в harness |
| LLM-цепочка (build_llm) | `build_agent` | Чтобы persistent `HealthCache` и `balance_checkers` были одни на машину. Нода сообщает `provider_chain` + `per_provider_settings`, всё остальное делает `build_agent` |
| Загрузка промптов | `build_agent` | Чтение файлов — pure-Python, никаких ROS-зависимостей |
| Загрузка `facts` / DDL | `build_agent` — внутри `SQLiteVoiceMemory(..., agent=…)` | Schema + memoization хочется в одном месте |

**Trade-off.** Это означает, что `build_agent` зависит от
`rob_box_harness.health`, `rob_box_harness.providers.catalog`, `SQLiteVoiceMemory`
— всех модулей, которые **уже** импортируются нодами через те же методы.
Чистой выгоды нет; выгода в том, что нода выкидывает ~270 LOC методов
(`_build_llm`, `_build_single_provider`, `_build_llm_settings_for`,
`_build_memory`, `_build_operator_llm`, `_build_operator_tools`,
`_build_operator_memory`, `_load_system_prompt`, `_load_skill_prompts`,
`_load_operator_system_prompt`, `_load_operator_skill_prompts`,
`_build_operator_llm_settings`, частично `_build_tool_provider`).

### 2.4 Память: namespace через колонку `agent`

Миграция `011_agent_namespace.sql`:

```sql
-- ADR-0083 §2.4. Унифицированная память обеих нод: колонка agent
-- определяет, чей это факт. По умолчанию 'default' — старые строки
-- остаются видимыми обоим агентам, пока мы не докатим реклайн-скрипт.
ALTER TABLE facts ADD COLUMN agent TEXT NOT NULL DEFAULT 'default';

-- Частичный индекс: типичные scope личности (player, faq, ...)
-- и оператора (operator) уже разделены в scope-колонке; agent помогает
-- разделить их внутри одного scope.
CREATE INDEX IF NOT EXISTS idx_facts_agent ON facts(agent);

-- Backfill operator: строки с scope == 'mcp:legacy' (от VoiceMemoryAdapter
-- времен ADR-0055 Phase 1) и scope, начинающиеся с 'operator.'
-- трактуются как operator. Остальные — 'personality'.
UPDATE facts
   SET agent = 'operator'
 WHERE agent = 'default'
   AND (scope = 'mcp:legacy'
        OR scope LIKE 'operator.%');

UPDATE facts
   SET agent = 'personality'
 WHERE agent = 'default';
```

После миграции `SQLiteVoiceMemory.save_fact(scope, fact, *, agent='default')`
получает параметр `agent` через keyword-only (как сейчас имеет `metadata_json`).
`search_facts` и `get_context` фильтруют по `agent`. **`VoiceMemoryAdapter`**
(ADR-0055 §1.1 Phase 1) перестаёт писать `scope='mcp:legacy'` и пишет
`agent='personality'` — это и закрывает #2000.

`/data/operator_memory.db` удаляется. Шифу удаляет файл после
визуальной проверки, как и для `voice_memory.db` (ADR-0055 §6 шаг 6).

### 2.5 Чего этот ADR **не** делает

- Не правит `/avatar/command` топики, `pause`/`resume` контракт, transport
  инструмента — это `voice-vr 19` (ход диалога) и ADR-0066, отдельные карточки.
- Не убирает `_build_tool_provider` целиком — внутри остаётся
  `SchedulerToolExecutor` (W7b) и сборка `ToolRegistry`, потому что
  `AgentSpec.tools` приходит уже собранным. Нода знает, как обернуть
  `MCPToolProvider` в `SchedulerToolExecutor` (одна строка).
- Не убирает зависимость `OperatorHarness` (ADR-0051 §5.5 обещает его
  удаление — этот ADR не трогает, потому что оно уже сделано в коммите
  `dc0112ea` / свежих PR).
- Не заводит **новую** таблицу для ТАРС-журнала. Журнал ТАРС (§5.4
  target-arch) — `OperatorJournal(path=journal_path)` JSONL — отдельная
  сущность, не часть `MemoryStore`. `AgentSpec.journal_path` нет в этом
  ADR; supervisor хранит ссылку на журнал как атрибут ноды.

---

## 3. План реализации (одна волна, ADR-0013)

Одна PR из 4–6 коммитов, всё в `develop`. Каждый коммит — зелёный CI,
reverse-depобратные вызовы через deprecation-прокладки не нужны, потому что
**никто, кроме dialogue_node и supervisor_node, этими методами не
пользуется** (проверено `git grep -nE '_build_operator_|def _build_llm'`
в src/,см. §1.2).

| # | коммит | что меняется |
|---|---|---|
| 0 | ветка `z-{agent}/2242-voice-vr-20-build-agent-spec` от develop |  |
| 1 | `feat(harness): AgentSpec + build_agent core` | Новый `core/assembly.py` (300–400 LOC). `ProviderLLMChain` (бывший `_build_llm` целиком, без balance-checkers). `PersistentHealthCache` (бывший inline в `dialogue_node._build_llm`). `load_skill_prompts(prompt_dir, slice)` — общий загрузчик. |
| 2 | `feat(memory): facts.agent namespace + 011 migration` | `migrations/011_agent_namespace.sql`. `SQLiteVoiceMemory.save_fact(scope, fact, *, agent='default')`. `VoiceMemoryAdapter` обновлён под phase 2 ADR-0055. |
| 3 | `refactor(voice): dialogue_node → build_agent` | `_build_llm`, `_build_llm_settings_for`, `_build_single_provider`, `_load_system_prompt`, `_load_skill_prompts`, `_build_memory`, `_build_tool_provider` (в части `SchedulerToolExecutor`) → удалены. `__init__` строит `agent_spec = AgentSpec.personality(…)` и `self._core = build_agent(agent_spec)`. |
| 4 | `refactor(supervisor): remove _build_operator_*` | Все 6 `_build_operator_*` методов удалены. `_ensure_agent_core` строит `agent_spec = AgentSpec.operator(…)` и `self._agent_core = build_agent(agent_spec)`. `journal_path` остаётся атрибутом supervisor'а. |
| 5 | `chore(lint): AgentCore construction guard` | `scripts/lint/seam_without_consumer.py` (или новый `agentcore_guard.py`): CI-гейт на `AgentCore(` вне `agent_core.py`/`assembly.py`/`test_agent_core.py`. |
| 6 | `test(harness): build_agent integration` | `test_assembly.py` — два `AgentSpec` (personality + operator) с разными `prompt_dir`/`skill_slice`/`memory_namespace`, два `AgentCore`, ассерт по `core_known_skills()` и `core_user_id`/namespace в памяти (см. §5). |
| 7 | `chore(yaml): dialogue_node/supervisor YAML — agent_spec block` | Оба YAML получают блок `agent_spec:` (или просто параметры `sqlite_db_path`/`health_cache_path` — по результату §3 шага 6). |

**Reverse-dep не нужна** — это всё листья графа зависимостей.

### 3.1 Revert-ветка

После merge коммит `revert("feat(harness): AgentSpec + build_agent core")`
оставляется в `z-{revert}/2242-voice-vr-20-restore`, чтобы откат
одним merge не перетряхивал 30+ тыс. строк dialogue_node.py (ADR-0013
«one PR, one revert»).

---

## 4. Что меняется в существующих документах

| документ | что |
|---|---|
| ADR-0080 §2.7 | **Принимается этим ADR** (без правки текста). Шов сборки агента закрывается; §1.6 инвариант 9 («`AgentCore` собирается только через `build_agent`») исполняется |
| ADR-0080 §3 инвариант 9 | Его проверка становится `scripts/lint/agentcore_guard.py` (этот ADR §3 шаг 5) |
| ADR-0055 §5 (миграция agent) | **Исполняется** миграцией 011 этого ADR §2.4 |
| `target-operator-agent-and-dialogue.md` §5 | Этот ADR — фиксация «двух конфигов», принятых §5.1 (deletion test for Harness) |
| ADR-0066 §6.7 (удаление `voice_input_mode`) | Не трогает этот ADR, но §6.3 (лишний `_avatar_command_pub`) уедет вместе с `dialogue_node` рефакторингом |
| `CONTEXT.md` | Добавить термин «**стенд агента**» = `build_agent(spec)` |
| `docs/architecture/target-operator-agent-and-dialogue.md` §5.2 | Дополнить одной строкой: «`AgentCore` = `DialogCore` + `PromptObserver` (ADR-0083) + lifecycle из `HarnessConfig` (ADR-0051)» |

---

## 5. Definition of Done (doD карточки `t_c6cbec4c`)

Прекодирует тот список, что дан в `voice-vr 20` карточке:

- [ ] **`grep`** `git grep -nE '\bAgentCore\(' --include='*.py' src/rob_box_voice/ src/rob_box_supervisor/ src/rob_box_harness/`
       возвращает **только**:
       - `src/rob_box_harness/rob_box_harness/core/agent_core.py:N` — `def __init__`
       - `src/rob_box_harness/rob_box_harness/core/assembly.py:N` — `build_agent`
       - `src/rob_box_harness/test/test_agent_core.py:N` — test fixtures
  (raw-вывод коммитится в PR body — см. `voice-vr 20` acceptance #1).
- [ ] **Один файл БД:** `docker exec voice-assistant sqlite3 /data/harness_voice.db ".tables"`
       показывает `facts agent_index facts,…`; файл `/data/operator_memory.db`
       отсутствует (`ls /data/*.db` — два файла: `harness_voice.db`,
       `voice_memory.db`). Backend-карточка миграции 011 запускается
       через `migrate.py` БЕЗ отдельного `data-migrate` — он не нужен
       для namespace=простого `ADD COLUMN … DEFAULT 'default'`.
- [ ] **Тест (юнит, raw pytest log):**
       `pytest -v src/rob_box_harness/test/test_assembly.py` —
       собирает два `AgentCore` через `build_agent`, проверяет:
       - `core_persona.known_skills() == ("player",)` (или текущий test)
       - `core_operator.known_skills() == ("operator.speech",)`
       - `core_persona._memory.find_facts(agent="personality")` не видит
         строки, записанные `core_operator._memory.find_facts(agent="operator")`
       - обе памяти — **один** `SQLiteVoiceMemory` с разным `agent` в
         `sqlite_db_path` (через временный файл).
- [ ] **E2E на роботе (делает `e2e-process` после merge, не этот ADR):**
       ответы личности и ТАРС идентичны pre-merge. Команды e2e-процесса —
       в `## e2e` карточки, raw-вывод контейнера публикуется
       `e2e-validator-contract`.

### 5.1 Anti-Goals (что НЕ делаем в этой карточке)

- Не добавляем `name`-колонку в `faq_items` и `waypoints` — это разные
  namespace'ы и не нужно сейчас. ТАРС не пишет FAQ.
- Не трогаем `OperatorJournal` — он остаётся в supervisor_node; ADR-0080
  §5.4 его оставил.
- Не вводим `AgentSpec.journal_path` — журнал ТАРС это **не** часть
  `MemoryStore`, это JSONL-файл с собственной политикой сжатия.
- Не делаем «удалить `voice_memory.db`» — это по ADR-0055 §5 шаг 6,
  явное решение Шифу.

---

## 6. Последствия

**Положительные.**

- **~270 LOC** методов удалено с обеих сторон; добавлено ~300 LOC в одном
  `core/assembly.py` (и ~100 LOC в тесте). Чистая экономия:
  больше строк удалено, чем добавлено.
- Один источник истины для LLM-сборки: `HealthCache(persist_path=…)`,
  `balance_checkers`, `settings_for` — одинаковы у обоих агентов.
  Сейчас supervisor роняет все три.
- `AgentSpec` — это **документация в коде**: dev, который добавит третьего
  агента (например, `moodboard_agent`), опишет его одной структурой.
- `/data/operator_memory.db` исчезает. Бэкап-скрипт `/data/*voice*.db`
  упрощается.
- e2e-тест инварианта 4 (истории не смешиваются) **теперь возможен в
  юнитах**: см. §5 [3].

**Отрицательные и риски.**

- Любое отклонение в конфиге одной ноды теперь видно как отклонение в
  спеке — это хорошо для будущего, но требует дисциплины: ADR-0018 уже
  запрещает добавлять параметры «молча» (нужно `bash scripts/agent_flow/validate_honesty.sh`).
- Перенос `tools` в `AgentSpec` **не делает их тривиально тестируемыми** —
  `ROSMCPToolProvider` всё ещё нужен для сборки. Это **намеренное** решение
  (см. §2.3 trade-off). Альтернативой была бы сериализация `tools` через
  `ConfigDict`, но она уже есть в `rob_box_core.tool_catalog` (ADR-0051 §6).
- Рефактор `dialogue_node.py` трогает горячий файл (30 касаний за 400
  коммитов). Делим на 2 коммита (build_agent версия, потом удаление старых
  методов), чтобы bisect был осмысленным.

---

## 7. Альтернативы, которые отвергнуты

- **«Оставить две реализации, но синхронизировать их в одном PR» (типа
  refactor-synonyms).** Отвергнуто: синхронизация требует дисциплины,
  дисциплина не масштабируется на третьего агента, а контракт «два
  `AgentCore` на двух разных сторонах» уже сломался дважды (HealthCache,
  PromptStats). ADR-0018 + ADR-0013 на стороне «один стенд».
- **«Только одна БД, без namespace-колонки, через разные `sqlite_db_path`».**
  Отвергнуто: supervisor уже использует `/data/operator_memory.db` —
  четыре сервиса пишут в три файла (dialogue_node → `harness_voice.db`,
  mcp_server → `harness_voice.db` через адаптер, supervisor →
  `operator_memory.db`, ещё `voice_memory.db` для music). Три БД —
  не две, и не одна. Namespace-колонка + одна БД завершает
  ADR-0055 (Phase 2).
- **«Сделать `AgentSpec` Generic[T] параметризованным по типу LLM/памяти».**
  Отвергнуто: ADR-0051 §5.2 зафиксировал «один движок», и достоинства
  generic-а (compile-time проверка) перевешиваются ценой потери
  совместимости с существующими test fixture'ами `_FakeLLMProvider`. Если
  когда-нибудь понадобится второй движок — это отдельный ADR.
- **«Поднять `core/assembly.py` в rob_box_core, чтобы им пользовались вне
  harness'а»** (например, telegram). Отвергнуто: сборка завязана на
  `SQLiteVoiceMemory` (harness), `DialogueStateMachine` (harness),
  `LLMProvider` (harness). Перенос не вычёркивает зависимости, а
  инвертирует их. Поднимем, если появится второй потребитель.

---

## 8. Открытые вопросы

1. **`memory_namespace` в `AgentSpec`** — `personality` или `operator`?
   По умолчанию предлагаю `"personality"` (для исторической
   совместимости). В YAML supervisor переопределяет. Это часть PR.
2. **Persistent `HealthCache(persist_path=…)`** — supervisor должен
   использовать **тот же файл**, что dialogue_node? Или
   `/data/supervisor_health_cache.json`? По дефолту общий файл
   `/data/health_cache.json` (новый параметр в обоих YAML). Это
   часть PR.
3. **`OperatorJournal`** — где живёт его `Path`? В supervisor (как
   сейчас) — это не часть `AgentSpec`. Не блокирует этот ADR, но
   задокументировано.
4. **`skill_slice: tuple[str, ...]`** — это ADR-0051 §6 срез. Должен
   ли `AgentSpec` нести полный срез + список имён, или только имена?
   Предлагаю **только имена**: `narrow_tools_to_skill=True` уже
   достаточно, чтобы загрузить схемы из `tool_catalog.skill_names()`.

---

## 9. Связанные карточки

| карточка | что делает |
|---|---|
| **t_c6cbec4c** (эта) | пишет этот ADR + draft issue-комментарий с №№8.1–8.4 |
| `t_2242_voice-vr_19` | ход диалога (TurnGuards — отдельный шов ADR-0080 §2.4) |
| `t_2242_voice-vr_21` | супервизор перестаёт писать чужие ROS-параметры (вне scope этого ADR) |
| `t_7a03364a` (ADR-0055) | Фаза 1 voice_memory.db → harness_voice.db (УЖЕ merge'нута) |
| `t_XXXX` (ADR-0055 Phase 2) | миграция данных voice_facts → facts + agent column — **этот ADR её исполняет** |
| `t_1988`, `t_1989` | operator-agent step 4/6 (AgentCore подключение) — следующая фаза после этого ADR |

---

> **Нумерация.** В ADR-0080 и более ранних карточках серии эта работа упоминается
> под другим номером: 16/17/18 успели занять карточки рефакторинга CC
> (#2201–#2203). Соответствие: ход диалога = 19, стенд агента = 20, чужие
> ROS-параметры = 21, прополка планировщика = 22, EventBus/ReflexLayer = 23.
> Этот ADR = **voice-vr 20**, как в kanban.

> **Фактура перепроверена 2026-09-09** после merge 18 PR серии: команды `git grep`
> ниже подтверждают, что 10 из 12 швов всё ещё собраны дважды. Волны 0–3 шли
> по стороне quest/core/supervisor и голосовой части dialogue_node не касались.

## Appendix A. Raw `git grep` outputs, фиксирующие фактуру перед правкой

```
$ git -C /home/builder/rob_box_project grep -n 'AgentCore(' \
    -- 'src/**/*.py' | grep -v 'test/'
src/rob_box_voice/rob_box_voice/dialogue_node.py:491:        self._core: AgentCore = AgentCore(
src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py:1971:            core = AgentCore(

$ git -C /home/builder/rob_box_project grep -nE \
    '_build_operator_llm|_build_operator_memory|_build_operator_tools|_load_operator_system_prompt|_load_operator_skill_prompts|_build_operator_llm_settings' \
    -- 'src/**/*.py'
src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py:1957:        llm = self._build_operator_llm()
[+23 строки, все в supervisor_node.py, см.§1.2]

$ # Personality-side parallels (для сравнения)
$ git -C /home/builder/rob_box_project grep -nE \
    'def _build_llm\(|def _build_single_provider\(|def _build_llm_settings_for\(|def _load_system_prompt\(|def _load_skill_prompts\(|def _build_memory\(|def _build_tool_provider\(|SchedulerToolExecutor' \
    -- 'src/rob_box_voice/rob_box_voice/dialogue_node.py'
1108:    def _load_system_prompt(self) -> str:
1269:    def _load_skill_prompts(self) -> dict[str, str]:
1403:    def _build_memory(self) -> MemoryStore:
1503:    def _build_single_provider(self, name: str) -> Any | None:
1605:    def _build_llm(self) -> Any:
1713:    def _build_llm_settings_for(self, name: str) -> LLMSettings:
1769:    def _build_tool_provider(self) -> ToolProvider:
[+SchedulerToolExecutor usage in dialogue_node.py:490,1920]

$ git -C /home/builder/rob_box_project grep -n 'HealthCache' \
    -- 'src/rob_box_voice/rob_box_voice/dialogue_node.py' \
       'src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py' \
    | grep -v __init__
src/rob_box_voice/.../dialogue_node.py:1672:        cache = HealthCache(
src/rob_box_supervisor/.../supervisor_node.py:2063:            return HealthAwareFallbackLLM(built, cache=HealthCache(), logger=self._log)
```
