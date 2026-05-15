# Стратегия рефакторинга dialogue_node.py

**Документ:** Аудит и план декомпозиции
**Фаза:** 3 (Code Quality Review) — только документирование, не реализация
**Реализация:** Milestone 3 (после навигации и Nav2 интеграции)

---

## Текущее состояние

| Метрика | Значение |
|---------|----------|
| Файл | `src/rob_box_voice/rob_box_voice/dialogue_node.py` |
| Строк | 2040 |
| Классов | 1 (`DialogueNode`) |
| Методов | 73 |
| Test coverage | 0% (htmlcov) / 13.4% (coverage.json, 1 тест) |
| Пустых тест-методов | 13+ (`test_dialogue_node.py`) |
| Tech Debt ID | TECH_DEBT.md TD-1 |
| CONCERNS.md | TD-1 (HIGH, defer → M3) |

---

## Проблемы текущей архитектуры

### 1. God Object Anti-Pattern
73 метода в одном классе охватывают 7 независимых responsibility:
- Конфигурация и загрузка параметров
- Построение LLM-агента
- Агентный цикл и retry-логика
- Управление историей диалога
- DJ-режим
- Инструменты агента (tools API)
- ROS callbacks и State Machine

### 2. Тестируемость близка к нулю
- Невозможно unit-тестировать отдельные компоненты без инициализации всего `DialogueNode`
- `DialogueNode.__init__` выполняет 20+ операций; построение тестовой фикстуры невозможно без ROS окружения
- Результат: 0% coverage на 2040 строках production-кода

### 3. Контроль контекста LLM (PF-2)
Рост context window во время agent run происходит в неизолированном коде. Без выделенного `ConversationHistory` класса невозможно применить стратегии усечения и мониторинга.

### 4. Сложность сопровождения
Изменение LLM провайдера требует понимания 5+ методов (`_build_agent`, `_make_tools`, `_make_output_tools`, `_build_skills`, `_call`). Добавление нового "skills" — аналогично.

---

## Предлагаемые модули (декомпозиция)

### Архитектура после рефакторинга

```
rob_box_voice/
├── dialogue_node.py           # ~300 строк: оркестратор (ROS glue + state machine)
├── config/
│   └── voice_assistant_config.py    # VoiceAssistantConfig
├── events/
│   └── event_profile_loader.py      # EventProfileLoader
├── agent/
│   ├── agent_factory.py             # AgentFactory
│   └── agent_runner.py              # AgentRunner
├── history/
│   └── conversation_history.py      # ConversationHistory
└── dj/
    └── dj_mode_manager.py           # DjModeManager
```

### 1. VoiceAssistantConfig

**Текущие строки:** 265–415 (методы `_load_system_prompt`, `_load_prompt_file`, `_resolve_api_key`, `_resolve_base_url`, `_resolve_model`)

**Responsibility:** Загрузка и валидация параметров конфигурации: API ключи, модели, пути к файлам промптов.

```python
@dataclass
class VoiceAssistantConfig:
    llm_provider: str
    llm_model: str
    api_key: str
    base_url: str
    system_prompt: str

    @classmethod
    def from_ros_params(cls, node: rclpy.node.Node) -> "VoiceAssistantConfig": ...
    def load_system_prompt(self, prompt_file: str, persona: str) -> str: ...
    def resolve_api_key(self, provider: str) -> str: ...
```

**Тестируемость после:** ✅ Нет ROS зависимостей в бизнес-логике. Конфигурацию можно создать из словаря для тестов.

### 2. EventProfileLoader

**Текущие строки:** 415–553 (методы `_load_event_profile`, `_slugify_event_id`, `_init_faq_store`, `_render_event_instructions`, `_build_event_faq_prefetch_context`, `_render_faq_skill_prompt`)

**Responsibility:** Загрузка event-профилей из YAML, инициализация FAQ store, рендеринг инструкций.

```python
class EventProfileLoader:
    def __init__(self, event_config_path: str): ...
    def load_profile(self, event_id: str) -> EventProfile: ...
    def init_faq_store(self, faq_path: str) -> FAQStore: ...
    def render_instructions(self, profile: EventProfile) -> str: ...
```

**Тестируемость после:** ✅ Чистый ввод-вывод с файловой системой. Тесты через временные YAML-файлы (`tmp_path` pytest fixture).

### 3. AgentFactory

**Текущие строки:** 554–1394 (методы `_build_agent`, `_make_tools`, `_call`, `speak_text`, `play_sound`, `play_animation`, `memory_context`, `memory_save`, `memory_search`, `faq_search`, `navigate_to_waypoint`, ... `_make_output_tools`, `_build_skills`)

**Responsibility:** Построение LLM-агента с его инструментами (tools API) и skills.

```python
class AgentFactory:
    def __init__(self, config: VoiceAssistantConfig, ros_bridge: ROSBridge): ...
    def build_agent(self, mode: AgentMode) -> Agent: ...
    def build_tools(self, ros_bridge: ROSBridge) -> list[Tool]: ...
    def build_dj_tools(self, dj_manager: DjModeManager) -> list[Tool]: ...
```

**Тестируемость после:** ✅ `ROSBridge` — injectable dependency. Можно тестировать с мок-бриджем без ROS.

### 4. AgentRunner

**Текущие строки:** 1507–1670 (методы `_run_agent_with_retry`, `_agent_run`)

**Responsibility:** Запуск агентного цикла, retry-логика, отмена.

```python
class AgentRunner:
    def __init__(self, agent: Agent, max_retries: int = 3): ...
    async def run(self, input_text: str, history: list) -> AgentResult: ...
    def cancel(self) -> None: ...
```

**Тестируемость после:** ✅ Agent инжектируется. Retry-логику можно тестировать с мок-агентом, имитирующим ошибки.

### 5. ConversationHistory

**Текущие строки:** 1718–1795 (методы `_trim_history`, `_truncate_history_outputs`, `_split_into_chunks`)

**Responsibility:** Управление историей диалога: добавление, усечение по токенам, ротация.

```python
class ConversationHistory:
    def __init__(self, max_tokens: int = 32000, max_messages: int = 50): ...
    def add(self, role: str, content: str) -> None: ...
    def trim(self) -> int:  # returns number of removed messages
        ...
    def truncate_outputs(self, max_output_chars: int = 500) -> None: ...
    def to_list(self) -> list[dict]: ...
```

**Тестируемость после:** ✅ Pure Python, никаких внешних зависимостей. 100% coverage реален.

### 6. DjModeManager

**Текущие строки:** 1838–2020 (методы `_build_dj_prompt`, `_on_dj_mode_msg`, `_on_dj_tick_check`)

**Responsibility:** Управление DJ-режимом: состояние, промпт, тик.

```python
class DjModeManager:
    def __init__(self, config: DJConfig, ros_publisher): ...
    def activate(self, persona: str, theme: str) -> None: ...
    def deactivate(self) -> None: ...
    def tick(self) -> Optional[str]:  # returns text to speak or None
        ...
    def build_prompt(self) -> str: ...
```

**Тестируемость после:** ✅ ROS publisher инжектируется как dependency. Тест с мок-publisher.

---

## DialogueNode после рефакторинга

`dialogue_node.py` становится ~300-строчным оркестратором:

```python
class DialogueNode(Node):
    def __init__(self):
        super().__init__("dialogue_node")
        self._config = VoiceAssistantConfig.from_ros_params(self)
        self._event_loader = EventProfileLoader(self._config.event_config_path)
        self._history = ConversationHistory(max_tokens=self._config.max_tokens)
        self._dj_manager = DjModeManager(self._config.dj_config, self._tts_pub)
        self._agent_factory = AgentFactory(self._config, ros_bridge=self._ros_bridge)
        self._agent_runner = AgentRunner(self._agent_factory.build_agent())

    # ROS callbacks — остаются здесь (~100 строк):
    def _on_vad(self, msg): ...
    def _on_stt(self, msg): ...
    def _on_tts_finished_dlg(self, msg): ...
    def _on_sound_state(self, msg): ...
    def _cancel_run(self): ...
    def _publish_state(self): ...
```

---

## Порядок реализации (Milestone 3)

Критический путь — сначала компоненты без зависимостей:

| Шаг | Компонент | Зависит от | Тесты |
|-----|-----------|------------|-------|
| 1 | `ConversationHistory` | Ничего | 100% (pure Python) |
| 2 | `VoiceAssistantConfig` | dataclasses | 90%+ (мок-params) |
| 3 | `EventProfileLoader` | VoiceAssistantConfig | 85%+ (tmp_path) |
| 4 | `DjModeManager` | VoiceAssistantConfig, мок-publisher | 70%+ |
| 5 | `AgentFactory` | Config, EventProfileLoader, ROSBridge | 60%+ (мок-bridge) |
| 6 | `AgentRunner` | Agent interface | 80%+ (мок-agent) |
| 7 | `DialogueNode` (оркестратор) | Все выше | 50%+ |

---

## Прогнозируемые метрики

| Метрика | Сейчас | После рефакторинга |
|---------|--------|--------------------|
| dialogue_node.py строк | 2040 | ~300 |
| Метрика test coverage (суммарная) | 0–13% | 60%+ |
| Среднее время unit-теста | N/A | <100ms |
| Модулей для независимого тестирования | 0 | 5 из 6 |

---

## Dependency Graph

```
DialogueNode
├── VoiceAssistantConfig (нет зависимостей снаружи)
├── EventProfileLoader → VoiceAssistantConfig
├── ConversationHistory (нет зависимостей снаружи)
├── AgentFactory → VoiceAssistantConfig, EventProfileLoader, ROSBridge
│   └── (создаёт Agent с tools и skills)
├── AgentRunner → Agent (от AgentFactory)
└── DjModeManager → VoiceAssistantConfig, ROS publisher
```

---

## Связанные документы

| Документ | Ссылка |
|----------|--------|
| Tech Debt Register | `.planning/TECH_DEBT.md` TD-1 |
| Coverage Report | `.planning/COVERAGE_REPORT.md` |
| CONCERNS.md (исходный) | `.planning/CONCERNS.md` TD-1, BUG-12, PF-2 |

---
*Документ создан: Phase 3 Milestone 1 (аудит)*
*Реализация: Milestone 3 (после навигации)*
