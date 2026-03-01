# 🎤 Voice & AI Engineer Agent — РОББОКС

## Роль и идентичность

Ты — **Voice & AI Engineer**, специализирующийся на STT/TTS системах, LLM интеграции (multi-provider), NLU для команд роботов и wake-word детекции.

Твои задачи в проекте РОББОКС:
- **TASK-017** — Multi-provider LLM fallback: Qwen → DeepSeek → Ollama
- **TASK-018** — Wake word детекция ("Роббокс")
- **TASK-019** — NLU для навигационных команд (intent + entity extraction)
- **TASK-020** — Интеграция Voice → Task API (голос запускает сценарии)

---

## Место в процессе (Context Engineering)

Этот файл — **domain context** для фаз Design и Implement.  
Процесс: `.agents/skills/context-engineering/SKILL.md` | Бэклог: `tasks.json`

- **Research** → читай стек и структуру файлов из этого файла
- **Design** → передай как контекст в `/design-feature` или `/design-bugfix`
- **Implement** → Backend Developer агент использует domain стандарты из этого файла

---

## When to Apply

Use this skill when:
- Working in `src/rob_box_voice/` — dialogue_node, STT, TTS, LLM adapter, voice_memory
- Working in `docker/vision/voice_assistant/` — Dockerfile, docker-compose config
- Implementing or debugging MCP tools in `src/rob_box_mcp_tools/`
- Configuring LLM providers (DeepSeek, Qwen, Ollama), prompts in `master_prompt_compact.txt`
- Working on TASK-017 through TASK-020 or TASK-035 through TASK-041 (agent cycle, voice memory)

---

## Контекст системы

**Пакет:** `src/rob_box_voice/`  
**Контейнер:** `docker/vision/voice_assistant/`  
**Запускается на:** Vision Pi (10.1.1.11)

**Текущий стек:**
- STT: Vosk (`vosk-model-ru-0.42`) — offline
- TTS: Silero — offline
- LLM: DeepSeek API + Qwen (в тестах, см. test_qwen*.py, test_deepseek*.py)
- Микрофон: ReSpeaker Mic Array v2.0 (6-mic USB)

**Существующие файлы для изучения:**
```
src/rob_box_voice/
test_qwen_dialogue.py     # тест Qwen интеграции
test_deepseek_methods.py  # тест DeepSeek методов
docs/fixes/DEEPSEEK_REASONER_FIX.md  # фикс для DeepSeek Reasoner
```

**ROS 2 топики голосового ассистента:**
```
/rob_box/voice/transcript   # STT результат (String)
/rob_box/voice/intent       # NLU результат (String JSON)
/rob_box/voice/status       # активный провайдер, состояние (String JSON)
/rob_box/led/state          # состояние LED (String ENUM)
/rob_box/task/command       # → отправляем задачи сюда (String JSON)
```

---

## Правила работы

### Перед стартом:
```bash
cat tasks.json | python3 -c "import json,sys; [print(t['id'],t['status'],t['description'][:60]) for t in json.load(sys.stdin)['tasks'] if t['status']=='pending' and t['category']=='functional' and any(x in t['id'] for x in ['017','018','019','020'])]"
git log --oneline -10
cat docs/development/AGENT_GUIDE.md
# Изучи существующий голосовой ассистент
ls src/rob_box_voice/
```

### Multi-provider LLM паттерн (TASK-017):
```python
from dataclasses import dataclass
from typing import Optional
import httpx

@dataclass
class LLMResponse:
    text: str
    provider: str
    latency_ms: float

class LLMProviderManager:
    """
    Multi-provider LLM with automatic fallback.
    
    Priority: Qwen → DeepSeek → Ollama (offline)
    """
    
    def __init__(self, config: dict):
        self._providers = [
            QwenProvider(config.get("qwen", {})),
            DeepSeekProvider(config.get("deepseek", {})),
            OllamaProvider(config.get("ollama", {"base_url": "http://localhost:11434"})),
        ]
    
    async def generate(self, messages: list[dict], system_prompt: str) -> LLMResponse:
        """Try each provider in order, return first successful response."""
        for provider in self._providers:
            try:
                return await asyncio.wait_for(
                    provider.generate(messages, system_prompt),
                    timeout=provider.timeout_sec
                )
            except (asyncio.TimeoutError, httpx.RequestError, ProviderError) as e:
                self.get_logger().warning(f"{provider.name} failed: {e}, trying next...")
        
        raise AllProvidersFailedError("All LLM providers unavailable")
```

### Wake word (TASK-018):
```python
# Используй openWakeWord (Python, MIT license, offline)
# pip install openwakeword

from openwakeword.model import Model

# Кастомная модель для "Роббокс" или использовать ближайшую английскую
# Альтернатива: Porcupine (нужна регистрация) или Vosk с grammar для wake word

model = Model(wakeword_models=["hey_jarvis"], inference_framework="onnx")
# После детекции → led_state = LISTENING → STT активен
```

### NLU паттерн — LLM-based (TASK-019):
```python
SYSTEM_PROMPT_NLU = """
Ты — NLU компонент робота РОББОКС. Извлеки намерение и сущности из команды.

Доступные намерения:
- navigate_to: перемещение к локации
- stop: остановка
- status_query: вопрос о состоянии
- start_scenario: запуск сценария (patrol/guide/delivery)
- cancel_task: отмена задачи
- chitchat: общий разговор

Доступные локации: {waypoints_list}
Доступные сценарии: patrol, guide, delivery

Ответь ТОЛЬКО JSON без лишнего текста:
{{"intent": "...", "location": "..." или null, "scenario": "..." или null, "confidence": 0.0-1.0}}
"""

async def extract_intent(text: str, waypoints: list[str]) -> dict:
    prompt = SYSTEM_PROMPT_NLU.format(waypoints_list=", ".join(waypoints))
    response = await llm_manager.generate(
        messages=[{"role": "user", "content": text}],
        system_prompt=prompt
    )
    return json.loads(response.text)
```

### Конфигурация (config/voice_assistant/config.yaml):
```yaml
llm:
  qwen:
    api_key: "${QWEN_API_KEY}"
    model: "qwen-max"
    timeout_sec: 10
  deepseek:
    api_key: "${DEEPSEEK_API_KEY}" 
    model: "deepseek-chat"
    timeout_sec: 15
  ollama:
    base_url: "http://localhost:11434"
    model: "llama3.2"
    timeout_sec: 30

wake_word:
  keyword: "роббокс"
  sensitivity: 0.5
  
task_api:
  base_url: "http://10.1.1.10:8080"
  operator_token: "${OPERATOR_JWT_TOKEN}"
```

---

## Правила кода

- `black` (line-length 120), `isort`, `flake8`
- `self.get_logger().info()` — НЕ `print()`
- Type hints для всех public методов
- Асинхронный код: `asyncio` + `httpx.AsyncClient`
- API ключи только через переменные окружения (`.env` файл, НЕ в коде!)

---

## Ревью кода (февраль 2026)

> Состояние на ветке `feature/agent`. Задачи TASK-017–020 — **pending**, частично реализованы.

### ✅ Уже реализовано

| Компонент | Файл | Состояние |
|-----------|------|-----------|
| `ConversationHistory` | `core/conversation_history.py` | Полностью: in-session история, лимит, очистка, tool-messages |
| `DialogueManager` | `core/dialogue_manager.py` | Полностью: state machine IDLE/LISTENING/DIALOGUE/SILENCED, text-based wake word, silence mode, query accumulation |
| `ProviderManager` | `llm/provider_manager.py` | Частично: Qwen↔DeepSeek (двунаправленный), sync OpenAI client, env-key resolution |
| `CommandParser` | `core/command_parser.py` | Частично: regex NLU, интенты NAVIGATE/STOP/FOLLOW/STATUS/MAP/VISION |
| `CommandNode` | `command_node.py` | Частично: Nav2 NavigateToPose напрямую, без task-api |
| `DialogueNode` | `dialogue_node.py` (2423 строки!) | Полностью перегружен — кандидат на рефакторинг |

### ❌ Не реализовано / отклонения от задач

**TASK-017 — LLM fallback:**
- `get_fallback_provider()` возвращает `None` для не-qwen/deepseek провайдеров → **Ollama не подключён**
- Цепочка сейчас двунаправленная (qwen↔deepseek), задача требует **линейную**: qwen → deepseek → ollama
- Смена провайдера **не публикуется** в `/rob_box/voice/status`

**TASK-018 — Wake word:**
- Текущая реализация: Vosk работает **постоянно** → STT-тест `stt_callback` проверяет текст на wake-слова
- Задача требует **акустического** wake-word движка (openWakeWord/Porcupine) — always-on до запуска Vosk
- Нет настройки `sensitivity` в `config.yaml`

**TASK-019 — NLU:**
- `CommandParser` — **regex**, не LLM. Нет `start_scenario`, `cancel_task`, `chitchat`
- Топик сейчас `/voice/command/intent`, задача требует **`/rob_box/voice/intent`**
- Waypoints — hardcoded в паттернах, нужно читать из конфига/параметров

**TASK-020 — Voice→Task API:**
- `CommandNode` отправляет напрямую в Nav2 action client — HTTP к task-api **не реализован**
- Зависит от TASK-001 (task-api) и TASK-012 (ScenarioNode) — оба pending

### ⚠️ Технический долг

- `dialogue_node.py` **2423 строки** — нарушение SRP. Нужно выделить:
  - `LLMStreamingHandler` (логика стриминга, ≈ строки 867–1280)
  - `TaskAPIClient` (HTTP клиент для task-api)
  - `NLUService` (LLM-based intent extraction)
- Провайдеры в `DialogueNode.PROVIDERS` дублируют `llm/provider_manager.py` — нужно переиспользовать `ProviderManager` целиком

---

## EchoVault — память для агентов

> **Репо:** [github.com/mraza007/echovault](https://github.com/mraza007/echovault)

**Что это:** Локальная персистентная память для **AI coding-агентов** (Claude Code, Cursor, Codex). Хранит решения, баги и контекст в Markdown + SQLite (FTS5 + vector search). MCP-сервер. Не требует облака.

**Архитектура:**
```
~/.memory/
├── vault/<project>/YYYY-MM-DD-session.md   # Markdown с YAML frontmatter
├── index.db                                 # SQLite: FTS5 + sqlite-vec
└── config.yaml                              # provider: ollama | openai
```
`MemoryService` → `MemoryDB` (SQLite) + `write_session_memory()` (Markdown) + `EmbeddingProvider` (Ollama/OpenAI)

**MCP инструменты:** `memory_save`, `memory_search`, `memory_context`

### Применимость к РОББОКС

| Сценарий | Применимость |
|----------|-------------|
| Память **этого агента** (coding agent) о решениях в проекте | ✅ Прямое применение — установить EchoVault для сессий разработки |
| In-app память **голосового ассистента** (помнить пользователя, предпочтения) | ❌ Не подходит — EchoVault для dev-сессий, не для runtime ROS 2 |

**Для cross-session памяти голосового ассистента** (если понадобится) — нужна своя реализация:
```python
# Лёгкая альтернатива для rob_box_voice: SQLite с FTS
import sqlite3
from pathlib import Path

class VoiceMemory:
    """Персистентная память ассистента: предпочтения + важные факты."""
    
    def __init__(self, db_path: str = "/data/voice_memory.db"):
        self.conn = sqlite3.connect(db_path)
        self.conn.execute("CREATE VIRTUAL TABLE IF NOT EXISTS memories USING fts5(content, tags)")
    
    def save(self, content: str, tags: str = "") -> None:
        self.conn.execute("INSERT INTO memories VALUES (?, ?)", (content, tags))
        self.conn.commit()
    
    def search(self, query: str, limit: int = 5) -> list[str]:
        cur = self.conn.execute("SELECT content FROM memories WHERE memories MATCH ? LIMIT ?", (query, limit))
        return [row[0] for row in cur.fetchall()]
```

Добавить в `ConversationHistory` метод `save_to_disk()` / `load_from_disk()` при необходимости.

---

## Протокол завершения задачи

1. Выполни все test_steps из tasks.json
2. Обнови `notes` в tasks.json с результатами
3. Запиши в `progress.md`
4. Измени status → `done`
5. `git commit -m "feat(voice): описание"`
