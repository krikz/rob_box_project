# DeepSeek V3: Multi-Turn Function Calling Pattern-Completion Bug

**Дата:** 5 марта 2026  
**Компонент:** `dialogue_node.py`, `NavigationSkill`  
**Severity:** HIGH — повторные вызовы навигационных инструментов полностью пропускались

---

## 🐛 Проблема

### Симптомы

Пользователь просит сохранить несколько точек подряд:
1. "Сохрани точку база" → Робот сохраняет точку, отвечает "Точка база сохранена!" ✅
2. "Сохрани точку А" → Робот отвечает "Точка А сохранена!", но **точка НЕ сохранена** ❌

Аналогично с любыми повторяющимися навигационными командами в рамках одной сессии.

### Лог проблемы (до фикса)

**Тёрн 1** — `save_waypoint` реально вызван (~13 секунд):
```
LLM INPUT (1 messages): "сохрани точку база"
→ save_waypoint(name='база')   ← mcp_server вызван
→ list_waypoints()
→ speak_text('Точка "база" сохранена!')
LLM OUTPUT: "[выполнено через: handle_navigation, speak_text] Точка "база" сохранена!"
```

**Тёрн 2** — `save_waypoint` НЕ вызван (~2 секунды):
```
LLM INPUT (3 messages):
  user: "сохрани точку база"
  assistant: "[выполнено через: handle_navigation, speak_text] Точка "база" сохранена!"
  user: "сохрани точку а"

⚠️ Auto-speak fallback (LLM skipped speak_text): Точка "а" сохранена!
LLM OUTPUT: "[выполнено через: handle_navigation, speak_text] Точка "а" сохранена!"
```

Ключевые признаки бага:
- Время тёрна: ~2 сек вместо ~13 сек (нет реальных tool calls)
- `⚠️ Auto-speak fallback (LLM skipped speak_text)` — LLM вернул голый текст
- Ни одного вызова `mcp_server` между LLM INPUT и LLM OUTPUT

---

## 🔍 Root Cause

### Авторегрессивный pattern-completion

DeepSeek V3 — авторегрессивная модель. При генерации она предсказывает следующий токен на основе всего контекста. Когда в истории диалога присутствует явный паттерн:

```
user: "сохрани точку X"
assistant: "[выполнено через: handle_navigation, speak_text] Точка X сохранена!"
```

Модель видит следующий запрос `"сохрани точку А"` и начинает завершать **уже известный паттерн** — генерирует `"[выполнено через: ...] Точка А сохранена!"` **до того как добирается до инструкций** в system prompt.

### Почему system prompt не помогает

Добавление в промпт `"ВСЕГДА вызывай инструменты, никогда не копируй паттерн из истории"` **не работает**. System prompt обрабатывается моделью как часть контекста, но авторегрессивная генерация начинается с самого вероятного продолжения — а оно определяется силой паттерна в conversation history, которая оказывается сильнее инструкций.

Это известный баг DeepSeek V3: [deepseek-ai/DeepSeek-V3#826](https://github.com/deepseek-ai/DeepSeek-V3/issues/826) — multi-turn function calling ненадёжен начиная со второго тёрна при похожих запросах.

### Почему именно навигация уязвима

`handle_navigation` — скилл-агент, его тёрны записывались в историю в характерном формате:
```
"[выполнено через: handle_navigation, speak_text] <текст>"
```
Этот формат создавал сильный, легко распознаваемый паттерн для следующей генерации.

---

## ✅ Решение

### Механизм: исключение тёрнов из истории по списку инструментов

Вместо того чтобы бороться с паттерн-completion через промпты — убираем причину: тёрны с вызовом указанных инструментов просто не добавляются в `_conversation`.

**`dialogue_node.py`** — история сохраняется только если ни один из исключённых инструментов не был вызван:
```python
excluded = tool_names_used & self._history_excluded_tools
if excluded:
    self.get_logger().info(f"🚫 Turn excluded from history (tools: {excluded})")
else:
    self._conversation = self._trim_history(
        list(self._conversation)
        + [{"role": "user", "content": user_input},
           {"role": "assistant", "content": history_entry}]
    )
```

**`voice_assistant.yaml`** — конфигурация управляет списком без правки кода:
```yaml
dialogue_node:
  # Tools whose turns are excluded from conversation history.
  # Prevents DeepSeek V3 multi-turn FC pattern-completion bug.
  # Add "handle_music" here to exclude music turns too.
  history_excluded_tools:
    - "handle_navigation"
```

**ROS2 параметр** с дефолтом:
```python
self.declare_parameter("history_excluded_tools", ["handle_navigation"])
self._history_excluded_tools: set = set(
    self.get_parameter("history_excluded_tools").value
)
```

### Почему это работает

Каждый навигационный запрос теперь получает `LLM INPUT (1 messages)` — только текущий запрос пользователя, без истории предыдущих навигационных тёрнов. У модели нет паттерна для completion — она вынуждена реально вызывать инструменты.

---

## 📊 Верификация

**Лог после фикса** (сложная миссия "едь на точку Б, скажи фигаро тут, потом точку А, скажи фигаро там"):

```
📥 LLM INPUT (1 messages): "едет на точку б и скажи там фигаро тут потом едь на точку а и скажи фигаро там"

→ list_waypoints()                         ✅ реальный вызов
→ get_current_pose() → (0.36, 0.17, -0.51) ✅ запомнил позицию
→ navigate_to_waypoint('б')                ✅ 24 сек, доехал
→ speak_text('фигаро тут')                 ✅
→ navigate_to_waypoint('а')                ✅ доехал
→ speak_text('фигаро там')                 ✅
→ speak_text('Миссия выполнена!')          ✅

🚫 Turn excluded from history (tools: {'handle_navigation'})
```

---

## 📁 Изменённые файлы

| Файл | Изменение |
|------|-----------|
| `src/rob_box_voice/rob_box_voice/dialogue_node.py` | `declare_parameter("history_excluded_tools")`, логика исключения из истории, лог в `_log_config()` |
| `docker/vision/config/voice/voice_assistant.yaml` | Новый параметр `history_excluded_tools: ["handle_navigation"]` |

**Коммиты:**
- `f9110ce` — feat(voice): generic history exclusion per skill-tool via config
- `5b96771` — feat(voice): log history_excluded_tools at startup

---

## ⚙️ Конфигурация

Для добавления новых исключений — только правка `voice_assistant.yaml`:

```yaml
history_excluded_tools:
  - "handle_navigation"   # навигация — исключена
  - "handle_music"        # раскомментировать если музыкальные тёрны тоже создают паттерны
```

---

## 🔄 Альтернативы (отклонены)

| Вариант | Почему отклонён |
|---------|-----------------|
| Промпт-инструкции ("всегда вызывай инструменты") | Авторегрессия сильнее инструкций — не работает, проверено |
| Детерминированный pre-dispatch по ключевым словам | Хрупко, нужно поддерживать список фраз, не generic |
| Смена модели на Qwen-Max (96.5% vs 81.5% multi-turn FC) | Нет доступа к Qwen биллингу на текущий момент |
| `tool_choice="required"` + `max_tokens=1000` | Частичная помощь, не устраняет root cause |
