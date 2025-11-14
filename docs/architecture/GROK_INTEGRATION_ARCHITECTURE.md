# Сравнение архитектур голосового ассистента

## Текущая архитектура (DeepSeek)

```mermaid
graph LR
    subgraph "Rob Box Voice Assistant - Текущее решение"
        A[ReSpeaker<br/>Mic Array] -->|Audio| B[STT Node<br/>Vosk/Whisper]
        B -->|Text| C[Dialogue Node<br/>DeepSeek LLM]
        C -->|Text| D[TTS Node<br/>Yandex TTS]
        D -->|Audio| E[Sound Node<br/>Playback]
        
        C -->|Commands| F[Command Node<br/>Nav2]
        A -->|VAD/DoA| G[LED Node<br/>Visual]
        
        style A fill:#e8f4f8,stroke:#2c5282
        style B fill:#fef3c7,stroke:#f59e0b
        style C fill:#dbeafe,stroke:#3b82f6
        style D fill:#fef3c7,stroke:#f59e0b
        style E fill:#e8f4f8,stroke:#2c5282
        style F fill:#d1fae5,stroke:#10b981
        style G fill:#fce7f3,stroke:#ec4899
    end
```

**Провайдеры:**
- **STT:** Vosk (offline, primary) / Whisper (offline, fallback) / Yandex (online, complex cases)
- **LLM:** DeepSeek API (online, $0.14 / 1M input tokens)
- **TTS:** Yandex Cloud TTS (online, бесплатно до 1M символов/месяц)

**Стоимость:** ~$5-10/месяц  
**Offline:** ✅ STT + TTS fallback (Silero)

---

## Вариант с Grok (замена только LLM)

```mermaid
graph LR
    subgraph "Rob Box Voice Assistant - С Grok API"
        A[ReSpeaker<br/>Mic Array] -->|Audio| B[STT Node<br/>Vosk/Whisper]
        B -->|Text| C[Dialogue Node<br/>GROK API]
        C -->|Text| D[TTS Node<br/>Yandex TTS]
        D -->|Audio| E[Sound Node<br/>Playback]
        
        C -->|Commands| F[Command Node<br/>Nav2]
        A -->|VAD/DoA| G[LED Node<br/>Visual]
        
        style A fill:#e8f4f8,stroke:#2c5282
        style B fill:#fef3c7,stroke:#f59e0b
        style C fill:#c7d2fe,stroke:#6366f1
        style D fill:#fef3c7,stroke:#f59e0b
        style E fill:#e8f4f8,stroke:#2c5282
        style F fill:#d1fae5,stroke:#10b981
        style G fill:#fce7f3,stroke:#ec4899
    end
```

**Изменения:**
- **STT:** Без изменений (Vosk/Whisper)
- **LLM:** ✅ **Grok API** вместо DeepSeek (1M токенов контекст, real-time search)
- **TTS:** Без изменений (Yandex TTS)

**Стоимость:** ~$???/месяц (требуется тестирование!)  
**Offline:** ⚠️ Только STT + TTS (LLM требует интернет)

---

## Рекомендуемая архитектура (гибрид)

```mermaid
graph LR
    subgraph "Rob Box Voice Assistant - Гибридный подход"
        A[ReSpeaker<br/>Mic Array] -->|Audio| B[STT Node<br/>Vosk/Whisper]
        B -->|Text| C{Dialogue Node<br/>Auto-select}
        
        C -->|Simple queries| D1[DeepSeek<br/>Primary]
        C -->|Complex queries| D2[Grok<br/>Fallback]
        
        D1 -->|Text| E[TTS Node<br/>Yandex TTS]
        D2 -->|Text| E
        
        E -->|Audio| F[Sound Node<br/>Playback]
        
        C -->|Commands| G[Command Node<br/>Nav2]
        A -->|VAD/DoA| H[LED Node<br/>Visual]
        
        style A fill:#e8f4f8,stroke:#2c5282
        style B fill:#fef3c7,stroke:#f59e0b
        style C fill:#f3e8ff,stroke:#a855f7
        style D1 fill:#dbeafe,stroke:#3b82f6
        style D2 fill:#c7d2fe,stroke:#6366f1
        style E fill:#fef3c7,stroke:#f59e0b
        style F fill:#e8f4f8,stroke:#2c5282
        style G fill:#d1fae5,stroke:#10b981
        style H fill:#fce7f3,stroke:#ec4899
    end
```

**Логика выбора:**

```yaml
dialogue_node:
  llm_provider: "auto"
  
  # Использовать DeepSeek для:
  use_deepseek:
    - simple_questions: true         # "Привет", "Как дела?"
    - navigation_commands: true      # "Поезжай вперёд"
    - context_length: "< 4000"       # Короткие диалоги
  
  # Использовать Grok для:
  use_grok:
    - complex_reasoning: true        # Сложные вопросы
    - long_context: "> 4000"         # Длинные диалоги
    - real_time_search: true         # Актуальная информация
    - advanced_function_calling: true # Сложные команды
```

**Преимущества:**
- 💰 Оптимальная стоимость (DeepSeek для 70-80% запросов)
- 🎯 Высокое качество (Grok для сложных задач)
- 🔄 Гибкость (ручное переключение через конфиг)
- 📊 A/B тестирование

**Стоимость:** ~$15-30/месяц (оценка)  
**Offline:** ⚠️ Только STT + TTS

---

## Сравнительная таблица

| Параметр | Текущее (DeepSeek) | Grok замена | Гибрид (рекомендуется) |
|----------|-------------------|-------------|------------------------|
| **STT** | Vosk/Whisper | Vosk/Whisper | Vosk/Whisper |
| **LLM** | DeepSeek | Grok | DeepSeek + Grok |
| **TTS** | Yandex | Yandex | Yandex |
| **Качество** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Стоимость** | $5-10/мес | $???/мес | $15-30/мес |
| **Offline** | ✅ STT+TTS | ⚠️ STT+TTS | ⚠️ STT+TTS |
| **Контекст** | 8K-32K токенов | 1M токенов | 1M токенов |
| **Real-time search** | ❌ | ✅ | ✅ |
| **Function calling** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Сложность** | Текущий | Простая (~4ч) | Средняя (~8ч) |

---

## Что НЕ меняется

Независимо от выбранной архитектуры, эти компоненты остаются без изменений:

```mermaid
graph TB
    subgraph "Компоненты без изменений"
        A[ReSpeaker Mic Array<br/>Захват аудио, VAD, DoA]
        B[STT Node<br/>Vosk offline STT<br/>Whisper fallback]
        C[TTS Node<br/>Yandex Cloud TTS<br/>Голос Anton]
        D[Sound Node<br/>Playback аудио]
        E[LED Node<br/>12x RGB LED индикация]
        F[Command Node<br/>Выполнение Nav2 команд]
        
        style A fill:#e0e7ff,stroke:#6366f1
        style B fill:#e0e7ff,stroke:#6366f1
        style C fill:#e0e7ff,stroke:#6366f1
        style D fill:#e0e7ff,stroke:#6366f1
        style E fill:#e0e7ff,stroke:#6366f1
        style F fill:#e0e7ff,stroke:#6366f1
    end
```

**Изменяется ТОЛЬКО:** `dialogue_node.py` — выбор LLM провайдера (DeepSeek / Grok / Гибрид)

---

## Вывод

**Grok API — это не замена voice assistant, а улучшение LLM компонента.**

### ✅ Рекомендация: Гибридный подход

1. Оставить текущую архитектуру (STT/TTS)
2. Добавить Grok как опциональный LLM провайдер
3. Использовать автоматический выбор на основе сложности запроса
4. Собрать метрики и оптимизировать баланс качество/стоимость

**Следующий шаг:** Протестировать Grok API — [GROK_API_QUICKSTART.md](guides/GROK_API_QUICKSTART.md)

---

**См. также:**
- [Полный анализ](reports/GROK_VOICE_MODE_ANALYSIS.md)
- [Рекомендации](../GROK_API_RECOMMENDATIONS.md)
- [Пример кода](examples/grok_api_integration_example.py)
