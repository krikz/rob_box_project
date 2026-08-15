# Offline LLM (Ollama local) — план для M3

> **Статус:** Plan (draft) — реализация отложена до Milestone 3 «Refactoring & Voice»
> **Источник:** TECH_DEBT.md SL-3 → GitHub issue #836
> **Роль:** devops (инфраструктура)
> **Дата:** 2026-08-15
> **Ветка:** `z-{agent}/836-sl-3-llm-providers-offline-fallback`

---

## 1. Проблема (verbatim)

> LLM providers are cloud-only, offline fallback (Ollama local) is disabled.
> Plan for offline LLM in M3.

Исходная запись в реестре техдолга:

| ID | Проблема | Файл | Severity | Disposition | Обоснование |
|----|----------|------|----------|-------------|-------------|
| SL-3 | LLM providers — только облако, offline fallback отключён | `rob_box_voice/dialogue_node.py` | medium | defer:M3 | Offline LLM (Ollama local) в планах для M3 |

---

## 2. Текущее состояние (проверено 2026-08-15)

### 2.1 LLM-цепочка — только облако

- `docker/vision/config/voice_assistant/voice_assistant.yaml`:
  `llm_providers: "minimax,deepseek"` — primary MiniMax, fallback DeepSeek (оба — облачные API).
- `src/rob_box_voice/config/dialogue_node.yaml`:
  `llm_providers: "minimax,deepseek"` — то же.
- `src/rob_box_voice/rob_box_voice/dialogue_node.py` `_LLM_PROVIDER_REGISTRY`:
  зарегистрированы `minimax`, `deepseek`, `mimo`, `qwen` — **ollama отсутствует**.
  Неизвестный провайдер в `_build_single_provider()` логирует warning и пропускается.
- `src/rob_box_voice/rob_box_voice/llm/provider_manager.py`:
  `get_fallback_provider()` — только qwen↔deepseek, для остальных `None`.

### 2.2 Ollama в системе уже есть, но НЕ в LLM-цепочке

| Место | Роль | Статус |
|-------|------|--------|
| `docker/vision/docker-compose.yaml` (`ollama`, profile `ai`) | Local LLM inference server | Выключен по умолчанию (profile `ai`), используется только для **эмбеддингов** |
| `src/rob_box_voice/rob_box_voice/core/voice_memory.py` (`OllamaEmbedder`) | Эмбеддинги для семантического поиска (nomic-embed-text) | Lazy, gracefully degrades |
| `src/rob_box_voice/rob_box_voice/core/faq_store.py` | Эмбеддинги FAQ | Опционально |
| `docker/vision/test/` (docker-compose.test.yml + qwen2.5:0.5b) | Offline-тесты dialogue_node | Уже работает (порт 11435) |

Вывод: **инфраструктура для локального LLM уже частично развёрнута** (сервис, volume, healthcheck, тестовый стек), но в боевую LLM-цепочку dialogue_node/voice-assistant она не включена.

### 2.3 Почему «только облако» — осознанное решение

- Облачные провайдеры (MiniMax-M3 primary, DeepSeek fallback) дают качество, скорость и tool-calling, недостижимые на CPU Pi 5.
- Локальный LLM на CPU Pi 5: ~5–8 tok/s (llama3.2:1b, см. `docs/reports/AI_HAT_UPGRADE_ANALYSIS.md` §варианты) — приемлемо как **последний** fallback, но не как primary.
- Milestone 3 = «Refactoring & Voice»: dialogue_node рефакторинг, когда LLM-порты переезжают в harness (`LLMProvider`), добавление Ollama-адаптера станет чистым расширением реестра.

---

## 3. Целевое состояние (M3)

LLM-цепочка: `minimax → deepseek → ollama (local, last-resort)`.

При недоступности обоих облачных провайдеров робот продолжает диалог через локальный
Ollama (qwen2.5:0.5b / llama3.2), а не молчит. Офлайн-режим — явный last-resort fallback,
никогда не primary.

### 3.1 Критерии приёмки (acceptance)

1. `ollama` добавлен в `_LLM_PROVIDER_REGISTRY` и собирается `_build_single_provider()`.
2. `docker/vision/config/voice_assistant/voice_assistant.yaml`:
   `llm_providers: "minimax,deepseek,ollama"`.
3. Секция `ollama:` в YAML: `base_url: http://localhost:11434/v1`, `model: qwen2.5:0.5b` (или `llama3.2`), без api_key.
4. Сервис `ollama` в `docker/vision/docker-compose.yaml` переведён из profile `ai` в default (или поднят вместе с voice-assistant).
5. Юнит-тест: `_resolve_provider_chain()` и `_build_single_provider("ollama")` без сети.
6. Интеграционный тест: dialogue_node с `llm_providers="ollama"` отвечает через локальный Ollama (повторно использовать существующий стек `docker/vision/test/`).
7. При живых облачных провайдерах поведение не меняется (ollama — последний в цепочке).

---

## 4. План работ (задачи для M3)

### Задача 1: Регистрация провайдера ollama в dialogue_node

**Файлы:**
- `src/rob_box_voice/rob_box_voice/dialogue_node.py` — `_LLM_PROVIDER_REGISTRY`

**Шаги:**
1. Добавить запись:
   ```python
   "ollama": {
       "display_name": "Ollama (local)",
       "has_balance_api": False,
       "default_base_url": "http://localhost:11434/v1",
       "default_model": "qwen2.5:0.5b",
       "env_key_var": "",  # без API-ключа
   },
   ```
2. В `_build_single_provider()` ветка `name == "ollama"` → OpenAI-совместимый
   `build_deepseek_provider(api_key="ollama", base_url=..., model=...)` (как в тестовом конфиге).
3. Юнит-тесты: registry содержит ollama; сборка провайдера без api_key не падает.

### Задача 2: Конфигурация (YAML)

**Файлы:**
- `docker/vision/config/voice_assistant/voice_assistant.yaml`
- `src/rob_box_voice/config/dialogue_node.yaml`

**Шаги:**
1. `llm_providers: "minimax,deepseek,ollama"`.
2. Секция `ollama:` с `base_url`, `model`, `timeout_s` (без `api_key` — пустая строка/отсутствует).
3. Комментарий: «последний fallback, офлайн-режим, low quality — только когда облако недоступно».

### Задача 3: Docker-сервис ollama в боевом compose

**Файлы:**
- `docker/vision/docker-compose.yaml`

**Шаги:**
1. Убрать `profiles: ["ai"]` (или добавить второй сервис `ollama-lite` в default profile).
2. Закрепить тег образа вместо `latest` (заодно закрывает SEC-6: «Unpinned ollama/ollama:latest»).
3. Валидация: `docker compose config` — сервис поднимается без флага `--profile`.

### Задача 4: Тесты

**Файлы:**
- `src/rob_box_voice/test/unit/llm/test_provider_manager.py` (расширить)
- `docker/vision/test/` (переиспользовать существующий стек)

**Шаги:**
1. Юнит: цепочка `minimax,deepseek,ollama` разбирается, ollama строится без сети.
2. Интеграция: прогон `docker compose --profile ollama up` из `docker/vision/test/`
   с `llm_providers="ollama"` — робот отвечает на «привет» локальной моделью.

### Задача 5: Документация

**Файлы:**
- `docs/architecture/SYSTEM_OVERVIEW.md` — таблица сервисов Vision Pi (ollama: Local LLM inference → отметить как LLM fallback, не только эмбеддинги)
- `docs/development/agents/voice-agent.md` — снять «Ollama не подключён» в TASK-017 после реализации

---

## 5. Риски и ограничения

| Риск | Влияние | Митигация |
|------|---------|-----------|
| CPU-инференс медленный (5–8 tok/s на Pi 5) | UX деградирует в офлайне | ollama только last-resort; короткий `timeout_s` (60–90с) |
| RAM Pi 5 (модель + voice-assistant + STT/TTS) | OOM при одновременной работе | Модель ≤1–2 GB (qwen2.5:0.5b ~350 MB, llama3.2:1b ~1.3 GB) |
| `ollama/ollama:latest` непропинан | Дрейф версий | Закрепить digest/версию (закрывает SEC-6) |
| Офлайн-эмбеддинги (voice_memory) уже используют Ollama | Конфликт портов/моделей | Один сервис на 11434, модель для чата и эмбеддингов разные — не пересекаются |
| Ошибочный выбор ollama как primary | Качество диалога падает | Порядок цепочки жёстко `minimax,deepseek,ollama`; health-aware fallback (rob_box_harness) не затронут |

---

## 6. Решения, принятые в этом плане (для исполнителей)

- **Порядок цепочки:** `minimax → deepseek → ollama`. Менять только через YAML `llm_providers`.
- **Модель по умолчанию:** `qwen2.5:0.5b` (уже проверена в `docker/vision/test/`, ~350 MB, русский).
- **API-ключ:** отсутствует. OpenAI-совместимый клиент с заглушкой `api_key="ollama"` (как в тестовом конфиге).
- **Порт:** `localhost:11434` (боевой) / `11435` (тестовый стек). Не менять без необходимости.
- **Профиль compose:** сервис переводится в default profile, чтобы fallback работал без ручных флагов.
- **НЕ делать:** не выносить ollama в primary; не добавлять в harness `rob_box_harness` раньше M3-рефакторинга dialogue_node (там отдельный ADR-0001 порт LLMProvider — при переезде адаптер добавляется в реестр портов).

---

## 7. Вне скоупа (явно)

- Переезд LLM-портов в harness (`rob_box_harness`) — это отдельный трек M3 (SPEC_CURRENT этап B, ADR-0001 §2.7).
- VLM/мультимодальность на локальном LLM — не в этой задаче.
- Эмбеддинги voice_memory/FAQ (PF-3 async embedding) — отдельный пункт техдолга.
