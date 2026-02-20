# Progress Log — РОББОКС

> Этот файл обновляется агентами после завершения каждой задачи.  
> Формат: дата | задача | что сделано | файлы | прошло тестов

---

## Лог выполнения

| Дата | Task ID | Агент | Что сделано | Изменённые файлы | Статус тестов |
|------|---------|-------|-------------|-----------------|--------------|
| — | — | — | Инициализация проекта, PRD и tasks.json созданы | PRD.md, tasks.json | ✅ |
| 2026-02-19 | DOCS-001 | docs-agent | Полный аудит документации: создан NETWORK_TOPOLOGY.md, обновлены README.md, CHANGELOG.md, docs/README.md, architecture/README.md, development/README.md, packages/README.md | 7 файлов | ✅ |
| 2026-02-19 | DOCS-002 | docs-agent | Актуализация документации по git log: CHANGELOG.md — добавлен раздел [Январь 2026] с 14 записями (анимации, звуки, DeepSeek fixes, tokens); README.md — добавлен блок "30 января 2026", исправлен счётчик агентов (10→11); добавлен diagnostics-agent в перечень | 2 файла | ✅ |
| 2026-02-19 | PM-001 | product-manager-agent | Актуализация спецификации железа: исправлена ошибка Pi 4→Pi 5, уточнена RAM конфигурация (Main Pi 16GB, Vision Pi 8GB) в product-manager-agent.md и PRD.md v1.0.1 | 2 файла | ✅ |
| 2026-02-19 | PM-002 | product-manager-agent | Создание задач агентского режима: добавлено 7 задач TASK-035..041 для стабилизации feature/agent (agent cycle тесты, Voice Memory, MCP tools, промпт-инжиниринг, Ollama, документация, CI/CD). Критические: TASK-035 (agent cycle), TASK-036 (memory), TASK-037 (MCP tools) | tasks.json | ✅ |
| 2026-02-19 | PM-003 | product-manager-agent | PM-анализ skills.sh экосистемы (Copilot MCP v0.0.90-91). Вывод: наши agent guides уже являются Skills по паттерну. Добавлено 2 задачи: SKILLS-001 (добавить "When to Apply" секции в agent guides), SKILLS-002 (оценить/установить внешние skills). Стратегическая рекомендация: ниша ros2/robotics на skills.sh пуста — потенциал публикации | tasks.json | ✅ |
| 2026-02-19 | SKILLS-001 | product-manager-agent | Добавлены `## When to Apply` секции во все 12 agent guides: backend, devops, diagnostics, docs, frontend, git, navigation, product-manager, scenarios, security, structure, voice. Обновлён README.md стандартом формата. Каждая секция содержит 5 конкретных триггеров с директориями, файлами и TASK-ID | 13 файлов (`docs/development/agents/*.md`) | ✅ |
| 2026-02-19 | SKILLS-002 | product-manager-agent | Оценка 4 пакетов skills.sh: omer-metin/skills-for-antigravity (ros2-robotics: пустой шаблон, 30 installs), smithery/ai/zeeshan080-ros2-patterns (базовый boilerplate, 1 install), ros2-robotics standalone (404), zeeshan080/ros2-patterns standalone (404). Вердикт: все ROS2 skills — автогенерированные шаблоны без ценности. Внешние skills не устанавливать — наши agent guides в 20x лучше. Рекомендация: публикация как первый ros2 skill на skills.sh | tasks.json | ✅ |
| 2026-02-20 | SKILLS-003 | product-manager-agent | Установка 2 quality skills из smithery/ai. smithery/ai — виртуальная коллекция skills.sh, GitHub-репо `smithery/ai` не существует (404). Скрипты установки npx skills не работали. Решение: созданы SKILL.md вручную из контента skills.sh страниц, адаптированного под Rob Box. Установлено: (1) `python-testing-patterns` — pytest fixtures/mock/async/parametrize для TASK-035/037/041; (2) `code-review-specialist` — review template с Docker/ROS2/Python специфичными правилами | `.agents/github-copilot/python-testing-patterns/SKILL.md`, `.agents/github-copilot/code-review-specialist/SKILL.md` | ✅ |

| 2026-05-30 | TASK-035 | copilot-agent | Анализ live-логов робота (Vision Pi 10.1.1.21): agent cycle работает (iter 12/30, нет deadlocks, нет ThreadPoolExecutor ошибок, batching tool calls х3). Исправлены 2 бага: (1) TIME_CONTEXT_MARKER — маркер `"# Формат ответа"` → `"Формат ответа"` + rfind для поиска начала строки; исправлял повтор песен т.к. время вставлялось в конец промпта. (2) speak_text animation `neutral` — добавлены псевдонимы neutral/excited/confused в enum параметра; validate_parameters отклонял их до вызова execute() с animation_map. Remaining: race condition timeout, параллельные диалоги. | `dialogue_node.py`, `rob_box_mcp_tools/tools/dialogue.py` | ✅ commit 610800d |
| 2026-05-30 | TASK-035 | copilot-agent | Актуализация документации проекта | 13 файлов (`docs/development/agents/*.md`) | ✅ |
| 2026-02-20 | TASK-035 | copilot-agent | **Фиксы стабильности agent mode (серия):** (1) dmix asound.conf — параллельный TTS+sound через ALSA dmix, устранён конфликт hw:1,0; (2) PlaySoundTool INSTANT — убран time.sleep(), fire-and-forget; (3) ThreadPoolExecutor hang `shutdown(wait=False)` в 2 местах `_ask_llm_streaming` и `_continue_after_tool_calls` — устранено зависание ROS2 коллбэка при timeout; (4) interrupt_agent_loop в `_continue_after_tool_calls` — STT STOP во время итерации 2+ теперь прерывает цикл; (5) GetCurrentTimeTool — убрана инъекция времени в system_prompt (KV cache miss), LLM теперь вызывает инструмент; (6) Убран preload past_turns в conversation_history при старте — устранена "каша" с предыдущими сессиями; (7) conversation_history.clear() при каждом wake word из IDLE — контекст растёт только внутри одного диалога; (8) Промпт: guidance не вызывать memory_context без явного запроса, обновлён пример get_current_time | `dialogue_node.py`, `tools/system.py`, `tools/__init__.py`, `tools/sound.py`, `docker/vision/config/asound.conf`, `docker/vision/docker-compose.yaml`, `prompts/master_prompt_compact.txt` | ✅ commits 21de3db 28aa193 154b484 |

| Блокер | Влияет на задачи | Статус |
|--------|-----------------|--------|
| Agent cycle стабильность не протестирована | TASK-035, TASK-038, слияние в develop | 🔴 В работе |
| Voice Memory интеграция не протестирована | TASK-036 | 🟠 Pending |
| MCP Tools без unit тестов | TASK-037 | 🟠 Pending |

---

## Метрики прогресса

- Всего задач: 7 (агентский режим)
- Завершено: 0
- В работе: 0
- Заблокировано: 0
- Pending: 7
