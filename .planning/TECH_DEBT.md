# Tech Debt Register — Rob Box

**Дата аудита:** 2026-05-15
**Источник:** .planning/codebase/CONCERNS.md
**Всего пунктов:** 30
**Scope:** Milestone 1 — аудит и документирование, не исправление

---

## Шкала severity

| Уровень | Критерий |
|---------|----------|
| critical | Блокирует production; данные теряются или система неработоспособна |
| high | Существенно ухудшает UX или безопасность; требует решения в текущем milestone |
| medium | Заметная деградация; должно быть решено до Milestone 3 |
| low | Косметика или незначительное неудобство; fix или defer произвольно |

## Disposition

| Статус | Значение |
|--------|----------|
| fix | Исправить в Milestone 1 |
| defer:M2 | Отложить до Milestone 2 (навигация) |
| defer:M3 | Отложить до Milestone 3 |
| accept | Принять как есть (обоснование указано) |

---

## Tech Debt (TD)

| ID | Проблема | Файл | Severity | Disposition | Обоснование |
|----|----------|------|----------|-------------|-------------|
| TD-1 | dialogue_node.py монолит 2040 строк, 74 метода | rob_box_voice/dialogue_node.py | high | defer:M3 | Задокументирована стратегия декомпозиции в DIALOGUE_NODE_REFACTORING.md; не блокирует навигацию |
| TD-2 | get_robot_status возвращает hardcoded stub (position 0,0; battery 85%) | rob_box_mcp_tools/tools/system.py:440 | high | defer:M2 | LLM получает ложные данные о положении; реальные данные появятся после Nav2 интеграции |
| TD-3 | reflection_node.py silent fallback без предупреждения при старте | rob_box_perception/reflection_node.py:668,702,747 | medium | defer:M3 | Добавить WARN log; не блокирует функциональность |
| TD-4 | command_node.py навигация/vision stubs с `pass` | rob_box_voice/command_node.py:343,359,370,394 | medium | defer:M2 | Голосовые команды навигации зависят от Nav2; помечены # STUB: |
| TD-5 | async_executor.py busy-wait polling 50ms | rob_box_mcp_tools/async_executor.py:348-357 | low | defer:M3 | CPU overhead незначителен на Raspberry Pi 5; performance sprint позже |
| TD-6 | setup.py placeholder metadata в rob_box_perception | rob_box_perception/setup.py:25-26 | low | accept | Исправлено в Phase 2; no impact runtime |
| TD-7 | LED compositor: 2 источника конфигурации (hardcoded + YAML) | led_matrix_driver/launch/ | low | defer:M3 | Запутывает конфигурацию; не блокирует текущую функциональность |

---

## Known Bugs (BG)

| ID | Bug ID | Описание | Severity | В tasks.json | Disposition |
|----|--------|----------|----------|-------------|-------------|
| BG-1 | TASK-043 | Unbounded local messages list в agent run → timeouts | high | ✅ TASK-043 | defer:M3 (исправление требует рефакторинга agent loop) |
| BG-2 | TASK-044 | Tool results накапливаются в conversation history | medium | ✅ TASK-044 | defer:M3 |
| BG-3 | TASK-046 | LLM забывает system prompt после 15+ iterations | low | ✅ TASK-046 | defer:M3 |
| BG-4 | TASK-047 | Barge-in регрессия (commit 37527df) | high | ✅ TASK-047 | defer:M3 |
| BG-5 | TASK-049 | BLE joystick blocked by kernel 6.14.0-raspi regression | critical | ✅ TASK-049 | defer:M2 (kernel patch или downgrade) |
| BG-6 | TASK-050 | VESC wheel jitter при старте/остановке (частично исправлен) | medium | ✅ TASK-050 | defer:M2 (VESC PID tuning) |

---

## Security Issues (SEC)

| ID | Проблема | Файл | Severity | Disposition | Обоснование |
|----|----------|------|----------|-------------|-------------|
| SEC-1 | Hardcoded SSH password 'open' в CI/CD workflow | .github/workflows/L-Deploy and Verify.yml | critical | fix | Заменить на GitHub Actions secret; аудит Phase 3, fix — отдельный PR |
| SEC-2 | Пароль 'open' в документации (docs/fixes/) | docs/fixes/ | high | fix | Заменить на `<ROBOT_PASSWORD>` placeholder; фикс возможен в Phase 3 |
| SEC-3 | Zenoh без аутентификации | zenoh config | medium | accept | Private network, lab use; приемлемо для текущей стадии |
| SEC-4 | privileged: true для 7 контейнеров | docker/main/, docker/vision/ | medium | accept | Hardware access требует privileged; нет альтернативы без device mapping |
| SEC-5 | network_mode: host для всех контейнеров | docker-compose файлы | low | accept | DDS/Zenoh requirement; задокументировано в архитектуре |
| SEC-6 | Unpinned ollama/ollama:latest | docker/vision/docker-compose.yml | low | defer:M3 | Pin в следующем спринте вместе с остальными образами |

---

## Performance Bottlenecks (PF)

| ID | Проблема | Файл | Severity | Disposition | Обоснование |
|----|----------|------|----------|-------------|-------------|
| PF-1 | async_executor busy-wait 50ms polling | rob_box_mcp_tools/async_executor.py:348-357 | low | defer:M3 | Overhead незначителен; performance sprint позже |
| PF-2 | dialogue_node.py context grows за один agent run | rob_box_voice/dialogue_node.py | high | defer:M3 | Связан с BUG-12 (TASK-043); решается декомпозицией M3 |
| PF-3 | voice_memory.py synchronous Ollama embedding блокирует ROS event loop | rob_box_voice/voice_memory.py | medium | defer:M3 | Async embedding требует рефакторинга voice pipeline |

---

## Fragile Areas (FA)

| ID | Проблема | Файл | Severity | Disposition | Обоснование |
|----|----------|------|----------|-------------|-------------|
| FA-1 | audio_node.py bare except: pass в shutdown | rob_box_voice/audio_node.py:325,331,337 | medium | defer:M3 | Низкий runtime риск; исправить при рефакторинге voice pipeline |
| FA-2 | calibrate_max_rpm.py/linearity_test.py глотают все исключения | local_test/ scripts | high | defer:M2 | Инструменты диагностики; исправить до моторного тестирования M2 |
| FA-3 | VESC hardware interface: нет NaN/infinity guard | vesc_nexus (submodule) | high | defer:M2 | Нет guard = потенциальный crash при битых данных VESC |
| FA-4 | Zenoh IPs hardcoded в router configs | docker/main/config/, docker/vision/config/ | medium | defer:M2 | Затрудняет переезд на другой стенд; параметризировать при M2 настройке |
| FA-5 | test_dialogue_node.py — 13+ пустых тест-методов (только pass) | rob_box_voice/test/ | high | defer:M3 | Создаёт ложное ощущение покрытия; документировано в COVERAGE_REPORT.md |
| FA-6 | led_matrix_driver.py exception в clear_matrix → LEDs остаются включёнными | rob_box_animations/ | low | defer:M3 | Визуальный артефакт без safety implications |

---

## Scaling Limits (SL)

| ID | Проблема | Файл | Severity | Disposition | Обоснование |
|----|----------|------|----------|-------------|-------------|
| SL-1 | docker/build/ — 8 полных копий репозитория для раннеров | docker/build/ | low | accept | CI/CD overhead; accept как компромисс простоты vs скорость |
| SL-2 | Один Zenoh router = SPOF | docker/main/zenoh-router/ | medium | defer:M2 | Redundancy возможна; low priority для lab env |
| SL-3 | LLM providers — только облако, offline fallback отключён | rob_box_voice/dialogue_node.py | medium | defer:M3 | Offline LLM (Ollama local) в планах для M3; план: `docs/plans/2026-08-15-offline-llm-m3.md` (issue #836) |

---

## Итоговая статистика

| Severity | Кол-во |
|----------|--------|
| critical | 2 (BG-5, SEC-1) |
| high | 11 |
| medium | 11 |
| low | 6 |

| Disposition | Кол-во |
|------------|--------|
| fix | 2 (SEC-1, SEC-2) |
| defer:M2 | 8 |
| defer:M3 | 14 |
| accept | 6 |

---
*Аудит выполнен: Phase 3 Milestone 1, 2026-05-15*
*Следующий аудит: начало Milestone 2*
