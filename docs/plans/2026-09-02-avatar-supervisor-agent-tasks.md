# Avatar Supervisor Agent — карточки-после (после AV-21)

Это список **follow-up** карточек, на которые распадается развитие
супервизор-агента после того, как AV-21 положит каркас.

| ID | Название | Описание | Зависит от |
|---|---|---|---|
| **AV-22** | Подключение входов | Quest STT (`voice_input_mode=quest_stt`) и Telegram текст — наполняют `/avatar/command`. Делает input-ы «равными» (план §1.1). | AV-21 |
| **AV-23** | `play_music` | Инструмент «включи музыку». TTS-канал через тот же executor-паттерн, что и `say`. | AV-21 |
| **AV-24** | `navigate_to_waypoint` | Инструмент «поехали к точке X». Схема и backend — уже в `rob_box_core.tool_catalog` (там есть `navigate_to_waypoint`; проверить в AV-21 — если нет, добавить в `_tool_catalog_data.py`). | AV-21 |
| **AV-25** | `stop_navigation` | Инструмент «стоп». Экстренный, `destructive=true`. | AV-21 |
| **AV-26** | `play_sound` | Инструмент «проиграть звук X» (эффекты через `sound_node`). | AV-21 |
| **AV-27** | Real backend для `say` | Заменить заглушку executor на публикацию в TTS-канал через `dialogue_node` (с голосом floor). | AV-21 + AV-22 |
| **AV-28** | Real backend для `play_animation` | Заменить заглушку на публикацию в `/avatar/led/animation` или эквивалент. | AV-21 |
| **AV-29** | `agent_enabled` через сервис | Сейчас параметр. Сделать сервис `SetAgentEnabled` для горячего включения из UI/Captain Bridge. | AV-21 |
| **AV-30** | `agent_during_voice_mode` per-client | Сейчас глобальный параметр. Разрешить клиенту задавать свой (Telegram: `"off"`, Quest: `"off"`). | AV-21 |
| **AV-31** | Snapshot/restore через `SessionStore` | Долгая сессия оператора → восстановление после рестарта supervisor. Уже есть harness-snapshot; нужен persistence-адаптер. | AV-21 |
| **AV-32** | e2e-тест на железе | Голос Quest → STT → `/avatar/command` → супервизор-агент → `say` → TTS. Ручной raw-evidence прогон на Vision Pi. | AV-22 + AV-27 |

Все карточки следуют ADR-0013 (incremental — по 1 PR), ADR-0018
(raw-evidence), ADR-0021 (без параллельных реализаций).
