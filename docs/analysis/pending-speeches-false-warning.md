# Анализ: ложный warning «Speech ... не найден в pending_speeches» (issue #776)

**Дата:** 2026-08-12
**Автор:** architect (kanban t_2cd151e0)
**Статус:** принято к реализации
**Связанные issue:** #776 (deployment warning), #980, #992 (batch_complete/prelude)

## 1. Симптом

Deployment-мониторинг (staging) ловит warning в контейнере `perception`:

```
[health_monitor-1]   [WARN] mcp_server (14s ago): [speak_text] ⚠️ Speech 999f09b9... не найден в pending_speeches
```

На живом стенде (10.1.1.11, voice-assistant) паттерн воспроизводится систематически:
22 вхождения за 24 часа, разные speech_id (7c637d10, e8f05d01, 25397a15, 83a494a0, 2e630a66...).

## 2. Где живёт код

- **mcp_server** (SpeakTextTool) физически работает **не в контейнере perception**, а на
  Vision Pi в voice-assistant (`/ws/install/rob_box_mcp_tools/lib/rob_box_mcp_tools/mcp_server`).
  В логи perception он попадает через `/rosout` → `health_monitor` (агрегирует warnings со всей ROS-сети).
- Код: `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/dialogue.py`
  - `SpeakTextTool.execute()` — регистрирует `pending_speeches[speech_id] = {...}`, публикует `/voice/tts/request`.
  - `SpeakTextTool._on_tts_finished()` — подписан на `/voice/tts/finished`, при получении делает
    `pending_speeches.pop(speech_id)`; если `entry is None` — **warning**.
- Генератор finished: `src/rob_box_voice/rob_box_voice/tts_node.py::_publish_tts_finished()`.

## 3. Корневая причина

Топик `/voice/tts/finished` — **общий** для всех отправителей TTS. Его публикует `tts_node`
для **любого** синтезированного speech_id, независимо от того, кто инициировал синтез:

- mcp_server (через `speak_text`) — регистрирует speech_id в `pending_speeches` заранее;
- **dialogue_node** — шлёт системные реплики напрямую (`/voice/tts/request`), **не регистрируя**
  speech_id в `pending_speeches` mcp_server (это отдельный процесс, у него своя логика batch-трекинга).

Когда `tts_node` заканчивает воспроизведение **чужого** для mcp_server speech_id (системная
реплика, триггер `thinking`/`confused`, DJ-прощание и т.п.), он публикует `/voice/tts/finished`
на общий топик. `SpeakTextTool._on_tts_finished()` получает его, пытается `pop` — записи нет,
логирует **warning**.

Подтверждение логами (voice-assistant, 10.1.1.11):

```
[tts_node-5]        📢 Публикую TTS finished event: speech_id=83a494a0..., success=True, duration=2.68s, batch=1/1
[mcp_server-10]     [INFO] [speak_text] 🔔 TTS finished: speech_id=83a494a0..., success=True
[mcp_server-10]     [WARN] [speak_text] ⚠️ Speech 83a494a0... не найден в pending_speeches (возможно уже удалён)
```

Рядом в логах видны `sound_node` триггеры `thinking`/`confused` и `dialogue_node` реплики —
это системные фразы dialogue_node, а НЕ вызовы `speak_text` от LLM.

### Второй источник: дублирующий finished от tts_node (issue #980)

`tts_node._publish_tts_finished()` для последнего чанка батча публикует finished **дважды**:
сначала обычный finished (строка 2818), затем republish с `batch_complete: true` (строка 2855).
Комментарий в tts_node прямо говорит: *«downstream subscribers are designed to be idempotent
on batch_complete»*. mcp_server не идемпотентен: второй finished для того же speech_id даёт
`entry is None` → тот же warning. Это второй сценарий того же симптома.

## 4. Оценка влияния

- **Функционального сбоя нет**: ветка `if entry is None: return` — это корректный no-op.
  mcp_server просто не должен реагировать на чужие finished.
- **Реальное влияние — ложный шум** в мониторинге: health_monitor/deployment-workflow
  помечают прогон как `warning_log`, создаются фейковые issue (#776 — одно из них).
- Вторичный риск: warning маскирует настоящие проблемы (рядом в логах: `MiniMax auth error`,
  `minimax в кэше мёртвых`), т.к. health_monitor показывает «последние warnings» и шум
  вытесняет реальные сигналы.

## 5. Решение

`SpeakTextTool._on_tts_finished()`: для `entry is None` **понизить уровень с warning до debug**
и уточнить сообщение. Топик `/voice/tts/finished` общий, поэтому finished для незарегистрированного
speech_id — **штатная ситуация** (чужой TTS от dialogue_node либо повторный finished от tts_node
с `batch_complete`). mcp_server должен быть идемпотентным потребителем общего топика.

Оставляем warning только для реальной рассинхронизации: `entry` найден, но `batch is None`
(внутреннее рассогласование `pending_speeches` ↔ `pending_batches` — там логика уже помечает
«Balance между двумя dict'ами мог разъехаться только при баге»).

### Альтернативы, которые рассматривались

| Вариант | Плюсы | Минусы | Вердикт |
|---|---|---|---|
| A. Понизить warning→debug в mcp_server | Минимальный diff, правильная семантика (идемпотентность), убирает шум | Не чинит «двойной finished» в tts_node (но он намеренный) | **Выбран** |
| B. Убрать republish finished в tts_node | Устраняет второй источник | Ломает контракт: `batch_complete` маркер нужен потребителям finished (audio_node grace, animation_player); комментарий в tts_node явно требует republish | Отклонён |
| C. Раздельные топики (finished_mcp / finished_dialogue) | Чистое разделение | Ломает существующих подписчиков, большой diff, нужен миграционный план на живых роботах | Отклонён (KISS) |

## 6. Что НЕ делаем

- Не трогаем tts_node: republish finished с `batch_complete` — намеренный контракт issue #980.
- Не трогаем dialogue_node: он не обязан знать про `pending_speeches` mcp_server.
- Не добавляем отдельный «реестр чужих speech_id»: это память и сложность без пользы —
  mcp_server'у достаточно игнорировать неизвестные finished.

## 7. Проверка

1. Юнит-тест: `_on_tts_finished` с чужим speech_id → НЕ warning (нет записи в
   `logger.warning_messages`), batch_complete не публикуется.
2. Юнит-тест: повторный finished для уже завершённого speech_id → НЕ warning.
3. На стенде: после деплоя `docker logs voice-assistant | grep "не найден"` → 0 вхождений,
   при этом нормальные `TTS finished` и `batch_complete` продолжают работать.
