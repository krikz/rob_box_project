# rob_box_telegram

Telegram-интерфейс оператора для Rob Box: управление роботом, просмотр камер, голосовые сообщения и выполнение MCP-команд через Telegram-бот.

## Описание

Пакет реализует Telegram-бота, работающего на Vision Pi, который соединяет Telegram Bot API с ROS 2 топиками. Оператор может отправлять текстовые команды, получать снимки с камер, запрашивать статус системы и управлять роботом движением через чат. Бот поддерживает LLM-диалог через MCP инструменты.

## Ноды

| Нода | Назначение |
|------|-----------|
| `telegram_node` | Главная нода: Telegram Bot API ↔ ROS 2 мост |
| `mcp_bridge` | Вспомогательный компонент: выполнение MCP tool calls с таймаутом |

## Топики

### Подписывается

| Топик | Тип | Описание |
|-------|-----|---------|
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | Фронтальная камера OAK-D |
| `/camera/camera/depth/image_rect_raw/compressedDepth` | `sensor_msgs/CompressedImage` | Карта глубины OAK-D |
| `/ceiling_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Потолочная USB-камера |
| `/rtabmap/grid_prob_map` | `nav_msgs/OccupancyGrid` | 2D SLAM карта |
| `/mcp/result` | `std_msgs/String` | Результаты выполнения MCP инструментов |
| `/mcp/tools` | `std_msgs/String` | Список доступных MCP инструментов |

### Публикует

| Топик | Тип | Описание |
|-------|-----|---------|
| `/voice/tts/request` | `std_msgs/String` | Запросы Text-to-Speech |
| `/cmd_vel_web` | `geometry_msgs/Twist` | Команды движения (приоритет 50 в twist-mux) |
| `/mcp/execute` | `std_msgs/String` | Запросы на выполнение MCP инструментов |

## Параметры

| Параметр | По умолчанию | Описание |
|----------|-------------|---------|
| `TELEGRAM_BOT_TOKEN` | *(env var)* | Токен бота из BotFather |
| `TELEGRAM_ALLOWED_USERS` | *(env var)* | Список разрешённых Telegram user_id через запятую |

## Запуск

Запускается в Docker контейнере `telegram-bot` на Vision Pi:

```bash
# Статус
docker logs telegram-bot -f

# Перезапуск
docker compose -f docker/vision/docker-compose.yaml restart telegram-bot
```

Токен бота задаётся через переменную окружения `TELEGRAM_BOT_TOKEN` в файле `docker/vision/.env`.

## Зависимости

- `python-telegram-bot >= 20.0` (async API)
- `rob_box_mcp_tools` (MCP инструменты для выполнения команд)
- Vision Pi: `zenoh-router-vision` должен быть запущен до старта ноды
