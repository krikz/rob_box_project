# Sound Pi (docker/sound)

Третий узел робота, `10.1.1.12` (eth0) / `10.1.1.22` (wlan0). ADR-0159. Здесь ReSpeaker (микрофон, колонки,
LED-кольцо) и TTL-USB конвертер сенсорного хаба.

| Сервис | Профиль | Откуда |
|---|---|---|
| `zenoh-router` (`zenoh-router-sound`) | — | общий шаблон из `../common`; `connect` на Main и на Vision (вариант Б) |
| `voice-assistant`, `supercollider`, `voice-action-server` | — | перенесены из `docker/vision` |
| `sensor-bridge` | `sensors` | образ `perception`, нода `perception_bridge`, `/dev/sensor_hub` |
| `cadvisor`, `promtail` | `monitoring` | как на Vision, лейбл `host: sound-pi` |

- Адреса узла — `ZENOH_LISTEN_IP`, `ZENOH_CONNECT_ENDPOINTS`, `OLLAMA_HOST_URL` (дефолты в compose, переопределение в `.env`).
- Секреты: `cp .env.secrets.template .env.secrets` (ключи голосового стека; Telegram-токены остаются на Vision).
- Данные состояния (`data/voice`, `cache/tts`) в git не лежат; на робота их переносит cutover (ADR-0159 §3.5).
- Каталоги сборки образов (`voice_base`, `voice_assistant`, `supercollider`) пока в `docker/vision` — переедут с workflow в PR-3.
- Проверка: `cd docker/sound && docker compose --profile sensors --profile monitoring config`.
