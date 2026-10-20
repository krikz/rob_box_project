# Sound Pi (docker/sound)

Третий узел робота, `10.1.1.12` (eth0) / `10.1.1.22` (wlan0). ADR-0159. Здесь ReSpeaker (микрофон, колонки,
LED-кольцо) и TTL-USB конвертер сенсорного хаба.

| Сервис | Профиль | Откуда |
|---|---|---|
| `zenoh-router` (`zenoh-router-sound`) | — | общий шаблон из `../common`; `connect` на Main и на Vision (вариант Б) |
| `voice-assistant`, `supercollider`, `voice-action-server` | — | перенесены из `docker/vision` |
| `supervisor` (`avatar-supervisor`, ТАРС) | — | перенесён из `docker/vision` (ADR-0159 В8): пишет в тот же `data/voice/harness_voice.db`, что и `voice-assistant` |
| `avatar-arbiter` | — | волна 2 (PR-7, В9, решение Шифу 20.10): образ `supervisor`, скрипт `scripts/supervisor/start_arbiter.sh` рядом со `start_supervisor.sh` |
| `quest` (`rob-box-quest`) | — | волна 2: WebXR-консоль Meta Quest, Caddy на `:443`/`:8443`, WS `:8766`; именованный том `quest-certs` (на Sound — новый, пустой) |
| `telegram-bot` | — | волна 2: операторский Telegram-интерфейс, метрики `:9101`; галерея лиц `data/faces` — копия-снимок (синхронизация — #3655) |
| `sensor-bridge` | `sensors` | образ `perception`, нода `perception_bridge`, `/dev/sensor_hub` |
| `cadvisor`, `promtail` | `monitoring` | как на Vision, лейбл `host: sound-pi` |

- Адреса узла — `ZENOH_LISTEN_IP`, `ZENOH_CONNECT_ENDPOINTS`, `OLLAMA_HOST_URL` (дефолты в compose, переопределение в `.env`).
- Секреты: `cp .env.secrets.template .env.secrets` (ключи голосового стека и Telegram-токены: `telegram-bot` живёт здесь с PR-7).
- Данные состояния (`data/voice`, `cache/tts`) в git не лежат; на робота их переносит cutover (ADR-0159 §3.5).
- Каталоги сборки образов (`voice_base`, `voice_assistant`, `supercollider`, `supervisor`, `quest`, `telegram_bot`) лежат здесь.
- Quest: адрес для шлема — `10.1.1.12` (DNS `quest.rob_box.local` на роутере должен указывать на него — переключает Шифу); при пустом `quest-certs` сертификат создаётся заново.
- Проверка: `cd docker/sound && docker compose --profile sensors --profile monitoring config`.
