# Игорёк на колонке: установка, обновление, диагностика (runbook)

Дата: 2026-11-08. Собрано на двух колонках: `10.1.1.13` (первая, Amlogic) и `10.1.1.141` (вторая, Unisoc).
Рут колонки — отдельный runbook: [yandex-station-root.md](yandex-station-root.md).
Репозиторий хоста — `krikz/rob_box_station` (границы — ADR-0167).

## Что где лежит на колонке

```
/data/opt/python/                 Питон 3.12 armv7 (python-build-standalone, обрезан tools/prune_python.py)
/data/opt/rbx/site/               колёса armv7: openai, httpx, pydantic, anyio, dbus-next, PyYAML
/data/opt/rbx/core/               ядро диалога (файлы из rob_box_project) + scripts/dialogue/.env
/data/opt/rbx/station_host.py     хост: микрофон → VAD → STT → ход ядра → TTS → динамик
/data/opt/rbx/station_setup.py    страница :8080 и Improv по Bluetooth
/data/opt/rbx/settings.json       настройки со страницы: RBX_WAKE, RBX_VOLUME, RBX_LED, RBX_VOICE
/data/opt/rbx/VERSION             тег, с которого залит код
/data/opt/rbx_host                маркер автозапуска (нет маркера — хост не поднимется сам)
/data/opt/yandex_off              маркер глушения Алисы
/tmp/station.log                  лог хоста (при рестарте сдвигается в station.log.1)
```

## Установка одной строкой

С компьютера, где уже есть SSH к колонке (рут по runbook):

```bash
curl -sL https://raw.githubusercontent.com/krikz/rob_box_station/main/install.sh | sh -s -- <IP колонки>
```

Переменные: `TAG=vX.Y.Z` — конкретный тег, `RUNTIME_URL=https://…` — свой адрес рунтайма,
`NEED_MB=NN` — порог свободного места на `/data` (по умолчанию 55 МБ), `DRY_RUN=1` — показать шаги,
`FORCE_RUNTIME=1` — переставить Питон и колёса.

Что делает скрипт по шагам: проверяет вход по SSH → берёт **последний тег** (`api.github.com/repos/<repo>/tags`) →
если нет `/data/opt/python` и `rbx/core`, ставит рунтайм (ассет `station-runtime-armv7.tar.gz` из релиза) →
заливает код тега в `/data/opt/rbx` → ставит `S99zzyandexoff` и `S99zzzrbxhost` → создаёт пустой `.env`,
если его нет → запускает глушение Алисы и хост.

## Чего `install.sh` НЕ делает (важно)

1. **Не ставит ядро диалога** `rbx/core`. Репозиторий `krikz/rob_box_dialog_core` пока пуст (Ш2 из ADR-0167
   не сделан), поэтому на чистой колонке после установки путь `/data/opt/rbx/core/scripts/dialogue/` отсутствует,
   и шаг с `.env` падает: `can't create …/.env: nonexistent directory`. На `10.1.1.141` ядро перенесено
   со старой колонки:

   ```bash
   ssh -p 2222 root@10.1.1.141 'killall python3.12'
   # потоком через ПК: с живой колонки в новую, без ключей и байткода
   ssh -p 2222 root@10.1.1.13  'cd /data/opt/rbx && tar cf - --exclude=__pycache__ --exclude=*.pyc --exclude=.env core' \
     | ssh -p 2222 root@10.1.1.141 'cd /data/opt/rbx && tar xf -'
   ```

2. **Не переносит ключи и настройки**: `.env` создаётся пустым (шаблон), `settings.json` не создаётся вообще.
   Без ключей хост не поднимется — с версии v0.1.7 он хотя бы отдаёт страницу :8080, чтобы ключи вписать.

3. **Не чистит** предыдущую установку: распаковывает поверх, файлы, удалённые из нового тега, остаются.

## Ключи и настройки

- `.env` — `/data/opt/rbx/core/scripts/dialogue/.env`, права `0600 root`. Минимум для работы: `MINIMAX_API_KEY`
  (STT/TTS) и `DEEPSEEK_API_KEY` (LLM). Остальные (`JEV_*`, `KIE_API`, `YANDEX_API_KEY`, `YANDEX_FOLDER_ID`,
  `MUSIC_CHOOSER_SHADOW`) — для прочих сервисов, для распознавания не используются.
- `settings.json` — выбор голоса и громкости. Голос «Весельчак» (`English_Jovialman`) появляется на странице
  только если в коде есть этот id (тег v0.1.7+), и **выбранным** он становится лишь после записи `RBX_VOICE`
  в `settings.json` (страница пишет туда при сохранении; перенос файла с другой колонки тоже работает).
- Страница `http://<IP>:8080` — запись разрешена только в окно после нажатия **любой** кнопки колонки.

## Перезапуск, остановка, проверка

```bash
ssh -p 2222 root@<IP> 'DELAY=0 /etc/init.d/S99zzzrbxhost restart'   # ручной рестарт
ssh -p 2222 root@<IP> '/etc/init.d/S99zzzrbxhost stop'              # погасить хост
ssh -p 2222 root@<IP> 'rm -f /data/opt/rbx_host && /etc/init.d/S99zzzrbxhost stop'  # погасить до перезагрузки
ssh -p 2222 root@<IP> 'touch /data/opt/rbx_host && DELAY=0 /etc/init.d/S99zzzrbxhost start'  # вернуть
ssh -p 2222 root@<IP> 'tail -20 /tmp/station.log'
```

Признак здорового старта в `station.log`:

```
env: /data/opt/rbx/core/scripts/dialogue/.env (8 перем.)
ядро готово за 0.3 с (вейк: игорёчек, …)
слушаю
версия колонки: v0.1.7
⚙ страница настроек: порт 8080
```

`S99zzzrbxhost start` **без `DELAY=0` спит 60 с** (это для загрузки, пока глушится Алиса) — при ручном
запуске это выглядит как «ничего не произошло».

## Две колонки одновременно: `STT 429`

MiniMax STT режет одновременные запросы с одного ключа: в одну и ту же секунду одна колонка получает текст,
а вторая — `HTTP 429`, хост пишет `STT 429 — пауза 20 с` и молчит. Наблюдение 08.11 (лог обеих колонок):

```
10.1.1.13  14:25:09 🎤 1.1 с → STT 1.58 с: «Игорёк, ты здесь?»  🔑 kws igorek 0.74
10.1.1.141 14:25:09 🎤 1.7 с → STT 0.98 с: «»                   → STT 429 — пауза 20 с
```

Что проверено по факту (burst 12 запросов с `10.1.1.141`, тишина 1 с):

```
2.2s  #1: ok
 minimax STT: rate-limited (HTTP 429) — falling back
3.4s  #2: ok … 21.6s #12: ok        итог: ok=12 err=0
```

То есть 429 **транзиентный**, и его имеет смысл пережидать (ретрай), а не молчать 20 с.

**Резервный провайдер:** у колонки только MiniMax (`stt_providers/minimax_provider.py`), Яндекса для STT нет.
Механика MiniMax → Yandex → Vosk написана для робота (`rob_box_voice/stt_fallback.py`, `stt_node.py`) и на
колонке не запустится: в `site` нет ни `grpc` (Yandex v3), ни `vosk`. Но **Yandex SpeechKit REST v1 работает
с нашим ключом** — проверено 08.11 на `10.1.1.141`:

```
ключ задан: True   folder задан: True
POST https://stt.api.cloud.yandex.net/speech/v1/stt:recognize  (lpcm 16k, 1 c тишины)
HTTP 200   body: {"result":""}
```

REST v1 — обычный HTTP (`httpx` на колонке уже есть), лимит 1 МБ на запрос: фраза 12 с × 16 кГц × 2 байта
= 384 КБ, влезает. Это дешёвый путь сделать fallback на Яндекса без gRPC.

## Рунтайм-ассет

`station-runtime-armv7.tar.gz` приложен к релизу v0.1.5 (`krikz/rob_box_station`), 15 477 753 б,
sha256 `c0e351da3a917dc851f88d645a2131a6d353de89838f129a25c07aa326cbb564`, внутри `python/` и `rbx/site/`.
**Собран с живой колонки** (`tar` по `/data/opt` без `__pycache__`), а не воспроизводимой сборкой —
это долг: нормальная сборка = `python-build-standalone` + `pip download --platform manylinux2014_armv7l`
(см. [inventory_2026-11-07.md](https://github.com/krikz/rob_box_station/blob/main/docs/inventory_2026-11-07.md)).

## Грабли при работе с колонкой из Windows

- **`curl … | sh` и `ssh` без `-n`**: скрипт читает сам себя из stdin, `ssh` съедает остаток и установка
  молча обрывается. В `install.sh` для одиночных вызовов `ssh -n`, для потоковых (`tar | ssh`) — без `-n`.
- **`raw.githubusercontent.com` кэширует до ~5 минут**: после мёржа правка в `main` не видна сразу.
  Проверять по `api.github.com/repos/<repo>/contents/<file>`, а не по raw.
- **Кириллица через пайп в native-программу**: `@'…'@ | python -` на Windows уходит в cp866, и в GitHub
  прилетают `???`. Текст писать файлом в UTF-8 (`[IO.File]::WriteAllText(...)`) и передавать `--input`/`-F`.
- **`adb shell '…'` с `|` и кавычками**: PowerShell ломает аргументы (`sh: yandex: not found`). Длинные
  команды — отдельным файлом и `ssh … 'sh -s' < file`.
