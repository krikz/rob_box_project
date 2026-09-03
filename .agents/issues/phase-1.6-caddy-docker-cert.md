## Phase 1.6 — Caddy + Docker-сервис + cert-логика

### Источник истины
- Дизайн (общий): `docs/plans/2026-08-24-meta-quest-telepresence.md` §Phase 1.6
- ADR HTTPS-стека: `docs/adr/0027-meta-quest-ar-control.md` §4.1 (Caddy выбран)
- API-контракт: `docs/architecture/meta-quest-api.md` §1, §3
- Phase 1.5 (клиент): commit b720c8af + последующие
- Phase 1.4 (сервер): commit b720c8af
- **Эталон Docker-сервиса на Vision Pi**: PR #1633 (AV-9 supervisor, merged 25.08.2026) — три файла, +181 строка, без раздутия
- **Опыт/антипаттерн**: PR #111 из [GOODWORKRINKZ/microbbox](https://github.com/GOODWORKRINKZ/microbbox/pull/111) (OPEN, 23 файла, +4900 строк, три режима cert в одном PR) — **не повторяем**

### Контекст

Phase 1.1–1.5 готовы: Python aiohttp WSS-сервер `rob_box_quest` отдаёт JPEG/LiDAR/teleop, Three.js клиент собирается Vite'ом. Не хватает **транспорта**: HTTPS-терминации на Vision Pi (10.1.1.11:8443) и Docker-сервиса для деплоя.

### Уроки из PR #111 (microbbox) — почему НЕ делаем так

| Что сделано в #111 | Почему плохо | Что делаем мы |
|---|---|---|
| 23 файла в одном PR, +4900 строк | Один PR = десяток решений, ревью невозможно | **3-5 файлов, ≤300 строк** |
| 3 режима cert (prod/staging/self-signed) сразу | YAGNI, нам нужен только self-signed для лабы | Только self-signed, prod — tech-debt |
| certbot + Let's Encrypt + renewal-цикл | Нет публичного домена, 80-й порт наружу не нужен | **Без certbot**, openssl на старте контейнера |
| 3200+ строк документации в PR | Документация отдельно, не блокирует merge | README 30-50 строк + ссылка на ADR |
| Шаблон nginx-конфига + sed-генерация | Overkill для 1 устройства | Один статический Caddyfile, без шаблонов |
| nginx | ADR-0027 §4.1 уже выбрал **Caddy** (hot-reload, единый бинарник, читаемый Caddyfile) | **Caddy** |

### Уроки из PR #33 (microbbox, MERGED) — что БЕРЁМ

- **Permissions-Policy заголовки** для WebXR (accelerometer, camera, gyroscope, magnetometer, xr-spatial-tracking) — на Quest критично
- **`proxy_buffering off`** для стриминга — низкая latency
- **WebSocket proxy** через `proxy_http_version 1.1` + `Upgrade`/`Connection` headers — для WSS
- **HSTS** (`Strict-Transport-Security`) — для браузеров кэшировать HTTPS-only
- **TLS 1.2/1.3 only** — без legacy

### Уроки из нашего PR #1633 (AV-9 supervisor, MERGED) — точный паттерн Docker-сервиса

- **3 файла**: `Dockerfile`, `start_*.sh`, **±50 строк** в `docker-compose.yaml`
- `network_mode: host`, `depends_on: zenoh-router (healthy)`, `restart: unless-stopped`, `healthcheck: pgrep -f`
- Volume-монтирования: `./config:/config:ro`, `./scripts:/ros_scripts:ro`, `./scripts/<svc>:/scripts:ro`
- ❌ НЕ `COPY config/`, ❌ НЕ `COPY scripts/` (DOCKER_STANDARDS.md)

### Что строим (минимум, чтобы Phase 1.7 e2e работала)

**3 новых файла, 1 модификация:**

1. `docker/vision/quest/Dockerfile` — multi-stage: `node:20-alpine` builder (Vite-сборка `webxr_client/dist/`) → runtime: `caddy:2-alpine` + aiohttp Python-server (`rob_box_quest`)
2. `docker/vision/quest/Caddyfile` — единый статический файл (без sed-шаблонов), reverse_proxy на `localhost:8765` (aiohttp), root на статику из builder-стадии
3. `docker/vision/scripts/quest/start_quest.sh` — генерит self-signed cert (если нет), запускает `rob_box_quest` + `caddy run --config /etc/caddy/Caddyfile`
4. `docker/vision/docker-compose.yaml` — сервис `quest` по образцу `supervisor` из PR #1633: `network_mode: host`, `depends_on: zenoh-router (healthy)`, volumes, healthcheck

**ВНЕ Phase 1.6 (явно):**
- ❌ certbot / Let's Encrypt — нет публичного домена (ADR-0027 §4.1)
- ❌ H.264 streaming — Phase 2
- ❌ Multi-device templates — один робот
- ❌ 3200 строк документации — README + ADR достаточно
- ❌ admin panel / observability — Phase 2

### Caddyfile (минимальный, ADR-0027 §4.1 + PR #33 lessons)

```
{
    auto_https off
    admin off
}

:8443 {
    tls /certs/selfsigned.crt /certs/selfsigned.key {
        protocols tls1.2 tls1.3
    }

    # Permissions-Policy для WebXR (PR #33 lessons)
    header {
        Strict-Transport-Security "max-age=31536000; includeSubDomains"
        Permissions-Policy "accelerometer=*, camera=*, gyroscope=*, magnetometer=*, xr-spatial-tracking=*"
        # Отключаем buffering для WebSocket и стримов (PR #33 lessons)
        X-Content-Type-Options "nosniff"
        -Server
    }

    # WebXR статика (dist/ из Vite build)
    root * /srv/quest_static
    encode gzip zstd
    file_server

    # Health check (для self-host observability + compose healthcheck)
    handle /healthz {
        reverse_proxy localhost:8765
    }

    # Всё остальное → aiohttp WSS (WebSocket + binary frames)
    reverse_proxy localhost:8765 {
        # PR #33: без buffering для WebSocket/стримов
        flush_interval -1
        transport http {
            versions h2c 1.1
        }
    }
}
```

### Dockerfile (multi-stage)

```dockerfile
# Stage 1: Vite build webxr_client
FROM node:20-alpine AS builder
WORKDIR /build
COPY src/rob_box_quest/webxr_client/package*.json ./
RUN npm ci
COPY src/rob_box_quest/webxr_client/ ./
RUN npm run build

# Stage 2: Runtime (Caddy + Python aiohttp)
FROM caddy:2-alpine
RUN apk add --no-cache python3 py3-pip openssl bash
# aiohttp + rob_box_quest — из ROS base image? см. вопрос 1
COPY --from=builder /build/dist /srv/quest_static
COPY docker/vision/quest/Caddyfile /etc/caddy/Caddyfile
COPY docker/vision/scripts/quest/start_quest.sh /scripts/start_quest.sh
RUN chmod +x /scripts/start_quest.sh
EXPOSE 8443
ENTRYPOINT ["/scripts/start_quest.sh"]
```

### start_quest.sh (логика)

1. Ждём Zenoh (как `start_supervisor.sh` — `ZENOH_ROUTER_CHECK_ATTEMPTS=10`)
2. Генерим self-signed cert, если нет (`openssl req -x509 -newkey rsa:2048 -days 365 -nodes -subj "/CN=quest.rob_box.local" -addext "subjectAltName=DNS:quest.rob_box.local,IP:10.1.1.11"`)
3. Запускаем `caddy run --config /etc/caddy/Caddyfile &`
4. Запускаем `ros2 run rob_box_quest quest_node` (или через `ros_with_namespace.sh`)
5. PID 1 = основной процесс, Caddy в фоне (или наоборот — supervisord-like, см. вопрос 2)

### Definition of Done

- [ ] `docker build` (на CI self-hosted runner, не локально) — exit 0
- [ ] `docker compose up -d quest` — контейнер стартует, healthcheck `pgrep -f 'caddy|quest_node'` → 0
- [ ] `docker logs quest` — последние 30 строк: cert generated, Caddy listening on :8443, ROS node ready, PIN в логах
- [ ] `curl -k https://10.1.1.11:8443/healthz` → 200 OK (с self-signed cert через `-k`)
- [ ] `curl -k -I https://10.1.1.11:8443/` → 200, заголовки `Strict-Transport-Security`, `Permissions-Policy` присутствуют
- [ ] `curl -k https://10.1.1.11:8443/index.html` → Vite-билд (HTML с `<script type="module">`)
- [ ] `docker exec quest ls /srv/quest_static` → `index.html`, `assets/*.js`, `assets/*.css` (из Phase 1.5)
- [ ] Статика `< 1.5 MB gzipped` (Phase 1.5 DoD)
- [ ] Нет `COPY config/`, нет `COPY scripts/` в Dockerfile
- [ ] `network_mode: host`, `depends_on: zenoh-router (healthy)`, `restart: unless-stopped`
- [ ] Volume `./scripts/quest:/scripts:ro`, `./config:/config:ro` — смонтированы
- [ ] **Размер PR**: ≤ 5 файлов, ≤ 300 строк, +1 сервис-блок в compose (по образцу PR #1633)
- [ ] Commit: `feat(quest): Caddy + self-signed TLS + docker service on Vision Pi (Phase 1.6)`
- [ ] **Raw-evidence в PR** (ADR-0018): ссылки на `docker logs`, `curl -I` output, `docker exec ls`, `docker compose ps`

### Открытые вопросы для Шифу

1. **Base image для runtime-стадии**: использовать наш `ghcr.io/krikz/rob_box:ros2-zenoh-humble-*` (с rclpy/aiohttp/msgpack) или взять `caddy:2-alpine` + отдельный pip-install aiohttp? Первое — единый image, дольше build. Второе — чище, но два image'а в compose.
   - **Моя рекомендация**: `caddy:2-alpine` + pip-install `aiohttp msgpack rclpy` лёгкий — НЕ тянуть весь pytorch/Silero из voice-assistant image. Аналогия с `supervisor` (PR #1633), который наследует voice-assistant, потому что зависимости общие. У `quest` зависимости легче.

2. **Supervisord vs shell-обёртка**: один entrypoint скрипт, который запускает Caddy в фоне + ROS-ноду (или наоборот), или полноценный supervisord? Простота → shell + `&` + `wait $!`. Robust → supervisord. **Рекомендация**: shell с trap для graceful-shutdown (kill обоих процессов).

3. **PIN-генерация**: где генерируется 6-значный PIN (ADR-0027 §4.5)? В `rob_box_quest.quest_node` (Python) или в `start_quest.sh` (bash)? Логичнее в Python — там же валидация. **Рекомендация**: в Python (уже TODO по Phase 1.5 plan).

### Acceptance (raw-evidence обязателен, ADR-0018)

- [ ] `docker compose ps` — статус `Up (healthy)` для `quest`
- [ ] `docker logs quest --tail=30` — полный вывод
- [ ] `curl -kI https://10.1.1.11:8443/` — заголовки + status
- [ ] `curl -k https://10.1.1.11:8443/healthz` — body
- [ ] `docker exec quest ls -la /srv/quest_static` — список файлов
- [ ] `docker exec quest openssl x509 -in /certs/selfsigned.crt -noout -subject -dates` — cert info
- [ ] `docker inspect quest --format '{{json .State.Health}}'` — health state
- [ ] На dev-машине (без Quest'а): `curl -k https://10.1.1.11:8443/` → HTML страница Vite-билда

### Phase 1.7 (отдельная карточка, не Phase 1.6)

- DNS `quest.rob_box.local → 10.1.1.11` (✅ готово 25.08.2026)
- Импорт self-signed в Quest (Settings → Privacy → Security → Trusted Sources)
- E2E: открыть `https://quest.rob_box.local`, ввести PIN, видеть 4 panels + LiDAR
- Teleop (WASD), grip dead-man
- Wi-Fi обрыв → safe-stop ≤ 500 мс

---

> *«Минимум кода, максимум композиции. PR #111 OPEN, PR #1633 MERGED — учись у merged.»*
