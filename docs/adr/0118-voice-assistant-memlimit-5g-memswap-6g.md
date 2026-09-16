# ADR-0118 — voice-assistant: mem_limit 4g → 5g + memswap_limit=6g (страховка от каскадных OOM kill)

| Поле | Значение |
|---|---|
| Статус | **Proposed** (kanban `t_880acd3a`, issue #2676) |
| Дата | 2026-09-16 |
| Автор | devops worker (Hermes Agent, kanban t_880acd3a, issue #2676) |
| Контекст | После ночного инцидента #2648 Vision Pi (10.1.1.21) поднялся, но `voice-assistant` контейнер работает в режиме OOM-жонгля: MEM USAGE 3.417 / 4 GiB = 85.42%, прыгало до 99.64%. Каскадные SIGKILL рестарты `tts_node` / `stt_node` / `speaker_id_node` (exit code -9) каждые ~30с. Голосовой e2e не проходит — STT упал, TTS перезапускается, health endpoint `[]`. |
| Затрагивает | `docker/vision/docker-compose.yaml` (только voice-assistant service: `mem_limit`, `memswap_limit`, комментарии) |
| Родители | issue #2676 (root), ADR-0111 (zram-swap + mem_limit для Vision Pi — ещё не задеплоен), ADR-0018 (честный FAIL), #929 (предыдущий OOM-kill tts_node, бампнули 2g→4g), #2648 (Vision Pi ночной инцидент) |
| Связанные | issue #2609 (torch CPU-only, чужая зона — backend), issue #2621 (Vision Pi zram-swap — kanban t_22362046, ещё не смержен), `docs/investigations/929-oom-findings-wip.md`, `docs/development/DOCKER_STANDARDS.md` §7 |

> **TL;DR.** На voice-assistant контейнере поднимаем `mem_limit` 4 GiB → 5 GiB
> и добавляем `memswap_limit: 6g`. Это даёт ~1 GB RAM headroom для cold-start
> всех 9 нод + Silero warm-load + всплески диалогового трафика, и 1 GB swap
> cushion на короткие пики (пока zram-swap по ADR-0111 ещё не задеплоен —
> file-based swap медленный, но это страховка от OOM-killer). Решение
> **временное**, не лечит причину (RSS 9 нод + PyTorch + Vosk = ~3.9 GB в
> одном cgroup). Долгосрочное решение — вынос тяжёлых нод в отдельные
> контейнеры, **НЕ в этой карточке** (это backend work, см. issue #2676 §2).

---

## 1. Проблема

### 1.1. Сырые замеры 2026-09-16 (Vision Pi, issue #2676)

После ночного инцидента #2648 Vision Pi поднялся, и `voice-assistant`
показывает OOM-жонгл:

```bash
$ docker stats voice-assistant --no-stream
# → MEM USAGE / LIMIT   MEM %
#   3.417GiB / 4GiB     85.42%
# (прыгало до 99.64%)

$ docker logs voice-assistant --tail 600 2>&1 | grep "process has died"
# → [ERROR] [tts_node-5]: process has died [pid 2988, exit code -9]
# → [ERROR] [stt_node-6]: process has died [pid 105,  exit code -9]
# → [ERROR] [speaker_id_node-9]: process has died [pid 111, exit code -9]

$ curl -sf -m 5 http://127.0.0.1:8000/health; echo
# → []

$ free -h
# → MemAvailable: 951Mi, Swap used: 1.8/1.9Gi
```

Контейнер `restart: always` маскирует OOM-killer рестарты — `docker ps`
показывает `Up 14 minutes (healthy)` хотя внутри ноды постоянно падают.

### 1.2. Голосовой e2e не проходит

- **STT упал** → голос не распознаётся («❌ ОТКЛЮЧЕНО (пустое)»)
- **TTS перезапускается** каждые ~30с (exit -9) → каждый рестарт = задержка
  ответа > 5с
- **health endpoint `[]`** — оператор/мониторинг не видит проблему
- **Blocker для production deploy** — runbook #2652 готов, но не поможет
  пока не вылечим OOM

### 1.3. Бюджет памяти

Контейнер хостит **9 respawn-able нод**:

| нода | RSS (примерно) |
|---|---|
| audio_node (PyAudio + ReSpeaker 6ch buffers) | ~150 MB |
| stt_node (Vosk model) | 200-500 MB |
| sound_node (sclang + scsynth + FoxDot) | 300-500 MB |
| tts_node (PyTorch + Silero v5 warm) | 150-200 MB |
| dialogue_node + mcp_server | 200-400 MB |
| animation_player + led + command | ~100 MB |
| **итого RSS** | **~3.4-3.9 GB с пиками до 4+** |

При cold-start всех нод одновременно + Silero warm-load пик превышает 4 GB
→ cgroup OOM-killer → SIGKILL exit code -9 → каскад.

---

## 2. Решение

### 2.1. Что меняем

`docker/vision/docker-compose.yaml`, секция `voice-assistant`:

```yaml
    mem_limit: 5g  # было 4g
    memswap_limit: 6g  # RAM 5 GB + swap до 1 GB (issue #2676, ADR-0118)
```

- `mem_limit: 5g` — даёт ~1 GB headroom для cold-start всех 9 нод + Silero
  warm-load + всплески диалогового трафика
- `memswap_limit: 6g` — разрешает контейнеру занять до 5 GB RAM + 1 GB swap
  поверх. На Vision Pi сейчас dphys-swapfile 2 GB (file-based, медленный),
  zram-swap по ADR-0111 ещё не поднят — но даже file-based swap даёт буфер
  на короткие пики, чтобы OOM-killer не убивал ноды сразу

### 2.2. Почему не 6-8 GB (issue #2676 §1)

Хост Vision Pi = 8 GB. Уже сейчас **сумма mem_limit контейнеров ≈ 9.5 GB**:

| контейнер | mem_limit |
|---|---|
| voice-assistant | 4g → **5g** |
| oak-d | 6g |
| vision-hailo | 1g |
| vision-face | 1g |
| telegram-bot | 512m |
| **итого** | **12.5 GB по лимитам на 8 GB хосте** |

Поднять voice-assistant до 6-8 GB = лимиты перестают быть защитой (сумма
> RAM = все контейнеры будут драться за одну память, кто-то получит
OOM-killer первым — но это будет случайный сосед, не виновник).
mem_limit должен быть **≤ реальный RSS в пике**, а не «сколько влезет».

5 GB — это RSS пик + 25% headroom (3.9 GB × 1.27 ≈ 5 GB), что соответствует
`docs/development/DOCKER_STANDARDS.md` §7 (overcommit ≤ 30%).

### 2.3. Почему memswap_limit = mem_limit + 1 GB

- memswap_limit = mem_limit: container can use ONLY RAM, no swap → OOM-killer
  при первом же пике > mem_limit (старая проблема #929)
- memswap_limit = mem_limit + 1 GB: при пике ядро может вытеснить 1 GB
  анонимных страниц в swap → нода выживает на ~5-6 секунд дольше →
  успевает завершить текущий TTS-запрос / Yandex gRPC retry → OOM-killer
  не вызывается
- memswap_limit >> mem_limit: container может сожрать весь swap →
  остальные контейнеры начнут свопиться → деградация всех

---

## 3. Альтернативы (отклонённые)

### 3.1. Вынести тяжёлые ноды в отдельные контейнеры (issue #2676 §2)

Правильное долгосрочное решение, **но**:
- Требует рефакторинга ros2 launch-топологии (сейчас все ноды стартуют
  одним `ros2 launch voice_assistant` внутри одного контейнера)
- Каждый контейнер = свои ROS_DOMAIN_ID discovery, ipc=host, /dev/shm
- Это **backend work** (agent:backend, agent:ros2), не DevOps
- Будет оформлено отдельной карточкой после #2676

### 3.2. Установить zram-swap по ADR-0111 (issue #2621)

Тоже правильно, **но**:
- Kanban t_22362046 ещё в работе (статус: ready на момент написания
  этого ADR)
- Зависит от того, задеплоит ли оператор `robbox-zram.service` на Vision Pi
- Даже **после** установки zram наш 5g+1g swap buffer останется
  разумной защитой (zram эффективнее file-based, но всё равно не
  бесплатен — CPU на сжатие, латентность)

### 3.3. Уменьшить RSS voice-assistant (torch CPU-only, #2609)

Кардинальное решение (1.6 GB → 0.5 GB за счёт выкидывания CUDA), **но**:
- Kanban t_e4e4c4e6 (или аналогичная) ещё в работе
- Тоже **backend work**

---

## 4. Что НЕ делаем

- **Не трогаем** health endpoint (issue #2676 §3) — это backend work, см.
  rob_box_voice/action_server/http.py. healthz уже есть, но не покрывает
  список нод и их статусы (OK/DEGRADED/DOWN). Требует рефакторинга
  health-router чтобы привязать к каждой ноде — это новая фича, не блокер.
- **Не убираем** `restart: always` (issue #2676 Actual §1) — это маскирует
  OOM, **но** без него каждый SIGKILL → оператор должен рестартить руками.
  После фикса лимита каскадные SIGKILL уйдут → health endpoint начнёт
  возвращать ненулевой список → мониторинг увидит.

---

## 5. Как проверить

1. **После раскатки PR** на Vision Pi (10.1.1.21):
   ```bash
   ssh ros2@10.1.1.21
   cd /opt/rob_box_project
   docker compose -f docker/vision/docker-compose.yaml up -d voice-assistant
   sleep 60  # дать контейнеру подняться и warm-load Silero
   docker stats voice-assistant --no-stream
   # ожидаем: MEM USAGE ≤ 5.0GiB, MEM % ≤ 100%
   docker logs voice-assistant --tail 600 2>&1 | grep "process has died"
   # ожидаем: пусто (или только старые записи до рестарта)
   curl -sf -m 5 http://127.0.0.1:8000/health; echo
   # всё ещё [], это OK — health endpoint НЕ покрывает ноды voice-assistant
   # (см. §4 — это отдельная задача)
   ```
2. **Прогон голосового e2e** через hermes e2e-process сценарий (после
   merge-gate):
   - wake word «робот» → STT распознаёт → dialogue → TTS отвечает → нет
     задержки > 5с
3. **Стресс-тест** (опционально): запустить 10 диалогов подряд, проверить
   что RSS не уходит выше 5 GB и нет каскадных SIGKILL

---

## 6. Откат

Если 5g+1g swap **недостаточно** (RSS пик > 6 GB), откатываемся одним
коммитом:

```bash
git revert <this-commit>
docker compose -f docker/vision/docker-compose.yaml up -d voice-assistant
```

Лимиты вернутся к 4 GB. Дальше — открывать issue на backend work (вынос
Vosk STT / Silero / MiniMax T2A в отдельные контейнеры) или
до-ставить zram по ADR-0111.

---

## 7. Связанные изменения

- `docker/vision/docker-compose.yaml`: +2 строки (`mem_limit`, `memswap_limit`),
  +22 строки комментариев, −2 строки старого `memswap_limit: 4.5g`
- Без новых файлов, без новых зависимостей, без изменений кода
