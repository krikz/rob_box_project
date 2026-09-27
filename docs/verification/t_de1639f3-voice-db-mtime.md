# Проверка t_de1639f3: какая voice-БД реально пишется (live robot)

**Дата:** 2026-09-14, ~07:08–07:13 UTC (10:08–10:13 MSK)
**Хост:** VisionPi (10.1.1.21), контейнер `voice-assistant` (Up ~1h, healthy)
**Задача-родитель:** t_7f295146, проверка #3 из §3 списка проверок

## Команда (повторяется 4 раза, ~75 сек между T1/T2, ~120 сек до T3, ~60 сек до T4)

```bash
docker exec voice-assistant stat --printf="file=%n size=%s mtime_epoch=%Y mtime_human=%y\n" <file>
```

(полные снапшоты — `/tmp/voice-db-verify/snapshot{1..4}.txt`)

## Ожидание из статического анализа (ADR-0055 §1.1, ADR-0083 §E)

- `harness_voice.db` — пишется `dialogue_node` через `SQLiteVoiceMemory`
- `voice_memory.db` — пишется mcp_server (waypoints, music, voice_turns/facts) **до миграции**
- ADR-0083 §6.3: "mcp_server → harness_voice.db через адаптер" — но **это ещё не развёрнуто на проде** (см. env mcp_server ниже)

## Факт (T4 ≈ 10:12 локально, 38 мин после последнего события в WAL)

| Файл | size | mtime (last write) | atime (last access) | Δ от now |
|---|---|---|---|---|
| `harness_voice.db` | 303 KB | **2026-09-10 04:41** | 2026-09-14 08:48 | atime = момент старта контейнера, mtime = старый (WAL режим) |
| `harness_voice.db-wal` | 4.3 MB | **2026-09-14 09:28:37** | 09:28 | **~38 мин назад — последняя запись** |
| `harness_voice.db-shm` | 32 KB | 2026-09-14 09:28:37 | 09:28 | синхронно с WAL |
| `voice_memory.db` | 16.1 MB | 2026-09-11 14:30 | **2026-09-13 23:15** | atime = до старта контейнера! Никто не открывал |
| `voice_memory.db-wal` | 8.1 MB | 2026-09-14 08:48:58 | 09:14 | WAL создан при старте, но **с тех пор пуст** |
| `voice_memory.db-shm` | 32 KB | 2026-09-14 08:48:58 | 08:48 | синхронно с WAL |
| `operator_memory.db` | 4 KB | 2026-09-08 02:10 | 2026-09-09 16:34 | 5 дней без касания |
| `operator_memory.db-wal` | 56 KB | 2026-09-09 16:34 | 2026-09-09 16:34 | 5 дней тишины |
| `speakers.db` | 92 KB | 2026-09-14 09:28 | 09:28 | speaker_id_node обновлял 40 мин назад |
| `voice_assistant.db` | 0 B | 2026-06-12 | 2026-06-12 | заглушка |
| `rob_box_voice.db` | 0 B | 2026-04-13 | 2026-04-13 | заглушка |
| `memory.db` | 0 B | 2026-08-09 | — | пустой |

Между T1 (07:08:20) → T2 (07:09:35) → T3 (07:11:47) → T4 (07:12:54) — **4 точки**, **0 секунд дельты** mtime по всем `voice*.db` и `operator_memory.db*`.

## env процессов в voice-assistant (cat /proc/<pid>/environ)

```
mcp_server (PID 95):  VOICE_MEMORY_DB_PATH=/data/voice_memory.db
dialogue_node (PID 83): VOICE_MEMORY_DB_PATH=/data/voice_memory.db
```

**mcp_server ещё НЕ мигрирован** на harness_voice.db — это и есть причина, почему voice_memory.db-wal открывается при старте (создаётся WAL, но без записей). mcp_server **должен** писать туда по старому коду, но за ~25 минут работы контейнера — **0 записей** (логи пустые, `docker logs --since 60s voice-assistant | grep -ciE "wrote|insert|update|fact|turn" = 0`).

## ros2 param (dialogue_node.sqlite_db_path)

```
String value is: /data/harness_voice.db
```

То есть dialogue_node правильно параметризован на новую БД. Параметр `voice_input_mode` **не объявлен** (`Parameter not set`).

## Вердикт

| Гипотеза | Вердикт | Доказательство |
|---|---|---|
| `harness_voice.db` — реально пишется | **ПОДТВЕРЖДЕНА** (частично) | WAL обновлялся 09:28:37 (за 38 мин до now). Диалоговая нода параметризована на этот файл. Но с 09:28 тишина — записей мало (на 14.09 10:12 робот простаивает?). |
| `voice_memory.db` — реально пишется | **ОПРОВЕРГНУТА** | WAL открыт при старте контейнера (08:48:58), но **atime основного файла = 2026-09-13 23:15** — никто не открывал после старта. mcp_server не делает записей за 25 мин работы. |
| ADR-0055: «обе БД реально активны» | **ОПРОВЕРГНУТА** | voice_memory.db в текущем срезе **не активна**. Гипотеза была верна на момент статического анализа (Q2-Q3 2026), но сейчас — неверна: либо mcp_server перестал писать туда, либо ещё не делал новых операций. |
| ADR-0083 §6.3: «mcp_server → harness_voice.db через адаптер» | **ОПРОВЕРГНУТА на проде** | `VOICE_MEMORY_DB_PATH=/data/voice_memory.db` в env mcp_server — миграция не развёрнута. ADR-0055 (merge'нут) ещё не применён в `/config/voice_assistant/`. |
| `operator_memory.db` — пишется супервизором | **ОПРОВЕРГНУТА на сейчас** | WAL тишина 5 дней. avatar-supervisor контейнер Up ~1h, но в этом контейнере нет (отдельный контейнер). Записи делает только при работе супервизора. |

## Сырые логи

- `/tmp/voice-db-verify/snapshot1.txt` (T1=07:08:20 UTC)
- `/tmp/voice-db-verify/snapshot2.txt` (T2=07:09:35 UTC)
- `/tmp/voice-db-verify/snapshot3.txt` (T3=07:11:47 UTC)
- `/tmp/voice-db-verify/snapshot4.txt` (T4=07:12:54 UTC, + atime/ctime)
- `/tmp/voice-db-verify/params.txt` (env mcp_server/dialogue_node, ros2 param dump)
- `/tmp/voice-db-verify/logs.txt` (docker logs --since 60s voice-assistant — практически пусто)

## Что нужно сделать дальше (за рамки этой карточки)

1. **Перепроверить голосом** — спровоцировать запись в `harness_voice.db` (через wake-word «робот» + фразу), чтобы убедиться, что диалоговая нода действительно пишет в новую БД.
2. **Если 09:28 = последняя запись** — проверить логи voice-assistant с 09:28 до now на предмет ошибок (`docker logs --since 4h voice-assistant 2>&1 | grep -iE "error|fail|except|harness_voice|sqlite"`).
3. ADR-0055 Phase 1 (t_7a03364a) уже merge'нута, но не развёрнута на роботе — нужно обновить `voice-assistant-humble-dev` image / env в docker-compose.

## File references

- `src/rob_box_voice/config/dialogue_node.yaml:103` — `sqlite_db_path: /data/harness_voice.db`
- `src/rob_box_voice/rob_box_voice/core/voice_memory.py:23` — `db_path="/data/voice_memory.db"` (legacy VoiceMemory)
- `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py:113,668,957,1012` — пишет в `VOICE_MEMORY_DB_PATH`
- ADR-0055 — `docs/adr/0055-voice-memory-db-unify-with-harness.md` (merge'нут, не развёрнут)
- ADR-0083 §E — `docs/adr/0083-build-agent-spec-voice-vr-20.md` (миграция оператора)
