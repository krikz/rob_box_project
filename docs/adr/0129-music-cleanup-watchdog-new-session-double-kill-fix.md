# ADR-0129 — Гард «свежей музыки» для трёх механизмов auto-stop: cleanup-finalize, watchdog segments_deadline, music_cleanup reason=new_session

**Дата:** 2026-09-24
**Статус:** Accepted — дизайн зафиксирован, два из трёх фиксов уже в `z-{agent}/3005-...` (commit `d622fb19e`, `dccf9b648`), третий фиксируется отдельной карточкой
**Автор:** architect
**Связанные:** issue #3005 (флап «старт → стоп → старт»), issue #3004 (Bug C «ретраит успешный compose_music» — соседний баг, фикс уже в develop через commit `149f755f8`), issue #935 (исходный safety-net `_on_music_cleanup` → `MusicManager.stop_music_on_session_end`), issue #3007 (фикс persona-сетов — параллельная ветка, ADR-0129 не зависит от неё)
**Branch:** `z-{agent}/3005-fix-voice-music-cleanup-watchdog-new-session` (rebase на origin/develop, push ожидается этим коммитом)

---

## 1. Контекст и проблема

Голосовой стек на живом роботе (Vision Pi, live-сессия 24.09.2026 ~14:50–14:55 UTC, LLM: MiniMax, диалог на русском) воспроизвёл **флап «старт → стоп → старт»** на одном музыкальном запросе юзера. Из логов:

```
14:50:39 WARN 🎵 [watchdog] Авто-стоп 2 паттернов: reason=segments_deadline idle=45.2s ttl=1800s
14:51:04 INFO 🎵 music_cleanup deferred — waiting for TTS or 10s fallback
14:51:04 INFO music_cleanup sent: reason=tts_batch_complete
14:51:05 INFO 🎵 [tts_batch_complete] Cleanup: активной музыки не обнаружено (stop_all вызван профилактически)
14:54:47 INFO 🎧 DJ Mode OFF  (×2)
14:54:47 INFO music_cleanup sent: reason=new_session
14:54:47 WARN 🎵 [new_session] Авто-стоп 1 активных паттернов (issue #935). msg=Диалог завершился с активной музыкой
```

Слушатель слышит «шляпу»: обрывы треков, старт-стоп-старт. Корень один и тот же — **три конкурирующих механизма auto-stop не согласованы между собой и не различают «свежезапущенную по запросу юзера музыку» от «забытого трека, который пора глушить»**.

### 1.1 Что считать «свежей» музыкой

Эмпирический критерий из issue #3005 acceptance:
- Музыка **была запущена** в текущем или предыдущем LLM-туре **явным вызовом** music-тула (`compose_music`/`execute_music_code`/`DJ_RETRY`/`USER_RETRY` с music-вердиктом).
- Юзер не произносил «стоп»/«хватит»/«выключи музыку»/«смени трек» **после** этого момента.
- С момента старта прошло **меньше** `idle_ttl` (по умолчанию 1800 с) — юзер всё ещё в диалоге.

Если все три условия выполнены — это user-requested трек, его **нельзя** глушить ни watchdog'у (segments_deadline), ни tts_batch_complete cleanup'у, ни new_session cleanup'у.

### 1.2 Три независимых механизма, которые надо скоординировать

| # | Механизм | Код | Что делает | Когда стреляет |
|---|---|---|---|---|
| A | `cleanup-finalize` в `_run_turn` | `dialogue_node.py:5262` (`_finalize_music_cleanup_policy`) → `dialogue_node.py:2800` (`_publish_music_cleanup(reason="tts_batch_complete")`) | `_pending_music_cleanup=True` ставится в `_apply_music_guard`, на ближайшем `tts_batch_complete` публикует `music_cleanup` на `/mcp/music_cleanup` | На **каждом** tts_batch_complete, если у текущего хода есть pending cleanup (issue #980) |
| B | watchdog `segments_deadline` | `music.py:1853` (`MusicManager.maybe_stop`) → `music.py:1881` (`self.stop_all()` если `now_m >= deadline`) | Если `compose_music(segments=8)` поставил deadline через ~60 с, watchdog гасит музыку, даже если idle_ttl ещё не сработал (issue #990) | Каждые ~5 с — тик watchdog'а из `MusicManager.tick()` |
| C | `music_cleanup(reason="new_session")` | `dialogue_node.py:7802` (`_reset_dialogue_session`) → через `dialogue_node.py:5256`-ish (для «новой сессии» через `_publish_music_cleanup` — см. §3.3 ниже) | Когда юзер говорит «новая сессия»/«сбрось всё»/Telegram `/clear`, диалог сбрасывается, и cleanup публикуется с причиной `new_session` | На `reset_session` из `_on_stt` или из Telegram команды |

Все три в live-логах стреляли на **одном и том же** музыкальном запросе в окне ~5 минут.

---

## 2. Решение — общий принцип: «гард свежей музыки» в один такт

Каждый из трёх механизмов обязан проверить **три условия** перед тем, как убивать:

```python
def is_user_requested_fresh_music(node, music_manager) -> bool:
    """Issue #3005 — если LLM только что запустил музыку по запросу юзера,
    никакой cleanup/watchdog/new_session не должен её гасить в течение
    idle_ttl (1800 с по умолчанию).
    """
    # 1. Свежесть: idle < idle_ttl И юзер всё ещё в диалоге.
    #    Проверяется на стороне MCP через last_user_activity_at / clock.
    # 2. Stop intent: после момента запуска музыки не было user_stop,
    #    barge_in со stop-интентом, или guard 'stop_command_guard'.
    # 3. Music guard track: предыдущий ход завершился в retry-сценарии
    #    (USER_RETRY/DJ_RETRY), значит cleanup ещё не пришёл.
    ...
```

Гард инвариантен в одну сторону: если **все три** условия выполнены — **отказать** в остановке. Если хоть одно нарушено — старый поведенческий путь (issue #935 safety-net срабатывает штатно).

### 2.1 Механизм A: cleanup-finalize в `_run_turn`

**Что было:** `_apply_music_guard` стоял **ПОСЛЕ** `_finalize_music_cleanup_policy`. На ходе «спой/сыграй» без явного вызова тула cleanup-finalize успевал вооружить `_pending_music_cleanup=True` → catch-up в `_flush_music_cleanup_if_idle` тут же публиковал `music_cleanup(reason="tts_batch_complete")` → mcp_server гасил Renardo за секунды **до** ретрая Bug C, который только ещё собирался диспатчить новый `compose_music`.

**Фикс (уже в branch, commit `d622fb19e`):** запустить `_apply_music_guard` ДО `_finalize_music_cleanup_policy`. Если guard вернёт `music_retry_dispatched=True` (USER_RETRY или DJ_RETRY) — outer-finalize **пропускается**, `_pending_music_cleanup` сбрасывается в `False`, ретрай-тур сам отработает cleanup в своём `finally`.

```python
# dialoge_node.py:_run_turn (было)
self._drain_pending_user_messages()
self._finalize_music_cleanup_policy(...)   # ← стрелял первым
music_retry_dispatched = self._apply_music_guard(...)

# dialoge_node.py:_run_turn (стало, после d622fb19e)
self._drain_pending_user_messages()
music_retry_dispatched = self._apply_music_guard(...)
if music_retry_dispatched:
    self._pending_music_cleanup = False
else:
    self._finalize_music_cleanup_policy(...)
```

**Acceptance (issue #3005 §1):** на ходе «спой/сыграй» без явного music-тула cleanup НЕ публикуется до того, как ретрай-тур успеет довести свой `compose_music` до MCP.

**Тесты (commit `d622fb19e`):** `src/rob_box_voice/test/unit/node/test_issue_3005_music_cleanup_race.py` — 4 AST-сценария, проверяющих порядок вызовов и состояние `_pending_music_cleanup` в обеих ветках. Прогон локально (см. §4 raw-evidence):

```
test_apply_music_guard_runs_before_finalize_cleanup_policy PASSED
test_retry_dispatched_branch_skips_finalize_cleanup_policy PASSED
test_retry_branch_clears_pending_music_cleanup_flag PASSED
test_apply_music_guard_still_disarms_when_no_retry PASSED
```

### 2.2 Механизм B: watchdog segments_deadline

**Что было:** на user-requested треке `compose_music(segments=8)` для трёхминутного эмбиента LLM занижал segments → deadline ~60 с срабатывал раньше, чем юзер успевал насладиться треком (live-лог 14:50:39 idle=45.2s ttl=1800s). DJ-режим уже имел гард (issue #990, `if self.dj_mode_enabled: return`) — там watchdog игнорировался, потому что DJ-сет непрерывен. User-requested трек был НЕ защищён.

**Фикс (уже в branch, commit `dccf9b648`):** симметричный гард. Если `idle < ttl` — сегментный deadline просто **сбрасывается в `None`** (а не убивает трек); следующий `compose_music`/`execute_music_code` поставит новый, если понадобится. Если `idle >= ttl` — старое поведение: deadline приоритетнее idle_ttl, сегменты заполняются в лог.

```python
# music.py:maybe_stop (стало, после dccf9b648)
deadline = self._music_deadline_at
if deadline is not None and now_m >= deadline:
    if self.dj_mode_enabled:
        # issue #990 — DJ-гард
        ...
    if idle < ttl:
        # issue #3005 — user-requested трек живёт по idle_ttl
        self._music_deadline_at = None
        self._music_deadline_segments = None
        return result
    # старая логика: idle >= ttl — segments_deadline приоритетнее idle_ttl
    segments_for_log = self._music_deadline_segments
    stop_result = self.stop_all()
    ...
```

**Acceptance (issue #3005 §2):** на user-requested треке watchdog сегменты **не убивает** музыку, пока юзер в диалоге (idle < idle_ttl).

**Тесты (commit `dccf9b648`):** `src/rob_box_mcp_tools/test/test_tools/test_music.py` — добавлены 4 сценария для сегментного watchdog'а (включая `test_segments_deadline_reset_when_user_in_dialogue` — главный кейс issue #3005). Прогон локально (см. §4 raw-evidence): 32 теста по фильтру `deadline or auto_stop or segments` — все зелёные.

### 2.3 Механизм C: music_cleanup(reason="new_session")

**Что есть в коде сейчас:** на момент написания этого ADR (commit `dccf9b648`) явного `_publish_music_cleanup(reason="new_session")` в `dialogue_node.py` **нет** (поиск: `grep -E '_publish_music_cleanup\(reason="new_session"\)' src/` → 0 совпадений). Однако live-лог на Vision Pi показывает `music_cleanup sent: reason=new_session` — это либо:

- (a) **новый код на vision-pi**, который ещё не в develop — и тогда нужен гард уже сейчас, потому что механизм проявился;
- (b) **артефакт** имён — на стороне mcp_server `_on_music_cleanup` (mcp_server.py:691) дефолтит `reason="dialogue_end"` и просто логирует с переданной reason. Возможно, из какой-то ветки в vision-pi реально публикуется `reason=new_session` через `_reset_dialogue_session` (например, добавленная строка после ретро по #1563).

Чтобы не зависеть от причины, по которой `new_session` появилось в live-логе, и не плодить race на той же ровной земле, гард должен быть **в одном из трёх мест** (расположены по убыванию инвазивности):

1. **На стороне mcp_server (`mcp_server.py:_on_music_cleanup`)** — общий гейт «is this music user-requested-fresh». Самая чистая точка: уже знает про `MusicManager` (idle, active_patterns, _last_music_activity_at). Не требует знания структуры диалога на стороне dialogue_node.
2. **На стороне MusicManager (`MusicManager.stop_music_on_session_end`)** — та же инвариантность, без дополнительного параметра. Гард «if `_last_music_activity_at < (now - ttl)` — stop, else — hold».
3. **На стороне dialogue_node `_publish_music_cleanup(reason="new_session")`** — единственный источник `reason="new_session"` (если он существует в live, его надо явно закоммитить в develop).

**Предлагаемое решение (для отдельной worker-карточки, Phase 2 ADR-0129):**

- Выбрать место **#1**: `mcp_server.py:_on_music_cleanup` получает гард по `was_active + (now - last_user_requested_music_at) < idle_ttl`. Если условие выполнено — **только логирует** `music held (issue #3005)`, **не вызывает** `stop_music_on_session_end()`.
- Источник `last_user_requested_music_at` — `_last_music_activity_at` на `MusicManager` (уже ставится в `execute_music_code` на успешный синтез, `music.py:1392`).
- Механизм A уже снимает `_pending_music_cleanup` в retry-ветке (см. §2.1) — это значит `tts_batch_complete` cleanup тоже отвалится естественным путём без публикации. Механизм C отдельной ветки `reason="new_session"` тоже использует тот же `_publish_music_cleanup`, поэтому A + C согласуются на стороне dialogue_node без новой логики.

**Acceptance (issue #3005 §3):** «новая сессия» **не убивает** user-requested трек, пока он свежий.

### 2.4 Что НЕ меняется этим ADR

- **DJ watchdog гард (issue #990)** — не трогаем. У DJ своя политика (segments_deadline игнорируется целиком пока `dj_mode_enabled=True`), и она согласуется с §2.2 по построению (`if self.dj_mode_enabled: return` стоит **выше** нового `if idle < ttl:` гарда).
- **Idle TTL stop (idle_ttl, #1812)** — не трогаем. Это нормальный путь выключения «забытой» музыки по таймауту. Гард §2.2 касается **только segments_deadline**, не idle_ttl.
- **`_run_turn` finally семантика** — не трогаем. Изменение §2.1 локально: меняется только ветка вызова между guard и finalize, cleanup-функционал не отключается — он сдвинут в `else`-ветку.

---

## 3. Файлы и коммиты

### 3.1 Уже в этой ветке (rebase на develop будет чистым)

| Commit | Файл | Что |
|---|---|---|
| `d622fb19e2a654be0be224da6e5c34938bcab2fc` | `src/rob_box_voice/rob_box_voice/dialogue_node.py` (`_run_turn`) | `_apply_music_guard` ПЕРЕД `_finalize_music_cleanup_policy`; в ветке `if music_retry_dispatched:` — finalize пропускается, `_pending_music_cleanup=False` |
| `d622fb19e2a654be0be224da6e5c34938bcab2fc` | `src/rob_box_voice/test/unit/node/test_issue_3005_music_cleanup_race.py` (new file, 258 строк) | 4 AST-теста структуры (`_run_turn` между guard и finalize, retry-ветка чистит pending-флаг, non-retry ветка всё ещё finalize) |
| `dccf9b648434298429bb66d109237d40815c1806` | `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py` (`maybe_stop`) | `if idle < ttl: сбросить deadline` — симметрично DJ-гарду |
| `dccf9b648434298429bb66d109237d40815c1806` | `src/rob_box_mcp_tools/test/test_tools/test_music.py` (+66 строк) | 4 теста сегментного watchdog'а (user-in-dialogue, idle >= ttl, deadline priority, segments field) |

### 3.2 Не в этой ветке, но требуется отдельной worker-карточкой

- `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py` — гард в `_on_music_cleanup` (см. §2.3 место #1) для `reason="new_session"`-cleanup'а. **Это новая worker-карточка**, потому что:
  - Содержит прямой edpoint к MusicManager, который уже знает про `_last_music_activity_at`.
  - Не имеет test-fixture понятной структуры (нужно mock'нуть MusicManager + подписку).
  - Не блокирует merge §3.1 — два PR можно мерджить независимо.
- Возможная пара: `src/rob_box_voice/rob_box_voice/dialogue_node.py:_publish_music_cleanup(reason="new_session")` — добавить явный гард «if fresh music in current turn: skip publish». **Альтернатива** гарду в mcp_server; выбирается в реализации worker-карточки.

### 3.3 Почему не сделать всё одним PR

C-C-механизм (new_session cleanup) **живёт на другой стороне** ROS-графа: dialogue_node публикует на `/mcp/music_cleanup` (JSON payload), mcp_server подписан и вызывает `MusicManager.stop_music_on_session_end()`. Если гард живёт на стороне dialogue_node — он знает про `_pending_music_cleanup` и `_last_music_activity_at` (если добавим поле в MusicState). Если на стороне mcp_server — он знает про `_last_music_activity_at` напрямую (уже есть в MusicManager). Обе точки валидны, решение принимается в worker-карточке по результатам grep `_last_music_activity_at`.

**Архитектурная рекомендация (этот ADR):** гард **в mcp_server._on_music_cleanup** — потому что:
1. Точка **приёма** `music_cleanup` уже знает про MusicManager (все поля доступны без нового кросс-пакета import).
2. Никаких изменений в dance между dialogue_node ↔ mcp_server не требуется.
3. Тестируется в изоляции: подписка-mock + MusicManager stub.
4. Точка **публикации** (`_publish_music_cleanup`) остаётся **единой**: никакая ветка этого кода не должна знать про гард.

Если future-фича «гард по user_requested flag в payload» понадобится — гард в dialogue_node будет лучше. Но в текущем контракте это over-engineering.

---

## 4. Raw-evidence (ADR-0018, ОБЯЗАТЕЛЬНО)

### 4.1 Тесты механизма A

```bash
$ cd src/rob_box_voice && PYTHONPATH=. python3 -m pytest test/unit/node/test_issue_3005_music_cleanup_race.py -v
collected 4 items
test_apply_music_guard_runs_before_finalize_cleanup_policy PASSED [ 25%]
test_retry_dispatched_branch_skips_finalize_cleanup_policy PASSED [ 50%]
test_retry_branch_clears_pending_music_cleanup_flag PASSED [ 75%]
test_apply_music_guard_still_disarms_when_no_retry PASSED [100%]
============================== 4 passed in 0.61s ===============================
```

### 4.2 Тесты механизма B (фильтр deadline/auto_stop/segments)

```bash
$ cd src/rob_box_mcp_tools && PYTHONPATH=. python3 -m pytest test/test_tools/test_music.py \
    -v -k "deadline or auto_stop or segments" 2>&1 | tail -3
=============== 32 passed, 261 deselected, 31 warnings in 3.56s ================
```

### 4.3 git diff против develop

```bash
$ git log --oneline origin/develop..HEAD
dccf9b648 wip(voice #3005): watchdog segments_deadline ресетится при idle < ttl
d622fb19e wip(voice #3005): music-guard переехал ПЕРЕД cleanup-finalize в _run_turn
212f85f54 report(component-review): 2026-09-22 (docker/main, t_6e86f127)
300ed870c report(component-review): docker/main 2026-09-22 (t_6e86f127, no-real-defect)
```

(Ничего лишнего. 2 новых WIP для #3005 + 2 component-review commits тянутся из develop — см. `git log --no-merges --first-parent origin/develop..HEAD | wc -l` = 4.)

### 4.4 Что НЕ проверено (явное признание)

- **Live-e2e прогон на Vision Pi** — НЕ прогонялся. По `r-card #3005` e2e не запускался (worker кейс). Это **ответственность merge-gate → e2e-process после PR**. ADR-0018 требует честно сказать «не проверено».
- **Полный pytest всех пакетов** — НЕ прогонялся. Проверены только два таргетных файла + детекторы вокруг них (см. §4.1, §4.2). Полный `pytest src/rob_box_voice src/rob_box_mcp_tools` в этой сессии — не успел (бывший запуск с #3004 на предыдущей сессии показал 3321 зелёных → после моих правок ожидается 3325; см. commit message `d622fb19e`).

---

## 5. Worker handoff

Для воркера на следующую фазу **механизма C** (отдельная карточка):

1. `ack робот: почитал ADR-0129 §2.3, разобрался с `mcp_server.py:_on_music_cleanup` (§ 691–715). Реализовать гард для `reason="new_session"` И для **всех** причин с одинаковой логикой — `if (now_m - self._music_manager._last_music_activity_at) < self._music_manager._idle_ttl_s: skip stop_all, log "music held (issue #3005)"`. Гард ОБЯЗАН быть без нового параметра в payload — только то, что MusicManager уже знает.
2. Тесты: добавить в `src/rob_box_mcp_tools/test/test_tools/test_music.py` (или отдельный файл, если `_on_music_cleanup` живёт в mcp_server, а не music_manager) — три кейса: (а) `last_music_activity_at` < `idle_ttl` ago → `stop_music_on_session_end` НЕ вызван; (б) `last_music_activity_at` > `idle_ttl` ago → вызван штатно; (в) `last_music_activity_at is None` → вызван штатно (нет user-requested музыки — старый путь).
3. Если в `dialogue_node.py:7802` (`_reset_dialogue_session`) **не было** `_publish_music_cleanup(reason="new_session")` — отдельно поднять карточку на этот кусок: «publish reason=new_session из reset», потому что иначе ADR-0129 §2.3 фиксит фикстуру без реальной проблемы.
4. Состояние state-машины music_manager (`_last_music_activity_at`) уже обновляется в `execute_music_code` после успешного синтеза (`music.py:1392`) — проверка по grep'у обязательна.
5. `pytest src/rob_box_mcp_tools` — 3325+; `pytest src/rob_box_voice` — стабильно. Без raw-вывода в PR description — не коммитить (ADR-0018).
6. `Closes #3005` в PR description, **e2e-done** через merge-gate → e2e-process.

---

## 6. Retro-сводная

| Что | Когда | Кем | Где |
|---|---|---|---|
| Bug C fix «ретраит успешный compose_music» (commit `149f755f8`) | 2026-09-24 17:18, карточка `t_3b8e9578` | backend-воркер | develop |
| Persona-сет «новый сет говорит голосом предыдущего диджея» (PR #3007) | 2026-09-24 17:19, карточка `t_cb10cab2` | architect | в merge-gate, ожидает Шифу |
| Race «cleanup vs fresh-start» (механизм A) | 2026-09-24 17:23, WIP `d622fb19e` | architect | в этой ветке |
| Race «segments_deadline vs fresh-start» (механизм B) | 2026-09-24 17:31, WIP `dccf9b648` | architect | в этой ветке |
| Race «new_session vs fresh-start» (механизм C) | **ожидает** | worker (новая карточка после merge) | — |

**Слабое место:** отсутствие единого «music freshness flag» — мы платим этим в трёх местах кода тремя разными проверками (механизм A через `_pending_music_cleanup`, механизм B через `idle < ttl`, механизм C ещё не реализован). Если в будущем добавится 4-й механизм (например, wake-word-barge-in тоже захочет глушить музыку), точка согласованности потеряется. **Альтернатива для будущего ADR:** завести `MusicFreshnessPolicy` как единый объект с `is_fresh(now_m) -> bool`, который все три (четыре?) места используют. Не делаем сейчас — KISS, фиксим только конкретные race.
