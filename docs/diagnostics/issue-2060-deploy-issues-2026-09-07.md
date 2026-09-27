# Issue #2060 — Deploy issues on develop (staging) 2026-09-07: root-cause + premise obsolete

**Reporter:** devops worker (t_daafaf23), 2026-09-15
**Issue:** [#2060](https://github.com/krikz/rob_box_project/issues/2060)
**Severity:** LOW (premise obsolete — fix already merged)
**Related:** PR #2108 (fix), PR #2105 (regression), issue #2099 (root cause), PR #2107 (ci build single service for quest)

## TL;DR

Карточка #2060 (Deploy issues на develop/staging от 2026-09-07) — **premise obsolete**. Root cause
(жёсткий импорт `rob_box_voice.tts_voice_registry` в `quest_node.py`) был устранён PR #2108
(commit `7022e89a3`, merged 2026-09-07 22:01:12Z), **до того как был сделан последний failing
deploy run #34153246013 в 18:50:26Z**. Все deploy'ы develop после фикса (включая #1370 от
2026-09-11 14:53Z) — success, `rob-box-quest` контейнер `running healthy restarts=0`.

Никаких новых PR/code-изменений для закрытия #2060 не требуется — закрываем по «premise obsolete,
fix уже merged как #2108».

## Raw-evidence (agent-flow contract)

### Issue #2060 timeline (по issue comments API)

```
$ gh api /repos/krikz/rob_box_project/issues/2060/comments
… 5 комментариев от github-actions[bot], все шаблонные "Ещё один failing deploy run":
  2026-09-07T14:07:18Z  → runs/34130705072
  2026-09-07T14:33:21Z  → runs/34133181445
  2026-09-07T14:53:03Z  → runs/34134981108
  2026-09-07T16:11:10Z  → runs/34141648182
  2026-09-07T18:56:54Z  → runs/34153246013
$ gh api /repos/krikz/rob_box_project/actions/runs/34153246013  → conclusion=success
  НО: внутри step #30 "🚨 Create Deployment Issue (any issues)" сработал и запостил
  комментарий в issue #2060, потому что rob-box-quest был "restarting" (логи показали
  ModuleNotFoundError в quest_node.py).
```

Workflow conclusion=success не означает «деплой чистый» — внутренний deployment-summary
пишет «Deployment Completed With Issues» и триггерит issue, если контейнеры unhealthy
или в логах critical errors (raw-evidence см. ниже).

### Root cause (raw-evidence из лога failing run #34153246013)

```
$ gh run view 34153246013 --log | grep -E "rob-box-quest" -A2
  rob-box-quest: restarting                                                   (step 22)
  === rob-box-quest CRITICAL ERRORS ===
  [ros2run]: Process exited with failure 1
  File "/ws/install/rob_box_quest/lib/python3.10/site-packages/rob_box_quest/quest_node.py",
       line 67, in <module>
    from rob_box_voice.tts_voice_registry import voices_for as _voices_for
  ModuleNotFoundError: No module named 'rob_box_voice.tts_voice_registry'
  [ros2run]: Process exited with failure 1
```

**Причина**: `Dockerfile` образа `rob-box-quest` (`docker/vision/quest/Dockerfile`) собирает
только `rob_box_supervisor_msgs + rob_box_core + rob_box_quest` (а не `rob_box_voice`).
Жёсткий импорт `from rob_box_voice.tts_voice_registry import voices_for` в `quest_node.py:67`
бьётся ModuleNotFoundError → restart loop.

Это регрессия PR #2105 (`z-{agent}/2100-operator-tts-return-channel`), который добавил
жёсткий импорт `tts_voice_registry` в `quest_node.py`. Импорт был нужен только для
валидации выбора голоса в UI Quest, но пакет `rob_box_voice` в образ не входит (конвенция
«пакет должен оставаться импортируемым без rob_box_voice», см. `mcp_tools/tools/dialogue.py:28`,
`mcp_tools/voice_state.py:33`).

### Fix (PR #2108, уже в develop)

```
$ git show --stat 7022e89a3
fix(quest #2099): защищённый импорт tts_voice_registry — жёсткий клал ноду в Restarting loop (#2108)

 src/rob_box_quest/rob_box_quest/quest_node.py    | 17 +++++++++-
 src/rob_box_quest/test/unit/test_quest_bridge.py | 38 ++++++++++++++++++++++++
 2 files changed, 54 insertions(+), 1 deletion(-)
```

Замена жёсткого импорта на защищённый:

```python
try:
    from rob_box_voice.tts_voice_registry import voices_for as _voices_for
except ImportError:  # pragma: no cover — образы без rob_box_voice
    def _voices_for(provider: str) -> list:
        """Fallback: реестр голосов недоступен в этом образе."""
        return []
```

+ AST-тест `test_voices_for_import_is_guarded` (ловит причину, а не симптом).

### Merge-base verification

```
$ git merge-base --is-ancestor 7022e89a3 <sha> && echo IS_ANCESTOR
7022e89a3 IS ancestor of 9db4699a   # run #1370 success
7022e89a3 IS ancestor of 4ab3a0a5   # run #1371 failure (SSH tools, не quest!)
7022e89a3 IS ancestor of 0cbe40d1   # origin/develop HEAD (2026-09-14)
7022e89a3 IS ancestor of 0749ab9b   # origin/develop HEAD (2026-09-15, latest)
```

### Post-fix deploy evidence

```
$ gh run view 34612785145 --log | grep -E "rob-box-quest:"
  rob-box-quest: status=running restarting=false health=healthy restarts=0   (sample #1)
  rob-box-quest: status=running restarting=false health=healthy restarts=0   (sample #2)
  ✅ All Vision Pi containers running and confirmed stable
```

Run #1370 от 2026-09-11 14:53Z на head=9db4699a (после PR #2108) — все контейнеры Vision Pi
включая `rob-box-quest` healthy. То же для runs #1358–#1369, #1371 (последний — failure по
SSH tools, не quest).

### Серийный паттерн (наблюдение, не блокер для #2060)

Issue #2060 — **самый старый** открытый «deploy issues» из серии. Более свежие аналоги:
- #2256 (2026-09-09) — avatar-supervisor restart-loop `ModuleNotFoundError: rob_box_core.utterance`
- #2343 (2026-09-10) — voice-assistant warnings (не restart-loop, но тот же шаблон бота)
- #2383 (2026-09-14) — vision-hailo restart-loop (см. issue #2527 — другое задание в работе)

Это серийный паттерн: «merge в develop → в течение дней регрессия → бот спамит issue». Для
**этой** карточки это не блокер — фикс уже merged в PR #2108, и последующие deploy'ы чистые.
Серия как процесс-улучшение — отдельный вопрос (стоит ли мержить процессный fix типа
авто-rebase registry после merge, чтобы не ждать ручного re-run образа — это за рамками
этой карточки, но отмечено для архитектора).

## Action

Закрыть #2060 с обоснованием «premise obsolete, fix already merged as PR #2108 (commit
7022e89a3) to origin/develop. Verified via merge-base: 7022e89a3 IS ancestor of all
post-2026-09-10 deploy SHAs and current origin/develop HEAD (0749ab9b). Verified via
run #1370 raw logs: rob-box-quest running healthy restarts=0, all Vision Pi containers
stable. No further code change required for this issue.»

## Файлы для фикса

Никаких файлов менять не нужно. Это диагностический документ для traceability.
