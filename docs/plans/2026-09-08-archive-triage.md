# Разбор архивных веток/тегов серии operator-agent — 2026-09-08

> **База проверки:** `origin/develop` @ `eac24efa` (2026-09-08).
> **Метод:** для каждого ref — `git merge-base --is-ancestor <ref> origin/develop`
> (бинарный факт), затем для невлитых — `git log origin/develop..<ref> --oneline`
> и построчный diff конкретных файлов против того, что реально лежит в
> `origin/develop` сейчас. Коммит-месседжи и текст PR использовались только как
> наводка, не как доказательство — доказательство везде из diff.
> **Честность (ADR-0018):** там, где не проверил дословно (например, работает
> ли реально фича на роботе) — не утверждаю, только «код присутствует в
> develop».

## Что разобрано

9 тегов + 1 ветка из архива серии `operator-agent` (#1984–#2004), созданного
хендоффом `docs/plans/2026-09-05-operator-agent-architecture-handoff.md`.
Сама ветка `archive/operator-agent-handoff-stale-2026-09-07` несёт коммит
`13afd0905` с явным списком «что через что влилось» — это использовано как
отправная точка, но каждое утверждение в нём перепроверено diff'ом, а не
принято на веру (автор коммита — `agent-flow`, автоматический процесс).

## Таблица: ref → влит? → уникальное → вердикт

| Ref | Тип | `is-ancestor` origin/develop | Уникальные коммиты | Что показал diff | Вердикт |
|---|---|---|---|---|---|
| `archive-zarchitect1992adr0054wakestream` | tag | **exit 0 (ДА)** | — | Буквальный предок, ноль уникального содержимого | **Безопасно удалить — удалено** |
| `archive-1985operatoragent02...` (dead-code #1985) | tag | exit 1 (нет) | 2: `67bb084f` (giant squash-снимок develop на момент ветвления) + `f52786f5` (`rm .github/README.md`) | `.github/README.md` отсутствует и в текущем develop (другим путём); весь дедлкод-контент 67bb084f — снимок develop восьми-с-лишним-дневной давности, полностью перекрыт 1818 более поздними коммитами develop. Целевая работа issue #1985 (удаление `DialogHarness`, `rob_box_voice/llm`, форков music, лишних state-машин) подтверждена смерженной через `2ce3ad4e` (#2006), `b091c61a`, `e587b51b`, `ed21145c` (#2075) — все в develop | Содержимого не теряем, но литерально не предок → **не удалено, см. «оставлено владельцу»** |
| `archive-zagent1992operatoragent05astreamidwakevad` (#1992, шаг 5а) | tag | exit 1 | 4: 3×`wip(av-17)` (msgpack/supervisor_state/HUD) + 1×`docs(adr-0054): ...5а` | `msgpack.ts` и `supervisor_state.ts` **побайтово идентичны** develop (0 строк diff) — это AV-17 база, уже слита. `main.ts`/`connection.ts` в теге — **более старая** версия: develop использует `WebSocket.OPEN`, тег — числовой литерал `1`; develop уже содержит финальный `sendVoiceAudio(payload, streamId: 1\|2)` с комментарием ADR-0054 шаг 5а (подтверждено grep в `origin/develop:.../wire/connection.ts:237-245`) — тег этого ещё не имел. Но **ADR-документ** `docs/adr/0054-operator-agent-step-5a-wake-stream.md` (526 строк, статус Accepted) и impl-план (176 строк) **в develop отсутствуют под любым именем** — слот `ADR-0054` в develop занят другим шагом (`0054-operator-agent-step-7b-eventbus-bridge.md`, влит через #2048) | Код полностью перекрыт и превзойдён через #2052; **ADR-документ — находка, см. ниже** |
| `archive-zagent1997operatoragent07breflexlayercommandn` (#1997, ReflexLayer) | tag | exit 1 | 2: `command_node.py` (241 строка) + `ReflexLayer.attach()` bridge, 3 тест-файла | Ровно тот же набор файлов, что и в смерженном PR #2048 (`gh pr view 2048 --json files` — точное совпадение путей); `command_node.py` в обеих версиях — 651 строка (после "clean rebase" в #2048) | Полностью перекрыто #2048. Подтверждено аудитом в issue #1997 (комментарий GOODWORKRINKZ): код на месте, `enable_reflex_layer` включён в проде через #2078 |
| `archive-zagent2002operatoragent12questseambridgeexec` (#2002, Quest seam) | tag | exit 1 | 4 уникальных (+3 общих av-17): IDL `Command.msg`/`Response.msg`/`ExecuteCommand.srv`, `Bridge.execute()` facade Phase 1, доки ADR-0051 + target-arch §13 шаг 12 | `rob_box_supervisor_msgs/msg/Command.msg` и `ExecuteCommand.srv` **уже в develop** — но введены не этим тегом, а PR #2056 (`git log` подтверждает: `9f7ccbe4 ... Phase 1 ... (#2056)`); Phase 2 (19 методов) — PR #2086 | Работа Phase 1 из тега вытеснена #2056, Phase 2 сделана отдельно в #2086. Issue #2002 всё ещё open, но только из-за отсутствия e2e PASS-доказательства (комментарии agent-flow: «PR #2086 смержен, но PASS-доказательства не найдено»), не из-за недостающего кода |
| `archive-zagent2003operatoragent13pregeneratettsnode` (#2003, pregenerate) | tag | exit 1 | 1: `docs/architecture/adr/pregenerate-in-tts-node-contract.md` (239 строк, proposal v0.1, заблокирован issue #1996) | develop содержит **полную реализацию**: `scheduler/pregen/{decision,estimator,pre_gen,quality,speculative_executor}.py`, `docs/adr/0056-speculative-tts-pregeneration-contract.md`, DoD-тесты через #2068 и бенчмарк через #2074 (`chunk_latency_bench.py`) — тег содержит только черновик-контракт, ранняя стадия того же дизайна | Черновик полностью превзойдён принятым ADR-0056 + реализацией |
| `archive-zbackend1992implwakestream` (#1992, клиентская часть) | tag | exit 1 | 3: RMS VAD-гейт (`vad_gate.ts`, отдельный модуль), `voice_capture.ts` изменения, `main.ts`, `protocol/frame.py` с именованными stream_id (`radio`/`radio_alias`/`wake`) | Функциональность (VAD-гейт с hangover 200мс, `voice_listen_start/stop`) присутствует в `origin/develop:.../voice_capture.ts` (grep подтверждает `VAD_RMS_THRESHOLD_DEFAULT`, `wakeGate`, `hangoverMs` — та же логика), но **встроена прямо в `voice_capture.ts`**, а не вынесена в отдельный `vad_gate.ts` (такого файла в develop нет). Именованные stream_id-константы не прижились — в `ws_server.py` роутинг идёт по числовому литералу `sid == 2` (текст PR #2052: «`_ws_handler`: ветка `FrameType.VOICE_AUDIO` роутит по `sid == 2`»). Сам PR #2052 прямым текстом признаёт: «Шаги 1-3 — клиент, уже были в ветке от предыдущей итерации (3 wip от архитектора)» | Функционально перекрыто через #2052; расхождение только стилистическое (структура файлов, именованные константы vs литералы) — не потеря функциональности |
| `archive-zbackend2000phase2waypointadapter` (#2000, одна БД/waypoint) | tag | exit 1 | 1: `waypoint_adapter.py` (527 строк) + `test_waypoint_adapter.py` (384 строки) | Тот же набор файлов при точном совпадении путей с PR #2094 (`gh pr view 2094 --json files`); `waypoint_adapter.py` в develop — тоже 527 строк | Полностью перекрыто #2094. Issue #2000 остаётся open по той же причине, что #1997/#2002 — не найдено e2e PASS-доказательство, не потому что кода не хватает |
| `backup-t_6d1b87d8` | tag | exit 1 | 1: `wip(voice #1976): TTSProviderChain module + Yandex-first default + sync 4 surfaces` — `tts_chain.py` (416 строк), `docs/adr/0044-tts-provider-chain-sync-contract.md` (196 строк), правки `tts_node.py` и конфигов | **Не относится к серии operator-agent.** Issue #1976 закрыт автором репозитория с комментарием: «Неправильно понял задачу — fallback должен быть в e2e test script, не в dialogue_node. См. issue #1977.» Issue #1977 (`feat(e2e): synth fallback chain в e2e_voice_test.sh`) до сих пор **open**, но требует другой архитектурной точки (уровень e2e-скрипта, не библиотечный модуль в `tts_node`) | Код — отвергнутый как неверный подход артефакт. Не годится «как есть» под #1977 (другое место в архитектуре). **Не удалено — оставлено владельцу** |
| `archive/operator-agent-handoff-stale-2026-09-07` | branch | exit 1 | 1: `13afd0905` — **нулевой diff** (`git diff 13afd0905^ 13afd0905 --stat` пусто) | Коммит только текстовый (список: что через какой PR влилось), файлов не меняет. Родитель `e68a282f` — **является** предком develop (`is-ancestor` = exit 0 для родителя). Т.е. по содержимому ветка = точка в истории develop + один pure-message коммит | Формально не предок (сам коммит), контента ноль → **не удалено по строгому правилу DoD, но по факту безопасно для владельца** |

## Находки, которые стоит поднять в работу

### 1. ADR-0054 «шаг 5а — wake stream» не существует в develop ни под каким номером

Код фичи (always-on микрофон шлема + RMS VAD-гейт + `voice_listen_start/stop`)
полностью реализован и слит в develop через PR #2052/#2094 — это подтверждено
grep'ом `origin/develop:src/rob_box_quest/webxr_client/src/input/voice_capture.ts`
(константы `VAD_RMS_THRESHOLD_DEFAULT`, `wakeGate`, комментарии «ADR-0054 §2.1»
прямо в коде) и `wire/connection.ts` (`sendVoiceAudio(payload, streamId: 1 | 2 = 1)`
с докстрингом про ADR-0054 шаг 5а). Но **сам ADR-документ**, описывающий это
решение (526 строк, статус «Accepted», автор — architect-профиль, kanban
`t_97a82c5a`), существует только в теге `archive-zagent1992operatoragent05astreamidwakevad`
по пути `docs/adr/0054-operator-agent-step-5a-wake-stream.md` — и никогда не
попадал в develop. Номер `ADR-0054` в develop занят другим документом
(`0054-operator-agent-step-7b-eventbus-bridge.md`, слит через #2048) —
классический namespace-collision (вероятно, ровно то, для чего позже завели
`ADR-0057-adr-namespace-collision-guard-in-ci.md`).

Автор PR #2052 сам признаёт коллизию прямым текстом в теле PR: «ADR-0054 (в
ветке архитектора, не в develop — пересечение)».

**Рекомендация:** восстановить `docs/adr/0054-operator-agent-step-5a-wake-stream.md`
и impl-план (`docs/adr/0054-operator-agent-step-5a-impl-plan.md`, 176 строк)
из тега, перенумеровать под свободный ADR-номер (уже реализованное решение
имеет право быть задокументированным), сослаться на issue #1992 (closed) как
на факт реализации. Это документационный долг, не блокер — но текущий ADR-0054
в develop вводит в заблуждение (закрывает шаг 7б, а не 5а, хотя название
ветки/тега подразумевало 5а).

### 2. Открытые карточки #1995/#1999/#2000/#2002/#2003/#2004 — работы «на потерю» в архиве нет

Проверил специально: ни `backup-t_6d1b87d8`, ни ветка
`archive/operator-agent-handoff-stale-2026-09-07` не содержат непримененного
кода по этим карточкам.

- `backup-t_6d1b87d8` — единственный коммит относится к **другой, не
  operator-agent задаче** (#1976, TTS provider chain), и эта задача уже
  закрыта как «неверно понятая» с явным редиректом на #1977 (другая точка
  архитектуры — e2e-скрипт, а не `tts_node`).
- `archive/operator-agent-handoff-stale-2026-09-07` — пустой по содержимому
  коммит (0 файлов), только текст.
- #2000, #2002, #2003 — код по факту **уже в develop** (см. таблицу выше);
  issue'ы остаются open только из-за отсутствия e2e PASS-доказательства
  (автоматические комментарии `agent-flow`), не из-за отсутствующего кода.
- #1995 (EventBus + отмена, `task_scheduler.py:923`) — по аудиту в issue #1997
  (комментарий GOODWORKRINKZ, 2026-09-07): «шаг 07 (#1995, EventBus + отмена)
  **не начат вообще** — ни ветки, ни PR». Ни один из разобранных 10 refs не
  затрагивает #1995 — подтверждаю: работы по ней действительно нигде нет,
  включая архив.
- #1999 (один владелец floor) — ни в одном из 10 refs файлы `ModeManager`/floor
  arbitration не встречаются вообще.

Вывод: архив не содержит скрытого прогресса по открытым карточкам. Единственная
находка — ADR-документ п.1 выше.

## Удалено

| Ref | Тип | Доказательство |
|---|---|---|
| `archive-zarchitect1992adr0054wakestream` | tag | `git merge-base --is-ancestor archive-zarchitect1992adr0054wakestream origin/develop` → **exit 0**. SHA `2e315b3d4283d7e0fd64a03a2af58e23730d9770` совпадает с `git ls-remote` до удаления. Удалено `git push origin --delete refs/tags/archive-zarchitect1992adr0054wakestream`, подтверждено повторным `git ls-remote` — тег отсутствует. |

Только этот ref прошёл строгую проверку `is-ancestor`. Название тега
(«adr-0054-wake-stream») вводит в заблуждение — по факту последний коммит
ветки (`2e315b3d`, «test(voice): fix bounded_fanout flake») не имеет отношения
к ADR-0054; ветка агента-архитектора просто указывала на точку в истории
develop без уникальных коммитов сверху.

## Оставлено владельцу (не влито литерально, решение — за владельцем)

| Ref | Почему не удалено | Рекомендация |
|---|---|---|
| `archive-1985operatoragent02...` | 2 уникальных коммита, оба содержательно перекрыты (см. таблицу) | Можно удалить — контента не теряем, но `is-ancestor` формально false из-за squash-истории |
| `archive-zagent1992operatoragent05astreamidwakevad` | Код превзойдён, но ADR-документ нигде больше не существует | **Не удалять**, пока ADR-0054-step-5a-wake-stream.md не извлечён и не влит под новым номером (см. находку №1) |
| `archive-zagent1997operatoragent07breflexlayercommandn` | Полностью перекрыто #2048 | Можно удалить |
| `archive-zagent2002operatoragent12questseambridgeexec` | Phase 1 перекрыта #2056, Phase 2 — #2086 | Можно удалить |
| `archive-zagent2003operatoragent13pregeneratettsnode` | Черновик-контракт превзойдён ADR-0056 + реализацией | Можно удалить |
| `archive-zbackend1992implwakestream` | Функционально перекрыто #2052 (признано автором PR), расхождение только стилистическое | Можно удалить |
| `archive-zbackend2000phase2waypointadapter` | Полностью перекрыто #2094 | Можно удалить |
| `backup-t_6d1b87d8` | Код по отклонённому подходу к закрытой (как «неверно понятой») задаче #1976; не относится к серии operator-agent | Низкий приоритет — оставить как есть или удалить по усмотрению владельца, переиспользовать «как есть» под #1977 нельзя (другое архитектурное место) |
| `archive/operator-agent-handoff-stale-2026-09-07` (branch) | Коммит-тип «сообщение», 0 файлов, но литерально не предок develop | Можно удалить — вся информация из его коммит-месседжа перенесена в этот документ |

Итого: **9 из 10** refs содержательно безопасны к удалению (контент либо
буквально в develop, либо содержательно перекрыт более новыми PR), но только
**1 из 10** прошёл строгий бинарный тест `is-ancestor`, поэтому по правилам
задания удалён только он. Оставшиеся 8 переданы на решение владельца списком
выше — включая явную рекомендацию **не удалять** тег с ADR-0054-step-5a, пока
документ не спасён.
