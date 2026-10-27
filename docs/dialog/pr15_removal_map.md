# Карта удаления старого пути диалога (v1) для PR-15 (#3711) — пересверка на `d076b8b55` (27.10.2026)

Карточка PR-15 эпика #3683: [#3711](https://github.com/krikz/rob_box_project/issues/3711), хвосты из
[#3704](https://github.com/krikz/rob_box_project/issues/3704) (PR-10b, `DialogueStateMachine`),
[#3705](https://github.com/krikz/rob_box_project/issues/3705) (PR-11), [#3706](https://github.com/krikz/rob_box_project/issues/3706)
(PR-12a). План ADR: [`docs/adr/0150-dialog-v2-session-command-event.md`](../adr/0150-dialog-v2-session-command-event.md) §11 PR-15,
§12, §14 A4/A6/A8, §16. Парный документ — [`acceptance_2026-10-23.md`](acceptance_2026-10-23.md) (приёмка PR-14).

**База:** `origin/develop @ d076b8b55` (проверено перед работой: `git rev-parse origin/develop` →
`d076b8b55972edc430eb49a98e9702903b1624d3`). Прошлая карта снята на `8178e2809` (23.10, влита PR #3910).
**Одна метрика размера — `wc -l`** (физические строки с пустыми и комментариями; скрипт считает `\n` в blob, это то же,
что `git show <ref>:<path> | wc -l`, сверка — приложение А.0). Размер метода — диапазон строк по AST (с декораторами).
Код не менялся, робот не использовался. Оценки без измерения помечены словом «оценка».

**Как пересобрать факты** (оба скрипта лежат рядом с картой, их вывод целиком — в приложении):

```bash
python docs/dialog/pr15_map_facts.py --ref origin/develop --old 8178e2809   # AST-граф, множество, размеры, части
bash docs/dialog/pr15_map_greps.sh origin/develop 8178e2809                  # grep-критерии карточек, топики, флаг
```

## 0. Что исправлено в этой редакции (ревью PR #4080, блокеры B1–B6)

| # | Было в редакции `23cf3bb69` | Стало |
|---|---|---|
| B1 | 15d удаляла `core/greeting_window.py` | **Не удаляется**: его импортирует живой v2 (`dialog_node.py:79`, `core/dialog_v2_parity.py:22`). Читателей 5, из них 2 в v2 (§3) |
| B2 | 15c удаляла `core/turn_context.py` целиком | Удаляются только `context_sections` и три помощника (строки 37–76, ≈40). `tts_state` остаётся на месте: его читает v2 (`core/plan_host.py:32`). Решение «куда» — §3, строка `turn_context` |
| B3 | размеры «27.10» посчитаны `Measure-Object -Line` (без пустых строк), Δ фиктивные | Всё пересчитано `wc -l`. Пометка «изм.» только там, где blob реально отличается (приложение А, раздел «Размеры»): изменились `meeting` 213→53, `startup_greeting` 41→14, `tars_status_feed` 115→41, `tier1_host` 230→236, `turn_context` 68→79, `turn_tools` (111→111, содержимое), а `dialogue_node.py`, `stt_admission.py`, `speak_helpers.py`, `dialogue_guards.py`, `dialogue_helpers.py`, `speech_compat.py`, `conversation_history.py` и DSM не изменились побайтно |
| B4 | граф, множество и читатели — от 23.10; D1–D8 не внесены; порядок частей старый | Граф перестроен скриптом `pr15_map_facts.py` (приложен); множество — **20 файлов / 7 687 строк**; читатели String-топиков перегенерированы с учётом #3917; 15-0e и 15-0f отмечены сделанными; D1–D8 внесены как принятые (§7); новый порядок частей с зависимостями — §6 |
| B5 | хвосты §8 «почти сняты» | Пересчитаны командами из карточек: #3704 10 = 10, #3706 51 = 51, #3705 — маркеры v1 на месте (14 строк); «Bug [A-F]» 26 = 26 (команда карточки сломана, §8). Владельцы #3704 — `dialogue_node`, `supervisor_node`, DSM |
| B6 | зависимости вне `src` не учтены | Граф теперь включает `scripts/`, `tools/`, `.github/`: гард `scripts/lint/dialogue_skip_reasons.py` (`G-Lint Code.yml:72`), REPL `scripts/dialogue/chat.py`, `e2e_voice_test.sh:676,681`, строковое имя `dialogue_node` в v2 (`mcp_auth.py:85`, `slice_policy.yaml:26`) вписаны в части 15p, 15m3a, 15r0, 15s; `llm_skip_reasons.py` — сирота после гарда (15m3a) |

Новое по пересверке (не было в ревью): (1) порядок «листья → корень» из карты 23.10 ломает импорт v1 уже на 15a и 15c
(§6.1); (2) откат флагом на v1 перестаёт работать с первой части, режущей `dialogue_node.py`, а не с 15s (§6.1, Q1);
(3) в `rob_box_harness` живёт второй цикл хода v1 — класс `AgentCore` (1 569 строк) и 5 модулей-сирот (1 080 строк) — в
карте 23.10 его не было (Q4); (4) критерий «`Bug \[A-F\]`» карточки всегда даёт 0 (Q5); (5) v1-топики идентичности
`/voice/speaker/{merge,observe,epithet}` при v2 мёртвые (Q6); (6) критерий `[TG:` = 0 из #3706 противоречит логу v2
(`address.py:194` пишет `[TG:<id>]` в строку `🎤 STT`, §8).

### 0а. Замечания повторной проверки N1–N10 (approve PR #4080, 27.10; внесены первым коммитом этапа 1/2)

Источник — комментарий проверки в PR #4080. Графа «перепроверено» — что воркер PR-15 (15a) сверил на `5a1c8d7dd`
командами `grep`/`sed`; остальное принято с проверки, не пересчитано.

| # | Замечание | Что поправлено в карте | Часть | Перепроверено |
|---|---|---|---|---|
| N1 | 15p удаляет блок `nodes.dialogue_node` в `ownership.yml`, а `core/plan_host.py` (`:43`, издатель `/dialog/plan_event`) нужен v2: он записан только в `nodes.dialogue_node.source` (`:38`), в `nodes.dialog_node.source` (`:79`) его нет → красный `ownership_check` | в 15p: перенести `plan_host.py` в `nodes.dialog_node.source` | 15p | да (`ownership.yml:37–38`, `:76–80`) |
| N2 | И `voice_assistant.launch.py:247`, и `voice_assistant_headless.launch.py:188` передают объект `dialogue_node` в `OpaqueFunction(dialog_engine.launch_nodes, args=[dialogue_node, …])`; 15p убирает `Node(dialogue_node)`, а `launch_nodes` переезжает только в 15s → `NameError` при сборке launch, не поднимется весь voice-assistant | 15p правит вызов и сигнатуру `launch_nodes` сама (или 15p и 15s сливаются); `dialog_engine: v1` после 15p — ошибка, а не запуск несуществующего исполняемого файла | 15p, 15s | да для `voice_assistant.launch.py:247` |
| N3 | `AgentCore` — читатель DSM (`agent_core.py:495, 616–683, 841–879, 1042`), 15r1 в карте — «+20–40 строк» | порядок: при Q4 «да» сначала 15r3 (`AgentCore`), потом 15r1 (DSM); при Q4 «нет» 15r1 = правка логики хода в ≈12 местах и 87 упоминаний DSM в `test_agent_core.py`, а не 20–40 строк | 15r1, 15r3 | частично (23 строки `dsm`/`DialogueStateKind` в `agent_core.py`) |
| N4 | 15a3 не называет правку `core/__init__.py:8` и `:11` (реэкспорт `SpeechFormatter`, `load_faq_items`) → `ImportError` у всего пакета `rob_box_voice.core` | в строке 15a3 правка `__init__.py:8, :11` указана; вместе с 15a1 (`:10`) в `__init__` остаются `CommandParser`, `FAQStore` | 15a1, 15a3 | да |
| N5 | «Вне тестов» в `pr15_map_greps.sh` применён к строке `путь:строка:текст`, а не к пути: `/voice/tts/request` 64/23 → 60/22, `/dialogue/control` 49/12 → 48/11, `/voice/tts/control` 7/4 → 8/5, `dialog_engine` 75 → 77 строк (27 файлов); остальные счётчики §4 совпали. `docker/vision/test/scenario_runner/runner.py` — читатель для 15-0a′, но в счёт «23/13» не входит | счётчики §4 читать с этой поправкой; скрипт `pr15_map_greps.sh` исправляется отдельно (фильтр по пути) | §4, 15-0a′ | нет |
| N6 | Писатели/читатели JSON `/voice/tts/request`, которые замолчат после 15q: `scripts/maintenance/scene_set/conductor.py:231` (издатель), `tools/gui/robot_control_gui.py:39` и `robot_control_gui_simple.py:52`, `tools/audio_capture_harness/audio_capture_harness.py:549, :828`, `scripts/diagnostics/watchdog-tts.sh:63–102` (считает издателей через `ros2 topic info`, при нуле пишет «не вмешиваюсь» и тихо перестаёт чинить зависший TTS) | к 15-0d добавлены эти пять мест (перевод на `speech_client.say`; watchdog — на счёт по `SpeechRequest`-топику); без них 15q оставит тихие поломки | 15-0d (расширить) → 15q | да (строки) |
| N7 | Тесты, которых AST не видит: `test/unit/utils/test_github_summary.py` (436) грузит `github_summary.py` по пути через `importlib` → 15a1 без его удаления красит CI; `test/ros_stubs.py:324` (`make_utterance`) лениво импортирует `parse_tg_prefix` из `core/stt_admission`, 12 тестов `unit/node/*` зовут хелпер → переписать в 15m3b; `rob_box_core/test/test_speaker_names.py:56` грузит `core/dialogue_helpers.py` по пути и проверяет алиас `INVALID_SPEAKER_NAMES`, а 15m5 оставляет там только `sanitize_speaker_name` | в строке `github_summary` «тестов 0» → 1 (436); `ros_stubs.make_utterance` — в 15m3b, `test_speaker_names` — в 15m5 | 15a1, 15m3b, 15m5 | да (все три места) |
| N8 | Внешние упоминания `dialogue_node` вне B6, живой v2 не ломают: `.github/workflows/L-E2E Voice Test.yml:716–744`, `.github/workflows/scripts/e2e_tool_match.py` (22), `scripts/voice_bench/run_bench.py:80`, `scripts/music/live_dj/an_set.py:37`, `night_dj.sh:14`, `scripts/dialog/logparse.py:7`, `bridge_protocol.py:626/913`, `protocol_generated.ts:159/316` (литерал service `dialogue_node` в `admin_logs`) | добавить в 15p как «правка текстов/логпарсеров», часть уже не работает при v2 по умолчанию | 15p | нет |
| N9 | D2 («`re.compile` = 0 в слое диалога: `rob_box_dialog`, `dialog_node`») — в «состоянии» только `speak_helpers` 18 и `dialogue_guards` 2; в `rob_box_dialog` ещё 12 строк в 7 файлах: `address` 2, `grammar` 1, `greeting_reply` 1, `reply_grammar` 1, `words` 3, `plan/rap_pauses` 3, `plan/segments` 1 | ни одна часть их не берёт → предложение: в 15v; если D2 про нормализацию перед синтезом, а не про весь слой — текст D2 надо сузить (решает координатор) | 15v (предложение) | да (12 строк, 7 файлов) |
| N10 | 15m3a: хвост `stt_admission` по AST начинается раньше строки 545 (комментарий «Default steps» `:538`, `EmptyTextStep` `:544–553`), 296 строк, а не 297–303 | размер 15m3a ≈ 343 принимается как оценка; границу резать по AST | 15m3a | да (`:538`, `:544`) |

## 1. Что изменилось в коде между картами (23.10 → 27.10)

1. Коммитов `8178e2809..d076b8b55` — 208, из них 48 трогают `rob_box_voice/rob_box_voice`, `rob_box_dialog/rob_box_dialog`
   и `rob_box_harness/core` (приложение Б, «коммиты»).
2. **PR #3916** (`dab053510`, issue #3912 открыта): паритет D1 — логика встречи лицом, стартовой фразы и фида ТАРС перенесена в
   `rob_box_dialog.meeting`, `startup`, `status_feed`; v2 вызывает её из `dialog_node.py:302–304`
   (`parity.start_v2_status_feeds`, `subscribe_meeting`, `arm_startup_greeting`). Модули v1 стали обёртками:
   `core/meeting.py` 213→53, `startup_greeting.py` 41→14, `core/tars_status_feed.py` 115→41. Тесты — юнит
   (`test_dialog_v2_parity.py`); **на роботе не проверено** (ни в PR #3916, ни позже).
3. **PR #3917** (`fd1e4fa55`, #3911): читатели состояния голоса на контрактах v2. `command_node` и
   `quest/streams/voice_state.py` старый топик `/voice/dialogue/state` больше не читают; таблица состояний — одна,
   `rob_box_core/voice_state.py` (`VoiceStateTracker`, адаптер v1 `on_legacy` + `LEGACY_STATE`); `led_node` получил
   `SessionState`; `context_aggregator_node` читает `SpeechRequest` (`perception/core/robot_speech.py`), старая подписка
   на `/voice/dialogue/response` оставлена до PR-15 (комментарий «Удалит PR-15»). Адаптеры v1 в quest, arbiter, led,
   context_aggregator уходят вместе с `dialogue_node` (часть 15p2).
4. **PR #3962** (`58187f868`, #3708 PR-13b): тулы ТАРС; путь v1 `AgentCore` в `supervisor_node.py` на месте.
5. Изменились файлы, которые v2 стал использовать: `core/turn_context.py` 68→79 (`tts_state` читает `core/plan_host.py:32`),
   `core/tier1_host.py` 230→236, `core/plan_host.py` 161→193, `dialog_node.py` 444→531, `core/dialog_v2_ports.py` 225→298,
   новый `core/dialog_v2_parity.py` (171).
6. Множество v1 сократилось с 22 файлов / 8 089 строк до **20 / 7 687**. Из них −255 строк — те же 20 файлов (−261 перенос
   #3916 в `rob_box_dialog`, +6 `tier1_host`), остальное (−147) — `greeting_window` и `turn_context` **перестали быть
   удаляемыми**, потому что их стал читать v2. Это не сделанная работа.
7. Не сдвинулись: `Bug [A-F]` 26 (8 вне тестов), `RULE #` 8 + 7, хвост #3704 10, хвост #3706 51, маркеры #3705 14 строк.
   Выросли: `#NNNN` в `rob_box_dialog` 158 → 369 строк (401 вхождение), флаг `dialog_engine` 52 → 75 строк (19 → 27 файлов,
   одна и та же команда, приложение Б).

## 2. Как собрана карта (и её пределы)

- **Граф** — AST всех отслеживаемых `*.py` репозитория на ссылке (`git ls-tree` + `git cat-file`, рабочая копия не читается):
  1 798 файлов (в `src` 1 375), 4 429 рёбер «файл → файл» (из `src` 4 033). Той же методикой на `8178e2809`: 1 712 (1 333),
  4 059 (3 721). Число «3 962 ребра» из карты 23.10 не воспроизводится: тот скрипт не сохранён; сравнивать только числа
  одного скрипта.
- **Ребро**: `import a.b`, `from a import b` (относительные и ленивые внутри функций тоже); `from pkg import Name`, где
  `pkg/__init__.py` реэкспортирует `Name`, даёт ребро к подмодулю. Импорт под `TYPE_CHECKING` считается.
- **Читатель** модуля — не-тестовый файл с ребром к нему, **в том числе вне `src`** (`scripts/`, `tools/`, `.github/`);
  `__init__.py`-реэкспорт читателем не считается, но печатается (его правят при удалении). Тест — путь с каталогом
  `test`/`tests`, `test_*.py`, `conftest.py`.
- **Замкнутое v1-множество** — неподвижная точка от `dialogue_node.py`: добавляется не-тестовый модуль `src`, у которого
  есть читатель и **все** читатели уже в множестве.
- **Чего граф не видит** (ищется grep'ом, `pr15_map_greps.sh`): точки входа `setup.py`, launch-файлы, строковые имена нод и
  топиков, пути к файлам в скриптах и workflow (`dialogue_skip_reasons.py` читает `dialogue_node.py` по пути), `self.<атрибут>`
  из `__init__` и вызовы методов друг друга внутри `DialogueNode` (раздел «Что не проверено»).

## 3. Файлы и модули v1

«Читатели» — AST, не-тест (вне `src` — отдельно). «Тесты» — файлов, импортирующих модуль, и их строк. «Часть» — по
стратегии R из §6 (через «/» — часть по стратегии L, если отличается).

| Файл (`rob_box_voice/rob_box_voice/…`) | Строк | Читатели | Тесты | Чем заменён в v2 | Критерий удаления | Часть |
|---|---|---|---|---|---|---|
| `dialogue_node.py` | 4 700 | `core/stt_admission_host.py` (под `TYPE_CHECKING`); `setup.py:49`, launch ×2; путь в `scripts/lint/dialogue_skip_reasons.py:12–14` | 47 (10 685) | `dialog_node.py` (531) + `core/dialog_v2_ports.py` (298) + `rob_box_dialog/turn.py` | `ls` → нет; `grep -n dialogue_node src/rob_box_voice/setup.py` = 0; `ros2 node list` без `/dialogue_node` | 15p / 15b…15p |
| `config/dialogue_node.yaml` ×2 (`src/rob_box_voice/config/`, `docker/sound/config/voice_assistant/`) | 141 + 141 | launch ×2 | — | `config/dialog_node.yaml` | `ls` обеих → нет | 15p |
| `core/dialogue_stage_mixin.py` | 81 | `dialogue_node` (база класса `DialogueNode`) | 1 (836) | `rob_box_dialog/turn_log.py`, `core/dialog_v2_ports.py` | `grep -rlw dialogue_stage_mixin src scripts --include=*.py` вне тестов = 0 | 15m2 |
| `core/turn_stages.py` | 275 | `dialogue_node`, `dialogue_stage_mixin` | 2 (1 026) | `core/dialog_v2_ports.py` + `rob_box_dialog/expression.py` | то же по имени = 0 | 15m2 |
| `core/turn_tools.py` | 111 | `dialogue_stage_mixin`, `turn_stages` | 0 | `rob_box_dialog/execute.py` | = 0 | 15m2 |
| `core/turn_reflexes.py` | 130 | `turn_stages`, `turn_tools` | 0 | `rob_box_dialog/expression.py` (`reflex()`) | = 0 | 15m2 |
| `core/stt_admission.py` | 840 | `dialogue_node` (`DefaultSttAdmission`, `:127`, `:457`), `dialogue_stage_mixin`, `stt_admission_host`, `tier1_host` | 6 (2 156) | `rob_box_dialog/address.py`, `backlog.py`, `grammar.py` | = 0; `parse_tg_prefix` вне тестов = 0 | 15m3a (545–840) + 15m3b (1–544) |
| `core/stt_admission_host.py` | 261 | `dialogue_node` | 0 | `dialog_node.py` | = 0 | 15m1 |
| `core/tier1_host.py` | 236 | `dialogue_node`, `stt_admission_host` (рантайм-импорт `:36`), `tools_view_host` | 2 (432) | `rob_box_dialog/grammar.py` + `interrupt.py` | = 0 | 15m1 |
| `core/tools_view_host.py` | 34 | `dialogue_node` | 1 (74) | `rob_box_dialog/tools_view.py` | = 0 | 15m1 |
| `core/stop_chooser.py` | 85 | `dialogue_node`, `tier1_host` | 1 (112) | `rob_box_dialog/interrupt.py` (`stop_verdict`) | = 0 | 15m5 |
| `core/media_plan_run.py` | 128 | `dialogue_node` | 8 (1 309) | `rob_box_dialog/plan/*` + `core/plan_host.py` | = 0 | 15m4 |
| `core/library_command.py` | 63 | `media_plan_run` | 1 (236) | `core/media_router.py` | = 0 | 15m4 |
| `core/turn_speech.py` | 163 | `dialogue_node` | 1 (94) | `rob_box_dialog/respond.py` | = 0 | 15m4 |
| `core/turn_identity.py` | 146 | `dialogue_node` | 2 (359) | `rob_box_dialog/speaker_adapter.py`, `context.py` | = 0 | 15m4 |
| `core/meeting.py` | 53 | `dialogue_node` | 2 (208) | **#3916**: `rob_box_dialog.meeting` + `dialog_v2_parity.subscribe_meeting` | `ls` → нет | 15m4 |
| `core/identity_dedup.py` | 83 | `dialogue_node` | 1 (234) | `rob_box_dialog/speaker_adapter.py` | = 0 | 15m5 |
| `core/session_epoch.py` | 62 | `dialogue_node` | 2 (197) | эпоха у `tts_node` (`SpeechCancel`, `interrupt.py`) | = 0 | 15m5 |
| `core/tars_status_feed.py` | 41 | `dialogue_node` | 1 (37) | **#3916**: `rob_box_dialog.status_feed` + `dialog_v2_parity.start_v2_status_feeds` | `ls` → нет | 15m5 |
| `startup_greeting.py` | 14 | `dialogue_node` | 2 (407) | **#3916**: `rob_box_dialog.startup` + `dialog_v2_parity.arm_startup_greeting` | `ls` → нет | 15m5 |
| `speaker_profiles.py` | 181 | `dialogue_node` | 2 (439) | не переносится (D1a): «кто говорит» — `speaker_id_node` → `SpeakerRef` | `ls` → нет | 15m5 |
| **Итого множество** | **7 687** | 20 файлов (`dialogue_node` 4 700 + 19 модулей 2 987) | | | | |

Модули вне множества (не удаляются целиком, сокращаются или удаляются по отдельному решению):

| Файл | Строк | Читатели | Тесты | Что с ним | Часть |
|---|---|---|---|---|---|
| `core/greeting_window.py` | 79 | `dialog_node` (`:79`), `core/dialog_v2_parity` (`:22`), `dialogue_stage_mixin`, `meeting`, `stt_admission_host` | 1 (154) | **Остаётся на месте (B1)**: окно ответа на приветствие v2. После 15m1/15m2/15m4 его читатели — только v2. Перенос в `rob_box_dialog` в PR-15 не нужен (живой v2 не трогаем без нужды) | — |
| `core/turn_context.py` | 79 | `plan_host` (v2, `tts_state`), `tier1_host` (`tts_state`), `dialogue_node` (`context_sections`) | 0 | **B2**: удаляются `_tts_context`, `_position`, `_performance`, `context_sections` (строки 37–76, ≈40) и их импорты; `tts_state` (22–35) **остаётся в этом файле** — так импорт v2 не меняется. Переименование файла (например, в `core/tts_state.py`) — не в PR-15 | 15m4 |
| `core/conversation_history.py` | 255 | нет (реэкспорт `core/__init__.py`) | 1 (250) | удалён | 15a1 — **влито #4092** (`1da450034`) |
| `core/llm_config.py` | 256 | нет | 1 (265) | удалён (D6) | 15a2 — **влито #4092** (`1da450034`) |
| `core/voice_memory_init.py` | 113 | нет | 1 (156) | удалён (D6) | 15a2 — **влито #4092** (`1da450034`) |
| `utils/github_summary.py` | 297 | нет | 1 (436; грузит файл по пути через `importlib`, AST не видит — N7) | удалён (D6) вместе с тестом | 15a1 — **влито #4092** (`1da450034`) |
| `core/skip_reason_checker.py` | 35 | **`scripts/lint/dialogue_skip_reasons.py:9`** (гард CI, `G-Lint Code.yml:70–72`) | 1 (54) | **не мёртвый (B6)**: гард проверяет ключи `_llm_skipped_counter` в `dialogue_node.py` по пути. Удаляется вместе с гардом и шагом workflow в 15p | 15p |
| `core/llm_skip_reasons.py` | 47 | `dialogue_node`, `dialogue_stage_mixin`, `skip_reason_checker` | 11 (1 998) | сирота после 15p и 15m2 | 15m3a |
| `core/faq_loader.py` | 151 | нет (реэкспорт `core/__init__.py`) | 1 | удалён (Q3 «да», Шифу 27.10) | 15a3 — **влито #4092** (`1da450034`) |
| `core/speech_formatter.py` | 262 | нет (реэкспорт `core/__init__.py`) | 1 | удалён (Q3 «да», Шифу 27.10) | 15a3 — **влито #4092** (`1da450034`) |
| `core/speech_compat.py` | 74 | `tts_node`, `speech_owner` | 2 (653) | удалить после teleop (15-0d) и корня (15p) | 15q |
| `core/speak_helpers.py` | 594 | `dialogue_node`, `tts_node`, **`scripts/dialogue/chat.py:571`** | 11 (1 627) | остаются `strip_bracket_asides`, `strip_markdown`, `unsupported_language_notice`, `provider_speaks` (97 строк функций по AST) — нормализация перед синтезом (D2); удаление до −497 | 15q |
| `core/dialogue_guards.py` | 169 | `dialogue_node`, `stt_admission_host`, `turn_speech`, `track_start_guard`, `tts_node` | 3 (325) | остаются `is_system_template_regurgitated_in_ssml`, `is_tool_call_markup` (`tts_node`) и `MUSIC_STARTING_TOOLS`; 169 → ≈70 (оценка 23.10) | 15s |
| `core/dialogue_helpers.py` | 142 | `dialogue_node`, `speaker_turn` (`sanitize_speaker_name`) | 1 (176) | остаётся `sanitize_speaker_name` (19); удаление до −123 | 15m5 |
| `core/dialogue_text.py` | 48 | `dialogue_node`, `stt_node`, `scripts/dialogue/chat.py`, `scripts/music/live_dj/router_offline.py` | 9 (1 950) | остаётся | — |
| `rob_box_harness/core/dialogue_state_machine.py` | 469 | `dialogue_node`, `meeting`, `stt_admission_host`, `tier1_host`, `agent_core` (`:50`, `:616`), `assembly` (`:50`, поле `AgentSpec.dsm` `:152` — **`AgentSpec` строит и v2**), `supervisor_node` (`:1971`, `:2305`), **`scripts/dialogue/chat.py:451,562,633`**; реэкспорт `harness/core/__init__.py:10–19` | 35 (12 231; 15 общих с тестами множества) | удалить; из `AgentSpec` убрать поле `dsm` (v2 его не передаёт) | 15r1 |
| `rob_box_harness/core/agent_core.py`, класс `AgentCore` | 1 569 из 2 349 | `dialogue_node`, `assembly.build_agent`, `scripts/dialogue/chat.py`; v2 берёт из модуля только `answering_provider` (7 строк, `dialog_v2_ports.py:35`) | 31 (9 615) на модуль | второй цикл хода v1 — **Q4** | 15r3 (если Q4 «да») |

## 4. Топики и контракты

Счёт — `git grep -F` по `src docker scripts architecture tools .github` вне тестов (строк / файлов, приложение Б);
`.md`-файлы и комментарии внутри этих каталогов в счёт попадают. «Рантайм» — аудит `run 40499068353` (§10).
**Поправка N5:** фильтр «вне тестов» скрипта применён к строке `путь:строка:текст`, а не к пути, поэтому `/voice/tts/request`
на деле 60 / 22 (не 64 / 23), `/dialogue/control` 48 / 11 (не 49 / 12), `/voice/tts/control` 8 / 5 (не 7 / 4), `dialog_engine`
77 строк в 27 файлах (не 75); остальные счётчики совпали.

| Топик | Писатель | Кто ещё держит топик (вне тестов) | Чем заменён | Критерий | Часть | Рантайм 27.10 |
|---|---|---|---|---|---|---|
| `/voice/dialogue/state` (23 / 13) | `dialogue_node.py` (1); при v2 никто | адаптеры v1 (TEMP, #3917): `led_node` 3, `arbiter_node` 3, `quest_node` 3, `rob_box_core/voice_state.py` 3 (`on_legacy`, `LEGACY_STATE`), `supervisor/core/aggregator.py` 2; `bridge_protocol.py` 1 (`source=`), `mock_robot.ts` 1, `agent_core` 1, `supervisor/core/__init__` 1, `SessionState.msg` 1 (комментарии); `create_avatar_issues.sh` 2, `led_face_lab/preview_template.html` 1. **Ушли**: `command_node`, `quest/streams/voice_state.py` | `SessionState` + `TurnEvent` + `SpeechEvent` через `voice_state.watch()` (#3917) | `grep -rFn "/voice/dialogue/state" src docker scripts --include=*.py --include=*.ts --include=*.yaml` вне тестов = 0 | 15-0a′ (bridge, mock, runner) → 15p2 (адаптеры) | `dead_input` (новый): читатели `/avatar_arbiter`, `/led_node`, `/quest_node` |
| `/voice/dialogue/response` (46 / 26) | `dialogue_node.py` (3); при v2 никто | `telegram_node` 4 (ветка v1 `_subscribe_replies`, `:132`), `context_aggregator_node` 3 (TEMP `:220–223`), `tts_node` 5, `speech_compat` 3, `agent_core` 2, `speak_helpers` 1, `robot_speech.py` 1, `dialog_engine.py`/`.yaml` ×2, `e2e_voice_test.sh` 1, `tools/gui/*` 2, документы `src/*/docs`, `README` | Telegram — `/dialog/output` (PR-12a); `context_aggregator` — `SpeechRequest` (#3917, D4) | `grep -rFn "/voice/dialogue/response" src --include=*.py` вне тестов = 0 | 15p2 (aggregator), 15s (telegram), 15q/15t (`tts_node`) | `dead_input` (новый): `/context_aggregator`; запись `multiple_writers` — `resolved` |
| `/voice/tts/state` (16 / 12) | `tts_node.py:1371` | `face_node` 3, `face_director` 1, `audio_node` 2, `stt_node` 1, `speech_compat` 1, `replay.py` 1, `face_node.yaml` 1 (комм.), `led_face_lab` 2, документы | `SpeechEvent` (`/voice/speech/event`) | `grep -rFn "/voice/tts/state"` вне тестов = 0 | 15-0b → 15t | — |
| `/voice/tts/finished` (39 / 16) | `tts_node.py:1373` (12 строк) | `supervisor_node` 4, `speak_helpers` 3, `speaker_id_node` 2, `dialogue_node` 2, `tars_stage_relay` 2, `quest_node` 2, `core/tars_stages` 2, `mcp_server` 2, `conductor.py` 2, `speech_owner` 1, `speech_compat` 1, `tars_stage.py` 1, `ws_server` 1, `replay.py` 1 | `SpeechEvent finished` по `speech_id` | = 0 | 15-0c → 15t | — |
| `/voice/tts/request` (64 / 23) | джойстик `rob_box_teleop` (`joystick_logic` 2, `joystick_control_node` 1), `scene_set/conductor.py` 1; вход `tts_node` 18 через `speech_compat.attach_legacy` (`tts_node.py:1277`) | `utterance.py` 4, `tts_params.py` 3, `speech_client.py` 1, `mcp_server` 1, `supervisor_node` 1, `watchdog-tts.sh` 5, `tools/gui` 2+2, `audio_capture_harness` 2+2, `ownership_check.py` 1, `runtime-baseline.json` 1 | `speech_client.say` → `SpeechRequest` | вне тестов и комментариев = 0 | 15-0d → 15q | запись `multiple_writers` — `resolved` |
| `/voice/tts/control` (7 / 4) | `speech_compat` | `tts_node` 2 | `SpeechCancel` | = 0 | 15q | `dead_input` (новый): `/tts_node`; `multiple_writers` — `resolved` |
| `/dialogue/control`, `/dialogue/control_ack` (49 / 12) | `supervisor_node` 28 (путь v1 ТАРС) | `dialogue_node` 11, `webxr_client` 2 (`main.ts`, `voice_pipeline_cmd.ts`), yaml ×3 (комм.), `seam_baseline.json`, `runtime-baseline.json` | `DialogControl.srv` (PR-10a) | `grep -rn '"/dialogue/control' src --include=*.py --include=*.ts` вне тестов = 0 | 15p (`dialogue_node`), 15r2 (`supervisor_node`) | `dead_output /dialogue/control_ack` — `resolved` |
| `/voice/stt/speaker` | `stt_node.py:513` | читатель только `dialogue_node.py:710` (метки Yandex, D1a) | `Utterance.speaker_tag`/биометрия | писатель удалён, запись в `ownership.yml:140–146` удалена | 15t | `dead_output` (новый): `/stt_node` |
| `/voice/speaker/{merge,observe,epithet}`, `/voice/speaker/epithet_request` | `dialogue_node.py:724,735,743` пишет, `:740` читает; v2 не пишет | `speaker_id_node` читает merge/observe/epithet, пишет epithet_request; владелец в `ownership.yml:179,185,191` — `dialogue_node` | **нет в v2 — Q6** | — | Q6 | `dead_input` merge/observe/epithet, `dead_output` epithet_request (новые) |
| `architecture/ownership.yml` | 34 упоминания `dialogue_node`: блок `nodes.dialogue_node` (`:34–38`), владелец 4 топиков (`:163,179,185,191`), `writers_allowed`/`readers`; `speech_compat` (`:88`, комментарий `:95`) | — | — | `grep -c dialogue_node architecture/ownership.yml` = 0 | 15p (блок узла и списки), 15t | — |
| Бюджеты | `cc_budget_baseline.json:16,110`, `class_budget_baseline.json:74` (`DialogueNode`), `seam_baseline.json`, `seam_allowlist.json:17`, `runtime-baseline.json` | — | пересчитать вниз | записи `dialogue_node.py` = 0 | 15p, 15t, 15w | — |

## 5. Читатели, которые надо перевести до удаления

Образы: **voice-assistant**, **supervisor** (`supervisor` и `avatar-arbiter`), **quest**, **telegram-bot** — Sound Pi;
**led-matrix** — Vision Pi; **teleop**, **perception** — Main Pi.

| Часть | Что переводим | Состояние 27.10 | Читатели (`wc -l` файла) | Образы | Размер правки |
|---|---|---|---|---|---|
| 15-0a | `/voice/dialogue/state` → `SessionState`/`TurnEvent`/`SpeechEvent` | **в основном сделано #3917**: `quest_node` (3 182) и `arbiter_node` (1 518) — `voice_state.watch()` + адаптер `on_legacy`; `led_node` (326) — `SessionState` + строка v1; `command_node` (380) — подписка удалена; `streams/voice_state.py` (33) — только отправка | оставалось (15-0a′, **влито #4096**, `e5cf959e1`): `bridge_protocol.py` (1 510; `source="/voice/dialogue/state"`), `webxr_client/src/dev/mock_robot.ts` (944), `docker/vision/test/scenario_runner/runner.py` (864), вход `snapshot().voice_state` арбитра (решение координатора в #3711); адаптеры v1 — в 15p2 | quest, supervisor | оценка 30–80 (протокол шлема — гард `G-Bridge-Protocol-Drift`) |
| 15-0b | `/voice/tts/state` → `SpeechEvent` | **влито #4099** (`73450cb86`): читатели (лицо, `audio_node`, `stt_node`, `replay.py`) — `speech_client.watch_speaking`; в `tts_node.py:1371` остался только издатель (15t) — было: не начато | `face_node.py` (274), `face_director.py` (369), `audio_node.py` (1 187), `stt_node.py` (1 894); скрипт `replay.py` | led-matrix, voice-assistant | оценка 80–150 |
| 15-0c | `/voice/tts/finished` → `SpeechEvent finished` | **частично влито #4094** (`20ad0aef1`): `speaker_id_node` (якорь роста галереи) и дирижёр сцен `conductor.py` ждут конца своей речи по `SpeechEvent`. **Заблокировано:** реле стадий ТАРС в шлеме (`quest_node` → `TarsStageRelay`) — у реплики в шлем нет `SpeechEvent`: `tts_node._on_avatar_tts_request` не делает `admit`, владелец отвергает `sink=headset`; путь — PR-13 #3708 (ТАРС на `SpeechRequest(agent=operator, sink=headset)`) или правка `tts_node` (15t). До этого `/voice/tts/finished` читают `quest_node` (ТАРС) и `dialogue_node` (v1). Было: не начато | `quest_node.py`, `core/tars_stages.py` (184), `tars_stage_relay.py` (145), `server/ws_server.py`, `supervisor_node.py` (3 065), `supervisor/tars_stage.py`, `speaker_id_node.py` (2 505), `mcp_server.py` (1 557); скрипты `conductor.py`, `replay.py` | voice-assistant, quest, supervisor | оценка 100–200 |
| 15-0d | teleop: `speech_client.say` вместо JSON `/voice/tts/request` (D8) | джойстик и образ teleop — **влито #4097** (`2422c7148`, на роботе не проверено); дирижёр сцен — сделан в #4094 (15-0c); остальные места N6 (`tools/gui` ×2, `audio_capture_harness`, `watchdog-tts.sh`) — не начаты (**15-0d влито частично**) | `joystick_control_node.py` (443), `joystick_logic.py` (102); тесты `test_issue_3107_tts_request.py`, `test_joystick_*.py`. Образец сборки — `docker/sound/quest/Dockerfile:192` (`COPY --from=msgs /ws/install /ws/install`) и `:211–212` (`src/rob_box_dialog_msgs`, `src/rob_box_core` в сборке) | **teleop (Main Pi)** — единственная пересборка Main Pi, предупредить координатора | оценка 60 + Dockerfile 10 |
| 15-0e | `/voice/dialogue/response` | **сделано по сути**: `context_aggregator_node` (712) читает `SpeechRequest` (`robot_speech.py`, 36) — #3917, D4; TEMP-подписка v1 → 15p2. `telegram_node` (688): ветка v1 `_subscribe_replies` (`:132`) живёт под флагом → 15s | — | — | — |
| 15-0f | паритет D1 | **сделано #3916** (`dab053510`): встреча лицом, стартовая фраза, фид ТАРС; юнит-тесты; **на роботе не проверено**. Отличие: в v1 `_on_vision_event` ещё зовёт `note_face` и `note_face_seen` (`dialogue_node.py:4161`, `:4174`) — у v2 потребителя нет, это `FaceSightings` #3920 (открыта, после эпика) | — | — | — |

Путь v1 ТАРС (`supervisor_node.py`, по AST): `_dialogue_control_publisher` 13, `_publish_dialogue_control` 33,
`_dialogue_control_swap` 48, `_ensure_agent_core` 14, `_build_agent_core_sync` 83 = **191** строка; ветка v1 `agent_path`
— `operator_turns.py:235–240`; часть 15r2.

## 6. План частей (новый порядок)

### 6.1. Почему порядок 23.10 не годится

1. **Импорт v1 ломается раньше, чем карта ожидала.** Модуль можно удалить, только когда удалены все его импортёры. Это не
   выполнялось: 15a удаляла хвост `stt_admission.py` (545–840), а `DefaultSttAdmission` из него `dialogue_node` строит в
   `__init__` (`:127`, `:457`); 15c удаляла `tier1_host.py`, а его на рантайме импортирует `stt_admission_host.py:36`
   (удалялся позже, в 15d). Вывод скрипта «где `dialogue_node.py` использует имена модулей v1» (приложение А) показывает и
   остальное: `tools_view_host` нужен методу части 15k, `turn_identity` и `speaker_profiles` — 15l, `session_epoch` — 15j,
   15l и `__init__`, `turn_speech` — 15h, 15k, 15l. Модули из 15c/15d/15e/15g/15h карты 23.10 удалять в своих частях нельзя.
2. **Откат флагом на v1 умирает с первой части, режущей `dialogue_node.py`.** Каждая такая часть лишает v1 функции
   (после 15c v1 не слышит реплику), поэтому `dialog_engine: v1` перестаёт быть рабочим запасом задолго до 15s. Промпт
   этапа ставит вердикт PR-14 только перед 15p и 15s — это решение надо уточнить (**Q1**).

### 6.2. Порядок

Стратегия **R (рекомендую)**: после вердикта PR-14 сначала удаляется корень `dialogue_node.py` (одна часть, чистое удаление),
затем модули-сироты по графу «импортёр раньше импортируемого». Каждое промежуточное состояние импортируется целиком, перекрёстные
вызовы методов `DialogueNode` анализировать не нужно. Цена — часть 15p ≈ 5 060 строк: больше потолка AF-0013 (3 000), нужна
метка `big-bang-override` от товарища Шифу. Стратегия **L** (если метки нет) — в §6.3.

«Строк» — удаляемые не-тестовые строки (`wc -l` файлов; диапазоны AST; «оценка» — не измерено). Тесты удаляются в тех же PR и
в размер не входят. Каждая часть меняет образ **voice-assistant**, если не сказано иное. Смоук каждой части: `ros2 node list`
(есть `/dialog_node`, нет `/dialogue_node`), последние 50 строк `docker logs voice-assistant` без `Traceback`; «фразы» —
инъекция `Робот, …` в `/voice/stt/utterance` (без пунктуации, один `ssh`, стенд свободен).

**Этап 1 — можно вливать до вердикта PR-14** (v1 не трогается):

| Часть | Что | Строк | Образы | Зависит от | Что проверить |
|---|---|---|---|---|---|
| 15-0a′ | **влито #4096** (`e5cf959e1`). Остаток 15-0a: `bridge_protocol.py` `source`, `mock_robot.ts`, `scenario_runner/runner.py`, вход `snapshot().voice_state` арбитра | оценка 30–80 | quest, supervisor | — | шлем: `voice_state (0x1202)` меняется на «Робот, который час» (listening → thinking → speaking → idle) |
| 15-0b | **влито #4099** (`73450cb86`). `/voice/tts/state` → `SpeechEvent` | оценка 80–150 | led-matrix, voice-assistant | — | лицо открывает рот на «Робот, расскажи скороговорку», `audio_node` не пишет поверх речи |
| 15-0c | **частично влито #4094** (`20ad0aef1`); реле ТАРС в шлеме — после #3708 или правки `tts_node` (15t). `/voice/tts/finished` → `SpeechEvent finished` | оценка 100–200 | voice-assistant, quest, supervisor | — | ТАРС: 3 команды с озвучкой, стадии в шлеме (после #3708); `speak_text` **асинхронный** (`async: true`, возвращает «принято» сразу) — конец чанка читает по `SpeechEvent` и пишет в лог (`SpeakTextTool._on_speech_event`), ждать конца речи он не обязан |
| 15-0d | **влито частично #4097** (`2422c7148`: джойстик, образ teleop); N6 не начат. teleop → `speech_client.say` (D8); **+ N6:** остальные места JSON `/voice/tts/request`, которые замолчат после 15q (`scripts/maintenance/scene_set/conductor.py` уже переведён в #4094): `tools/gui/robot_control_gui.py:39`, `robot_control_gui_simple.py:52`, `tools/audio_capture_harness/audio_capture_harness.py:549, :828`, `scripts/diagnostics/watchdog-tts.sh:63–102` (счёт издателей — на новый топик) | оценка 70 + 60–120 (N6) | **teleop (Main Pi)** | — | кнопка голоса джойстика — фраза звучит; в `ros2 topic info /voice/tts/request` писателей 0 |
| 15a1 | **влито #4092** (`1da450034`). `core/conversation_history.py` (255), `utils/github_summary.py` (297), реэкспорт в `core/__init__.py:10` (N4); тесты `test_conversation_history.py` (250) и `test_github_summary.py` (436, N7) | 552 | voice-assistant | — | смоук; `grep -rlw "conversation_history\|github_summary" src scripts --include=*.py` вне тестов = 0 (docstring-ссылки на удалённый `ConversationHistory` в `voice_memory.py:28,30,578` убраны хвостами этапа 1) |
| 15a2 | **влито #4092** (`1da450034`). `core/llm_config.py` (256), `core/voice_memory_init.py` (113) — D6 | 369 | voice-assistant | — | смоук; grep по именам = 0 (docstring `deepseek.py:746` с `llm_config.PROVIDERS` исправлен хвостами этапа 1) |
| 15a3 | **влито #4092** (`1da450034`). `core/faq_loader.py` (151), `core/speech_formatter.py` (262) — Q3 «да» (Шифу 27.10); **правка `core/__init__.py:8` и `:11`** (реэкспорт `SpeechFormatter`, `load_faq_items`; без неё `ImportError` у всего пакета, N4) | 413 | voice-assistant | — | смоук; `python -c "import rob_box_voice.core"` |

**Гейт: вердикт товарища Шифу по PR-14 (#3710) и ответ на Q1.**

**Этап 2 — после вердикта**, стратегия R:

| Часть | Что | Строк | Образы | Зависит от | Что проверить |
|---|---|---|---|---|---|
| 15p | корень: `dialogue_node.py` (4 700); `setup.py:49`; `config/dialogue_node.yaml` ×2 (282); ветки `dialogue_node` в `voice_assistant.launch.py:114–120` и `voice_assistant_headless.launch.py:88–93` **и вызов `OpaqueFunction(dialog_engine.launch_nodes, args=[dialogue_node, …])` (`:247`, `:188`) — иначе `NameError` при сборке launch (N2)**; блок `nodes.dialogue_node` и списки в `ownership.yml` **с переносом `core/plan_host.py` в `nodes.dialog_node.source` (N1)**; тексты/логпарсеры с `dialogue_node` вне B6 (N8: `L-E2E Voice Test.yml:716–744`, `e2e_tool_match.py`, `run_bench.py:80`, `an_set.py:37`, `night_dj.sh:14`, `logparse.py:7`, `bridge_protocol.py:626/913`, `protocol_generated.ts:159/316`); гард skip-reasons: `scripts/lint/dialogue_skip_reasons.py` (36), `core/skip_reason_checker.py` (35), шаг `G-Lint Code.yml:70–72`, тест `test_dialogue_skip_reasons.py`; записи в `cc_budget_baseline.json:16,110`, `class_budget_baseline.json:74`; тексты `/dialogue_node` в `.github/e2e/scenarios/turn_occasion_meeting_v1.json:23`, `docker/vision/test/config/voice_assistant_test.yaml:6`; 47 тестов ноды по AST (10 685 строк) и ещё 7, которые ссылаются на ноду строкой (`patch("rob_box_voice.dialogue_node…")`, всего 54 по grep) | 5 053 измерено + оценка 20–60 (launch, ownership, baselines) | voice-assistant; **метка `big-bang-override`** | вердикт PR-14, Q1 | смоук + набор из 10 фраз; `ls …/dialogue_node.py` → нет. **Не трогать**: `PERSONALITY_SENDER = "dialogue_node"` (`mcp_auth.py:85` ← `dialog_node.py:211`) и ключ `dialogue_node:` в `slice_policy.yaml:26` — это подпись и срез тулов живого v2; переименование — отдельное решение |
| 15p2 | адаптеры v1 в читателях (TEMP #3917): `quest_node.py:2145–2150, 2473–2474`, `arbiter_node.py:352, 1004–1005`, `led_node.py:29, 175`, `context_aggregator_node.py:220–223, 440`, `rob_box_core/voice_state.py` (`LEGACY_STATE`, `on_legacy`) | оценка 80–150 | quest, supervisor, perception, voice-assistant | 15p | LED-кольцо гаснет на «Робот, хватит», шлем показывает состояние, в памяти перцепции есть «ответы робота» |
| 15m1 | `core/stt_admission_host.py` (261), `core/tier1_host.py` (236), `core/tools_view_host.py` (34) | 531 | voice-assistant | 15p | «Робот, стой», «Робот, хватит», «Робот, покажи анимацию стоп» (спорный стоп — не стоп) |
| 15m2 | `core/dialogue_stage_mixin.py` (81), `turn_stages.py` (275), `turn_tools.py` (111), `turn_reflexes.py` (130) | 597 | voice-assistant | 15p | «Робот, который час»: стадии `addressed → thinking → done`, рефлекс LED и earcon |
| 15m3a | `core/stt_admission.py` хвост 545–840 (296; по AST начинается раньше — «Default steps» `:538`, `EmptyTextStep` `:544`, N10) + `core/llm_skip_reasons.py` (47) | 343 | voice-assistant | 15m1, 15m2 | смоук |
| 15m3b | `core/stt_admission.py` строки 1–544 (весь файл); переписать `test/ros_stubs.py:324` `make_utterance` (лениво импортирует `parse_tg_prefix`; зовут 12 тестов `unit/node/*`, N7) | 544 | voice-assistant | 15m3a | смоук; `grep -rn parse_tg_prefix src` вне тестов = 0 |
| 15m4 | `core/media_plan_run.py` (128), `library_command.py` (63), `turn_speech.py` (163), `turn_identity.py` (146), `meeting.py` (53); `turn_context.py` строки 37–76 (≈40, B2) | 593 | voice-assistant | 15p | «Робот, включи музыку», «Робот, выключи музыку»; знакомый в кадре → приветствие v2 (#3916) |
| 15m5 | `speaker_profiles.py` (181), `core/identity_dedup.py` (83), `session_epoch.py` (62), `stop_chooser.py` (85), `tars_status_feed.py` (41), `startup_greeting.py` (14); `dialogue_helpers.py` до −123 (остаётся `sanitize_speaker_name`; **тест `rob_box_core/test/test_speaker_names.py:56` грузит файл по пути и проверяет алиас `INVALID_SPEAKER_NAMES` — переписать, N7**) | 589 | voice-assistant | 15p, 15m1 | рестарт → стартовая фраза v2; ТАРС видит статус LLM; e2e-режим БД дикторов: «Робот, меня зовут Саша» |
| 15r0 | REPL v1 `scripts/dialogue/` (`chat.py` 777, `local_tools.py` 209, `run.sh` 39, `run.ps1` 54, `README.md` 118) — B6: импортирует DSM и `strip_done_marker`. Аналог v2 — `scripts/dialog/dry_stand.py` (#3906) | 1 197 (чистое удаление каталога скриптов; > 600 — с разрешения координатора или двумя PR) | образ не меняется | 15p | `python scripts/dialog/dry_stand.py --help` работает |
| 15q | `core/speech_compat.py` (74), `core/speak_helpers.py` до −497; `attach_legacy` (`tts_node.py:1277`), `meta_from_legacy` (`speech_owner.py:100`). Нормализация перед синтезом — один модуль рядом с `tts_node` (D2), число `re.compile` фиксируется в PR (сейчас 18 + 2) | ≤ 571 + оценка 25 | voice-assistant | 15-0d, 15p, 15r0 | `/say` из Telegram (русский и CJK), скороговорка с markdown — звёздочки не звучат, джойстик |
| 15r2 | путь v1 ТАРС: 5 методов `supervisor_node.py` (191 по AST), `/dialogue/control*` в `supervisor_node` (28 строк), ветка v1 `agent_path` (`operator_turns.py:235–240`) | 191 + оценка 30–60 | supervisor | 15p | ТАРС: 8 команд как в #3708; `hold`/`resume` через `DialogControl`; «замолчи» → «говори» |
| 15r1 | **порядок (N3): при Q4 «да» — после 15r3, `AgentCore` читает DSM (`agent_core.py:495, 616–683, 841–879, 1042`); при Q4 «нет» — правка логики хода в ≈12 местах и 87 упоминаний DSM в `test_agent_core.py`, а не 20–40 строк**. `rob_box_harness/core/dialogue_state_machine.py` (469); импорт и поле DSM в `agent_core.py:50–53, 616`, `assembly.py:50, 152` (`AgentSpec.dsm` — общий с v2, поле убрать), реэкспорт `harness/core/__init__.py:10–19`, `supervisor_node.py:2305` | 469 + оценка 20–40 | voice-assistant, supervisor | 15p, 15m1, 15m4, 15r0, 15r2 | ТАРС и Личность: по 3 фразы; `pytest -v src/rob_box_harness` |
| 15r3 | **только если Q4 «да»**: класс `AgentCore` (`agent_core.py:586–2154`, 1 569), `DialogResult` (99), `assembly.build_agent` (109), сироты `tool_loop/{confirmation 70, outcomes 102, output_format 117, retry 171}` и `core/acceptance.py` (620); `answering_provider` (7) перенести к v2 | ≈ 2 860 измерено + помощники v1 в `agent_core` (не измерено) → 5 PR | voice-assistant, supervisor | 15r2, 15r0 (**15r1 идёт после 15r3, N3**) | `pytest -v src/rob_box_harness src/rob_box_voice`; 10 фраз |
| 15s | флаг `dialog_engine`: 75 строк в 27 файлах (без `rob_box_core/dialog_engine.py`), `dialog_engine.py` (77), `dialog_engine.yaml` ×2, `scripts/robot_dev/dialog_engine.sh`, ветка v1 `telegram_node` (`:132`, `:562`), ветка v1 в `e2e_voice_test.sh:634–691` (сброс сессии через `/dialogue_node`, `:676`, `:681`), `operator_prompt.py` (импорт `:18`, правило `V1` `:29`); `dialogue_guards.py` 169 → ≈70; `dialog_node` в launch ×2 обычным `Node(executable=…, name='dialog_node')` — снимет `unplaced_nodes` (§10) | оценка 350–450 | voice-assistant, supervisor, telegram-bot, perception (`node_monitor`) | 15p, 15p2, 15r2 | рестарт трёх контейнеров: 3 фразы голосом, 3 сообщения в Telegram (`Клод …`, `/say`), 3 команды ТАРС; `ros2 param get /dialog_node dialog_engine` → параметра нет |
| 15t | String-топики в `tts_node`: `/voice/tts/state` (`:1371`), `/voice/tts/finished` (`:1373`, 12 строк), JSON-вход `dialogue_callback` (`:2064`, если его не читает путь `SpeechRequest` — не проверено); `/voice/stt/speaker` (`stt_node.py:513`, `ownership.yml:140–146`); записи `ownership.yml` (`speech_compat` `:88, 95`), `runtime-baseline.json`, `seam_baseline.json`, `seam_allowlist.json` | оценка 150–250 | voice-assistant | 15-0b, 15-0c, 15q | `ros2 topic info` по каждому: писателей 0; аудит без этих записей; 5 фраз с LED, лицом, шлемом |
| 15u | промпты: `RULE #` 8 + 7 → 0 (D3, A/B) | оценка 80–150 | voice-assistant | — (не зависит от удаления) | A/B: набор честности ≥ 60 фраз до и после (A3 не хуже), A1/A14 по `turn_metrics.py` |
| 15v | `#NNNN` в `rob_box_dialog/rob_box_dialog`: 369 строк → ≤ 20; `Bug [A-F]` остаток (`media_command_grammar.py` 1, `dialogue_guards.py` 1) → 0; **+ N9 (предложение): `re.compile` в слое диалога (D2) — 12 строк в 7 файлах `rob_box_dialog`: `address` 2, `grammar` 1, `greeting_reply` 1, `reply_grammar` 1, `words` 3, `plan/rap_pauses` 3, `plan/segments` 1** | оценка ≈350 строк правок комментариев + 12 регулярок → 2 PR | — | 15p…15m5 | `pytest -v src/rob_box_dialog`, смоук |
| 15w | `test_issue_*`: в `rob_box_voice/test` 83 (28 уходят с v1-множеством, ещё 2 с DSM; ≈53 переписать на поведение), в других пакетах `src` 34, в `tests/` 24; 12 смешанных тестов (приложение А); `CONTEXT.md` по ADR §13; baseline `cc_budget`/`class_budget`/`seam` вниз | оценка 200–400 | — | все | `pytest -v` всех пакетов; гарды `cc_budget`, `class_budget`, `seam_without_consumer`, `cc_budget_refs`, `ownership_check`; `L: Architecture Audit` |

Счёт частей (R): до вердикта 7 (6 и 15a3: Q3 «да», Шифу 27.10), после — 18 (15p, 15p2, 15m1, 15m2, 15m3a, 15m3b, 15m4, 15m5, 15r0, 15q, 15r2,
15r1, 15s, 15t, 15u, 15v ×2, 15w) и ещё 5 при Q4. Ориентир 600 строк превышают: 15p (5 053+, метка), 15r0 (1 197, скрипты), 15r3 (по Q4).

### 6.3. Стратегия L (если метки `big-bang-override` нет)

`dialogue_node.py` режется по методам — диапазоны AST те же, что 23.10 (файл не менялся побайтно): 15b 154, 15c 148,
15d 207, 15f 482, 15g 318, 15h 206, 15i 208, 15j 412, 15k 465, 15l 327, 15n 509, 15o 230 (+ `__init__` 484 и 550 строк вне
методов — 15p). В части L удаляются **только методы `DialogueNode` и использующие их строки `__init__`**; модули v1 удаляются
после 15p теми же частями 15m1–15m5, что в R (иначе ломается импорт, §6.1). Перекрёстные вызовы `self._…` между частями не
проверены (раздел «Что не проверено»). Частей на 12 больше, чем в R; все — после вердикта PR-14.

## 7. Решения (приняты; промпт этапа 0, раздел 3) и вопросы к товарищу Шифу

| # | Решение | Состояние на `d076b8b55` | Где в плане |
|---|---|---|---|
| D1a | `SpeakerTracker`, метки Yandex `speaker_tag` — удалить, не переносить (Шифу 23.10, комментарий #3711 6360247947) | на месте: `speaker_profiles.py` (181) | 15m5; писатель `/voice/stt/speaker` — 15t |
| D1b | встреча лицом — перенести в v2 | **слито #3916** (`dab053510`): `rob_box_dialog.meeting`, `dialog_node.py:303`; юнит-тесты; на роботе не проверено. Без `FaceSightings` (#3920) | обёртка v1 `core/meeting.py` (53) — 15m4 |
| D1c | стартовое приветствие — перенести | **слито #3916**: `rob_box_dialog.startup`, `dialog_node.py:304`; на роботе не проверено | `startup_greeting.py` (14) — 15m5 |
| D1d | фид статуса ТАРС — перенести | **слито #3916**: `rob_box_dialog.status_feed`, `dialog_node.py:302`; PR-13b #3708 влит отдельно (`58187f868`) | `core/tars_status_feed.py` (41) — 15m5 |
| D2 | `re.compile` = 0 в слое диалога (`rob_box_dialog`, `dialog_node`); нормализация перед синтезом — один модуль рядом с `tts_node`, число фиксируется в PR | `speak_helpers.py` 18, `dialogue_guards.py` 2 | 15q, 15s |
| D3 | `RULE #` → 0 отдельным PR с A/B (≥ 60 фраз до и после; откат, если A3 хуже) | 8 + 7 | 15u |
| D4 | `context_aggregator_node` — перевести с `/voice/dialogue/response` | **сделано #3917**, но на `SpeechRequest` (запрошенная речь агента `personality`, `commit=true`), а не на `SpeechEvent`/`/dialog/output`, как сказано в D4. Если нужна именно сказанная речь — отдельная правка | TEMP-подписка — 15p2 |
| D5 | после 15s откат только гитом; 15s — последней из функциональных частей, после вердикта PR-14 | см. Q1: фактически откат флагом умирает раньше | 15s |
| D6 | мёртвые модули удалить в 15a с grep-доказательством | `llm_config` (256), `voice_memory_init` (113), `utils/github_summary` (297) — читателей нет. **`skip_reason_checker` не мёртвый** (B6): его импортирует гард CI `scripts/lint/dialogue_skip_reasons.py:9` | 15a1, 15a2; `skip_reason_checker` и гард — 15p |
| D7 | в PR-15 закрыть только хвосты v1 (`discard_last_reply`, `_HISTORY_MARKER_RE`, `_META_PREFIX_RE`); `voice_memory.db` — отдельная карточка | хвосты **на месте**: `_HISTORY_MARKER_RE` 2 (`speak_helpers.py:34,154`), `_META_PREFIX_RE` 2 (`:74,215`), `discard_last_reply` 10 (`agent_core` 6, `dialogue_node` 3, `turn_speech` 1); `voice_memory.db` 47 строк в 15 файлах — карточка [#4082](https://github.com/krikz/rob_box_project/issues/4082) | 15q, 15p, 15m4; `agent_core` — 15r3 (Q4): без неё 6 строк останутся |
| D8 | teleop → `speech_client.say`, сборка `rob_box_dialog_msgs` + `rob_box_core` в `docker/main/teleop/Dockerfile` по образцу quest | PR #4097 (влит `2422c7148`, образ teleop — к пересборке) | 15-0d |

**Вопросы к товарищу Шифу (новые, по этой пересверке):**

- **Q1. Гейт и стратегия.** (а) Любая часть, режущая `dialogue_node.py` или модули из его импортов, — только после вердикта
  PR-14 (иначе откат флагом на v1 умирает раньше 15s)? Рекомендую **да**. (б) Корень одним PR с меткой `big-bang-override`
  (стратегия R, ≈5 060 строк чистого удаления) или 12 PR по методам (L)? Рекомендую **R**.
- **Q2. Критерий §6 «Architecture Audit без `multiple_writers`» недостижим.** Из 14 записей 6 новых — топики-запросы и события
  исполнителей v2, писателей много по замыслу ADR-0150 (§10). Предложение: в `ownership.yml` у таких топиков поле
  `multi_writer: by_design` со ссылкой на раздел ADR и списком `runtime_writers` (имена рантайм-нод);
  `tools/architecture_runtime_findings.py` не считает `multiple_writers`, если рантайм-писатели ⊆ `runtime_writers`; критерий
  PR-15 — «0 `multiple_writers` по топикам диалога вне объявленных и 0 записей с `debt` на PR-15». Запасной вариант без кода —
  внести ключи в `runtime-baseline.json` с `debt: "by-design ADR-0150 §…"` и критерий «нет записей с долгом PR-15».
- **Q3.** Ещё два мёртвых модуля `rob_box_voice/core`, не названных в D6: `faq_loader.py` (151) и `speech_formatter.py` (262) —
  читателей нет, только реэкспорт `core/__init__.py` и по одному тесту. Удалить в 15a3? — **решено «да» (Шифу 27.10), влито #4092** (`1da450034`).
- **Q4.** Второй цикл хода v1 — класс `AgentCore` (1 569 строк) с `assembly.build_agent` и 5 модулями-сиротами (1 080):
  после 15p/15r0/15r2 его читателей нет, v2 берёт из `agent_core.py` только `answering_provider`. Удалить в PR-15 (15r3, ≈5 PR)
  или отдельной карточкой после эпика? Без него критерий #3705 (`discard_last_reply`) останется ≠ 0 (6 строк в `agent_core`).
- **Q5.** Критерий `grep -rn "Bug \[A-F\]" …` в #3711 и в §6 промпта ищет литерал «`[A-F]`» и всегда даёт 0. Заменить на
  `grep -rnE "Bug [A-F]\b" src/rob_box_voice src/rob_box_dialog --include=*.py` (сейчас 26, вне тестов 8)?
- **Q6.** Топики идентичности v1 `/voice/speaker/{merge,observe,epithet}` (владелец `dialogue_node`) и
  `/voice/speaker/epithet_request` при v2 мёртвые (аудит §10): v2 их не пишет и не читает. Их ведут методы v1
  `_publish_confirmed_identity_growth`, `_publish_speaker_observation`, `_on_epithet_request`, `_generate_epithet`.
  Перенести в v2 (как D1) или удалить вместе с подписками `speaker_id_node`?

## 8. Хвосты PR-10b, PR-11, PR-12a — команды карточек на `d076b8b55` (raw — приложение Б)

| Карточка | Команда карточки | 23.10 → 27.10 (вне тестов) | Где лежит остаток | Часть |
|---|---|---|---|---|
| #3704 PR-10b | `ls src/rob_box_voice/rob_box_voice/core/dialogue_state_machine.py` → нет | путь карточки неверный: файла там нет и не было; DSM — `src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py` (есть) | — | 15r1 |
| | `grep -rn '"/dialogue/control\|_pause_reason\|check_silence_timeout' src --include=*.py` = 0 | **10 → 10** (всего 22 → 22) | `dialogue_node` 6, `supervisor_node` 2, `dialogue_state_machine` 2 | 15p, 15r2, 15r1 |
| #3705 PR-11 | `grep -rn "_HISTORY_MARKER_RE\|_SPEAKER_TAG_RE\|_META_PREFIX_RE\|discard_last_reply\|voice_memory.db" src --include=*.py` = 0 | **59 → 61** (всего 144 → 150) | v1-хвосты 14: `speak_helpers.py:34,74,154,215`; `discard_last_reply` — `agent_core.py` 6, `dialogue_node.py:3225,3233,3241`, `turn_speech.py:13`. `voice_memory.db` 47 (было 45) — отдельная карточка #4082 (D7) | 15q, 15p, 15m4, 15r3 (Q4) |
| #3706 PR-12a | `grep -rn "\[TG:\|tg_chat_id\|parse_tg_prefix" src --include=*.py` вне тестов = 0 | **51 → 51** | `stt_admission` 19 (15m3), `dialogue_node` 18 (15p), `speak_helpers` 4 (15q), `turn_identity` 3 (15m4), `tier1_host` 1 (15m1), `speech_compat` 1 (15q), `telegram_node` 2 (`:225` комм., `:562` ветка v1 → 15s), `utterance.py:180` (комм.), **`rob_box_dialog/address.py:187,194` — живой v2 пишет `[TG:<id>]` в лог `🎤 STT`** (по этой строке координатор узнаёт сет Шифу) | критерий надо сузить: исключить формат лога v2 — решение координатора |
| | `grep -rn '"/voice/dialogue/response"\|"/voice/sound/stop"' src/rob_box_telegram` = 0 | 2 вне тестов | `telegram_node.py:132` (ветка v1 → 15s); `:279` — писатель `/voice/sound/stop` работает и при v2 (не v1-хвост; замена — команда владельцу, вне удаления v1) | 15s; `:279` — отдельно |
| #3711 | `Bug [A-F]` (Q5), `#NNNN`, `RULE #` | 26 (8) → 26 (8); 158 → 369 строк; 8 + 7 → 8 + 7 | `dialogue_node` 4, `turn_speech` 1, `session_epoch` 1 (уходят с v1); `media_command_grammar` 1, `dialogue_guards` 1 (15v) | 15p, 15m4, 15m5, 15v, 15u |
| #3708 PR-13b | влит `58187f868` — не хвост PR-15 | — | — | — |

## 9. Сводка

- **Удаляется целиком (измерено `wc -l`)**: множество v1 7 687 (`dialogue_node.py` 4 700 + 19 модулей 2 987),
  `conversation_history` 255, D6 666 (`llm_config` 256 + `voice_memory_init` 113 + `github_summary` 297), гард skip-reasons
  118 (`dialogue_skip_reasons.py` 36 + `skip_reason_checker` 35 + `llm_skip_reasons` 47), `dialogue_node.yaml` ×2 282,
  `speech_compat` 74, DSM 469, путь v1 ТАРС 191 (AST) = **9 742**; плюс REPL `scripts/dialogue/` 1 197 (15r0) = 10 939.
- **Сокращается (верхняя граница по AST)**: `speak_helpers` до −497, `dialogue_helpers` до −123, `turn_context` ≈ −40.
- **По решениям Q3/Q4**: `faq_loader` + `speech_formatter` 413; `AgentCore` и сироты ≈ 2 860.
- **Не измерено (оценка)**: перевод читателей 15-0a′…15-0d 270–500; launch/ownership/baselines в 15p 20–60; адаптеры 15p2
  80–150; флаг 15s 350–450; String-топики в `tts_node` 150–250; `#NNNN` ≈350 строк комментариев; промпты 80–150; CONTEXT.md
  и baselines 200–400.
- **Сравнение с 23.10 в том же объёме** (множество + `conversation_history` + `speech_compat` + `speak_helpers` + DSM +
  `dialogue_helpers` + часть `turn_context`): 9 507 → **9 145** (−362 = −261 перенос #3916 в `rob_box_dialog` + 6 `tier1_host`
  − 79 `greeting_window` больше не удаляется − 28 `turn_context` удаляется лишь частично). Удалено за эти дни — 0 строк v1.
- **Тесты**: импортируют множество (+D6, гард) 73 файла, 16 209 строк; DSM — 35 файлов, 12 231 строка (15 общих);
  смешанных (v1 + v2) — 12, их переписывают, а не удаляют; ноды `dialogue_node` — 47 файлов, 10 685 строк.
- **Образы**: voice-assistant — почти все части; **supervisor** — 15-0a′, 15-0c, 15p2, 15r2, 15r1, 15s; **quest** — 15-0a′,
  15-0c, 15p2; **telegram-bot** — 15s; **perception** — 15p2, 15s; **led-matrix (Vision Pi)** — 15-0b; **teleop (Main Pi)** —
  15-0d (единственная пересборка Main Pi).
- **До старта этапа 1 решать нечего** (D1–D8 приняты); **до этапа 2** — Q1, Q2, Q4; Q3 решён («да», влито #4092), Q5, Q6 — по ходу.

## 10. Рантайм-аудит 27.10 (`L: Architecture Audit`, run 40499068353, develop `d076b8b55`)

Источник: лог шага «Runtime findings» и артефакт `architecture-local-runtime` (`runtime-findings.json`). Сводка:
`{"dead_input": 30, "dead_output": 80, "duplicate_publishers": 2, "findings": 126, "multiple_writers": 14, "new": 20,
"resolved_since_baseline": 20}`; граф: `"unplaced_nodes": ["dialog_node"]`. Выборка по диалогу — приложение В.

- **`dead_input` (новые)**: `/voice/dialogue/state` (`/avatar_arbiter`, `/led_node`, `/quest_node`) и `/voice/dialogue/response`
  (`/context_aggregator`) — это TEMP-адаптеры v1 из #3917, подтверждают 15-0a/15-0e, уходят в 15p2; `/voice/tts/control`
  (`/tts_node`) — 15q; `/voice/speaker/{merge,observe,epithet}` (`/speaker_id_node`) — Q6.
- **`dead_output` (новые) по диалогу**: `/voice/stt/speaker` (`/stt_node`, читатель был только у v1) — 15t;
  `/voice/speaker/epithet_request` — Q6. Вне PR-15, к сведению: `/voice/expression/event` (писатели face/led/sound, у v2 нет
  подписчика), `/mcp/tools`, `/voice/sound/state`, `/perception/observations`.
- **`multiple_writers` (новые, 6)**: `/voice/speech/request` (`avatar_supervisor`, `dialog_node`, `mcp_server`, `stt_node`,
  `telegram_node`), `/voice/stt/utterance` (`stt_node`, `telegram_node`), `/voice/providers/state` (`dialog_node`, `mcp_server`,
  `stt_node`, `tts_node`), `/voice/tts/voice_request` (`dialog_node`, `mcp_server`; **в `ownership.yml` не объявлен**),
  `/voice/expression/event` (`face_node`, `led_node`, `sound_node`), `/perception/observations` (`vision_face`, `vision_hailo`).
  В `ownership.yml` четыре из них уже описаны со списком писателей и пометкой «by-design» (`writers_allowed`,
  `:130–138`, `:216–268`), `/voice/tts/voice_request` и `/perception/observations` там нет; аудит `ownership.yml` не читает. Предложение — Q2.
- **`resolved_since_baseline`** среди прочих: `multiple_writers` `/voice/tts/request`, `/voice/tts/control`,
  `/voice/dialogue/response`, `dead_output /dialogue/control_ack` — их записи в `runtime-baseline.json` можно снимать (15t).
- **`duplicate_publishers /mcp/execute` ×2 в `/dialog_node`** — находка. По коду это не «старый провайдер тулов»: `dialog_node`
  создаёт два `LLMToolCallAdapter` — Личности (`build_tools`, `dialog_node.py:213`) и чата Telegram (`build_chat_tools`, `:222`),
  каждый заводит свой издатель (`llm_adapter.py:151`). Разбор — карточка [#4081](https://github.com/krikz/rob_box_project/issues/4081).
- **`unplaced_nodes: ["dialog_node"]`**: ноду создаёт `rob_box_core/dialog_engine.py:71` словарём (`"name": "dialog_node"`), а
  `tools/architecture_runtime_graph.py:260` ищет в launch-файлах только `name='…'`. Уходит в 15s, когда `dialog_node`
  станет обычным `Node(...)` в launch ×2.

## Что не проверено

- На роботе ничего не запускалось; из CI — только проверки самого PR #4080 (run_id — в описании PR, не в карте). Ни одна
  часть не реализована. Размеры частей — `wc -l` файлов и диапазоны AST, а не готовые дифы; правки `__init__`, импортов и
  тестов дадут другой дифф.
- Граф видит только Python. Импорты из TypeScript (`webxr_client`), shell и YAML ищутся grep'ом по именам и могут быть
  неполными; тесты, ссылающиеся на ноду строкой, посчитаны отдельно (54 по grep против 47 по AST).
- Перекрёстные вызовы методов `DialogueNode` друг друга (`self._…`) и атрибуты из `__init__` скрипт не отслеживает: для
  стратегии L это главный риск, для R — нет (корень удаляется целиком).
- Не доказано, что JSON-вход `tts_node.dialogue_callback` не нужен пути `SpeechRequest` — поэтому он в 15t помечен «не
  проверено».
- Семантическое соответствие замен v2 (рефлексы, `stop_verdict`, окно ответа 15 с, `user_profile_lines`, встреча лицом #3916)
  не сверялось поведением — столбец «чем заменён» по назначению модуля. Живой проверки #3916 нет ни у кого.
- Оценки 15-0a′…15-0d, 15p2, 15s, 15t, 15u, 15v, 15w не измерены. Размер 15r3 — без помощников v1 внутри `agent_core.py`.
- Список «смешанных» тестов — по AST-импорту (v1-множество + `rob_box_dialog.*`/`dialog_node`/`dialog_v2_*`); тесты,
  подменяющие модули строкой (`sys.modules`, `patch("…")`), в счёт не попали.
- Число рёбер графа 23.10 (3 962) не воспроизводится — старый скрипт не сохранён; сравнивать только числа `pr15_map_facts.py`.
- `markdownlint` не запускался (в репозитории не настроен); `flake8` по `pr15_map_facts.py` — чисто (приложение А.0).
- Предложение Q2 (критерий `multiple_writers`) — схема, не код; не проверено, что `ownership_check.py` примет новое поле без
  правки.

## Приложение А. `python docs/dialog/pr15_map_facts.py --ref origin/develop --old 8178e2809`

### А.0. Сверка метрики и линт скрипта

```text
$ git show 8178e2809:src/rob_box_voice/rob_box_voice/dialogue_node.py | wc -l
4700
$ git show origin/develop:src/rob_box_voice/rob_box_voice/dialogue_node.py | wc -l
4700
$ git show 8178e2809:src/rob_box_voice/rob_box_voice/core/stt_admission.py | wc -l
840
$ git show origin/develop:src/rob_box_voice/rob_box_voice/core/stt_admission.py | wc -l
840
$ git show 8178e2809:src/rob_box_voice/rob_box_voice/core/meeting.py | wc -l
213
$ git show origin/develop:src/rob_box_voice/rob_box_voice/core/meeting.py | wc -l
53
$ git rev-parse 8178e2809:src/rob_box_voice/rob_box_voice/dialogue_node.py origin/develop:src/rob_box_voice/rob_box_voice/dialogue_node.py
4aa550f038cb732dc6b0c2fca8c599b95346baad
4aa550f038cb732dc6b0c2fca8c599b95346baad
(те же числа в выводе скрипта ниже — он считает b"\n" в blob)

$ python -m flake8 --max-line-length=120 --extend-ignore=E203,W503,E501 docs/dialog/pr15_map_facts.py; echo flake8=$?
flake8=0
```

### А.1. Вывод скрипта

```text
## ref origin/develop = d076b8b55; old 8178e2809 = 8178e2809
py-файлов: всего 1798, в src 1375; рёбер (файл→файл) всего 4429, из src 4033

### Замкнутое v1-множество от dialogue_node.py: 20 файлов, 7687 строк (wc -l)
     81  core/dialogue_stage_mixin.py                  читатели 1: dialogue_node.py | тестов 1 (836 строк)
     83  core/identity_dedup.py                        читатели 1: dialogue_node.py | тестов 1 (234 строк)
     63  core/library_command.py                       читатели 1: core/media_plan_run.py | тестов 1 (236 строк)
    128  core/media_plan_run.py                        читатели 1: dialogue_node.py | тестов 8 (1309 строк)
     53  core/meeting.py                               читатели 1: dialogue_node.py | тестов 2 (208 строк)
     62  core/session_epoch.py                         читатели 1: dialogue_node.py | тестов 2 (197 строк)
     85  core/stop_chooser.py                          читатели 2: core/tier1_host.py, dialogue_node.py | тестов 1 (112 строк)
    840  core/stt_admission.py                         читатели 4: core/dialogue_stage_mixin.py, core/stt_admission_host.py, core/tier1_host.py, dialogue_node.py | тестов 6 (2156 строк)
    261  core/stt_admission_host.py                    читатели 1: dialogue_node.py | тестов 0 (0 строк)
     41  core/tars_status_feed.py                      читатели 1: dialogue_node.py | тестов 1 (37 строк)
    236  core/tier1_host.py                            читатели 3: core/stt_admission_host.py, core/tools_view_host.py, dialogue_node.py | тестов 2 (432 строк)
     34  core/tools_view_host.py                       читатели 1: dialogue_node.py | тестов 1 (74 строк)
    146  core/turn_identity.py                         читатели 1: dialogue_node.py | тестов 2 (359 строк)
    130  core/turn_reflexes.py                         читатели 2: core/turn_stages.py, core/turn_tools.py | тестов 0 (0 строк)
    163  core/turn_speech.py                           читатели 1: dialogue_node.py | тестов 1 (94 строк)
    275  core/turn_stages.py                           читатели 2: core/dialogue_stage_mixin.py, dialogue_node.py | тестов 2 (1026 строк)
    111  core/turn_tools.py                            читатели 2: core/dialogue_stage_mixin.py, core/turn_stages.py | тестов 0 (0 строк)
   4700  dialogue_node.py                              читатели 1: core/stt_admission_host.py | тестов 47 (10685 строк)
    181  speaker_profiles.py                           читатели 1: dialogue_node.py | тестов 2 (439 строк)
     14  startup_greeting.py                           читатели 1: dialogue_node.py | тестов 2 (407 строк)

### Сироты после удаления множества + D6 + гарда skip-reasons: 1
     47  core/llm_skip_reasons.py

### Модули карты вне множества: размер, читатели (не-тест), тесты
    255  core/conversation_history.py                  в множестве=нет; читатели 0:  | реэкспорт: core/__init__.py | тестов 1 (250 строк)
     74  core/speech_compat.py                         в множестве=нет; читатели 2: core/speech_owner.py, tts_node.py | тестов 2 (653 строк)
    594  core/speak_helpers.py                         в множестве=нет; читатели 3: dialogue_node.py, scripts/dialogue/chat.py, tts_node.py | тестов 11 (1627 строк)
    169  core/dialogue_guards.py                       в множестве=нет; читатели 5: core/stt_admission_host.py, core/track_start_guard.py, core/turn_speech.py, dialogue_node.py, tts_node.py | тестов 3 (325 строк)
    142  core/dialogue_helpers.py                      в множестве=нет; читатели 2: core/speaker_turn.py, dialogue_node.py | тестов 1 (176 строк)
     48  core/dialogue_text.py                         в множестве=нет; читатели 4: dialogue_node.py, scripts/dialogue/chat.py, scripts/music/live_dj/router_offline.py, stt_node.py | тестов 9 (1950 строк)
     79  core/greeting_window.py                       в множестве=нет; читатели 5: core/dialog_v2_parity.py, core/dialogue_stage_mixin.py, core/meeting.py, core/stt_admission_host.py, dialog_node.py | тестов 1 (154 строк)
     79  core/turn_context.py                          в множестве=нет; читатели 3: core/plan_host.py, core/tier1_host.py, dialogue_node.py | тестов 0 (0 строк)
     47  core/llm_skip_reasons.py                      в множестве=нет; читатели 3: core/dialogue_stage_mixin.py, core/skip_reason_checker.py, dialogue_node.py | тестов 11 (1998 строк)
     35  core/skip_reason_checker.py                   в множестве=нет; читатели 1: scripts/lint/dialogue_skip_reasons.py | тестов 1 (54 строк)
    256  core/llm_config.py                            в множестве=нет; читатели 0:  | тестов 1 (265 строк)
    113  core/voice_memory_init.py                     в множестве=нет; читатели 0:  | тестов 1 (156 строк)
    297  utils/github_summary.py                       в множестве=нет; читатели 0:  | тестов 0 (0 строк)
    531  dialog_node.py                                в множестве=нет; читатели 0:  | тестов 2 (1019 строк)
    298  core/dialog_v2_ports.py                       в множестве=нет; читатели 4: dialog_node.py, rob_box_supervisor/rob_box_supervisor/operator_turns.py, scripts/dialog/dry_marathon.py, scripts/dialog/dry_stand.py | тестов 4 (1389 строк)
    171  core/dialog_v2_parity.py                      в множестве=нет; читатели 2: core/tars_status_feed.py, dialog_node.py | тестов 1 (203 строк)
    193  core/plan_host.py                             в множестве=нет; читатели 3: core/dialogue_stage_mixin.py, dialog_node.py, dialogue_node.py | тестов 3 (723 строк)
    469  rob_box_harness/rob_box_harness/core/dialogue_state_machine.py в множестве=нет; читатели 8: core/meeting.py, core/stt_admission_host.py, core/tier1_host.py, dialogue_node.py, rob_box_harness/rob_box_harness/core/agent_core.py, rob_box_harness/rob_box_harness/core/assembly.py, rob_box_supervisor/rob_box_supervisor/supervisor_node.py, scripts/dialogue/chat.py | реэкспорт: rob_box_harness/rob_box_harness/core/__init__.py | тестов 35 (12231 строк)

### Мёртвые модули rob_box_voice (нет не-тестовых читателей ни в src, ни вне; не точка входа)
    255  core/conversation_history.py  (тестов 1; реэкспорт core/__init__.py)
    151  core/faq_loader.py  (тестов 1; реэкспорт core/__init__.py)
    256  core/llm_config.py  (тестов 1)
    262  core/speech_formatter.py  (тестов 1; реэкспорт core/__init__.py)
    113  core/voice_memory_init.py  (тестов 1)
    297  utils/github_summary.py  (тестов 0)

### Тесты, импортирующие v1-множество (+D6, гард): 73 файлов, 16209 строк; из них смешанных (импортируют и v2: rob_box_dialog., rob_box_voice.dialog_node, rob_box_voice.core.dialog_v2_): 12
     37  src/rob_box_voice/test/identity_v17_legacy.py
    229  src/rob_box_voice/test/unit/core/test_adr0149_pr6_media_router_v2.py
    836  src/rob_box_voice/test/unit/core/test_command_bridge.py
    154  src/rob_box_voice/test/unit/core/test_greeting_window.py
    234  src/rob_box_voice/test/unit/core/test_identity_dedup.py
    112  src/rob_box_voice/test/unit/core/test_stop_chooser.py
    917  src/rob_box_voice/test/unit/core/test_stt_admission.py
     74  src/rob_box_voice/test/unit/core/test_tools_view_host.py
    243  src/rob_box_voice/test/unit/node/test_address_backlog.py
    265  src/rob_box_voice/test/unit/node/test_issue_2828_identity_question_collision.py
    247  src/rob_box_voice/test/unit/node/test_issue_3801_one_voice_one_person.py
    268  src/rob_box_voice/test/unit/node/test_speaker_turn_v17.py
Тесты DSM (AST, вкл. реэкспорт через rob_box_harness.core): 35 файлов, 12231 строк; пересечение с тестами множества: 15
Читатели DSM (не-тест): core/meeting.py, core/stt_admission_host.py, core/tier1_host.py, dialogue_node.py, rob_box_harness/rob_box_harness/core/agent_core.py, rob_box_harness/rob_box_harness/core/assembly.py, rob_box_supervisor/rob_box_supervisor/supervisor_node.py, scripts/dialogue/chat.py | реэкспорт: rob_box_harness/rob_box_harness/core/__init__.py
test_issue_*: всего 141; rob_box_voice/test 83 (из них импортируют v1-множество 28, v1 или DSM 30); другие пакеты src 34; tests/ 24

### Размеры wc -l: 8178e2809 → d076b8b55 (blob равен = файл не менялся)
    255 →   255  =     core/conversation_history.py
      - →   171  изм.  core/dialog_v2_parity.py
    225 →   298  изм.  core/dialog_v2_ports.py
    169 →   169  =     core/dialogue_guards.py
    142 →   142  =     core/dialogue_helpers.py
     81 →    81  =     core/dialogue_stage_mixin.py
     48 →    48  =     core/dialogue_text.py
     79 →    79  =     core/greeting_window.py
     83 →    83  =     core/identity_dedup.py
     63 →    63  =     core/library_command.py
    256 →   256  =     core/llm_config.py
     47 →    47  =     core/llm_skip_reasons.py
    128 →   128  =     core/media_plan_run.py
    213 →    53  изм.  core/meeting.py
    161 →   193  изм.  core/plan_host.py
     62 →    62  =     core/session_epoch.py
     35 →    35  =     core/skip_reason_checker.py
    594 →   594  =     core/speak_helpers.py
     74 →    74  =     core/speech_compat.py
     85 →    85  =     core/stop_chooser.py
    840 →   840  =     core/stt_admission.py
    261 →   261  =     core/stt_admission_host.py
    115 →    41  изм.  core/tars_status_feed.py
    230 →   236  изм.  core/tier1_host.py
     34 →    34  =     core/tools_view_host.py
     68 →    79  изм.  core/turn_context.py
    146 →   146  =     core/turn_identity.py
    130 →   130  =     core/turn_reflexes.py
    163 →   163  =     core/turn_speech.py
    275 →   275  =     core/turn_stages.py
    111 →   111  изм.  core/turn_tools.py
    113 →   113  =     core/voice_memory_init.py
    444 →   531  изм.  dialog_node.py
   4700 →  4700  =     dialogue_node.py
    469 →   469  =     rob_box_harness/rob_box_harness/core/dialogue_state_machine.py
    181 →   181  =     speaker_profiles.py
     41 →    14  изм.  startup_greeting.py
    297 →   297  =     utils/github_summary.py
Граф на 8178e2809 той же методикой: py-файлов 1712, в src 1333; рёбер 4059, из src 3721
Множество на 8178e2809: 22 файлов, 8089 строк; ушли: core/greeting_window.py, core/turn_context.py; пришли: —

### dialogue_node.py по AST: 112 методов DialogueNode, 4047 строк в методах; класс 396–4662; функций модуля 4; файл 4700 строк
  15b:   6 методов/функций,  154 строк
  15c:   4 методов/функций,  148 строк
  15d:  14 методов/функций,  207 строк
  15f:  14 методов/функций,  482 строк
  15g:  11 методов/функций,  318 строк
  15h:  10 методов/функций,  206 строк
  15i:  11 методов/функций,  208 строк
  15j:  18 методов/функций,  412 строк
  15k:   4 методов/функций,  465 строк
  15l:   4 методов/функций,  327 строк
  15n:  15 методов/функций,  509 строк
  15o:   4 методов/функций,  230 строк
  15p:   1 методов/функций,  484 строк
  не распределены по частям (0): 
  вне методов и функций: 550 строк

### Где dialogue_node.py использует имена из модулей v1 (часть: методы); импорт в шапке не считается
  core/dialogue_stage_mixin.py ← шапка: <модуль>
  core/identity_dedup.py ← 15p: __init__
  core/library_command.py ← только импорт / не используется в dialogue_node
  core/media_plan_run.py ← 15i: _execute_media_plan, _execute_play_named
  core/meeting.py ← 15b: _on_vision_event; 15o: _declare_params
  core/session_epoch.py ← 15j: _session_epoch_gate; 15l: _run_turn; 15p: __init__
  core/stop_chooser.py ← 15p: __init__
  core/stt_admission.py ← 15c: _finish_stt_admission, _on_stt, _resume_stt_after_media; 15p: __init__
  core/stt_admission_host.py ← 15c: _on_stt
  core/tars_status_feed.py ← 15p: __init__
  core/tier1_host.py ← 15j: _stop_music_by_owner
  core/tools_view_host.py ← 15k: _invoke_llm_with_telemetry
  core/turn_identity.py ← 15l: _build_dynamic_system_context
  core/turn_reflexes.py ← только импорт / не используется в dialogue_node
  core/turn_speech.py ← 15h: _publish_turn_text, _release_turn_speech; 15k: _handle_result; 15l: _run_turn
  core/turn_stages.py ← 15p: __init__
  core/turn_tools.py ← только импорт / не используется в dialogue_node
  speaker_profiles.py ← 15l: _handle_speaker_turn; 15p: __init__
  startup_greeting.py ← 15b: _on_startup_greeting_speak
  core/greeting_window.py ← только импорт / не используется в dialogue_node
  core/turn_context.py ← 15l: _build_dynamic_system_context
  core/speak_helpers.py ← 15h: _publish_response, _publish_response_batch, _publish_turn_text, _speak_direct; 15k: _handle_result; 15p: __init__
  core/dialogue_guards.py ← 15k: _handle_result
  core/dialogue_helpers.py ← 15d: _detect_pitch_intent, _generate_fallback_response, _map_emotion_to_animation; 15f: _apply_speaker_identity, _current_acquaintance; 15g: _tentative_identity_parts; 15h: fn:_clean_name; 15j: _maybe_log_skip_summary; 15l: _build_dynamic_system_context; 15n: _confident_speaker_name; шапка: <модуль>
  core/llm_skip_reasons.py ← 15p: __init__; шапка: <модуль>
  rob_box_harness/rob_box_harness/core/dialogue_state_machine.py ← 15b: _on_startup_greeting; 15c: _on_stt; 15d: _handle_silence; 15j: _apply_operator_pause, _apply_operator_resume, _finalize_turn_dsm, _publish_control_ack, _reset_dialogue_session; 15k: _invoke_llm_with_telemetry; 15p: __init__

### Цикл хода v1 в rob_box_harness: что осиротеет, если удалить ещё scripts/dialogue/{chat,local_tools}.py, agent_core.py (класс AgentCore) и DSM
    620  rob_box_harness/rob_box_harness/core/acceptance.py ← rob_box_harness/rob_box_harness/core/agent_core.py, rob_box_harness/rob_box_harness/core/tool_loop/confirmation.py; тестов 2
     70  rob_box_harness/rob_box_harness/core/tool_loop/confirmation.py ← rob_box_harness/rob_box_harness/core/agent_core.py; тестов 1
    102  rob_box_harness/rob_box_harness/core/tool_loop/outcomes.py ← rob_box_harness/rob_box_harness/core/agent_core.py, rob_box_harness/rob_box_harness/core/tool_loop/output_format.py; тестов 1
    117  rob_box_harness/rob_box_harness/core/tool_loop/output_format.py ← rob_box_harness/rob_box_harness/core/agent_core.py; тестов 1
    171  rob_box_harness/rob_box_harness/core/tool_loop/retry.py ← rob_box_harness/rob_box_harness/core/agent_core.py; тестов 1
  итого сирот: 5 файлов, 1080 строк
   2349  core/agent_core.py — читатели: core/dialog_v2_ports.py, dialogue_node.py, rob_box_harness/rob_box_harness/core/assembly.py, rob_box_harness/rob_box_harness/core/tool_loop/retry.py, scripts/dialogue/chat.py; тестов 31 (9615 строк)
    461  core/assembly.py — читатели: core/dialog_v2_ports.py, dialogue_node.py, rob_box_supervisor/rob_box_supervisor/operator_turns.py, rob_box_supervisor/rob_box_supervisor/supervisor_node.py; тестов 1 (501 строк)
    247  core/tool_loop/command_turn.py — читатели: core/dialog_v2_ports.py, rob_box_harness/rob_box_harness/core/agent_core.py; тестов 1 (346 строк)
     99  core/tool_loop/text_classify.py — читатели: dialogue_node.py, rob_box_harness/rob_box_harness/core/agent_core.py, rob_box_harness/rob_box_harness/core/tool_loop/retry.py, scripts/research/minimax_model_compat_probe.py; тестов 2 (677 строк)
    620  core/acceptance.py — читатели: rob_box_harness/rob_box_harness/core/agent_core.py, rob_box_harness/rob_box_harness/core/tool_loop/confirmation.py; тестов 2 (992 строк)
    465  core/confirmation_policy.py — читатели: rob_box_harness/rob_box_harness/core/acceptance.py, rob_box_harness/rob_box_harness/core/agent_core.py, rob_box_harness/rob_box_harness/decision/acceptance_escalation.py, rob_box_harness/rob_box_harness/decision/evaluation.py, scripts/research/jev_eval.py; тестов 6 (1679 строк)
  agent_core.py верхний уровень: PromptStats 88–107 (20); _raw_user_utterance 154–179 (26); _executed_actions_block 244–273 (30); _extract_track_name 399–424 (26); _extract_music_call_args 427–452 (26); DialogResult 461–559 (99); AgentCore 586–2154 (1569); _is_hallucinated_speak_text 2197–2242 (46); _suppressed_speak_text_result 2265–2284 (20); _recover_written_tool_calls 2287–2325 (39)

### Определения верхнего уровня (начало–конец, строк)
  core/stt_admission.py (840): parse_tg_prefix 59–109 (51); parse_speaker_event 112–143 (32); SttContext 151–200 (50); SttOutcomeKind 203–222 (20); SttOutcome 225–244 (20); drop 251–253 (3); handled 256–262 (7); SttAdmissionHost 270–392 (123); SttStep 400–425 (26); SttAdmission 428–536 (109); EmptyTextStep 544–553 (10); RejectedMarkerStep 556–573 (18); SilencedStateStep 576–587 (12); AddressStep 590–619 (30); _recent_phrases 622–628 (7); BareWakeStep 631–641 (11); Tier1Step 644–675 (32); CommandIntentGateStep 678–698 (21); MediaCommandStep 701–717 (17); TakeTurnStep 720–734 (15); DispatchTriggerStep 737–757 (21); default_steps 765–796 (32); DefaultSttAdmission 826–840 (15)
  core/turn_context.py (79): tts_state 22–35 (14); _tts_context 38–42 (5); _position 45–49 (5); _performance 52–56 (5); context_sections 59–76 (18)
  core/speak_helpers.py (594): strip_bracket_asides 123–142 (20); strip_history_marker 150–154 (5); strip_thinking_blocks 157–175 (19); strip_meta_markers 178–219 (42); strip_done_marker 222–254 (33); strip_markdown 284–309 (26); split_into_chunks 313–344 (32); provider_speaks 378–396 (19); unsupported_language_notice 399–430 (32); build_ssml_payload 433–492 (60); EffectAwaiterRegistry 495–581 (87)
  core/dialogue_helpers.py (142): map_emotion_to_animation 43–50 (8); generate_fallback_response 53–76 (24); detect_pitch_intent 79–90 (12); sanitize_speaker_name 93–111 (19); format_llm_skipped_summary 114–142 (29)
  core/dialogue_guards.py (169): is_system_template_regurgitated_in_ssml 94–111 (18); is_tool_call_markup 161–169 (9)
```

## Приложение Б. `bash docs/dialog/pr15_map_greps.sh origin/develop 8178e2809`

```text
REF=origin/develop d076b8b55  OLD=8178e2809 8178e2809
рабочее дерево = origin/develop по src scripts docker architecture tools tests .github

=== #3704 PR-10b (DSM) ===
$ ls src/rob_box_voice/rob_box_voice/core/dialogue_state_machine.py   # путь из карточки
ls: cannot access 'src/rob_box_voice/rob_box_voice/core/dialogue_state_machine.py': No such file or directory
$ ls src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py   # фактический путь
src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py
$ grep -rn '"/dialogue/control\|_pause_reason\|check_silence_timeout' src --include=*.py
  всего 22, вне тестов 10
      6 src/rob_box_voice/rob_box_voice/dialogue_node.py
      2 src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py
      2 src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py
  на 8178e2809: всего 22, вне тестов 10

=== #3705 PR-11 (маркеры памяти) ===
$ grep -rn "_HISTORY_MARKER_RE\|_SPEAKER_TAG_RE\|_META_PREFIX_RE\|discard_last_reply\|voice_memory.db" src --include=*.py
  всего 150, вне тестов 61
  _HISTORY_MARKER_RE: вне тестов 2
  _SPEAKER_TAG_RE: вне тестов 0
  _META_PREFIX_RE: вне тестов 2
  discard_last_reply: вне тестов 10
  voice_memory.db: вне тестов 47
  _HISTORY_MARKER_RE / _META_PREFIX_RE / discard_last_reply вне тестов:
src/rob_box_harness/rob_box_harness/core/agent_core.py:255
src/rob_box_harness/rob_box_harness/core/agent_core.py:318
src/rob_box_harness/rob_box_harness/core/agent_core.py:699
src/rob_box_harness/rob_box_harness/core/agent_core.py:1107
src/rob_box_harness/rob_box_harness/core/agent_core.py:1195
src/rob_box_harness/rob_box_harness/core/agent_core.py:2104
src/rob_box_voice/rob_box_voice/core/speak_helpers.py:34
src/rob_box_voice/rob_box_voice/core/speak_helpers.py:74
src/rob_box_voice/rob_box_voice/core/speak_helpers.py:154
src/rob_box_voice/rob_box_voice/core/speak_helpers.py:215
src/rob_box_voice/rob_box_voice/core/turn_speech.py:13
src/rob_box_voice/rob_box_voice/dialogue_node.py:3225
src/rob_box_voice/rob_box_voice/dialogue_node.py:3233
src/rob_box_voice/rob_box_voice/dialogue_node.py:3241
  voice_memory.db вне тестов по файлам:
     18 src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py
      5 src/rob_box_voice/rob_box_voice/speaker_id_node.py
      4 src/rob_box_voice/rob_box_voice/utils/legacy_voice_facts.py
      3 src/rob_box_mcp_tools/rob_box_mcp_tools/waypoint_adapter.py
      2 src/rob_box_voice/rob_box_voice/utils/identity_seam.py
      2 src/rob_box_voice/rob_box_voice/core/voice_memory_init.py
      2 src/rob_box_voice/rob_box_voice/core/voice_memory.py
      2 src/rob_box_music/rob_box_music/diversity.py
      2 src/rob_box_mcp_tools/rob_box_mcp_tools/core/rtttl_library.py
      2 src/rob_box_harness/rob_box_harness/memory/voice_memory_adapter.py
      1 src/rob_box_voice/rob_box_voice/dialog_node.py
      1 src/rob_box_voice/rob_box_voice/core/speaker_memory.py
      1 src/rob_box_music/rob_box_music/works.py
      1 src/rob_box_mcp_tools/rob_box_mcp_tools/waypoint_store.py
      1 src/rob_box_core/rob_box_core/e2e_mode.py
  на 8178e2809: всего 144, вне тестов 59

=== #3706 PR-12a (Telegram v1) ===
$ grep -rn "\[TG:\|tg_chat_id\|parse_tg_prefix" src --include=*.py
  всего 154, вне тестов 51
     19 src/rob_box_voice/rob_box_voice/core/stt_admission.py
     18 src/rob_box_voice/rob_box_voice/dialogue_node.py
      4 src/rob_box_voice/rob_box_voice/core/speak_helpers.py
      3 src/rob_box_voice/rob_box_voice/core/turn_identity.py
      2 src/rob_box_telegram/rob_box_telegram/telegram_node.py
      2 src/rob_box_dialog/rob_box_dialog/address.py
      1 src/rob_box_voice/rob_box_voice/core/tier1_host.py
      1 src/rob_box_voice/rob_box_voice/core/speech_compat.py
      1 src/rob_box_core/rob_box_core/utterance.py
  на 8178e2809: всего 154, вне тестов 51
$ grep -rn '"/voice/dialogue/response"\|"/voice/sound/stop"' src/rob_box_telegram
src/rob_box_telegram/rob_box_telegram/telegram_node.py:132:        return node.create_subscription(String, "/voice/dialogue/response", node._on_response, _RE,
src/rob_box_telegram/rob_box_telegram/telegram_node.py:279:        self._voice_stop_pub = self.create_publisher(String, "/voice/sound/stop", _RE)
src/rob_box_telegram/test/test_telegram_bridge.py:566:        sub = next(s for s in node._created_subscriptions if s.topic == "/voice/dialogue/response")
src/rob_box_telegram/test/test_telegram_bridge.py:572:        self.assertNotIn("/voice/dialogue/response", topics)
src/rob_box_telegram/test/test_telegram_bridge.py:581:        self.assertNotIn("/voice/dialogue/response", [s.topic for s in node._created_subscriptions])

=== #3711 критерии ===
$ grep -rn "Bug \[A-F\]" src/rob_box_voice src/rob_box_dialog --include=*.py | wc -l   # как в карточке
  0   <- в BRE «\[» — литерал «[»: команда ищет текст «Bug [A-F]» и всегда даёт 0
$ grep -rnE "Bug [A-F]\b" src/rob_box_voice src/rob_box_dialog --include=*.py   # по смыслу: Bug A … Bug F
  всего 26, вне тестов 8
      4 src/rob_box_voice/rob_box_voice/dialogue_node.py
      1 src/rob_box_voice/rob_box_voice/core/turn_speech.py
      1 src/rob_box_voice/rob_box_voice/core/session_epoch.py
      1 src/rob_box_voice/rob_box_voice/core/media_command_grammar.py
      1 src/rob_box_voice/rob_box_voice/core/dialogue_guards.py
  на 8178e2809: всего 26, вне тестов 8
$ grep -rc "#[0-9]\{4\}" src/rob_box_dialog/rob_box_dialog | awk -F: '{s+=$2} END {print s}'
  строк: 369
  вхождений (grep -o): 401
  git grep -c (строки): origin/develop 369, 8178e2809 158
$ grep -rlE "rob_box_voice\.dialogue_node|from rob_box_voice import dialogue_node|dialogue_node import" src tests --include=*.py   # тесты ноды по строке
  тестовых файлов: 54 (по AST-импорту — 47, приложение А; разница — patch("…")/sys.modules)
$ grep -c "RULE #" src/rob_box_voice/prompts/*.txt
src/rob_box_voice/prompts/master_prompt_compact.txt:8
src/rob_box_voice/prompts/telegram_chat_prompt.txt:7
$ grep -c 're.compile' speak_helpers.py dialogue_guards.py
src/rob_box_voice/rob_box_voice/core/speak_helpers.py:18
src/rob_box_voice/rob_box_voice/core/dialogue_guards.py:2

=== String-топики v1: файлы вне тестов и docs (src docker scripts architecture tools .github) ===
--- /voice/dialogue/state: строк 23, файлов 13
      3 src/rob_box_voice/rob_box_voice/led_node.py
      3 src/rob_box_supervisor/rob_box_supervisor/arbiter_node.py
      3 src/rob_box_quest/rob_box_quest/quest_node.py
      3 src/rob_box_core/rob_box_core/voice_state.py
      2 src/rob_box_supervisor/rob_box_supervisor/core/aggregator.py
      2 scripts/agent_flow/create_avatar_issues.sh
      1 tools/led_face_lab/preview_template.html
      1 src/rob_box_voice/rob_box_voice/dialogue_node.py
      1 src/rob_box_supervisor/rob_box_supervisor/core/__init__.py
      1 src/rob_box_quest/webxr_client/src/dev/mock_robot.ts
      1 src/rob_box_harness/rob_box_harness/core/agent_core.py
      1 src/rob_box_dialog_msgs/msg/SessionState.msg
      1 src/rob_box_core/rob_box_core/bridge_protocol.py
--- /voice/dialogue/response: строк 46, файлов 26
      5 src/rob_box_voice/rob_box_voice/tts_node.py
      4 src/rob_box_voice/docs/PHASE2_IMPLEMENTATION.md
      4 src/rob_box_telegram/rob_box_telegram/telegram_node.py
      3 src/rob_box_voice/rob_box_voice/dialogue_node.py
      3 src/rob_box_voice/rob_box_voice/core/speech_compat.py
      3 src/rob_box_perception/rob_box_perception/context_aggregator_node.py
      3 src/rob_box_perception/README.md
      2 src/rob_box_voice/docs/PHASE3_STT_IMPLEMENTATION.md
      2 src/rob_box_harness/rob_box_harness/core/agent_core.py
      1 tools/gui/robot_control_gui_simple.py
      1 tools/gui/robot_control_gui.py
      1 tools/gui/README_GUI.md
      1 src/rob_box_voice/scripts/example_voice_usage.py
      1 src/rob_box_voice/rob_box_voice/core/speak_helpers.py
      1 src/rob_box_voice/docs/VOICE_OPTIMIZATION.md
      1 src/rob_box_voice/docs/ARCHITECTURE_OVERVIEW.md
      1 src/rob_box_voice/config/dialog_engine.yaml
      1 src/rob_box_perception/rob_box_perception/core/robot_speech.py
      1 src/rob_box_dialog_msgs/msg/SpeechRequest.msg
      1 src/rob_box_dialog_msgs/msg/DialogOutput.msg
      1 src/rob_box_core/rob_box_core/dialog_engine.py
      1 docker/sound/config/voice_assistant/dialog_engine.yaml
      1 architecture/runtime-baseline.json
      1 architecture/ownership.yml
      1 architecture/minimax-provider.md
      1 .github/workflows/scripts/e2e_voice_test.sh
--- /voice/tts/state: строк 16, файлов 12
      3 src/rob_box_animations/rob_box_animations/face_node.py
      2 tools/led_face_lab/preview_template.html
      2 src/rob_box_voice/rob_box_voice/audio_node.py
      1 src/rob_box_voice/rob_box_voice/tts_node.py
      1 src/rob_box_voice/rob_box_voice/stt_node.py
      1 src/rob_box_voice/rob_box_voice/core/speech_compat.py
      1 src/rob_box_dialog_msgs/msg/SpeechEvent.msg
      1 src/rob_box_animations/rob_box_animations/face_director.py
      1 src/rob_box_animations/README.md
      1 scripts/maintenance/scene_set/replay.py
      1 docker/vision/config/led_matrix/face_node.yaml
      1 architecture/minimax-provider.md
--- /voice/tts/finished: строк 39, файлов 16
     12 src/rob_box_voice/rob_box_voice/tts_node.py
      4 src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py
      3 src/rob_box_voice/rob_box_voice/core/speak_helpers.py
      2 src/rob_box_voice/rob_box_voice/speaker_id_node.py
      2 src/rob_box_voice/rob_box_voice/dialogue_node.py
      2 src/rob_box_quest/rob_box_quest/tars_stage_relay.py
      2 src/rob_box_quest/rob_box_quest/quest_node.py
      2 src/rob_box_quest/rob_box_quest/core/tars_stages.py
      2 src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py
      2 scripts/maintenance/scene_set/conductor.py
      1 src/rob_box_voice/rob_box_voice/core/speech_owner.py
      1 src/rob_box_voice/rob_box_voice/core/speech_compat.py
      1 src/rob_box_supervisor/rob_box_supervisor/tars_stage.py
      1 src/rob_box_quest/rob_box_quest/server/ws_server.py
      1 scripts/maintenance/scene_set/replay.py
      1 architecture/minimax-provider.md
--- /voice/tts/request: строк 64, файлов 23
     18 src/rob_box_voice/rob_box_voice/tts_node.py
      5 scripts/diagnostics/watchdog-tts.sh
      4 src/rob_box_perception/README.md
      4 src/rob_box_core/rob_box_core/utterance.py
      4 scripts/lint/test_ownership_check.py
      3 src/rob_box_voice/rob_box_voice/tts_params.py
      3 src/rob_box_voice/rob_box_voice/core/speech_compat.py
      2 tools/gui/robot_control_gui_simple.py
      2 tools/gui/robot_control_gui.py
      2 tools/gui/README_GUI.md
      2 tools/audio_capture_harness/audio_capture_harness.py
      2 tools/audio_capture_harness/README.md
      2 src/rob_box_voice/rob_box_voice/dialogue_node.py
      2 src/rob_box_teleop/rob_box_teleop/joystick_logic.py
      1 src/rob_box_teleop/rob_box_teleop/joystick_control_node.py
      1 src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py
      1 src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py
      1 src/rob_box_dialog_msgs/msg/SpeechRequest.msg
      1 src/rob_box_core/rob_box_core/speech_client.py
      1 scripts/maintenance/scene_set/conductor.py
      1 scripts/lint/ownership_check.py
      1 architecture/runtime-baseline.json
      1 architecture/ownership.yml
--- /voice/tts/control: строк 7, файлов 4
      3 src/rob_box_voice/rob_box_voice/core/speech_compat.py
      2 src/rob_box_voice/rob_box_voice/tts_node.py
      1 src/rob_box_dialog_msgs/msg/SpeechCancel.msg
      1 architecture/runtime-baseline.json
--- /dialogue/control: строк 49, файлов 12
     28 src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py
     11 src/rob_box_voice/rob_box_voice/dialogue_node.py
      1 src/rob_box_voice/config/dialogue_node.yaml
      1 src/rob_box_voice/config/dialog_engine.yaml
      1 src/rob_box_quest/webxr_client/src/wire/voice_pipeline_cmd.ts
      1 src/rob_box_quest/webxr_client/src/main.ts
      1 src/rob_box_dialog_msgs/srv/DialogControl.srv
      1 src/rob_box_dialog_msgs/msg/TurnEvent.msg
      1 scripts/lint/test_ownership_check.py
      1 scripts/lint/seam_baseline.json
      1 docker/sound/config/voice_assistant/dialog_engine.yaml
      1 architecture/runtime-baseline.json

=== флаг dialog_engine (вне тестов, без rob_box_core/dialog_engine.py) ===
  строк 75, файлов 27
     10 src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py
      7 scripts/robot_dev/dialog_engine.sh
      5 src/rob_box_supervisor/rob_box_supervisor/operator_prompt.py
      5 .github/workflows/scripts/e2e_voice_test.sh
      4 src/rob_box_voice/rob_box_voice/dialog_node.py
      4 src/rob_box_voice/rob_box_voice/core/speech_compat.py
      4 src/rob_box_voice/config/dialog_engine.yaml
      4 docker/sound/config/voice_assistant/dialog_engine.yaml
      3 src/rob_box_voice/rob_box_voice/tts_node.py
      3 src/rob_box_voice/launch/voice_assistant.launch.py
      3 src/rob_box_supervisor/rob_box_supervisor/operator_turns.py
      3 src/rob_box_perception/rob_box_perception/utils/node_monitor.py
      2 src/rob_box_voice/config/dialog_node.yaml
      2 src/rob_box_telegram/rob_box_telegram/telegram_node.py
      2 src/rob_box_mcp_tools/rob_box_mcp_tools/tools/operator_admin.py
      2 docker/sound/config/voice_assistant/voice_assistant_headless.launch.py
      2 docker/sound/config/voice_assistant/dialog_node.yaml
      1 src/rob_box_voice/setup.py
      1 src/rob_box_voice/rob_box_voice/led_node.py
      1 src/rob_box_supervisor/rob_box_supervisor/arbiter_node.py
      1 src/rob_box_quest/rob_box_quest/quest_node.py
      1 src/rob_box_perception/rob_box_perception/core/robot_speech.py
      1 src/rob_box_perception/rob_box_perception/context_aggregator_node.py
      1 src/rob_box_core/rob_box_core/voice_state.py
      1 docker/sound/scripts/telegram_bot/start_telegram_bot.sh
      1 docker/sound/scripts/supervisor/start_supervisor.sh
      1 .github/workflows/scripts/e2e_events.py
  на 8178e2809: строк 52, файлов 19

=== зависимости вне src (B6) ===
.github/workflows/G-Lint Code.yml:72:          PYTHONPATH=src/rob_box_voice:src/rob_box_dialog python scripts/lint/dialogue_skip_reasons.py
scripts/lint/dialogue_skip_reasons.py:9:from rob_box_voice.core.skip_reason_checker import find_unknown_skip_reasons
scripts/dialogue/chat.py:53:    "rob_box_dialog",  # knowledge table (wake words, silence) for dialogue_text and the harness DSM
scripts/dialogue/chat.py:56:    "rob_box_voice",  # only pure helpers (dialogue_text); never the node
scripts/dialogue/chat.py:451:        from rob_box_harness.core.dialogue_state_machine import DialogueStateMachine
scripts/dialogue/chat.py:562:        from rob_box_harness.core.dialogue_state_machine import (
scripts/dialogue/chat.py:566:        from rob_box_voice.core.dialogue_text import (
scripts/dialogue/chat.py:571:        from rob_box_voice.core.speak_helpers import strip_done_marker, strip_markdown
scripts/dialogue/chat.py:633:        from rob_box_harness.core.dialogue_state_machine import DialogueStateKind
scripts/music/live_dj/router_offline.py:23:from rob_box_voice.core.dialogue_text import strip_wake_word
--- имя ROS-ноды /dialogue_node вне src (не путь к файлу):
.github/e2e/scenarios/turn_occasion_meeting_v1.json:23:      "ros2 node info /dialogue_node показывает подписку на /vision/hailo/events",
.github/workflows/scripts/e2e_voice_test.sh:644:#      «новая сессия», но молча — ros2 param set /dialogue_node
.github/workflows/scripts/e2e_voice_test.sh:676:    robot_ros "ros2 param get /dialogue_node e2e_session_reset_token --no-daemon" 2>/dev/null \
.github/workflows/scripts/e2e_voice_test.sh:681:    robot_ros "ros2 param set /dialogue_node e2e_session_reset_token $token --no-daemon" >/dev/null 2>&1
.github/workflows/scripts/e2e_voice_test.sh:689:    echo "           проверь вручную: ssh <robot> \"docker exec voice-assistant bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 param get /dialogue_node e2e_session_reset_token --no-daemon'\"" >&2
docker/vision/test/config/voice_assistant_test.yaml:6:/dialogue_node:
scripts/lint/seam_allowlist.json:17:    "/voice/music/event": "ADR-0149 §2.3, PR-4 (#3312): события владельца плеера v2 (started/rejected), публикуются владельцем плеера v2. Подписчики — SetSession/dialogue_node в PR-5/PR-6 той же миграции (ADR-0149 §9); до них наблюдатель — живая проверка `ros2 topic echo /voice/music/event`. Запись удалить в PR, где появится подписчик.",
scripts/robot_dev/dialog_engine.sh:5:#   scripts/robot_dev/dialog_engine.sh status  # только показать значение и строки dialog_node/dialogue_node из логов
--- строковое имя dialogue_node в живом v2 (подпись /mcp/execute и срез тулов):
src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_auth.py:85:PERSONALITY_SENDER = "dialogue_node"
src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_auth.py:90:DEFAULT_ALLOWED_SENDERS = frozenset({PERSONALITY_SENDER, TELEGRAM_SENDER, "avatar_supervisor", "harness"})
src/rob_box_voice/rob_box_voice/dialog_node.py:211:    from rob_box_mcp_tools.mcp_auth import PERSONALITY_SENDER  # noqa: PLC0415
src/rob_box_voice/rob_box_voice/dialog_node.py:213:    return plan_host.make_tool_executor(node, _mcp_provider(node, PERSONALITY_SENDER), lambda *_: None)
src/rob_box_mcp_tools/rob_box_mcp_tools/data/slice_policy.yaml:4:# (``dialogue_node``, ``avatar_supervisor``, ``harness``) перечисляет,
src/rob_box_mcp_tools/rob_box_mcp_tools/data/slice_policy.yaml:24:  # ЛLM-личность (dialogue_node): всё, что умеет робот в обычном
src/rob_box_mcp_tools/rob_box_mcp_tools/data/slice_policy.yaml:26:  dialogue_node:
src/rob_box_mcp_tools/rob_box_mcp_tools/data/slice_policy.yaml:117:  # срезах. Sender'у ``dialogue_node`` это всё равно доступно через
--- v1 собирает допуск реплики из хвоста stt_admission (DefaultSttAdmission):
src/rob_box_voice/rob_box_voice/dialogue_node.py:127:    DefaultSttAdmission,
src/rob_box_voice/rob_box_voice/dialogue_node.py:457:        self._stt_admission: SttAdmission = DefaultSttAdmission(
src/rob_box_voice/rob_box_voice/dialogue_node.py:2034:        # ``self._stt_admission = DefaultSttAdmission(...)``.
src/rob_box_voice/rob_box_voice/dialogue_node.py:2037:            admission = DefaultSttAdmission(logger=self.get_logger())

=== /mcp/execute: где создаются издатели (для находки duplicate_publishers) ===
src/rob_box_mcp_tools/rob_box_mcp_tools/llm_adapter.py:151:        self.execute_pub = node.create_publisher(String, "/mcp/execute", qos_profile)
src/rob_box_voice/rob_box_voice/dialog_node.py:191:def _mcp_provider(node: Any, sender: str, for_telegram: bool = False) -> Any:
src/rob_box_voice/rob_box_voice/dialog_node.py:198:    adapter = LLMToolCallAdapter(node, sender=sender, turn_context=tool_turn_context)
src/rob_box_voice/rob_box_voice/dialog_node.py:213:    return plan_host.make_tool_executor(node, _mcp_provider(node, PERSONALITY_SENDER), lambda *_: None)
src/rob_box_voice/rob_box_voice/dialog_node.py:222:    return PlanToolExecutor(_mcp_provider(node, TELEGRAM_SENDER, True), personality_tools.planner)

=== коммиты между картами ===
  все коммиты 8178e2809..origin/develop: 208
  в src/rob_box_voice src/rob_box_dialog src/rob_box_harness src/rob_box_core: 58
  только в файлах карты (rob_box_voice/rob_box_voice, rob_box_dialog/rob_box_dialog, harness/core): 48
```

## Приложение В. Рантайм-аудит run 40499068353: новые записи, `multiple_writers`, `resolved`

Команды: `gh run view 40499068353 --log` (сводки шагов) и `gh run download 40499068353 -n architecture-local-runtime`
(файл `runtime-findings.json`); выборка — Python по полям `new`, `type`, `resolved`.

```text
captured_from=remote:MainPi+VisionPi+SoundPi baseline_used=True
summary {"findings": 126, "repo_scope": 47, "new": 20, "resolved_since_baseline": 20, "type_mismatch": 0, "qos_incompatible": 0, "dead_input": 30, "dead_output": 80, "multiple_writers": 14, "duplicate_publishers": 2}

# новые (new=true), все типы:
NEW  dead_input           /face_node/scene  {"subscribers": ["/face_node"]}
NEW  dead_input           /voice/dialogue/response  {"subscribers": ["/context_aggregator"]}
NEW  dead_input           /voice/dialogue/state  {"subscribers": ["/avatar_arbiter", "/led_node", "/quest_node"]}
NEW  dead_input           /voice/speaker/epithet  {"subscribers": ["/speaker_id_node"]}
NEW  dead_input           /voice/speaker/merge  {"subscribers": ["/speaker_id_node"]}
NEW  dead_input           /voice/speaker/observe  {"subscribers": ["/speaker_id_node"]}
NEW  dead_input           /voice/tts/control  {"subscribers": ["/tts_node"]}
NEW  dead_output          /mcp/tools  {"publishers": ["/mcp_server"]}
NEW  dead_output          /perception/observations  {"publishers": ["/vision_face", "/vision_hailo"]}
NEW  dead_output          /voice/expression/event  {"publishers": ["/face_node", "/led_node", "/sound_node"]}
NEW  dead_output          /voice/sound/state  {"publishers": ["/sound_node"]}
NEW  dead_output          /voice/speaker/epithet_request  {"publishers": ["/speaker_id_node"]}
NEW  dead_output          /voice/stt/speaker  {"publishers": ["/stt_node"]}
NEW  multiple_writers     /perception/observations  {"publishers": ["/vision_face", "/vision_hailo"]}
NEW  multiple_writers     /voice/expression/event  {"publishers": ["/face_node", "/led_node", "/sound_node"]}
NEW  multiple_writers     /voice/providers/state  {"publishers": ["/dialog_node", "/mcp_server", "/stt_node", "/tts_node"]}
NEW  multiple_writers     /voice/speech/request  {"publishers": ["/avatar_supervisor", "/dialog_node", "/mcp_server", "/stt_node", "/telegram_node"]}
NEW  multiple_writers     /voice/stt/utterance  {"publishers": ["/stt_node", "/telegram_node"]}
NEW  multiple_writers     /voice/tts/voice_request  {"publishers": ["/dialog_node", "/mcp_server"]}
NEW  duplicate_publishers /mcp/execute /dialog_node {"node": "/dialog_node", "publishers": 2}

# multiple_writers и duplicate_publishers из baseline (не новые):
base multiple_writers     /avatar/preview_voice/audio  {"publishers": ["/avatar_supervisor", "/tts_node"]}
base multiple_writers     /avatar/preview_voice/error  {"publishers": ["/avatar_supervisor", "/tts_node"]}
base multiple_writers     /avatar/preview_voice/result  {"publishers": ["/avatar_supervisor", "/tts_node"]}
base multiple_writers     /avatar/tars/panel_request  {"publishers": ["/avatar_supervisor", "/mcp_server"]}
base multiple_writers     /cmd_vel  {"publishers": ["/behavior_server", "/controller_server"]}
base multiple_writers     /vision/hailo/events  {"publishers": ["/vision_face", "/vision_hailo"]}
base multiple_writers     /voice/sound/stop  {"publishers": ["/quest_node", "/telegram_node"]}
base multiple_writers     /voice/sound/trigger  {"publishers": ["/health_monitor", "/mcp_server"]}
base duplicate_publishers /cmd_vel /behavior_server {"node": "/behavior_server", "publishers": 4}

# resolved_since_baseline:
  dead_input|/voice/current_dialogue_id|
  dead_input|/voice/state|
  dead_input|/voice_animation_player/load_animation|
  dead_output|/camera/camera/depth/image_rect_raw|
  dead_output|/dialogue/control_ack|
  dead_output|/tts/speak|
  dead_output|/voice/stt/request|
  dead_output|/voice_animation_player/status|
  duplicate_publishers|/panel_image|/voice_animation_player
  duplicate_publishers|/voice/animation/request|/mcp_server
  duplicate_publishers|/voice/generated_music/state|/mcp_server
  duplicate_publishers|/voice/sound/stop|/mcp_server
  duplicate_publishers|/voice/tts/set_provider|/mcp_server
  multiple_writers|/avatar/command|
  multiple_writers|/voice/dialogue/response|
  multiple_writers|/voice/dj_mode|
  multiple_writers|/voice/generated_music/state|
  multiple_writers|/voice/speaker/register|
  multiple_writers|/voice/tts/control|
  multiple_writers|/voice/tts/request|

# лог шага «Render runtime architecture graph» (gh run view 40499068353 --log):
{"connected_topics": 92, "container_edges": 54, "containers": 18, "hidden_helper_nodes": 12, "hidden_infra_topics": 19, "runtime_nodes": 52, "shown_nodes": 43, "unplaced_nodes": ["dialog_node"], "unresolved_class_nodes": 23}
```
