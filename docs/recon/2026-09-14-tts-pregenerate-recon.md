# Recon: tts_node + scheduler/ перед реализацией `pregenerate`

Card: t_9ab8ab70 · Дата: 2026-09-14 · Коммит: ce60bb48 · Scope: разведка,
без правок кода. Acceptance — карта `файл:строка` для 5 пунктов карточки +
явное подтверждение блокера §4.8 хендоффа.

---

## §4.8 «pregenerate не существует» — статус СНЯТ

В хендоффе от 2026-09-05 (`docs/plans/2026-09-05-operator-agent-architecture-handoff.md:145-149`)
зафиксировано: `pregenerate` встречается один раз — в комментарии к
`config/tts_node.yaml:8`; в 4198 строках `tts_node.py` реализации нет.

По текущему состоянию ветки (commit ce60bb48) реализация есть.
Картину ниже даём самим recon-ом — каждый пункт списка — публичный
API-метод ноды или модуль пакета pregen, а не ссылки в комментариях.

| Артефакт | Файл:строка | Статус |
|---|---|---|
| `TTSNode.pregenerate()` | `src/rob_box_voice/rob_box_voice/tts_node.py:3754` | **живой** (ADR-0056 §3.2) |
| `TTSNode.claim_pregen()` | `src/rob_box_voice/rob_box_voice/tts_node.py:3837` | **живой** |
| `TTSNode.cancel_pregen()` | `src/rob_box_voice/rob_box_voice/tts_node.py:3862` | **живой** |
| `TTSNode._ensure_prefetch()` | `src/rob_box_voice/rob_box_voice/tts_node.py:3635` | **lazy-init** |
| ROS-params `pregenerate_*` | `src/rob_box_voice/rob_box_voice/tts_node.py:1678-1687` | **живые** |
| `_PrefetchEngine` (dict) | `src/rob_box_voice/rob_box_voice/tts_node.py:1674-1675` | **lazy** |

Расхождение документации с кодом:

* `docs/architecture/target-operator-agent-and-dialogue.md:1139` пишет «✅
  сделано — `scheduler/pregen/*` подключён в `tts_node`,
  `pregenerate_enabled=True` (ADR-0056)»;
* но `§8а.4` (строки 835-839) и `SCHEDULER_DESIGN.md:38-40` повторяют
  старую формулировку «pregenerate встречается ровно один раз в
  комментарии YAML».

Карточка t_9ab8ab70 не пишет новой фичи — она фиксирует текущее
состояние. Если следующая карточка будет добавлять что-то рядом
с `pregenerate`, она должна явно ссылаться на эти 6 строк и не
повторять «не существует».

---

## 1. Документация (rele­vant куски, не полные файлы)

| Раздел | Файл:строка | Содержание |
|---|---|---|
| §8a.4 «Порядок включения» | `docs/architecture/target-operator-agent-and-dialogue.md:827-839` | Шаг 3 явно говорит «ключ pregenerate встречается ровно один раз, в комментарии к config/tts_node.yaml:8». Это — формулировка 2026-09-05, снимается §4.8. |
| §13 «Шаг 13. Спекулятивная генерация» (таблица) | `docs/architecture/target-operator-agent-and-dialogue.md:1139` | «✅ сделано» — но ниже, §8a.4, противоречит. |
| Карточка-acceptance | `docs/architecture/target-operator-agent-and-dialogue.md:1223-1225` | «Требует реализации `pregenerate` в `tts_node` — сегодня это только комментарий в YAML. Последним.» |
| §0.0 «СТАТУС НА 2026-09-05» | `docs/design/SCHEDULER_DESIGN.md:14-46` | Таблица статусов модулей scheduler. Про pregen/* в ней НЕ упоминается — таблица по 7 модулям scheduler/, преген сидит в подпакете pregen/. |
| §6.5 «Speculative pre-gen (TTS)» | `docs/design/SCHEDULER_DESIGN.md:975-980` | Архитектурное описание pre-gen: момент старта, MERGE-отмена через `InterruptibleTask.cancel()`. |
| §11.6 «Статус реализации на develop» | `docs/design/SCHEDULER_DESIGN.md:1663+` (фаза 3) | «эстиматоры + speculative pre-generation» — план, не отчёт. |

---

## 2. Структура `tts_node.py` (6808 LOC, не 4198)

| Точка интереса | Файл:строка |
|---|---|
| `class TTSNode(Node)` | `src/rob_box_voice/rob_box_voice/tts_node.py:884` |
| `__init__` | `src/rob_box_voice/rob_box_voice/tts_node.py:887` |
| **Формирование чанков** (Yandex/MiniMax/Silero с retry-halve) | `src/rob_box_voice/rob_box_voice/tts_node.py:5526` (`_chunk_text`), `5566` (`_synthesize_yandex`), `4666` (`_sap_synthesize_minimax`), `5423` (`_synthesize_silero`) |
| **Чанки-в-payload** (`batch_id/batch_index/batch_total`) | принимаются в `dialogue_callback` и распознаются на `src/rob_box_voice/rob_box_voice/tts_node.py:2543-2544` |
| **`/voice/tts/batch_complete` publisher** | `src/rob_box_voice/rob_box_voice/tts_node.py:1614-1619` |
| **Условие «последний чанк batch'а» для batch_complete** | `src/rob_box_voice/rob_box_voice/tts_node.py:2538-2542` (комментарий «single event per multi-chunk TTS batch, rap, poetry») |
| **FIFO-порядок воспроизведения (play_seq)** | `src/rob_box_voice/rob_box_voice/tts_node.py:1645-1647` (`_play_seq_counter`, `_next_play_seq`, `_play_order_cond`) |
| **Приоритетная очередь перед динамиком (issue #1996, шаг 7a)** | `src/rob_box_voice/rob_box_voice/tts_node.py:1649-1659` (`_pending_seqs`, `_play_active_seq`) |
| **`_assign_priority_play_seq`** | `src/rob_box_voice/rob_box_voice/tts_node.py:3432` |
| **`_resolve_operator_priority_slot`** (каскад сдвига) | `src/rob_box_voice/rob_box_voice/tts_node.py:3477` |
| **`_cancel_pregen_for_priority_replace`** | `src/rob_box_voice/rob_box_voice/tts_node.py:3494` |
| **`_submit_synthesis`** | `src/rob_box_voice/rob_box_voice/tts_node.py:3336` |
| **`dialogue_callback`** (точка входа `/voice/tts/request` JSON) | `src/rob_box_voice/rob_box_voice/tts_node.py:2364` |
| **`_on_synthesis_done`** | `src/rob_box_voice/rob_box_voice/tts_node.py:3412` |
| **Speculative exec import** | `src/rob_box_voice/rob_box_voice/tts_node.py:106-119` |
| **`_publish_tts_finished`** с batch_complete-флагом | `src/rob_box_voice/rob_box_voice/tts_node.py:2538-2542` (условие), см. также 1614-1619 (topic registration) |

---

## 3. `scheduler/` — что реально объявлено и подключено

Реальная файловая раскладка (по `wc -l src/.../scheduler/*.py`):

| Модуль | LOC | Статус на 2026-09-14 |
|---|---|---|
| `scheduler/delta.py` | 109 | живой (data) |
| `scheduler/event_bus.py` | 45 | **EventEnvelope** жив, самой шины нет (ADR-0086) |
| `scheduler/quick_decide.py` | 127 | живой — импорт `dialogue_node.py:102` |
| `scheduler/task_scheduler.py` | 1312 | живой, фаза 1 + 1.5 (cancel), MVP |
| `scheduler/tool_executor.py` | 536 | живой — ленивый импорт `dialogue_node.py:1920` |
| `scheduler/pregen/__init__.py` | 101 | живой (public re-exports) |
| `scheduler/pregen/pre_gen.py` | 238 | живой — `build_pregen_task` |
| `scheduler/pregen/estimator.py` | 217 | живой — `estimate_confidence`, CONFIDENCE_FLOOR |
| `scheduler/pregen/quality.py` | 192 | живой — `check_audio_quality`, три эвристики |
| `scheduler/pregen/decision.py` | 82 | живой — `decide` (ACCEPT iff pass+conf) |
| `scheduler/pregen/speculative_executor.py` | 565 | живой — orchestrator + `PreGenMetrics` |

**Что было «заглушкой» в SCHEDULER_DESIGN.md:28** (т.н. scheduler-level
`pre_gen` / `speculative_executor` / `decision` / `estimator` /
`quality` на верхнем уровне пакета) — **удалено**. Подтверждение:

* `scheduler/__init__.py:17-25` (`README` внутри): «The older
  scheduler.{pre_gen,speculative_executor,decision,estimator,quality,
  reflex} modules were removed because they were never wired to a
  live caller — see ADR-0080 §2.8 and ADR-0086».
* ADR-0086 — `docs/adr/0086-eventbus-reflexlayer-delete-not-wire.md`.
* Текущий `wc -l` показывает только `pregen/`-папку.

⇒ Заменять ничего не нужно. Подключение пред-генерации уже идёт
через `scheduler/pregen/*`, а не через плоский scheduler.

| Точка интереса | Файл:строка |
|---|---|
| Пакетная документация | `src/rob_box_voice/rob_box_voice/scheduler/__init__.py:1-34` |
| `scheduler/README.md` (для разработчиков пакета) | `src/rob_box_voice/rob_box_voice/scheduler/README.md:1` |
| `TaskScheduler` фасад | `src/rob_box_voice/rob_box_voice/scheduler/task_scheduler.py` |
| `SchedulerToolExecutor` | `src/rob_box_voice/rob_box_voice/scheduler/tool_executor.py` |
| `SpeculativeExecutor` (живой) | `src/rob_box_voice/rob_box_voice/scheduler/pregen/speculative_executor.py:1` |
| `pregen/` API | `src/rob_box_voice/rob_box_voice/scheduler/pregen/__init__.py:25-46` |

---

## 4. `docker/vision/config/voice_assistant/tts_node.yaml:8` — текущая роль ключа `pregenerate`

* Конкретный файл **не содержит** ключа `pregenerate` —
  `grep pregenerate docker/vision/config/voice_assistant/tts_node.yaml`
  пуст.
* Соседняя (downstream не используется) копия `src/rob_box_voice/config/tts_node.yaml:8` —
  упоминание в **комментарии**: «Ключи cache_dir/cache_enabled/
  **pregenerate**/prosody_pitch и вложенный yandex:{...} удалены —
  код их НЕ читает; YAML молча игнорировался, issue #1004». То есть
  это **исторический артефакт** описания удалённого кэша, а не
  активная конфигурация pre-gen.
* Активная конфигурация pre-gen — три параметра, зашитых в
  `declare_parameter(...)` и читаемых в `__init__`:
  * `pregenerate_enabled` (bool, kill-switch, строка 1678),
  * `pregenerate_confidence_floor` (float [0..1], 1682),
  * `pregenerate_history_window` (int ≥1, 1685).
* В `docker/vision/config/voice_assistant/tts_node.yaml` этих трёх
  параметров ещё нет, хотя контейнер стартует с этим оверлеем.
  Это — потенциальная дыра для следующей карточки: если она
  захочет дефолтно включать pre-gen, YAML надо синхронизировать с
  `declare_parameter(...)` (см. пункт «что подключить и где»).

---

## 5. Смежные тесты

| Зона | Файл | Что проверяет |
|---|---|---|
| `pregen/` пакет | `src/rob_box_voice/test/unit/pregen/test_pregen_decision.py` | ACCEPT iff verdict+PASS+conf |
| | `src/rob_box_voice/test/unit/pregen/test_pregen_estimator.py` | confidence-эвристика |
| | `src/rob_box_voice/test/unit/pregen/test_pregen_quality.py` | rms_low/duration_ratio/silence_ratio |
| | `src/rob_box_voice/test/unit/pregen/test_pregen_pre_gen.py` | `build_pregen_task` |
| | `src/rob_box_voice/test/unit/pregen/test_pregen_speculative_executor.py` | lifecycle `SpeculativeExecutor` |
| | `src/rob_box_voice/test/unit/pregen/test_pregen_tts_integration.py` | интеграция c нодой |
| | `src/rob_box_voice/test/unit/pregen/test_speculative_path_latency.py` | latency chunk-to-chunk |
| `tts_node` core | `src/rob_box_voice/test/unit/tts/test_tts_priority_queue.py` | FIFO-gate + `_assign_priority_play_seq` |
| | `src/rob_box_voice/test/unit/tts/test_bounded_fanout.py` | пул синтеза, sem-slot |
| | `src/rob_box_voice/test/unit/tts/test_chunking_module.py` | retry-halve (issue #933) |
| | `src/rob_box_voice/test/unit/tts/test_issue_1563_stop_immune.py` | IMMUNE-окно (issue #1563) |
| | `src/rob_box_voice/test/unit/tts/test_issue_1562_late_stop_after_synth.py` | STOP между synth/play |
| `tts_node` batch | `src/rob_box_voice/test/test_tts_batch_complete.py` | issue #980, batch_id/index/total + 1×/batch batch_complete |
| `dialogue↔tts` | `src/rob_box_voice/test/test_dialogue_tts_sync.py` | dialogue_id roundtrip |
| | `src/rob_box_voice/test/test_tts_node.py` | нода крупным планом |
| `scheduler` core | `src/rob_box_voice/test/test_task_scheduler.py` | FIFO per channel, ETA snapshot |
| | `src/rob_box_voice/test/test_task_scheduler_integration.py` | integration: 2×speak_text → stop_music |
| | `src/rob_box_voice/test/test_task_scheduler_eventbus.py` | (после удаления шины — лишь проверяет удаление) |
| | `src/rob_box_voice/test/test_scheduler_delta.py` | delta-операции (rewrite/replace/append) |
| | `src/rob_box_voice/test/test_quick_decide.py` | уровень 1 (правила) |

---

## Что подключить и где — сводный список для следующей карточки

Если следующая карточка в этом ряду — **«выкатить pregenerate на прод»
/ «дописать то, что отсутствует»**, то:

1. **Синхронизировать YAML.** Добавить в
   `docker/vision/config/voice_assistant/tts_node.yaml` (актуальный
   overlay для контейнера) три параметра, чтобы они попали в ноду:
   * `pregenerate_enabled: false` (kill-switch по умолчанию)
   * `pregenerate_confidence_floor: 0.6` (соответствует
     `scheduler/pregen/estimator.py:CONFIDENCE_FLOOR`)
   * `pregenerate_history_window: 10`
   Эти же параметры объявлены в `src/rob_box_voice/rob_box_voice/tts_node.py:1678-1687`
   через `self.get_parameter("…").value` — declare_parameter
   добавлять не нужно, они уже внутри ноды, читаются через
   `get_parameter`.

2. **Сверить `tts_node.yaml:8` upstream/downstream.**
   `src/rob_box_voice/config/tts_node.yaml:8` (комментарий
   про удалённый cache_dir/cache_enabled/**pregenerate**/prosody_pitch)
   — исторический; оверлей не трогает, но строка 8 в апстриме может
   смутить читателя. Кандидат на редакторскую правку комментария,
   без функциональных изменений.

3. **Устранить расхождение документации** с текущим кодом. В
   `docs/architecture/target-operator-agent-and-dialogue.md:1139`
   уже стоит «✅ сделано», но §8a.4 (строки 835-839) и
   `docs/design/SCHEDULER_DESIGN.md:38-40` повторяют «ключ pregenerate
   встречается ровно один раз в комментарии». Логически §8a.4 — это
   «порядок включения», строка описывает почему выбрана фаза 3 «в
   конце»; она не требует правки ради правды, но координатор
   следующей карточки может захотеть footnote («с тех пор реализован
   ADR-0056, см. commit YYY»).

4. **Никаких новых модулей в `scheduler/`.** Подключение идёт
   через `scheduler/pregen/*`. Если нужно расширение — добавлять
   `pregen/*.py`, не возвращать удалённый плоский `scheduler.pre_gen`.

5. **Тесты уже покрывают** все ключевые пути. Дополнительные
   тесты добавлять только под новый код; ничего из текущих
   не дублировать.

---

## Контрольные строки для kanban-карточки

Карточка `t_9ab8ab70` говорит «компактная сводка в формате
`файл:строка` + короткий список «что подключить и где»». Эти
строки покрывают все 5 пунктов карточки:

* (1) документы — раздел 1 выше;
* (2) tts_node — раздел 2 выше;
* (3) scheduler/ — раздел 3 выше;
* (4) tts_node.yaml:8 — раздел 4 выше;
* (5) смежные тесты — раздел 5 выше.

Подтверждение блокера §4.8 — таблица в начале файла, статус СНЯТ.
