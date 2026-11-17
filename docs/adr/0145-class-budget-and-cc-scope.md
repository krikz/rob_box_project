# ADR-0145: Бюджет размера класса, расширение CC-budget и вердикты по god-классам (архитектурный аудит 29.09)

| Поле | Значение |
|---|---|
| Статус | Proposed (на утверждение товарищу Шифу) |
| Дата | 2026-09-29 |
| Автор | Claude Code (сводка отчётов шиди-воркеров, каждый факт перепроверен по коду/git) |
| Контекст | Статический аудит `G: Architecture Audit` run [36577898577](https://github.com/krikz/rob_box_project/actions/runs/36577898577) (класс-метрики `tools/architecture_class_metrics.py`) и runtime-аудит `L: Architecture Audit` run [36468381546](https://github.com/krikz/rob_box_project/actions/runs/36468381546) + свежий [36597906491](https://github.com/krikz/rob_box_project/actions/runs/36597906491) (29.09, success; артефакт runtime-findings в этой сессии **не прочитан** — egress-политика окружения блокирует blob-storage, поэтому runtime-выводов в этом ADR нет) |
| Затрагивает | `scripts/lint/cc_budget.py`, `scripts/lint/cc_budget_baseline.json`, новый `scripts/lint/class_budget.py` + `class_budget_baseline.json`, self-tests, `.github/workflows/G-Lint Code.yml`; дедупы в `rob_box_llm`, `rob_box_core`, `rob_box_voice`, `rob_box_mcp_tools` |
| Родители | ADR-0021 (R1 CC-budget), ADR-0021-r1 (ratchets), AF-0013 (инкрементальная поставка), ADR-0018 (честный FAIL) |

---

## 1. Что показал аудит (develop @ `2865630f`)

`python tools/architecture_class_metrics.py` → 687 классов, **20 god-классов** (WMC ≥ 47, TCC < 0.33, LOC ≥ 500), 12 файлов ≥ 2000 строк.

| Класс | LOC | Методов | WMC | Max CC |
|---|---:|---:|---:|---|
| `DialogueNode` (`rob_box_voice/dialogue_node.py`) | 9872 | 218 | 1184 | 82 `_handle_result` |
| `TTSNode` (`rob_box_voice/tts_node.py`) | 6647 | 112 | 707 | 28 `__init__` |
| `AvatarSupervisor` (`supervisor_node.py`) | 2478 | 63 | 320 | 19 |
| `ComposeMusicTool` (`tools/music.py`) | 1697 | 52 | 271 | 15 |
| `MusicManager` (`tools/music.py`) | 2368 | 60 | 254 | 19 |
| `STTNode` | 1773 | 42 | 223 | 22 |
| `WSSServer` (`rob_box_quest`) | 1528 | 54 | 215 | 13 |
| `SpeakerIdNode` | 2091 | 38 | 204 | 15 |
| `AgentCore` (`rob_box_harness`) | 1702 | 29 | 175 | 19 |
| `QuestBridge` / `QuestNode` | 1274 / 1483 | 59 / 36 | 170 / 173 | 13 / 15 |
| `MCPServer` | 1573 | 29 | 168 | 13 |

## 2. Где в истории были приняты неверные решения

Проверено по `git log` / `git show <sha>:<file> | wc -l` (репозиторий распакован из shallow-клона).

| Решение | Когда / где | Что вышло (цифры) | Вердикт |
|---|---|---|---|
| Переписать `dialogue_node.py` в «тонкую оболочку над DialogCore» **без бюджета на размер** | `2a0aee26`, 28.07 | 348 строк → 2525 строк через 9 дней (`db006074` +584, серия `fix(992)`) → 4089 (18.08) → **10687** (29.09), `def` 21 → 239 | Декомпозиция была верной, но без сторожа её откатили фичами. **Ошибка — отсутствие ограничителя, а не сам рефакторинг** |
| ADR-0021 R1: бюджет только на CC **метода** | ADR-0021, 18.08 | `_handle_result` CC 44 → 85; класс 79 → 218 методов. Около 11 методов `DialogueNode` и 12 `TTSNode` прямо помечены «вынесено ради cc_budget» — и все остались **в том же классе**. Основной рост (≈95%) — обычные фичи/фиксы, которые складывались в тот же файл, потому что никакой гард на размер класса их не останавливал | **Ошибочное (неполное)** — нужен бюджет на класс (§3.2) |
| ADR-0021: «big-bang DialogueNode → 6 классов» отклонён, «вынос в core не нужен» | ADR-0021, «Альтернативы» | Из трёх запланированных экстракций (#1406 stt_gate, #1407 startup_greeting, #1408 music_guard) сделана одна (`music_guard`, нода −46 строк: остался адаптер `_apply_music_guard` на 215 строк). `startup_greeting` в `core/` так и не вынесен; stt-gate появился только 16.09 под именем `core/stt_admission.py`, а `_DialogueSttHost` (218 строк) остался в ноде | **Ошибочное**: «серия маленьких PR» не выполнялась, а запрет на большой рефакторинг стал оправданием для роста |
| ADR-0021 ссылается на «ADR-0013 incremental» | ADR-0021 | Файл `0013-*` — про ReSpeaker DSP; правило размера PR — в `AF-0013-…`. Правило AF-0013 ограничивает **PR** (≤3000 строк), но не **модуль** | Ссылка исправлена в этом PR |
| `cc_budget.py` введён через 18 дней после ADR-0021, ADR-0021 и ADR-0021-r1 до сих пор `proposed` | `7a21d158`, 05.09 | В сентябре 8 bump'ов baseline без карточек (задокументировано в ADR-0021-r1) | Запоздалое; статус ADR не приведён в соответствие с практикой |
| Экстракция «guard heuristics» в один модуль | `core/dialogue_guards.py`, `1811b8eb`, 15.08 | 306 → **3134** строк: модуль-мусорка вместо разделения по ответственностям | **Ошибочное**: god-класс заменён god-модулем |
| Параллельная реализация guard'ов в `core/turn.py` за выключенным флагом | `TurnGuards` + 12 классов в `core/turn.py`; `self._use_turn_guards = False` (`dialogue_node.py:1198`) | В ноде живут 9 legacy-методов `_check_*_and_retry` (`dialogue_node.py:5571-6365`, ≈700 строк) — **та же логика в двух местах**, работает только legacy | **Ошибочное состояние** (шов без потребителя): флаг нужно переключить или удалить одну из копий (§4, карточка 2) |

## 3. Решение

### 3.1. CC-budget: исправить дыры ratchet'а (ADR-0021-r1 выполнен не полностью)

Все дефекты воспроизведены на develop до правки:

1. **R-1b работал только ниже лимита.** Метод, который остаётся над лимитом, но упал ниже baseline (например, `_handle_result` 82 при baseline 85, `AgentCore.process_input` 19 при 23), проходил как `[ok]`, и запас можно было снова нарастить молча. Таких было 7. Теперь любое `cc < baseline` → FAIL «обнови baseline в том же PR»; 7 записей опущены до измеренных значений (вместе с `_legacy_acknowledged[].cc`, как требует R-1a).
2. **`--update-baseline` стирал `_refactor_cards` и `_legacy_acknowledged`**, хотя именно его сторож советует запускать. Теперь метаданные сохраняются, legacy-cc синхронизируется только вниз, рост legacy → отказ (R-1a), новая запись с CC>30 без `_adr_reference` → отказ (R-1e, раньше было только в тексте ADR).
3. **Запуск с явными путями давал ложные phantom** для всех непросканированных файлов. Теперь phantom считается только по просканированным файлам (плюс удалённые файлы остаются phantom).
4. **Scope.** Добавлены `rob_box_perception`, `rob_box_telegram`, `rob_box_llm`, `rob_box_core`: там 16 методов над лимитом, включая `MiniMaxTTSProvider.stream` с CC=38. Все взяты в baseline как legacy; CC=38 привязан к этому ADR через `_adr_reference` (R-1e).
5. У `cc_budget.py` впервые есть self-tests (`scripts/lint/test_cc_budget.py`).

### 3.2. Новый гард: бюджет размера класса (`scripts/lint/class_budget.py`)

- «Большой» класс: **WMC > 80 или методов > 40** (WMC = сумма CC методов, та же метрика, что в `cc_budget.py`).
- Все текущие большие классы зафиксированы в `class_budget_baseline.json`. Это shrink-only ratchet:
  - новый большой класс → FAIL: разрезать до merge;
  - рост WMC или числа методов → FAIL. Хелпер, вынесенный в тот же класс, **не уменьшает** класс; выносить нужно в отдельный модуль или класс;
  - уменьшение → FAIL с требованием обновить baseline в том же PR. Выигрыш закрепляется, как в R-1b;
  - класс перестал быть большим → убрать из baseline.
- LOC намеренно не лимитируется: docstring/комментарии не должны ломать CI.
- **Осознанный trade-off:** любой фикс, добавляющий `if` в `DialogueNode`, теперь требует либо вынести эквивалентную сложность, либо поднять baseline с `_refactor_cards` → `#issue`. Это и есть давление, которого не хватало с 18.08. Если Шифу сочтёт это слишком жёстким — вариант смягчения: допуск `+N` WMC на PR с обязательной карточкой.

### 3.3. Дубли: что делаем сейчас, что — карточками

Клоны искали AST-детектором (нормализованные имена, ≥8 statements) и `pylint duplicate-code` (≥12 строк) по 387 продовым файлам `src/`. В этом PR исправлены только безопасные группы без изменения поведения; что именно сделано, с raw-выводом тестов, — в описании PR.

| Группа | Решение |
|---|---|
| `MiniMaxTTSProvider` объявлен **дважды в одном файле** (`rob_box_llm/providers/minimax_tts.py:493` заглушка, `:571` реальный) | Удалить мёртвую заглушку (этот PR) |
| Списки «мусорных» имён спикера — 3 копии, **уже разошлись** (`utils/speaker_embeddings.py`, `core/dialogue_helpers.py`, `mcp_tools/tools/dialogue.py`) | Один источник `rob_box_core/speaker_names.py` = объединение (этот PR). Поведенческое изменение: MCP-тул дополнительно отвергает `null/none/undefined`, БД — `гость/user/speaker/…` |
| `_notify_music_state` ×3 в `tools/music.py` | Один module-level helper (этот PR) |
| `_parse_optional_int/float` в `tts_node.py` | Один generic (этот PR) |
| `start_metrics_server` telegram ≡ voice | В `rob_box_core`, если тесты-патчи позволяют (этот PR или честный skip) |
| Hailo-init ×3 в `rob_box_perception` (≈150 строк) | Карточка: mixin из ADR-0121 уже описан, но не доделан; нужен прогон на железе |
| legacy `_check_*_and_retry` ≡ `core/turn.py` guards | Карточка 2 ниже — самый дорогой дубль |
| `_parse_op` (mcp_tools) ≡ `_parse_delta_op` (voice scheduler), stream-replay (harness LLM ≡ TTS), `utterance` fallback (осознанный, #2233), XML-escape ×3 | Карточки P3 |

## 4. Вердикты по god-классам и порядок карточек

Правило для всех карточек: один шаг = один PR ≤ ~600 изменённых строк (AF-0013). На ноде остаётся делегат с прежней сигнатурой, потому что 61 тест строит `DialogueNode` через `object.__new__` и патчит приватные методы. В том же PR опускаются `cc_budget` и `class_budget` baseline.

**DialogueNode → тонкий ROS-клей + коллабораторы** (P1):
1. Module-level и `_DialogueSttHost` → `core/stt_admission_host.py`, `core/identity_ack.py`, `core/dialogue_helpers.py`. Примерно −600 строк, почти без риска.
2. Включить `TurnGuards` и удалить legacy `_check_*_and_retry`. Сначала golden-тест эквивалентности, затем 2a/2b. До −700 строк и −75 WMC.
3. `_declare_params` и resolve-хелперы → `dialogue_params.py`.
4. `_handle_result` (CC 82) → `core/result_pipeline.py`: `ResultContext` + упорядоченный список шагов (`Error`, `MusicToolAck`, `DjAutoSuppress`, `PlanningMute`, `GuardChain`, `EmptySpoken`, `ServiceTextFilter`, `Publish`). **Не** «ещё приватные методы».
5. `SpeakerIdentityService` → `core/speaker_identity.py` (34 метода, ≈1080 строк).
6. `MusicTurnCoordinator` → `core/music_turn.py`.
7. `LlmTurnLoop` → `core/llm_turn_loop.py` (три метода с CC ≥ 18).
8. `PromptContextBuilder` → `core/prompt_context.py`.

**TTSNode** (P1):
1. Module-level чистые функции → `tts_ssml.py` / `utils/audio_utils.py` / `tts_params.py`.
2. `SsmlParser`.
3. `ProviderChain` + `ProviderHealth`.
4. `PlaybackQueue`.
5. `PregenCache`.
6. `__init__` (CC 28) → таблица параметров + `TtsConfig.from_node` + `ProviderFactory`.
7. `SynthesisBackend` ×3 (по PR на backend).
8. `AudioSinkPublisher`.

Все переносы должны пополнять `_SAP_HELPER_NAMES`.

| Класс / модуль | Вердикт | Prio |
|---|---|---|
| `tools/music.py` (7158 строк, 19 классов) | SPLIT на соседние `tools/music_*.py` + фасад `music.py` с реэкспортами. Не пакет `tools/music/`, потому что `gen_tool_catalog.py` сканирует `tools/*.py`. Перенацелить AST-читающие тесты и 3 патча имён | P1 |
| `MCPServer` | SPLIT: music orchestration → `music_orchestrator.py` | P2 |
| `STTNode` | SPLIT: `stt_providers.py` | P2 |
| `WSSServer` | SPLIT по 3 группам (send/rate-limit, deliver, json/session) | P2 |
| `AvatarSupervisor` | EXTRACT-PURE (21 stateless метод), класс не делить (TCC 0.41) | P2 |
| `TelegramNode` | SPLIT publishers → `telegram_publishers.py` | P2 |
| `core/dialogue_guards.py` | EXTRACT по семействам + фасад (62 импорта) | P2 |
| `TextNormalizer` (`scripts/text_normalizer.py`) | Живой код: грузится в `tts_node.py` через `sys.path.insert`. Перенести в `core/` | P2 |
| `QuestBridge` / `QuestNode`, `SpeakerIdNode`, `core/arranger.py` | SPLIT / EXTRACT-PURE позже | P3 |
| `AgentCore`, `TaskScheduler`, `AudioNode`, `SoundNode`, `FaceRecognizer`, `ContextAggregatorNode`, `SQLiteVoiceMemory`, `HarnessMiniMaxProvider`, `SupervisorClient` | KEEP: одна ответственность (Resp=1) или DAO, метрика TCC вводит в заблуждение | — |
| `_tool_catalog_data.py` (6632 строки) | KEEP — генерируется `tools/gen_tool_catalog.py` | — |
| `scripts/silero_tts_gui.py`, `scripts/robbox_chat*.py` | DELETE-кандидаты: нет entry points, launch, docker и импортов. **Нужно решение Шифу** | P3 |

## 5. Альтернативы, которые не выбрали

- **Лимит LOC на файл/класс.** Ломается на docstring/комментариях и провоцирует сжатие кода вместо разделения. WMC + число методов измеряют то, что важно.
- **Один большой PR «распилить DialogueNode».** Противоречит AF-0013; 61 тест завязан на приватные методы.
- **Поднять только порог CC (например, 10).** Усилило бы именно тот механизм, который выносит хелперы внутрь того же класса.

## 6. Acceptance

- [ ] `cc_budget.py`, `class_budget.py` и self-tests зелёные в `G: Lint Code` на PR (run_id — в описании PR).
- [ ] Шифу утвердил пороги класс-бюджета (WMC > 80 / методов > 40) и политику «shrink тоже требует refresh».
- [ ] Шифу решил судьбу `silero_tts_gui.py` / `robbox_chat*.py`.
- [ ] Карточки §4 заведены отдельными issue (не в этом PR).
- [ ] После утверждения: ADR-0021, ADR-0021-r1 и этот ADR → Active.
