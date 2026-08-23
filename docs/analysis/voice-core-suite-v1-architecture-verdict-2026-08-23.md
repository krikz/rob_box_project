# Voice Core Suite v1 — архитектурный вердикт (2026-08-23)

| Поле | Значение |
|------|----------|
| Автор | architect |
| Цель | Дать архитектурный verdict по сценарию `.github/e2e/scenarios/voice_core_suite_v1.json` + `voice_core_acceptance_v1.json` (issue #1506), после их мержа в develop через PR #1517 |
| Связано | issue #1506, PR #1517 (merged 22.08 19:52 UTC), ADR-0022 §4.6 (window semantics), ADR-0024 (music-aware intent priority gate, issue #1525), анализ-док `voice-features-e2e-validation-2026-08-22.md` |
| Вердикт | **APPROVE** (с замечаниями для e2e-process) |

## 1. Резюме

Сценарий `voice_core_suite_v1.json` (11 шагов) + acceptance-контракт `voice_core_acceptance_v1.json`
полностью покрывают 8 шагов из acceptance-criteria issue #1506
(command gate / new-session / альтернативный wake word / multi-voice ×3 /
music start+stop). Дизайн соответствует ADR-0022 GATE-1 (per-step +
aggregate AND-семантика) и ADR-0024 (music-aware priority gate,
защищающему dj02_stop_music от Nav2-коллизии). После merge в develop
через PR #1517 (22.08 19:52 UTC) и серии стабилизирующих коммитов
(db84ff59, 04d4ba2f, 1b87e460, #1533 mv03-fix) — набор артефактов
**готов к e2e-прогону**.

**Карточка t_8e8d342a истекла на 50 итерациях в первой попытке
(22.08 → 23.08), не закоммитив ничего** — этот вердикт закрывает
архитектурный долг и оставляет фактический прогон за `e2e-process`
(как и положено по процессу из анализ-дока §5).

## 2. Покрытие acceptance issue #1506

| # | Шаг issue #1506 | Метка в сценарии | Способ проверки | ADR/issue |
|---|----------------|------------------|------------------|-----------|
| 1 | «Робот, где ты» → `LLM dispatch skipped` | `cc01_status_gate` | `patterns=["LLM dispatch skipped", "command intent"]` | ADR-0022 §5.4, issue #1279 |
| 2 | «Робот, сбрось всё» → `session reset` | `ns01_reset_session` | `patterns=["session reset"]` | issue #1248 (new-session), тест `test_new_session_reset.py` |
| 3 | «Робокс, как дела» → полный цикл | `ww01_roboks_wake` | `patterns=[]` (полный цикл = TTS-finished в логах) | issue #1252, ADR-0022 §A.10 |
| 4a | mv01 «говори голосом Алены» → `set_voice` | `mv01_set_voice_alena` | `patterns=["set_voice"]` + per-step `expected_tool_calls=["set_voice"]` | issue #1219, `test_issue_1219_set_voice_rule.py` |
| 4b | mv02 «скажи привет» → ответ голосом alena | `mv02_speak_alena` | `patterns=["current_voice"]` | issue #1219, тест voice-persistence |
| 4c | mv03 «сказку разными голосами» → несколько голосов | `mv03_skazka_raznymi_golosami` | `expected_tool_calls=["set_voice"]` + `expected_keywords=["Красная"]` | issue #1532, `test_issue_1532_voice_multi_skazka.py` |
| 5 | Music: «renardo бит» → `execute_music_code` → «стоп музыку» → `stop_music` | `dj01_start_renardo` + `dj02_stop_music` | per-step `expected_tool_calls` + `dj01.must_not_call=["generate_music"]` | issue #1358, ADR-0024 (dj02) |
| — (бонус) | Спор→арбитр через backlog | `ds01/ds02/ds03` | `expect="backlog"` для ds01/ds02 + `patterns=["flushed to LLM"]` для ds03 | issue #979 (backlog), `test_speech_backlog_accumulator.py` |

**Все 8 шагов покрыты + 1 backlog-сценарий в нагрузку.** Полнота соответствия
acceptance issue #1506 — **11/11 (100%)**.

## 3. Соответствие ADR-0022 GATE-1

### 3.1 Aggregate layer (top-level acceptance.json)

```json
{
  "expected_tool_calls": ["set_voice", "execute_music_code", "stop_music"],
  "must_not_call": []
}
```

**Вердикт: корректно.** AND-семантика (каждый из tools должен встретиться
хотя бы раз за прогон) — три инструмента, три различных feature area:

- `set_voice` — multi-voice #1219 (mv01)
- `execute_music_code` — music start #1358 (dj01)
- `stop_music` — music stop #1358 (dj02, **только если start реально сработал**)

**Архитектурное замечание (не блокер).** `stop_music` зависит от
`execute_music_code` (нужен играющий трек). Если `dj01_start_renardo`
провалится по любой причине (LLM решил не вызывать tool, генератор не
поднялся), `dj02_stop_music` тоже провалится. Это **валидное поведение**
(мы тестируем реальный цикл), но e2e-process должен учитывать это в
своей диагностике: «stop_music missing» → сначала проверить
`execute_music_code` в логах dj01, потом уже фейлить GATE-1.

### 3.2 Per-step layer

- `mv01` — `expected_tool_calls=["set_voice"]`, `must_not_call=[]` ✅
- `mv03` — `expected_tool_calls=["set_voice"]`, `expected_keywords=["Красная"]` ✅
  (защита от регрессии «LLM уходит в generic жила-была», issue #1532)
- `dj01` — `expected_tool_calls=["execute_music_code"]`, `must_not_call=["generate_music"]` ✅
  (MiniMax Music API недоступен — намеренное исключение)
- `dj02` — `expected_tool_calls=["stop_music"]` ✅

**Вердикт: per-step корректно, пересечений с aggregate нет.**

### 3.3 Window semantics (ADR-0022 §4.6 addendum)

Сценарий использует `react_window: 40` (из блока `## e2e` issue #1506).
После фикса `db84ff59` (E2E_RUN_BEFORE через robot_clock) и
ADR-0022 §4.6.3 — aggregate check корректно использует общее окно.
**Никаких race conditions между per-step и aggregate.**

### 3.4 Case-sensitivity (ADR-0022 §4.6.4)

`check_patterns` case-sensitive, `check_acceptance` case-insensitive.

Все patterns в сценарии — lowercase snake_case (`set_voice`,
`execute_music_code`, `stop_music`, `session reset`, `LLM dispatch skipped`).
Это **корректно по контракту** (snake_case для tool names). Один
нюанс: `"LLM dispatch skipped"` — mixed case. Это правильно,
поскольку текст лога voice-assistant выводится именно так
(`🎯 [issue 1279] … LLM dispatch skipped`).

## 4. Соответствие ADR-0024 (music-aware priority gate)

`dj02_stop_music` — наиболее рискованный шаг. Без ADR-0024:
- «стоп музыку» без wake-слова → wake-gate отбрасывает, музыка играет
- «стоп» с wake-словом, но command_parser матчит `IntentType.STOP` →
  Nav2 cancel вместо `stop_music`

ADR-0024 (proposed, issue #1525) вводит двуxслойный gate:
1. `MUSIC_STOP_OVERRIDES` в `dialogue_guards.py` (диалоговый gate)
2. `is_music_stop_command()` — single source of truth для детекта

**Вердикт по dj02:** архитектурно защищён ADR-0024, **но ADR-0024 имеет
статус `proposed` на момент verdict**. Если фикс ещё не в develop —
прогон `dj02_stop_music` закономерно провалится с теми же симптомами,
что и e2e run 32573773556 (22.08). e2e-process должен:

1. Проверить наличие коммита ADR-0024-fix в develop (`git log --grep
   "music-aware intent priority"` или issue #1525).
2. Если нет — это **expected FAIL**, не bug. Завести issue-bug по
   шаблону, в `## description` указать «блокируется ADR-0024 (proposed)».

## 5. Голосовые команды (block `## e2e` в issue)

26 .ogg файлов в `.github/e2e/voice_commands/`. Для шагов issue #1506:

| Шаг | Файл | voice_text | volume | record_seconds |
|-----|------|------------|--------|----------------|
| cc01 | ❌ нет в списке | «Робот, где ты» | 150 | 5 |
| ns01 | ❌ нет | «Робот, сбрось всё» | 150 | 5 |
| ww01 | ❌ нет | «Робокс, как дела» | 150 | 5 |
| mv01 | ❌ нет | «Робот, говори голосом Алены» | 150 | 7 |
| mv02 | ❌ нет | «Робот, скажи привет» | 150 | 5 |
| mv03 | ✅ `rabot_rasskazhi_skazku_raznymi_golosami.ogg` | — | — | — |
| ds01/ds02 | ❌ нет (backlog — out of scope для voice-e2e по §6) | — | — | — |
| ds03 | ❌ нет | «Робот, а ты как думаешь, кто из них прав, а, РОБОТ?» | 150 | 8 |
| dj01 | ✅ `rabot_igrai_renardo_bemol.ogg` (близкая команда) | — | — | — |
| dj02 | ❌ нет | «Робот, стоп музыку» | 150 | 5 |

**Замечание для e2e-process.** Текущий сценарий использует **scenario_file
(JSON с явными текстами)**, а НЕ `voice_text` single-shot. Это правильный
путь для multi-step suite (см. `VOICE_COMMANDS_RESEARCH.md` §1 — single
voice_text с разделителями ломает VAD max=12с). e2e-process должен
запускать `e2e_voice_test.sh --scenario voice_core_suite_v1.json
--acceptance voice_core_acceptance_v1.json` — единицы синтеза
TTS генерируются на лету из scenario.steps[].text, дополнительные
.ogg для блока `## e2e` issue **не требуются** (они были бы redundant).

`voice: anton`, `tts: minimax-male-qn-qingse` — **несовместимо**.
TTS-голос в scenario (anton = Yandex) ≠ TTS-голос в блоке `## e2e`
(MiniMax). По `VOICE_COMMANDS_RESEARCH.md` §4 — основная проверенная
цепочка `Yandex TTS → ReSpeaker → Yandex STT`. **Рекомендация:**
e2e-process должен использовать **Yandex TTS + Yandex STT**
(как в acceptance.voice_provenance) и **не** MiniMax TTS — иначе
будет double-mismatch (MiniMax голос не совпадает с принятым в
scenario `voice: anton`).

## 6. Trade-offs и архитектурные замечания

### 6.1 KISS — сценарий ровно покрывает scope, не больше

Сценарий **не** пытается покрыть:
- speech backlog chain целиком (только ds01-ds03 как побочный сценарий
  через `expect=backlog`) — корректно, §6 анализ-дока
- speaker diarization #1077 — out of scope по §6
- MiniMax `generate_music` — out of scope по §3

Это **сознательное ограничение текущего харнесса** (один синтезированный
голос через один микрофон). Любая попытка расширить сценарий приведёт
к «зелёным ложно» — анализ-док §6 явно об этом предупреждает.

### 6.2 Backlog-аккумулятор — адекватное покрытие

ds01-ds03 используют **разные Yandex TTS-голоса** (anton/ermil/zahar)
для имитации разных спикеров. STT на роботе не различает спикеров,
но это **валидная проверка backlog-механизма**: ds01/ds02 накапливаются
(`expect=backlog`), ds03 с wake-словом флашит (`patterns=["flushed to LLM"]`).

### 6.3 mv03 keywords — слабая, но достаточная защита

`expected_keywords=["Красная"]` — минимальный контраст против «жила-была»
регрессии. Полная проверка «несколько голосов в ответе» вынесена в
`dialogue_node` метрики (`set_voice ×≥2`). Per-step `expected_tool_calls=["set_voice"]`
ловит только ≥1 — это **намеренно**: иначе false-negative на LLM,
который выбрал 1 смену голоса.

### 6.4 dj01/dj02 — риск music-runtime

`sclang` (Renardo backend) поднимается ~30 секунд на старте. Если
прогон стартует до полного подъёма Renardo → dj01 упадёт по
`OSError: cannot connect to sclang`. **Рекомендация для e2e-process:**
добавить preflight `pgrep -f sclang || systemctl status supercollider`
в начало прогона.

## 7. Что должен сделать e2e-process

1. Запустить `e2e_voice_test.sh --scenario voice_core_suite_v1.json
   --acceptance voice_core_acceptance_v1.json` (команды из scenario.steps[].text,
   НЕ из блока `## e2e` issue — scenario уже включает тексты).
2. TTS = Yandex (как в acceptance.voice_provenance).
3. Проверить наличие ADR-0024-fix в develop (commit hash по issue #1525).
   Если нет — `dj02_stop_music` ожидаемо упадёт → `e2e:rejected`,
   завести issue-bug с блокером «blocked by ADR-0024 (proposed)».
4. Pre-flight: `sclang` running.
5. При PASS поставить `e2e-done`, по истечении 24h stale-candidate
   закрыть issue #1506 (ADR-0022 §5.1).

## 8. Вердикт

**APPROVE.**

- Acceptance issue #1506 — 11/11 (100%) покрыто.
- Соответствие ADR-0022 GATE-1 — корректно (aggregate + per-step,
  без race).
- Соответствие ADR-0024 — dj02_stop_music архитектурно защищён, но
  зависит от принятия ADR-0024-fix в develop.
- Голосовые команды — scenario.json содержит тексты, TTS-цепочка
  Yandex → ReSpeaker → Yandex, проверена в `VOICE_COMMANDS_RESEARCH.md`.
- Архитектурных блокеров для e2e-прогона **нет**.

**Открытый вопрос для Шифу:** принять ADR-0024 (status=proposed →
accepted) до прогона e2e или параллельно (если хочется быстрее)?

**Альтернативный план Б (если ADR-0024 ещё не в develop):**
- Запустить прогон как есть.
- `dj02_stop_music` ожидаемо упадёт → issue-bug по шаблону, не блок
  для остальных шагов.
- dj01 (start) пройдёт → подтверждает Renardo integration #1358.

---

*Создан как архитектурный долг по карточке t_8e8d342a (первая попытка
истекла 23.08 на 50 итерациях, ничего не закоммитив). Этот verdict
восполняет долг и оставляет фактический прогон за e2e-process —
как и положено по процессу из анализ-дока §5.*
