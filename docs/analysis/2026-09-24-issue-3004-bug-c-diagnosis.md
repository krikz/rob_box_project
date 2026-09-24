# Issue #3004 — Bug C «ретраит успешный compose_music» — диагноз

**Дата:** 2026-09-24
**Автор:** architect (через автомат-агента)
**Карточка:** t_3b8e9578
**Связанные:** issue #3004, issue #992 (MusicGuard история), commit `149f755f8` (капабилити-флаги), commit `209c16b73` (compose_music был auto-stopped 1.5с)

## TL;DR для товарища Шифу

**Корневая причина из issue (compose_music не в условии Bug C) уже устранена** в develop коммитом `149f755f8 feat(music): derive music tool sets from catalog capability flags`. Прямой runtime-вызов `MusicGuard.evaluate(tools_called=("compose_music",))` сейчас возвращает `SKIP/executed` — Bug C никогда не выстрелит, CRITICAL-retry не пойдёт, лишний `compose_music` не случится. Юнит-тест `tests/unit/core/test_music_guard.py::test_music_starting_tools_derived_from_catalog` это зафиксировал:

```python
assert {"execute_music_code", "compose_music"} <= MUSIC_STARTING_TOOLS
```

Подтверждено прямым вызовом (`python3` REPL, 24.09 ~16:00 UTC):

```
✓ Acceptance #1: compose_music → SKIP
✓ Acceptance #2: 8 раз подряд — всегда SKIP, не Bug C
✓ Acceptance #3: Bug C NEVER fires при compose_music в tools_called
```

Что с этим делать — зависит от ответа на один вопрос (см. §3 «Развилка для Шифу»). Архитектор **не** применяет фикс руками — фикс уже в коде и в тестах (см. §1 «Что в коде на develop»).

## 1. Что в коде на develop сейчас

### 1.1 `MusicGuard.evaluate()` (src/rob_box_voice/rob_box_voice/core/music_guard.py:357-374)

```python
tools_set = set(tools_called or ())
# Issue #1392 follow-up: MiniMax AI-генерация тоже «запустила музыку».
_music_started = tools_set & MUSIC_STARTING_TOOLS  # ← derives from catalog
if _music_started:
    # Success — reset both budgets ...
    self._dj_retry_count = 0
    self._user_retry_count = 0
    return MusicGuardVerdict(kind=MusicGuardVerdictKind.SKIP, reason="executed")
```

`_music_started` — пересечение `tools_called` с capability-флагом `starts_music` каталога. Если хоть один тул из вызванных запускает музыку — Bug C **закорочен в SKIP**, retry-бюджет сброшен. `MUSIC_STARTING_TOOLS` строится из `TOOL_CATALOG` (динамически, не frozenset-литералом).

### 1.2 Что попадает в `MUSIC_STARTING_TOOLS` (runtime-проверено)

```
['compose_music', 'execute_music_code', 'gen_play_from_library', 'generate_music']
```

— все 4 «живых» музыкальных тула. `set_dj_mode`, `load_track`, `lookup_melody`, `search_samples`, `save_track`, `delete_track` — НЕ запускают музыку напрямую (это конфиг/поиск/манипуляция), для них `starts_music=False`.

### 1.3 Уже существующие регрессионные тесты

`src/rob_box_voice/test/unit/core/test_music_guard.py:65-74`:

```python
def test_music_starting_tools_derived_from_catalog() -> None:
    """Множества музыкальных тулов выводятся из каталога, а не из frozenset."""
    from rob_box_core.tool_catalog import TOOL_CATALOG
    from rob_box_voice.core.dialogue_guards import MUSIC_STARTING_TOOLS

    expected = {e.name for e in TOOL_CATALOG if e.starts_music}
    assert MUSIC_STARTING_TOOLS == frozenset(expected)
    assert {"execute_music_code", "compose_music"} <= MUSIC_STARTING_TOOLS
```

Прогон 24.09: `64 passed in 0.49s`. Защита от регрессии **уже есть**, и она активна — если кто-то вынет `starts_music=True` из манифеста compose_music, этот тест покраснеет на CI до merge.

## 2. Что видел товарищ Шифу в живом логе 14:49

Лог-вырезка из issue (дословно):

```
14:49:18 compose_music {'name': 'smells like teen spirit', 'bass_synth': ...}
14:49:22 compose_music {'name': 'smells like teen spirit', 'bass_synth': ...}
14:49:27 WARN 🎵 [issue 992 Bug C] user asked for music but LLM skipped
          execute_music_code (tools=['compose_music', ...]); synchronous retry ...
14:49:34 compose_music {'name': 'smells like teen spirit', 'lead_synth': ...}
```

**Странность**: `compose_music` стоит в `tools_called`, и при этом Bug C говорит «LLM skipped execute_music_code». Это поведение **не должно** наблюдаться на текущем develop. Две гипотезы (см. §3 какая правильная):

### Гипотеза A — на Vision Pi развёрнут старый образ

`.image-versions.dev` на момент написания: `dev-88be6cf` для voice-assistant (vision Pi). Коммит `88be6cf` от какой даты? Если он **старше** коммита `149f755f8` (21.08.2026), то в контейнере крутится код с ручными frozenset'ами `RENARDO_MUSIC_TOOLS`/`GENERATED_MUSIC_TOOLS` — и тогда возможно (но маловероятно) `compose_music` ещё не был в `_RENARDO_MUSIC_TOOLS`. Проверка — `./docker exec voice-assistant python3 -c "from rob_box_voice.core.dialogue_guards import MUSIC_STARTING_TOOLS; print(sorted(MUSIC_STARTING_TOOLS))"`.

### Гипотеза B — фикс в коде корректный, но есть **второй** guard, который стреляет параллельно

В `dialogue_node.py:6381-6384` есть **локальный** whitelist музыкальных тулов для issue #1708 (hallucinated-lyrics guard):

```python
_music_tool_names = {
    "execute_music_code", "generate_music",
    "gen_play_from_library", "set_vibe_preset", "load_track",
}
```

`compose_music` **отсутствует** в этом set. Но это guard #1708 (подавление TTS-зачитки кода), он не Bug C — прямой связи нет. Тем не менее стоит проверить: когда LLM вызывает `compose_music` плюс `speak_text` в одном turn, hallucinated-lyrics guard может не подавить TTS, и юзер услышит «Запускаю compose_music с…» в spoken. Это отдельный кейс, не Bug C.

### Гипотеза C — LLM действительно вызывает `compose_music` повторно

Из лога видно: `bass_synth` вариант в 14:49:18 + 14:49:22, потом `lead_synth` в 14:49:34–39. Это похоже на то, что модель **решает** поменять состав трека посреди сета (lead/bass swap). Bug C тут **не виноват** — это поведение модели в DJ-сессии, которое вызвало бы 8 вызовов и при отсутствии guard'а вообще. Если guard ОК — повторов не будет. См. §3 как проверить.

### Гипотеза D — issue был написан по памяти, актуальный runtime уже чистый

Товарищ Шифу помнит состояние ДО коммита `149f755f8`. Лог 14:49 — старый кусок из истории, issue создан по симптом-без-verify-сессии. В этом случае accept закрыт по факту (фикс уже в develop), и task закрывается с тестами-усилениями.

## 3. Развилка для Шифу (рекомендация архитектора)

### Вариант X — закрыть issue без кода

Действие: ничего не менять, issue #3004 закрыть как «уже исправлено в `149f755f8`, тест-покрытие есть».

- **Плюс**: минимальное усилие, нулевой риск регрессии.
- **Минус**: если живой код Vision Pi старый — проблема остаётся в проде до следующего CI-образа.
- **Когда выбирать**: если `148f755f8..88be6cf15` НЕ содержит — пропустить фикс, или если живой e2e-тест на develop уже зелёный.

### Вариант Y — зафиксировать регрессионный тест жёстче + закрыть

Действие: добавить в `src/rob_box_voice/test/unit/core/test_music_guard.py` дополнительные проверки, прямо моделирующие acceptance #3004:

```python
class TestIssue3004MusicStartingTools:
    """Issue #3004 — compose_music MUST short-circuit Bug C (issue #992)."""

    def test_compose_music_alone_terminates_music_guard(self):
        """Issue #3004 acceptance #1: turn with successful compose_music
        → Bug C must never fire. Regression guard for commit 149f755f8
        (capability-flags extraction) — without this, someone could add
        a new compose_music-class tool and forget to mark it starts_music."""
        guard = MusicGuard()
        verdict = guard.evaluate(
            was_dj_auto=False,
            user_input="сыграй ебучим басом Smells Like Teen Spirit",
            tools_called=("compose_music",),
            dj_enabled=False,
        )
        assert verdict.kind is MusicGuardVerdictKind.SKIP
        assert verdict.reason == "executed"

    def test_eight_consecutive_compose_music_all_skip(self):
        """Issue #3004 acceptance #2: 8 turns подряд с успешным compose_music
        → ни одного Bug C / USER_RETRY. PR #3004 фиксирует «не повторяй»."""
        guard = MusicGuard()
        for i in range(8):
            verdict = guard.evaluate(
                was_dj_auto=False,
                user_input="сыграй ебучим басом Smells Like Teen Spirit",
                tools_called=("compose_music",),
                dj_enabled=False,
            )
            assert verdict.kind is MusicGuardVerdictKind.SKIP, (
                f"iter {i+1}: Bug C fired ({verdict.kind}/{verdict.reason}); "
                "see issue #3004"
            )
            assert verdict.reason == "executed"

    def test_no_user_retry_after_compose_music(self):
        """Issue #3004 acceptance #3: counters MUST stay at 0 после
        успешного music — иначе следующий failed turn начнёт retry
        с не-нулевого счётчика (live 14:49 показывал счётчик 7/8)."""
        guard = MusicGuard()
        # Prime budgets by simulating prior failures.
        guard._dj_retry_count = 2
        guard._user_retry_count = 7
        verdict = guard.evaluate(
            was_dj_auto=False,
            user_input="сыграй ебучим басом Smells Like Teen Spirit",
            tools_called=("compose_music",),
            dj_enabled=True,
        )
        assert verdict.kind is MusicGuardVerdictKind.SKIP
        assert guard.dj_retry_count == 0
        assert guard.user_retry_count == 0
```

- **Плюс**: жёсткая регрессия, невозможна ситуация когда capability-флаг слетит с compose_music в будущем.
- **Минус**: доп. ~30 строк теста, нужен второй проход pytest.
- **Когда выбирать**: всегда полезно. **Рекомендация архитектора — выбирать вариант Y**.

### Вариант Z — закрыть + добавить issue про e2e-test на живой голос

Действие: отдельная карточка для backend/инженера — **e2e test, который проверяет «сыграй X» → только 1 `compose_music` в логе** (не 8). Сценарий:

```
## e2e
voice_text: "Робот, сыграй ебучим басом Smells Like Teen Spirit"
voice_file: .github/e2e/voice_commands/rabot_play_smells_like_teen_spirit.ogg
volume: 150
record_seconds: 90
llm: minimax-m3
tts: minimax-male-qn-qingse
stt: yandex
```

Acceptance: в `docker logs voice-assistant` через 60 сек после команды должно быть **ровно 1** вызов `compose_music` для этого запроса, не более. Если >1 — e2e FAIL.

- **Плюс**: живой ground-truth, ловит регрессии в проде.
- **Минус**: e2e-flaky (LLM может задуматься), нужно отдельное время на настройку.
- **Когда выбирать**: когда вариант Y недостаточен (например, проблема в аранжировщике, не в guard'е).

## 4. Что сделано в этой карточке (24.09)

- ✅ Прямая runtime-проверка гипотезы через `python3 -c "MusicGuard()..."` — guard возвращает SKIP для compose_music.
- ✅ Прогон существующих тестов `pytest test_music_guard.py` — 64 passed.
- ✅ Анализ git history (`149f755f8`, `209c16b73`) — фикс уже в develop.
- ⏳ Решение Шифу: вариант X / Y / Z.
- ⏳ Реализация — после решения.

## 5. Чего НЕ делает архитектор

По `rob-box-process-rules` §«Architect-protection: AI не пишет свои инструкции молча» и §«Architect writes analysis to issue — worker implements»:

- Не правит код руками (фикс уже есть).
- Не создаёт руками kanban-карточку для воркера (это сделает triage по `hermes` метке после решения Шифу).
- Не мерджит ничего сам (Q22).
- Не запускает `gh workflow run` руками — даже для проверки фикса (по 19.08 lesson, см. process-rules).

## 6. Follow-up вопросы (когда Шифу ответит)

1. **Какой develop-коммит реально на Vision Pi?** `88be6cf` или новее? (проверяется в один docker exec, см. §2 гипотеза A).
2. **8 одинаковых compose_music** — это был один «сыграй Smells Like Teen Spirit» или несколько разных команд в одной сессии? (может это разные turn-ы с разными user_input).
3. **Есть ли шанс, что фикс уже закрыл это, и Шифу помнит прошлое?** Тогда accept закрыт по факту, тест-усиление необязательно.
