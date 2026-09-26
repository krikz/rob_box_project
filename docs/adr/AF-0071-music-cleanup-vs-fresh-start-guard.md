# ADR-AF-0071: защита music vs cleanup (fresh-start окно)

Статус: accepted (verdict, ждёт Шифу) · 2026-09-25 · по разведке t_66d74380 · #935/#3005/#1812/#992

**Проблема.** Три cleanup (tts_batch_complete, watchdog/segments_deadline, new_session-guard) сносят музыку, начатую по запросу юзера в этом же turn'е, не различая «юзер просил» vs «DJ/system шум».

**Решение — одна переменная + 4 точки правки ≤10 строк каждая.**

1. `MusicManager._music_started_by_user_at: Optional[float]` (tools/music.py рядом с `_music_form_cycle_ends_at`, ~:543).
2. Запись при user-trigger: `tools/music.py:1358` (_schedule_stop), `:1367` (set_form_deadline, после `_music_form_deadline_at`), `:1387` (set_form_cycle_end).
3. Guard в watchdog: `tools/music.py:2014-2080` auto_stop_idle_music → если `_music_started_by_user_at` и `now - age < MUSIC_FRESH_START_TTL_S=30.0` → return `{"held_reason":"fresh_start_window",...}`. Константа рядом с `MIN_SEGMENTS_DEADLINE_SECONDS=60.0` (:373). TTL=30с покрывает «старт→стоп→старт» flap и типичный segments_deadline=60с (первая половина), а для экзотики `segments=2,bpm=180` (deadline 4с) — целиком.
4. Guard tts_batch_complete: `dialogue_node.py:3320-3350` (_on_tts_batch_complete) + `:6606-6623` (_flush_music_cleanup_if_idle) — если `manager.get_state().get("fresh_start_active")` → return. get_state уже есть (:1950); добавить `fresh_start_active` bool в него. Без новых ROS-топиков (KISS).

**Не трогаем:** stop_command_guard (явная воля юзера), DJ-mode (уже обнуляет дедлайн, music.py:2067-2072), Bug C-retry (LLM-уровень, не cleanup).

**Trade-off.** +5 строк MusicManager, +6 watchdog, +2 в каждом set_form, +6 dialogue (poll). Никаких новых топиков/state-машины. Деградация: через 30с watchdog возвращается к норме — при segments_deadline=60с теряется только самая ранняя часть; при segments=2/bpm=180 — фикс полный.

**Acceptance.** (1) «Робот, сыграй Баха» → tts_batch_complete через 5с → музыка живёт ≥30с (`🎵 cleanup SKIPPED (fresh-start, age=Xs)`). (2) Через 31с watchdog снова стопает по idle_ttl/segments_deadline. (3) Явный «стоп» <1с (Bug F не сломан). (4) Юнит: `_schedule_stop(segments=4,bpm=120)` → `auto_stop_idle_music` через 5с → `{"held_reason":"fresh_start_window","age_seconds":5}` без stop_reason; через 31с — обычная логика; при `segments=2,bpm=180` — stop не происходит.

**Executor file:line.** music.py `:373` (конст) → `:543` (поле) → `:1358/:1367/:1387` (запись) → `:2014-2080` (guard) → `:1950` get_state (expose `fresh_start_active` bool). dialogue_node.py `:3320-3350` + `:6606-6623` (guard). Не трогать: `stop_command_guard` (`:7106` _publish), `_reset_session_music_and_dj` (`:5155`).
