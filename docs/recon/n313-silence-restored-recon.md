# Recon: n313_silence_restored verbal-only LLM answer

**Task:** t_06ebf45c (kanban), issue **#2347**, CI run
[`34522852773`](https://github.com/krikz/rob_box_project/actions/runs/34522852773/job/103024356145)
(`L: E2E Voice Test`, develop @ `5e670940`, 2026-09-10).

## 1. Acceptance-факты (ADR-0018 raw)

Из `gh run view 34522852773 --log` (atomic harness):

```
>>> STEP n312_stop_music: ... voice=anton text="Робот, всё, спасибо, выключай музыку, курьер уехал."
>>> STEP n312_stop_music: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
>>> ACCEPTANCE[n312_stop_music]: ✅ all checks passed
>>> STEP n312_stop_music: ✅ acceptance PASS
>>> E2E_STEP n312_stop_music OK
>>> === STEP n313_silence_restored (safe=n313_silence_restored): voice=anton text="Робот, теперь тихо?" ===
>>> STEP n313_silence_restored: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
>>> ACCEPTANCE[n313_silence_restored]: ❌ expected tool calls not invoked: ['get_music_state']
>>> STEP n313_silence_restored: ❌ проверка не прошла после retry (см. /tmp/e2e_v2_34522852773/acceptance.json)
>>> E2E_STEP n313_silence_restored FAIL
>>> [hint] GATE-1 soft-fail: voice-cycle OK but tool skipped — get_music_state
>>> GATE-1: ❌ expected tool calls not invoked during run: get_music_state; voice cycle completed
         (TTS finished x13, speak_text x228) but expected tool call(s) skipped: get_music_state.
         LLM сделал verbal-only answer (RULE #MUSIC для stop_music / RULE #VOICE-MULTI для
         multi-voice могут не enforce). Проверь master_prompt_compact.txt и/или добавь explicit
         tool-call enforcement в LLM-system reminder.
```

**Подтверждение:**
- n313 user utterance **литерально** = `"Робот, теперь тихо?"` (аргумент `voice=anton text=` шага).
- stop_music был вызван **в предыдущем шаге того же рана** — `n312_stop_music` прошёл
  `✅ all checks passed`, атомарный цикл `акцепт + LLM + TTS`. Контекст правдоподобный.
- LLM в n313 не вызвал `get_music_state` → verbal-only ответ подтверждён GATE-1.

## 2. File:line map (anchor для fix-карточек)

| Что | Путь | Строки |
| --- | --- | --- |
| `_build_dynamic_system_context` (XML `<system_context>` с user/hardware/tts_context) | `src/rob_box_voice/rob_box_voice/dialogue_node.py` | `3070` |
| `_build_music_state_snapshot` (снимок `<music_state>` для всех источников) | `src/rob_box_voice/rob_box_voice/dialogue_node.py` | `3016` |
| Точка встраивания `<music_state>` в system_context | `src/rob_box_voice/rob_box_voice/dialogue_node.py` | `3174` |
| Существующий reminder «если играет — вызови stop_music» (зеркало для нового `get_music_state`) | `src/rob_box_voice/rob_box_voice/dialogue_node.py` | `3179-3185` |
| `RULE #MUSIC` (текущий про `stop_music`, не про `get_music_state`) | `src/rob_box_voice/prompts/master_prompt_compact.txt` | `274-278` |
| Tools table (строка `stop_music()` — место, где должен появиться `get_music_state()`) | `src/rob_box_voice/prompts/master_prompt_compact.txt` | `357-361` |
| `get_music_state — что играет прямо сейчас` (уже описано) | `src/rob_box_voice/prompts/skills/composer.txt` | `12-13` |
| Тот же «стоп если играет» reminder | `src/rob_box_voice/prompts/skills/composer.txt` | `14-16` |

## 3. Где не хватает enforcement

Существующий system-reminder (`dialogue_node.py:3179-3185`) описывает только ветку
`stop_music`. Для n313 (`"теперь тихо?"`) LLM должен сначала узнать состояние через
`get_music_state`, но ни:

- `RULE #MUSIC` (`master_prompt_compact.txt:274-278`),
- ни system-reminder (`dialogue_node.py:3179-3185`),
- ни skill-блок (`composer.txt:12-16`)

не говорят «на вопрос о текущем состоянии — сначала `get_music_state`». LLM решает
verbal-only «да, тихо» — отсюда GATE-1 fail.

**Предложение для fix-карточки (ничего не пишу, это out-of-scope recon):**
добавить зеркало существующего reminder'а на ветку «юзер спрашивает про состояние музыки» →
первый tool call = `get_music_state`.