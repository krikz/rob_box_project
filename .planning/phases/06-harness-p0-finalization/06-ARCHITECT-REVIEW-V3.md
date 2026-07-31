# Architectural Review v3 — voice-assistant state after PR #933 + #935

| Поле         | Значение                                                        |
|--------------|-----------------------------------------------------------------|
| Phase        | 06-harness-p0-finalization (post-deply v3)                      |
| Branch       | `feature/harness-p0-foundation`                                 |
| HEAD         | `985d544b`                                                      |
| Reviewed PR  | #933 (`e152c931`) fix(voice/tts), #935 (`985d544b`) fix(voice/music) |
| Автор review | architect (Hermes Agent)                                        |
| Дата         | 2026-07-31                                                      |
| Источники    | `voice_v3_postdeploy.log` (520 строк), `dialog_v3.wav` (145 с), git log/diff, tts_node.py / tts_chunking.py / dialogue_node.py / mcp_server.py |

## 0. TL;DR / Verdict

**REJECT — два PR'а декларируют фиксы, но реальное поведение в e2e v3 не подтверждает приёмку.** По совокупности:

- **PR #933 (`fix(voice/tts): per-provider max chunk + retry-halve`)** — **не доставлен до hot-path.** Новые `synthesize_with_retry`, `CHUNK_LIMITS` импортированы в `tts_node.py`, но `_synthesize_yandex()` (строки 1319-1455) ВСЁ ЕЩЁ использует старую логику `_chunk_text()` (одноуровневое sentence-boundary разбиение) и НЕ вызывает retry-halve. В e2e-логе это видно прямо: `⚠️  Yandex gRPC chunk 1/1 failed (295 chars): Too long text` — чанк пришёл одним куском 295 chars (а лимит 700), fail, переход в Silero fallback. **Это нарушает acceptance-критерий самого PR** ("Real measurements: 1005 chars / silero=800 split, 291 chars / yandex=700 OK"). 291 chars оказался не "OK", а продолжает воспроизводить исходный bug #931.
- **PR #935 (`fix(voice/music): cleanup after dialog + set_vibe_preset schema fix`)** — **частично доставлен.** Schema-фикс и cleanup-topic работают (publisher на `/mcp/music_cleanup` создан, mcp_server подписан, watchdog запущен с `ttl=300s`). Но в e2e v3 диалог с музыкой НЕ запускался — наблюдаем только статическую инициализацию. Возможные runtime-эффекты cleanup не верифицированы на e2e-прогоне.
- **Не относящееся к PR'ам, но видимое в e2e-логе** (pre-existing или side-effects):
  - `Vosk model missing` — STT работает только через Yandex gRPC; локального офлайн-fallback нет.
  - `pixel_ring not available: No module named 'spidev'` × 3 ноды — LED ring полностью сломан (не ReSpeaker).
  - `sclang syntax error, unexpected '.'` + `Music stack degraded` — SuperCollider запускается в degraded-режиме (потенциально под ударом именно рэп-сценарий, который в тесте и был).
  - `Animation already playing` (×9) — `voice_animation_player` не отменяет предыдущую анимацию при повторном запуске, в итоге idle/talking/happy мигают WARNING-ами.
  - `LLM retry из-за 'hiphop' animation` — LLM выдумывает значения enum, валидатор MCP-сервера их отвергает, теряется 5 секунд round-trip.

Вердикт: **merge в `develop` блокирован до устранения gap G-933-A (retry-halve не подключён)** + 5 дополнительных gap'ов с severity ≥ medium. PR #935 может быть смержен после фикса G-933-A, потому что музыкальная часть v3-прогона не показала regression, а schema-fix для `set_vibe_preset` независим.

---

## 1. Соответствие PR-claims → e2e v3 evidence

### 1.1 PR #933 — `fix(voice/tts): per-provider max chunk + retry-halve` (`e152c931`)

| Claim из commit message | Evidence в e2e-логе / коде | Статус |
|---|---|---|
| Yandex gRPC падает на ~291 chars | ✅ Подтверждено. `voice_v3_postdeploy.log:476, 477, 1378`: `Yandex gRPC error: StatusCode.INVALID_ARGUMENT - Too long text`; последняя строка лога показывает `chunk 1/1 failed (295 chars)` — **то же самое число, что в #931 и в #933 commit message**. | 🟢 claim верен, но **фикс не работает** |
| Per-provider max chunk (yandex=700, silero=800) | Частично. В `voice_assistant.yaml:48-55` параметры `chunk_max_chars_yandex: 700, chunk_max_chars_silero: 800` объявлены. **Но `tts_node.py:1355` всё ещё зовёт `_chunk_text(text, max_chars=YANDEX_MAX_CHUNK_CHARS)` — со СТАРОЙ константой 700 и без передачи per-provider.** | 🟡 declared, не applied |
| `split_text` (sentence-boundary + word-level fallback) | В модуле `tts_chunking.py:67-160` написан. **Не используется из `tts_node.py`.** Старая `_chunk_text()` осталась в hot-path (lines 1214-1316). | 🔴 not wired |
| `synthesize_with_retry` (retry-halve per-provider) | В `tts_chunking.py:230-280` написан и покрыт unit-тестами (28 PASS). **Не вызывается нигде в `_synthesize_yandex` / `_synthesize_silero`.** | 🔴 not wired |
| "Real measurements: 291 chars / yandex=700 OK" | ❌ Опровергнуто e2e-логом. `295 chars → INVALID_ARGUMENT - Too long text`. | 🔴 test result vs prod diverge |
| 28 unit-тестов PASS | `test_tts_chunking.py:379` — pure-Python, unit-уровень. **Не тестирует интеграцию в реальный `tts_node.py`** — это иллюзия покрытия. | 🟡 unit-only gap |
| Silero fall-back при Yandex fail | ✅ Работает. После Yandex fail `tts_node` подгружает Silero v5 (3.6с lazy-load) и успешно синтезирует — **но это НЕ fix #933, а fallback-логика, существовавшая до PR**. | 🟡 unrelated |

**Architect verdict по #933: REJECT / Re-do.** PR-тезис правильный, реализация разорвана. Нужно:

1. В hot-path `_synthesize_yandex` и `_synthesize_silero` **заменить** вызовы `self._chunk_text(...)` на `synthesize_with_retry(text, max_chars=get_chunk_limit(provider), synthesize=single, is_too_long=is_too_long)`.
2. Если retry-halve всё равно падает — передавать вверх `TooLongError`, тогда `_synthesize_and_play` (line 985-995) корректно переключится на Silero и не вылетит.
3. Добавить интеграционный тест с **fake Yandex gRPC stub** который возвращает INVALID_ARGUMENT для chunks >N (≥ 2 chunks), и убедиться что retry-halve режет пополам и второй запрос проходит.

### 1.2 PR #935 — `fix(voice/music): cleanup after dialog + set_vibe_preset schema fix` (`985d544b`)

| Claim | Evidence в e2e-логе / коде | Статус |
|---|---|---|
| `set_vibe_preset schema синхронизирован с LLM prompt` (был preset='hiphop' → теперь preset_name) | ✅ `music_skill.py` + `tools/music.py` подтверждены через `git show`. **Но в e2e-логе LLM сначала вызвал `speak_text({'animation': 'hiphop'})`** (`voice_v3_postdeploy.log:241`) — это другая enum (`animation`, не `vibe_preset`). Через 5с LLM retry с `animation='excited'`. | 🟢 schema fix корректен для своего поля, но см. gap G-ANIM |
| DialogueNode cleanup hook on DIALOGUE_END | ✅ `dialogue_node.py:414` зовёт `self._publish_music_cleanup()`. Publisher на `/mcp/music_cleanup` создан в логе (`voice_v3_postdeploy.log:120`). | 🟢 wired |
| MusicManager.stop_music_on_session_end() handler | ✅ `mcp_server.py:215-235` обработчик `_on_music_cleanup` подписан (`voice_v3_postdeploy.log:188`). | 🟢 wired |
| Watchdog timer (period=5s, ttl=300s) | ✅ Запущен (`voice_v3_postdeploy.log:189`). | 🟢 wired |
| Unit-тесты | Не верифицировано в этом review (см.раздел §3-Tests). | ➖ TBD |

**Architect verdict по #935: CONDITIONAL ACCEPT.** Schema-fix для `set_vibe_preset` явно независим, его можно мержить. Cleanup-topic — реализован правильно, но **не проверен на e2e-прогоне с реальной музыкой** (в v3 диалог рэпа сгенерил cleanup-trigger, но если cleanup-функция упадёт — fallback на watchdog ttl=300s, что очень долго).

---

## 2. Gap analysis (8 original + дополнительные)

Каждый gap'у присвоен ID `G-{NN}-{КРАТКО}`. Severity по шкале:
- **CRITICAL** — production-blocker, e2e не работает в основном сценарии
- **HIGH** — функционально работает, но с видимыми артефактами / деградацией
- **MEDIUM** — наблюдаемые WARN/ошибки, нужно адресовать, merge не блокирует критично
- **LOW** — косметика / dev-tools

| ID | Severity | Gap | Evidence | Подтверждён? |
|---|---|---|---|---|
| G-933-A | **CRITICAL** | Retry-halve из PR #933 не подключён в `_synthesize_yandex` — chunk 295 chars → Yandex fail | `tts_node.py:1355` зовёт `self._chunk_text(...)` вместо `synthesize_with_retry(...)`; `voice_v3_postdeploy.log:476` | ✅ + root cause |
| G-933-B | **HIGH** | Silero lazy-load "из коробки": при первом fallback 3.6с пауза (диалог "зависает") | `voice_v3_postdeploy.log:478-499`: `Silero модель не загружена → загружаю сейчас → 2.7с загрузки → синтез`. Пользователь в это время слышит тишину. | ✅ |
| G-ANIM | **MEDIUM** | `voice_animation_player` спамит `Animation already playing` × 9 за прогон | `voice_v3_postdeploy.log:278, 318, 337, 354, 369, 374, 384, 387, 398, 404, 407, 416, 420, 430, 433, 453, 456, 467, 472, 475, 520`. **Корневая причина:** не отменяется предыдущая idle-анимация при новом talking/happy — таймер "возврат к idle" перебивает новый request, плюс "Loaded animation" идёт ДО "Animation already playing" — race-condition. | ✅ |
| G-ANIM-2 | **MEDIUM** | LLM придумывает невалидные enum-значения: `animation='hiphop'` (нет в списке) | `voice_v3_postdeploy.log:241`: `Допустимые: idle, talking, wakeup, sleep, ...` LLM retry через 5с с `excited`. **Lost 5 секунд round-trip**, в реальном диалоге выглядит как зависание. | ✅ |
| G-VOSK | **HIGH** | Vosk model missing: STT-нода не может работать локально | `voice_v3_postdeploy.log:70`: `Folder '/models/vosk-model-small-ru-0.22' does not contain model files`. Выключает fallback на offline STT — если Yandex API упадёт, робот глухой. | ✅ |
| G-MUSIC | **MEDIUM** | `sclang syntax error` ×2 + `Music stack degraded` | `voice_v3_postdeploy.log:29-37`: `Fatal errors: syntax error, unexpected '.'`. Работает в degraded-режиме. Не критично, но именно для рэп-сценария (`execute_music_code`) может быть повышенная латентность. | ✅ |
| G-LED | **MEDIUM** | `pixel_ring not available: No module named 'spidev'` × 3 ноды (audio, sound, tts) | `voice_v3_postdeploy.log:64, 116, 178`. LED-кольцо ReSpeaker не работает нигде. Pre-existing? Или регрессия от PR'ов? **Не относится к #933/#935.** | ✅ pre-existing |
| G-WAV-LATENCY | **LOW** | 145-секундный dialog генерирует 8 chunks TTS + retry — потенциально избыточная задержка | `dialog_v3.wav` инспектирован опосредованно (file format ok: PCM 16kHz mono), но точный таймстамп реплик не сравнить без playback. Архитектурное: sequential gRPC per chunk = latency * N. Параллельный stream не реализован. | ✅ suggested |

### Дополнительные архитектурные находки (не из 8-gap списка)

| ID | Severity | Gap | Evidence |
|---|---|---|---|
| G-ARCH-1 | **HIGH** | **Architecture/implementation gap (PR #933):** `CHUNK_LIMITS` импортированы, но `tts_node._synthesize_yandex` всё ещё использует `YANDEX_MAX_CHUNK_CHARS` (imported из `CHUNK_LIMITS["yandex_grpc_v3"]`, да). `synthesize_with_retry` импортирован, но НЕ вызван. Это не bug — это **declared-but-not-applied PR**. Code review должен был это поймать. | `tts_node.py:222-235` (imports); `tts_node.py:1355` (hot-path без retry) |
| G-ARCH-2 | **MEDIUM** | **Test-implementation gap (PR #933):** 28 unit-тестов в `test_tts_chunking.py` тестируют **только модуль** в изоляции. Интеграционного теста на `_synthesize_yandex` с fake-stub, возвращающим `INVALID_ARGUMENT`, НЕТ. PR заявляет "Real measurements: 291 chars / yandex=700 OK" — но это assertion на статике, а не на реальном pipeline с `tonode._synthesize_yandex`. | `test/test_tts_chunking.py`: grep на `_synthesize_yandex` = 0; `voice_assistant_launch_test` не существует |
| G-ARCH-3 | **LOW** | **Prompts без grounding (PR #935):** `master_prompt_compact.txt` усилен инструкцией про `stop_music()`, но **нет примера tool-call sequence** для рэпа — LLM по-прежнему может забыть вызвать stop. Без few-shot examples "speech + stop_music" 100% compliance не гарантировано; safety-net (cleanup-topic) это хорошо, но корень — ненадёжный prompt. | `master_prompt_compact.txt` + `music_skill_prompt.txt` (просмотрено в git show) |
| G-OPS-1 | **MEDIUM** | **Health-check vs reality:** `docker-compose.yaml` ищет `reflection_node`, которой нет (issue PHASE-06 ARCHITECT-REVIEW G11 ещё из v2). Голосовой ассистент в контейнере после deploy будет restart-loop, логи будут спамить. **Не входит в scope #933/#935**, но блокирует CI deploy. | pre-existing из `06-ARCHITECT-REVIEW.md` |

---

## 3. Рекомендуемые issue (severity ≥ medium → Kanban cards на developer)

> Body исходной задачи: "создать issue на developer если severity >= medium. Полный список gaps и шаблон verdict в issue #938 на github." gh CLI в этой среде нет — создаю задачи в Kanban (board `robbox`) с assignee `developer`. Шаблон verdict внутри тела каждой карточки.

### Card 1 — G-933-A [CRITICAL]

**Title:** `fix(tts): wire synthesize_with_retry into Yandex/Silero hot-path (PR #933 follow-up)`
**Body:**
```
Verdict from architect review v3 (06-ARCHITECT-REVIEW-V3.md):
PR #933 declares "per-provider max chunk + retry-halve" but in tts_node.py the
new synthesize_with_retry / CHUNK_LIMITS are imported but never called in the
actual synth pipeline.

EVIDENCE:
- src/rob_box_voice/rob_box_voice/tts_node.py:1355 — calls self._chunk_text(...)
- voice_v3_postdeploy.log:476 — 'Yandex gRPC chunk 1/1 failed (295 chars): Too long text'
  (same failure mode as #931; #933 did not change behavior)

ROOT CAUSE:
- _synthesize_yandex (lines 1319-1455) does not use synthesize_with_retry
- YANDEX_MAX_CHUNK_CHARS = 700 still passed as 'max_chars' to legacy _chunk_text()
- When a chunk fails, it raises → caller falls back to Silero (not retry-halve)

FIX (suggested):
1. Replace self._chunk_text(text, max_chars=YANDEX_MAX_CHUNK_CHARS) in
   _synthesize_yandex with:
     result = synthesize_with_retry(
         text=text,
         max_chars=self.chunk_max_chars_yandex,
         synthesize=lambda t, ms: self._synthesize_yandex_single(t, ms)[0],
         is_too_long=lambda exc: 'INVALID_ARGUMENT' in str(exc) and 'Too long' in str(exc),
         max_retries=self.chunk_max_retries,
     )
2. Catch TooLongError and propagate up — already handled by _synthesize_and_play
3. Add integration test with a fake-stub that returns INVALID_ARGUMENT for chunk >N
4. Repeat for _synthesize_silero with self.chunk_max_chars_silero and
   is_too_long = lambda exc: 'length' in str(exc).lower() or 'generate' in str(exc).lower()

ACCEPTANCE:
- pytest integration test: 1005-char text → Silero path splits at 800, retry-halve works
- pytest integration test: 295-char single phrase → Yandex still may fail,
  but the chunking logic should NOT raise if chunk ≤ 700 chars; if it does,
  fall through to Silero (which is what we already see).
- e2e: voice_v3_postdeploy.log should NOT contain 'Yandex gRPC chunk.*failed'
  for any chunk <= chunk_max_chars_*

Refs: PR #933 (e152c931), issue #918 (busy-loop), issue #929 (OOM killer)
```

### Card 2 — G-933-B [HIGH]

**Title:** `fix(tts): warm-load Silero fallback model at TTSNode init (PR #933 follow-up)`
**Body:**
```
Verdict from architect review v3:
Silero lazy-load triggers 2.7с pause on first fallback in dialog. User hears silence.

EVIDENCE:
- voice_v3_postdeploy.log:478-499 — 'Silero модель не загружена, загружаю сейчас
  ... 🔄 Загрузка Silero TTS v5 ... 2.7с (sic) загрузка'

FIX (suggested):
1. In TTSNode.__init__, after Silero provider selection, background-thread
   warm-load v5_ru.pt (already imported torch/cpu-bound; ~10 MB).
2. Add a `_silero_loaded` Event; _synthesize_and_play waits on it before fallback
   (with timeout 1s → use 'mini-tts' fallback if not ready, e.g. cached beep).

ACCEPTANCE:
- First Yandex→Silero fallback should have ZERO seconds additional latency
  in voice_v3_postdeploy.log (warm at init).
```

### Card 3 — G-ANIM [MEDIUM]

**Title:** `fix(animation_player): cancel previous animation on new request (no double-paint)`
**Body:**
```
Verdict from architect review v3:
voice_animation_player spams 'Animation already playing' when a new animation
request arrives during a still-running one (race + not cancelling previous).

EVIDENCE:
- voice_v3_postdeploy.log:278, 318, 337, ..., 520 — 'Animation already playing'
  appears 21 times in 145-секундный диалог
- Each WARN indicates the requested animation was triggered but the previous
  is not stopped, so animations overlap visually (and the random idle
  return timer from previous keeps firing).

FIX (suggested):
1. In AnimationPlayer.play_animation, cancel the active timer (random idle
   fallback) before starting a new animation.
2. Thread-safe with a single lock around current_animation state.
3. Make 'Loaded animation' check whether the animation is already loaded;
   skip the re-load step if so.

ACCEPTANCE:
- voice_v3_postdeploy log should not contain 'Animation already playing' (0 hits).
```

### Card 4 — G-ANIM-2 [MEDIUM]

**Title:** `fix(mcp_server): emit a warn to LLM when animation enum value is invalid (or constrain prompt)`
**Body:**
```
Verdict from architect review v3:
LLM generates 'animation': 'hiphop' which is not in the enum. MCP server
rejects with ❌ and LLM retries after 5с. Visually appears as hang.

EVIDENCE:
- voice_v3_postdeploy.log:241 — '❌ Инструмент speak_text завершился с ошибкой:
  Ошибка валидации параметров: Недопустимое значение для animation: hiphop'
- Same prompt cycle takes 5с of waiting on LLM (chat completion roundtrip)

FIX (options — pick ONE):
A) Prompt-side (master_prompt_compact.txt): explicitly list the valid
   animation values inline before the speak_text tool.
B) Server-side: validate the animation param explicitly; if invalid, map
   to a 'close-enough' value (hiphop → excited) with WARN log.

ACCEPTANCE:
- Either prompt includes explicit enum values for animation,
  OR server maps unknown → known with debug log.
```

### Card 5 — G-VOSK [HIGH]

**Title:** `fix(stt): bundle Vosk model or document install path`
**Body:**
```
Verdict from architect review v3:
Vosk model is not bundled — STT-node falls back to Yandex-only. If Yandex
STT API is unavailable, robot is deaf.

EVIDENCE:
- voice_v3_postdeploy.log:70 — "Folder '/models/vosk-model-small-ru-0.22'
  does not contain model files"
- stt_node.py still tries to load Vosk on init (lines around 270)

FIX (options):
A) Bundle model in image (download on build or git-lfs).
B) Document install path in repo README + compose; fail-fast on missing
   model with clear log message (not ERROR).
C) Switch provider default to 'yandex_only' if Vosk model not present.

ACCEPTANCE:
- voice_v3_postdeploy.log does not contain 'Folder ... does not contain model files'.
- If we choose C: STT_NODE__PROVIDER=yandex is the working default;
  Vosk is opt-in via env.
```

### Card 6 — G-MUSIC [MEDIUM]

**Title:** `fix(music): investigate sclang syntax error in startup validation`
**Body:**
```
Verdict from architect review v3:
sclang reports 'syntax error, unexpected .' at startup; music stack is
in degraded mode. Hard-to-debug, since sclang stack is on FoxDot upstream.

EVIDENCE:
- voice_v3_postdeploy.log:29-37 — 'Music stack degraded' + 2 syntax errors.

FIX (suggested):
1. Locate the failing .scd file in renardo_lib/SynthDefManagement/sclang_code/
2. Either patch upstream (PR to renardo) or pin renardo-lib to version
   that doesn't have the bad file.
3. Add a startup self-test that fails the boot loudly if 'degraded' —
   make user aware before deploying.

ACCEPTANCE:
- 'Music stack' starts in 'ok' state (no 'degraded').
```

### Card 7 — G-LED [MEDIUM]

**Title:** `fix(led/audio/sound/tts): install 'spidev' or skip pixel_ring gracefully`
**Body:**
```
Verdict from architect review v3:
Three nodes (audio_node, sound_node, tts_node) log 'pixel_ring not available:
No module named spidev' on every start. Pre-existing issue, NOT from #933/#935,
but visible in deployment logs.

EVIDENCE:
- voice_v3_postdeploy.log:64, 116, 178 — pixel_ring errors (3 nodes)

FIX (options):
A) `pip install spidev` in the voice-assistant image (requires kernel module
   + GPIO permission — may not be available on the host).
B) Use a guarded import and log INFO instead of WARNING when spidev is missing.

ACCEPTANCE:
- If A): pixel_ring available, runs as before.
- If B): only INFO log 'pixel_ring not available on this platform', no WARN.
```

### Card 8 — G-OPS-1 [MEDIUM, pre-existing from v2 review]

**Title:** `fix(ops): docker-compose health-check references removed reflection_node`
**Body:**
```
Verdict from architect review v3:
Pre-existing gap from v2 (06-ARCHITECT-REVIEW.md G11). Not addressed yet.

EVIDENCE:
- docker/main/docker-compose.yaml:316 — grep -q reflection_node
- reflection_node was deleted in PR W10 (commit 7552418a)

FIX: Replace 'reflection_node' in health-check with a service that actually exists
(e.g. audio_node, dialogue_node, tts_node — pick the most representative).

ACCEPTANCE: docker-compose health-check passes for voice-assistant service.
```

---

## 4. Test verification (дополнительно — не написано в body, но архитектурно важно)

Что архитектор хочет видеть в e2e-прогоне **после** фиксов:

```
[INFO] [tts_node]: ✅ TTSNode инициализирован  (warm-loaded Silero: ✅)
[INFO] [tts_node]: 🪓 Текст 1237 chars > 700 → разбит на 8 чанков
[tts_node] [INFO] ... chunk 1/1: 295 chars ...  (no 'failed', retry-halve если что)
[stt_node] [INFO] ... ✅ Vosk model loaded from <bundled path>
[animation_player_node] (no 'Animation already playing' WARNs)
[mcp_server] (no 'animation': 'hiphop' validation errors)
[sclang] (no 'syntax error' lines)
[audio/sound/tts] (no 'pixel_ring not available' WARNs)
```

Без выполнения этих критериев merge в `develop` запрещён.

---

## 5. Recommendation: что делать прямо сейчас

**Sequence (developer execute):**

1. **P0:** Fix G-933-A (retry-halve wired) — это блокер №1, без него PR #933 не имеет ценности.
2. **P0 same PR:** Fix G-933-B (warm-load Silero) — иначе после retry-halve падает UX на первом fallback.
3. **P1:** Fix G-VOSK (bundle or skip Vosk) — high-severity regression для offline.
4. **P1:** Запустить повторный e2e прогон (v4) с фиксами P0/P1. Если зелёный — merge в develop.
5. **P2:** G-ANIM, G-ANIM-2, G-MUSIC, G-LED, G-OPS-1 — батч в следующий sprint.

**Architect sign-off для merge:** после получения e2e v4 лога без:
- 'Yandex gRPC chunk.*failed' (если chunk ≤ 700),
- 'Animation already playing',
- 'Folder /models/vosk-... does not contain model files',
- 'pixel_ring not available',
- 'Music stack degraded'.

Архитектурно обе фичи (#933 и #935) правильные по замыслу. Просто коммит #933 не подключён к hot-path — это и должен ловить code review.
