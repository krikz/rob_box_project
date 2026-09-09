# Checklist P1.4 — TelegramHarness v1 (source-of-truth)

**Назначение.** Этот чек-лист — single-source-of-truth для задачи
`t_a23d7bb0` (P1.4: TelegramHarness v1, TelegramCommandRegistry). Он
синхронизирован с ADR-0001 §2.7.3 + §2.8 + §2.9.3, refactoring-plan.md §5
(P1.5 в старой нумерации, P1.4 в task body), а также с фактическим
состоянием кода в `feature/harness-p0-foundation` (HEAD = `dbdf1fa8`).
Используется для понимания, что **уже сделано**, что **осталось**, и в
каком порядке делать оставшееся.

**Конвенции статусов.**
- ✅ — соответствует ADR-0001 и покрыто тестом.
- ⚠️ — частично: сделано «параллельно» (parallel implementation) или
  реализовано как заглушка, не выдерживает контракт ADR.
- ❌ — отсутствует или противоречит ADR.
- 🧪 — закрыто тестом, нужна лишь верификация.

---

## A. TelegramCommandRegistry (ADR §2.7.3 «Что переезжает внутрь»)

| # | Требование | Статус | Где живёт | Что не сделано |
|---|------------|:------:|-----------|----------------|
| A1 | `TelegramCommandRegistry.register(command, handler)` — declarative API как Flask routes | ✅ | `src/rob_box_harness/rob_box_harness/harnesses/telegram.py:90-114` | OK, sync-handler / coroutine оба поддержаны |
| A2 | `@command("name")` декоратор (Flask-стиль) | ❌ | — | Сейчас только `register("/start", handler)` |
| A3 | `dispatch(command, args, state) → str` (async) | ✅ | `telegram.py:116-148` | OK, ловит исключения, возвращает friendly-error |
| A4 | Handlers — pure functions `(ctx, args) → Effect`, не `(args, state) → str` | ⚠️ | `telegram.py:275-367` | Текущие handlers возвращают `str`; сигнатура `(args, state) → str`, не `(ctx, args) → Effect`. Нужно перейти к `(ctx, args) → Effect` для `SendReply`/`Speak` |
| A5 | Авто-генерация `Application.add_handler` из реестра | ❌ | — | Делается вручную в `init()`. Должен быть `add_telegram_handlers(registry, app)` |
| A6 | Декларативная таблица всех 25 команд (из `commands.py` 534 LOC) | ⚠️ | `telegram.py:397-405` | Только 9 placeholder-handler'ов. **534-строчный `commands.py` НЕ перенесён.** |
| A7 | Каждая команда: docstring + тест | ⚠️ | `test_telegram_harness.py:76-127` | Тесты на 4 команды (`/start`, `/help`, `/status`, `/stop`). На 5 команд и `/unknown` — позитивные кейсы. На остальные (5 из 9) тестов нет |

## B. TelegramHarness (ADR §2.7.3 «TelegramHarness»)

| # | Требование | Статус | Где живёт | Что не сделано |
|---|------------|:------:|-----------|----------------|
| B1 | `TelegramHarness(Harness[TelegramState])` | ✅ | `telegram.py:375-499` | OK |
| B2 | Порты: LLMProvider, ToolProvider, MemoryStore (shared с Dialog) | ✅ | `telegram.py:391-417` | Все 5 портов инициализируются через `super().init()` |
| B3 | Общий `MemoryStore` (не отдельный TelegramMemory) | ✅ | `telegram.py:467-487` | Scope = `"tg:{chat_id}"`, append_turn для user+assistant |
| B4 | Общий `FAQStore` (ADR §2.7.3 «voice_memory, faq_store → MemoryStore») | ❌ | — | **FAQStore как сущность в MemoryStore НЕ реализован.** Текущий `MemoryStore.save_fact` использует key/value, не FAQ-shape (нет question/answer/tags). |
| B5 | `SnapshotStore` (camera cache, порт) | ⚠️ | `telegram.py:216-267` | `SnapshotStore` существует, но НЕ привязан к `Transport`/фреймворку как порт; живёт внутри TelegramHarness. ADR §2.4.7/§2.8 — это должен быть **порт**, а реализация — `CameraCache` (TG-side) или `InMemorySnapshots` |
| B6 | `AuthMiddleware` (auth.py 89 LOC → middleware) | ✅ | `telegram.py:166-208` | OK; `wrap()` создаёт обёртку. Покрыт тестами (`TestAuth*`, 4 кейса) |
| B7 | `Harness.run(TelegramUpdate)` — каждый update как 1 turn | ✅ | `telegram.py:427-492` | OK; нормализация input (dict / str) |
| B8 | `TelegramTransport` — отдельный port | ⚠️ | `transport.py:48-60, 100-174` | `Transport` ABC уже определяет `on_telegram_update(TelegramUpdate)`, но `TelegramHarness.step()` принимает dict, а не `TelegramUpdate` dataclass. Нет моста dict→TelegramUpdate |
| B9 | `aclose()`-идемпотентность для всех портов | ✅ | `harness.py:215-240` | Наследуется от `Harness.teardown` |
| B10 | `init()` / `teardown()` / `run()` контракт | ✅ | `harness.py:142-240` | OK, контракт из ADR §2.3 |

## C. SideEffects (ADR §2.4.4, §2.7.3)

| # | Требование | Статус | Где живёт | Что не сделано |
|---|------------|:------:|-----------|----------------|
| C1 | `Effect.SendReply(channel, text)` — декларативный TG-reply | ❌ | — | **Отсутствует как Effect в `effects.py`.** В текущем коде reply возвращается строкой из `step()`, отправляется в TG-чат оркестратором снаружи. ADR §2.4.4 явно требует tagged union эффектов |
| C2 | `Effect.Speak(text, ssml)` — голос | ❌ | — | **Отсутствует в `effects.py`.** Только `LogEffect` и `EchoEffect` (212 LOC) |
| C3 | `Effect.PlaySound(name)` | ❌ | — | Отсутствует |
| C4 | `Effect.SetLED(pattern)` | ❌ | — | Отсутствует |
| C5 | `Effect.Move(vel)` | ❌ | — | Отсутствует |
| C6 | `SideEffectBus` маршрутизирует `SendReply` → TG, `Speak` → TTS | ⚠️ | `effects.py:114-200` | `NoopBus` / `RecordingBus` / `CompositeBus` есть. Конкретные bus'ы TTSBus / TelegramBus / SoundBus / LEDBus — отсутствуют (ADR §2.4.4 требует продакшен-конфигурацию) |
| C7 | `TelegramHarness.step` диспатчит `SendReply(text)` вместо `return text` | ❌ | `telegram.py:487` | Прямой `return str(response.content)`; `self.effects` ни разу не дёргается. Это нарушает §2.10 «Все внешние эффекты — через `SideEffectBus`» |
| C8 | `Speak` диспатчится, если юзер захотел голосом | ❌ | `telegram.py:330-339` | `/voice <text>` — заглушка, возвращает строку, а не dispatch |

## D. Voice ↔ Telegram мост (ADR §2.7.3 «Voice через TG идёт через отдельный skill»)

| # | Требование | Статус | Где живёт | Что не сделано |
|---|------------|:------:|-----------|----------------|
| D1 | `MemoryStore` общий для Dialog и Telegram | ✅ | `memory.py:33-48, 103-178` | OK, scope-based (`"tg:{chat_id}"` / `"voice:{user_id}"`) |
| D2 | `FAQStore` общий (помечен в ADR как «voice_memory, faq_store → MemoryStore») | ❌ | — | FAQStore отсутствует как concept, `MemoryStore.save_fact` слишком generic |
| D3 | `/say <text>` → `AgentSession.on_user_input(text, source=TG)` → TTS | ⚠️ | `telegram.py:330-339` | `/voice <text>` — заглушка. Нет `AgentSession`-style API в текущем `Harness` (`Harness.run` ≠ `on_user_input`); мост невозможен без P1.1 |
| D4 | `STTForTelegramSkill` (ADR §2.7.3 «отдельный навык STTForTelegramSkill — обёртка Yandex STT») | ❌ | — | P2.3 задача; **явно вне scope P1.4** |
| D5 | `MemoryStore` параметризуется по `scope: voice/telegram/both` (ADR §2.4.4) | ✅ | — | OK через `scope: str` параметр; `share_context` feature flag — нет, но P2.4 задача |

## E. e2e «Telegram → AgentSession → LLM → Reply» (Acceptance)

| # | Требование | Статус | Где живёт | Что не сделано |
|---|------------|:------:|-----------|----------------|
| E1 | `telegram_node.py` coverage ≥ 50% | 🧪 | `src/rob_box_telegram/test/test_commands.py` (146 LOC) + `test_mcp_bridge.py` (старая версия) | Измерим после `pytest --cov`; baseline 0% в ADR §1.1 |
| E2 | `TelegramHarness` coverage ≥ 80% на новый код | 🧪 | `test_telegram_harness.py` (305 LOC) | Только 305 LOC тестов, не измерено |
| E3 | e2e тест: update → registry → handler → LLM → MemoryStore → SendReply | ⚠️ | — | Есть `test_text_message` (вызывает `step({"text": "..."})` → LLM → return), но **SendReply в эффекты не диспатчится** |
| E4 | mypy strict-clean | 🧪 | `mypy.ini` | Проверим `mypy --strict src/rob_box_harness/.../telegram.py` |
| E5 | Устранить smells T1, T2, T3, T5, T7 (5 из 12) | ⚠️ | — | T1 (LLM-клиент дубль) ✅ решено LLMProvider. T2 (commands.py 534 строки) ⚠️ перенесено 9 placeholder, не все 25. T3 (нет моста к VoiceMemory) ❌ FAQStore. T5 (side-effects разбросаны) ❌ SendReply не Effect. T7 (STT отдельно) — P2.3 |
| E6 | TDD: RED → GREEN → REFACTOR на каждом behaviour | ⚠️ | — | Существующий код **написан ДО** тестов. Для P1.4 применим TDD заново (см. секцию «Дорожная карта») |

---

## F. Что осталось сделать (P1.4 scope, по приоритету)

### F.1 M0 — зафиксировать контракт (1-2 ч, нельзя дальше без этого)

1. **Расширить `effects.py` typed union**:
   - `SendReplyEffect(channel: str, text: str, reply_markup=None)` → `TelegramBus`
   - `SpeakEffect(text: str, ssml: str | None = None, voice: str | None = None)` → `TTSBus`
   - `PlaySoundEffect(name: str)`, `SetLEDEffect(pattern: str, color: str, duration_ms: int)`, `MoveEffect(linear: float, angular: float)` — чтобы контракт из ADR §2.4.4 был полным.
2. **Ввести порт `SnapshotStore`** (`src/rob_box_harness/.../snapshot_store.py`) с `put/get_latest` и TTL; `CameraCache` из TG — реализация.
3. **Мост `TelegramUpdate` dataclass ↔ dict** — единый парсер в `TelegramTransport.bind()`.

### F.2 M1 — TelegramCommandRegistry: declarative API + перенести handlers (1 день, TDD)

- Добавить декоратор `@registry.command("/name", description=...)`.
- Переписать handlers как `async (ctx, args) → Effect` (не `str`).
- Перенести handlers из существующего `src/rob_box_telegram/.../commands.py` (534 LOC) — **минимум 9 уже в реестре, довести до ≥15**.
- TDD: RED-тесты на каждую команду.

### F.3 M2 — TelegramHarness: интеграция `SideEffectBus` (1 день, TDD)

- `step()`:
  - `await self.effects.dispatch(SendReplyEffect(channel=chat_id, text=assistant_text))` вместо `return text`.
  - Для `/voice`: `await self.effects.dispatch(SpeakEffect(text=args))` + `SendReplyEffect(text=…)`.
- `MemoryStore` scope — параметризуемо из `config.harness.state["share_context"]`.
- `FakeBus` или `RecordingBus` в существующих тестах — заменить `return` на assert `bus.effects`.

### F.4 M3 — Voice ↔ Telegram общая память (1 день, TDD)

- Расширить `MemoryStore` методом `load_facts(scope, category)` / `save_faq(scope, question, answer, tags)` — типизированный FAQStore поверх существующего `save_fact`.
- Тест: один `MemoryStore` инстанс, два `TelegramHarness`-like harness'а с разными `scope` → общая видимость facts (если `share_context=true`).

### F.5 M4 — e2e тест + mypy + coverage (0.5-1 день)

- Новый `test_telegram_session.py` (из task body): `telegram update → AgentSession-style flow → SendReply через RecordingBus → assert`.
- `pytest src/rob_box_harness/test/test_telegram_harness.py -v` — все 305 LOC + новые.
- `mypy --strict --config-file src/rob_box_harness/mypy.ini src/rob_box_harness/.../telegram.py` — 0 issues.
- Coverage на новый код ≥ 80% (acceptance не gates, но в `pytest.ini` гейт снят по решению владельца в P1.1).

### F.6 M5 — коммит + PR

- Conventional commit `feat(harness): P1.4 TelegramHarness v1 declarative registry + side-effects`.
- `gh pr create` против `feature/harness-p0-foundation`.

---

## G. Что НЕ входит в P1.4 (явно)

- **D4 (STTForTelegramSkill)** — P2.3.
- **Полная миграция всех 25 команд** — частично: 9 placeholder + tests. Полный перенос — P2.4.
- **Multimodal vision через TG** — отдельная задача вне P1/P2.
- **Web dashboard для TG-истории** — вне scope харнесов.
- **Изменение ROS2-топиков** — инвариант.

---

## H. Решения, которые надо подтвердить с владельцем

1. **Handlers signature**: `(ctx, args) → Effect` vs `(args, state) → str`. ADR §2.7.3 говорит «декларативно: команда → skill», что предполагает возвращение Effect. Но текущий код возвращает str; нужно явно зафиксировать переход.
2. **FAQStore**: делать как метод `MemoryStore` (минимально-инвазивно) или как отдельный порт (чище)?
3. **Effects fan-out bus'ы (TTSBus, TelegramBus, SoundBus)**: реализовать в P1.4 или отложить в P2.4 (TelegramHarness v2)? Без них `SendReply` не достигнет TG API, и acceptance E3 не выполняется.
4. **Тест на TDD**: писать заново с RED-GREEN или «покрыть тестами» уже существующий код? Рекомендация: TDD на новые behaviour (M1, M2, M3), для существующих handlers — регрессионные тесты, чтобы acceptance зафиксировать.

---

**Версия чек-листа:** v1 (2026-07-28), привязан к feature/harness-p0-foundation @ dbdf1fa8.
**Поддерживается:** Kanban task `t_a23d7bb0`, owner — developer profile.
**Обновлять:** после каждого merge в feature/harness-p0-foundation, перед коммитом.
