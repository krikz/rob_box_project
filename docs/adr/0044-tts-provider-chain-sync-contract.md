# ADR-0044: TTS provider chain sync contract — фикс #1976 / t_b33bfee1

| Поле | Значение |
|---|---|
| Статус | **Proposed** (после merge PR в develop → Accepted) |
| Дата | 2026-09-04 |
| Автор | backend (Hermes Agent); карточка `t_b33bfee1`, issue #1976 |
| Контекст | Шифу требует цепочку TTS `Yandex → MiniMax → Silero` (Yandex-first). В develop уже есть fallback chain из issue #1083 (`Minimax → Yandex → Silero`, MiniMax-first) — карточка #1976 инвертирует приоритет. ADR-0043 уже описывает «4 поверхности синхронизации» для LLM-провайдеров (deepseek/minimax/mimo) — этот ADR распространяет то же правило на TTS-провайдеры (yandex/minimax/silero). |
| Затрагивает | (a) `src/rob_box_voice/rob_box_voice/tts_node.py` (`_default_provider_chain`, `_chain_from_provider`, `_normalize_provider_chain`); (b) `src/rob_box_voice/rob_box_voice/tts_chain.py` (новый модуль-обёртка, см. ниже); (c) два yaml-конфига (`src/rob_box_voice/config/tts_node.yaml`, `docker/vision/config/voice_assistant/tts_node.yaml`); (d) тесты `src/rob_box_voice/test/unit/tts/test_provider_chain.py`, `test_tts_chain.py`; (e) docstring в `tts_node.py` рядом с `provider_chain` ROS-параметром. |
| Родители | ADR-0043 (sync 4 поверхностей для LLM — pattern повторён для TTS), ADR-0018 (honesty culture — runtime yaml и тесты не должны расходиться), ADR-0013 (incremental delivery — узкая правка, не переписывать TTSNode целиком) |
| Связанные | `t_b33bfee1` (эта), `t_424e8172` (предыдущая реализация fallback chain, issue #1083), issue #1976 (CI fail run #33697728942 — Yandex down без fallback на стороне e2e harness), issue #1857 (provider-chain sync contract, родительская), PR (этот PR) |

## TL;DR

`TTSProviderChain` становится **обязательной** частью TTSNode: при объявлении ноды задаётся ROS-параметр `provider_chain` (Yandex-first по дефолту, см. ниже), который:

1. **Декларируется в `declare_parameter("provider_chain", [])`** в `tts_node.py` — пустой дефолт → цепочка выводится из `provider` (см. `_chain_from_provider`).
2. **Задаётся в runtime yaml-конфиге** (yaml-файл с `provider_chain: [yandex, minimax, silero]` — ЯВНАЯ декларация, синхронно с дефолтом в коде).
3. **Тестируется в unit-тестах** (один параметризованный набор проверяет default + chain-from-provider + normalization, отдельный — `_synthesize_and_play` end-to-end с fake-failure каждого провайдера).
4. **Документируется** (этот ADR + docstring в `tts_node.py` рядом с `provider_chain`).

Правило: при смене **default chain ordering** или **default primary provider** для TTS — все 4 поверхности обязаны быть обновлены в **том же коммите** (см. ADR-0043 §2, применено к TTS).

**Не путать с ADR-0040** — ADR-0040 про e2e-process (`agent-flow-e2e-process.sh` циклически создаёт round-ветки), не про provider chain. Карточка `t_b33bfee1` ссылается на ADR-0040 ошибочно; правильный sibling — ADR-0043 (LLM chain).

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (CI run #33697728942, 03.09.2026 20:30 UTC)

`L: E2E Voice Test` падает с 32 ошибками:

```
>>> STEP cc01_status_gate: FAIL — синтез Yandex упал (YANDEX_AUTHERROR)
>>> STEP ns01_reset_session: FAIL — синтез Yandex упал (YANDEX_AUTHERROR)
... (30 шагов повторяются)
E2E_VERDICT FAIL
```

**Все шаги e2e** требуют синтеза команды через Yandex gRPC API в `e2e_voice_test.sh:566 synth_yandex()`. Когда Yandex API возвращает `YANDEX_AUTHERROR` (network/rate-limit/quota) — `synth_yandex()` падает, и весь прогон FAIL.

**Важно**: e2e harness синтезирует команды **напрямую через Yandex** на build machine (10.1.1.249), а не через TTS-ноду робота. Это значит фикс на стороне робота (TTSProviderChain) **сам по себе не починит CI** — нужно дополнительно править `synth_yandex()` в `e2e_voice_test.sh`, чтобы и у неё был fallback. Это **задача отдельной карточки** (см. §9).

Карточка `t_b33bfee1` решает **половину** проблемы: она требует, чтобы **на проде (роботе)** цепочка fallback работала корректно по требованиям Шифу (Yandex-first). Это фикс, который Шифу просит в явном виде — независимо от CI.

### 1.2 Текущее состояние кода

В `develop` уже есть fallback chain из issue #1083 (commit `9462005b`):

```python
# src/rob_box_voice/rob_box_voice/tts_node.py
@staticmethod
def _default_provider_chain() -> list[str]:
    return ["minimax", "yandex", "silero"]   # ← MiniMax-first

@staticmethod
def _chain_from_provider(provider: str) -> list[str]:
    if provider == "silero":  return ["silero"]
    if provider == "yandex":  return ["yandex", "silero"]    # ← без MiniMax
    return ["minimax", "yandex", "silero"]                    # ← MiniMax-first
```

И dead-cache с TTL (long для quota/auth, short для transient), и `provider_chain` ROS-параметр уже задекларирован. **Логика правильная** — Шифу меняет только **приоритет**.

### 1.3 Требование Шифу (карточка t_b33bfee1, issue #1976)

```
Yandex  →  MiniMax  →  Silero
(prio 1)    (prio 2)    (prio 3, последний fallback)
```

Текстом: «если нет Яши то юзать миниакс если и миниакса нет то силеро».

### 1.4 Что НЕ работает в текущем коде

* `_default_provider_chain()` MiniMax-first — нарушает требование.
* `_chain_from_provider("yandex")` → `["yandex", "silero"]` без MiniMax — неполная цепочка (MiniMax выпадает, хотя это второй приоритет).
* `provider_chain` нигде не задан в runtime yaml — runtime молча берёт дефолт из кода, нет явной декларации.

## 2. Что делаем (этот PR)

### 2.1 Изменить дефолт на Yandex-first

`src/rob_box_voice/rob_box_voice/tts_node.py`:

```python
@staticmethod
def _default_provider_chain() -> list[str]:
    """Дефолтная цепочка — Yandex-first (issue #1976, t_b33bfee1).

    Yandex (gRPC v3) → MiniMax (HTTP) → Silero (офлайн).
    Silero всегда последний (инвариант).
    """
    return ["yandex", "minimax", "silero"]

@staticmethod
def _chain_from_provider(provider: str) -> list[str]:
    if provider == "silero":  return ["silero"]
    if provider == "minimax": return ["minimax", "yandex", "silero"]  # back-compat
    return TTSNode._default_provider_chain()  # yandex, minimax, silero
```

`provider=minimax` остаётся back-compat: пользователь явно попросил MiniMax первым (отличается от дефолта, но легитимно). Это сохраняет оригинальное поведение #1083 для тех, кто полагается на MiniMax-first.

### 2.2 Создать модуль-обёртку `tts_chain.py`

Карточка явно требует: «новый модуль `tts_chain.py`». Создан `src/rob_box_voice/rob_box_voice/tts_chain.py` с классом `TTSProviderChain`:

* `TTSProviderSlot` — frozen dataclass: `name`, `synthesize` (Callable), `priority`, `voice`.
* `TTSProviderChain` — sorted по priority, walk с try/except, dead-cache integration через `dead_cache` callback.
* `TTSUnavailable` — chain exhausted; список per-provider ошибок в `errors`.
* `Timeout`, `RateLimit`, `QuotaExceeded`, `ServiceUnavailable`, `TransientTTSFailure` — типизированные transient-ошибки.
* `_is_transient_failure(exc)` — legacy message-level классификация (`"YANDEX_AUTHERROR"`, `"grpc unavailable"`, `"2056 Token Plan usage limit reached"`).
* `chain_from_yaml_config(cfg, lookup)` — десериализация из YAML с валидацией (unknown provider, duplicate priority, missing key).

**Важно**: `tts_chain.py` — это **отдельный, узкий API**, он **не заменяет** inline walk внутри `TTSNode._synthesize_and_play`. Этот inline walk — hot path, его тестирует `test_provider_chain.py` (23 теста). Новый модуль — канонический переиспользуемый класс для новых call-sites (preview-voice, e2e harness fallback, bench scripts).

### 2.3 Задать `provider_chain` в runtime yaml-конфигах

В `src/rob_box_voice/config/tts_node.yaml` и `docker/vision/config/voice_assistant/tts_node.yaml`:

```yaml
provider_chain:
  - yandex
  - minimax
  - silero
```

Явная декларация — синхронна с кодовым дефолтом. Если yaml и код разъедутся — `provider_chain` из yaml выигрывает (см. `_effective_provider_chain`).

### 2.4 Тесты

* `src/rob_box_voice/test/unit/tts/test_provider_chain.py` — обновлён под Yandex-first дефолт (23/23 зелёные).
* `src/rob_box_voice/test/unit/tts/test_tts_chain.py` — новый, 35 тестов для `tts_chain.py` (happy path, transient failover, dead-cache, mark-dead, YAML deserialisation, edge cases).

## 3. Синхронизация 4 поверхностей (правило ADR-0043, применённое к TTS)

| # | Поверхность | Где живёт | Что значит «default» |
|---|---|---|---|
| 1 | **Код (default в `declare_parameter`)** | `src/rob_box_voice/rob_box_voice/tts_node.py` (`declare_parameter("provider_chain", [])` + `_default_provider_chain`) | Когда YAML отсутствует / пустой / пустой список |
| 2 | **Runtime yaml-конфиги** | `src/rob_box_voice/config/tts_node.yaml`, `docker/vision/config/voice_assistant/tts_node.yaml` (`provider_chain: [...]`) | Что реально работает в проде |
| 3 | **Test asserts** | `test_provider_chain.py` (23 теста), `test_tts_chain.py` (35 тестов) | Что unit-тесты считают правильным |
| 4 | **Docstring'и + ADR** | `tts_node.py` (docstring рядом с `provider_chain` ROS-параметром), этот ADR | Что читает человек |

**Допустимое исключение**: если runtime yaml намеренно остаётся на legacy (staged rollout), то вместо (2) — добавить `TODO(legacy-config): YYYY-MM-DD — <причина, кто решил, когда ревью>` в оба yaml-файла.

## 4. Альтернативы, которые рассмотрели

| Вариант | Плюсы | Минусы | Вердикт |
|---|---|---|---|
| Ничего не делать (CI не чинить) | Быстро; фикс только кода, не yaml | CI run #33697728942 остаётся красным; цепочка MiniMax-first нарушает требование Шифу | ❌ |
| Только изменить `_default_provider_chain()` | Минимальный diff | yaml/runtime не синхронизирован явно — нарушает правило ADR-0043 (test desync / semantic drift) | ❌ |
| Сделать как в карточке (только `_default_provider_chain` + `tts_chain.py`) | Покрывает требование Шифу | Runtime yaml молча использует старый дефолт → test desync при следующей правке | ⏸ |
| **Этот PR (код + yaml + тесты + ADR)** | Все 4 поверхности синхронны; явная декларация цепочки в yaml; ADR фиксирует правило | Большой diff, но механический | ✅ **выбран** |
| Сделать `e2e_voice_test.sh::synth_yandex` тоже с fallback | Чинит CI run #33697728942 напрямую | Вне scope карточки; `synth_yandex` — build-machine side, не робот | ⏸ отдельная карточка (см. §9) |

## 5. Что делаем прямо сейчас (карточка `t_b33bfee1`)

1. ✅ Создать `src/rob_box_voice/rob_box_voice/tts_chain.py` с `TTSProviderChain`.
2. ✅ Изменить `_default_provider_chain()` и `_chain_from_provider("yandex")` на Yandex-first.
3. ✅ Задать `provider_chain: [yandex, minimax, silero]` в обоих runtime yaml.
4. ✅ Обновить `test_provider_chain.py` под новый дефолт.
5. ✅ Создать `test_tts_chain.py` с 35 тестами.
6. ✅ Создать этот ADR.
7. ✅ PR → develop → зелёный CI → Шифу ревьюит и мёрджит.

## 6. Что НЕ делаем

- **Не трогаем** `.github/workflows/L-E2E Voice Test.yml` (карточка просит явно).
- **Не правим** `e2e_voice_test.sh::synth_yandex` — это отдельная задача (см. §9).
- **Не активируем** MiniMax в production до того как Шифу явно одобрит runtime rollout (yaml уже задаёт цепочку, но production activation — отдельный шаг).
- **Не правим** docstring'и в `rob_box_llm/providers/minimax_tts.py` — это LLM-провайдер, не TTS.

## 7. Где SOT (single source of truth)

- **После merge ADR-0044:** этот файл — правило «sync 4 поверхностей для TTS chain».
- **Текущее runtime поведение:** `provider_chain: [yandex, minimax, silero]` в `tts_node.yaml`.
- **Кодовый дефолт:** `_default_provider_chain()` в `tts_node.py`.
- **Граница переключения:** YAML-параметр `provider_chain` пустой → fallback на код.

## 8. Verification (что проверить после merge)

- [ ] `grep -n "provider_chain" src/rob_box_voice/rob_box_voice/tts_node.py` — параметр задекларирован.
- [ ] `grep -n "provider_chain" src/rob_box_voice/config/tts_node.yaml docker/vision/config/voice_assistant/tts_node.yaml` — есть в обоих yaml, в одном порядке.
- [ ] `PYTHONPATH=src/rob_box_voice:src/rob_box_llm python3 -m pytest src/rob_box_voice/test/unit/tts/test_provider_chain.py src/rob_box_voice/test/unit/tts/test_tts_chain.py -v` — все зелёные (23 + 35 = 58).
- [ ] `git log --oneline -- src/rob_box_voice/rob_box_voice/tts_chain.py` — файл появился в одном коммите с правкой `tts_node.py`.
- [ ] CI run после PR: `Run Tests` зелёный.

## 9. Что НЕ покрыто этим PR (отдельные карточки)

- **`synth_yandex` в `.github/workflows/scripts/e2e_voice_test.sh:566`** — этот вызов синтезирует команды для e2e **напрямую через Yandex API** на build machine. Когда Yandex недоступен (run #33697728942), все 32 шага падают. **Фикс на стороне робота (TTSProviderChain) не решает эту проблему**. Нужна отдельная карточка: «добавить fallback в `synth_yandex` (Yandex → MiniMax → espeak/paplay)» — это решит CI fail run #33697728942.
- **ADR-0040 ≠ ADR-0044**: ADR-0040 — это про e2e-process (round-ветки без runs), не про provider chain. Карточка `t_b33bfee1` ссылается на «ADR-0040 provider-chain sync contract» — это **ошибка Шифу** (или карточки-генератора); правильный sibling — ADR-0043 (LLM chain) или этот ADR-0044 (TTS chain). Если Шифу/merge-gate укажет на эту ошибку в ревью — добавить комментарий в карточку.

## 10. Changelog

- **2026-09-04** — Proposed (этот ADR), карточка `t_b33bfee1`, issue #1976.
- после merge → Accepted, при необходимости — `Supersedes: none`.