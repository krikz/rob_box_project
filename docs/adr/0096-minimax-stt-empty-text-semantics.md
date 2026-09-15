# ADR-0096 — MiniMax STT: различать «пустой текст» и «нет поля text»

| Поле         | Значение                                                                |
|--------------|-------------------------------------------------------------------------|
| Статус       | Accepted                                                                |
| Дата         | 2026-09-15                                                              |
| Автор        | architect (Hermes Agent)                                                |
| Контекст     | Issue #2470 (review), PR #2369 (MiniMax STT Phase 1 PoC, merged), commit `490918d` |
| Расширяет    | ADR-0091 §7 (маппинг ошибок MiniMax → FallbackReason)                    |
| Связанные    | ADR-0091, ADR-0018, issue #2365, `docs/architecture/stt-provider-contract.md` §4.2 |

---

## 1. Контекст

Issue #2470 зафиксировал две связанные находки в
`src/rob_box_voice/rob_box_voice/stt_providers/minimax_provider.py`
(MiniMax STT provider, Phase 1 PoC):

### 1.1 Находка A — `_extract_text()` мапит валидный пустой результат на `None`

Файл: `stt_providers/minimax_provider.py:478-495`.

```python
def _extract_text(payload):
    ...
    if isinstance(text, str):
        return text.strip() or None   # ← пустая строка → None
```

В `transcribe()` это `None` превращается в `MiniMaxSTTInvalidResponseError`:

```python
text = _extract_text(payload)
if text is None:
    raise MiniMaxSTTInvalidResponseError(...)
```

То есть когда MiniMax возвращает `{"text": ""}` (валидный ответ «тишина
распознана как пусто»), мы raise'им исключение, логируем WARNING, и
`recognize()` отдаёт `None`. Это **семантически неверно** и **нарушает
ADR-0091 §7**, который явно фиксирует маппинг:

| HTTP-ответ | FallbackReason |
|---|---|
| HTTP 200 + `text=""` | `empty` (no retry) |
| HTTP 200 + valid text | `ok` |

То есть спецификация контракта уже говорит «пустой текст = empty»,
а реализация трактует это как `error`.

### 1.2 Находка B — мёртвая ветка в `recognize()`

Файл: `stt_providers/minimax_provider.py:397`.

```python
return response.text if response else None
```

`transcribe()` либо raise'ит `MiniMaxSTTError`-подкласс (тогда
ветка `except` отдаёт `None`), либо возвращает `MiniMaxSTTResponse`
(non-None dataclass). `response is None` недостижим, поэтому
`if response else None` — dead code.

### 1.3 Последствия

1. **На реальном e2e с тишиной** (например, wake-word ложно
   сработал, VAD прислал PCM без речи, 1с+ фоновый шум):
   - `MiniMaxSTTInvalidResponseError` спамит WARNING в логи
     каждый раз, когда ASR честно говорит «тишина».
   - Через `_handle_rejected_text` (issue 989 Fix A) это всё равно
     классифицируется как `empty`, и робот молчит (правильно),
     но **alert-fatigue** в логах маскирует настоящие ошибки.
2. **Тестовое покрытие дырявое**: `test_empty_text_is_none`
   (line 315) тестирует только хелпер, не полную цепочку
   `recognize()`.

### 1.4 Бизнес-проблема

Не критично для функциональности (робот не глючит), но:

- **Phase 2 wiring** (`t_99e504d2`, добавление MiniMax в реальную
  цепочку `vosk → minimax → yandex`) будет иметь **production-лог
  полный WARNING-ов на каждой тишине**, что сделает невозможной
  реальную диагностику (см. issue #1193 — MiniMax 402/429 alert-fatigue).
- **Мёртвая ветка** — code-smell, который сбивает с толку следующего
  ревьюера: «а что если `response` действительно `None`? Может тут
  какой-то баг?» Нет, не баг, просто LLM-копипаст.

---

## 2. Решение

### 2.1 Семантика трёх состояний `_extract_text`

Переопределяем контракт хелпера явно:

| Вход | Выход | Семантика |
|---|---|---|
| `{"text": "привет"}` | `"привет"` | OK, текст распознан |
| `{"text": ""}` или `{"text": "   "}` | `""` | OK, тишина распознана как пусто |
| `{"text": 42}` (но ключ есть) | `""` | OK, нестроковое значение → пусто (degraded, но не error) |
| `{}` или `{"foo": "bar"}` (ключа нет) | `None` | Invalid response |
| `"not a dict"` или `None` | `None` | Invalid response |
| `{"data": {"text": ""}}` (nested) | `""` | OK, зеркало-форма, тишина |

Ключевое изменение: «валидный ответ с пустым текстом» ≠ «нет ответа».
Различаем три уровня через optional-возврат:

- `Optional[str]` где `None` = «нет ключа text вообще» (raise).
- `""` = «текст есть, но пустой» (валидный результат).
- `non-empty str` = «текст распознан» (валидный результат).

### 2.2 `transcribe()` перестаёт raise'ить на пустом тексте

```python
text = _extract_text(payload)
if text is None:  # только когда ключа НЕТ
    raise MiniMaxSTTInvalidResponseError(
        f"minimax STT: missing 'text' in response: {payload!r}"
    )
# text — guaranteed str (может быть "")
```

`MiniMaxSTTResponse.text` остаётся `str` (frozen dataclass), но
**семантически допускает `""`**. Это согласовано с
`docs/architecture/stt-provider-contract.md` §4.2, который
говорит: «`text`: финальный распознанный текст (НЕ пустой при
`reason="ok"`). Гарантированно strip()».

### 2.3 `recognize()` упрощается до `response.text`

```python
def recognize(self, audio_bytes: bytes) -> Optional[str]:
    try:
        response = self.transcribe(audio_bytes)
    except MiniMaxSTTAuthError as exc:
        _log.warning("minimax STT: auth error (%s) — skipping", exc)
        return None
    except MiniMaxSTTRateLimitError as exc:
        _log.warning("minimax STT: rate-limited (%s) — falling back", exc)
        return None
    except MiniMaxSTTUnavailableError as exc:
        _log.info("minimax STT: unavailable (%s)", exc)
        return None
    except MiniMaxSTTError as exc:
        _log.warning("minimax STT: %s", exc)
        return None
    return response.text  # str, может быть ""
```

Мёртвая ветка `if response` удаляется. Возвращаемое значение
по-прежнему `Optional[str]`, но теперь `""` достижим и означает
«ASR честно сказал тишина» (через `select_recognition` это
классифицируется как `reason="empty"` и идёт на следующего
провайдера — это **корректное** поведение per ADR-0091 §7).

### 2.4 Тестовое покрытие

Добавить `TestRecognizeEmpty` (расширение `test_minimax_provider.py`):

- `test_recognize_with_empty_text_returns_empty_string` —
  полная цепочка: stub `200 {"text": ""}` → `recognize()` → `""`
  (НЕ `None`, НЕ exception).
- `test_recognize_with_whitespace_only_returns_empty_string` —
  `{"text": "   "}` → `""`.
- `test_transcribe_with_empty_text_returns_empty_response` —
  `transcribe()` возвращает `MiniMaxSTTResponse(text="")`,
  НЕ raise.
- `test_transcribe_with_missing_text_key_still_raises` —
  **negative test**: `{"foo": "bar"}` всё ещё raise'ит
  `MiniMaxSTTInvalidResponseError` (это инвариант).
- `test_no_warning_logged_on_empty_text` — `caplog` ловит
  WARNING; с пустым текстом НЕ должно быть WARNING
  (раньше был `MiniMaxSTTInvalidResponseError` WARNING).

Существующие тесты (`test_empty_text_is_none`) **обновляются** —
теперь ожидаемое поведение `_extract_text({"text":""})` это `""`,
не `None`. Это breaking в helper-контракте, но helper
приватный (`_`-prefix), не публичный API.

### 2.5 ADR-0091 §7 остаётся неизменным

Таблица маппинга в ADR-0091 §7 уже правильная. Этот ADR не правит
ADR-0091, а **исправляет реализацию**, чтобы она соответствовала
уже зафиксированному контракту. Trade-off анализ:

| Альтернатива | Плюс | Минус |
|---|---|---|
| A. **Ничего не делать** | 0 работы | Логи WARNING-спам в Phase 2; alert-fatigue (issue #1193 lesson) |
| B. Менять ADR-0091 (например, добавить «`text=""` → `error`») | «Валидный» с точки зрения текущего кода | Нарушает семантику STT-домена: тишина ≠ ошибка. Ломает ожидания dialogue_node и issue 989 Fix A |
| C. **Различать три состояния в helper (этот ADR)** | Минимальный, additive, исправляет и логи и dead code | Меняет сигнатуру `_extract_text` (private, OK) |

---

## 3. Границы решения

- **Не трогаем** `select_recognition` в `stt_fallback.py` —
  он уже корректно трактует `text is None or not text.strip()`
  как `empty` (line 230-235).
- **Не трогаем** ADR-0091 §7 (маппинг правильный с самого начала).
- **Не трогаем** `stt_node.py` (Phase 2 wiring — отдельная карточка
  `t_99e504d2`, после того, как Phase 1 перестанет спамить WARNING).
- **Не вводим** новый `FallbackReason` (типа `silent`); текущий
  `empty` уже корректно описывает ситуацию per ADR-0091 §7.
- **Не убираем** exception-классы; они нужны для typed-failure
  (`AuthError`/`RateLimitError`/etc.) и `transcribe()` API
  (для advanced callers, которые хотят различать типы ошибок).

---

## 4. Cross-references

- Issue #2470 (эта находка).
- Issue #2365 (MiniMax STT origin).
- Issue #989 (Fix A: rejected(empty) ≠ rejected(short)).
- Issue #1193 (alert-fatigue: lesson не плодить WARNING-спам).
- ADR-0091 §7 (маппинг FallbackReason — этот ADR делает реализацию
  соответствующей контракту).
- ADR-0018 (raw-evidence обязателен — pytest -v вывод будет в PR).
- `docs/architecture/stt-provider-contract.md` §4.2 (STTResult.text
  «НЕ пустой при reason=ok, гарантированно strip()» — наше `""`
  идёт с `reason=empty`, не `ok`, что согласуется).
