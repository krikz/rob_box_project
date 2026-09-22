"""Issue #2760 — recover tool calls the model WROTE instead of made.

Причина, а не симптом
---------------------

Живой робот, прогон 35704637846 (акт 2, шаги ``n204``/``n206`` — оба про
сохранение факта). Модель — **MiniMax-M3**, ``provider=minimax``,
``mode=stream``, в запросе честно переданы 57 тулов. Ответ приходит
такой::

    content='<function_calls>\\n<invoke name="memory_save">\\n
             <parameter name="fact">Болеет за Спартак…</parameter>\\n
             <parameter name="speaker_id">05ff0881-…</parameter>\\n
             </invoke>\\n</function_calls>'
    tool_calls=()  finish_reason='stop'

То есть модель приняла решение и сформировала аргументы, но отправила
вызов не в канал function-calling, а в текст. Проверено, что это НЕ наша
сторона:

* в промптах (``prompts/``, ``rob_box_core``) такой разметки нет вообще —
  модели неоткуда её копировать как пример;
* агрегация стриминговых ``tool_calls``-дельт штатная (OpenAI wire
  format, ``providers/deepseek.py``) и на соседних ходах того же прогона
  возвращает настоящие вызовы;
* ``finish_reason='stop'``, ``truncated_tool_args=False`` — обрыва не
  было.

Что делал робот без этого модуля: текст уходил в TTS (юзер слышал теги)
и оседал в окне истории как реплика ассистента — и модель копировала его
ходом позже. Ретрай эту цену не отбивает: лишний round-trip, а намерение
модели известно точно.

Поэтому здесь — восстановление намерения. Новых прав это не даёт:
исполняются ТОЛЬКО тулы, которые мы сами предложили в этом же запросе
(``allowed``), с их же схемой и через тот же executor. Имя, которого в
``allowed`` нет, не исполняется — такой ответ уходит на общий ретрай.
"""

from __future__ import annotations

import re
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple

#: ``<invoke name="tool">…</invoke>``, включая префиксы ``minimax:`` /
#: ``antml:``. ``DOTALL`` — блок многострочный.
_INVOKE_RE = re.compile(
    r"<\s*(?:minimax:|antml:)?invoke\s+name\s*=\s*[\"']([^\"']+)[\"']\s*>(.*?)"
    r"<\s*/\s*(?:minimax:|antml:)?invoke\s*>",
    re.DOTALL | re.IGNORECASE,
)

#: ``<parameter name="key">value</parameter>`` внутри одного ``invoke``.
_PARAM_RE = re.compile(
    r"<\s*(?:minimax:|antml:)?parameter\s+name\s*=\s*[\"']([^\"']+)[\"']\s*>(.*?)"
    r"<\s*/\s*(?:minimax:|antml:)?parameter\s*>",
    re.DOTALL | re.IGNORECASE,
)


def _coerce(value: str, schema: Optional[Mapping[str, Any]]) -> Any:
    """Привести строку из XML к типу, который объявлен в схеме тула.

    В разметке ВСЁ приходит строкой, а наш executor валидирует аргументы
    по JSON-схеме: ``set_volume(steps="3")`` упёрся бы в проверку типа и
    ход всё равно пропал бы — только на шаг позже и с менее понятным
    диагнозом. Официальный ``parse_tool_calls()`` MiniMax делает ровно
    это же приведение по схеме (``docs/tool_calling_guide.md``).

    Неизвестный тип и неразбираемое значение остаются строкой: пусть
    валидация скажет об этом явно, чем мы молча подставим ``None``.
    """
    declared = (schema or {}).get("type")
    if declared in (None, "string"):
        return value
    if declared == "boolean":
        low = value.strip().lower()
        if low in ("true", "1", "yes"):
            return True
        if low in ("false", "0", "no"):
            return False
        return value
    try:
        if declared == "integer":
            return int(value.strip())
        if declared == "number":
            return float(value.strip())
        if declared in ("array", "object"):
            import json

            return json.loads(value)
    except (TypeError, ValueError):
        return value
    return value


def _flat(name: str) -> str:
    """Имя без подчёркиваний — для сверки с ``allowed``.

    На стороне voice текст успевает пройти ``strip_markdown``, который
    снимает ``_..._`` парами через весь текст: в логе робота видно
    ``memorysave`` вместо ``memory_save``. Сюда, в harness, контент
    приходит ещё нетронутым, но сверка по нормализованному имени стоит
    ничего и закрывает случай, если порядок обработки когда-нибудь
    поменяется.
    """
    return name.replace("_", "").lower()


def parse_tool_call_markup(
    content: Optional[str],
    *,
    tools: Iterable[Mapping[str, Any]],
) -> Tuple[Tuple[str, Mapping[str, Any]], ...]:
    """Разобрать написанные текстом вызовы тулов.

    Args:
        content: текст ответа модели.
        tools: тулы в OpenAI-форме, предложенные модели в ЭТОМ запросе
            (``{"type": "function", "function": {"name", "parameters"}}``).
            Имя, которого здесь нет, не исполняется; схема оттуда же
            нужна для приведения типов (см. :func:`_coerce`).

    Returns:
        Кортеж пар ``(имя_тула, аргументы)`` в порядке появления.
        Пустой кортеж — восстанавливать нечего (или среди вызовов есть
        неизвестный, см. ниже).

    Если хотя бы одно имя не опознано, не восстанавливается **ничего**:
    частично исполненный ход хуже честного ретрая — юзер услышал бы
    подтверждение половины действия.
    """
    if not content:
        return ()
    by_flat: Dict[str, Tuple[str, Mapping[str, Any]]] = {}
    for spec in tools:
        fn = spec.get("function") or {}
        name = fn.get("name") or ""
        if not name:
            continue
        props = (fn.get("parameters") or {}).get("properties") or {}
        by_flat[_flat(name)] = (name, props)
    if not by_flat:
        return ()

    out: List[Tuple[str, Mapping[str, Any]]] = []
    for raw_name, body in _INVOKE_RE.findall(content):
        known = by_flat.get(_flat(raw_name.strip()))
        if known is None:
            return ()
        canonical, props = known
        args: Dict[str, Any] = {}
        for key, value in _PARAM_RE.findall(body):
            key = key.strip()
            args[key] = _coerce(value.strip(), props.get(key))
        out.append((canonical, args))
    return tuple(out)


__all__ = ["parse_tool_call_markup"]
