"""Диктовку факта сохраняет код, не модель (#5128, ADR-0148).

Живой n206 (марафон №3, 22.11): «робот запомни про меня борис болеет за спартак…» —
``actual_tool_calls: []``, в ``voice_facts`` пусто, а робот сказал «Запомнил». Правило
промпта модель не исполнила; ловить форму её ответа регексом — гонка и ложные срабатывания
(«Запомнил, тебя зовут Саша», «запомнил двоих»). Что человек продиктовал, код знает из его слов.

Код кладёт в команду обычный ``ToolCall`` тула ``knowledge.MEMORY_SAVE_TOOL``. Дальше — тот же
путь, что у вызова модели: исполнитель привяжет спикера снимка (``memory.bind_person``),
MCP позовёт ``VoiceMemory.save_fact``. Второй писатель в ``voice_facts`` не заводится.

Тул уже в команде или исполнен в ходе — код молчит: ноль дополнительных записей.
Модуль чистый: без ROS и I/O.
"""

from __future__ import annotations

from dataclasses import replace
from types import MappingProxyType
from typing import Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.words import done_claims, has_form, name_tokens, recording_claims, strip_wake_word

#: Ведущее слово диктовки. Прошедшее «запомнил» — ответ, не команда: его нет в таблице.
_LEAD = "запомни"
#: «Запомни меня как Саша» — самопредставление (``self_intro``), не факт.
_INTRO_MARK = "как"
#: «Запомни меня» без хвоста-факта — просьба запомнить человека (``register_speaker``), не факт.
#: Имя в этой реплике необязательно: его называет команда регистрации (#4184).
_REMEMBER_SELF = "меня"
#: «Запомни точку/место» — точка навигации (``save_waypoint``), не факт о человеке.
_PLACE_WORDS = frozenset({"точка", "точку", "точки", "место", "места"})
#: «Запомни, что я сказал» / «запомни это» — мета, «держи в уме»: сохранять нечего.
_META_TAILS = frozenset({"это", "то"})
_META_SAID = frozenset({"я сказал", "что я сказал", "я говорила", "что я говорила"})
#: После этих оборотов хвост — факт дословно. Более длинный оборот проверяется первым.
_PREFIXES = (("про", "меня"), ("что",), ())
#: Вейк, которым STT начинает диктовку. Таблица хода по умолчанию пуста (ADR-0167 §2.4).
_WAKE = ("робот",)


def _saves(call: ToolCall) -> bool:
    return call.tool == kn.MEMORY_SAVE_TOOL


def _tail(tokens: Tuple[str, ...]) -> Optional[str]:
    """Хвост после «запомни», если оборот — диктовка, иначе ``None``.

    «про меня» и «что» снимаются. «меня как» — представление, не факт: проверяется
    до голого хвоста, иначе «как» останется в факте. Голый хвост из «это»/«то»
    или «я сказал» — мета, не факт.
    """
    if not tokens or tokens[0] in _PLACE_WORDS or (len(tokens) >= 2 and tokens[1] == _INTRO_MARK):
        return None
    if tokens == (_REMEMBER_SELF,):  # «запомни меня»: регистрация человека, факта нет (#4184)
        return None
    for prefix in _PREFIXES:
        if tokens[: len(prefix)] != prefix:
            continue
        rest = tokens[len(prefix) :]
        if not rest:
            return None
        said = " ".join(rest)
        if said in _META_TAILS or said in _META_SAID:
            return None
        return said
    return None


def dictated_fact(text: str) -> Optional[str]:
    """Продиктованный факт дословно (нижний регистр, ё → е) или ``None``.

    Ловит «запомни про меня: …», «запомни, что …», «запомни: …» и те же обороты без
    двоеточия: живой n206 пришёл без него. Не ловит вопрос («что ты про меня помнишь»),
    самопредставление («запомни меня как …»), просьбу запомнить человека без факта
    («запомни меня»: имя скажет регистрация, #4184), прошедшее («запомнил…») и мету
    («запомни, что я сказал», «запомни это»).
    """
    heard = strip_wake_word(text or "", _WAKE)
    if not has_form(heard, (_LEAD,)):
        return None
    tokens = name_tokens(heard)
    at = tokens.index(_LEAD)
    if at != 0:
        return None
    return _tail(tokens[1:])


def owed(text: str, actions: Sequence[ToolCall], acted: Sequence[ToolCall]) -> Optional[str]:
    """Факт, который код должен сохранить, или ``None``.

    Диктовки нет — нечего. ``memory_save`` уже в команде или исполнен в ходе — ноль
    дополнительных записей, даже если текст факта у модели другой.
    """
    fact = dictated_fact(text)
    if fact is None or any(_saves(call) for call in (*actions, *acted)):
        return None
    return fact


def save_call(fact: str) -> ToolCall:
    """Вызов записи факта: аргументы те, что ждёт схема ``memory_save``.

    Категория — умолчание тула (``general``): код не классифицирует. Спикера подставит
    исполнитель из снимка, не из текста факта.
    """
    return ToolCall(kn.MEMORY_SAVE_TOOL, MappingProxyType({"fact": fact, "category": "general"}))


def with_dictated(text: str, cmd: Command, acted: Sequence[ToolCall] = ()) -> Tuple[Command, str]:
    """Команда с записью продиктованного факта и имя добавленного тула.

    Пустая строка — добавлять нечего: диктовки нет, либо тул уже в ходе. Слова модели
    эта функция не подменяет: что звучит при успехе диктовки, решает :func:`saved`.
    """
    fact = owed(text, cmd.actions, acted)
    if fact is None:
        return cmd, ""
    return replace(cmd, actions=(*cmd.actions, save_call(fact))), kn.MEMORY_SAVE_TOOL


def claimed(speech: Optional[str]) -> bool:
    """Речь модели утверждает запись факта.

    Прошедшее — ``DONE_CLAIMS["remember_fact"]``, та же сверка, что у валидатора:
    «Запомнил, Борис: …» — да, «Привет, Саша!» — нет. Настоящее — ``RECORDING_CLAIMS``
    (#5215): «Записываю: …» звучит после тула и при отказе выдаёт несделанное за сделанное.
    В ``DONE_CLAIMS`` его нет: фраза плана кода «Запоминаю: {items}» теми же словами законна
    (#4403). Регекса по ответу нет.
    """
    said = speech or ""
    return "remember_fact" in done_claims(said) or "remember_fact" in recording_claims(said)


def saved(text: str, results: Sequence[object], speech: Optional[str] = None) -> Optional[object]:
    """Успех записи, чью фразу говорит код, или ``None``.

    Человек продиктовал факт, либо речь модели утверждает запись, и ``memory_save``
    завершился простым успехом с фактом в данных события — этот результат. Фраза
    строится из него (``memory.saved``), не из слов модели: модель дописывает в
    подтверждение то, чего человек не говорил (#5166). Ни диктовки, ни утверждения —
    ``None``: «Привет, Саша!» рядом с честным ``memory_save`` не дублируется
    «Запомнил» (``QUIET_WITH_SPEECH_TOOLS``). Отказ и успех с причиной — тоже ``None``:
    о них говорит код, а ложное утверждение гасит :func:`drop_speech`.
    """
    if dictated_fact(text) is None and not claimed(speech):
        return None
    return next((r for r in results if _plain_save(r)), None)


def drop_speech(text: str, speech: Optional[str], results: Sequence[object]) -> bool:
    """Слова модели о записи не звучат: успех уже сказал код, либо запись не вышла.

    Успех — :func:`saved`. Отказ, провал и срок вышли при утверждении записи — тоже
    (#5215): «Записываю: …» при ``rejected{no_person}`` выдаёт несделанное за сделанное,
    и диктовка тут не исключение — отказ уже сказал код своей фразой. Реплика, которая
    записи не утверждает («расскажу, как дойти»), звучит рядом с отказом как раньше
    (#5172).
    """
    if saved(text, results, speech) is not None:
        return True
    return bool(_save_failed(results) and claimed(speech))


def _save_failed(results: Sequence[object]) -> bool:
    """``memory_save`` хода закрылся неуспехом: отказ, провал или срок вышел."""
    return any(
        getattr(r, "tool", None) == kn.MEMORY_SAVE_TOOL
        and getattr(r, "status", None) in ("rejected", "failure", "timeout")
        for r in results
    )


def unkept_claim(speech: Optional[str], actions: Sequence[ToolCall], acted: Sequence[ToolCall] = ()) -> bool:
    """Речь утверждает запись факта, а ``memory_save`` нет ни в команде, ни в исполненном хода (#5128).

    Та же сверка, что у валидатора: «Запомнил, Борис: …» при пустых действиях — неправда.
    Диктовка сюда не доходит с пустой командой: хук хода уже положил тул, и утверждение
    держит он. Фразу вместо лжи говорит код (``command.done_claim``), модель второй раз
    не зовётся: повтор моста этот случай уже отдал, а тест хода мост не зовёт.
    """
    if not claimed(speech):
        return False
    return not any(_saves(call) for call in (*actions, *acted))


def _plain_save(result: object) -> bool:
    """Простой успех ``memory_save`` с фактом в данных события владельца."""
    data = getattr(result, "data", None) or {}
    return (
        getattr(result, "tool", None) == kn.MEMORY_SAVE_TOOL
        and getattr(result, "status", None) == "done"
        and not getattr(result, "reason", None)
        and bool(str(data.get("fact") or "").strip())
    )


def salvage_with_dictated(
    text: str, salvage: Sequence[ToolCall], acted: Sequence[ToolCall] = ()
) -> Tuple[ToolCall, ...]:
    """Допустимые действия невалидной команды плюс запись продиктованного факта.

    Повтор моста не помог — диктовку всё равно сохраняет код.
    """
    fact = owed(text, salvage, acted)
    return (*salvage, save_call(fact)) if fact is not None else tuple(salvage)
