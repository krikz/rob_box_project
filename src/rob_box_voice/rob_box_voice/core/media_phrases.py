"""media_phrases.py — фразы об успехе запуска музыки: одна таблица (ADR-0148 §2.1, issue #3455).

Фразу «музыка пошла» строит КОД по событию ``started`` плеера (роутер медиакоманд, заказ по имени).
Шаблоны живут здесь, роутер (:mod:`.media_router`) и заказ по имени (:mod:`.named_play`) их импортируют.

Живой прогон 06.10 06:56Z: на «сыграй интерстеллар сэт» модель сама сказала дословно фразу кода
«Включаю диджей-сет. Тема — интерстеллар.» с ``tools=[]`` — дважды подряд, музыки не было. Откуда она её
знает: ход роутера ложится в историю модели парой «реплика → фраза кода»
(``AgentCore.record_external_turn``, #3165), а тулы этого хода — только строкой в шапке «выполнено в
прошлых ходах». Для модели это пример «на просьбу о сете отвечают такой фразой», и она его копирует.
Гарды #2549/#2559 фразу не ловят: в их словарях глаголов нет настоящего времени «включаю».

:func:`honest_launch_reply` сверяет ответ модели с ОСНОВАМИ шаблонов этого модуля (не свободный регекс):
есть основа, а тула запуска в этом ходе нет — наружу уходит :data:`NOT_LAUNCHED_TEXT`.
TEMP(ADR-0148): проверка ответа LLM после хода — временная заплатка, пока ход роутера учит модель
фразе успеха; карточка на удаление — #3461.
"""

from __future__ import annotations

from typing import Callable, Iterable, Optional, Tuple


def dj_started_text(persona: str, theme: str) -> str:
    """Фраза после ``started`` трека 1 сета (шаблон кода, I5)."""
    about = f" Тема — {theme}." if theme else ""  # «тема — X»: падеж слов человека не ломается
    return f"Я {persona}, включаю сет.{about}" if persona else f"Включаю диджей-сет.{about}"


def request_ok_text(name: str) -> str:
    """Фраза после ``started`` заказа «поставь клубный трек» (``request_music``)."""
    return f"Включаю {name}." if name else "Включаю музыку."


def play_ok_text(title: str) -> str:
    """Короткая фраза после успешного заказа мелодии по имени."""
    return f"Ставлю «{title}»."


#: Тулы, которые запускают музыку (движок v2, ``rob_box_mcp_tools.engine.tools_v2``): только они в ЭТОМ ходе
#: подкрепляют фразу запуска. Фраза в настоящем времени — действие этого хода, а не пересказ прошлого.
LAUNCH_TOOLS: frozenset = frozenset({"dj_set", "request_music"})

#: Неизменяемые части шаблонов выше — по ним узнаётся фраза кода в ответе модели
#: (``test_issue_3455_launch_phrase_honesty`` сверяет, что каждый шаблон содержит свою основу).
LAUNCH_PHRASE_STEMS: Tuple[str, ...] = (
    "включаю диджей-сет",  # dj_started_text без персоны
    ", включаю сет",  # dj_started_text с персоной
    "включаю музыку",  # request_ok_text без названия
    "ставлю «",  # play_ok_text
)

#: Что сказать вместо фразы запуска без тула. Без глаголов действия (их ловят гарды #2549/#2559) и с
#: подсказкой, которую исполняет роутер без LLM.
NOT_LAUNCHED_TEXT = "Музыка не играет — сет я не начинал. Скажи: включи диджей сет на тему, и назови тему."


def _norm(text: str) -> str:
    return " ".join(text.lower().replace("ё", "е").replace("-", " ").split())


def launch_claim(spoken: str) -> Optional[str]:
    """Основа шаблона запуска из :data:`LAUNCH_PHRASE_STEMS`, которая есть в ``spoken``, или ``None``."""
    low = _norm(spoken or "")
    return next((stem for stem in LAUNCH_PHRASE_STEMS if _norm(stem) in low), None)


def honest_launch_reply(
    spoken: str,
    tools_called: Iterable[str],
    *,
    log: Optional[Callable[[str], None]] = None,
    on_replace: Optional[Callable[[], None]] = None,
) -> str:
    """``spoken`` как есть или :data:`NOT_LAUNCHED_TEXT`, если это фраза запуска без тула запуска в ходе.

    ``on_replace`` — отозвать неправду из истории модели (иначе следующий ход повторит её, как 06.10).
    TEMP(ADR-0148) — #3461.
    """
    stem = launch_claim(spoken)
    if stem is None or LAUNCH_TOOLS & set(tools_called or ()):
        return spoken
    if log is not None:
        log(f"🎭 [issue 3455] фраза запуска музыки без тула ({stem!r} в {spoken[:80]!r}) — честная фраза")
    if on_replace is not None:
        on_replace()
    return NOT_LAUNCHED_TEXT


__all__ = [
    "LAUNCH_PHRASE_STEMS",
    "LAUNCH_TOOLS",
    "NOT_LAUNCHED_TEXT",
    "dj_started_text",
    "honest_launch_reply",
    "launch_claim",
    "play_ok_text",
    "request_ok_text",
]
