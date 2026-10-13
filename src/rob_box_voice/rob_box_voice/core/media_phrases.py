"""media_phrases.py — фразы о запуске музыки: одна таблица (ADR-0148 §2.1, issue #3455).

Фразу «музыка пошла» / «не заиграла» строит КОД по событию ``started`` плеера и результату тула: роутер
медиакоманд (:mod:`.media_router`), заказ по имени (:mod:`.named_play`) и ход LLM (:mod:`.music_turn`)
импортируют шаблоны отсюда.

Живой прогон 06.10 06:56Z: на «сыграй интерстеллар сэт» модель сама сказала дословно фразу кода
«Включаю диджей-сет. Тема — интерстеллар.» с ``tools=[]`` — дважды подряд, музыки не было. Откуда она её
знает: ход роутера ложится в историю модели парой «реплика → фраза кода»
(``AgentCore.record_external_turn``, #3165), а тулы этого хода — только строкой в шапке «выполнено в
прошлых ходах». Для модели это пример «на просьбу о сете отвечают такой фразой», и она его копирует.
Гарды #2549/#2559 фразу не ловят: в их словарях глаголов нет настоящего времени «включаю».

:func:`honest_launch_reply` сверяет ответ модели с ОСНОВАМИ шаблонов этого модуля (не свободный регекс):
есть основа, а тула запуска в этом ходе нет — наружу уходит :data:`NOT_LAUNCHED_TEXT`.
TEMP(ADR-0148): проверка ответа LLM после хода — временная заплатка, пока ход роутера учит модель
фразе успеха; карточка на удаление — #3461. Ход, в котором тул запуска БЫЛ (``dj_set``/``request_music``),
она не трогает: там речь о музыке решает код по результату тулов (:mod:`.music_turn`).
#3565: фразу запуска роутера в окно модели больше не кладут (``_record_media_turn``), а просьбы о сете и о пьесе по
имени без запуска закрывает :func:`.music_turn.turn_reply` по словам человека. Гард остаётся только для хода БЕЗ
такой просьбы (вопрос, болтовня, «сыграй что-нибудь грустное»), где модель сама сказала «ставлю…» с ``tools=[]``:
узнать это можно лишь по тексту ответа. Снять его — когда такие ходы закроет структура (#3461).
"""

from __future__ import annotations

from typing import Any, Callable, Iterable, Mapping, Optional, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.instruments import neighbors


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


def piece_text(played: Mapping[str, Any]) -> str:
    """Что сыграно по заказу одной пьесы — из поля ``played`` результата ``request_music`` (ADR-0157 §2.3, R6):
    название из каталога/индекса партитур, источник и замена. LLM текст про пьесу не сочиняет.

    Название — из каталога с автором («Toccata», Bach); ``source=rtttl`` — играем одну мелодию: нот для полной
    аранжировки нет (отзыв Шифу 12.10: «мелодию-рингтон» непонятно); ``replacement_for`` — названного не нашлось,
    играем ближайшее. Все фразы — с основой «ставлю «» (:data:`LAUNCH_PHRASE_STEMS`, гард #3455)."""
    title = f"«{played.get('title') or ''}»"
    composer = str(played.get("composer") or "")
    if composer and composer.lower() not in title.lower():
        title = f"{title}, {composer}"
    asked = str(played.get("replacement_for") or "")
    if asked:
        return f"«{asked[:1].upper() + asked[1:]}» не нашёл. Ставлю ближайшее: {title}."
    if played.get("source") == "rtttl":
        return f"Ставлю {title} — мелодией, нот для полной аранжировки нет."
    return f"Ставлю {title}."


def launch_text(phrase: str, data: Mapping[str, Any]) -> str:
    """Фраза после ``started``: заказ пьесы (поле ``played``) — :func:`piece_text`, иначе ``phrase``; плюс фраза об
    инструменте (:func:`with_instrument`)."""
    played = data.get("played")
    return with_instrument(piece_text(played) if isinstance(played, Mapping) else phrase, data)


def instrument_text(data: Mapping[str, Any]) -> str:
    """Что сказать об инструменте заказа по результату ``request_music`` (поля ``instrument``/``instrument_played``,
    ADR-0157 §2.4, решение Шифу В2): синт есть — «Играю на тромбоне.»; нет — «Для балалайки у меня нет пака, могу
    сыграть на гитаре или на арфе — играю на гитаре.» Соседи — из таблицы ``knowledge.INSTRUMENTS``, не от LLM.
    Инструмент не просили — ``""``."""
    asked = kn.INSTRUMENTS.get(str(data.get("instrument") or ""))
    played = kn.INSTRUMENTS.get(str(data.get("instrument_played") or ""))
    if asked is None:
        return ""
    if played is asked:
        return f"Играю на {asked.prepositional}."
    if played is None:
        return f"Для {asked.genitive} у меня нет пака — играю своим звуком."
    others = " или ".join(f"на {kn.INSTRUMENTS[k].prepositional}" for k in neighbors(str(data["instrument"])))
    return f"Для {asked.genitive} у меня нет пака, могу сыграть {others} — играю на {played.prepositional}."


def with_instrument(phrase: str, data: Mapping[str, Any]) -> str:
    """Фраза запуска и фраза об инструменте (:func:`instrument_text`) одной репликой."""
    return " ".join(p for p in (phrase, instrument_text(data)) if p)


#: ADR-0149 PR-6: плеер отказал (``rejected``), ``started`` не пришло или тул не прошёл.
DJ_FAIL_TEXT = "Не получилось включить диджей-сет — музыка не заиграла."
REQUEST_FAIL_TEXT = "Не получилось включить музыку."

#: Код отказа ``request_music`` (``rob_box_mcp_tools.engine.tools_v2.SET_PLAYING``): сет запущен в этом же
#: ходе и играет, заказ его не снял. Равенство кодов держит ``test_music_turn_set_vs_request``.
SET_PLAYING_REASON = "set_playing"
#: Что сказать после фразы о сете, когда заказ поверх него отклонён (:data:`SET_PLAYING_REASON`).
SET_KEPT_TEXT = "Другую музыку поверх сета не ставлю."

#: Код отказа ``request_music`` на «продолжай играть Вагнера» (#3599): всё, что есть у этого автора, уже играло.
#: Текст отказа — ``"<код>: <фраза>"`` (:func:`no_more_text`, :func:`no_more_phrase`): роутер говорит фразу кода как есть.
NO_MORE_REASON = "no_more"


def no_more_text(asked: str, repeat: str) -> str:
    """Фраза кода, когда следующей пьесы автора нет: что просили (слова человека) и что можно повторить. Падеж слов
    человека не склоняется — название заказа в кавычках."""
    return (f"По запросу «{asked}» новых пьес больше нет, всё уже играло. "
            + (f"Могу повторить «{repeat}» или сыграть что-то другое." if repeat else "Могу сыграть что-то другое."))


def composer_missing_text(asked: str) -> str:
    """Фраза кода, когда в заказе назван композитор, а его пьес по запросу нет (#3604): чужое вместо него не играет.
    Слова человека в кавычках, не склоняются."""
    return (f"По запросу «{asked}» у названного автора ничего не нашлось, чужое вместо него не ставлю. "
            "Могу сыграть что-то другое.")


def no_more_error(phrase: str) -> str:
    """Текст отказа ``request_music`` с кодом :data:`NO_MORE_REASON` (его читает :func:`no_more_phrase`)."""
    return f"{NO_MORE_REASON}: {phrase}"


def no_more_phrase(content: str) -> str:
    """Фраза кода из текста отказа тула или ``""``, если это отказ по другой причине."""
    head = f"{NO_MORE_REASON}: "
    return (content or "")[len(head):].strip() if (content or "").startswith(head) else ""


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
    # piece_text (ADR-0157 PR-2): прогон 12.10 16:48 — модель без тула сказала «Токкату ре минор Баха ставлю
    # мелодией.», скопировав фразу кода из истории без кавычек. TEMP(ADR-0148) — снимается вместе с гардом (#3461).
    "мелодией",
)

#: Что сказать вместо фразы запуска без тула. Без глаголов действия (их ловят гарды #2549/#2559) и с
#: подсказкой, которую исполняет роутер без LLM.
#: Состояние плеера не утверждает: прошлый сет может играть (06.10 15:30 UTC), нового в этом ходе не было.
NOT_LAUNCHED_TEXT = "Новый сет я не начинал. Скажи: включи диджей сет на тему, и назови тему."


#: Просили пьесу/мелодию по имени, а в ходе ничего не заиграло (запуска нет, модель могла сказать «ставлю…»).
NOT_PLAYED_TEXT = "Ничего не заиграло — я мелодию не запускал. Назови, что сыграть, ещё раз."


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
    "DJ_FAIL_TEXT",
    "LAUNCH_PHRASE_STEMS",
    "LAUNCH_TOOLS",
    "NOT_LAUNCHED_TEXT",
    "NO_MORE_REASON",
    "NOT_PLAYED_TEXT",
    "REQUEST_FAIL_TEXT",
    "SET_KEPT_TEXT",
    "SET_PLAYING_REASON",
    "composer_missing_text",
    "dj_started_text",
    "honest_launch_reply",
    "instrument_text",
    "launch_claim",
    "launch_text",
    "no_more_error",
    "no_more_phrase",
    "no_more_text",
    "piece_text",
    "play_ok_text",
    "request_ok_text",
    "with_instrument",
]
