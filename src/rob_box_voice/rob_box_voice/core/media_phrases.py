"""media_phrases.py — фразы о запуске музыки: одна таблица (ADR-0148 §2.1, issue #3455).

Фразу «музыка пошла» / «не заиграла» строит КОД по событию ``started`` плеера и результату тула: роутер
медиакоманд (:mod:`.media_router`), заказ по имени (:mod:`.named_play`) и ход LLM (:mod:`.music_turn`)
импортируют шаблоны отсюда.

ADR-0150 PR-7 ч.7d-4 (#3698): проверка ответа модели по основам этих шаблонов (``honest_launch_reply``, #3455/#3461)
удалена — в режиме команд речь при действиях не звучит, а о запуске говорит код по ``started`` плеера; просьбы о
сете и о пьесе по имени без запуска закрывает :func:`.music_turn.turn_reply` по словам человека. Основы шаблонов
(:data:`LAUNCH_PHRASE_STEMS`) остаются одной таблицей для аудита честности (``scripts/dialog/honesty_audit.py``).

Тексты фраз — в одной таблице ``rob_box_dialog/phrases/ru.yaml`` (ключи ``music.*``, ADR-0150 PR-1); здесь —
только сборка фразы по результату тула и событию плеера.
"""

from __future__ import annotations

from typing import Any, Mapping, Sequence, Tuple

from rob_box_dialog.phrases import text as _phrase
from rob_box_music import knowledge as kn
from rob_box_music.instruments import neighbors


def theme_text(theme: str) -> str:
    """«Тема — X.»: падеж слов человека не ломается; темы нет — «»."""
    return _phrase("music.theme", theme=theme) if theme else ""


def dj_started_text(persona: str, theme: str) -> str:
    """Фраза после ``started`` трека 1 сета (шаблон кода, I5)."""
    about = " " + theme_text(theme) if theme else ""
    head = _phrase("music.dj_started_persona", persona=persona) if persona else _phrase("music.dj_started")
    return head + about


def request_ok_text(name: str) -> str:
    """Фраза после ``started`` заказа «поставь клубный трек» (``request_music``)."""
    return _phrase("music.request_ok_named", name=name) if name else _phrase("music.request_ok")


def play_ok_text(title: str) -> str:
    """Короткая фраза после успешного заказа мелодии по имени."""
    return _phrase("music.play_ok", title=title)


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
        return _phrase("music.piece_replacement", asked=asked[:1].upper() + asked[1:], title=title)
    if played.get("source") == "rtttl":
        return _phrase("music.piece_melody", title=title)
    return _phrase("music.piece", title=title)


def launch_text(phrase: str, data: Mapping[str, Any]) -> str:
    """Фраза после ``started``: заказ пьесы (поле ``played``) — :func:`piece_text`, иначе ``phrase``; плюс фраза об
    инструменте (:func:`with_instrument`). Путь музыки (ADR-0158 PR-3, ``rob_box_music.source``): трек кэша — вся
    реплика кода ``source_message``; синтез/пьеса не того, что просили, — перед фразой ``source_note`` («сгенерировать
    сейчас не могу: …»)."""
    if data.get("source_message"):
        return str(data["source_message"])
    played = data.get("played")
    head = piece_text(played) if isinstance(played, Mapping) else phrase
    return with_instrument(" ".join(p for p in (str(data.get("source_note") or ""), head) if p), data)


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
        return _phrase("music.instrument_played", played=asked.prepositional)
    if played is None:
        return _phrase("music.instrument_no_pack", asked=asked.genitive)
    others = " или ".join(f"на {kn.INSTRUMENTS[k].prepositional}" for k in neighbors(str(data["instrument"])))
    return _phrase("music.instrument_neighbors", asked=asked.genitive, others=others, played=played.prepositional)


def with_instrument(phrase: str, data: Mapping[str, Any]) -> str:
    """Фраза запуска и фраза об инструменте (:func:`instrument_text`) одной репликой."""
    return " ".join(p for p in (phrase, instrument_text(data)) if p)


#: Ответ на «что играет?», когда дека пуста (роутер и ``get_music_state`` — одна фраза, #3613).
NOTHING_PLAYING_TEXT = _phrase("music.nothing_playing")
#: Трек поставлен на деку, а ``started`` ещё не пришло: о звуке не говорим (A14).
PENDING_TEXT = _phrase("music.pending")

#: Источник звучащего трека (``rob_box_mcp_tools.engine.tools_v2.SOURCE_*``) — словами для фразы «что играет».
SOURCE_WORDS: Mapping[str, str] = {k: _phrase(f"music.source.{k}") for k in ("synth", "piece", "melody", "generated")}


def _plural(n: int, one: str, few: str, many: str) -> str:
    if n % 10 == 1 and n % 100 != 11:
        return one
    return few if 2 <= n % 10 <= 4 and not 12 <= n % 100 <= 14 else many


def duration_text(seconds: float, accusative: bool = False) -> str:
    """«1 минута 5 секунд» (``accusative`` — «играет 1 минуту 5 секунд») — длительность для голоса: числа и
    склонение — код, не LLM."""
    total = max(0, int(round(seconds)))
    minutes, secs = divmod(total, 60)
    parts = []
    if minutes:
        parts.append(f"{minutes} {_plural(minutes, 'минуту' if accusative else 'минута', 'минуты', 'минут')}")
    if secs or not minutes:
        parts.append(f"{secs} {_plural(secs, 'секунду' if accusative else 'секунда', 'секунды', 'секунд')}")
    return " ".join(parts)


def now_playing_track_text(facts: Mapping[str, Any]) -> str:
    """«Что играет» про одиночный трек деки из фактов владельца плеера (``PlayerOwner.now_playing`` или поле ``dj``
    снимка + ``elapsed_s``): название как во фразе запуска, источник, темп, инструмент, сколько играет и сколько
    осталось (у пьесы — один проход формы; клубный трек повторяется до стопа). Фразу строит код (ADR-0148, #3613)."""
    if not facts.get("playing"):
        return PENDING_TEXT if facts.get("pending") else NOTHING_PLAYING_TEXT
    name = str(facts.get("name") or facts.get("title") or "")
    head = _phrase("music.now_playing_named", name=name) if name else _phrase("music.now_playing")
    if name and facts.get("composer"):  # автор — за кавычками, как во фразе запуска (piece_text)
        head += f", {facts['composer']}"
    about = _track_about(facts)
    parts = [head + (" — " + ", ".join(about) if about else "") + "."]
    if facts.get("elapsed_s") is not None and facts.get("source"):  # трек сета: время — у сета, не у трека
        parts.append(_track_time(facts))
    return " ".join(parts)


#: Сколько названий называть во фразе списка библиотеки (остальные — числом).
LIBRARY_LIST_NAMES = 5
LIBRARY_EMPTY_TEXT = _phrase("music.library_empty")
LIBRARY_NO_TRACK_TEXT = _phrase("music.library_no_track")
LIBRARY_FAIL_TEXT = _phrase("music.library_fail")


def library_list_text(titles: Sequence[str], total: int) -> str:
    """Фраза списка библиотеки генераций (``music_library(list)``, ADR-0158 PR-9): число и первые названия из кэша,
    не от LLM."""
    if total <= 0:
        return LIBRARY_EMPTY_TEXT
    names = list(dict.fromkeys(t for t in titles if t))[:LIBRARY_LIST_NAMES]  # варианты одного заказа — одно имя
    head = _phrase("music.library_list", total=total, tracks=_plural(total, "трек", "трека", "треков"))
    examples = ", ".join("«" + n + "»" for n in names)
    return head + _phrase("music.library_examples", names=examples) if names else head + "."


def library_rate_text(title: str, liked: bool) -> str:
    """Фраза после оценки трека кэша: запись в БД уже сделана (``music_library(rate)``)."""
    name = f"«{title}»" if title else "этот трек"
    return _phrase("music.library_liked" if liked else "music.library_disliked", name=name)


def library_delete_text(title: str) -> str:
    """Фраза после удаления трека из библиотеки."""
    return _phrase("music.library_deleted", name=f"«{title}»" if title else "трек")


def library_play_text(title: str, seconds: float) -> str:
    """Фраза после ``started`` трека из библиотеки генераций (``play_library_track``, ADR-0158 PR-2): название и
    длина — из кэша и файла, не от LLM."""
    name = f"«{title}»" if title else "трек"
    return _phrase("music.library_play", name=name, duration=duration_text(seconds))


def _track_about(facts: Mapping[str, Any]) -> list:
    """Источник, темп и инструмент трека — словами (части фразы «что играет»)."""
    about = [SOURCE_WORDS[str(facts["source"])]] if str(facts.get("source") or "") in SOURCE_WORDS else []
    if facts.get("bpm"):
        bpm = int(round(float(facts["bpm"])))
        about.append(f"{bpm} {_plural(bpm, 'удар', 'удара', 'ударов')} в минуту")
    lead = kn.INSTRUMENTS.get(str(facts.get("instrument_played") or ""))
    if lead is not None:
        about.append(f"на {lead.prepositional}")
    return about


def _track_time(facts: Mapping[str, Any]) -> str:
    """Сколько трек играет и сколько осталось (пьеса — один проход формы; клубный трек — по кругу до стопа)."""
    when = _phrase("music.playing_for", elapsed=duration_text(float(facts["elapsed_s"]), accusative=True))
    if facts.get("remaining_s") is not None:
        return when + _phrase("music.remaining", remaining=duration_text(float(facts["remaining_s"])))
    return when + _phrase("music.repeats")


#: ADR-0149 PR-6: плеер отказал (``rejected``), ``started`` не пришло или тул не прошёл.
DJ_FAIL_TEXT = _phrase("music.dj_fail")
REQUEST_FAIL_TEXT = _phrase("music.request_fail")

#: Код отказа ``request_music`` (``rob_box_mcp_tools.engine.tools_v2.SET_PLAYING``): сет запущен в этом же
#: ходе и играет, заказ его не снял. Равенство кодов держит ``test_music_turn_set_vs_request``.
SET_PLAYING_REASON = "set_playing"
#: Что сказать после фразы о сете, когда заказ поверх него отклонён (:data:`SET_PLAYING_REASON`).
SET_KEPT_TEXT = _phrase("music.set_kept")

#: Код отказа ``request_music`` на «продолжай играть Вагнера» (#3599): всё, что есть у этого автора, уже играло.
#: Текст отказа — ``"<код>: <фраза>"`` (:func:`no_more_text`, :func:`no_more_phrase`): роутер говорит фразу кода как есть.
NO_MORE_REASON = "no_more"


def no_more_text(asked: str, repeat: str) -> str:
    """Фраза кода, когда следующей пьесы автора нет: что просили (слова человека) и что можно повторить. Падеж слов
    человека не склоняется — название заказа в кавычках."""
    tail = _phrase("music.no_more_repeat", repeat=repeat) if repeat else _phrase("music.no_more_other")
    return _phrase("music.no_more", asked=asked) + " " + tail


def composer_missing_text(composer: str) -> str:
    """Фраза кода, когда у названного композитора нет ни одной пьесы (#3604): чужое вместо него не играет. Называет
    автора, а не цитирует реплику (#3608: в реплике бывают слова просьбы и мат); имя в кавычках, не склоняется."""
    name = composer.strip()
    who = _phrase("music.author_named", name=name[:1].upper() + name[1:]) if name else _phrase("music.author_unnamed")
    return _phrase("music.composer_missing", who=who)


#: Код отказа ``request_music``: назван композитор, а его пьес нет (#3604). Текст отказа - ``"<код>: <фраза>"``
#: (:func:`composer_missing_error`, :func:`composer_missing_phrase`): роутер говорит фразу как есть.
NO_COMPOSER_REASON = "no_composer"


def composer_missing_error(phrase: str) -> str:
    """Текст отказа ``request_music`` с кодом :data:`NO_COMPOSER_REASON`."""
    return f"{NO_COMPOSER_REASON}: {phrase}"


def composer_missing_phrase(content: str) -> str:
    """Фраза кода из текста отказа тула или ``""``, если это отказ по другой причине."""
    head = f"{NO_COMPOSER_REASON}: "
    return (content or "")[len(head):].strip() if (content or "").startswith(head) else ""


#: Код отказа ``request_music``/``generate_music``: чойсер пути не нашёл, чем играть просьбу (ADR-0158 PR-3: «спой»,
#: жанр без синтеза, а генерации нет). Текст отказа — ``"<код>: <фраза>"``; фразу строит ``rob_box_music.source``.
SOURCE_DECLINED_REASON = "music_source"


def source_error(phrase: str) -> str:
    """Текст отказа с кодом :data:`SOURCE_DECLINED_REASON` (его читает :func:`source_phrase`)."""
    return f"{SOURCE_DECLINED_REASON}: {phrase}"


def source_phrase(content: str) -> str:
    """Фраза отказа пути из результата тула (код может стоять не в начале: обёртки транспорта) или ``""``."""
    head = f"{SOURCE_DECLINED_REASON}: "
    at = (content or "").find(head)
    return content[at + len(head):].strip() if at >= 0 else ""


def no_more_error(phrase: str) -> str:
    """Текст отказа ``request_music`` с кодом :data:`NO_MORE_REASON` (его читает :func:`no_more_phrase`)."""
    return f"{NO_MORE_REASON}: {phrase}"


def no_more_phrase(content: str) -> str:
    """Фраза кода из текста отказа тула или ``""``, если это отказ по другой причине."""
    head = f"{NO_MORE_REASON}: "
    return (content or "")[len(head):].strip() if (content or "").startswith(head) else ""


#: Неизменяемые части шаблонов выше — по ним аудит честности (``scripts/dialog/honesty_audit.py``) узнаёт фразу
#: запуска в речи робота (``test_launch_phrase_stems`` сверяет, что каждый шаблон содержит свою основу).
LAUNCH_PHRASE_STEMS: Tuple[str, ...] = (
    "включаю диджей-сет",  # dj_started_text без персоны
    ", включаю сет",  # dj_started_text с персоной
    "включаю музыку",  # request_ok_text без названия
    "ставлю «",  # play_ok_text
    # piece_text (ADR-0157 PR-2): прогон 12.10 16:48 — модель без тула сказала «Токкату ре минор Баха ставлю
    # мелодией.», скопировав фразу кода из истории без кавычек.
    "мелодией",
)

#: Что сказать, когда просили музыку, а запуска в ходе не было (:mod:`.music_turn`). Без глаголов действия и с
#: подсказкой, которую исполняет роутер без LLM.
#: Состояние плеера не утверждает: прошлый сет может играть (06.10 15:30 UTC), нового в этом ходе не было.
NOT_LAUNCHED_TEXT = _phrase("music.not_launched")


#: Просили пьесу/мелодию по имени, а в ходе ничего не заиграло (запуска нет, модель могла сказать «ставлю…»).
NOT_PLAYED_TEXT = _phrase("music.not_played")


__all__ = [
    "DJ_FAIL_TEXT",
    "NOTHING_PLAYING_TEXT",
    "PENDING_TEXT",
    "SOURCE_WORDS",
    "duration_text",
    "LIBRARY_EMPTY_TEXT",
    "LIBRARY_FAIL_TEXT",
    "LIBRARY_NO_TRACK_TEXT",
    "library_delete_text",
    "library_list_text",
    "library_play_text",
    "library_rate_text",
    "now_playing_track_text",
    "LAUNCH_PHRASE_STEMS",
    "NOT_LAUNCHED_TEXT",
    "NO_COMPOSER_REASON",
    "NO_MORE_REASON",
    "NOT_PLAYED_TEXT",
    "REQUEST_FAIL_TEXT",
    "SET_KEPT_TEXT",
    "SET_PLAYING_REASON",
    "SOURCE_DECLINED_REASON",
    "source_error",
    "source_phrase",
    "composer_missing_error",
    "composer_missing_phrase",
    "composer_missing_text",
    "dj_started_text",
    "theme_text",
    "instrument_text",
    "launch_text",
    "no_more_error",
    "no_more_phrase",
    "no_more_text",
    "piece_text",
    "play_ok_text",
    "request_ok_text",
    "with_instrument",
]
