"""«Эта песня» - ссылка на только что сочинённую, и подставляет её код, а не модель (ADR-0148).

Марафон-5 29.10 (n707/n708): робот сочинил «Енота-механика» (``[song] track=db2045…``), через полторы минуты человек
сказал «что за песня» и «включи эту песню про енота», а модель взяла ``track_id`` старой версии с тем же названием
из списка (``b90b61bf…``) - рассказала про вчерашний трек и включила его. Выбор трека по названию из списка - угадывание.

Правило кода: реплика называет песню указательным словом («эта/эту/этот … песню/трек») или «только что сочинённую», и
вызов ``music_library`` берёт id трека - тогда ему ставится ссылка ``track_ref=last_generated``. Сам id знает один владелец -
библиотека сочинённых треков (``GeneratedMusicLibrary.last_generated``: из двух вариантов заказа тот, что сыграл; свежесть - её
срок): тул подставляет его вместо ``track_id`` модели. Свежего сочинённого нет - ссылка пуста, ``track_id`` модели работает
как прежде (прежнее поведение). ``rate`` без id и так берёт играющий трек («этот трек нравится» про играющий) - ему ссылку
код не ставит; ``list``/``search`` id не берут.

Песню называют и темой (М11 05.11 n707: «расскажи про песню про енота» через полторы минуты после заказа «про
енота-механика» - модель позвала ``info`` по песне прошлого прогона ``3feb08b7``, а не по свежей ``e7dd8301``): реплика
говорит о песне (``песн``/``трек``…) и называет слово темы последнего заказа генерации этого разговора
(:class:`GeneratedTopic`: ``theme`` вызова ``generate_music``, что не провалился) - та же ссылка. Тема другая («про
кота»), песня названа старой («вчерашнюю», «другую») или заказа не было - вызов как был.
"""

from __future__ import annotations

import time
from types import MappingProxyType
from typing import Any, Callable, Optional, Sequence, Tuple

from rob_box_core.topic_words import topic_stems

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import ToolCall
from rob_box_dialog.words import name_tokens, strip_wake_word

#: Аргумент тула и его значение «последний сочинённый» (то же, что ``MusicLibraryTool`` принимает в схеме).
REF_ARG = "track_ref"
LAST_GENERATED = "last_generated"
#: Действия тула, где нужен id трека и «эта песня» - это последний сочинённый (``rate`` и ``play`` играющего - нет).
REF_ACTIONS: Tuple[str, ...] = ("info", "play", "update", "delete")
#: Указательные слова («эта песня», «эту песню про енота»): целые, строчные, без «ё».
_THIS = ("эта", "эту", "этой", "этот", "этого", "этом")
#: «Последняя песня» библиотеки ``music_library`` (там только сочинённое роботом) - та же ссылка; основа слова.
_LAST = "последн"
#: Основы существительных песни/трека; начало слова.
_NOUNS = ("песн", "трек", "композиц", "мелоди", "инструментал")
#: Сколько слов между указательным и существительным допустимо («эту новую песню», «этот твой трек»); дальше -
#: уже другая фраза («на этой неделе включи песню»).
_GAP = 1
#: «Только что сочинённая»: ``только что`` + глагол создания в окне ``_JUST_WINDOW`` слов; основы.
_MADE = ("сочин", "сгенер", "написа", "созда", "приду", "сделал", "спел", "пропел")
_JUST_WINDOW = 4
#: Сколько секунд после заказа генерации его тема - «только что сочинённая»: песня пишется ~90–150 с, и ещё
#: ``GENERATED_FRESH_S`` (900 с) тула она «эта». Дальше выбирает тул: свежего нет - ссылка пуста, ``track_id`` модели.
ORDER_FRESH_S = 1200.0
#: Слова о другой, не свежей версии («вчерашнюю песню про енота», «другую», «старую»): темой ссылка не ставится; основы.
_OLDER = ("вчераш", "позавчераш", "стар", "прошл", "прежн", "друг", "давн")


def refers_to_last_generated(text: str) -> bool:
    """Реплика называет песню, которую робот сочинил только что: «эта/эту песню», «этот трек», «последнюю песню»,
    «только что сочинённую»."""
    tokens = name_tokens(text)
    for i, token in enumerate(tokens):
        deictic = token in _THIS or token.startswith(_LAST)
        if deictic and any(w.startswith(_NOUNS) for w in tokens[i + 1:i + 2 + _GAP]):
            return True
        if token == "только" and tokens[i + 1:i + 2] == ("что",):
            if any(w.startswith(_MADE) for w in tokens[i + 2:i + 2 + _JUST_WINDOW]):
                return True
    return False


def names_generated_topic(text: str, topic: Sequence[str], wake_words: Optional[Sequence[str]] = None) -> bool:
    """Реплика говорит о песне и называет тему последнего заказа (``topic`` - основы :func:`topic_stems` его ``theme``):
    «расскажи про песню про енота» при заказе «про енота-механика». Вейк-слово - обращение, а не тема («Робот, включи
    песню» при песне «про робота» - не она); «вчерашняя/другая песня про енота» - не свежая."""
    if not topic:
        return False
    tokens = name_tokens(strip_wake_word(text, wake_words))
    if not any(w.startswith(_NOUNS) for w in tokens) or any(w.startswith(_OLDER) for w in tokens):
        return False
    said = topic_stems(" ".join(tokens))
    return any(s.startswith(t) or t.startswith(s) for s in said for t in topic)


class GeneratedTopic:
    """Тема последнего заказа генерации в этом разговоре: основы слов ``theme`` вызова ``generate_music``, исход
    которого не провал (``rejected``/``timeout``/``cancelled``), и когда; старше :data:`ORDER_FRESH_S` - пусто."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._noted: Optional[Tuple[Tuple[str, ...], float]] = None

    def note(self, actions: Sequence[ToolCall], results: Sequence[Any]) -> None:
        """Запомнить тему заказа хода (``actions`` - исполненные вызовы, ``results`` - их исходы ``ActionResult``)."""
        if not any(r.tool == kn.GENERATE_TOOL and r.status == "done" for r in results):
            return
        for call in actions:
            if call.tool == kn.GENERATE_TOOL:
                self._noted = (topic_stems(str(call.args.get("theme") or "")), self._clock())

    def stems(self) -> Tuple[str, ...]:
        noted = self._noted
        if noted is None or self._clock() - noted[1] > ORDER_FRESH_S:
            return ()
        return noted[0]


def bind_last_generated(actions: Sequence[ToolCall], text: str, topic: Sequence[str] = (), *,
                        wake_words: Optional[Sequence[str]] = None) -> Tuple[ToolCall, ...]:
    """Вызовы с ссылкой ``track_ref=last_generated`` там, где реплика (``text``) говорит «эта песня» или называет тему
    последнего заказа генерации (``topic``, :meth:`GeneratedTopic.stems`), и действие берёт id трека
    (:data:`REF_ACTIONS`); остальные вызовы - как были. Что бы модель ни написала в ``track_id``, тул заменит его id
    свежего сочинённого трека; свежего нет - останется её ``track_id``."""
    if not any(a.tool == kn.LIBRARY_TOOL for a in actions):
        return tuple(actions)
    if not refers_to_last_generated(text) and not names_generated_topic(text, topic, wake_words):
        return tuple(actions)
    return tuple(_ref(a) if _takes_ref(a) else a for a in actions)


def _takes_ref(call: ToolCall) -> bool:
    """Вызов ``music_library`` с id трека, где выбор ещё не сделан: подтверждённое удаление (``confirm``) - ответ на вопрос
    о КОНКРЕТНОМ треке (``tool_ask``), «да, удаляй эту песню» его не меняет."""
    return call.tool == kn.LIBRARY_TOOL and call.args.get("action") in REF_ACTIONS and not call.args.get("confirm")


def _ref(call: ToolCall) -> ToolCall:
    args: Any = {**call.args, REF_ARG: LAST_GENERATED}
    return ToolCall(call.tool, MappingProxyType(args))


__all__ = ["LAST_GENERATED", "ORDER_FRESH_S", "REF_ACTIONS", "REF_ARG", "GeneratedTopic", "bind_last_generated",
           "names_generated_topic", "refers_to_last_generated"]
