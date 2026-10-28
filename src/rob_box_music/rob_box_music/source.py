"""Путь музыки: синтез / кэш / генерация — кандидаты кода, выбор, фразы (ADR-0158 §2.2, §2.6, §3.2; ADR-0148).

Живой прогон 13.10: «Робот, сгенерируй пиздатый музон своим голосом» → LLM ``request_music(intent=track)`` → синтез и
«Включаю музыку.» — тихая подмена: генерировать робот ещё не умеет и петь синтезом не может. Решает код:

1. :func:`source_ask` — слоты просьбы из слов человека (закрытые основы ``knowledge.SOURCE_ASK_STEMS``): «сгенерируй»,
   «спой»/«своим голосом», «новую», «сочини», жанр (``GEN_GENRE_WORDS``, окно «гранж», слова стилей синтеза).
2. :func:`source_candidates` — кандидаты с признаками ``SOURCE_WEIGHTS`` и **жёсткими фильтрами до выбора**:
   синтеза нет, если просили петь или ``Style.synth_fit`` < ``SYNTH_FIT_MIN``; ``generate:*`` нет без исполнителя
   генерации, ключа, живого провайдера, денег (баланс, лимиты дня и сета) и в режиме ``synth`` (:func:`gen_block`);
   кэш — по жанру и голосу. Звучащих нет — один ``decline:<причина>``.
3. Выбор — тот же ``choice.Chooser`` (``DeterministicChooser`` → цепочка Jev → LLM → кубик): модель видит только список
   кода. Провал генерации после выбора — :func:`choose_with_fallback` спрашивает ещё раз без ``generate:*`` (В3).
4. Фразу о выбранном строит код (:func:`route_note`, :func:`cache_text`, :func:`decline_text`): если играет не то, что
   просили, фраза называет и просьбу, и причину.
5. Вилка Шифу 27.10 (2×2, без отказа «движок так не играет») — одна таблица ``knowledge.MUSIC_FORK``
   (:func:`fork_cell`): синт назван — без пения синтез (жанр, который синт не играет, — ближайшим стилем,
   :func:`synth_key`), с пением — ``live:song``/``live:rap`` (робот поёт сам: ``perform`` под синт-подложку); генерация
   — ``generate:(robot_)song`` с пением, ``generate:instrumental``/кэш без. «На синтезаторе» = «на Ренардо» при любом
   пути (решение Шифу 27.10, #4090 D). Путь пения (#4079): :func:`choose_sing` до основной модели диалога v2. Пение —
   узко, словами целиком (:func:`sing_kind`; правка звучащего у «сделай/запиши» — не заказ, #4090 A); маркеры пути
   (``knowledge.SING_ROUTE_MARKERS`` по приоритету, слова движка ``MUSIC_ENGINES`` — живое всегда) сужают кандидатов
   кодом; голое «спой» — чойсер с контекстом (последние реплики, играет ли музыка, жива ли генерация) и перевесом
   живому (``sing_live``; жанр, который синт не играет, — перевес генерации ``synth_poor``).

Модуль чистый (без сети и БД): кэш и деньги собирает ``rob_box_mcp_tools.engine.source_route``.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, replace
from typing import Callable, FrozenSet, List, Mapping, Optional, Sequence, Set, Tuple

from . import knowledge as kn
from .choice import Choice, ChoiceQuestion, Chooser, Option
from .theme import match_style_text

SYNTH, CACHE, GENERATE, DECLINE = "synth", "cache", "generate", "decline"
#: Живое пение робота (#4079): ``live:song``/``live:rap`` — исполняет диалог v2 (``perform`` под синт-подложку), не тул.
LIVE = "live"
#: Кто решил путь пения: слова или один кандидат — код; иначе ``chooser:<провайдер>``.
BY_CODE = "code"
#: Синтез-кандидат одиночной пьесы по названию (наш материал: партитура/RTTTL, годность — ADR-0157).
PIECE = "piece"
#: Почему кандидатов ``generate:*`` нет / генерация не вышла → слова для фразы. Причины провайдера —
#: ``engine.gen.provider.REASONS``; ``not_wired``, ``mode_synth``, ``pending`` и причины песни — наши.
REASON_WORDS = {
    "not_wired": "генерация у меня ещё не подключена",
    "mode_synth": "генерация выключена",
    "pending": "песня ещё пишется",
    # запись песни (генерация), а не пение вообще: вживую робот читает под синт и этим голосом (``LIVE_OFFER_BY_REASON``)
    "silero_voice": "запасной голос для записи песен пока не годится",
    # песня голосом робота (``engine.gen.song``, ADR-0158 PR-7): наши причины провала
    "lyrics": "текст не сложился",
    "no_voice": "не знаю, каким голосом петь",
    "no_preset": "для этого голоса нет пресета пения",
    "vocal": "голос не спел текст",
    # вокал: ``vocal_`` + ``SingSynthError.reason`` (``rob_box_voice.sing_synth``); нет слов — общее «vocal»
    "vocal_no_api_key": "нет ключа голоса",
    "vocal_auth": "голос не принял ключ",
    "vocal_rate_limit": "голос просит подождать: лимит",
    "vocal_timeout": "голос не ответил вовремя",
    "vocal_network": "связь с голосом оборвалась",
    "vocal_rejected": "голос не принял текст",
    "no_track": "сервис не вернул песню",
    "play_failed": "песня не заиграла",
    "track_play_failed": "трек не заиграл",
    "internal": "что-то пошло не так",
    "no_key": "нет ключа сервиса генерации",
    "no_credits": "на счёте генерации нет кредитов",
    "budget_exhausted": "лимит генерации на сегодня вышел",
    "rate_limited": "сервис генерации просит подождать",
    "provider_down": "сервис генерации не отвечает",
    "rejected": "сервис генерации отклонил заказ",
    "timeout": "генерация не уложилась по времени",
    "no_material": "мелодии для живой версии у меня нет",
    "theme_cache": "под тему хватает сочинённого раньше",
}
INSTRUCTIONS = ("Выбери, откуда взять музыку роботу: своё (синтез, сразу и бесплатно), вживую (робот поёт сам под "
                "синт: сразу и бесплатно, не сохранится), сгенерированное раньше (кэш) или новая генерация (кредиты и "
                "ожидание 1-3 минуты, сохранится в библиотеке). Учитывай заказ и последние реплики: просили сохранить "
                "или записать - генерация, просили быстро - вживую. У кандидатов - очки кода в одной шкале; обычно "
                "лучший - с наибольшим итогом. Выбери один id.")
#: Тул музыки получил просьбу спеть вживую (агент без формы пения диалога v2: чат Telegram, оператор, v1) — фраза кода
#: вместо генерации (#4090 G.2). Пения здесь не обещает (ревью #4071 круг 2 NIT-5: в этом агенте выступления нет) и не
#: оставляет человека ни с чем: честно называет, где робот поёт вживую (вслух, из динамиков), и что сказать, чтобы он
#: спел. От первого лица робота: чат Telegram, шлем оператора и v1 говорят его словами, «скажи ему» там не к месту.
LIVE_TOOL_TEXT = ("Вживую под синт я пою только вслух, голосом из динамиков, — отсюда не выйдет. Скажи рядом со мной: "
                  "«Робот, спой под синт». Генерацию для этого не запускаю.")
#: Генерации песни нет (ключ, деньги, сервис) — другая клетка вилки Шифу 27.10: спеть самому вживую под синт.
LIVE_OFFER = "Могу спеть сам вживую под синт."
#: То же предложение, когда причина отказа - голос, а не сервис (#4090 G.1): запасной голос (Silero) песню для записи не
#: поёт, но вживую робот читает под синт и им (``perform`` идёт через TTS любого провайдера, паузы без разметки), - так и
#: говорим, а не «петь не умею» рядом с «могу спеть». Причина вне таблицы - :data:`LIVE_OFFER`.
LIVE_OFFER_BY_REASON = {
    "silero_voice": "Могу спеть сам вживую под синт — этим же голосом, без записи.",
}


_WORD_RE = re.compile(r"[^\W\d_]+")


def _spans(text: str) -> List[Tuple[str, int, int]]:
    """Слова речи в нижнем регистре без «ё» и их позиции ``(слово, начало, конец)`` в исходной строке."""
    return [(m.group().lower().replace("ё", "е"), m.start(), m.end()) for m in _WORD_RE.finditer(text or "")]


def _words(text: str) -> List[str]:
    return [word for word, _start, _end in _spans(text)]


def _has(words: Sequence[str], stems: Sequence[str]) -> bool:
    return any(w.startswith(s.replace("ё", "е")) for w in words for s in stems)


@dataclass(frozen=True)
class SourceAsk:
    """Слоты просьбы о пути (ADR-0158 §2.2): ``generated`` «сгенерируй», ``vocal`` петь, ``robot_voice`` «своим
    голосом», ``fresh`` «новую», ``own`` «сочини», ``genre`` — ключ ``GEN_STYLES`` или «», ``engine`` — ключ
    ``MUSIC_ENGINES``, названный человеком («на Ренардо», «на синтезаторе», «вживую»), или «»; ``tweak`` — просьба
    правит играющее («ускорь», «добавь баса», «развивай»: ``TRACK_EDITS``), а не заказывает другое; ``live`` —
    петь вживую названо словами («спой под синт», «спой вживую», #4079): вид исполнения (``song``/``rap``) или «»."""

    generated: bool = False
    vocal: bool = False
    robot_voice: bool = False
    fresh: bool = False
    own: bool = False
    genre: str = ""
    engine: str = ""
    tweak: bool = False
    live: str = ""

    def describe(self) -> str:
        flags = [name for name in ("generated", "vocal", "robot_voice", "fresh", "own", "live") if getattr(self, name)]
        return f"asked=[{','.join(flags)}] genre={self.genre or '—'}" + (f" engine={self.engine}" if self.engine else "")

    @property
    def asks_path(self) -> bool:
        """Просьба называет иной путь, чем «играй сам»: «сгенерируй», «спой», «своим голосом», «новую», «сочини»."""
        return self.generated or self.vocal or self.robot_voice or self.fresh or self.own

    @property
    def plain(self) -> bool:
        """Просьба не называет путь: ни «сгенерируй»/«спой»/«новую»/«сочини», ни жанра («ускорь», «добавь баса»)."""
        return not (self.asks_path or self.genre)


def ask_genre(text: str) -> str:
    """Жанр просьбы: жанры без синтеза (``GEN_GENRE_WORDS``), окно стиля («гранж»), слово стиля синтеза; нет — «»."""
    words = _words(text)
    for stems in (kn.GEN_GENRE_WORDS, kn.WINDOW_WORDS):
        found = next((key for stem, key in stems.items() if key in kn.GEN_STYLES and _has(words, (stem,))), "")
        if found:
            return found
    style = match_style_text(text or "")
    return style if style in kn.GEN_STYLES else ""


def _phrase_at(words: Sequence[str], i: int, parts: Sequence[str]) -> bool:
    """Слова ``parts`` стоят в речи подряд, точно, начиная с позиции ``i`` (запись из нескольких слов таблицы)."""
    return list(words[i:i + len(parts)]) == list(parts)


def ask_tweak(text: str) -> bool:
    """Просьба правит играющее («ускорь», «добавь баса», «в том же духе»): вид правки ``TRACK_EDITS`` в словах — тем же
    чтением, что правка играющего (``track_edit.edits_in``: части реплики, отрицание, направление баса; #4029)."""
    from .track_edit import edits_in

    return bool(edits_in(text))


def _names_engine(engine: kn.MusicEngine, word: str) -> bool:
    """Слово речи называет движок по имени: начинается с основы или содержит ядро (слово-двойник ядром не считается)."""
    if word.startswith(engine.stems):
        return True
    return not word.startswith(engine.lookalikes) and any(core in word for core in engine.cores)


def _straddles(glued: str, split: int, piece: str) -> bool:
    """``piece`` стоит в склейке ``glued`` через стык ``split`` (часть в первом слове, часть во втором)."""
    start = glued.find(piece)
    while start != -1:
        if start < split < start + len(piece):
            return True
        start = glued.find(piece, start + 1)
    return False


def _joined_names_engine(engine: kn.MusicEngine, first: str, second: str) -> bool:
    """Соседние слова, склеенные, называют движок через стык («ре нардо», «нарен дарда»): STT разрезал имя. Основа или
    ядро обязаны заходить во второе слово — имя, целиком лежащее в первом, пару не берёт (лишний сосед не съедается);
    слово-двойник склейку не даёт."""
    if first.startswith(engine.lookalikes) or second.startswith(engine.lookalikes):
        return False
    glued = first + second
    return (any(glued.startswith(stem) and len(stem) > len(first) for stem in engine.stems)
            or any(_straddles(glued, len(first), core) for core in engine.cores))


def _word_hits(entries: Mapping[str, Tuple[str, ...]], words: Sequence[str]) -> List[Tuple[int, int]]:
    """Слова движка в речи (``MusicEngine.name_words``, ``MusicEngine.words``): ``(первое слово, слово после записи)``.
    Запись не берётся, если следом слово из её списка: «на синт» перед «вейв» — составной стиль."""
    hits: List[Tuple[int, int]] = []
    for i in range(len(words)):
        for entry, unless in entries.items():
            parts = entry.split()
            end = i + len(parts)
            if _phrase_at(words, i, parts) and (words[end] if end < len(words) else "") not in unless:
                hits.append((i, end))
                break
    return hits


def _name_hits(engine: kn.MusicEngine, words: Sequence[str]) -> List[Tuple[int, int]]:
    """Имя движка в речи (основы, ядра, склейка пары слов): ``(первое слово, слово после имени)``. Пары первыми: в «ре
    нардо» второе слово само ядро, но «ре» — часть имени."""
    pairs = [(i, i + 2) for i in range(len(words) - 1) if _joined_names_engine(engine, words[i], words[i + 1])]
    return pairs + [(i, i + 1) for i, word in enumerate(words) if _names_engine(engine, word)]


def _engine_spans(engine: kn.MusicEngine, words: Sequence[str], by_meaning: bool) -> List[Tuple[int, int]]:
    """Где движок назван в словах речи: слова по смыслу (при ``by_meaning``), слова движка и имя — без проверки
    отрицания и темы."""
    return [*(_word_hits(engine.words, words) if by_meaning else []), *_word_hits(engine.name_words, words),
            *_name_hits(engine, words)]


def _engine_hits(words: Sequence[str], by_meaning: bool) -> List[Tuple[str, int, int]]:
    """Движки, названные в словах речи: ``(ключ, первое слово, слово после)``; после отрицания и ввода темы — нет."""
    hits: List[Tuple[str, int, int]] = []
    taken: Set[int] = set()
    for key, engine in kn.MUSIC_ENGINES.items():
        for first, end in _engine_spans(engine, words, by_meaning):
            if not taken & set(range(first, end)):
                taken.update(range(first, end))
                if not _negated(words, first) and (words[first - 1] if first else "") not in kn.ENGINE_TOPIC_LEADS:
                    hits.append((key, first, end))
    return hits


def engine_words(text: str, by_meaning: Optional[bool] = None) -> List[Tuple[str, int, int]]:
    """Слова речи, называющие движок: ``(ключ, начало, конец)`` в исходной строке. Единственное место, где код узнаёт
    движок в услышанной фразе (ADR-0148): по таблице ``MUSIC_ENGINES`` — имя (основы, ядра, склейка пары слов) и
    слова движка («на синтезаторе», «на синте» — как имя при любом пути, решение Шифу 27.10, #4090 D) всегда, слова
    по смыслу («вживую», «живьём») — только пока просьба не называет иной путь (``SourceAsk.asks_path``: «сгенерируй
    трек вживую» — все пути открыты); ``by_meaning`` решает за просьбу, когда её путь уже известен вызывающему (тул
    «сгенерируй»). Слово после отрицания («не на синтезаторе», «без синтезатора», «а не пой вживую») или ввода темы
    («песню про синтезатор», «про Ренардо») движок не называет (``ENGINE_NEGATIONS``, ``ENGINE_TOPIC_LEADS``)."""
    tokens = _spans(text)
    words = [word for word, _start, _end in tokens]
    if by_meaning is None:
        by_meaning = not _path_ask(text).asks_path
    hits = [(key, tokens[first][1], tokens[end - 1][2]) for key, first, end in _engine_hits(words, by_meaning)]
    return sorted(hits, key=lambda hit: hit[1])


def ask_engine(text: str, by_meaning: Optional[bool] = None) -> str:
    """Движок, названный в речи (``MUSIC_ENGINES``): имя «Ренардо» и искажения STT, в том числе слитые из двух слов,
    слова движка «на синтезаторе», «на синте» или слова по смыслу «вживую» (:func:`engine_words`); не назван — «»."""
    found = engine_words(text, by_meaning)
    return found[0][0] if found else ""


def blank_engine(text: str) -> str:
    """Текст без слов, называющих движок (на их месте пробелы той же длины): имя движка — не тема трека."""
    out = text or ""
    for _key, start, end in engine_words(out):
        out = out[:start] + " " * (end - start) + out[end:]
    return out


def engine_route(engine: str) -> str:
    """Путь, который играет названный движок (``SYNTH`` у синтеза); движок не назван — «»."""
    known = kn.MUSIC_ENGINES.get(engine)
    return known.route if known is not None else ""


def settle_engine(ask: SourceAsk, slot: Optional[str], playing: str = "",
                  own_subject: bool = False) -> Tuple[SourceAsk, str]:
    """Движок заказа: ``(просьба, строка для лога)``. Решает код по словам человека и снимку плеера, а не LLM (#4013).

    * Слова называют движок (:func:`ask_engine`) — он и есть; слот ``engine`` схемы (его заполнила LLM) ничего не
      прибавляет: слот, которого слова не подтверждают («сыграй на балалайке» + ``engine=synth``), отбрасывается
      с причиной в строке, иначе молчаливое сужение пути до синтеза (кэш и генерация не рассматриваются).
    * Не названного движка заказ наследует играющий (``playing``, снимок плеера) только когда просьба **правит
      играющее** (``SourceAsk.tweak``, ``TRACK_EDITS``: «ускорь», «добавь баса», «развивай» под синтезом — движок
      тот же), не называет путь (``SourceAsk.plain``) и не называет своего предмета (``own_subject``: пьеса по
      названию или инструмент реплики). Всякая прочая просьба — «сыграй ещё», «включи музыку», «поставь Нирвану»,
      «про космос», «что-нибудь другое», громкость — идёт обычным путём: кавер и кэш остаются кандидатами, синтез
      до стопа не липнет.

    Строка пуста, когда добавить нечего."""
    unsupported = slot in kn.MUSIC_ENGINES and slot != ask.engine  # слова человека не называют этот движок
    note = f"слот engine={slot} без опоры в словах человека — отброшен" if unsupported else ""
    if ask.engine or playing not in kn.MUSIC_ENGINES or own_subject or not ask.plain or not ask.tweak:
        return ask, note
    return replace(ask, engine=playing), "; ".join(filter(None, (note, f"движок наследуется от играющего: {playing}")))


def _path_ask(text: str, kind: Optional[str] = None) -> SourceAsk:
    """Слоты пути, который просят словами: «сгенерируй», «спой» (:func:`sing_kind`), «своим голосом» — только в просьбе
    спеть, «новую», «сочини» (жанр, движок и правка играющего — в :func:`source_ask`)."""
    words = _words(text)
    kind = sing_kind(text) if kind is None else kind
    generated, own = _has(words, kn.SOURCE_ASK_STEMS["generated"]), _has(words, kn.SOURCE_ASK_STEMS["own"])
    return SourceAsk(generated, bool(kind), bool(kind) and _robot_voice(words),
                     _has(words, kn.SOURCE_ASK_STEMS["fresh"]), own)


def source_ask(text: str) -> SourceAsk:
    """Слоты просьбы о пути из слов человека (код, без LLM)."""
    kind = sing_kind(text)
    live = kind if kind and sing_markers(text) == LIVE else ""
    return replace(_path_ask(text, kind), genre=ask_genre(text), engine=ask_engine(text), tweak=ask_tweak(text),
                   live=live)


_CLAUSE_TOKEN_RE = re.compile(r"[^\W\d_]+(?:-[^\W\d_]+)*|[.,!?;:—–]")
#: Конец предложения: хвост дополнения-песни дальше не идёт (запятая и тире — идут: «сделай песню, пожалуйста, тише»).
_SENTENCE_ENDS = (".", "!", "?", ";")


def _is_word(token: str) -> bool:
    return token[:1].isalpha()


@dataclass(frozen=True)
class _Phrase:
    """Фраза для «глагол + дополнение»: ``tokens`` — слова (нижний регистр, без «ё», слова через дефис — по
    отдельности: «спой-ка») и знаки препинания по порядку; ``breaks`` — позиции границ частей фразы (знак или слово
    ``SING_CLAUSE_BREAKS`` целиком: «что-нибудь» — не граница); флаги всей фразы: «без вокала», «давай», повтор."""

    tokens: Tuple[str, ...]
    breaks: FrozenSet[int]
    no_vocal: bool = False
    joint: bool = False
    repeat: bool = False


def _phrase(text: str) -> _Phrase:
    tokens: List[str] = []
    breaks: Set[int] = set()
    for token in _CLAUSE_TOKEN_RE.findall((text or "").lower().replace("ё", "е")):
        if not _is_word(token) or token in kn.SING_CLAUSE_BREAKS:
            breaks.add(len(tokens))
        tokens.extend(token.split("-") if _is_word(token) else (token,))
    words = [t for t in tokens if _is_word(t)]
    return _Phrase(tuple(tokens), frozenset(breaks), _no_vocal(words), bool(set(words) & set(kn.SING_JOINT_ASKS)),
                   any(_has_exact(words, entry) for entry in kn.SING_REPEAT_PHRASES))


def _clauses(phrase: _Phrase) -> List[Tuple[int, List[str]]]:
    """Части фразы для «глагол + объект» (ревью #4071 B2): слова между знаками препинания и словами-границами
    (``SING_CLAUSE_BREAKS``: «запиши что я люблю песни» — объект в другой части) и позиция первого слова части во
    фразе (хвост дополнения идёт дальше части, :func:`_edited`)."""
    out: List[Tuple[int, List[str]]] = []
    for k, token in enumerate(phrase.tokens):
        if k in phrase.breaks:
            continue
        if out and out[-1][0] + len(out[-1][1]) == k:
            out[-1][1].append(token)
        else:
            out.append((k, [token]))
    return out


def _negated_vocal(words: Sequence[str], i: int) -> bool:
    return i > 0 and words[i - 1] in kn.VOCAL_NEGATIONS


def _has_exact(words: Sequence[str], entry: str) -> bool:
    """Запись таблицы в речи точно: одно слово — это слово целиком, несколько — эти слова подряд."""
    parts = entry.split()
    return any(_phrase_at(words, i, parts) for i in range(len(words)))


def _robot_voice(words: Sequence[str]) -> bool:
    """«Своим/твоим голосом» (``ROBOT_VOICE_PHRASES``, вид генерации ``robot_song``); «свой голос» — сам голос."""
    return any(_has_exact(words, entry) for entry in kn.ROBOT_VOICE_PHRASES)


def _voiced(words: Sequence[str]) -> bool:
    """Музыку поют: «своим голосом», «с вокалом» (не после «без»), «со словами» (``VOCAL_PHRASES``)."""
    vocal = any(w.startswith(kn.VOCAL_STEMS) and not _negated_vocal(words, i) for i, w in enumerate(words))
    return vocal or _robot_voice(words) or any(_has_exact(words, entry) for entry in kn.VOCAL_PHRASES)


def _no_vocal(words: Sequence[str]) -> bool:
    """Просьба инструментала, не песни (#4090 E): «без вокала/слов/пения/голоса/текста» (``NO_VOCAL_STEMS`` сразу после
    ``VOCAL_NEGATIONS``) или слово инструментала (``INSTRUMENTAL_STEMS``: «сгенерируй инструментальную песню») - если
    рядом не просят голос прямо (:func:`_voiced`: «песню своим голосом под инструментал» - песня). Одна таблица знания -
    ``knowledge``; решает ``_verb_kind``: глагол создания с таким отказом пением не считается."""
    refused = any(w.startswith(kn.NO_VOCAL_STEMS) and _negated_vocal(words, i) for i, w in enumerate(words))
    return refused or (_has(words, kn.INSTRUMENTAL_STEMS) and not _voiced(words))


#: Вид прямого дополнения по слову (``SING_OBJECTS``, мн. ч. — ``SING_PLURAL_OBJECTS``) и правило глагола по слову
#: (``SING_VERB_RULES``).
_OBJECT_KIND = {word: kind for kind, words in kn.SING_OBJECTS.items() for word in words}
_PLURAL_KIND = {word: kind for kind, words in kn.SING_PLURAL_OBJECTS.items() for word in words}
_VERB_RULE = {verb: rule for rule in kn.SING_VERB_RULES for verb in rule.verbs}
#: Маркеры пути пения по строке (``SING_ROUTE_MARKERS``): строка ``synth`` — синт назван подложкой («под бит»).
_ROUTE_MARKERS = dict(kn.SING_ROUTE_MARKERS)


def _filler(word: str) -> bool:
    """Слово между глаголом и дополнением: служебное, «эту/мою», слово количества («две») или
    прилагательное-определение («новую», «пиздатый»)."""
    return (word in kn.SING_OBJECT_FILLERS or word in kn.SING_EXISTING_WORDS or word in kn.SING_QUANTITY_WORDS
            or word.endswith(kn.SING_OBJECT_ADJ_ENDINGS))


def _counted(clause: Sequence[str], j: int) -> bool:
    """Перед ``clause[j]`` — слово количества (``SING_QUANTITY_WORDS``) не дальше трёх слов, между ними только
    прилагательные мн. ч. (``SING_PLURAL_ADJ_ENDINGS``: «две новые песни»); существительное между ними («пару копий
    песни») — не счёт этой песни."""
    k, stop = j - 1, max(0, j - 3)
    while k > stop and clause[k] not in kn.SING_QUANTITY_WORDS and clause[k].endswith(kn.SING_PLURAL_ADJ_ENDINGS):
        k -= 1
    return k >= stop and clause[k] in kn.SING_QUANTITY_WORDS


def _object_word(clause: Sequence[str], j: int) -> str:
    """Вид дополнения ``clause[j]`` или «»: мн. ч. («песни», «песен») — только после слова количества
    (:func:`_counted`: «сгенерируй две песни»), иначе это и родительный («слова песни»), и чужие песни («запиши песни
    Цоя»)."""
    word = clause[j]
    if word in _OBJECT_KIND:
        return _OBJECT_KIND[word]
    return _PLURAL_KIND.get(word, "") if _counted(clause, j) else ""


def _object_at(clause: Sequence[str], i: int, step: int) -> int:
    """Индекс прямого дополнения глагола ``clause[i]`` в сторону ``step`` (1 — после глагола, −1 — перед ним): первое
    значимое слово — слово ``SING_OBJECTS``; между ними только служебные слова (:func:`_filler`) и после глагола
    «под/на + слово» (``SING_OBJECT_PREPOSITIONS``), не дальше ``SING_OBJECT_REACH``. Нет — −1: «запиши название
    песни» — первое значимое слово «название»."""
    j, budget = i + step, kn.SING_OBJECT_REACH
    while 0 <= j < len(clause) and budget > 0:
        if _object_word(clause, j):
            return j
        skip = 2 if step > 0 and clause[j] in kn.SING_OBJECT_PREPOSITIONS else int(_filler(clause[j]))
        if not skip:
            return -1
        j, budget = j + step * skip, budget - skip
    return -1


def _existing(clause: Sequence[str], i: int, j: int) -> bool:
    """У дополнения ``clause[j]`` глагола ``clause[i]`` — «эту/мою» (``SING_EXISTING_WORDS``): песня звучащая или
    чужая («запиши эту песню», «эту песню запиши»)."""
    near = clause[i + 1:j] if j > i else [*clause[max(0, j - 2):j], *clause[j + 1:i]]
    return any(w in kn.SING_EXISTING_WORDS for w in near)


def _object_of(clause: Sequence[str], i: int, made_new: bool) -> Tuple[str, int]:
    """``(вид, индекс)`` прямого дополнения глагола ``clause[i]`` (``song``/``rap``/``music``) или ``("", -1)``:
    сначала после глагола, затем перед ним («рэп мне зачитай»). У просьбы создать (``made_new``) звучащая или чужая
    песня — не дополнение."""
    for step in (1, -1):
        j = _object_at(clause, i, step)
        if j >= 0:
            return ("" if made_new and _existing(clause, i, j) else _object_word(clause, j)), j
    return "", -1


def _skipped(word: str) -> bool:
    """Слово рядом с парой «глагол + песня», которое правку не решает: служебное или слово степени («чуть»)."""
    return word in kn.SING_OBJECT_FILLERS or word in kn.SING_DEGREE_WORDS


def _comparative(word: str) -> bool:
    """Сравнительная степень по окончанию (``SING_COMPARATIVE_ENDINGS``): «слышнее», «короче», «лучше», «поярче»."""
    return len(word) >= kn.SING_COMPARATIVE_MIN_LEN and word.endswith(kn.SING_COMPARATIVE_ENDINGS)


def _edit_word(seq: Sequence[str], k: int) -> bool:
    """С ``seq[k]`` начинается правка звучащего: сравнительная степень или запись ``SING_CREATE_MODIFIERS``."""
    return _comparative(seq[k]) or any(_phrase_at(seq, k, entry.split()) for entry in kn.SING_CREATE_MODIFIERS)


def _tail_words(seq: Sequence[str], k: int, size: int) -> List[str]:
    """До ``size`` слов подряд с позиции ``k`` (до первого знака препинания)."""
    out: List[str] = []
    for token in seq[k:k + size]:
        if not _is_word(token):
            break
        out.append(token)
    return out


def _order_tail(seq: Sequence[str], k: int) -> bool:
    """Хвост «предлог ``seq[k]`` + слово» — слот заказа, а не правка звучащего: тема, повод, язык, стиль
    (``SING_ORDER_TAILS``), голос («с вокалом», «со словами»), движок («на Ренардо», «на синтезаторе», «без
    синтезатора» — движок и с отрицанием слот: заказ не на синте)."""
    seg = _tail_words(seq, k, 3)
    if len(seg) < 2:
        return False
    if seg[1].startswith(tuple(kn.SING_ORDER_TAILS.get(seg[0], ()))) or _voiced(seg):
        return True
    return any(first <= 1 for engine in kn.MUSIC_ENGINES.values() for first, _end in _engine_spans(engine, seg, True))


def _edit_before(clause: Sequence[str], first: int) -> bool:
    """Правка прямо перед парой «глагол + песня» (служебные слова пропускаются): «погромче сделай песню»."""
    k = first - 1
    while k >= 0 and _skipped(clause[k]):
        k -= 1
    return k >= 0 and _edit_word(clause, k)


def _edit_after(phrase: _Phrase, pos: int) -> bool:
    """Хвост после пары «глагол + песня» с позиции ``pos`` во фразе — правка звучащего (``SING_CREATE_MODIFIERS``):
    первое значимое слово до конца предложения (запятые, служебные слова и степень пропускаются) — сравнительная
    степень или слово таблицы правки, предлог правки не со слотом заказа (:func:`_order_tail`) или придаточное
    «который …» с говорящим («запиши песню, которую я сейчас спою» — его песня, не новая)."""
    tokens, k = phrase.tokens, pos
    while k < len(tokens) and tokens[k] not in _SENTENCE_ENDS and (not _is_word(tokens[k]) or _skipped(tokens[k])):
        k += 1
    if k >= len(tokens) or not _is_word(tokens[k]):
        return False
    if tokens[k] in kn.SING_RELATIVE_WORDS:
        return any(w in kn.SING_SELF_WORDS for w in _tail_words(tokens, k + 1, len(tokens)))
    if tokens[k] in kn.SING_EDIT_PREPOSITIONS:
        return not _order_tail(tokens, k)
    return _edit_word(tokens, k)


def _edited(phrase: _Phrase, start: int, clause: Sequence[str], i: int, j: int) -> bool:
    """Глагол второго чтения (``SING_EDIT_VERBS``: «сделай», «запиши») с песней ``clause[j]`` правит или записывает
    звучащее, а не создаёт новое (#4090 A, структура — ``SING_CREATE_MODIFIERS``): повтор во фразе («ещё раз»,
    «снова»), правка перед парой, предлог правки между глаголом и песней («запиши на флешку песню») или хвост после
    пары («слышнее», «на репите», «без шума», «с эхом», «на диктофон», «которую я спою»)."""
    between = range(i + 1, j) if j > i else range(0)
    repeat = phrase.repeat and not any(w in kn.ENGINE_TOPIC_LEADS for w in clause)  # с темой — новый заказ
    return (repeat or _edit_before(clause, min(i, j))
            or any(clause[k] in kn.SING_EDIT_PREPOSITIONS and not _order_tail(clause, k) for k in between)
            or _edit_after(phrase, start + max(i, j) + 1))


def _not_asked(clause: Sequence[str], i: int, joint: bool) -> bool:
    """Глагол ``clause[i]`` — не просьба: отрицание прямо перед ним («не пой больше», «не спой, а сыграй»; перед 2-м
    лицом ед. ч. — вежливая просьба: «ты не споёшь?», ``SING_POLITE_ENDING``), 1-е лицо мн. без «давай» («мы споём»,
    ``SING_JOINT_ENDINGS``) или инфинитив без слова просьбы (``SING_INFINITIVE_ASKS``: «можешь спеть?») либо о себе
    («я хочу спеть караоке», «мне надо спеть на конкурсе»; «я прошу тебя спеть», «можешь спеть мне?» — просьба)."""
    verb = clause[i]
    if i and clause[i - 1] in kn.SING_NEGATIONS and not verb.endswith(kn.SING_POLITE_ENDING):
        return True
    if verb.endswith(kn.SING_JOINT_ENDINGS):
        return not joint
    if not verb.endswith(kn.SING_INFINITIVE_ENDING):
        return False
    words = set(clause)
    robot = bool(words & set(kn.SING_ROBOT_AGENT_WORDS)) or any(kn.SING_INFINITIVE_ASKS.get(w) for w in words)
    about_self = bool(words & set(kn.SING_SELF_WORDS)) and not robot
    return about_self or not words & set(kn.SING_INFINITIVE_ASKS)


def _verb_kind(phrase: _Phrase, start: int, clause: Sequence[str], i: int) -> str:
    """Что просит спеть глагол ``clause[i]`` по своему правилу (``SING_VERB_RULES``): дополнение-рэп — ``rap``,
    песня/музыка с голосом — ``song``, без дополнения — ``bare`` правила; не глагол пения или не просьба — «»."""
    rule = _VERB_RULE.get(clause[i])
    if rule is None or (phrase.no_vocal and not rule.bare) or _not_asked(clause, i, phrase.joint):
        return ""
    obj, j = _object_of(clause, i, rule.made_new)
    if rule.edits and obj and _edited(phrase, start, clause, i, j):
        return ""
    if obj in rule.objects or (obj in rule.voiced and _voiced(clause)):
        return "rap" if obj == "rap" else "song"
    return rule.bare


def _sentences(phrase: _Phrase) -> List[List[str]]:
    """Слова каждого предложения фразы (граница — ``.!?;``; запятая и тире — внутри предложения)."""
    out: List[List[str]] = [[]]
    for token in phrase.tokens:
        if token in _SENTENCE_ENDS:
            out.append([])
        elif _is_word(token):
            out[-1].append(token)
    return [words for words in out if words]


def _marker_spans(words: Sequence[str], entry: str) -> List[Tuple[int, int]]:
    """Где в словах стоит запись маркера (``SING_ROUTE_MARKERS``: одно слово — основа, несколько — подряд, точно)."""
    parts = entry.split()
    if len(parts) == 1:
        return [(i, i + 1) for i, w in enumerate(words) if w.startswith(entry)]
    return [(i, i + len(parts)) for i in range(len(words)) if _phrase_at(words, i, parts)]


def _synth_named(words: Sequence[str]) -> Set[int]:
    """Позиции слов, которыми назван синт: слова движка (``MUSIC_ENGINES``, не после отрицания и не тема) с предлогом
    прямо перед ними («на синтезаторе», «на Ренардо», ``SING_OBJECT_PREPOSITIONS``) и подложка («под бит» — строка
    ``synth`` ``SING_ROUTE_MARKERS``)."""
    spans = [(first, end) for _key, first, end in _engine_hits(words, True)]
    spans += [span for entry in _ROUTE_MARKERS[SYNTH] for span in _marker_spans(words, entry)]
    named = {n for first, end in spans for n in range(first, end)}
    return named | {n - 1 for n in named if n and words[n - 1] in kn.SING_OBJECT_PREPOSITIONS}


def _ellipsis_word(word: str) -> bool:
    """Слово, которое просьбу без глагола не меняет: служебное (``SING_OBJECT_FILLERS``: «мне», «пожалуйста», «плиз»,
    «своим голосом») или «давай» (``SING_JOINT_ASKS``)."""
    return word in kn.SING_OBJECT_FILLERS or word in kn.SING_JOINT_ASKS


def _elliptic(words: Sequence[str]) -> str:
    """Просьба спеть без глагола (#4090 C) — предложение ``words`` и есть заказ, целиком: синт назван
    (:func:`_synth_named`), а кроме него и служебных слов (:func:`_ellipsis_word`) в предложении ровно одно слово —
    дополнение-песня или рэп (``SING_ELLIPSIS_OBJECTS``): «рэп под бит», «давай песню на Ренардо», «рэп под бит,
    пожалуйста» — клетка «синт + пение» вилки. Любое другое слово — отрицание («песню на синтезаторе не пой», «не
    надо», «хватит»), оценка («рэп под бит классный», «рэп под бит — это круто»), сказуемое («мне нравится», «мы
    слушали вчера»), глагол не пения («включи рэп под бит») — и это не просьба спеть: «» (решает модель; ревью
    PR #4107: код не делает обратное сказанному). Обращение («Робот, …») снимает диалог v2 до пути пения
    (``rob_box_dialog.address.decide``: текст хода и ``heard_text`` тула — без вейк-слова)."""
    named = _synth_named(words)
    rest = [w for n, w in enumerate(words) if n not in named and not _ellipsis_word(w)]
    kind = _OBJECT_KIND.get(rest[0], "") if len(rest) == 1 else ""
    return kind if named and kind in kn.SING_ELLIPSIS_OBJECTS else ""


def _asked_verbs(text: str) -> List[Tuple[str, str]]:
    """``(глагол, вид пения)`` каждой просьбы спеть во фразе: глагол и его прямое дополнение в одной части фразы
    (:func:`_clauses`); просьба без глагола (:func:`_elliptic`, предложение целиком) — с пустым глаголом."""
    phrase = _phrase(text)
    found: List[Tuple[str, str]] = []
    for start, clause in _clauses(phrase):
        found.extend((clause[i], kind) for i in range(len(clause)) for kind in (_verb_kind(phrase, start, clause, i),)
                     if kind)
    found.extend(("", kind) for kind in map(_elliptic, _sentences(phrase)) if kind)
    return found


def sing_kind(text: str) -> str:
    """Что просят спеть (код, без LLM; ревью #4071 B1/B2 и круг 2, #4090 — узко: глагол и его прямое дополнение):
    ``rap``/``song``; «» — не пение. Правило глагола — одна таблица ``SING_VERB_RULES``:

    * Глагол пения (``SING_VERBS``: «спой», «пой», «спеть», «споёшь») — пение, с дополнением-рэпом — ``rap``; правка
      звука рядом («спой погромче») пения не отменяет. «Зачитай/прочитай» — только с дополнением-рэпом, «порепь» —
      рэп и без него.
    * «Исполни», глагол создать (``SING_CREATE_VERBS``: «сгенерируй», «сочини», «запиши», «сделай») — с прямым
      дополнением-песней или рэпом (``SING_OBJECTS``: винительный, точные слова, рядом с глаголом; мн. ч. — после слова
      количества: «две песни»), музыка — только с голосом («сгенерируй музон своим голосом», «трек с вокалом»).
      «Запиши название песни», «создай плейлист песен», «сделай свой голос помягче» — дополнение не песня; «запиши эту
      песню» — звучащая песня; у «сделай/запиши» правка звучащего рядом с песней («сделай песню слышнее / на репите /
      ещё раз», «запиши песню на диктофон», :func:`_edited`) — не просьба создать.
    * «Сыграй» — только с голосом робота («сыграй песню своим голосом»). Голос и песня без глагола («мне нравится твой
      голос в этой песне») — не пение, кроме песни или рэпа с названным синтом («рэп под бит», :func:`_elliptic`);
      «без вокала» — инструментал; «не пой», «я хочу спеть», «мне надо спеть», «мы споём» — не просьба; «ты не
      споёшь?», «давай споём/сочиним» — просьба."""
    kinds = {kind for _verb, kind in _asked_verbs(text)}
    return "rap" if "rap" in kinds else "song" if kinds else ""


def _generation_asked(text: str) -> bool:
    """Маркер генерации: глагол со значением ``generate`` (``SING_CREATE_VERBS``: «сгенерируй», «запиши», «сделай»)
    просит спеть — с прямым дополнением-песней, рэпом или музыкой с голосом, без правки звука и не звучащую песню."""
    return any(kn.SING_CREATE_VERBS.get(verb) == GENERATE for verb, _kind in _asked_verbs(text))


def _negated(words: Sequence[str], i: int) -> bool:
    """Отрицание (``ENGINE_NEGATIONS``) прямо перед ``words[i]`` или через предлог/глагол пения («не на синтезаторе»,
    «а не пой вживую»); через 2-е лицо ед. ч. — вежливая просьба, не отрицание («ты не споёшь на синте?»)."""
    before = words[i - 1] if i else ""
    sung = before in kn.SING_VERBS and not before.endswith(kn.SING_POLITE_ENDING)
    bridged = before in kn.ENGINE_NEGATION_BRIDGES or sung
    return before in kn.ENGINE_NEGATIONS or (bridged and i > 1 and words[i - 2] in kn.ENGINE_NEGATIONS)


def _marked(words: Sequence[str], entry: str) -> bool:
    """Запись маркера (``SING_ROUTE_MARKERS``: одно слово — основа, несколько — подряд) в речи не после отрицания:
    «но не нейросетью» путь не называет."""
    return any(not _negated(words, i) for i, _end in _marker_spans(words, entry))


def fork_cell(row: str, sung: bool) -> kn.ForkCell:
    """Клетка вилки Шифу 27.10 (``knowledge.MUSIC_FORK``, #4090 H): строка ``synth`` (синт назван) или ``generate``
    (не назван) × поют ли."""
    return kn.MUSIC_FORK[(row, sung)]


def fork_gen_kind(vocal: bool, robot_voice: bool) -> str:
    """Вид генерации — клетка «генерация × поют» вилки (``song``/``instrumental``); «своим голосом» — ``robot_song``."""
    return "robot_song" if robot_voice else fork_cell(GENERATE, vocal).kind


def _marker_route(row: str) -> str:
    """Путь строки маркеров ``SING_ROUTE_MARKERS``: строка вилки (``synth``, ``generate``) — её клетка с пением."""
    return fork_cell(row, True).route if (row, True) in kn.MUSIC_FORK else row


def sing_markers(text: str) -> str:
    """Путь пения, названный словами (вилка Шифу 27.10, клетки — ``knowledge.MUSIC_FORK``), по приоритету
    ``SING_ROUTE_MARKERS``, первое совпадение: синт назван — слова движка (``MUSIC_ENGINES``: имя «Ренардо» и «на
    синтезаторе», «вживую»; не после отрицания и не тема — :func:`engine_words`) или «под синт/бит» — клетка «синт +
    пение» (``live``), даже рядом с «сгенерируй»; затем ``generate`` («нейросетью», «чтобы сохранилась», «сгенерируй/
    запиши песню», :func:`_generation_asked`); затем ``live`` («спой сам», «прямо сейчас»); не назван — «» (решает
    чойсер, :func:`choose_sing`). Маркер после отрицания не считается. Пение ли это вообще — :func:`sing_kind`."""
    if engine_words(text, by_meaning=True):
        return fork_cell(SYNTH, True).route
    words = _words(text)
    for row, entries in kn.SING_ROUTE_MARKERS:
        if any(_marked(words, entry) for entry in entries) or (row == GENERATE and _generation_asked(text)):
            return _marker_route(row)
    return ""


@dataclass(frozen=True)
class SingContext:
    """Контекст выбора пути пения (собирает хост диалога, без сети и БД): ``recent`` — последние реплики человека
    (``SING_CONTEXT_REPLIES``, старые первыми), ``playing`` — звучит музыка, ``gen_ok`` — генерация жива (здоровье
    провайдера ``music_gen`` годно или неизвестно), ``gen_health`` — это здоровье словом для чойсера («» — неизвестно).
    Баланса KIE хост диалога не знает: деньги и лимиты проверяет тул генерации (:func:`gen_block`) с фразой кода."""

    recent: Tuple[str, ...] = ()
    playing: bool = False
    gen_ok: bool = True
    gen_health: str = ""


@dataclass(frozen=True)
class SingRoute:
    """Путь пения до основной модели диалога: ``id`` кандидата (``live:song``, ``generate:robot_song:V6``; «» — не
    пение), ``by`` — кто решил (``code`` — слова или один кандидат, ``chooser:<провайдер>``), кандидаты, путь, названный
    словами (``named``), вид пения (``song``/``rap``)."""

    id: str = ""
    by: str = BY_CODE
    candidates: Tuple[str, ...] = ()
    named: str = ""
    kind: str = ""

    def line(self) -> str:
        """Строка лога для харнесса: ``[source] пение … кандидаты=[live:song, generate:robot_song:V6] → <id> (<кто>)``."""
        return (f"[source] пение {self.kind} слова={self.named or '—'} кандидаты=[{', '.join(self.candidates)}] → "
                f"{self.id} ({self.by})")


def _live_option(kind: str, asked: bool, recent: bool, poor_genre: str = "") -> Option:
    """Живое пение: робот поёт сам под синт-подложку; жанр, который синт не играет (``poor_genre`` — имя), подложка
    честно своим битом — так и сказано в описании кандидата."""
    tail = f"; «{poor_genre}» синт не сыграет — подложка своим битом" if poor_genre else ""
    return Option(f"{LIVE}:{kind}", {"sing_live": 1.0, "asked": float(asked), "recent_asked": float(recent),
                                     "wait_s": kn.INSTANT_WAIT_S},
                  f"вживую ({kind}): пою сам под синт-подложку — сразу и бесплатно, не сохранится{tail}")


def _sing_gen_option(robot_voice: bool, asked: bool, recent: bool, synth_poor: bool = False) -> Option:
    kind, model = fork_gen_kind(True, robot_voice), kn.GEN_MODELS[kn.DEFAULT_GEN_MODEL]
    return Option(f"{GENERATE}:{kind}:{kn.DEFAULT_GEN_MODEL}", {
        "asked": float(asked), "recent_asked": float(recent), "synth_poor": float(synth_poor),
        "cost": model.credits_per_task, "wait_s": model.typical_s},
        f"новая генерация ({kind}): 1-3 минуты, {model.credits_per_task:g} кредитов KIE, сохранится в библиотеке")


def sing_candidates(text: str, ctx: SingContext) -> Tuple[Option, ...]:
    """Кандидаты пути пения (код, жёсткие фильтры до выбора): не пение — пусто; путь назван словами
    (:func:`sing_markers`) — только он (просили сгенерировать — генерация и при мёртвом провайдере: тул скажет причину
    сам и предложит спеть вживую); иначе живое и генерация (генерация жива, ``gen_ok``). Маркер пути в последних
    репликах — признак ``recent_asked``; жанр, который синт не играет («спой метал»), — признак генерации
    ``synth_poor`` (перевес генерации, решает модель чойсера, ``SOURCE_WEIGHTS``)."""
    kind = sing_kind(text)
    if not kind:
        return ()
    named, recent = sing_markers(text), {sing_markers(r) for r in _recent(ctx)}
    poor = _poor_genre(SourceAsk(genre=ask_genre(text)))
    live = _live_option(kind, named == LIVE, LIVE in recent, poor)
    gen = _sing_gen_option(_path_ask(text, kind).robot_voice, named == GENERATE, GENERATE in recent, bool(poor))
    if named:
        return (live,) if named == LIVE else (gen,)
    return (live, gen) if ctx.gen_ok else (live,)


def _recent(ctx: SingContext) -> Tuple[str, ...]:
    """Последние реплики человека, которые видит выбор пути пения (``SING_CONTEXT_REPLIES``)."""
    return ctx.recent[-kn.SING_CONTEXT_REPLIES:] if kn.SING_CONTEXT_REPLIES else ()


def _sing_question(text: str, kind: str, ctx: SingContext, options: Tuple[Option, ...]) -> ChoiceQuestion:
    """Вопрос чойсеру: модель видит заказ с контекстом (``ChoiceQuestion.text``) и кандидатов с очками кода."""
    recent = " | ".join(_recent(ctx)) or "—"
    order = (f"спеть ({kind}): «{text}»; последние реплики человека: {recent}; играет музыка: "
             f"{'да' if ctx.playing else 'нет'}; генерация: {ctx.gen_health or 'неизвестно'}")
    state = {"recent": list(_recent(ctx)), "playing": ctx.playing, "gen_health": ctx.gen_health}
    return ChoiceQuestion(order, options, state=state, weights=kn.SOURCE_WEIGHTS, instructions=INSTRUCTIONS)


def choose_sing(text: str, ctx: SingContext, chooser: Chooser) -> SingRoute:
    """Путь пения до основной модели диалога (#4079): кандидаты кода (:func:`sing_candidates`) → один — код, иначе
    ``chooser`` (перевес живому — ``SOURCE_WEIGHTS["sing_live"]``; контекст сужает разрыв — решает модель чойсера).
    Не пение — пустой путь: форма хода не меняется."""
    options = sing_candidates(text, ctx)
    ids, kind, named = tuple(o.id for o in options), sing_kind(text), sing_markers(text) if options else ""
    if len(options) <= 1:
        return SingRoute(ids[0] if ids else "", BY_CODE, ids, named, kind)
    choice = chooser.choose(_sing_question(text, kind, ctx, options))
    return SingRoute(choice.id, f"chooser:{choice.provider}", ids, named, kind)


def synth_style(genre: str) -> Optional[str]:
    """Стиль синтеза, который играет жанр (без жанра — ``DEFAULT_STYLE``); синтеза нет — ``None``."""
    return kn.GEN_STYLES[genre].synth if genre else kn.DEFAULT_STYLE


def synth_fit(genre: str) -> float:
    style = synth_style(genre)
    return kn.STYLES[style].synth_fit if style else 0.0


@dataclass(frozen=True)
class CacheTrack:
    """Трек кэша генераций: ``group`` — варианты одной задачи (одна группа — один кандидат)."""

    id: str
    title: str
    kind: str = "instrumental"
    style_key: str = ""
    instrumental: bool = True
    rating: int = 0
    played_ago_h: Optional[float] = None
    group: str = ""
    work_id: str = ""      # кавер: каталожное название пьесы (ADR-0158 §2.5)
    instrument: str = ""   # кавер: ключ ``INSTRUMENTS`` или «»
    voice: str = ""  # голос, которым спета песня (``tts``-голос из кэша); «» — неизвестен
    prompt: str = ""  # теги заказа (``generated_tracks.prompt``): тема сета — по тегам темы (``GEN_THEME_TAGS``)
    seconds: float = 0.0  # фактическая длина файла (``duration_ms``); 0 — неизвестна


@dataclass(frozen=True)
class SourceContext:
    """Контекст пути (собирает код): кэш, готовность и деньги генерации, режим (ADR-0158 §2.2, §2.6, §2.8).
    ``gen_ready`` — исполнитель генерации подключён (нет — ``not_wired``); ``balance`` ``None`` — неизвестен (402
    провайдера приведёт к выбору заново); ``set_left`` ``None`` — не сет; ``gen_failed`` — генерация уже упала (В3)."""

    cache: Tuple[CacheTrack, ...] = ()
    gen_ready: bool = False
    has_key: bool = True
    provider_ok: bool = True
    balance: Optional[float] = None
    day_left: float = 0.0
    set_left: Optional[float] = None
    mode: str = "auto"
    gen_failed: str = ""
    cover_ready: bool = False  # исполнитель кавера пьесы подключён (ADR-0158 PR-4, ``engine.gen.cover``)
    voice: str = ""  # текущий голос робота: «спой своим голосом» берёт из кэша только песню, спетую им
    song_block: str = ""  # песню заказывать нельзя (``SongStudio.blocked``: Silero) — кавер это не касается

    def without_generate(self, reason: str) -> "SourceContext":
        return replace(self, gen_failed=reason)


def gen_kind(option_id: str) -> str:
    """Вид генерации кандидата ``generate:<вид>:<модель>`` (instrumental, song, robot_song, cover); иначе «»."""
    parts = option_id.split(":")
    return parts[1] if len(parts) == 3 and parts[0] == GENERATE else ""


def route_seconds(route: "Route", last: bool = False) -> float:
    """Сколько секунд сета занимает трек по выбранному пути (одно место, ADR-0148): длина файла — кавер
    ``COVER_DURATION_S``, прочая генерация ``GEN_SET_DURATION_S``, трек кэша — своя (неизвестна — как у генерации) —
    как её играет дека (``render.audio.set_track_seconds``: файл целиком, не последний трек за вычетом кроссфейда
    ``GEN_XFADE_S``); синтез — ``SET_TRACK_SECONDS``; отказ — 0."""
    from .render.audio import set_track_seconds

    if route.kind == GENERATE:
        return set_track_seconds(float(kn.COVER_DURATION_S if gen_kind(route.id) == "cover"
                                       else kn.GEN_SET_DURATION_S), last)
    if route.kind == CACHE:
        return set_track_seconds((route.track.seconds if route.track else 0.0) or float(kn.GEN_SET_DURATION_S), last)
    return float(kn.SET_TRACK_SECONDS) if route.kind == SYNTH else 0.0


def gen_block(ctx: SourceContext, cost: float) -> str:
    """Почему кандидата генерации нет (ключ :data:`REASON_WORDS`) или «» — генерировать можно."""
    checks = ((ctx.gen_failed, ctx.gen_failed), (ctx.mode == "synth", "mode_synth"), (not ctx.gen_ready, "not_wired"),
              (not ctx.has_key, "no_key"), (not ctx.provider_ok, "provider_down"),
              (ctx.balance is not None and ctx.balance < cost, "no_credits"),
              (ctx.day_left < cost or (ctx.set_left is not None and ctx.set_left < cost), "budget_exhausted"))
    return next((reason for blocked, reason in checks if blocked), "")


def synth_key(genre: str) -> str:
    """Стиль синтеза для жанра: жанр, который синт играет (``synth_fit`` ≥ ``SYNTH_FIT_MIN``), — его стиль; иначе
    ближайший сильный стиль (``GEN_BRIDGE_STYLES``: метал, рок — брейкбит; джаз, шансон — синтвейв), нет в таблице —
    ``DEFAULT_STYLE``. Названный синт играет так любой жанр (вилка Шифу 27.10: отказа «движок так не играет» нет)."""
    if synth_fit(genre) >= kn.SYNTH_FIT_MIN:
        return synth_style(genre) or kn.DEFAULT_STYLE
    return kn.GEN_BRIDGE_STYLES.get(genre, kn.DEFAULT_STYLE)


def _synth_option(ask: SourceAsk, ctx: SourceContext, piece: bool) -> Optional[Option]:
    # «на синтезаторе»/«на Ренардо»: синт играет всегда — ни годность стиля, ни режим ``generate`` не отговорка
    named = engine_route(ask.engine) == SYNTH
    if ask.vocal or (not named and (ctx.mode == "generate"
                                    or (not piece and synth_fit(ask.genre) < kn.SYNTH_FIT_MIN))):
        return None
    key = PIECE if piece else synth_key(ask.genre)
    return Option(f"{SYNTH}:{key}", {"fallback": 1.0, "theme_match": 1.0 if ask.genre else 0.0,
                                     "asked": 1.0 if named else -1.0 if ask.generated else 0.0,
                                     "wait_s": kn.INSTANT_WAIT_S},
                  "своё: синтез" + ("" if piece else f" «{kn.GEN_STYLES.get(str(key), kn.GEN_STYLES['club']).name}»"))


def _gen_option(ask: SourceAsk, ctx: SourceContext) -> Tuple[Optional[Option], str]:
    kind = fork_gen_kind(ask.vocal, ask.robot_voice)
    model = kn.GEN_MODELS[kn.DEFAULT_GEN_MODEL]
    reason = ctx.song_block or gen_block(ctx, model.credits_per_task)
    if reason:
        return None, reason
    asked = ask.generated or ask.vocal or ask.fresh or ask.own
    return Option(f"{GENERATE}:{kind}:{kn.DEFAULT_GEN_MODEL}", {
        "theme_match": 1.0 if ask.genre else 0.0, "asked": 1.0 if asked else 0.0,
        "cost": model.credits_per_task, "wait_s": model.typical_s}, f"новая генерация ({kind})"), ""


def _theme(ask: SourceAsk, track: CacheTrack) -> Optional[float]:
    """Совпадение трека кэша с жанром просьбы: тот же — 1, та же семья синтеза (гранж — рок) — 0.5, чужой — ``None``."""
    if not ask.genre:
        return 0.0
    if track.style_key == ask.genre:
        return 1.0
    mine, asked = kn.GEN_STYLES.get(track.style_key), kn.GEN_STYLES[ask.genre]
    return 0.5 if mine is not None and mine.synth is not None and mine.synth == asked.synth else None


def _not_my_voice(ask: SourceAsk, track: CacheTrack, voice: str) -> bool:
    """«Своим голосом» — не песня голосом робота, или спета другим голосом, чем сейчас (ADR-0158 В8)."""
    return ask.robot_voice and (track.kind != "robot_song" or bool(voice and track.voice and track.voice != voice))


def _cache_option(ask: SourceAsk, track: CacheTrack, voice: str = "") -> Optional[Option]:
    """Кандидат трека кэша или ``None``: чужой жанр, «не нравится», петь просили — инструментал, «своим голосом» —
    не песня голосом робота (или не текущим)."""
    theme = _theme(ask, track)
    if theme is None or track.rating < 0 or (ask.vocal and track.instrumental) or _not_my_voice(ask, track, voice):
        return None
    asked = next((value for slot, value in kn.SOURCE_CACHE_ASKED if getattr(ask, slot)), 0.0)
    if not ask.vocal and not track.instrumental:  # петь не просили — песня хуже инструментала
        asked += kn.SOURCE_CACHE_SONG_PENALTY
    fresh = track.played_ago_h is None or track.played_ago_h >= kn.GEN_FRESH_HOURS
    return Option(f"{CACHE}:{track.id}", {"cache_theme": theme, "asked": asked, "fresh": float(fresh),
                                          "rating": float(track.rating), "wait_s": kn.INSTANT_WAIT_S},
                  f"сгенерированное раньше «{track.title}» ({track.kind}, {track.style_key or '—'})")


def _cache_options(ask: SourceAsk, ctx: SourceContext) -> List[Option]:
    """Кандидаты кэша: варианты одной задачи — один кандидат (первый в порядке кэша)."""
    seen, out = set(), []
    for track in ctx.cache:
        group = track.group or f"{track.kind}|{track.title}"
        option = None if group in seen else _cache_option(ask, track, ctx.voice)
        if option is not None:
            seen.add(group)
            out.append(option)
    return out


def set_cache(ask: SourceAsk, ctx: SourceContext, theme_tags: str = "") -> List[CacheTrack]:
    """Треки кэша для сета (ADR-0158 §2.7): инструментал жанра просьбы (тот же жанр или семья синтеза), не
    «не нравится», с тегами темы сета (``theme_tags``, «» — любая тема); давно не игранные первыми, затем свой жанр
    раньше семьи. Варианты одной задачи — все (соседями их не ставит ``engine.gen.gen_session``)."""
    def order(track: CacheTrack) -> Tuple[int, float, float]:
        fresh = track.played_ago_h is None or track.played_ago_h >= kn.GEN_FRESH_HOURS
        return (0 if fresh else 1, -(_theme(ask, track) or 0.0),
                -(track.played_ago_h if track.played_ago_h is not None else 1e9))

    usable = [t for t in ctx.cache if t.instrumental and t.kind == "instrumental" and t.rating >= 0
              and _theme(ask, t) is not None and theme_tags in t.prompt]
    return sorted(usable, key=order)


def _theme_stems(text: str) -> Set[str]:
    return {w[:kn.THEME_STEM_LEN] for w in _words(text)
            if len(w) >= kn.THEME_WORD_MIN and not w.startswith(kn.THEME_FIT_STOP)}


def theme_fit(theme_tags: str, track: CacheTrack) -> float:
    """Насколько трек кэша по теме сета (#3775, решение Шифу 21.10 «подогнать под тему что есть»): метка темы целиком
    в ``prompt`` — 1; иначе доля основ слов темы (``theme_tags``), которые есть в ``prompt``, названии и имени стиля
    трека. Темы нет — 0. По теме — от ``THEME_FIT_MIN``."""
    if not theme_tags:
        return 0.0
    if theme_tags in track.prompt:
        return 1.0
    want = _theme_stems(theme_tags)
    style = kn.GEN_STYLES.get(track.style_key)
    have = _theme_stems(" ".join((track.prompt, track.title, style.name if style else "")))
    return len(want & have) / len(want) if want else 0.0


def theme_cache(ask: SourceAsk, ctx: SourceContext, theme_tags: str) -> List[CacheTrack]:
    """Треки кэша сета (:func:`set_cache` без фильтра темы) по теме (:func:`theme_fit` ≥ ``THEME_FIT_MIN``): ближе к
    теме — раньше, при равных — порядок :func:`set_cache`."""
    fits = [(theme_fit(theme_tags, t), t) for t in set_cache(ask, ctx)]
    ranked = sorted(((f, i, t) for i, (f, t) in enumerate(fits) if f >= kn.THEME_FIT_MIN), key=lambda x: (-x[0], x[1]))
    return [t for _f, _i, t in ranked]


def _clip(text: str) -> str:
    """Слова ``text`` через пробел, не длиннее ``GEN_THEME_MAX_CHARS`` (режется по слову)."""
    out = ""
    for word in text.split():
        if len(out) + len(word) + (1 if out else 0) > kn.GEN_THEME_MAX_CHARS:
            break
        out = f"{out} {word}" if out else word
    return out


def theme_tags(theme: str) -> str:
    """Тема в тегах заказа Suno и её метка в кэше (``CacheTrack.prompt``, фильтр :func:`set_cache`): EN-теги строки
    ``GEN_THEME_TAGS`` («патриотический» → patriotic, anthemic…); строки нет — слова темы как есть («пленников
    прибывших эскобар», #3775: без них сет на свою тему брал любой кэш стиля, а заказ уходил без темы).
    Темы нет — «»."""
    words = _words(theme)
    found = next((tags for stems, tags in kn.GEN_THEME_TAGS.items() if _has(words, stems)), "")
    return found or _clip(" ".join(words))


def theme_title(theme: str) -> str:
    """Название заказа по теме человека («Пленников прибывших эскобар»); темы нет — «» (название решает заказ)."""
    text = _clip(" ".join(re.findall(r"[^\W_]+", theme or "")))
    return text[:1].upper() + text[1:]


def _slot_genre(token: str, mark: Optional[str]) -> str:
    """Ключ ``GEN_STYLES`` слова перечня: жанр без синтеза, окно («гранж»), слово целиком («клуб»), слово стиля."""
    for stems in (kn.GEN_GENRE_WORDS, kn.WINDOW_WORDS):
        found = next((key for stem, key in stems.items() if key in kn.GEN_STYLES and token.startswith(stem)), "")
        if found:
            return found
    found = kn.SET_STYLE_EXACT.get(token) or mark or ""
    return found if found in kn.GEN_STYLES else ""


def set_genres(text: str) -> Tuple[Tuple[str, ...], Tuple[str, ...]]:
    """Стили сета по порядку слов человека и непонятые слова перечня стилей (ADR-0148: слоты — код, не LLM).
    «в стилях клуб джангл рэйв синтвейв и джаз» → (club, dnb, rave, synthwave, jazz); «в стиле клуб и полька» —
    «полька» непонята. Перечень — после «стил…/жанр…» до знака препинания или слова-конца (``SET_STYLE_LIST_STOP``)."""
    from .theme import style_marks

    tokens = re.findall(r"[^\W_]+|[.,!?;:]", (text or "").lower().replace("ё", "е"))
    marks = style_marks(tokens)
    genres = [_slot_genre(tok, mark) for tok, mark in zip(tokens, marks)]
    unknown: List[str] = []
    head = next((i for i, tok in enumerate(tokens) if tok.startswith(kn.SET_STYLE_LIST_STEMS)), None)
    for i in range(len(tokens) if head is None else head + 1, len(tokens)):
        tok = tokens[i]
        if not tok.isalnum() or tok.isdigit() or tok.startswith(kn.SET_STYLE_LIST_STOP):
            break
        if not genres[i] and tok not in kn.SET_STYLE_LIST_JOIN:
            unknown.append(tok)
    return tuple(dict.fromkeys(g for g in genres if g)), tuple(unknown)


@dataclass(frozen=True)
class CoverWork:
    """Пьеса заказа для кавера (ADR-0158 §2.9, В14): каталожное название (``work_id`` кэша), наш материал мелодии
    (id мелодии RTTTL/партитуры, из которой собран лид; «» — мелодии нет, кавера нет), инструмент заказа (ключ
    ``INSTRUMENTS``), название для фразы."""

    title: str
    material: str
    instrument: str = ""
    spoken: str = ""


def cover_style_key(work: CoverWork, ask: SourceAsk) -> str:
    """Стиль кавера: жанр заказа; без жанра и инструмента — ``COVER_DEFAULT_STYLE``; с инструментом — «»."""
    return ask.genre or ("" if work.instrument else kn.COVER_DEFAULT_STYLE)


def _same_cover(track: CacheTrack, work: CoverWork, style: str) -> bool:
    from .works import norm

    return (track.kind == "cover" and norm(track.work_id) == norm(work.title) and track.instrument == work.instrument
            and track.style_key == style and track.rating >= 0)


def _cover_options(ask: SourceAsk, ctx: SourceContext, work: CoverWork) -> Tuple[List[Option], str]:
    """Кавер пьесы: трек кэша того же ``work_id``/``instrument``/``style`` (из вариантов — давнее игранный) и
    ``generate:cover`` — есть наш материал мелодии (В14, решение Шифу 14.10: «если у нас есть материал — идём через
    ветку кавер») и деньги (жёсткие фильтры :func:`gen_block`)."""
    style = cover_style_key(work, ask)
    same = sorted((t for t in ctx.cache if _same_cover(t, work, style)),
                  key=lambda t: -1e9 if t.played_ago_h is None else -t.played_ago_h)
    out = [Option(f"{CACHE}:{t.id}", {"live": 1.0, "theme_match": 1.0, "asked": -1.0 if ask.fresh else 0.0,
                                      "fresh": float(t.played_ago_h is None or t.played_ago_h >= kn.GEN_FRESH_HOURS),
                                      "rating": float(t.rating), "wait_s": kn.INSTANT_WAIT_S},
                  f"кавер из сочинённого раньше «{t.title}»") for t in same[:1]]
    model = kn.GEN_MODELS[kn.COVER_MODEL]
    reason = ("" if work.material else "no_material") or gen_block(
        replace(ctx, gen_ready=ctx.cover_ready), model.credits_per_task)
    if not reason:
        out.append(Option(f"{GENERATE}:cover:{kn.COVER_MODEL}", {
            "live": 1.0, "asked": 1.0 if (ask.generated or ask.fresh) else 0.0, "cost": model.credits_per_task,
            "wait_s": kn.COVER_WAIT_S}, f"живая версия: кавер нашей мелодии ({style or work.instrument})"))
    return out, reason


def _named_cell(ask: SourceAsk, ctx: SourceContext, piece: bool) -> Optional[Tuple[Option, ...]]:
    """Клетка строки «синт назван» вилки (:func:`fork_cell`), названная словами, — один кандидат: синт назван
    (``ask.engine``) или петь вживую названо (``ask.live``: робот поёт под синт-подложку) — с пением ``live:<вид>``,
    без пения синтез (кэш и генерация не кандидаты, молчаливой подмены нет). Не названа — ``None``: путь выбирают
    кандидаты."""
    if not ask.live and engine_route(ask.engine) != SYNTH:
        return None
    cell = fork_cell(SYNTH, bool(ask.live or ask.vocal))
    if cell.route == LIVE:
        return (_live_option(ask.live or cell.kind, True, False),)
    synth = _synth_option(ask, ctx, piece)
    return (synth,) if synth is not None else None


def source_candidates(ask: SourceAsk, ctx: SourceContext, piece: bool = False,
                      work: Optional[CoverWork] = None) -> Tuple[Tuple[Option, ...], str]:
    """``(кандидаты, почему генерации нет)``. Порядок — синтез, генерация, кэш; звучащих нет — ``decline:<причина>``
    (отказ — только когда генерации физически нет: ключ, деньги, сервис; фраза предлагает другую клетку вилки).
    Пьеса с ``work`` (петь не просили) — вместо генерации кандидаты кавера (:func:`_cover_options`, ADR-0158 PR-4).
    Вилка Шифу 27.10 (2×2): синт назван и петь просят (``ask.live`` — #4079, или пение от тула генерации) — один
    ``live:<вид>``: поёт робот сам (``perform`` диалога); синт назван без пения — один синтез (жанр, который синт не
    играет, — ближайшим стилем, :func:`synth_key`)."""
    fixed = _named_cell(ask, ctx, piece)
    if fixed is not None:
        return fixed, ""
    synth = _synth_option(ask, ctx, piece)
    if piece and work is not None and not ask.vocal:
        covers, reason = _cover_options(ask, ctx, work)
        options = [o for o in [synth, *covers] if o is not None]
    else:
        gen, reason = _gen_option(ask, ctx)
        options = [o for o in (synth, gen) if o is not None] + ([] if piece else _cache_options(ask, ctx))
    return tuple(options) or (Option(f"{DECLINE}:{reason or 'no_source'}", {}, "честный отказ"),), reason


def _not_replayed(options: Sequence[Option], ctx: SourceContext) -> Tuple[Option, ...]:
    """Трек сета (В16): есть новый заказ — трек кэша, игранный ближе ``GEN_FRESH_HOURS``, не кандидат. Признак
    ``fresh`` (вес 0.5) его не опускал: у кэша кавера ``theme_match`` 1.5 — кэш выигрывал при любом прослушивании,
    и сет 14.10 повторил 4 трека сета часом раньше. Нового заказа нет — сыгранное лучше синтеза, остаётся."""
    if not any(o.id.startswith(GENERATE) for o in options):
        return tuple(options)
    recent = {f"{CACHE}:{t.id}" for t in ctx.cache
              if t.played_ago_h is not None and t.played_ago_h < kn.GEN_FRESH_HOURS}
    return tuple(o for o in options if o.id not in recent)


def set_candidates(ask: SourceAsk, ctx: SourceContext,
                   work: Optional[CoverWork] = None) -> Tuple[Tuple[Option, ...], str]:
    """Кандидаты трека сета (ADR-0158 §2.7, В14): у темы есть наш материал ``work`` и кавер возможен (кэш того же
    кавера или ``generate:cover`` при деньгах) — кавер и синтез-запас; инструментал по тегам (новый и из кэша) не
    кандидат: узнаваемость в приоритете (Шифу 14.10: «кавер в приоритете»). Иначе — как у трека без материала.
    Недавно игранный кэш при новом заказе — не кандидат (:func:`_not_replayed`)."""
    options, reason = source_candidates(ask, ctx)
    if work is None or ask.vocal or ask.engine:
        return _not_replayed(options, ctx), reason
    covers, why = _cover_options(ask, ctx, work)
    if not covers:
        return _not_replayed(options, ctx), why if why != "no_material" else reason
    return _not_replayed(covers + [o for o in options if o.id.startswith(SYNTH)], ctx), why


#: Ключ слота сета «кавер произведения темы в стиле трека» (В14, В16): ``cover|<стиль>|<произведение>`` — у каждого
#: произведения свой слот, второй вариант его задачи закрывает только его же трек; инструментал — ключ = стиль.
COVER_SLOT = "cover|"


def cover_slot(genre: str, title: str) -> str:
    """Ключ слота кавера произведения ``title`` (``CoverWork.title`` / ``work_id`` кэша) в стиле ``genre``."""
    from .works import norm

    return f"{COVER_SLOT}{genre}|{norm(title)}"


def slot_style(key: str) -> str:
    """Стиль слота сета: ``cover|<стиль>|…`` → стиль; ключ инструментала — сам стиль."""
    return key[len(COVER_SLOT):].split("|")[0] if key.startswith(COVER_SLOT) else key


@dataclass(frozen=True)
class Route:
    """Выбранный путь: ``kind`` (synth/cache/generate/decline), ``id`` кандидата, просьба, почему генерации нет,
    стиль синтеза, трек кэша, ``failed`` — генерация была выбрана и не вышла (В3)."""

    kind: str
    id: str
    ask: SourceAsk
    gen_reason: str = ""
    style: str = ""
    track: Optional[CacheTrack] = None
    failed: bool = False
    provider: str = ""
    work: Optional[CoverWork] = None


def _route(choice: Choice, ask: SourceAsk, ctx: SourceContext, reason: str,
           work: Optional[CoverWork] = None) -> Route:
    kind, _sep, rest = choice.id.partition(":")
    track = next((t for t in ctx.cache if t.id == rest), None) if kind == CACHE else None
    return Route(kind, choice.id, ask, reason, rest if kind == SYNTH else "", track, provider=choice.provider,
                 work=work)


def choose_route(ask: SourceAsk, ctx: SourceContext, chooser: Chooser, piece: bool = False,
                 work: Optional[CoverWork] = None, for_set: bool = False) -> Route:
    """Кандидаты кода (трек сета — :func:`set_candidates`) → ``chooser`` (один кандидат — без вопроса) →
    :class:`Route`."""
    options, reason = set_candidates(ask, ctx, work) if for_set else source_candidates(ask, ctx, piece, work)
    if len(options) == 1:
        return _route(Choice(options[0].id, 1.0, "code"), ask, ctx, reason, work)
    question = ChoiceQuestion(f"путь {ask.describe()}", options, weights=kn.SOURCE_WEIGHTS, instructions=INSTRUCTIONS)
    return _route(chooser.choose(question), ask, ctx, reason, work)


def choose_with_fallback(ask: SourceAsk, ctx: SourceContext, chooser: Chooser,
                         attempt: Optional[Callable[[Route], str]] = None, piece: bool = False,
                         work: Optional[CoverWork] = None) -> Route:
    """Выбор пути; выбрана генерация и ``attempt`` вернул причину провала (``""`` — вышло; исполнителя нет —
    ``not_wired``) — выбор заново с тем же контекстом без ``generate:*``, второго круга генерации нет (В3)."""
    route = choose_route(ask, ctx, chooser, piece, work)
    if route.kind != GENERATE:
        return route
    failed = attempt(route) if attempt is not None else "not_wired"
    if not failed:
        return route
    return replace(choose_route(ask, ctx.without_generate(failed), chooser, piece, work), failed=True)


def _asked_text(ask: SourceAsk) -> str:
    """Что просили у генерации (для фразы «… сейчас не могу»): спеть робот может всегда сам — не может только
    сгенерировать песню."""
    return ("сгенерировать песню своим голосом" if ask.robot_voice else "сгенерировать песню" if ask.vocal
            else "сгенерировать" if ask.generated else "")


def _poor_genre(ask: SourceAsk) -> str:
    """Имя жанра просьбы, который синтез не играет («гранж»), иначе «»."""
    return kn.GEN_STYLES[ask.genre].name if ask.genre and synth_fit(ask.genre) < kn.SYNTH_FIT_MIN else ""


def _excuse(route: Route, tail: str = ".") -> str:
    """Почему играет не то, что просили: «Сгенерировать сейчас не могу: <причина>.»; просьба исполнима — «»."""
    words = REASON_WORDS.get(route.gen_reason, route.gen_reason)
    if route.failed:
        return f"Сгенерировать не вышло: {words}{tail}"
    asked, genre = _asked_text(route.ask), _poor_genre(route.ask)
    if not words:
        return ""
    if asked:
        return f"{asked[:1].upper()}{asked[1:]} сейчас не могу: {words}{tail}"
    return f"{genre[:1].upper()}{genre[1:]} синтезом у меня не выходит, а {words}{tail}" if genre else ""


def _cover_what(work: CoverWork, ask: SourceAsk) -> str:
    """«на ситаре» / «в стиле «оркестр»» — чем будет живая версия (из таблиц, не от LLM)."""
    ins = kn.INSTRUMENTS.get(work.instrument)
    style = kn.GEN_STYLES.get(cover_style_key(work, ask))
    return f"на {ins.prepositional}" if ins else f"в стиле «{style.name}»" if style else ""


def cover_note(route: Route) -> str:
    """Фраза пути пьесы с кавером (В4, ADR-0148: от кода): мост — «живую готовлю, через ~N с, а пока своя»; кавера
    нет из-за денег/сервиса — «живую сейчас не сделаю: <причина>»; нашей мелодии нет — «»."""
    work = route.work
    if work is None or route.ask.vocal:
        return ""
    what = " ".join(p for p in ("Живую версию", _cover_what(work, route.ask)) if p)
    if route.kind == GENERATE:
        return f"{what} готовлю — секунд через {int(kn.COVER_WAIT_S)}, а пока своя."
    if route.kind == SYNTH and work.material and route.gen_reason:
        words = REASON_WORDS.get(route.gen_reason, route.gen_reason)
        return f"{what} {'сделать не вышло' if route.failed else 'сейчас не сделаю'}: {words}."
    return ""


def cover_failed_text(work: CoverWork, reason: str) -> str:
    """Генерация кавера упала, мост доигрывает (В3): фраза кода с причиной, без тихой подмены."""
    return f"Живой версии «{work.spoken or work.title}» не будет: {REASON_WORDS.get(reason, reason)}."


def cover_rejected_text(work: CoverWork, others: bool) -> str:
    """Suno не взял ни одну мелодию произведения трека сета (В17): пьесу называет код; дальше — другие произведения
    темы (``others``) или сет без неё."""
    return f"«{work.spoken or work.title}» сервис не взял — {'играю другие' if others else 'доиграю без него'}."


def cover_ready_text(work: CoverWork, ask: SourceAsk) -> str:
    """Кавер сгенерирован и встал на деку вместо моста."""
    tail = _cover_what(work, ask)
    return f"А вот живая версия «{work.spoken or work.title}»{' ' + tail if tail else ''}."


#: Причины, о которых синтез-запас молчит: генерации нет совсем или её выключили (просили своё).
QUIET_REASONS: Tuple[str, ...] = ("", "not_wired", "mode_synth")


def fallback_note(reason: str) -> str:
    """Синтез — запас (В13): генерация была бы, но нельзя — «Сгенерировать сейчас не могу: <причина>, играю своё.»;
    генерации нет совсем — «»."""
    if reason in QUIET_REASONS:
        return ""
    return f"Сгенерировать сейчас не могу: {REASON_WORDS.get(reason, reason)}, играю своё."


def _nearest_note(route: Route) -> str:
    """Синт назван, а жанр он не играет («метал на Ренардо»): играет ближайший стиль — фраза кода говорит какой
    (без тихой подмены и без отказа, вилка Шифу 27.10); иначе «»."""
    genre, style = _poor_genre(route.ask), kn.GEN_STYLES.get(route.style)
    if engine_route(route.ask.engine) != SYNTH or not genre or style is None:
        return ""
    return f"{genre[:1].upper()}{genre[1:]} на синте играю ближайшим стилем — {style.name}."


def route_note(route: Route) -> str:
    """Фраза перед фразой запуска синтеза/пьесы: почему своё, а не то, что просили или что шло бы по умолчанию
    (генерация, В13), или каким стилем названный синт играет жанр, которого не умеет; просьба исполнена — «»."""
    if route.work is not None:
        return cover_note(route)
    excuse = _excuse(route)
    if route.kind != SYNTH or route.style == PIECE:
        return excuse
    nearest = _nearest_note(route)
    if nearest:
        return nearest
    return f"{excuse[:-1]}, играю своё." if excuse else fallback_note(route.gen_reason)


def cache_text(route: Route) -> str:
    """Фраза запуска трека кэша (вся реплика): причина, если просили новое, и что ставим."""
    track = route.track
    title = f"«{track.title}»" if track else "трек"
    what = {"robot_song": "спетое раньше своим голосом", "song": "песню из сочинённого раньше",
            "cover": "кавер из сочинённого раньше"}.get(track.kind if track else "", "сочинённое раньше")
    return " ".join(p for p in (_excuse(route), f"Ставлю {what}: {title}.") if p)


def decline_text(route: Route) -> str:
    """Честный отказ (звука нет; только когда генерации физически нет — ключ, деньги, сервис): что просили, почему
    нельзя и другая клетка вилки Шифу 27.10 — песню робот споёт сам вживую под синт (:data:`LIVE_OFFER`), инструментал
    сыграет своим (стили, которые синтез играет). Путь ``live:*`` (#4079) — поёт сам робот, не тул музыки: тулу —
    :data:`LIVE_TOOL_TEXT` (генерацию не запускаю)."""
    if route.kind == LIVE:
        return LIVE_TOOL_TEXT
    head = _excuse(route) or f"Сыграть это сейчас не могу: {REASON_WORDS.get(route.gen_reason, 'нечем')}."
    if route.ask.vocal:
        return f"{head} {LIVE_OFFER_BY_REASON.get(route.gen_reason, LIVE_OFFER)}"
    offer = ", ".join(st.name for key, st in kn.GEN_STYLES.items()
                      if st.synth == key and kn.STYLES[key].synth_fit >= kn.SYNTH_FIT_MIN)
    return f"{head} Могу сыграть своё: {offer}."


__all__ = ["CACHE", "COVER_SLOT", "CacheTrack", "CoverWork", "DECLINE", "cover_failed_text", "cover_note",
           "cover_rejected_text",
           "cover_ready_text", "cover_slot", "cover_style_key", "slot_style", "GENERATE", "INSTRUCTIONS", "PIECE",
           "REASON_WORDS", "Route", "SYNTH",
           "SourceAsk", "SourceContext", "ask_genre", "ask_engine", "ask_tweak", "blank_engine", "cache_text", "choose_route", "choose_with_fallback",
           "decline_text", "engine_route", "engine_words", "fallback_note", "gen_block", "gen_kind", "QUIET_REASONS", "route_note", "route_seconds",
           "set_cache",
           "set_candidates", "set_genres", "settle_engine", "source_ask", "source_candidates", "synth_fit", "synth_style", "theme_tags",
           "theme_cache", "theme_fit", "theme_title"]
__all__ += ["BY_CODE", "LIVE", "LIVE_OFFER", "LIVE_OFFER_BY_REASON", "LIVE_TOOL_TEXT", "SingContext", "SingRoute", "choose_sing",
            "sing_candidates", "sing_kind", "sing_markers", "synth_key"]
__all__ += ["fork_cell", "fork_gen_kind"]
