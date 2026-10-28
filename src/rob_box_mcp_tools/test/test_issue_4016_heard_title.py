"""#4016 — английское название, записанное STT кириллицей, не урезается молча до одного понятого слова.

Живой прогон 24.10 (Sound Pi): «робот спой раннинг ап дэд хилл на балалайке и гуслях» → ``непонятые=['спой',
'раннинг','балалайке','гуслях'] → «хилл»`` (LLM-разбор опоздал: 3–8 с против окна 1.5 с) → «King Of The Hill». Теперь:
непонятых слов названия больше понятых — код ищет запись архива, чьё название звучит этими словами (транслит одной
функцией + звуковой ключ), пока LLM думает; LLM опоздала — играет находка; не нашлось и LLM не подтвердила — фраза
кода «не уверен, что это за песня — «…»?», а не пьеса по обрывку. Тесты — на настоящем архиве RTTTL.

Ревью PR #4124: переспрос не должен ломать заказы, которые на develop играли верно, — пояснение после названия
(«имперский марш дарта вейдера», «марио только быстрее») не делает название неясным; жанровое слово в начале названия
(«рок эраунд зе клок») звучит; двухсловное название не собирается из слов вразбивку («си … хэлл» — не «Say Hello»).

Ревью PR #4124, круг 2: слова-связки перед понятым именем («что-то похожее на марио», «без сета, просто Баха») не
переспрашиваются — понятое имя называет мелодию само; находка на слух — догадка, её сверяют кэш и LLM (назвали другое —
решают они); частичное слабое совпадение («уи вил рок ю» → «Rock With You») и исполнитель не целиком — не находка.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from rob_box_mcp_tools.base import MCPToolResult
from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_mcp_tools.engine.request_slots import Catalog, RequestSlotParser, doubt_text
from rob_box_mcp_tools.engine.search import (HEARD_SLACK_WORD, ThemeHits, _aligned, _vocab, find, heard_title,
                                             names_itself, phonetic_key, terms, warm_heard)
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, NOT_FOUND, RequestMusicTool, _classic_outcome
from rob_box_music import knowledge as kn
from rob_box_music.request_slots import RequestSlots
from rob_box_voice.core.media_command_grammar import parse_media_command

from .test_adr0158_pr7_song import FakeStudio
from .test_engine_session import _rig
from .test_issue_3609_request_slots import FakeLLM, Log, NoScores
from .test_issue_3604_composer_filter import HITS, _both

pytestmark = pytest.mark.unit

LIVE_PHRASE = "спой раннинг ап дэд хилл на балалайке и гуслях"  # дословно из лога 24.10
NOWHERE = "сыграй абырвалг шмабырвалг хилл"  # одно понятое слово из трёх, такого названия в архиве нет


@pytest.fixture(scope="module")
def archive(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("heard") / "voice_memory.db"))


def _parser(archive, llm=None):
    log = Log()
    parser = RequestSlotParser(lambda text: terms(archive, text), Catalog(lambda: archive, NoScores()), ask=llm,
                               spawn=lambda fn: fn(), log=lambda: log)
    return parser, log


def _parse(parser, text):
    command = parse_media_command(text)
    return parser.parse(text, command.name or text, command)


# ── звучание слова: одна функция транслита + звуковой ключ ─────────────────────────────────────────────────────

@pytest.mark.parametrize("heard, spelled", [("раннинг", "running"), ("ап", "up"), ("дэд", "that"), ("хилл", "hill"),
                                            ("стоунз", "stones"), ("блэк", "black"), ("пэинт", "paint"),
                                            ("джуд", "jude"), ("вонт", "want")])
def test_cyrillic_transcription_sounds_like_the_english_word(heard, spelled):
    assert phonetic_key(heard) == phonetic_key(spelled)


def test_different_words_do_not_sound_alike():
    assert phonetic_key("раннинг") != phonetic_key("run")
    assert phonetic_key("хилл") != phonetic_key("hills")
    assert phonetic_key("брейк") != phonetic_key("be")


# ── название на слух по архиву ─────────────────────────────────────────────────────────────────────────────────

def test_live_phrase_is_running_up_that_hill(archive):
    heard = heard_title(archive, LIVE_PHRASE)
    assert heard.words == ("раннинг", "ап", "дэд", "хилл")  # без «спой» и инструментов — слоты, не название
    assert (heard.known, heard.unknown) == (1, 3) and heard.unclear and heard.truncated
    assert heard.record["name"] == "runningu" and heard.name == "Running Up That Hill" and heard.field == "title"


@pytest.mark.parametrize("text, name", [("раннинг ап зэт хилл", "runningu"), ("пэинт ит блэк", "paintitb"),
                                        ("бэк ин блэк", "backinbl"), ("хэй джуд", "heyjude")])
def test_other_transcriptions_find_their_record(archive, text, name):
    assert heard_title(archive, text).record["name"] == name


def test_clear_phrases_are_not_searched_by_sound(archive):
    """Понятных не меньше непонятых — разбор как был: поиска на слух нет, урезания нет."""
    for text in ("спой калинку на балалайке", "кинг оф зе хилл", "давай вагнера полет волкирий ебанем"):
        heard = heard_title(archive, text)
        assert not heard.unclear and heard.record is None, text


def test_unknown_title_is_not_guessed_from_one_word(archive):
    """«хилл» — единственное понятое из трёх: уверенного выбора нет (не «King Of The Hill»)."""
    heard = heard_title(archive, NOWHERE)
    assert heard.record is None and heard.truncated and heard.spoken == "абырвалг шмабырвалг хилл"
    assert heard_title(archive, "ай вонт ту брейк фри").record is None  # «I Want To Be Free» — уже другое название


# ── разбор заказа: на слух без LLM, иначе — честный переспрос ──────────────────────────────────────────────────

def test_live_phrase_resolves_by_sound_when_llm_is_late(archive):
    """24.10 LLM отвечала 3–8 с при окне 1.5 с: не успела — играет находка на слух. Находка — догадка кода, не
    подтверждение (ревью #4124, круг 2): LLM спрошена, ``work_confirmed`` у находки нет."""
    llm = FakeLLM(outcome="late")
    parser, log = _parser(archive, llm)
    slots = _parse(parser, LIVE_PHRASE)
    assert llm.calls == [LIVE_PHRASE] and slots.source == "sound" and slots.by_sound and slots.doubt == ""
    assert slots.query == slots.work == "Running Up That Hill" and not slots.work_confirmed
    assert find(archive, slots.query).record["name"] == "runningu"  # строка поиска ведёт к хуку runningu
    assert "source=sound" in log.lines[-1] and "«Running Up That Hill»" in log.lines[-1]


def test_unknown_title_gives_an_honest_question_not_a_truncated_query(archive):
    parser, log = _parser(archive, FakeLLM())  # LLM ответила, но произведения не подтвердила
    slots = _parse(parser, NOWHERE)
    assert slots.doubt == "абырвалг шмабырвалг хилл" and not slots.work_confirmed
    assert doubt_text(slots.doubt) == "Не уверен, что это за песня — «абырвалг шмабырвалг хилл»?"
    assert doubt_text(slots.doubt) == kn.REQUEST_SLOTS_DOUBT_TEXT.format(heard=slots.doubt)
    assert "не уверен: «абырвалг шмабырвалг хилл»" in log.lines[-1]
    late, _log = _parser(archive, FakeLLM(outcome="late"))  # LLM не успела — то же, а не «хилл»
    assert _parse(late, NOWHERE).doubt == "абырвалг шмабырвалг хилл"


def test_title_confirmed_by_llm_is_played_not_questioned(archive):
    llm = FakeLLM(answers={NOWHERE: {"work": "King Of The Hill"}})
    slots = _parse(_parser(archive, llm)[0], NOWHERE)
    assert slots.work_confirmed and slots.doubt == "" and slots.query == "King Of The Hill"


# ── заказ целиком: переспрос кода, ни пьесы, ни песни по обрывку ───────────────────────────────────────────────

def _request(tmp_path, archive, studio=None):
    rig = _rig()
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: _both(HITS, False), seed=lambda: 1)
    parser = RequestSlotParser(lambda text: terms(archive, text), Catalog(lambda: archive, NoScores()),
                               spawn=lambda fn: fn())
    return RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 5, records=lambda ids: {},
                            words=lambda word: _both(HITS, False), slots=parser,
                            gen_library=lambda: GeneratedMusicLibrary(str(tmp_path / "lib")),
                            play_cached=lambda track_id: MCPToolResult(success=True, data={}), songs=studio), rig


def test_tool_asks_back_instead_of_playing_a_fragment(tmp_path, archive):
    req, rig = _request(tmp_path, archive)
    result = req.execute(intent="melody", text=NOWHERE, heard_text=f"Робот, {NOWHERE}")
    assert not result.success and result.data["reason"] == NOT_FOUND
    assert result.error == "Не уверен, что это за песня — «абырвалг шмабырвалг хилл»?"
    assert not rig.owner.is_playing()


def test_sing_request_with_an_unclear_title_does_not_start_a_song(tmp_path, archive):
    studio = FakeStudio()
    req, rig = _request(tmp_path, archive, studio)
    text = "спой абырвалг шмабырвалг хилл"
    result = req.execute(intent="melody", text=text, heard_text=f"Робот, {text}")
    assert not result.success and result.error == doubt_text("абырвалг шмабырвалг хилл")
    assert studio.requests == [] and not rig.owner.is_playing()


# ── ревью #4124: заказы, которые на develop играли верно, не переспрашиваются ─────────────────────────────────

@pytest.mark.parametrize("text, title, query", [("имперский марш дарта вейдера", "Имперский марш", "imperial march"),
                                                ("музыку как у марио только быстрее", "Марио", "mario"),
                                                ("марио только быстрее", "Марио", "mario")])
def test_known_piece_with_an_explanation_plays_and_is_not_asked_back(tmp_path, archive, text, title, query):
    """Пояснение после названия («дарта вейдера», «только быстрее») — не название: непонятые слова после последнего
    понятого не делают его неясным. Заказ играет (как на develop) и с LLM, которая опоздала (24.10: late=4/4)."""
    heard = heard_title(archive, text)
    assert not heard.unclear and not heard.truncated
    req, _rig_ = _request(tmp_path, archive)
    result = req.execute(intent="melody", text=text, heard_text=f"Робот, {text}")
    assert result.success and result.data["title"] == title, result.error
    slots = _parse(_parser(archive, FakeLLM(outcome="late"))[0], text)
    assert slots.doubt == "" and slots.query == query and slots.source == "code"


def test_library_alias_is_understood_as_a_whole(archive):
    """«имперский марш» — алиас библиотеки (imperial march): поодиночке «имперский» архиву непонятно, целиком —
    понятно. Автор перед названием («дарта вейдера имперский марш») заказ неясным не делает: два на два."""
    heard = heard_title(archive, "дарта вейдера имперский марш")
    assert (heard.known, heard.unknown) == (2, 2) and not heard.unclear


@pytest.mark.parametrize("text", ["рок эраунд зе клок", "сыграй рок эраунд зе клок"])
def test_genre_word_starting_the_title_is_heard(tmp_path, archive, text):
    """«рок» — слот заказа (жанр) и первое слово «Rock Around The Clock»: в звучании оно есть, даже когда грамматика
    сняла его перед названием («сыграй рок …» → название «эраунд зе клок»). Целевой класс #4016 — английское название
    кириллицей — не уходит в переспрос."""
    slots = _parse(_parser(archive)[0], text)
    assert slots.source == "sound" and slots.query == "Rock Around The Clock" and slots.doubt == ""
    req, _rig_ = _request(tmp_path, archive)
    result = req.execute(intent="melody", text=text, heard_text=f"Робот, {text}")
    assert result.success and result.data["title"] == "Rock Around The Clock", result.error


def _keys(*words):
    return [(phonetic_key(w), len(w) <= HEARD_SLACK_WORD) for w in words]


def test_two_word_title_is_not_assembled_from_scattered_words(archive):
    """Ревью #4124: «эй ти ди си хайвэй ту хэлл» находило «Say Hello» («си» … «хэлл» через два чужих слова) и ещё
    подтверждало его. У названия из двух значимых слов чужое слово слышно только на месте его служебного слова."""
    heard = heard_title(archive, "эй ти ди си хайвэй ту хэлл")
    assert heard.record["name"] != "sayhello" and heard.name == "Highway To Hell"
    assert _aligned("Say Hello", _keys("си", "хайвэй", "ту", "хэлл")) is None
    assert _aligned("King Of The Hill", _keys("кинг", "оф", "зе", "хилл")) is not None  # «зе» на месте the
    assert _aligned("Hey Jude", _keys("хэй", "джуд")) is not None


def test_weak_key_word_is_heard_only_next_to_its_neighbour(archive):
    """Ревью #4124 (минимальная сила ключа): слово названия с ключом из одной согласной («эту» — ``at``, «си» — ``s``)
    совпадает со словами фразы случайно — через чужие слова оно не слышно. «ну эту как её там с хэллоу» — не «Out Of
    Time» («эту» … «там» через «её»); названия, где слабые слова стоят рядом, находятся как были."""
    assert heard_title(archive, "ну эту как её там с хэллоу").record is None
    assert _aligned("Out Of Time", _keys("эту", "ее", "там")) is None
    assert _aligned("Say Hello", _keys("си", "ди", "хэлл")) is None
    for text, title in (("лет ит би", "Let It Be"), ("уопс ай дид ит эгейн", "Oops I Did It again"),
                        ("хайвэй ту хэлл", "Highway To Hell"), ("тайм ту сэй гудбай", "Time To Say Goodbye")):
        assert heard_title(archive, text).name == title, text


# ── ревью #4124, круг 2: слова-связки перед названием или автором — не переспрос ────────────────────────────────

#: Заказы, которые на develop играли верно (зонд ревью: полный тул, настоящий архив): слова-связки перед понятым словом,
#: которое само называет мелодию (алиас реестра) или автора (``knowledge.COMPOSER_RU``), и строка поиска develop.
#: «сыграй без сета, просто Баха» — фраза живой проверки ROBOT_SIX (``test_adr0157_pr1_instrument_slot.py``). «Ещё
#: раз марио»: непонятых слов у ``terms`` нет (короткие отброшены) — строка поиска develop — фраза целиком.
FILLERS = [("сыграй что-то похожее на марио но медленнее", "mario"), ("сыграй что-нибудь похожее на марио", "mario"),
           ("сыграй что-то вроде марио", "mario"), ("давай ту самую штуку из марио", "mario"),
           ("сыграй мне ещё раз марио", "сыграй мне ещё раз марио"), ("сыграй что-то похожее на тетрис", "tetris"),
           ("сыграй тот самый тетрис", "tetris"), ("сыграй без сета, просто Баха", "баха"),
           ("сыграй просто какую-нибудь вещь баха", "баха"), ("без всяких сетов сыграй моцарта", "моцарта"),
           ("сыграй маленькую ночную серенаду моцарта", "моцарта"), ("сыграй времена года вивальди", "вивальди"),
           ("сыграй что-нибудь вроде вивальди", "вивальди"), ("сыграй на ре нардо космос", "космос")]


@pytest.mark.parametrize("text, query", FILLERS)
def test_filler_words_before_a_known_name_are_not_asked_back(archive, text, query):
    """Ревью #4124, круг 2: три непонятых слова-связки («что-то похожее на», «тот самый», «без сета, просто») перед
    понятым словом давали переспрос, хотя понятое само называет мелодию или автора. Решает код
    (``search.names_itself``), а не таблица слов-связок: разбор как на develop — и без LLM, и с опоздавшей LLM (бой
    24.10: late=4/4)."""
    heard = heard_title(archive, parse_media_command(text).name or text, text)
    assert heard.named and not heard.truncated and heard.record is None  # и на слух не ищется: название понятно
    for llm in (None, FakeLLM(outcome="late")):
        slots = _parse(_parser(archive, llm)[0], text)
        assert slots.doubt == "" and slots.query == query and slots.source == "code", (text, slots)


@pytest.mark.parametrize("text", ["сыграй без сета, просто Баха", "сыграй что-то похожее на марио но медленнее",
                                  "сыграй тот самый тетрис"])
def test_tool_plays_filler_phrases_instead_of_asking_back(tmp_path, archive, text):
    req, _rig_ = _request(tmp_path, archive)
    result = req.execute(intent="melody", text=text, heard_text=f"Робот, {text}")
    assert "doubt" not in (result.data or {}) and "Не уверен" not in str(result.error or ""), result


def test_names_itself_is_the_registry_or_a_composer_not_a_title_word(archive):
    """Алиас реестра (латиница: «марио» → mario), семя-понятие («космос»), фамилия композитора («баха» → bach) — имя;
    слово, которое архив знает только как слово названий («хилл» → hill), и цифра — нет."""
    def named(word):
        return [names_itself(t) for t in terms(archive, word)]

    assert named("марио") == [True] and named("космос") == [True] and named("баха") == [True]
    assert named("вивальди") == [True] and named("хилл") == [False] and named("4") == [False]


# ── ревью #4124, круг 2: находка на слух — догадка, её сверяют LLM и кэш; слабая или частичная — не находка ─────

@pytest.mark.parametrize("text, answer, confirmed", [
    ("сыграй уи вил рок ю", {"composer": "Queen", "work": "We Will Rock You"}, "We Will Rock You"),
    ("сыграй раннинг ап дэд хилл", {"composer": "Kate Bush", "work": "Running Up That Hill"},
     "Running Up That Hill")])
def test_llm_in_time_confirms_the_work_itself(archive, text, answer, confirmed):
    """LLM вовремя подтвердила каталогом то же произведение — находка подтверждена; строка поиска — название записи, а
    не «автор LLM + название» (у раннинга в архиве кавер Within Temptation: develop с такой LLM отвечал «Пьес автора
    «Bush hill» у меня не нашлось»)."""
    llm = FakeLLM(answers={text: answer})
    slots = _parse(_parser(archive, llm)[0], text)
    assert llm.calls == [text] and slots.source == "llm" and slots.work_confirmed and slots.work == confirmed
    assert slots.query == confirmed


def test_unclear_title_asks_llm_before_asking_back(archive):
    """«ты ди си хэлл»: ``terms`` отбросил короткие «ты ди си» — непонятых для LLM не было, и заказ сразу шёл в
    переспрос. Переспрос — только после LLM: вовремя назвала «Highway To Hell» — играет оно."""
    text = "сыграй ты ди си хэлл"
    llm = FakeLLM(answers={text: {"composer": "AC/DC", "work": "Highway To Hell"}})
    slots = _parse(_parser(archive, llm)[0], text)
    assert llm.calls == [text] and slots.doubt == "" and slots.work_confirmed and slots.work == "Highway To Hell"
    assert _parse(_parser(archive)[0], text).doubt == "ты ди си хэлл"  # без LLM — честный переспрос


def test_llm_that_named_another_work_overrules_the_sound_guess(archive):
    """«тико тико» звучит как запись «Tick Tick», а LLM вовремя назвала «Tico Tico no Fuba» — её в архиве нет: решает
    разбор LLM (как на develop — ``not_found`` дальше), а не находка на слух; переспроса нет — понятых слов нет."""
    text = "сыграй тико тико"
    assert heard_title(archive, "тико тико").name == "Tick Tick"  # без LLM находка та же (ложная, см. тело PR)
    llm = FakeLLM(answers={text: {"work": "Tico Tico no Fuba"}})
    slots = _parse(_parser(archive, llm)[0], text)
    assert llm.calls == [text] and slots.source == "llm" and not slots.by_sound
    assert slots.query == "тико тико" and not slots.work_confirmed and slots.rejected == ("work=Tico Tico no Fuba",)
    assert slots.doubt == ""


def test_llm_is_asked_before_the_sound_search_and_thinks_meanwhile(archive):
    """Ревью #4124 (скорость): поиск на слух — сотни мс (строки поиска по архиву). Есть непонятые слова — вопрос LLM
    уходит до него, и окно ожидания LLM (1.5 с) с поиском не складывается."""
    llm, order = FakeLLM(outcome="late"), []

    class Spy(Catalog):
        def heard(self, text, said=""):
            order.append(len(llm.calls))
            return super().heard(text, said)

    parser = RequestSlotParser(lambda text: terms(archive, text), Spy(lambda: archive, NoScores()), ask=llm,
                               spawn=lambda fn: fn())
    assert _parse(parser, "сыграй раннинг ап дэд хилл").by_sound and order == [1]


def test_silent_llm_leaves_the_sound_guess(archive):
    """LLM ответила, но ни произведения, ни автора не назвала — находке на слух не возражает: играет она, без
    подтверждения каталогом."""
    llm = FakeLLM()  # ответ без слотов
    slots = _parse(_parser(archive, llm)[0], "сыграй раннинг ап дэд хилл")
    assert llm.calls and slots.by_sound and slots.query == "Running Up That Hill" and not slots.work_confirmed


@pytest.mark.parametrize("text, wrong", [("уи вил рок ю", "Rock With You"), ("ты ди си хэлл", "Say Hello"),
                                         ("вот эту ну там где та та та", "Out Of Time"),
                                         ("во поле берёза стояла", "Belle Perez")])
def test_weak_or_partial_sound_match_is_not_a_find(archive, text, wrong):
    """Ревью #4124, круг 2 (зонд: 5 неверных находок из 26): название, которое звучит лишь частью услышанных слов,
    засчитывается только сильным (согласных в ключах прозвучавших слов не меньше ``HEARD_EVIDENCE_MIN``); исполнитель
    — только целиком."""
    assert heard_title(archive, text).name != wrong


def test_sound_match_strength_rules():
    assert _aligned("Rock With You", _keys("уи", "вил", "рок", "ю")) is None  # «рок» + «ю» — 2 согласных из 4 слов
    assert _aligned("We Will Rock You", _keys("уи", "вил", "рок", "ю")) is not None  # все слова — название
    assert _aligned("Say Hello", _keys("ты", "ди", "си", "хэлл")) is None
    assert _aligned("I Got You Babe", _keys("ай", "гот", "ю", "бейб")) is not None  # слабое, но целиком
    assert _aligned("Highway To Hell", _keys("эй", "си", "ди", "си", "хайвэй", "ту", "хэлл")) is not None  # h+t+hl
    assert _aligned("Belle Perez", _keys("поле", "береза", "стояла"), whole=True) is None  # исполнитель — целиком
    assert _aligned("Rolling Stones", _keys("роллинг", "стоунз"), whole=True) is not None


def test_partner_word_finds_the_title_a_common_anchor_hides(archive):
    """«рок» — слово сотен записей: «We Will Rock You» в первые полсотни по одному «rock» не попадал, и находкой была
    «Rock With You». Строка поиска «rock will» (опора + напарник ``HEARD_PARTNER_MIN``) находит саму запись."""
    assert heard_title(archive, "уи вил рок ю").name == "We Will Rock You"


def test_sound_match_counts_as_named_for_a_sing_request():
    """Название узнано на слух (``RequestSlots.by_sound``) — пьесу назвал человек (``played["named"]``, #4015): «спой
    раннинг ап дэд хилл» — чужая песня по названию, хотя каталог её не подтвердил."""
    pick = SimpleNamespace(program="p", title="Running Up That Hill", melody_id="runningu", bpm=100, key="C",
                           history={}, track={}, work="Running Up That Hill")
    staged = _classic_outcome(None, "Running Up That Hill", (pick, {"named": False}, [], 1), None, True)
    assert staged["played"]["named"] is True
    assert _classic_outcome(None, "x", (pick, {"named": False}, [], 1), None)["played"]["named"] is False
    assert RequestSlots(source="sound").by_sound and not RequestSlots(source="llm").by_sound


def test_sound_index_is_warmed_at_start_not_in_the_first_order(archive):
    """Ревью #4124: указатель звучаний (14 тыс. слов) строился в первом заказе с непонятым названием (0.15–1 с).
    Прогрев ноды (``RequestMusicTool.warm`` → ``RequestSlotParser.warm``) строит его в фоне при старте."""
    assert warm_heard(archive) > 1000
    _vocab(archive)._sounds = None  # noqa: SLF001 — как после рестарта: указателя нет
    rig = _rig()
    parser, log = _parser(archive)
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 1)
    tool = RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, records=lambda ids: {},
                            words=lambda word: ThemeHits(), slots=parser)
    thread = tool.warm()
    thread.join(120)
    assert not thread.is_alive() and _vocab(archive)._sounds  # noqa: SLF001
    assert any("прогрев: указатель звучаний" in line for line in log.lines)
