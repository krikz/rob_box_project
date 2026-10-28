"""#4016 — английское название, записанное STT кириллицей, не урезается молча до одного понятого слова.

Живой прогон 24.10 (Sound Pi): «робот спой раннинг ап дэд хилл на балалайке и гуслях» → ``непонятые=['спой',
'раннинг','балалайке','гуслях'] → «хилл»`` (LLM-разбор опоздал: 3–8 с против окна 1.5 с) → «King Of The Hill». Теперь:
непонятых слов названия больше понятых — код ищет запись архива, чьё название звучит этими словами (транслит одной
функцией + звуковой ключ), без ожидания LLM; не нашлось и LLM не подтвердила — фраза кода «не уверен, что это за
песня — «…»?», а не пьеса по обрывку. Тесты — на настоящем архиве RTTTL.

Ревью PR #4124: переспрос не должен ломать заказы, которые на develop играли верно, — пояснение после названия
(«имперский марш дарта вейдера», «марио только быстрее») не делает название неясным; жанровое слово в начале названия
(«рок эраунд зе клок») звучит; двухсловное название не собирается из слов вразбивку («си … хэлл» — не «Say Hello»).
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.base import MCPToolResult
from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_mcp_tools.engine.request_slots import Catalog, RequestSlotParser, doubt_text
from rob_box_mcp_tools.engine.search import (HEARD_SLACK_WORD, ThemeHits, _aligned, _vocab, find, heard_title,
                                             phonetic_key, terms, warm_heard)
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, NOT_FOUND, RequestMusicTool
from rob_box_music import knowledge as kn
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

def test_live_phrase_resolves_by_sound_without_waiting_for_llm(archive):
    llm = FakeLLM(delay=5.0)  # LLM 24.10 отвечала 3–8 с: её не ждём вовсе
    parser, log = _parser(archive, llm)
    slots = _parse(parser, LIVE_PHRASE)
    assert llm.calls == [] and slots.source == "sound" and slots.doubt == ""
    assert slots.query == slots.work == "Running Up That Hill" and slots.work_confirmed
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
