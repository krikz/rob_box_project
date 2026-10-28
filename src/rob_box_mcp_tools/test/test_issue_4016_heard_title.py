"""#4016 — английское название, записанное STT кириллицей, не урезается молча до одного понятого слова.

Живой прогон 24.10 (Sound Pi): «робот спой раннинг ап дэд хилл на балалайке и гуслях» → ``непонятые=['спой',
'раннинг','балалайке','гуслях'] → «хилл»`` (LLM-разбор опоздал: 3–8 с против окна 1.5 с) → «King Of The Hill». Теперь:
непонятых слов названия больше понятых — код ищет запись архива, чьё название звучит этими словами (транслит одной
функцией + звуковой ключ), без ожидания LLM; не нашлось и LLM не подтвердила — фраза кода «не уверен, что это за
песня — «…»?», а не пьеса по обрывку. Тесты — на настоящем архиве RTTTL.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.base import MCPToolResult
from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_mcp_tools.engine.request_slots import Catalog, RequestSlotParser, doubt_text
from rob_box_mcp_tools.engine.search import find, heard_title, phonetic_key, terms
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
    assert heard.known == 1 and heard.unclear and heard.truncated
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
