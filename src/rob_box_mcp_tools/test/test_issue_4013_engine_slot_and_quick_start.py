"""#4013 и #3960 п.1/п.2 — путь музыки с движком Ренардо: слот ``engine`` решает код, старт трека не ждёт LLM.

(A) LLM ставила ``engine=synth`` на любую реплику с инструментом («сыграй X на балалайке»), а код принимал слот без
проверки по словам человека: путь сужался до синтеза, кавер (KIE/Suno) не рассматривался. Теперь слот принимается,
только если в словах хода есть опора (``knowledge.MUSIC_ENGINES``: основы и ядро имени, искажения STT), иначе
отбрасывается с причиной в лог; играющий движок наследует код по снимку плеера («ускорь», «добавь баса»).

(B) ``request_music(engine=synth, mood)`` ждал LLM-разбор темы (theme-links, ``submit_catalog_queries``) 4–10 с и
упирался в потолок действия музыки 10 с: ложное «Музыка пока не заиграла.» при старте трека через 1.4 с (марафон 23.10,
n809). Тема одиночного трека — без глаголов, названия движка и служебных слов, поиск — без ожидания LLM.
"""

import logging
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from rob_box_core.tool_catalog import get_tool
from rob_box_dialog.command import _schema_error
from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine import tools_v2
from rob_box_mcp_tools.engine.classic import ClassicPick
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.theme_links import ThemeLinks
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, RequestMusicTool
from rob_box_music import knowledge as kn

from .test_engine_session import _rig

pytestmark = pytest.mark.unit

BALALAIKA = "Робот, сыграй что-нибудь на балалайке"
RHYTHM = "Робот, поставь фоном ровный ритм наррендарда только не громко"  # дословно из лога марафона 23.10, n809


class Finders:
    """Три поиска темы одиночного трека: кто из них позван, то и решение кода (LLM-ждущий — ``dj._find``)."""

    def __init__(self, dj):
        self.calls = {"quick": [], "track": [], "llm": []}
        dj.quick_find = lambda theme: self.calls["quick"].append(theme) or ThemeHits()
        dj.track_find = lambda theme: self.calls["track"].append(theme) or ThemeHits()
        dj._find = lambda theme: self.calls["llm"].append(theme) or ThemeHits()  # noqa: SLF001


def _tools(rig):
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 4242)
    req = RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 777,
                           classic=lambda query, seed: ClassicPick(query=query, found=False, reason="нет в базе"),
                           records=lambda ids: {}, words=lambda word: ThemeHits())
    return dj, req


def _asks(req):
    """Просьбы, с которыми заказ пошёл в выбор пути (по ним видно, какой движок решил код)."""
    seen = []
    real = req._source.route  # noqa: SLF001

    def spy(ask, **kwargs):
        seen.append(ask)
        return real(ask, **kwargs)

    req._source.route = spy  # noqa: SLF001
    return seen


# ── (A) слот engine ──────────────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("heard", [BALALAIKA, "Робот, сыграй рок", "Робот, сыграй Running Up That Hill на балалайке"])
def test_llm_engine_slot_without_support_in_the_words_is_dropped_with_a_reason(heard, caplog):
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    with caplog.at_level(logging.INFO):
        result = req.execute(intent="track", text=heard, engine="synth", heard_text=heard)
    assert asks[0].engine == "" and "engine" not in result.data
    assert any("слот engine=synth без опоры в словах человека — отброшен" in r.getMessage() for r in caplog.records)


@pytest.mark.parametrize("heard", [
    "Робот, поставь ровный ритм на виндарда",  # искажение STT дальше основ — опора есть (ядро имени)
    "Робот, сыграй нарен дарда", "Робот, сыграй на индардо", RHYTHM,
])
@pytest.mark.parametrize("slot", ["synth", None])
def test_engine_with_support_in_the_words_is_kept_with_or_without_the_slot(heard, slot):
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    result = req.execute(intent="track", text=heard, engine=slot, heard_text=heard)
    assert asks[0].engine == "synth" and result.success and result.data["engine"] == "synth"


@pytest.mark.parametrize("heard", [
    "Робот, сыграй на синтезаторе спокойный ночной бит",
    "Робот, сыграй вживую на синте что-нибудь",
    "Робот, сыграй живьём",
    "робот сыграй на рем дарда спокойный ночной бит что нибудь под пайку",  # живой прогон 27.10 (#4070): имя разрезано
])
def test_synth_path_named_by_meaning_plays_the_synth_not_a_library_track(heard):
    """#4070: «играй сам, синтезом» словами — тот же движок, что и по имени «Ренардо»; слот LLM не нужен."""
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    result = req.execute(intent="track", text=heard, heard_text=heard)
    assert asks[0].engine == "synth" and result.success and result.data["engine"] == "synth"
    assert rig.owner.now_playing()["engine"] == "synth"


def test_style_words_of_synth_do_not_name_the_synth_path():
    """«Синтвейв» — стиль, не путь: слот пуст, путь не сужен до синтеза."""
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    result = req.execute(intent="track", text="Робот, поставь синтвейв", heard_text="Робот, поставь синтвейв")
    assert asks[0].engine == "" and "engine" not in result.data


def test_words_of_the_synth_path_are_not_the_theme_of_the_track():
    """Слова по смыслу («на синтезаторе», «вживую», «на рем дарда») вырезаны из темы: поиск по ним не идёт."""
    for heard, theme in (("Робот, сыграй на синтезаторе спокойный ночной бит", "спокойный ночной бит"),
                         ("Робот, сыграй вживую спокойный ночной бит", "спокойный ночной бит"),
                         ("Робот, сыграй на рем дарда спокойный ночной бит", "спокойный ночной бит")):
        rig = _rig()
        dj, req = _tools(rig)
        finders = Finders(dj)
        assert req.execute(intent="track", text=heard, heard_text=heard).success
        assert finders.calls == {"quick": [], "track": [theme], "llm": []}, heard


def test_playing_engine_is_inherited_by_a_plain_follow_up_like_n506(caplog):
    """n504–n506: «ускорь», «добавь баса» под Ренардо — движок тот же, его наследует код по снимку плеера."""
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    first = req.execute(intent="track", text="сыграй на ренардо", heard_text="Робот, сыграй на ренардо")
    assert first.success and rig.owner.now_playing()["engine"] == "synth"  # движок — в снимке плеера
    with caplog.at_level(logging.INFO):
        for heard in ("Робот, и ускорь темп, а то мы тут засыпаем оба.",
                      "Робот, продолжай развивать эту тему и добавь баса пожирнее."):
            result = req.execute(intent="track", text=heard, heard_text=heard)
            assert asks[-1].engine == "synth" and result.success and result.data["engine"] == "synth"
    assert any("движок наследуется от играющего: synth" in r.getMessage() for r in caplog.records)


@pytest.mark.parametrize("heard,intent", [
    (BALALAIKA, "track"), ("Робот, сыграй рок", "track"), ("Робот, сгенерируй новый трек", "track"),
    ("Робот, ускорь и добавь баса на балалайке", "track"),  # правка, но с инструментом реплики — свой предмет
    ("Робот, поставь Калинку", "melody"),  # пьеса по названию
])
def test_playing_engine_is_not_inherited_by_a_request_with_its_own_subject(heard, intent):
    """Под играющим Ренардо «сыграй что-нибудь на балалайке» и «сыграй рок» — обычный путь, а не молчаливый синтез.
    Ренардо заводится заново в каждом случае: заказ другого трека снимает его факт из снимка плеера."""
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    assert req.execute(intent="track", text="на ренардо", heard_text="Робот, сыграй на ренардо").success
    req.execute(intent=intent, text=heard, heard_text=heard, engine="synth")
    assert len(asks) == 2 and asks[-1].engine == "", heard


#: Просьбы, на которых ревьюер PR #4037 показал липкое наследование: Ренардо держался до стопа, кавер и кэш исключались.
NOT_A_TWEAK = ("Робот, сыграй ещё", "Робот, поставь трек про космос", "Робот, включи музыку", "Робот, поставь Нирвану",
               "Робот, включи что-нибудь другое", "Робот, сделай потише")


@pytest.mark.parametrize("heard", NOT_A_TWEAK)
def test_playing_engine_is_not_inherited_by_a_request_that_does_not_tweak_what_plays(heard):
    """Наследуют только правки играющего (темп, бас, тембр, развитие); новая просьба идёт обычным путём."""
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    assert req.execute(intent="track", text="на ренардо", heard_text="Робот, сыграй на ренардо").success
    assert rig.owner.now_playing()["engine"] == "synth"  # Ренардо играет (поставлен на деку)
    result = req.execute(intent="track", text=heard, heard_text=heard)
    assert len(asks) == 2 and asks[-1].engine == "", heard
    assert "engine" not in result.data


@pytest.mark.parametrize("heard", [
    "Робот, ускорь темп", "Робот, замедли", "Робот, добавь баса", "Робот, жирнее", "Робот, развивай",
    "Робот, продолжи этот бит", "Робот, ещё такой же", "Робот, давай в том же духе",
])
def test_playing_engine_is_inherited_by_each_kind_of_tweak(heard):
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    assert req.execute(intent="track", text="на ренардо", heard_text="Робот, сыграй на ренардо").success
    result = req.execute(intent="track", text=heard, heard_text=heard)
    assert len(asks) == 2 and asks[-1].engine == "synth" and result.data["engine"] == "synth", heard


def test_engine_of_a_track_whose_start_failed_is_not_inherited():
    """``PlayerOwner.play`` кладёт факты заказа до ``adapter.start``: старт упал — факт ``engine`` остаётся в снимке при
    пустой деке. «Ускорь» после такого провала Ренардо не наследует: играть нечему."""
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)

    def boom(program, started):
        raise RuntimeError("scsynth недоступен")

    rig.adapter.start = boom
    failed = req.execute(intent="track", text="на ренардо", heard_text="Робот, сыграй на ренардо")
    assert not failed.success
    snap = rig.owner.now_playing()
    assert snap["engine"] == "synth" and not snap["playing"] and not snap["pending"]  # факт остался при пустой деке
    assert tools_v2.playing_engine(rig.owner) == ""
    heard = "Робот, ускорь темп"
    req.execute(intent="track", text=heard, heard_text=heard)
    assert len(asks) == 2 and asks[-1].engine == ""


def test_nothing_is_inherited_when_nothing_plays():
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    result = req.execute(intent="track", text="ускорь темп", heard_text="Робот, и ускорь темп")
    assert asks[0].engine == "" and "engine" not in result.data


def test_request_music_schema_takes_rock_and_asks_for_engine_only_when_named():
    """LLM предложила ``genre='rock'`` — схема его не принимала (``invalid:bad_args``, повтор хода +1.7 с)."""
    schema = get_tool("request_music").parameters
    args = {"intent": "melody", "text": "сыграй Running Up That Hill на балалайке в рок стиле", "genre": "rock",
            "engine": "synth"}
    assert _schema_error(schema, args, "request_music") is None
    rig = _rig()
    _dj, req = _tools(rig)
    assert "rock" in {p.name: p for p in req.parameters}["genre"].enum


# ── (B) старт трека не ждёт LLM-разбор темы ──────────────────────────────────────────────────────────────────────


def test_synth_mood_request_does_not_ask_the_llm_about_the_theme():
    """n809: «ровный ритм на Ренардо, только не громко» + mood=calm — каталог и хранилище, LLM-разбора темы нет."""
    rig = _rig()
    dj, req = _tools(rig)
    finders = Finders(dj)
    result = req.execute(intent="track", text="поставь фоном ровный ритм на Ренардо только не громко",
                         mood="calm", engine="synth", heard_text=RHYTHM)
    assert result.success and result.data["engine"] == "synth"
    assert finders.calls == {"quick": ["фоном ровный ритм громко"], "track": [], "llm": []}


def test_track_name_is_not_the_text_of_the_request():
    """Тема без находок — не название трека: не «поставь фоном ровный ритм на Ренардо только не громко»."""
    rig = _rig()
    dj, req = _tools(rig)
    Finders(dj)
    text = "поставь фоном ровный ритм на Ренардо только не громко"
    data = req.execute(intent="track", text=text, mood="calm", engine="synth", heard_text=RHYTHM).data
    assert data["name"] == "клубный трек" and data["title"].startswith("клубный трек")
    assert text not in data["title"] and "ренардо" not in data["title"].lower()


@pytest.mark.parametrize("heard,mood,theme,who", [
    ("Робот, поставь клубный трек", None, "", "quick"),  # тема пуста — искать нечего, LLM не нужна
    ("Робот, сыграй эпичное", "epic", "эпичное", "quick"),  # настроение — не произведение
    ("Робот, сыграй трек про динозавров", None, "динозавров", "track"),  # LLM — только фоном
])
def test_single_track_never_waits_for_the_llm_on_the_sound_path(heard, mood, theme, who):
    rig = _rig()
    dj, req = _tools(rig)
    finders = Finders(dj)
    assert req.execute(intent="track", text=heard, mood=mood, heard_text=heard).success
    # ждущий LLM поиск (как у сета, ``dj._find``) на одиночном треке не зовётся никогда
    assert finders.calls == {"quick": [], "track": [], "llm": [], **{who: [theme]}}


@pytest.fixture(scope="module")
def archive(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("links4013") / "voice_memory.db"))


HOLD_S = 30.0  # сколько LLM держит ответ, если тест её не отпустил


class SlowLLM:
    """LLM разбора темы (``reasoner.ask``): ждёт, пока тест её отпустит; ``calls`` — что у неё спросили."""

    def __init__(self):
        self.calls, self.release, self.finished = [], threading.Event(), threading.Event()
        self.asked = threading.Event()

    def __call__(self, *args):
        self.calls.append(args)
        self.asked.set()
        self.release.wait(HOLD_S)
        self.finished.set()
        return "fail", None, ""


#: Ожидание LLM у сета в этих тестах — дольше, чем она держит ответ (``SlowLLM``): заказ, который стал бы её ждать,
#: дождался бы ответа (``llm.finished``), а не отсидел таймаут. Без порогов по стене: тест не сравнивает время.
LONG_SET_WAIT_S = 60.0


@pytest.fixture()
def real_chain(archive, tmp_path):
    """Настоящая цепочка поиска темы (архив, хранилище связей, ``ThemeLinks``) с медленной LLM-разборщицей."""
    db = str(tmp_path / "links.db")
    library = SimpleNamespace(**{k: getattr(archive, k) for k in ("search", "get", "vocabulary")}, db_path=db,
                              _db_path=db)
    llm = SlowLLM()
    rig = _rig()
    links = lambda node, reasoner: ThemeLinks(ask=reasoner.ask, wait_s=LONG_SET_WAIT_S)  # noqa: E731
    with patch.object(tools_v2, "_rtttl_library", lambda: library), patch.object(tools_v2, "theme_links", links):
        dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, seed=lambda: 4242,
                       reasoner=SimpleNamespace(enabled=True, ask=llm))
        req = RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 777,
                               classic=lambda query, seed: ClassicPick(query=query, found=False, reason="нет в базе"),
                               records=lambda ids: {}, words=lambda word: ThemeHits())
    yield req, llm, rig
    llm.release.set()


def test_n809_synth_mood_request_starts_at_once_and_asks_no_llm_theme_links(real_chain):
    """Живой n809: между выбором синтеза и стартом стоял LLM ``submit_catalog_queries`` 9.7 с — теперь LLM не зовётся."""
    req, llm, rig = real_chain
    result = req.execute(intent="track", text="поставь фоном ровный ритм на Ренардо только не громко", mood="calm",
                         engine="synth", heard_text=RHYTHM)
    assert result.success and result.data["engine"] == "synth"
    assert rig.owner.is_playing()  # трек запущен (поставлен на деку, ждёт ``started``)
    assert llm.calls == []  # LLM-разбора темы нет совсем


def test_track_with_a_theme_asks_the_llm_only_in_the_background(real_chain):
    """Тема без находок и без настроения: LLM спрашивается, но заказ её не ждёт. Ожидание сета тут 60 с, а LLM держит
    ответ 30 с: заказ, ждущий её, вернулся бы после ответа (``finished``) — этот вернулся, пока она молчит."""
    req, llm, rig = real_chain
    result = req.execute(intent="track", text="трек про динозавров", heard_text="Робот, сыграй трек про динозавров")
    assert result.success and rig.owner.is_playing()
    assert not llm.finished.is_set()
    assert llm.asked.wait(10)  # вопрос ушёл в фоне, в своём потоке
    assert len(llm.calls) == 1  # связь запишется в хранилище, повтор найдёт её сразу
    llm.release.set()
    assert llm.finished.wait(5)


def test_zero_wait_asks_the_llm_in_the_background_and_does_not_block(tmp_path):
    """``wait_s=0`` (``knowledge.THEME_ASK_WAIT_TRACK_S``): заказ уходит дальше сразу, вопрос к LLM — в своём потоке."""
    pending, asked = [], []
    links = ThemeLinks(ask=lambda *a: asked.append(a) or ("fail", None, ""), spawn=pending.append)
    db = str(tmp_path / "links.db")
    library = SimpleNamespace(db_path=db, _db_path=db)
    hits = ThemeHits(missing=("динозавров",), named=True)
    assert links.expand(library, "динозавров", hits, wait_s=kn.THEME_ASK_WAIT_TRACK_S) is hits
    assert asked == [] and len(pending) == 1  # вопрос ещё не ушёл: поток не успел, а заказ не ждёт
    pending[0]()  # фон доработал
    assert len(asked) == 1


def test_single_track_does_not_wait_for_the_llm_while_the_set_still_does():
    """Одиночный трек не ждёт LLM-разбор темы вовсе (``THEME_ASK_WAIT_TRACK_S`` = 0: трек играет сразу, ответ допишется
    в хранилище фоном), у сета прежнее ожидание."""
    assert kn.THEME_ASK_WAIT_TRACK_S == 0
    assert kn.THEME_ASK_WAIT_S > 0
    assert ThemeLinks()._wait_s == kn.THEME_ASK_WAIT_S  # noqa: SLF001 — у сета прежнее ожидание
