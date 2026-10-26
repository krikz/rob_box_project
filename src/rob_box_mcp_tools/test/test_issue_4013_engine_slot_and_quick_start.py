"""#4013 и #3960 п.1/п.2 — путь музыки с движком Ренардо: слот ``engine`` решает код, старт трека не ждёт LLM.

(A) LLM ставила ``engine=renardo`` на любую реплику с инструментом («сыграй X на балалайке»), а код принимал слот без
проверки по словам человека: путь сужался до синтеза, кавер (KIE/Suno) не рассматривался. Теперь слот принимается,
только если в словах хода есть опора (``knowledge.MUSIC_ENGINES``: основы и ядро имени, искажения STT), иначе
отбрасывается с причиной в лог; играющий движок наследует код по снимку плеера («ускорь», «добавь баса»).

(B) ``request_music(engine=renardo, mood)`` ждал LLM-разбор темы (theme-links, ``submit_catalog_queries``) 4–10 с и
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
from rob_box_dialog import knowledge as dialog_kn
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
        result = req.execute(intent="track", text=heard, engine="renardo", heard_text=heard)
    assert asks[0].engine == "" and "engine" not in result.data
    assert any("слот engine=renardo без опоры в словах человека — отброшен" in r.getMessage() for r in caplog.records)


@pytest.mark.parametrize("heard", [
    "Робот, поставь ровный ритм на виндарда",  # искажение STT дальше основ — опора есть (ядро имени)
    "Робот, сыграй нарен дарда", "Робот, сыграй на индардо", RHYTHM,
])
@pytest.mark.parametrize("slot", ["renardo", None])
def test_engine_with_support_in_the_words_is_kept_with_or_without_the_slot(heard, slot):
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    result = req.execute(intent="track", text=heard, engine=slot, heard_text=heard)
    assert asks[0].engine == "renardo" and result.success and result.data["engine"] == "renardo"


def test_playing_engine_is_inherited_by_a_plain_follow_up_like_n506(caplog):
    """n504–n506: «ускорь», «добавь баса» под Ренардо — движок тот же, его наследует код по снимку плеера."""
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    first = req.execute(intent="track", text="сыграй на ренардо", heard_text="Робот, сыграй на ренардо")
    assert first.success and rig.owner.now_playing()["engine"] == "renardo"  # движок — в снимке плеера
    with caplog.at_level(logging.INFO):
        for heard in ("Робот, и ускорь темп, а то мы тут засыпаем оба.",
                      "Робот, продолжай развивать эту тему и добавь баса пожирнее."):
            result = req.execute(intent="track", text=heard, heard_text=heard)
            assert asks[-1].engine == "renardo" and result.success and result.data["engine"] == "renardo"
    assert any("движок наследуется от играющего: renardo" in r.getMessage() for r in caplog.records)


def test_playing_engine_is_not_inherited_by_a_request_with_its_own_subject():
    """Под играющим Ренардо «сыграй что-нибудь на балалайке» и «сыграй рок» — обычный путь, а не молчаливый синтез."""
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    assert req.execute(intent="track", text="на ренардо", heard_text="Робот, сыграй на ренардо").success
    for heard in (BALALAIKA, "Робот, сыграй рок", "Робот, сгенерируй новый трек"):
        req.execute(intent="track", text=heard, heard_text=heard, engine="renardo")
        assert asks[-1].engine == "", heard
    req.execute(intent="melody", text="Калинка", heard_text="Робот, поставь Калинку")  # пьеса по названию
    assert asks[-1].engine == ""


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
            "engine": "renardo"}
    assert _schema_error(schema, args, "request_music") is None
    rig = _rig()
    _dj, req = _tools(rig)
    assert "rock" in {p.name: p for p in req.parameters}["genre"].enum


# ── (B) старт трека не ждёт LLM-разбор темы ──────────────────────────────────────────────────────────────────────


def test_renardo_mood_request_does_not_ask_the_llm_about_the_theme():
    """n809: «ровный ритм на Ренардо, только не громко» + mood=calm — каталог и хранилище, LLM-разбора темы нет."""
    rig = _rig()
    dj, req = _tools(rig)
    finders = Finders(dj)
    result = req.execute(intent="track", text="поставь фоном ровный ритм на Ренардо только не громко",
                         mood="calm", engine="renardo", heard_text=RHYTHM)
    assert result.success and result.data["engine"] == "renardo"
    assert finders.calls == {"quick": ["фоном ровный ритм громко"], "track": [], "llm": []}


def test_track_name_is_not_the_text_of_the_request():
    """Тема без находок — не название трека: не «поставь фоном ровный ритм на Ренардо только не громко»."""
    rig = _rig()
    dj, req = _tools(rig)
    Finders(dj)
    text = "поставь фоном ровный ритм на Ренардо только не громко"
    data = req.execute(intent="track", text=text, mood="calm", engine="renardo", heard_text=RHYTHM).data
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


class SlowLLM:
    """LLM разбора темы (``reasoner.ask``): ждёт, пока тест её отпустит; ``calls`` — что у неё спросили."""

    def __init__(self):
        self.calls, self.release, self.finished = [], threading.Event(), threading.Event()

    def __call__(self, *args):
        self.calls.append(args)
        self.release.wait(10)
        self.finished.set()
        return "fail", None, ""


@pytest.fixture()
def real_chain(archive, tmp_path):
    """Настоящая цепочка поиска темы (архив, хранилище связей, ``ThemeLinks``) с медленной LLM-разборщицей."""
    db = str(tmp_path / "links.db")
    library = SimpleNamespace(**{k: getattr(archive, k) for k in ("search", "get", "vocabulary")}, db_path=db,
                              _db_path=db)
    llm = SlowLLM()
    rig = _rig()
    with patch.object(tools_v2, "_rtttl_library", lambda: library):
        dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, seed=lambda: 4242,
                       reasoner=SimpleNamespace(enabled=True, ask=llm))
        req = RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 777,
                               classic=lambda query, seed: ClassicPick(query=query, found=False, reason="нет в базе"),
                               records=lambda ids: {}, words=lambda word: ThemeHits())
    yield req, llm
    llm.release.set()


def test_n809_renardo_mood_request_starts_at_once_and_asks_no_llm_theme_links(real_chain):
    """Живой n809: между выбором синтеза и стартом стоял LLM ``submit_catalog_queries`` 9.7 с — теперь LLM не зовётся."""
    req, llm = real_chain
    started = time.monotonic()
    result = req.execute(intent="track", text="поставь фоном ровный ритм на Ренардо только не громко", mood="calm",
                         engine="renardo", heard_text=RHYTHM)
    assert result.success and result.data["engine"] == "renardo"
    assert llm.calls == []  # LLM-разбора темы нет совсем
    assert time.monotonic() - started < kn.THEME_ASK_WAIT_S  # раньше заказ отсиживал ожидание сета (8 с): 9.7 с в n809


def test_track_with_a_theme_asks_the_llm_only_in_the_background(real_chain):
    """Тема без находок и без настроения: LLM спрашивается, но заказ её не ждёт (раньше стоял до 8 с)."""
    req, llm = real_chain
    started = time.monotonic()
    result = req.execute(intent="track", text="трек про динозавров", heard_text="Робот, сыграй трек про динозавров")
    elapsed = time.monotonic() - started
    # LLM ещё держит ответ (release не отпущен), а заказ уже играет: ожидание сета (8 с) он не отсиживает
    assert result.success and elapsed < kn.THEME_ASK_WAIT_S
    assert not llm.finished.is_set()
    deadline = time.monotonic() + 5
    while not llm.calls and time.monotonic() < deadline:
        time.sleep(0.01)
    assert len(llm.calls) == 1  # вопрос ушёл в фоне: связь запишется в хранилище, повтор найдёт её сразу
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


def test_single_track_wait_fits_the_music_action_deadline():
    """Ожидание LLM у сета (8 с) и запуск трека не умещаются в потолок действия музыки в диалоге (10 с) — у одиночного
    трека своё ожидание, из знания музыки, и умещается."""
    assert kn.THEME_ASK_WAIT_S + dialog_kn.MUSIC_START_WAIT_S > dialog_kn.MUSIC_ACTION_DEADLINE_S
    assert kn.THEME_ASK_WAIT_TRACK_S + dialog_kn.MUSIC_START_WAIT_S <= dialog_kn.MUSIC_ACTION_DEADLINE_S
    assert ThemeLinks()._wait_s == kn.THEME_ASK_WAIT_S  # noqa: SLF001 — у сета прежнее ожидание
