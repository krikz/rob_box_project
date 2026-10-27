"""
test_gen_library.py — тесты библиотеки сгенерированных треков и gen_* тулов.

Issue #1392 / ADR-0158 PR-1: покрыто
  * engine/gen/library.py — CRUD, поиск, play_count (прежний контракт)
  * tools/gen_library.py — один тул music_library (ADR-0158 PR-9)
  * #4025: info / update / delete — только по id из ответа list / search, delete — через вопрос человеку

Клиент MiniMax Music и тул ``generate_music`` удалены (API 410 Gone с
20.08.2026); новый клиент KIE — ``test_gen_kie_suno.py``, новая схема и
миграция — ``test_gen_library_cache.py``. ROS 2 не нужен.
"""


import importlib.util
import json
import sys
import wave
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock

import pytest
from rob_box_music import knowledge as kn

from .._ros_stubs import RosStubs

# ── ROS-free mocking (matches existing test_music.py pattern) ───────────────
_ros = RosStubs()


# ── Helpers ──────────────────────────────────────────────────────────────────


class _FakeLogger:
    def info(self, m): pass
    def warning(self, m): pass
    def error(self, m): pass
    def debug(self, m): pass


def _fake_node():
    node = SimpleNamespace(get_logger=lambda: _FakeLogger())
    node.create_publisher = MagicMock(return_value=MagicMock())
    return node


# ── Imports under test (load gen_library directly, bypassing
#    tools/__init__.py which would pull in ros-bound .music + .navigation) ──

def _load_gen_library_module():
    """Load rob_box_mcp_tools.tools.gen_library as a sub-package so its
    ``from ..base import …`` relative import resolves correctly.

    Avoids importing the full ``tools/__init__.py`` which transitively
    requires ``rob_box_voice`` (unavailable in unit-test environment).
    """
    pkg_name = "_rob_box_mcp_tools_test_isolation"
    tools_pkg = ModuleType(pkg_name + ".tools")
    tools_pkg.__path__ = []  # mark as package
    sys.modules[pkg_name + ".tools"] = tools_pkg

    # Stand-in for rob_box_mcp_tools (parent package)
    parent_pkg = ModuleType(pkg_name)
    parent_pkg.tools = tools_pkg
    parent_pkg.__path__ = [str(Path(__file__).resolve().parents[2] / "rob_box_mcp_tools")]
    sys.modules[pkg_name] = parent_pkg

    # Load gen_library as if it were rob_box_mcp_tools.tools.gen_library
    file_path = (Path(__file__).resolve().parents[2]
                 / "rob_box_mcp_tools" / "tools" / "gen_library.py")
    spec = importlib.util.spec_from_file_location(
        pkg_name + ".tools.gen_library", str(file_path),
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[pkg_name + ".tools.gen_library"] = module
    spec.loader.exec_module(module)
    return module


with _ros:
    from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary

    _mm = _load_gen_library_module()
_ros_stubs = _ros.fixture()
MusicLibraryTool = _mm.MusicLibraryTool


# ════════════════════════════════════════════════════════════════════════════
# engine/gen/library.py
# ════════════════════════════════════════════════════════════════════════════


@pytest.fixture
def tmp_lib(tmp_path):
    """Fresh GeneratedMusicLibrary per test."""
    lib = GeneratedMusicLibrary(str(tmp_path / "music_library"))
    yield lib
    # cleanup happens automatically with tmp_path


class TestGeneratedMusicLibrary:
    def test_empty_library_count(self, tmp_lib):
        assert tmp_lib.count == 0
        assert tmp_lib.list_tracks()["total"] == 0

    def test_save_track_creates_directory_and_files(self, tmp_lib):
        result = tmp_lib.save_track(
            audio_bytes=b"ID3...fake mp3 bytes...",
            prompt="Indie folk, melancholic, 92 bpm",
            lyrics="[Verse]\nStreetlights flicker...",
            model="music-3.0-free",
            duration_ms=45000,
            tags=["rainy", "love"],
            mood="melancholic",
            genre="indie folk",
            title="Rainy Evening",
        )
        assert result["success"]
        track_id = result["id"]
        assert len(track_id) == 32  # uuid4 hex
        assert Path(result["path"]).exists()
        assert Path(result["meta_json"]).exists()

        meta_on_disk = json.loads(Path(result["meta_json"]).read_text(encoding="utf-8"))
        assert meta_on_disk["title"] == "Rainy Evening"
        assert meta_on_disk["duration_ms"] == 45000
        assert meta_on_disk["tags"] == ["rainy", "love"]

        assert tmp_lib.count == 1

    def test_save_empty_audio_fails(self, tmp_lib):
        result = tmp_lib.save_track(audio_bytes=b"", prompt="x")
        assert not result["success"]
        assert "empty" in result["error"]

    def test_save_empty_prompt_fails(self, tmp_lib):
        result = tmp_lib.save_track(audio_bytes=b"x", prompt="")
        assert not result["success"]

    def test_list_filters_and_sorts(self, tmp_lib):
        for i in range(3):
            tmp_lib.save_track(
                audio_bytes=b"x",
                prompt=f"track {i}",
                tags=["common"],
                mood="happy" if i % 2 else "sad",
                rating=i,
            )
        all_tracks = tmp_lib.list_tracks()
        assert all_tracks["total"] == 3

        # sort_by=rating
        by_rating = tmp_lib.list_tracks(sort_by="rating")
        assert by_rating["tracks"][0]["rating"] == 2  # highest first

        # mood filter
        happy_only = tmp_lib.list_tracks(mood="happy")
        assert happy_only["total"] == 1

        # tag filter
        common = tmp_lib.list_tracks(tag="common")
        assert common["total"] == 3

    def test_search_by_keyword(self, tmp_lib):
        tmp_lib.save_track(audio_bytes=b"x", prompt="Indie folk about rain", lyrics="drip drop")
        tmp_lib.save_track(audio_bytes=b"x", prompt="Synthwave about love", lyrics="neon glow")
        tmp_lib.save_track(audio_bytes=b"x", prompt="Lo-fi beats", lyrics="chill vibes")

        r = tmp_lib.search_tracks("rain")
        assert r["total"] == 1
        assert "rain" in r["tracks"][0]["prompt"].lower()

        # Search across lyrics
        r2 = tmp_lib.search_tracks("neon")
        assert r2["total"] == 1

    def test_search_empty_query_rejected(self, tmp_lib):
        r = tmp_lib.search_tracks("")
        assert not r["success"]
        assert r["total"] == 0

    def test_get_track_info(self, tmp_lib):
        saved = tmp_lib.save_track(
            audio_bytes=b"abc", prompt="x", title="My Track", rating=4,
        )
        info = tmp_lib.get_track_info(saved["id"])
        assert info["success"]
        assert info["title"] == "My Track"
        assert info["exists_on_disk"] is True
        assert info["path"].endswith("track.mp3")

    def test_get_nonexistent_track(self, tmp_lib):
        info = tmp_lib.get_track_info("deadbeef" * 4)
        assert not info["success"]
        assert "не найден" in info["error"]

    def test_delete_track_removes_files(self, tmp_lib):
        saved = tmp_lib.save_track(audio_bytes=b"abc", prompt="x")
        track_id = saved["id"]
        assert tmp_lib.count == 1

        result = tmp_lib.delete_track(track_id)
        assert result["success"]
        assert tmp_lib.count == 0
        assert not (tmp_lib.root_dir / track_id).exists()

    def test_delete_nonexistent(self, tmp_lib):
        result = tmp_lib.delete_track("deadbeef" * 4)
        assert not result["success"]

    def test_play_count_increments(self, tmp_lib):
        saved = tmp_lib.save_track(audio_bytes=b"x", prompt="x")
        track_id = saved["id"]

        r1 = tmp_lib.increment_play_count(track_id)
        assert r1["success"]
        assert r1["play_count"] == 1

        r2 = tmp_lib.increment_play_count(track_id)
        assert r2["play_count"] == 2


# ════════════════════════════════════════════════════════════════════════════
# tools/gen_library.py — один тул music_library (ADR-0158 PR-9)
# ════════════════════════════════════════════════════════════════════════════

NO_ID = "0000000000000000" * 2


def _tool(lib, node=None, **kwargs):
    return MusicLibraryTool(node or _fake_node(), lib, **kwargs)


class TestToolShape:
    @pytest.fixture
    def tool(self, tmp_path):
        return _tool(GeneratedMusicLibrary(str(tmp_path / "lib")))

    def test_one_narrow_tool_with_closed_action_enum(self, tool):
        action = next(p for p in tool.parameters if p.name == "action")
        assert tool.name == "music_library" and action.required
        assert action.enum == ["list", "search", "info", "play", "rate", "update", "delete"]
        verdict = next(p for p in tool.parameters if p.name == "verdict")
        assert verdict.enum == ["", "like", "dislike"]
        assert all(p.description for p in tool.parameters)

    def test_unknown_action_is_refused(self, tool):
        assert not tool.execute(action="rm_rf").success

    @pytest.mark.parametrize("action", ["info", "play", "update", "delete"])
    def test_track_actions_need_an_id(self, tool, action):
        result = tool.execute(action=action)
        assert not result.success and "track_id" in result.error


class TestListSearchInfo:
    @pytest.fixture
    def library(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        lib.save_track(audio_bytes=b"x", prompt="happy indie", title="Утро", mood="happy", rating=5)
        lib.save_track(audio_bytes=b"y", prompt="sad ballad", title="Дождь", mood="sad", rating=3)
        lib.save_track(audio_bytes=b"z", prompt="energetic dance", title="Танец", mood="energetic", rating=4)
        return lib

    def test_list_phrase_is_built_by_code(self, library):
        result = _tool(library).execute(action="list")
        assert result.success and result.data["total"] == 3
        assert result.message.startswith("В библиотеке 3 трека, например: ")
        assert "«Дождь»" in result.message and "lyrics" not in result.data["tracks"][0]

    def test_list_total_is_the_whole_library_not_the_limit(self, library):
        result = _tool(library).execute(action="list", limit=2)
        assert result.data["total"] == 3 and result.data["shown"] == 2
        assert result.message.startswith("В библиотеке 3 трека")

    def test_empty_library_says_so(self, tmp_path):
        result = _tool(GeneratedMusicLibrary(str(tmp_path / "empty"))).execute(action="list")
        assert result.success and result.message == "В библиотеке сгенерированной музыки пока пусто."

    def test_list_limit_and_sort_by_rating(self, library):
        result = _tool(library).execute(action="list", limit=2, sort_by="rating")
        assert [t["rating"] for t in result.data["tracks"]] == [5, 4]

    def test_search_finds_and_misses(self, library):
        found = _tool(library).execute(action="search", query="ballad")
        assert found.success and found.data["total"] == 1 and "найдено 1" in found.message
        missed = _tool(library).execute(action="search", query="polka")
        assert missed.success and missed.data["total"] == 0 and "ничего не найдено" in missed.message
        assert not _tool(library).execute(action="search", query="").success

    def test_info_returns_full_metadata(self, library):
        tool = _tool(library)
        track = tool.execute(action="list", sort_by="rating").data["tracks"][0]
        info = tool.execute(action="info", track_id=track["id"])
        assert info.success and info.data["title"] == "Утро" and info.data["exists_on_disk"] is True
        assert not tool.execute(action="info", track_id=NO_ID).success


class _Clock:
    """Часы теста: время идёт, только когда его двигают."""

    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now


class TestUpdateDelete:
    @pytest.fixture
    def setup(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        saved = lib.save_track(audio_bytes=b"x", prompt="x", title="Original", rating=2)
        clock = _Clock()
        tool = _tool(lib, clock=clock)
        assert tool.execute(action="list").success  # id назван в ответе тула (#4025)
        return SimpleNamespace(library=lib, track_id=saved["id"], tool=tool, clock=clock)

    def test_update_changes_named_fields_only_and_meta_json(self, setup):
        result = setup.tool.execute(action="update", track_id=setup.track_id, tags="любимый, свадьба", mood="romantic")
        assert result.success, result.error
        info = setup.library.get_track_info(setup.track_id)
        assert info["tags"] == ["любимый", "свадьба"] and info["mood"] == "romantic"
        assert info["title"] == "Original" and info["rating"] == 2
        disk = json.loads((setup.library.root_dir / setup.track_id / "meta.json").read_text(encoding="utf-8"))
        assert disk["mood"] == "romantic"

    def test_update_without_fields_or_track_fails(self, setup):
        assert not setup.tool.execute(action="update", track_id=setup.track_id).success
        assert not setup.tool.execute(action="update", track_id=NO_ID, title="x").success

    def test_delete_removes_track_and_says_so(self, setup):
        assert not setup.tool.execute(action="delete", track_id=setup.track_id).success  # сначала вопрос
        setup.clock.now += 5.0  # человек ответил «да»
        result = setup.tool.execute(action="delete", track_id=setup.track_id, confirm=True)
        assert result.success and result.message == "Удалил «Original» из библиотеки."
        assert result.data == {"track_id": setup.track_id, "title": "Original"}
        assert setup.library.count == 0
        assert not setup.tool.execute(action="delete", track_id=setup.track_id, confirm=True).success


class TestNamedTracks:
    """#4025 (n707/n710 23.10): info / update / delete принимают только id, который тул сам назвал в list / search."""

    @pytest.fixture
    def setup(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        mine = lib.save_track(audio_bytes=b"x", prompt="x", title="Мой")["id"]
        stranger = lib.save_track(audio_bytes=b"y", prompt="y", title="Чужой")["id"]  # есть в библиотеке, не называли
        clock = _Clock()
        return SimpleNamespace(library=lib, mine=mine, stranger=stranger, clock=clock, tool=_tool(lib, clock=clock))

    @pytest.mark.parametrize("action,extra", [("info", {}), ("update", {"title": "Взломан"}),
                                              ("delete", {"confirm": True})])
    def test_existing_but_never_named_id_is_refused(self, setup, action, extra):
        """Существующий трек, id которого в разговоре не звучал: отказ ``unknown_track``, трек не тронут."""
        result = setup.tool.execute(action=action, track_id=setup.stranger, **extra)
        assert not result.success
        assert result.data["status"] == "rejected" and result.data["reason"] == "unknown_track"
        assert result.data["track_id"] == setup.stranger
        assert setup.library.count == 2
        assert setup.library.get_track_info(setup.stranger)["title"] == "Чужой"

    def test_id_made_up_out_of_thin_air_is_refused_the_same_way(self, setup):
        result = setup.tool.execute(action="delete", track_id=NO_ID)
        assert not result.success and result.data["reason"] == "unknown_track"

    @pytest.mark.parametrize("junk", [["a", "b"], {"id": "x"}, 12345, ["a"]])
    @pytest.mark.parametrize("action", ["info", "update", "delete"])
    def test_non_string_id_is_a_refusal_not_a_crash(self, setup, action, junk):
        """Список или словарь от небрежной модели вместо строки: ``unknown_track``, а не ``TypeError`` в ``execute``."""
        result = setup.tool.execute(action=action, track_id=junk, confirm=True)
        assert not result.success and result.data["reason"] == "unknown_track"
        assert setup.library.count == 2

    def test_non_string_id_in_the_window_check_itself(self):
        window = _mm.NamedTracks(limit=2)
        window.name(["a"])
        assert ["a"] not in window and {"a": 1} not in window and "a" in window

    def test_info_does_not_name_an_id(self, setup):
        """n707 -> n710: ``info`` по чужому id отказывает сам, а значит и не открывает дорогу ``delete``."""
        assert setup.tool.execute(action="info", track_id=setup.stranger).data["reason"] == "unknown_track"
        again = setup.tool.execute(action="delete", track_id=setup.stranger, confirm=True)
        assert not again.success and again.data["reason"] == "unknown_track" and setup.library.count == 2

    def test_list_names_the_ids_it_returned(self, setup):
        named = setup.tool.execute(action="list", limit=1).data["tracks"][0]["id"]
        assert setup.tool.execute(action="info", track_id=named).success
        other = ({setup.mine, setup.stranger} - {named}).pop()
        assert setup.tool.execute(action="info", track_id=other).data["reason"] == "unknown_track"

    def test_search_names_the_ids_it_found(self, setup):
        refused = setup.tool.execute(action="update", track_id=setup.mine, title="Новый")
        assert refused.data["reason"] == "unknown_track"
        found = setup.tool.execute(action="search", query="x")
        assert found.data["total"] == 1 and found.data["tracks"][0]["id"] == setup.mine
        assert setup.tool.execute(action="update", track_id=setup.mine, title="Новый").success
        other = setup.tool.execute(action="update", track_id=setup.stranger, title="Новый")
        assert other.data["reason"] == "unknown_track"

    def test_play_and_rate_keep_their_contract(self, setup):
        """Не названный id для ``rate`` допустим по-прежнему: правило касается необратимого и правки."""
        assert setup.tool.execute(action="rate", track_id=setup.stranger, verdict="like").success

    def test_window_forgets_the_oldest(self):
        window = _mm.NamedTracks(limit=2)
        window.name(["a", "b"])
        assert "a" in window  # обращение освежает
        window.name(["c"])
        assert "a" in window and "c" in window and "b" not in window
        window.name(["", None])  # пустое имя - не id
        assert "" not in window


class TestDeleteNeedsAnAnswer:
    """#4025: удаление необратимо (кредиты KIE 0, восстановить нечем) - тул сам спрашивает, ``confirm`` без вопроса не в счёт."""

    @pytest.fixture
    def setup(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        track_id = lib.save_track(audio_bytes=b"x", prompt="x", title="Дождь")["id"]
        other_id = lib.save_track(audio_bytes=b"y", prompt="y", title="Ветер")["id"]
        clock = _Clock()
        tool = _tool(lib, clock=clock)
        tool.execute(action="list")
        return SimpleNamespace(library=lib, track_id=track_id, other_id=other_id, clock=clock, tool=tool)

    def delete(self, setup, track_id=None, **kwargs):
        return setup.tool.execute(action="delete", track_id=track_id or setup.track_id, **kwargs)

    def test_without_confirm_it_asks_by_title_and_deletes_nothing(self, setup):
        result = self.delete(setup)
        assert not result.success
        assert result.data["status"] == "rejected" and result.data["reason"] == "needs_confirmation"
        assert result.data["title"] == "Дождь" and result.data["track_id"] == setup.track_id
        assert result.data["question"] == result.message == "Удалить «Дождь»? Скажи «да, удаляй»."
        assert setup.library.count == 2

    def test_confirm_after_the_question_deletes(self, setup):
        self.delete(setup)
        setup.clock.now += 6.0  # робот договорил вопрос, человек ответил, речь распозналась
        result = self.delete(setup, confirm=True)
        assert result.success and result.message == "Удалил «Дождь» из библиотеки."
        assert setup.library.count == 1
        assert setup.library.get_track_info(setup.track_id)["success"] is False

    def test_confirm_without_a_question_is_a_question(self, setup):
        """Модель сразу прислала ``confirm=true`` - вопроса не было, поэтому спрашиваем, а не удаляем."""
        result = self.delete(setup, confirm=True)
        assert not result.success and result.data["reason"] == "needs_confirmation"
        assert setup.library.count == 2

    def test_confirm_in_the_same_breath_is_not_an_answer(self, setup):
        """Цепочка «удали, удали с confirm» одной командой: человек ещё не мог ответить."""
        self.delete(setup)
        setup.clock.now += 0.3
        early = self.delete(setup, confirm=True)
        assert not early.success and early.data["reason"] == "needs_confirmation" and setup.library.count == 2
        setup.clock.now += 3.0  # ранний confirm срок вопроса не сбросил: от вопроса прошло 3.3 с
        assert self.delete(setup, confirm=True).success and setup.library.count == 1

    def test_stale_question_is_asked_again(self, setup):
        self.delete(setup)
        setup.clock.now += 121.0
        late = self.delete(setup, confirm=True)
        assert not late.success and late.data["reason"] == "needs_confirmation" and setup.library.count == 2
        setup.clock.now += 5.0
        assert self.delete(setup, confirm=True).success

    def test_confirm_for_another_track_asks_about_that_one(self, setup):
        self.delete(setup)
        setup.clock.now += 6.0
        other = self.delete(setup, track_id=setup.other_id, confirm=True)
        assert not other.success and other.data["title"] == "Ветер" and setup.library.count == 2
        setup.clock.now += 6.0
        assert self.delete(setup, track_id=setup.other_id, confirm=True).success
        assert setup.library.get_track_info(setup.track_id)["success"] is True  # первый трек цел

    def test_the_answer_is_spent_once(self, setup):
        self.delete(setup)
        setup.clock.now += 6.0
        assert self.delete(setup, confirm=True).success
        setup.clock.now += 6.0
        again = self.delete(setup, track_id=setup.other_id, confirm=True)  # прежний ответ другого трека не купит
        assert not again.success and again.data["reason"] == "needs_confirmation" and setup.library.count == 1

    @pytest.mark.parametrize("confirm", [False, "", 0, "нет", None, "yes", 1])
    def test_only_true_counts(self, setup, confirm):
        self.delete(setup)
        setup.clock.now += 6.0
        result = self.delete(setup, confirm=confirm)
        assert not result.success and result.data["reason"] == "needs_confirmation" and setup.library.count == 2

    def test_string_true_from_a_sloppy_model_counts(self, setup):
        self.delete(setup)
        setup.clock.now += 6.0
        assert self.delete(setup, confirm="true").success

    def test_untitled_track_is_still_asked_about(self, setup):
        untitled = setup.library.save_track(audio_bytes=b"z", prompt="z")["id"]
        setup.tool.execute(action="list")
        result = self.delete(setup, track_id=untitled)
        assert result.data["reason"] == "needs_confirmation" and result.message.startswith("Удалить")

    def test_schema_offers_a_boolean_confirm(self, setup):
        confirm = next(p for p in setup.tool.parameters if p.name == "confirm")
        assert confirm.type == "boolean" and not confirm.required and confirm.default is False
        assert "needs_confirmation" in setup.tool.description and "unknown_track" in setup.tool.description


class TestRate:
    """«Робот, этот трек нравится / не нравится»: рейтинг пишет код, фразу строит код (G6)."""

    @pytest.fixture
    def setup(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        first = lib.save_track(audio_bytes=b"a", prompt="grunge", title="Пыль")["id"]
        second = lib.save_track(audio_bytes=b"b", prompt="grunge", title="Гул")["id"]
        node = _fake_node()
        node._player_owner = SimpleNamespace(now_playing=lambda: {"playing": False})
        return SimpleNamespace(library=lib, first=first, second=second, node=node, tool=_tool(lib, node))

    def test_dislike_named_track_writes_minus_one(self, setup):
        result = setup.tool.execute(action="rate", track_id=setup.first, verdict="dislike")
        assert result.success and result.data["rating"] == kn.RATING_DISLIKE
        assert result.message == "Запомнил: «Пыль» не нравится, больше не включу."
        assert setup.library.get_track_info(setup.first)["rating"] == -1
        assert setup.library.get_track_info(setup.second)["rating"] == 0
        assert setup.library.count == 2  # не удаляется

    def test_like_writes_five(self, setup):
        result = setup.tool.execute(action="rate", track_id=setup.second, verdict="like")
        assert result.message == "Запомнил: «Гул» нравится."
        assert setup.library.get_track_info(setup.second)["rating"] == kn.RATING_LIKE

    def test_without_id_rates_the_track_playing_now(self, setup):
        setup.node._player_owner = SimpleNamespace(now_playing=lambda: {"playing": True, "library_id": setup.second})
        setup.library.increment_play_count(setup.first)  # игравший раньше — не он
        result = setup.tool.execute(action="rate", verdict="dislike")
        assert result.success and result.data["library_id"] == setup.second

    def test_without_id_after_the_track_ended_rates_the_last_played(self, setup):
        setup.library.increment_play_count(setup.first)
        result = setup.tool.execute(action="rate", verdict="like")
        assert result.success and result.data["library_id"] == setup.first

    def test_nothing_played_is_an_honest_refusal(self, setup):
        result = setup.tool.execute(action="rate", verdict="dislike")
        assert not result.success and result.error.startswith("Не знаю, о каком треке")

    def test_verdict_is_required(self, setup):
        assert not setup.tool.execute(action="rate", track_id=setup.first).success

    def test_disliked_track_is_not_a_candidate_of_the_path(self, setup):
        """G6: после «не нравится» следующий заказ жанра не берёт трек из кэша, соседний — берёт."""
        from rob_box_mcp_tools.engine.source_route import cache_tracks
        from rob_box_music.source import SourceAsk, SourceContext, source_candidates

        ask = SourceAsk(genre="grunge")

        def cached(lib):
            ctx = SourceContext(cache=cache_tracks(lib), day_left=0.0)
            return {o.id for o in source_candidates(ask, ctx)[0] if o.id.startswith("cache:")}

        for track_id in (setup.first, setup.second):
            setup.library.meta.update(track_id, genre="grunge")
        before = cached(setup.library)
        assert f"cache:{setup.first}" in before and f"cache:{setup.second}" in before
        setup.tool.execute(action="rate", track_id=setup.first, verdict="dislike")
        after = cached(setup.library)
        assert f"cache:{setup.first}" not in after and f"cache:{setup.second}" in after


class TestPlay:
    """ADR-0158 PR-2: трек кэша играет на деке ``PlayerOwner`` (wav сервера, ``render_audio``), ``ok`` — по started."""

    @pytest.fixture
    def setup(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        saved = lib.save_track(audio_bytes=b"ID3fake", prompt="space", title="Космос", duration_ms=3000)
        plays, closed, decodes = [], [], []

        def play(program, dj=None, once=False):
            plays.append((program, dj, once))
            return {"ok": True, "track_id": program.track_id}

        def run(cmd, **_):  # ffmpeg: пишет wav 16 кГц стерео на 2.5 с в последний аргумент
            decodes.append(cmd)
            with wave.open(cmd[-1], "wb") as out:
                out.setnchannels(2)
                out.setsampwidth(2)
                out.setframerate(16000)
                out.writeframes(b"\x00\x00" * 2 * 40000)

        events = {}
        node = _fake_node()
        node._player_owner = SimpleNamespace(play=play)
        node._dj_set_tool = SimpleNamespace(close_set=closed.append)
        node._music_confirm = lambda track_id: events.get("event", SimpleNamespace(event="started", fields={}))
        tool = _tool(lib, node, clock=lambda: 1000.0, run=run)
        return SimpleNamespace(library=lib, track_id=saved["id"], tool=tool, plays=plays, closed=closed,
                               decodes=decodes, events=events, node=node)

    def play(self, setup, track_id=None):
        return setup.tool.execute(action="play", track_id=track_id or setup.track_id)

    def test_plays_on_the_deck_once_after_closing_the_set(self, setup):
        before = setup.library.get_track_info(setup.track_id)["play_count"]
        result = self.play(setup)
        assert result.success, result.error
        program, dj, once = setup.plays[0]
        assert once is True and setup.closed == ["music_library"]
        assert program.track_id == f"gen_{setup.track_id[:8]}_1000" and program.bpm == 60
        assert program.form_beats == 4.0 and "sus=2.5," in program.code
        wav = str(setup.library.root_dir / setup.track_id / "track.16k.wav")
        assert program.sample_files == frozenset({wav}) and f"loop({wav!r}" in program.code
        assert dj == {"enabled": False, "title": "Космос", "source": "generated", "library_id": setup.track_id,
                      "bpm": None}
        assert result.message == "Включаю «Космос» из библиотеки — 2 секунды."
        info = setup.library.get_track_info(setup.track_id)
        assert info["play_count"] == before + 1 and info["last_played_at"]

    def test_internal_callers_use_the_function_not_the_tool(self, setup):
        """Кавер / песня / сет / путь зовут ``play_library_track`` напрямую — без тула в реестре."""
        from rob_box_mcp_tools.engine.gen.library_play import play_library_track

        result = play_library_track(setup.node, setup.library, setup.track_id, clock=lambda: 7.0, run=setup.tool._run)
        assert result.success and setup.plays[0][0].track_id.endswith("_7")

    def test_decodes_once(self, setup):
        self.play(setup)
        self.play(setup)
        assert len(setup.decodes) == 1
        cmd = setup.decodes[0]
        assert cmd[cmd.index("-ar") + 1] == "16000" and cmd[cmd.index("-ac") + 1] == "2"

    def test_not_started_is_not_ok(self, setup):
        setup.events["event"] = SimpleNamespace(event="rejected", fields={"reason": "missing_sample", "detail": "x"})
        result = self.play(setup)
        assert not result.success and "missing_sample" in result.error
        assert setup.library.get_track_info(setup.track_id)["play_count"] == 0

    def test_missing_track_fails(self, setup):
        assert not self.play(setup, NO_ID).success and setup.plays == []

    def test_missing_file_on_disk_fails(self, setup):
        (setup.library.root_dir / setup.track_id / "track.mp3").unlink()
        result = self.play(setup)
        assert not result.success and "no_file" in result.error and setup.plays == []

    def test_no_engine_no_sound_claimed(self, setup):
        setup.node._player_owner = None
        result = self.play(setup)
        assert not result.success and "движок" in result.error
