"""#4216 - «эта песня» после генерации: тул берёт сочинённый только что трек, а не старую версию с тем же названием.

Марафон-5 29.10 (n707/n708): робот сочинил «Енота-механика» (``[song] track=db2045…``), через полторы минуты
``music_library info/play`` ушли с ``track_id`` вчерашней версии (``b90b61bf…``) - модель взяла id из списка по названию.
Здесь настоящая библиотека (SQLite на tmp), настоящий ``MusicLibraryTool`` и настоящие ``SongStudio`` /
``InstrumentalMaker`` на фейках провайдера: проверяется ПОВЕДЕНИЕ - какой трек получил ``info`` / ``play`` / ``rate``.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.engine.gen.instrumental import InstrumentalMaker
from rob_box_mcp_tools.engine.gen.song import SongRequest
from rob_box_mcp_tools.engine.gen.last_generated import GENERATED_FRESH_S
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary

from .test_adr0158_pr7_song import GOOD, _studio
from .test_adr0158_pr8_gen_set import Kie
from .test_tools.test_gen_library import MusicLibraryTool, _fake_node, _mm

pytestmark = pytest.mark.unit

REF = "last_generated"


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def library(tmp_path, clock):
    return GeneratedMusicLibrary(str(tmp_path / "music_library"), clock=clock)


def _save(library, title, task, variant=0):
    return library.save_track(b"MP3", prompt="rock", title=title, task_id=task, variant=variant, duration_ms=44000)["id"]


def _two_versions(library):
    """Вчерашняя и только что сочинённая версии с одним названием; сочинённая отмечена как сыгранная генерацией."""
    old = _save(library, "Енот-механик", "OLD")
    new = _save(library, "Енот-механик", "NEW")
    library.generated.note(new)
    return old, new


# ---------------------------------------------------------------------------------------------------------------
# библиотека: единственное место знания «какая песня - эта»
# ---------------------------------------------------------------------------------------------------------------

def test_last_generated_is_the_noted_track_while_fresh(library, clock):
    _, new = _two_versions(library)
    clock.now += GENERATED_FRESH_S
    assert library.generated.fresh() == new


def test_last_generated_expires(library, clock):
    _two_versions(library)
    clock.now += GENERATED_FRESH_S + 1
    assert library.generated.fresh() is None


def test_nothing_generated_or_track_deleted_means_no_reference(library):
    assert library.generated.fresh() is None
    _, new = _two_versions(library)
    library.delete_track(new)
    assert library.generated.fresh() is None


def test_the_newest_note_wins(library):
    _, first = _two_versions(library)
    second = _save(library, "Другая", "T3")
    library.generated.note(second)
    assert library.generated.fresh() == second != first


# ---------------------------------------------------------------------------------------------------------------
# тул: ссылка track_ref заменяет id модели
# ---------------------------------------------------------------------------------------------------------------

def _tool(library):
    return MusicLibraryTool(_fake_node(), library)


class _Played:
    success, data, error, message = True, {}, None, ""


def test_n707_info_after_song_answers_about_the_new_version_not_the_one_the_model_picked(library):
    old, new = _two_versions(library)
    tool = _tool(library)
    tool.execute(action="list")  # модель видела список: обе версии «названы»
    result = tool.execute(action="info", track_id=old, track_ref=REF)
    assert result.success and result.data["id"] == new != old


def test_info_by_reference_needs_no_list_first(library):
    _, new = _two_versions(library)
    result = _tool(library).execute(action="info", track_ref=REF)
    assert result.success and result.data["id"] == new


def test_n708_play_after_song_plays_the_new_version(library, monkeypatch):
    played = []
    old, new = _two_versions(library)
    monkeypatch.setattr(_mm, "play_library_track", lambda node, lib, track_id, **kw: played.append(track_id) or _Played())
    _tool(library).execute(action="play", track_id=old, track_ref=REF)
    assert played == [new]


def test_rate_by_reference_rates_the_new_version(library):
    old, new = _two_versions(library)
    _tool(library).execute(action="rate", verdict="like", track_ref=REF)
    assert library.get_track_info(new)["rating"] > 0 and library.get_track_info(old)["rating"] == 0


def test_without_fresh_generated_the_models_id_works_as_before(library, clock, monkeypatch):
    played = []
    old, _ = _two_versions(library)
    clock.now += GENERATED_FRESH_S + 1
    monkeypatch.setattr(_mm, "play_library_track", lambda node, lib, track_id, **kw: played.append(track_id) or _Played())
    tool = _tool(library)
    tool.execute(action="list")
    assert tool.execute(action="info", track_id=old, track_ref=REF).data["id"] == old
    tool.execute(action="play", track_id=old, track_ref=REF)
    assert played == [old]


def test_without_fresh_generated_and_without_id_the_old_error_stays(library):
    result = _tool(library).execute(action="info", track_ref=REF)
    assert not result.success and "track_id" in result.error


def test_without_reference_another_title_is_left_alone_even_with_a_fresh_track(library):
    _two_versions(library)
    other = _save(library, "Дождь", "T9")
    tool = _tool(library)
    tool.execute(action="list")
    assert tool.execute(action="info", track_id=other).data["id"] == other
    refused = tool.execute(action="info", track_id="f" * 32)  # выдуманный id по-прежнему отказ #4025
    assert refused.data["reason"] == "unknown_track"


def test_schema_declares_the_reference_as_a_closed_enum():
    param = next(p for p in MusicLibraryTool(_fake_node(), object()).parameters if p.name == "track_ref")
    assert param.enum == ["", REF] and not param.required


# ---------------------------------------------------------------------------------------------------------------
# генерация сама отмечает сыгранный вариант
# ---------------------------------------------------------------------------------------------------------------

def test_song_marks_the_variant_it_plays(library):
    studio, *_ = _studio(library, lyrics=lambda *a: GOOD)
    result = studio.make(SongRequest("про енота", "rock"))
    assert library.count == 2 and library.generated.fresh() == result.track_id


def test_instrumental_marks_the_variant_it_returns(library):
    maker = InstrumentalMaker(Kie(seconds=30.0), lambda: library, balance=lambda: 900.0)
    result = maker.make("rock", "V5", "")
    assert library.count == 2 and library.generated.fresh() == result.track_id


# ---------------------------------------------------------------------------------------------------------------
# без ссылки: версия с тем же названием из списка заменяется свежесочинённой (n707 «расскажи про песню про енота»)
# ---------------------------------------------------------------------------------------------------------------

def test_n707_the_model_picks_yesterdays_twin_from_the_list_and_gets_the_new_song(library):
    old, new = _two_versions(library)
    tool = _tool(library)
    tool.execute(action="list")
    result = tool.execute(action="info", track_id=old)
    assert result.success and result.data["id"] == new


def test_n708_play_of_the_twin_plays_the_new_song(library, monkeypatch):
    played = []
    old, new = _two_versions(library)
    monkeypatch.setattr(_mm, "play_library_track", lambda node, lib, track_id, **kw: played.append(track_id) or _Played())
    tool = _tool(library)
    tool.execute(action="list")
    tool.execute(action="play", track_id=old)
    assert played == [new]


def test_server_logs_the_id_the_tool_will_run_with(library):
    """``bind_arguments`` - то, что сервер пишет в «Запрос выполнения» до исполнения: харнесс сверяет id по этой строке."""
    old, new = _two_versions(library)
    tool = _tool(library)
    assert tool.bind_arguments({"action": "info", "track_id": old}) == {"action": "info", "track_id": new}
    assert tool.bind_arguments({"action": "play", "track_id": old, "track_ref": REF})["track_id"] == new
    same = {"action": "info", "track_id": new}
    assert tool.bind_arguments(same) is same


def test_other_titles_and_other_actions_keep_the_models_id(library):
    old, new = _two_versions(library)
    other = _save(library, "Дождь", "T9")
    tool = _tool(library)
    for action in ("info", "play"):
        assert tool.bind_arguments({"action": action, "track_id": other})["track_id"] == other
    for action in ("update", "delete", "rate"):  # правка и стирание старой версии - выбор человека
        assert tool.bind_arguments({"action": action, "track_id": old})["track_id"] == old
    assert tool.bind_arguments({"action": "list"}) == {"action": "list"}


def test_twin_rule_is_off_when_nothing_is_fresh(library, clock):
    old, _ = _two_versions(library)
    clock.now += GENERATED_FRESH_S + 1
    args = {"action": "info", "track_id": old}
    assert _tool(library).bind_arguments(args) is args


def test_invented_id_is_still_refused_and_not_rewritten(library):
    _two_versions(library)
    tool = _tool(library)
    args = {"action": "info", "track_id": "f" * 32}
    assert tool.bind_arguments(args) is args
    assert tool.execute(**args).data["reason"] == "unknown_track"
