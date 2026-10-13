"""
test_music.py - Unit тесты музыкальных тулов вокруг движка v2

Тестирует:
- StopMusicTool, GetMusicStateTool — от владельца плеера v2 (#3614, #3613)
- LookupMelodyTool

Старый путь (``MusicManager``, ``execute_music_code``) удалён в #3614; подъём Renardo и
OSC — ``test_engine_renardo_runtime.py``.
"""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from .._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.tools.music import (
        StopMusicTool,
        GetMusicStateTool,
        LookupMelodyTool,
    )
    from rob_box_mcp_tools.engine.player_owner import PlayerOwner
_ros_stubs = _ros.fixture()


class _Adapter:
    """Дека без Renardo: ``start`` сразу зовёт ``started`` на доле 0, как адаптер на живом клоке."""

    def __init__(self):
        self.stops = 0

    def check(self, program):
        return None

    def start(self, program, on_started):
        on_started({"track_id": program.track_id, "deck": "A", "form_beats": 64.0, "start_beat": 0.0,
                    "phase_in_form": 0, "players_aligned": True, "bpm": program.bpm})
        return {}

    def at(self, beat, fn):
        pass

    def stop(self):
        self.stops += 1


class _Clock:
    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now


def _owner(clock=None):
    adapter = _Adapter()
    owner = PlayerOwner(adapter, lambda _s: None, lambda _e: None, clock=clock or _Clock())
    return owner, adapter


def _program(track_id="req1:01:A", bpm=120.0):
    return SimpleNamespace(track_id=track_id, bpm=bpm, deck="A")


@pytest.fixture
def music_node():
    """Нода с владельцем плеера v2, сетом и хелперами стопа — как в ``MCPServer``."""
    owner, adapter = _owner()
    calls = []
    node = SimpleNamespace(
        _player_owner=owner,
        _dj_set_tool=SimpleNamespace(running=False,
                                     status=lambda: {"active": False, "message": "Диджей-сета не было."}),
        _stop_deck=lambda reason: (calls.append(("deck", reason)), owner.is_playing() and owner.stop(reason)),
        get_logger=lambda: SimpleNamespace(info=lambda *a, **k: None, warning=lambda *a, **k: None,
                                           error=lambda *a, **k: None, debug=lambda *a, **k: None),
    )
    node.calls, node.clock, node.adapter = calls, owner._clock, adapter
    return node


@pytest.mark.unit
class TestStopMusicTool:
    """``stop_music``: сет и дека — через их владельцев (трек кэша тоже на деке, ADR-0158 PR-2); фраза — от кода."""

    def test_tool_name_and_narrow_schema(self, music_node):
        tool = StopMusicTool(music_node)
        assert tool.name == "stop_music"
        assert tool.parameters == []  # имени паттерна больше нет: паттернов LLM не пишет (#3614)

    def test_stops_the_playing_track_through_the_owner(self, music_node):
        owner = music_node._player_owner
        owner.play(_program(), dj={"enabled": False, "title": "космос · 120 BPM"})
        assert owner.is_playing()
        result = StopMusicTool(music_node).execute()
        assert result.success is True
        assert result.data == {"was_playing": True}
        assert result.message == "Музыка остановлена."
        assert not owner.is_playing()
        assert ("deck", "user_stop") in music_node.calls

    def test_silence_still_takes_the_deck_down_and_says_so(self, music_node):
        result = StopMusicTool(music_node).execute()
        assert result.success is True and result.data == {"was_playing": False}
        assert "и так не играла" in result.message
        assert music_node.adapter.stops == 1  # явный стоп в тишине: группа гасится всё равно

    def test_node_without_engine_does_not_crash(self):
        """Renardo не поднялся — владельца нет; стоп не падает."""
        node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda *a, **k: None))
        result = StopMusicTool(node).execute()
        assert result.success is True and result.data == {"was_playing": False}


@pytest.mark.unit
class TestGetMusicStateTool:
    """#3613: при играющем треке v2 тул говорил «Диджей-сета не было … Активные паттерны: нет» — LLM: «тишина»."""

    def _tool(self, node, available=True):
        runtime = SimpleNamespace(available=available, last_error=None if available else "ConnectionError: scsynth")
        return GetMusicStateTool(node, runtime)

    def test_tool_name_and_read_only(self, music_node):
        tool = self._tool(music_node)
        assert tool.name == "get_music_state"
        assert tool.parameters == [] and tool.read_only is True

    def test_silence_is_one_phrase_from_code(self, music_node):
        result = self._tool(music_node).execute()
        assert result.success is True
        assert result.message == "Сейчас ничего не играет."
        assert result.data["playing"] is False

    def test_live_3613_single_v2_track_is_reported_as_playing(self, music_node):
        """Живой лог #3613: ``[music v2] started req99682…`` — а ответ «Диджей-сета не было. Renardo: доступен…»."""
        owner = music_node._player_owner
        owner.play(_program("req99682:03:A", bpm=124.0),
                   dj={"enabled": False, "title": "космос · 124 BPM", "name": "космос", "source": "synth",
                       "style": "club", "bpm": 124})
        music_node.clock.now += 65
        result = self._tool(music_node).execute()
        assert result.data["playing"] is True
        assert result.data["deck"]["track_id"] == "req99682:03:A"
        assert result.data["deck"]["source"] == "synth"
        assert result.message.startswith("Сейчас играет «космос» — трек из синтов, его собрал код, 124 удара в минуту.")
        assert "Играет уже 1 минуту 5 секунд" in result.message and "повторяется" in result.message
        assert "Диджей-сета не было" not in result.message and "паттерн" not in result.message

    def test_piece_tells_remaining_time(self, music_node):
        owner = music_node._player_owner
        owner.play(_program("classic:x", bpm=120.0),
                   dj={"enabled": False, "title": "Toccata", "name": "Toccata, Бах", "source": "piece",
                       "instrument_played": "organ"}, once=True)
        music_node.clock.now += 10  # форма 64 доли на 120 BPM = 32 с, прошло 10
        result = self._tool(music_node).execute()
        assert result.data["deck"]["remaining_s"] == pytest.approx(22.0)
        assert "«Toccata, Бах» — пьеса по нотам партитуры" in result.message
        assert "осталось 22 секунды" in result.message

    def test_running_set_answers_with_the_set_status(self, music_node):
        status = {"active": True, "track_no": 2, "tracks": 5, "message": "Идёт диджей-сет: трек 2 из 5."}
        music_node._dj_set_tool = SimpleNamespace(running=True, status=lambda: status)
        result = self._tool(music_node).execute()
        assert result.message == "Идёт диджей-сет: трек 2 из 5."
        assert result.data["dj_set"] == status and result.data["playing"] is True

    def test_finished_set_and_empty_deck_is_silence(self, music_node):
        """Сет доиграл сам, дека пуста — «ничего не играет» (факт о сете — в ``data``)."""
        status = {"active": False, "ended": True, "message": "Диджей-сет закончился сам: сыграны все 3 из 3."}
        music_node._dj_set_tool = SimpleNamespace(running=False, status=lambda: status)
        result = self._tool(music_node).execute()
        assert result.message == "Сейчас ничего не играет."
        assert result.data["dj_set"] == status

    def test_engine_down_is_said_honestly(self, music_node):
        result = self._tool(music_node, available=False).execute()
        assert "Звуковой движок недоступен: ConnectionError: scsynth" in result.message


class TestLookupMelodyTool:
    """Тул поиска известной мелодии — возвращает ноты, НЕ играет."""

    def _make_tool(self, mock_node, rtttl_library=None) -> LookupMelodyTool:
        return LookupMelodyTool(mock_node, rtttl_library)

    def test_tool_name(self, mock_node):
        assert self._make_tool(mock_node).name == "lookup_melody"

    def test_tool_is_not_destructive(self, mock_node):
        assert self._make_tool(mock_node).destructive is False

    def test_tool_is_read_only(self, mock_node):
        assert self._make_tool(mock_node).read_only is True

    def test_found_melody_returns_raw_rtttl_without_playing(self, mock_node):
        rtttl_library = Mock()
        rtttl_library.get.return_value = {
            "name": "starwars_3",
            "title": "Imperial March",
            "rtttl": "StarWars:d=4,o=5,b=80:8d",
        }
        rtttl_library.vocabulary.return_value = frozenset({"imperial", "march"})
        rtttl_library.search.return_value = []
        tool = LookupMelodyTool(mock_node, rtttl_library)

        result = tool.execute("imperial march")

        assert result.success is True
        assert result.data["rtttl"] == "StarWars:d=4,o=5,b=80:8d"
        assert result.data["title"] == "Imperial March"
        # Играет request_music движка v2, не compose_music (ADR-0149 PR-13b).
        assert "compose_music" not in result.message

    def test_miss_returns_honest_error(self, mock_node):
        rtttl_library = Mock()
        rtttl_library.get.return_value = None
        rtttl_library.search.return_value = []
        rtttl_library.vocabulary.return_value = frozenset()
        tool = self._make_tool(mock_node, rtttl_library)
        result = tool.execute("шопен")
        assert result.success is False
        assert "не знаешь точных нот" in result.error

    def test_key_words_are_a_slot_so_the_router_plays_toccata_in_d_minor_by_code(self, mock_node, tmp_path):
        """ADR-0157 PR-3, #3491: «Токкату ре минор» искалась целиком → «Minority» (Green Day), «токкату» — вне
        поиска (``ignored``) → роутер отдавал заказ LLM (6–13 с). Тональность — слот: запись — Токката Баха,
        сверка полная, роутер (``named_play.melody_hit``) играет кодом."""
        from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
        from rob_box_voice.core.named_play import melody_hit

        rtttl_library = RtttlLibrary(db_path=str(tmp_path / "key_slot.db"))
        result = LookupMelodyTool(mock_node, rtttl_library).execute("токкату ре минор")
        assert result.success and result.data["key"] == "D minor"
        record = rtttl_library.get(result.data["name"])
        assert "toccata" in result.data["title"].lower() and "bach" in str(record.get("artist")).lower()
        hit, reason = melody_hit(result.data)
        assert hit is not None, reason

    def test_variants_are_tried_in_order(self, mock_node):
        """LLM может дать несколько вариантов названия — пробуем по порядку."""
        rtttl_library = Mock()
        vader = {
            "name": "starwars_3",
            # issue #2964: covers_tokens() сверяет запись с кандидатом,
            # который её реально нашёл («darth vader») — title обязан
            # покрывать оба его токена, иначе честный резолв отклонит
            # совпадение как вероятно другую песню.
            "title": "Imperial March (Darth Vader Theme)",
            "rtttl": "x:d=4,o=5,b=80:c",
        }
        # primary name не нашёлся ни как есть, ни поиском по словам (#3399: engine.search.find)
        rtttl_library.get.side_effect = lambda q: vader if q == "darth vader" else None
        rtttl_library.search.return_value = []
        rtttl_library.vocabulary.return_value = frozenset()
        tool = LookupMelodyTool(mock_node, rtttl_library)

        result = tool.execute("imperial march", variants=["darth vader", "star wars"])

        assert result.success is True
        assert result.data["name"] == "starwars_3"
        # Останавливаемся на первом совпадении — третий вариант не нужен.
        assert [c.args[0] for c in rtttl_library.get.call_args_list] == [
            "imperial march",  # engine.search.find
            "imperial march",
            "darth vader",
        ]


class TestLookupMelodyToolTransparencyRealArchive:
    """issue #2964: живой DJ-сет 24.09 — ``lookup_melody`` находил ДРУГУЮ
    песню и предупреждал в message «сверь title», а minimax предупреждение
    игнорировал. Товарищ Шифу отклонил жёсткий гейт found=False (история
    отката #2882→#2896 — единая эвристика отказа ломает сильные
    совпадения). Вместо гейта — честная СТРУКТУРИРОВАННАЯ сверка
    (``data['match']`` — :func:`match_info`, IDF по корпусу, без
    хардкод-списка стоп-слов): какие значимые слова запроса нашлись/не
    нашлись. Решение — у модели, по общему правилу в промпте composer."""

    def _make_tool(self, mock_node, tmp_path):
        from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary

        rtttl_library = RtttlLibrary(db_path=str(tmp_path / "lookup_honest.db"))
        return LookupMelodyTool(mock_node, rtttl_library)

    def test_gin_and_juice_flags_juice_as_unmatched(self, mock_node, tmp_path):
        tool = self._make_tool(mock_node, tmp_path)
        result = tool.execute("Gin and Juice")
        assert result.success is True
        assert result.data["title"] == "Everybody's Changing"
        assert result.data["match"]["unmatched"] == ["juice"]
        assert result.data["match"]["coverage"] < 1.0
        assert result.data["alternatives"] is not None

    def test_nuthin_but_a_g_thang_flags_nuthin_as_unmatched(self, mock_node, tmp_path):
        tool = self._make_tool(mock_node, tmp_path)
        result = tool.execute("Nuthin But A G Thang")
        assert result.success is True
        assert result.data["title"] == "If I Can Poppin Them Thangs"
        assert "nuthin" in result.data["match"]["unmatched"]
        assert result.data["match"]["coverage"] < 1.0

    def test_terminator_theme_reports_full_coverage_via_artist(self, mock_node, tmp_path):
        """theme_178 (title «Theme», artist «Terminatorv v2.0») — правильная
        тема; полное покрытие («terminator» покрыт artist, «theme» —
        title), и display_title добавляет исполнителя, раз голое «Theme»
        неинформативно по корпусу архива (сотни записей так называются)."""
        tool = self._make_tool(mock_node, tmp_path)
        result = tool.execute("terminator theme")
        assert result.success is True
        assert result.data["title"] == "Theme"
        assert "Terminatorv" in result.data["display_title"]
        assert result.data["match"]["unmatched"] == []
        assert result.data["match"]["coverage"] == 1.0
        assert result.data["name"] == "theme_178"
