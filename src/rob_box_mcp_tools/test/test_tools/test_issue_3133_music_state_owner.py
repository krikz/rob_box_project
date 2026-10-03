"""Issue #3133 (ADR-0141) — плеер сам закрывает сессию, когда конечный трек доиграл.

Живой прогон 28.09.2026 18:43–18:45 UTC: club-трек 79 с (``repeat=False``)
замолчал около 18:43:30 (Renardo ``Clock.future(end, Clock.clear)``), а
``/voice/music/state`` ещё ~30 минут публиковал ``playing``: сессию закрывали
только ``stop_all`` и idle-TTL 1800 с. Робот говорил «Музыка играет» в тишине.

Здесь — сторона ``MusicManager``: конец формы переводит сессию в idle с id
доигравшего трека; зацикленный трек (DJ-сет) не трогается; удержание формы
от idle-TTL (#1812) и segments-дедлайн (#990) работают как раньше.
"""

import time
from unittest.mock import Mock, patch

import pytest

from .._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.tools.music import ComposeMusicTool, MusicManager
_ros_stubs = _ros.fixture()


def _make_manager() -> MusicManager:
    """MusicManager без Renardo/SC — те же дефолты, что в ``__init__``."""
    from rob_box_voice.core.music_stack_validation import MusicStackStatus

    mgr = MusicManager.__new__(MusicManager)
    mgr._max_amp = 0.7
    mgr._pattern_history = {}
    mgr._active_patterns = set()
    mgr._synthdefs_added = set()
    mgr._current_preset = None
    mgr._renardo_available = True
    mgr._renardo_last_error = None
    mgr._renardo_context = {}
    mgr._music_stack_status = MusicStackStatus(
        is_healthy=True, oscdef_registered=True, missing_synths=(), fatal_errors=(),
    )
    mgr._require_healthy = True
    mgr._critical_synths = MusicManager.DEFAULT_CRITICAL_SYNTHS
    mgr._auto_stop_ttl_seconds = 300
    mgr._music_session_active_since = None
    mgr._last_music_activity_at = None
    mgr._last_stop_at = None
    mgr._auto_stop_count = 0
    mgr._music_deadline_at = None
    mgr._music_deadline_segments = None
    mgr._music_form_deadline_at = None
    mgr._music_form_cycle_ends_at = None
    mgr.last_track_arrangement = None
    mgr._dj_mode_enabled = False
    mgr._check_supercollider = Mock(return_value=True)
    # Phase 4 (ADR-0134) — ``MusicManager.execute_code``/etc. стали
    # 1-строчными wrapper'ами ``return self._runtime.<method>(...)``.
    # ``_make_manager`` обходит ``__init__`` через ``__new__``, поэтому
    # обязана сама навесить ``_runtime`` (см. test_music.py::_make_manager).
    from rob_box_mcp_tools.core.music_pattern_runtime import MusicPatternRuntime
    mgr._runtime = MusicPatternRuntime(mgr)
    return mgr


_COMMON_KWARGS = dict(
    bpm=100,
    root="C",
    scale="minor",
    form="arc",
    drums="X..o.X.o",
    bass_synth="dub",
    bass_notes="0, 0, 3, -2",
    lead_synth="blip",
    lead_notes="0, 2, 4, 7",
)


def _compose(mock_node, mgr, *, repeat: bool):
    tool = ComposeMusicTool(mock_node, mgr)
    with patch("builtins.exec"):
        result = tool.execute(repeat=repeat, **_COMMON_KWARGS)
    assert result.success is True, result.error
    return result


def _is_playing(mgr) -> bool:
    """То же правило, что ``MCPServer.publish_music_state``."""
    state = mgr.get_state()
    return bool(state["active_patterns"]) or state["music_session_active_since"] is not None


@pytest.mark.unit
class TestFiniteTrackFinishes:
    def test_finished_after_form_end_carries_the_track_id(self, mock_node):
        mgr = _make_manager()
        _compose(mock_node, mgr, repeat=False)
        track_id = mgr.get_state()["track_id"]
        assert track_id
        assert _is_playing(mgr)

        finished = mgr.finish_form_if_ended(now=mgr._music_form_deadline_at + 0.01)

        assert finished == {
            "track_id": track_id,
            "track_name": None,
            "reason": "form_end",
        }
        state = mgr.get_state()
        assert _is_playing(mgr) is False
        assert state["track_id"] is None
        assert state["last_finished_track_id"] == track_id
        # Конечного трека больше нет — и остатка до его остановки тоже.
        assert state["form_stop_remaining_s"] is None

    def test_no_playing_after_stops_at_even_with_idle_ttl_far_away(self, mock_node):
        """Живой случай: idle-TTL 1800 с не должен держать «playing» после формы."""
        mgr = _make_manager()
        _compose(mock_node, mgr, repeat=False)
        after_end = mgr._music_form_deadline_at + 1.0

        mgr.finish_form_if_ended(now=after_end)
        watchdog = mgr.auto_stop_idle_music(ttl_seconds=1800, now=after_end)

        assert _is_playing(mgr) is False
        # Сессия уже закрыта — idle-TTL стопать нечего.
        assert watchdog["stopped"] is False

    def test_not_finished_before_the_form_ends(self, mock_node):
        mgr = _make_manager()
        _compose(mock_node, mgr, repeat=False)

        assert mgr.finish_form_if_ended(now=mgr._music_form_deadline_at - 0.5) is None
        assert _is_playing(mgr)

    def test_finish_does_not_touch_renardo(self, mock_node):
        """Renardo уже замолчал сам — /g_freeAll оборвал бы хвосты релизов."""
        mgr = _make_manager()
        _compose(mock_node, mgr, repeat=False)
        mgr._send_osc_raw = Mock()

        with patch("builtins.exec") as fake_exec:
            mgr.finish_form_if_ended(now=mgr._music_form_deadline_at + 0.01)

        fake_exec.assert_not_called()
        mgr._send_osc_raw.assert_not_called()

    def test_second_call_is_a_no_op(self, mock_node):
        mgr = _make_manager()
        _compose(mock_node, mgr, repeat=False)
        end = mgr._music_form_deadline_at + 0.01
        assert mgr.finish_form_if_ended(now=end) is not None
        assert mgr.finish_form_if_ended(now=end + 5.0) is None


@pytest.mark.unit
class TestLoopingAndReplacedTracksKeepPlaying:
    def test_repeat_true_never_finishes(self, mock_node):
        """DJ-сет играет repeat=True — конец прохода формы не конец музыки."""
        mgr = _make_manager()
        _compose(mock_node, mgr, repeat=True)

        far_future = time.monotonic() + 10_000.0
        assert mgr.finish_form_if_ended(now=far_future) is None
        assert _is_playing(mgr)
        assert mgr.get_state()["track_id"]

    def test_new_track_before_the_end_is_not_finished_retroactively(self, mock_node):
        mgr = _make_manager()
        _compose(mock_node, mgr, repeat=False)
        first_id = mgr.get_state()["track_id"]
        old_end = mgr._music_form_deadline_at

        _compose(mock_node, mgr, repeat=True)  # DJ-переход на зацикленный трек

        assert mgr.finish_form_if_ended(now=old_end + 1.0) is None
        state = mgr.get_state()
        assert _is_playing(mgr)
        assert state["track_id"] and state["track_id"] != first_id

    def test_explicit_stop_is_not_reported_as_finished(self, mock_node):
        mgr = _make_manager()
        _compose(mock_node, mgr, repeat=False)
        end = mgr._music_form_deadline_at
        mgr.finish_form_if_ended(now=end + 0.01)
        assert mgr.get_state()["last_finished_track_id"]

        with patch("builtins.exec"):
            mgr.stop_all()

        state = mgr.get_state()
        assert state["last_finished_track_id"] is None
        assert state["track_id"] is None

    def test_new_track_clears_the_previous_finished_marker(self, mock_node):
        mgr = _make_manager()
        _compose(mock_node, mgr, repeat=False)
        mgr.finish_form_if_ended(now=mgr._music_form_deadline_at + 0.01)

        _compose(mock_node, mgr, repeat=False)

        assert mgr.get_state()["last_finished_track_id"] is None


@pytest.mark.unit
class TestExistingSafetyNetsUnchanged:
    def test_idle_ttl_still_held_while_the_form_plays(self, mock_node):
        """#1812: форма ещё играет — idle-TTL не гасит (и finish тоже)."""
        mgr = _make_manager()
        _compose(mock_node, mgr, repeat=False)
        inside = mgr._music_form_deadline_at - 1.0

        assert mgr.finish_form_if_ended(now=inside) is None
        result = mgr.auto_stop_idle_music(ttl_seconds=1, now=inside)
        assert result["stopped"] is False
        assert result.get("held_reason") == "form_not_finished"

    def test_segments_deadline_still_stops_before_the_form_ends(self, mock_node):
        """#990: зависший TTS-батч гасит музыку раньше конца формы."""
        mgr = _make_manager()
        _compose(mock_node, mgr, repeat=False)
        mgr._music_deadline_at = mgr._music_form_deadline_at - 10.0
        mgr._music_deadline_segments = 4

        with patch("builtins.exec"):
            result = mgr.auto_stop_idle_music(
                ttl_seconds=1800, now=mgr._music_deadline_at + 0.1
            )

        assert result["stopped"] is True
        assert result["stop_reason"] == "segments_deadline"
        assert mgr.get_state()["last_finished_track_id"] is None

    def test_dj_flag_is_in_the_state_snapshot(self):
        mgr = _make_manager()
        assert mgr.get_state()["dj_mode_enabled"] is False
        mgr.set_dj_mode(True)
        assert mgr.get_state()["dj_mode_enabled"] is True
