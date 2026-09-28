"""Issue #3161 — ``<music_state>`` в промпте строится из снимка плеера.

Живой прогон 28.09 22:32 (деплой develop 8888d6f02)::

    22:32:07  «Робот, выключи музыку» → роутер stop_music
    22:32:20  «Робот, а какую музыку ты сейчас включал?»
    +4.4s     spoken='Сейчас играет клубный трек …'   ← музыка стоит 10 с

Тег собирался эвристикой ``beat`` (cleanup-флаг / живые TTS-батчи), а не
из ``/voice/music/state``. Здесь — что тег говорит только то, что сообщил
плеер: играет/нет, какой трек, конечный/повтор, DJ, и что играло недавно.
"""

from __future__ import annotations

from rob_box_voice.core.music_player_state import (
    STOPS_AT_GRACE_S,
    MusicPlayerState,
    parse_music_state,
    build_music_state_payload,
)
from rob_box_voice.core.music_state_prompt import (
    ENDED_FINISHED,
    ENDED_REPLACED,
    ENDED_STOPPED,
    RECENT_END_WINDOW_S,
    MusicStateMemory,
)

T0 = 1_790_000_000.0


def _snap(**kw) -> MusicPlayerState:
    return parse_music_state(build_music_state_payload(**kw))


def _playing(memory: MusicStateMemory, track_id="club-1", name="клубный трек",
             at=T0, **kw) -> None:
    memory.observe_state(_snap(playing=True, track_id=track_id, ts=at, **kw))
    memory.observe_track_name(name)


class TestNoSnapshot:
    def test_unknown_before_player_reported(self):
        tag = MusicStateMemory().render(now=T0)
        assert 'playing="unknown"' in tag
        assert "last_track" not in tag
        assert tag.count("<music_state") == 1 and tag.endswith("/>")


class TestPlaying:
    def test_looping_track_with_name(self):
        memory = MusicStateMemory()
        _playing(memory)
        tag = memory.render(now=T0 + 5)
        assert 'playing="yes"' in tag
        assert 'track="клубный трек"' in tag
        assert 'repeat="loop"' in tag
        assert "ends_in_s" not in tag
        assert 'dj="off"' in tag

    def test_finite_track_reports_seconds_left(self):
        memory = MusicStateMemory()
        _playing(memory, stops_at=T0 + 60)
        tag = memory.render(now=T0 + 18)
        assert 'repeat="once"' in tag
        assert 'ends_in_s="42"' in tag

    def test_dj_flag_comes_from_snapshot(self):
        memory = MusicStateMemory()
        _playing(memory, dj=True)
        assert 'dj="on"' in memory.render(now=T0 + 1)

    def test_track_without_name_is_labelled_unnamed(self):
        memory = MusicStateMemory()
        memory.observe_state(_snap(playing=True, track_id="x", ts=T0))
        assert 'track="без названия"' in memory.render(now=T0 + 1)

    def test_generated_mp3_is_reported_separately(self):
        memory = MusicStateMemory()
        memory.observe_state(_snap(playing=False, ts=T0))
        tag = memory.render(generated_title="Synthwave", now=T0 + 1)
        assert 'ai="playing: Synthwave"' in tag
        assert 'playing="no"' in tag

    def test_name_is_xml_escaped(self):
        memory = MusicStateMemory()
        _playing(memory, name='Techno "rave" & <more>')
        tag = memory.render(now=T0 + 1)
        assert 'track="Techno &quot;rave&quot; &amp; &lt;more&gt;"' in tag


class TestLiveStopThenQuestion:
    """Дословный живой сценарий #3161: стоп → через 13 с вопрос."""

    def test_idle_after_stop_says_what_played_and_when(self):
        memory = MusicStateMemory()
        _playing(memory, at=T0)
        # 22:32:07 роутер: stop_music → плеер публикует idle (без finished).
        memory.observe_state(_snap(playing=False, ts=T0 + 100))
        memory.observe_track_name(None)  # form после idle: track=None
        tag = memory.render(now=T0 + 113)
        assert 'playing="no"' in tag
        assert 'playing="yes"' not in tag
        assert 'last_track="клубный трек"' in tag
        assert f'last_ended="{ENDED_STOPPED}"' in tag
        assert 'last_ended_ago_s="13"' in tag

    def test_heuristic_inputs_do_not_matter(self):
        """Нет больше ``beat``/``cleanup`` — только снимок."""
        memory = MusicStateMemory()
        memory.observe_state(_snap(playing=False, ts=T0))
        tag = memory.render(now=T0 + 1)
        assert "beat=" not in tag
        assert "cleanup=" not in tag


class TestEndings:
    def test_finished_by_itself(self):
        memory = MusicStateMemory()
        _playing(memory, stops_at=T0 + 30)
        memory.observe_state(
            _snap(playing=False, finished_track_id="club-1", ts=T0 + 30.3)
        )
        tag = memory.render(now=T0 + 40)
        assert f'last_ended="{ENDED_FINISHED}"' in tag
        assert 'last_ended_ago_s="10"' in tag

    def test_stale_finished_id_of_other_track_means_stopped(self):
        memory = MusicStateMemory()
        _playing(memory, track_id="b-2")
        memory.observe_state(
            _snap(playing=False, finished_track_id="a-1", ts=T0 + 5)
        )
        assert f'last_ended="{ENDED_STOPPED}"' in memory.render(now=T0 + 6)

    def test_replaced_by_another_track(self):
        memory = MusicStateMemory()
        _playing(memory, track_id="a-1", name="Марио")
        memory.observe_state(_snap(playing=True, track_id="b-2", ts=T0 + 20))
        tag = memory.render(now=T0 + 21)
        # Имя нового ещё не пришло из form — не выдаём старое за новое.
        assert 'track="без названия"' in tag
        assert 'last_track="Марио"' in tag
        assert f'last_ended="{ENDED_REPLACED}"' in tag
        memory.observe_track_name("клубный трек")
        assert 'track="клубный трек"' in memory.render(now=T0 + 22)

    def test_finite_track_past_stops_at_without_idle(self):
        """idle потерялся — после stops_at + grace трек уже «доиграл»."""
        memory = MusicStateMemory()
        _playing(memory, stops_at=T0 + 30)
        tag = memory.render(now=T0 + 30 + STOPS_AT_GRACE_S + 5)
        assert 'playing="no"' in tag
        assert 'last_track="клубный трек"' in tag
        assert f'last_ended="{ENDED_FINISHED}"' in tag

    def test_old_ending_is_forgotten(self):
        memory = MusicStateMemory()
        _playing(memory)
        memory.observe_state(_snap(playing=False, ts=T0 + 1))
        tag = memory.render(now=T0 + 1 + RECENT_END_WINDOW_S + 1)
        assert "last_track" not in tag

    def test_repeated_idle_does_not_overwrite_ending(self):
        """Watchdog публикует idle каждые ~5 с — «когда» не сдвигается."""
        memory = MusicStateMemory()
        _playing(memory)
        memory.observe_state(_snap(playing=False, ts=T0 + 10))
        memory.observe_state(_snap(playing=False, ts=T0 + 15))
        memory.observe_state(_snap(playing=False, ts=T0 + 20))
        assert 'last_ended_ago_s="20"' in memory.render(now=T0 + 30)

    def test_name_from_form_ignored_while_idle(self):
        memory = MusicStateMemory()
        memory.observe_state(_snap(playing=False, ts=T0))
        memory.observe_track_name("призрак")
        assert memory.track_name is None
        assert "призрак" not in memory.render(now=T0 + 1)
