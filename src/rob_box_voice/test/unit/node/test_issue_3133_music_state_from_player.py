"""Issue #3133 (ADR-0141) — «играет» знает только плеер; стоп-тулы музыки.

Живой прогон 28.09.2026 18:43–18:45 UTC: конечный club-трек 79 с доиграл,
jack_rec писал −180 dBFS, а диалог логировал «TRACK играет с прошлого
хода», и робот сказал «Музыка играет, а вот эту просьбу я выполнить не смог».

PR-15 (#3711, часть 15p): чтение снимка плеера нодой диалога v1 удалено вместе
с ней (v2 читает ``/voice/music/state`` в ``dialog_node``); здесь остаётся
набор стоп-тулов музыки.
"""

from __future__ import annotations

from rob_box_voice.core.dialogue_guards import MUSIC_STOP_TOOLS


class TestStopToolSet:
    def test_set_dj_mode_is_not_a_stop_tool(self):
        # ADR-0149 PR-13a: set_dj_mode — тул старого пути, DJ-сет ведёт dj_set движка v2.
        assert "set_dj_mode" not in MUSIC_STOP_TOOLS
        assert MUSIC_STOP_TOOLS == frozenset({"stop_music"})
