"""``CompositionSpec`` -> события: проверка на маленькой ручной спеке.

Не use-case архива (это делает ``sampler``/``run_bench``) — маленькая
спека, собранная напрямую через ``arranger``, чтобы проверить контракт
:func:`extract_events` без зависимости от содержимого RTTTL-архива.
"""

from __future__ import annotations

from tools.music_bench import _repo_paths  # noqa: F401
from tools.music_bench.note_extract import extract_events, midi_to_freq

from rob_box_mcp_tools.core.arranger import BEATS_PER_BAR, CompositionSpec, Layer


def test_midi_to_freq_a4_is_440():
    assert abs(midi_to_freq(69) - 440.0) < 1e-6


def test_midi_to_freq_octave_doubles():
    assert abs(midi_to_freq(81) - 2 * midi_to_freq(69)) < 1e-6


def _tiny_spec(**layer_kwargs) -> CompositionSpec:
    layer = Layer(role="bass", synth="moogbass", midi=(36, None, 43), durs=(1.0, 1.0, 2.0), **layer_kwargs)
    return CompositionSpec(bpm=120.0, root="C", scale="minor", form="ambient", layers=(layer,), theme_bars=1)


def test_extract_events_skips_rests():
    spec = _tiny_spec()
    events, total_beats = extract_events(spec)
    assert total_beats > 0
    # 3 записи в midi, одна — None (пауза) -> она не должна дать событие.
    assert all(ev.freqs_hz for ev in events)


def test_extract_events_note_frequency_matches_midi():
    spec = _tiny_spec()
    events, _total = extract_events(spec)
    assert events, "ожидались события для непустого баса"
    freqs = {round(ev.freqs_hz[0], 2) for ev in events}
    assert round(midi_to_freq(36), 2) in freqs


def test_extract_events_pattern_tiles_across_whole_form():
    """Форма ``ambient`` длиннее суммы durs слоя (1+1+2=4 бита) -> тема
    должна повториться (тайлинг), а не сыграть один раз и замолчать."""
    spec = _tiny_spec()
    events, total_beats = extract_events(spec)
    assert total_beats > 4 * BEATS_PER_BAR  # ambient точно длиннее одного цикла темы
    starts = sorted(ev.start_beat for ev in events)
    assert len(starts) > 2  # больше одного прохода 2 звучащих нот темы


def test_extract_events_empty_spec_has_no_events():
    """``extract_events`` читает спеку, а не валидирует её (это дело ``arranger.render``)."""
    spec = CompositionSpec(bpm=120.0, root="C", scale="minor", form="arc", layers=())
    events, _total = extract_events(spec)
    assert events == []
