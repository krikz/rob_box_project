"""Тесты аранжировщика формы (docs/analysis/2026-08-30-music-quality-audit.md, RC4).

Главный контракт модуля: LLM отдаёт материал, аранжировщик отдаёт форму.
Поэтому тесты проверяют не «код сгенерировался», а что в коде ЕСТЬ форма —
огибающие, регистровое разделение, вход и уход слоёв.
"""

import re
import sys
from unittest.mock import MagicMock

import pytest

# ``arranger`` itself is ROS-free by design, but TestGeneratedCodePassesExisting
# Guards reaches into ``tools.music`` to run the real filters — and that import
# chain pulls rclpy. Same stub block as test_tools/test_music.py.
for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
    "std_msgs",
    "std_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "action_msgs",
    "action_msgs.srv",
    "action_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.core.arranger import (  # noqa: E402
    BEATS_PER_BAR,
    DEFAULT_FORM,
    FORMS,
    ROLE_PROFILE,
    ArrangementError,
    CompositionSpec,
    Layer,
    form_summary,
    render,
    resolve_form,
    spec_from_flat,
)


def _spec(**kwargs) -> CompositionSpec:
    """Полноценная спецификация из четырёх слоёв — типичный запрос."""
    base = dict(
        bpm=104,
        root="D",
        scale="dorian",
        form="arc",
        layers=(
            Layer(role="drums", pattern="X..o.X.o", sample=2),
            Layer(role="hats", pattern="--.-", sample=3),
            Layer(role="bass", synth="dub", degrees=(0, 0, 3, -2), dur=1),
            Layer(role="lead", synth="blip", degrees=(0, 2, 4, 7, 4, 2), dur=0.5),
            Layer(role="pad", synth="warmpad", degrees=(0, 4, 7), dur=4),
        ),
    )
    base.update(kwargs)
    return CompositionSpec(**base)


class TestForm:
    def test_amp_envelope_is_emitted_per_layer(self):
        """Без var() на amp слой играет одинаково вечно — это и есть RC4."""
        code = render(_spec())
        for line in code.splitlines():
            if ">>" in line:
                assert "amp=var(" in line, f"слой без огибающей формы: {line}"

    def test_envelope_durations_are_beats_not_bars(self):
        """TimeVar считает в битах (bar_length()=4), а форма задана в тактах."""
        code = render(_spec(form="arc", filter_sweep=False))
        bass = next(l for l in code.splitlines() if l.startswith("p1 >>"))
        durs = re.search(r"amp=var\(\[[^\]]*\], \[([^\]]*)\]\)", bass).group(1)
        total = sum(int(x) for x in durs.split(","))
        expected = sum(bars for _n, bars, _i in FORMS["arc"]) * BEATS_PER_BAR
        assert total == expected

    def test_adjacent_equal_sections_are_merged(self):
        """var([0.5,0.5],[32,32]) — лишнее переключение и нечитаемый код."""
        code = render(_spec())
        for match in re.finditer(r"amp=var\(\[([^\]]*)\]", code):
            values = [v.strip() for v in match.group(1).split(",")]
            assert all(
                a != b for a, b in zip(values, values[1:])
            ), f"соседние одинаковые значения не склеены: {values}"

    def test_layer_enters_and_leaves_across_the_form(self):
        """Барабанов нет в intro и они возвращаются позже — это вход слоя."""
        code = render(_spec())
        drums = next(l for l in code.splitlines() if l.startswith("d1 >>"))
        values = re.search(r"amp=var\(\[([^\]]*)\]", drums).group(1)
        levels = [float(v) for v in values.split(",")]
        assert levels[0] == 0.0, "барабаны обязаны молчать в intro"
        assert max(levels) > 0.0

    def test_unknown_form_falls_back_to_default(self):
        """LLM регулярно выдумывает названия — лучше сыграть, чем отказать."""
        assert resolve_form("психоделический_джаз") is FORMS[DEFAULT_FORM]
        assert resolve_form(None) is FORMS[DEFAULT_FORM]
        assert render(_spec(form="нет такой формы"))

    def test_ambient_form_omits_silent_percussion_layers(self):
        """В ambient барабанов нет ни в одной секции — плеер не создаём."""
        code = render(_spec(form="ambient"))
        assert "d1 >>" not in code
        assert "p3 >>" in code  # пэд — основа этой формы


class TestRegisters:
    def test_roles_are_spread_across_octaves(self):
        """RC2: бас, пэд и лид в соседних октавах = каша вместо аранжировки."""
        code = render(_spec())
        octaves = {
            line.split(" >>")[0]: int(re.search(r"oct=(\d+)", line).group(1))
            for line in code.splitlines()
            if ">>" in line and "oct=" in line
        }
        assert octaves["p1"] < octaves["p3"] < octaves["p2"], (
            f"бас/пэд/лид должны быть разведены по регистрам, получено {octaves}"
        )

    def test_oct_shift_is_applied_and_clamped(self):
        code = render(
            _spec(
                layers=(
                    Layer(role="bass", synth="dub", degrees=(0, 3), dur=1, oct_shift=-1),
                    Layer(role="pad", synth="warmpad", degrees=(0, 4), dur=4),
                )
            )
        )
        assert "oct=2" in code
        code = render(
            _spec(
                layers=(
                    Layer(role="lead", synth="blip", degrees=(0, 2), dur=0.5,
                          oct_shift=99),
                    Layer(role="pad", synth="warmpad", degrees=(0, 4), dur=4),
                )
            )
        )
        assert "oct=7" in code and "oct=99" not in code

    def test_players_stay_within_the_deployable_range(self):
        """d4+/p4+ на этом роботе не звучат — см. music_skill_prompt.txt."""
        code = render(_spec())
        players = {line.split(" >>")[0] for line in code.splitlines() if ">>" in line}
        assert players <= {"d1", "d2", "d3", "p1", "p2", "p3"}
        assert players <= {p for p, _o, _a in ROLE_PROFILE.values()}


class TestHarmonyAndMotion:
    def test_progression_spans_exactly_one_form(self):
        """Иначе гармония и аранжировка разъезжаются на длинной дистанции."""
        code = render(_spec(progression=(0, 0, 5, 3)))
        step = int(re.search(r"Root\.default = var\(\[.*?\], (\d+)\)", code).group(1))
        total = sum(bars for _n, bars, _i in FORMS["arc"]) * BEATS_PER_BAR
        assert step * 4 == total

    def test_static_root_when_no_progression(self):
        code = render(_spec(root="F"))
        assert 'Root.default = "F"' in code

    def test_invalid_root_falls_back_rather_than_raising(self):
        assert 'Root.default = "C"' in render(_spec(root="H"))

    def test_filter_sweep_skips_percussion(self):
        """Срезанная атака бочки слышна как проваленный грув."""
        code = render(_spec())
        for line in code.splitlines():
            if line.startswith(("d1 >>", "d2 >>", "d3 >>")):
                assert "lpf=gflt" not in line
        assert "gflt = linvar(" in code
        assert "lpf=gflt" in next(l for l in code.splitlines() if l.startswith("p1 >>"))

    def test_filter_sweep_can_be_disabled(self):
        code = render(_spec(filter_sweep=False))
        assert "gflt" not in code


class TestEnding:
    def test_repeat_true_never_schedules_a_stop(self):
        """DJ-режим: форма зацикливается сама, останавливает система."""
        assert "Clock.future" not in render(_spec(repeat=True))

    def test_repeat_false_ends_after_exactly_one_form(self):
        code = render(_spec(repeat=False))
        total = sum(bars for _n, bars, _i in FORMS["arc"]) * BEATS_PER_BAR
        assert f"Clock.future({total}, Clock.clear)" in code

    def test_ending_passes_a_callable_not_a_lambda(self):
        """lambda режется AST-фильтром безопасности."""
        assert "lambda" not in render(_spec(repeat=False))


class TestValidation:
    def test_bpm_is_clamped(self):
        assert "Clock.bpm = 180" in render(_spec(bpm=9000))
        assert "Clock.bpm = 60" in render(_spec(bpm=1))

    def test_unknown_role_is_rejected(self):
        with pytest.raises(ArrangementError, match="Неизвестная роль"):
            render(_spec(layers=(Layer(role="теремин", synth="blip", degrees=(0,)),)))

    def test_drum_layer_without_pattern_is_rejected(self):
        with pytest.raises(ArrangementError, match="pattern"):
            render(_spec(layers=(Layer(role="drums", sample=1),)))

    def test_melodic_layer_without_degrees_is_rejected(self):
        with pytest.raises(ArrangementError, match="degrees"):
            render(_spec(layers=(Layer(role="lead", synth="blip"),)))

    def test_empty_spec_is_rejected(self):
        with pytest.raises(ArrangementError, match="без слоёв"):
            render(_spec(layers=()))

    def test_numbers_are_formatted_readably(self):
        """Код попадает в логи и в save_track — 0.30000000000000004 там лишний."""
        code = render(_spec())
        assert not re.search(r"\d\.\d{6,}", code), code


class TestPatternLengthNormalization:
    """``spec_from_flat`` приводит drums/hats/perc к степени двойки (#1803).

    Живые прогоны 30-31.08, четыре трека подряд: модель писала рисунки
    вроде ``"X..o.X.o."`` (9 шагов) рядом с ``"....o..."`` (8) — период не
    делит такт, и грув «плывёт» на каждом повторе. Модель символы не
    считает, поэтому приведение — на стороне ``spec_from_flat``, до того
    как рисунок попадёт в ``render()``.
    """

    def _flat(self, **kwargs):
        base = dict(bpm=120, root="C", scale="minor", form="arc")
        base.update(kwargs)
        return spec_from_flat(**base)

    def test_nine_step_drums_padded_to_sixteen(self):
        spec = self._flat(drums="X..X.o...")  # 9 шагов, живой инцидент
        drums = next(l for l in spec.layers if l.role == "drums")
        assert drums.pattern == "X..X.o...-------"
        assert len(drums.pattern) == 16

    def test_power_of_two_pattern_is_untouched(self):
        spec = self._flat(hats="--.-")  # уже 4 шага — трогать нечего
        hats = next(l for l in spec.layers if l.role == "hats")
        assert hats.pattern == "--.-"

    def test_eighteen_step_perc_padded_to_thirty_two(self):
        spec = self._flat(perc=".......O.......O..")  # 19 шагов, live 31.08
        perc = next(l for l in spec.layers if l.role == "perc")
        assert len(perc.pattern) == 32
        assert perc.pattern.startswith(".......O.......O..")

    def test_single_step_pattern_is_left_alone(self):
        # Длина 0/1 тривиально делит такт — приводить нечего.
        spec = self._flat(drums="X")
        drums = next(l for l in spec.layers if l.role == "drums")
        assert drums.pattern == "X"


class TestGeneratedCodePassesExistingGuards:
    """Аранжировщик обязан укладываться в уже существующие фильтры.

    Иначе он генерирует красивый код, который отклоняется на входе в
    Renardo — ровно та ловушка, в которую попали GitHub-пресеты из
    миграции 006 (их отключили миграцией 008 вместо адаптации).
    """

    @pytest.fixture
    def manager(self):
        from rob_box_mcp_tools.tools.music import MusicManager

        mgr = MusicManager.__new__(MusicManager)
        mgr._max_amp = 0.85
        return mgr

    @pytest.mark.parametrize("form", sorted(FORMS))
    def test_output_passes_safety_filter(self, manager, form):
        code = render(_spec(form=form, repeat=False))
        is_safe, error = manager._filter_code(code)
        assert is_safe, f"форма {form}: {error}"

    @pytest.mark.parametrize("form", sorted(FORMS))
    def test_output_passes_music_quality_validator(self, manager, form):
        code = render(_spec(form=form))
        errors, warnings = manager._validate_music_code(code)
        assert errors == [], f"форма {form}: {errors}"
        assert warnings == [], f"форма {form}: {warnings}"

    def test_amp_cap_does_not_flatten_the_envelope(self, manager):
        """RC1: покомпонентный кап не должен съедать динамику формы.

        Раньше max_amp=0.42 срезал все пики до одного уровня и форма
        превращалась в плоскую линию.
        """
        code = manager._cap_amp(render(_spec()))
        lead = next(l for l in code.splitlines() if l.startswith("p2 >>"))
        levels = {
            float(v)
            for v in re.search(r"amp=var\(\[([^\]]*)\]", lead).group(1).split(",")
        }
        assert len(levels) > 1, f"огибающая схлопнулась в одно значение: {levels}"


class TestAutofill:
    """The prompt asks for 3+ layers; the model keeps sending 2.

    Live 30.08, twice in a row: lead + pad, no bass. The form still plans a
    bass part (``ambient``'s ``swell`` section), so the middle of the piece
    caves in. Enforced in code rather than by another prompt rule.
    """

    def _flat(self, **kwargs):
        base = dict(
            bpm=88,
            root="A",
            scale="lydian",
            form="ambient",
            lead_synth="sitar",
            lead_notes="0, 2, 4, 7",
            pad_synth="ambi",
            pad_notes="2, 4, 7",
        )
        base.update(kwargs)
        return spec_from_flat(**base)

    def test_missing_bass_is_derived_from_the_harmony(self):
        spec = self._flat()
        bass = next(l for l in spec.layers if l.role == "bass")
        # Root of the pad, not an invented degree — it has to consonate
        # with what is already sounding.
        assert bass.degrees[0] == 2
        assert 4 in [d - bass.degrees[0] for d in bass.degrees]

    def test_lead_is_the_fallback_source_when_there_is_no_pad(self):
        spec = self._flat(pad_synth=None, pad_notes=None)
        bass = next(l for l in spec.layers if l.role == "bass")
        assert bass.degrees[0] == 0

    def test_supplied_bass_is_never_overridden(self):
        spec = self._flat(bass_synth="fuzz", bass_notes="0, -3")
        basses = [l for l in spec.layers if l.role == "bass"]
        assert len(basses) == 1
        assert basses[0].synth == "fuzz"
        assert basses[0].degrees == (0, -3)

    def test_no_bass_invented_for_a_form_that_never_uses_one(self):
        original = dict(FORMS)
        FORMS["_test_no_bass"] = [
            ("a", 4, {"pad": 0.6, "lead": 0.4}),
            ("b", 8, {"pad": 0.8, "lead": 0.6}),
        ]
        try:
            spec = self._flat(form="_test_no_bass")
            assert not any(l.role == "bass" for l in spec.layers)
        finally:
            FORMS.clear()
            FORMS.update(original)

    def test_autofilled_bass_renders_in_its_own_register(self):
        code = render(self._flat())
        bass_line = next(l for l in code.splitlines() if l.startswith("p1 >>"))
        assert "oct=3" in bass_line


class TestOpeningIsAudible:
    """A form must not open with one quiet layer for half a minute.

    Live 30.08: ``ambient`` began with 16 bars of pad alone. At 80 BPM that
    is 48 seconds before anything moves — the listener concludes the robot
    is broken. Applies to every form, so it is pinned as an invariant
    rather than patched per-form.
    """

    @pytest.mark.parametrize("form", sorted(FORMS))
    def test_first_section_has_at_least_two_voices(self, form):
        _name, _bars, intensities = FORMS[form][0]
        audible = [role for role, level in intensities.items() if level > 0]
        assert len(audible) >= 2, (
            f"форма {form!r} открывается одним слоем {audible} — "
            "слушателю нечего услышать"
        )

    @pytest.mark.parametrize("form", sorted(FORMS))
    def test_opening_section_is_not_longer_than_eight_bars(self, form):
        _name, bars, _intensities = FORMS[form][0]
        assert bars <= 8, (
            f"вступление формы {form!r} длится {bars} тактов — на медленном "
            "темпе это десятки секунд до первого события"
        )

    @pytest.mark.parametrize("form", sorted(FORMS))
    def test_sections_are_not_a_symmetric_grid(self, form):
        """16/16/16/16 на слух — тот же луп, только длиннее."""
        lengths = [bars for _n, bars, _i in FORMS[form]]
        assert len(set(lengths)) > 1, (
            f"форма {form!r} состоит из одинаковых секций {lengths}"
        )


class TestSummary:
    def test_summary_lists_sections_and_total(self):
        summary = form_summary("verse_chorus")
        assert "intro(8)" in summary
        total = sum(bars for _n, bars, _i in FORMS["verse_chorus"])
        assert f"{total} тактов" in summary
