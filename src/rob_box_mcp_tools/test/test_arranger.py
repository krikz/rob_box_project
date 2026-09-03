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
    BARS_PER_CHORD,
    BEATS_PER_BAR,
    BPM_RANGE,
    DEFAULT_FORM,
    FORMS,
    OCTAVE_STEP,
    ROLE_PROFILE,
    ArrangementError,
    CompositionSpec,
    Layer,
    form_duration_seconds,
    form_summary,
    render,
    resolve_form,
    spec_from_flat,
)


def _all_degree_values(code: str, player: str):
    """Every scale-degree number ``player`` actually plays in the rendered
    code — from a plain degree list, or, if the role develops material,
    every variant packed into its ``{player}_motif = Pvar(...)`` line.
    """
    motif_match = re.search(
        rf"^{re.escape(player)}_motif = Pvar\((\[\[.*?\]\])", code, re.M
    )
    if motif_match:
        groups = re.findall(r"\[([^\[\]]+)\]", motif_match.group(1))
        return [float(v) for g in groups for v in g.split(",")]
    line = next(l for l in code.splitlines() if l.startswith(f"{player} >>"))
    # Пэд рендерится аккордом — PGroup в круглых скобках; остальные роли
    # играют последовательность в квадратных.
    plain = re.search(r"\(\[([^\]]+)\]|\(\(([^)]+)\)", line)
    return [float(v) for v in (plain.group(1) or plain.group(2)).split(",")]


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
        """RC2: бас, пэд и лид в соседних октавах = каша вместо аранжировки.

        Проверяет только заявленный ``oct=``. Этого НЕ достаточно, чтобы
        поймать регистровый разъезд — код может честно писать ``oct=4`` и
        всё равно играть на две октавы выше через сдвинутые ступени лада
        (ровно так проскочила регрессия #1805.1, см. тест ниже).
        """
        code = render(_spec())
        octaves = {
            line.split(" >>")[0]: int(re.search(r"oct=(\d+)", line).group(1))
            for line in code.splitlines()
            if ">>" in line and "oct=" in line
        }
        assert octaves["p1"] < octaves["p3"] < octaves["p2"], (
            f"бас/пэд/лид должны быть разведены по регистрам, получено {octaves}"
        )

    def test_material_development_never_widens_a_roles_own_register(self):
        """RC5.1 — regression, post-review fix for #1805.

        Первая версия материала по секциям применяла транспозицию и
        инверсию ко всем мелодическим ролям без учёта того, что ``degrees``
        — это ступени лада, а не полутона: открытая раскладка пэда
        ``(0, 4, 7, 11)`` (уже больше октавы сама по себе) после инверсии
        и транспозиции превращалась в
        ``Pvar([[0,4,7,11],[2,6,9,13],[0,-4,-7,-11], ...])`` — ступень 13
        при oct=4 звучит почти на две октавы выше лида (oct=5), ступень
        -11 — ниже баса (oct=3) и ниже HPF мастер-шины. ``oct=`` при этом
        не менялся, поэтому ``test_roles_are_spread_across_octaves`` этого
        не ловил.

        Правило теперь: лид — единственная роль, которую транспонируют и
        инвертируют (это тема, ей и положено развитие); бас может менять
        только ПОРЯДОК своих нот (ретроград); пэд держит гармонию и
        material вообще не меняет. Тест проверяет фактические ступени
        (учитывая все варианты внутри ``Pvar``, если он есть) — ни одна
        роль не должна выйти за пределы диапазона, который сама же и
        получила от LLM.
        """
        code = render(
            _spec(
                layers=(
                    Layer(role="bass", synth="jbass", degrees=(0, 0, 5, 3), dur=1),
                    Layer(role="lead", synth="rhpiano", degrees=(0, 2, 4, 7), dur=0.5),
                    Layer(role="pad", synth="warmpad", degrees=(0, 4, 7, 11), dur=4),
                )
            )
        )
        for role, player, original in (
            ("bass", "p1", (0, 0, 5, 3)),
            ("lead", "p2", (0, 2, 4, 7)),
            ("pad", "p3", (0, 4, 7, 11)),
        ):
            values = _all_degree_values(code, player)
            assert min(values) >= min(original), (
                f"{role}: ступень {min(values)} ниже собственного диапазона "
                f"{original} — слой провалился в чужой регистр"
            )
            assert max(values) <= max(original), (
                f"{role}: ступень {max(values)} выше собственного диапазона "
                f"{original} — слой залез в чужой регистр"
            )

    def test_roles_occupy_non_decreasing_absolute_registers(self):
        """Фактическая высота (``oct + ступень/OCTAVE_STEP``), не только
        ``oct=``: во всех вариантах материала бас не должен подниматься
        выше пэда, а пэд — выше лида. Касание границы (общий тон — верх
        пэда совпадает с низом лида) это нормальное голосоведение, а не
        баг; настоящая бага — это низ одной роли ВЫШЕ верха соседней.
        """
        code = render(_spec())
        bounds = {}
        for role, player in (("bass", "p1"), ("pad", "p3"), ("lead", "p2")):
            values = _all_degree_values(code, player)
            octave = int(
                re.search(rf"^{player} >>.*?oct=(\d+)", code, re.M).group(1)
            )
            bounds[role] = (
                octave + min(values) / OCTAVE_STEP,
                octave + max(values) / OCTAVE_STEP,
            )
        assert bounds["bass"][1] <= bounds["pad"][0], bounds
        assert bounds["pad"][1] <= bounds["lead"][0], bounds

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
    def test_progression_changes_on_a_four_bar_grid(self):
        """Аккорд держится ровно 4 такта — квадрат, а не «одна прогрессия
        на всю форму».

        Прежний контракт растягивал прогрессию на форму целиком: на
        buildup (76 тактов) шаг выходил 76 БИТОВ, то есть 19 тактов, и
        тоника менялась посреди фразы, посреди секции, под держащейся
        нотой пэда. Смена гармонии, не попадающая в сетку, слышится не
        как гармония, а как сбой (живая жалоба «какофония», 02.09).
        ``var`` в Renardo зацикливается сам, поэтому прогрессия просто
        прокручивается по кругу до конца формы.
        """
        code = render(_spec(progression=(0, 0, 4, 3)))
        step = int(re.search(r"Root\.default = var\(\[.*?\], (\d+)\)", code).group(1))
        assert step == BARS_PER_CHORD * BEATS_PER_BAR

    def test_progression_is_relative_to_the_requested_root(self):
        """``root`` не должен теряться при наличии прогрессии.

        ``Root.default`` в Renardo — ХРОМАТИЧЕСКИЙ номер ноты (Root.py,
        ``CHROMATIC_NOTES``), поэтому прежний
        ``Root.default = var([0, 0, 5, 3], ...)`` играл в до независимо от
        запрошенной тоники: 91 из 99 живых вызовов шли с прогрессией, и
        весь сет звучал в одном тональном центре.
        """
        code = render(_spec(root="A", scale="minor", progression=(0, 0, 4, 3)))
        # A = 9 полутонов; в миноре V ступень = +7, IV = +5.
        assert "Root.default = var([9, 9, 16, 14]," in code

    def test_progression_degrees_are_scale_steps_not_semitones(self):
        """И схема тула, и промпт обещают «ступени лада» — лад и должен
        решать, на сколько полутонов уедет тоника."""
        minor = render(_spec(root="C", scale="minor", progression=(0, 2)))
        major = render(_spec(root="C", scale="major", progression=(0, 2)))
        assert "Root.default = var([0, 3]," in minor  # III ступень минора
        assert "Root.default = var([0, 4]," in major  # III ступень мажора

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


class TestFormDurationSeconds:
    """Issue #1812: длительность одного прохода формы должна совпадать с
    ``Clock.future(total_beats, Clock.clear)``, который ``render()`` ставит
    для ``repeat=False`` — иначе watchdog и реальный конец трека разъедутся.
    """

    def test_matches_the_clock_future_deadline_render_schedules(self):
        """Тот же ``total_beats``, что и в ``render()``, переведённый в секунды."""
        bpm = 104.0
        total_beats = sum(bars for _n, bars, _i in FORMS["arc"]) * BEATS_PER_BAR
        expected = total_beats * 60.0 / bpm
        assert form_duration_seconds("arc", bpm) == pytest.approx(expected)

    def test_faster_tempo_gives_a_shorter_duration(self):
        assert form_duration_seconds("arc", 160.0) < form_duration_seconds("arc", 80.0)

    def test_unknown_form_falls_back_to_default_like_render_does(self):
        assert form_duration_seconds("bogus_form", 120.0) == pytest.approx(
            form_duration_seconds(DEFAULT_FORM, 120.0)
        )

    def test_bpm_is_clamped_like_render(self):
        """Абсурдный bpm не даёт нулевую/бесконечную длительность."""
        assert form_duration_seconds("arc", 1.0) == pytest.approx(
            form_duration_seconds("arc", BPM_RANGE[0])
        )
        assert form_duration_seconds("arc", 9000.0) == pytest.approx(
            form_duration_seconds("arc", BPM_RANGE[1])
        )

    def test_positive_for_every_known_form(self):
        for name in FORMS:
            assert form_duration_seconds(name, 120.0) > 0.0


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

    def test_nine_step_drums_trimmed_to_eight_not_padded_to_sixteen(self):
        """Хвостовые паузы снимаются, а не добиваются до удвоенного такта.

        🔴 live 01.09: первая версия давала "X..X.o..." + 7 точек = 16, и
        грув начинал бить вдвое реже задуманного, а половину такта занимала
        тишина. Девятый символ здесь — пауза, её отбрасывание даёт ровно 8.
        """
        spec = self._flat(drums="X..X.o...")  # 9 шагов, живой инцидент
        drums = next(l for l in spec.layers if l.role == "drums")
        assert drums.pattern == "X..X.o.."
        assert len(drums.pattern) == 8

    def test_padding_never_adds_a_sounding_hit(self):
        """``-`` — реальный сэмпл ("hyphen", renardo_gatherer/collections.py;
        каталог samples/0_foxdot_default/_/hyphen существует на роботе), а
        не пауза — проверено вживую. Добивка не должна добавлять НИ ОДНОГО
        звучащего символа, кроме точек ``.`` (для них сэмпл-каталога нет ни
        в одном паке).
        """
        original = "X..o.X.o."  # 9 шагов, диско трек 6, live 31.08
        spec = self._flat(drums=original)
        drums = next(l for l in spec.layers if l.role == "drums")
        sounding = lambda p: [c for c in p if c != "."]
        assert sounding(drums.pattern) == sounding(original), (
            "звучащие символы обязаны сохраниться один в один"
        )
        assert "-" not in drums.pattern

    def test_power_of_two_pattern_is_untouched(self):
        spec = self._flat(hats="--.-")  # уже 4 шага — трогать нечего
        hats = next(l for l in spec.layers if l.role == "hats")
        assert hats.pattern == "--.-"

    def test_eighteen_step_perc_trimmed_to_sixteen(self):
        """Две хвостовые паузы снимаются — получается ровно такт, а не два."""
        spec = self._flat(perc=".......O.......O..")  # 18 шагов, live 31.08
        perc = next(l for l in spec.layers if l.role == "perc")
        assert perc.pattern == ".......O.......O"
        assert len(perc.pattern) == 16

    def test_trailing_rests_are_not_stripped_past_a_power_of_two(self):
        """'X.....' — это «бочка раз в шесть шагов».

        Снять ВСЕ хвостовые паузы значило бы получить 'X' и заставить бочку
        бить на каждом шаге — это громче и быстрее задуманного, то есть
        хуже исходной ошибки. Останавливаемся на первой степени двойки.
        """
        spec = self._flat(drums="X.....")
        drums = next(l for l in spec.layers if l.role == "drums")
        assert drums.pattern == "X..."
        assert drums.pattern.count("X") == 1

    def test_pattern_without_trailing_rests_is_padded_as_before(self):
        """'-.---' звучит до последнего символа — резать нечего, добиваем."""
        spec = self._flat(hats="-.---")
        hats = next(l for l in spec.layers if l.role == "hats")
        assert hats.pattern == "-.---..."
        assert len(hats.pattern) == 8

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

    @pytest.mark.parametrize("form", sorted(FORMS))
    def test_output_with_swing_and_motif_pvar_passes_both_guards(self, manager, form):
        """#1805/#1806: the new ``{role}_motif = Pvar(...)`` line and the
        ``dur=var(...)`` argument must not confuse the ``dur=`` regex in
        ``_validate_music_code`` (it stops at the first ``)`` in the args,
        which a naively-inlined ``Pvar(...)`` would break — see the comment
        in ``_render_layer``)."""
        code = render(_spec(form=form, swing=0.15))
        is_safe, error = manager._filter_code(code)
        assert is_safe, f"форма {form}: {error}"
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


class TestMotifDevelopment:
    """#1805: sections used to change only ``amp`` — the melody played the
    same seven notes for three minutes, only louder or quieter. A role that
    passes through more than one distinct dramatic moment must now state a
    transformation of its own motif per moment, via ``Pvar`` — never a note
    the LLM didn't give it.
    """

    def test_lead_material_varies_across_the_form(self):
        """The exact complaint: lead is audible in main/break/peak with three
        different intensities, so it must play three different ideas."""
        code = render(_spec())
        assert "p2_motif = Pvar(" in code
        assert "p2 >> blip(p2_motif," in code

    def test_motif_pvar_durations_span_exactly_one_form(self):
        """Same invariant as the amp envelope: the Pvar loop must not drift
        relative to the rest of the form."""
        code = render(_spec())
        durs = re.search(r"p2_motif = Pvar\(\[\[.*?\]\], \[([^\]]*)\]\)", code).group(1)
        total = sum(int(x) for x in durs.split(","))
        expected = sum(bars for _n, bars, _i in FORMS["arc"]) * BEATS_PER_BAR
        assert total == expected

    def test_first_variant_states_the_motif_verbatim(self):
        """The theme has to be stated plainly before it can be varied —
        otherwise nothing establishes what is being developed."""
        code = render(_spec())
        nested = re.search(r"p2_motif = Pvar\((\[\[.*?\]\])", code).group(1)
        first = re.match(r"\[\[([^\]]*)\]", nested).group(1)
        assert first == "0, 2, 4, 7, 4, 2"  # исходный мотив лида из _spec()

    def test_every_variant_is_the_same_length_as_the_original(self):
        """A transform re-shapes the given motif; it never invents or drops
        notes."""
        code = render(_spec())
        nested = re.search(r"p2_motif = Pvar\((\[\[.*?\]\])", code).group(1)
        groups = re.findall(r"\[([^\[\]]+)\]", nested)
        assert len(groups) > 1, "меньше двух вариантов — развития нет"
        for group in groups:
            assert len(group.split(",")) == 6  # len(lead.degrees) == 6

    def test_role_audible_in_only_one_section_skips_the_pvar_wrapper(self):
        """Nothing to develop against — a single appearance renders as a
        plain degree list, exactly like before #1805."""
        code = render(
            _spec(
                form="ambient",
                layers=(Layer(role="bass", synth="dub", degrees=(0, 0, 4, 0), dur=1),),
            )
        )
        assert "p1_motif" not in code
        assert "p1 >> dub([0, 0, 4, 0]," in code

class TestNoteDensity:
    """#1806: a constant ``dur`` for the whole piece reads as mechanical
    regardless of genre. Density now tracks the section's own intensity —
    busier where the role is loudest, sparser where it is quiet."""

    def test_lead_dur_thickens_and_thins_with_intensity(self):
        code = render(_spec())
        lead = next(l for l in code.splitlines() if l.startswith("p2 >>"))
        match = re.search(r"dur=var\(\[([^\]]*)\], \[([^\]]*)\]\)", lead)
        assert match, f"ожидался var() на dur, получено: {lead}"
        values = [float(v) for v in match.group(1).split(",")]
        assert len(set(values)) > 1, "плотность нот не меняется — та же жалоба #1806"
        # peak — самая громкая секция лида — обязана быть самой плотной.
        assert min(values) < 0.5  # base dur лида в _spec() == 0.5

    def test_dur_var_durations_span_exactly_one_form(self):
        code = render(_spec())
        lead = next(l for l in code.splitlines() if l.startswith("p2 >>"))
        durs = re.search(r"dur=var\(\[[^\]]*\], \[([^\]]*)\]\)", lead).group(1)
        total = sum(int(x) for x in durs.split(","))
        expected = sum(bars for _n, bars, _i in FORMS["arc"]) * BEATS_PER_BAR
        assert total == expected

    def test_role_with_no_change_in_intensity_keeps_a_scalar_dur(self):
        """No development to reflect -> no var() noise in the code.

        Unlike the Pvar-motif case, being audible everywhere is not enough
        by itself — a role that is silent in some sections still gets a
        denser dur in its loud ones. It takes the SAME intensity in every
        audible section (no silence, no loud/quiet contrast) for density to
        have nothing to track.
        """
        original = dict(FORMS)
        FORMS["_test_flat_intensity"] = [
            ("a", 4, {"lead": 0.5}),
            ("b", 8, {"lead": 0.5}),
        ]
        try:
            code = render(
                _spec(
                    form="_test_flat_intensity",
                    layers=(Layer(role="lead", synth="blip", degrees=(0, 2, 4), dur=0.5),),
                )
            )
        finally:
            FORMS.clear()
            FORMS.update(original)
        lead = next(l for l in code.splitlines() if l.startswith("p2 >>"))
        assert "dur=0.5," in lead
        assert "var(" not in lead.split("dur=")[1].split(",")[0]


class TestSwing:
    """#1806: even eighth notes never read as jazz/blues/shuffle no matter
    the instrument choice — ``Clock.swing()`` is the one-line fix already
    proven to work in Renardo (``renardo_lib/TempoClock.py:290``)."""

    def test_swing_is_off_by_default(self):
        assert "Clock.swing" not in render(_spec())

    def test_swing_emits_a_single_declarative_line(self):
        code = render(_spec(swing=0.15))
        assert "Clock.swing(0.15)" in code

    def test_swing_is_clamped_to_a_musically_sane_range(self):
        assert "Clock.swing(0.3)" in render(_spec(swing=5))
        assert "Clock.swing" not in render(_spec(swing=-2))

    def test_spec_from_flat_threads_swing_through(self):
        base = dict(
            bpm=120, root="C", scale="minor",
            bass_synth="dub", bass_notes="0, 3",
        )
        assert spec_from_flat(**base).swing == 0.0
        spec = spec_from_flat(**base, swing=0.12)
        assert spec.swing == 0.12
        assert "Clock.swing(0.12)" in render(spec)
