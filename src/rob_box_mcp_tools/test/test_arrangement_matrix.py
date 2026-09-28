"""Тесты матрицы аранжировки «слой × 2-тактовый блок» (план §7.10, §7.6 п.10)."""

import pytest

from rob_box_mcp_tools.core.arrangement_matrix import (
    FULL,
    LOFI_FROOS_ORIGINAL,
    SECTION_TEMPLATES,
    ArrangementMatrix,
    cell_quarters,
    cycle_lane,
    original_cycle_blocks,
    parse_cell,
    parse_lane,
)
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando

LANES = ("kick", "clap", "hats", "perc", "bass", "lead", "pad")


class TestParsing:
    @pytest.mark.parametrize("cell,expected", [("x", 15), ("X", 15), (0, 0), ("0", 0), (7, 7), ("9", 9), ("15", 15)])
    def test_parse_cell_ok(self, cell, expected):
        assert parse_cell(cell) == expected

    @pytest.mark.parametrize("cell", [16, -1, "16", "y", "", "1.5", True, None, 2.0])
    def test_parse_cell_invalid(self, cell):
        with pytest.raises(ValueError):
            parse_cell(cell)

    def test_parse_lane_repetition(self):
        assert parse_lane("0 0 1 x x!4 7 x 9") == (0, 0, 1, 15, 15, 15, 15, 15, 7, 15, 9)

    def test_parse_lane_strudel_brackets(self):
        assert parse_lane("<0 1 x>") == (0, 1, 15)

    @pytest.mark.parametrize("spec", ["", "   ", "x!0", "x!", "x!a", "0 z", "0 16"])
    def test_parse_lane_invalid(self, spec):
        with pytest.raises(ValueError):
            parse_lane(spec)

    def test_error_message_is_russian(self):
        with pytest.raises(ValueError, match="Маска ячейки вне диапазона"):
            parse_cell(16)


class TestQuarters:
    def test_msb_is_first_quarter(self):
        assert cell_quarters(1) == (False, False, False, True)
        assert cell_quarters(8) == (True, False, False, False)
        assert cell_quarters(7) == (False, True, True, True)
        assert cell_quarters(9) == (True, False, False, True)
        assert cell_quarters(FULL) == (True, True, True, True)
        assert cell_quarters(0) == (False, False, False, False)


class TestMatrix:
    def test_unequal_lengths_rejected(self):
        with pytest.raises(ValueError, match="разной длины"):
            ArrangementMatrix.from_specs({"a": "x x", "b": "x"})

    def test_empty_and_bad_params_rejected(self):
        with pytest.raises(ValueError):
            ArrangementMatrix(lanes={})
        with pytest.raises(ValueError):
            ArrangementMatrix.from_specs({"a": "x"}, block_bars=0)

    def test_unknown_lane(self):
        m = ArrangementMatrix.from_specs({"a": "x"})
        with pytest.raises(ValueError, match="Нет слоя"):
            m.gate_segments("b")

    def test_sizes(self):
        m = ArrangementMatrix.from_specs({"a": "0 x 1"})
        assert m.n_blocks == 3
        assert m.total_beats == 24

    def test_segments_merge_and_sum(self):
        # 0 → 8 долей тишины; x → 8 звучит; 1 → 6 тишины + 2 звучит
        m = ArrangementMatrix.from_specs({"a": "0 x 1"})
        assert m.gate_segments("a") == [(0, 8), (1, 8), (0, 6), (1, 2)]

    def test_adjacent_equal_merged_across_blocks(self):
        # 1 (…#) + 8 (#…) + 0 → тишина 6, звук 2+2=4, тишина 6+8=14
        m = ArrangementMatrix.from_specs({"a": "1 8 0"})
        assert m.gate_segments("a") == [(0, 6), (1, 4), (0, 14)]

    def test_gate_var_exact(self):
        m = ArrangementMatrix.from_specs({"a": "0 x 1"})
        assert m.gate_var("a", 0.5) == "var([0, 0.5, 0, 0.5], [8, 8, 6, 2])"

    def test_gate_var_rounding_and_fractional_beats(self):
        m = ArrangementMatrix.from_specs({"a": "9"}, block_bars=1, beats_per_bar=3)
        assert m.gate_var("a", 0.12345) == "var([0.123, 0, 0.123], [0.75, 1.5, 0.75])"

    def test_gate_var_shortcuts(self):
        m = ArrangementMatrix.from_specs({"on": "x!3", "off": "0 0 0"})
        assert m.gate_var("on", 0.7) == "0.7"
        assert m.gate_var("off", 0.7) == "0"

    def test_gate_var_negative_level(self):
        m = ArrangementMatrix.from_specs({"a": "x"})
        with pytest.raises(ValueError):
            m.gate_var("a", -0.1)

    def test_active_blocks(self):
        m = ArrangementMatrix.from_specs({"a": "0 1 x 0 9"})
        assert m.active_blocks("a") == [1, 2, 4]

    def test_to_text(self):
        m = ArrangementMatrix.from_specs({"kick": "0 1 x 7 9", "pad": "x!5"})
        lines = m.to_text().splitlines()
        assert lines[1] == "kick | .... ...# #### .### | #..#"
        assert lines[2] == "pad  | #### #### #### #### | ####"

    def test_cycle_lane(self):
        assert cycle_lane((1, 2, 3), 7) == (1, 2, 3, 1, 2, 3, 1)
        assert cycle_lane((1, 2, 3), 2) == (1, 2)


class TestTemplates:
    @pytest.mark.parametrize("name", sorted(SECTION_TEMPLATES))
    def test_template_valid(self, name):
        m = ArrangementMatrix.from_specs(SECTION_TEMPLATES[name])
        assert set(m.lanes) == set(LANES)
        assert len({len(v) for v in m.lanes.values()}) == 1
        for lane in m.lanes:
            assert sum(b for _, b in m.gate_segments(lane)) == m.total_beats

    def test_dj_dave_32_is_32_bars(self):
        m = ArrangementMatrix.from_specs(SECTION_TEMPLATES["dj_dave_32"])
        assert m.n_blocks == 16 and m.n_blocks * m.block_bars == 32
        # раскрытие: хэты с блока 0, бас с 1, бочка — затакт в 1, целиком со 2
        assert m.active_blocks("hats")[0] == 0
        assert m.active_blocks("bass")[0] == 1
        assert m.lanes["kick"][1] == 1 and m.lanes["kick"][2] == FULL
        # пред-дроп: бочка выпадает перед дропом (блок 8)
        assert m.lanes["kick"][7] == 0 and m.lanes["kick"][6] in (12, 14)
        # дроп: всё звучит
        assert all(m.lanes[lane][8] == FULL for lane in LANES)
        # последний блок каждой фразы — брейк 7/9 хотя бы на одном слое
        for last in (3, 7, 11, 15):
            assert any(m.lanes[lane][last] in (7, 9) for lane in LANES)

    def test_lofi_literal_prefix(self):
        m = ArrangementMatrix.from_specs(SECTION_TEMPLATES["lofi_froos"])
        drums = parse_lane(LOFI_FROOS_ORIGINAL["drums"])
        guitar = parse_lane(LOFI_FROOS_ORIGINAL["guitar"])
        bass = parse_lane(LOFI_FROOS_ORIGINAL["bass"])
        assert (len(drums), len(guitar), len(bass)) == (22, 24, 28)
        assert m.n_blocks == 28
        assert m.lanes["kick"][:22] == drums
        assert m.lanes["kick"][22:] == drums[:6]  # Strudel зациклил бы слой
        assert m.lanes["lead"][:24] == guitar
        assert m.lanes["bass"] == bass
        assert original_cycle_blocks(LOFI_FROOS_ORIGINAL) == 1848


class TestSanitizerCompat:
    def test_gate_var_survives_sanitizer(self):
        m = ArrangementMatrix.from_specs(SECTION_TEMPLATES["dj_dave_32"])
        expr = m.gate_var("lead", 0.5)
        assert expr.startswith("var([")
        code = f"p1 >> pluck([0], dur=1, amp={expr})"
        result = sanitize_renando(code, 0.85)
        assert not result.quality_errors
        assert result.security_error is None or not result.security_error
        assert not result.slot_error
        assert result.code == code

    def test_amp_var_is_not_capped(self):
        # Фиксируем факт: amp=var(...) санитайзер НЕ капает (уровень 0.95 > 0.85).
        m = ArrangementMatrix.from_specs({"a": "0 x"})
        code = f"p1 >> pluck([0], dur=1, amp={m.gate_var('a', 0.95)})"
        assert sanitize_renando(code, 0.85).code == code
