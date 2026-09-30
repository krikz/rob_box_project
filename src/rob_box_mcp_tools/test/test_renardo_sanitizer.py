"""Тесты единого seam очистки Renardo-кода — ``sanitize_renando``.

Модуль ``core.renardo_sanitizer`` вынесен из ``MusicManager`` (архитектурный
обзор 2026-09-10, кандидат C). Контракт: один вызов прогоняет весь pipeline
(безопасность → музыкальный валидатор → перестановка слотов → pianovel→rhpiano
→ длина рисунка → кап amp) и возвращает структуру, из которой вызывающий
собирает тот же ответ тула, что и раньше.
"""

import pytest

from rob_box_mcp_tools.core.renardo_sanitizer import _cap_amp, sanitize_renando

MAX_AMP = 0.7


def test_valid_code_passes_and_is_unchanged():
    result = sanitize_renando("p1 >> pluck([0, 2, 4], dur=0.5, amp=0.4)", MAX_AMP)
    assert result.security_error is None
    assert result.quality_errors == ()
    assert result.slot_error is None
    assert result.code == "p1 >> pluck([0, 2, 4], dur=0.5, amp=0.4)"


def test_security_error_blocks_execution():
    result = sanitize_renando("import os; p1 >> pluck([0])", MAX_AMP)
    assert result.security_error is not None
    assert "Запрещённый токен" in result.security_error


def test_dunder_escape_is_blocked_by_ast_layer():
    result = sanitize_renando("f = (lambda: 0).__globals__", MAX_AMP)
    assert result.security_error is not None


def test_absolute_frequency_is_hard_quality_error():
    result = sanitize_renando("p1 >> pluck([0], freq=440)", MAX_AMP)
    assert result.quality_errors
    assert any("Абсолютные частоты" in e for e in result.quality_errors)


def test_amp_and_oct_are_capped():
    result = sanitize_renando("p1 >> pluck([0, 2, 4], dur=0.5, amp=0.9, oct=9)", MAX_AMP)
    assert result.code == "p1 >> pluck([0, 2, 4], dur=0.5, amp=0.7, oct=5)"


def test_pianovel_is_rewritten_to_rhpiano():
    result = sanitize_renando("p1 >> pianovel([0, 2, 4], dur=0.5)", MAX_AMP)
    assert "rhpiano" in result.code
    assert "pianovel" not in result.code


def test_illegal_p4_slot_is_remapped_to_a_free_p_slot():
    result = sanitize_renando(
        "p1 >> blip([0, 2, 4], dur=0.25, amp=0.4)\n"
        'p4 >> play("..o...o.", amp=0.2)\n',
        MAX_AMP,
    )
    assert result.slot_error is None
    assert "p4" not in result.code
    assert 'd1 >> play("..o...o."' in result.code


def test_pattern_length_is_normalized():
    # 9 шагов с одной хвостовой паузой → срезается до 8 (не добивается до 16).
    result = sanitize_renando('d1 >> play("X..o.X.o.")', MAX_AMP)
    assert 'play("X..o.X.o")' in result.code


def test_soft_warnings_survive_to_the_result():
    # Без dur= — мягкое предупреждение, но выполнение не блокируется.
    result = sanitize_renando("p1 >> pluck([0, 2, 4])", MAX_AMP)
    assert result.warnings
    assert result.security_error is None
    assert result.quality_errors == ()


# ---------------------------------------------------------------------------
# Live-инцидент 21.09.2026 — compose_music(bass_synth='supersaw') принимал
# несуществующий синт, execute_code() рапортовал success=True, а бас молча
# пропадал (реальная ошибка scsynth видна только в docker logs supercollider).
# ---------------------------------------------------------------------------


KNOWN_SYNTHS = frozenset({"pluck", "strings", "wobblebass", "supersawlead", "blip"})


def test_known_synths_none_disables_the_check_entirely():
    """Обратная совместимость: без known_synths поведение не меняется."""
    result = sanitize_renando("p1 >> totallymadeupname([0, 2, 4], dur=0.5)", MAX_AMP)
    assert result.quality_errors == ()
    assert result.security_error is None


def test_unknown_synth_is_a_hard_quality_error_when_known_synths_given():
    result = sanitize_renando(
        "p1 >> supersaw([38, 43, 39], dur=[2, 2, 2], amp=0.4)",
        MAX_AMP,
        known_synths=KNOWN_SYNTHS,
    )
    assert result.quality_errors
    assert any("supersaw" in e for e in result.quality_errors)


def test_unknown_synth_error_suggests_the_closest_real_name():
    """RAW-инцидент 21.09.2026: bass_synth='supersaw' → должен предложить 'supersawlead'."""
    result = sanitize_renando(
        "p1 >> supersaw([38, 43, 39], dur=[2, 2, 2], amp=0.4)",
        MAX_AMP,
        known_synths=KNOWN_SYNTHS,
    )
    assert any("supersawlead" in e for e in result.quality_errors)


def test_known_synth_passes_when_known_synths_given():
    result = sanitize_renando(
        "p1 >> supersawlead([0, 2, 4], dur=0.5, amp=0.4)",
        MAX_AMP,
        known_synths=KNOWN_SYNTHS,
    )
    assert result.quality_errors == ()


def test_play_sample_pattern_is_never_validated_as_a_synth():
    """play() — сэмплер по символам, не SynthDef; не должен ловиться проверкой."""
    result = sanitize_renando(
        'd1 >> play("x-o-", amp=0.4)',
        MAX_AMP,
        known_synths=KNOWN_SYNTHS,
    )
    assert result.quality_errors == ()


def test_unknown_synth_check_is_case_insensitive():
    result = sanitize_renando(
        "p1 >> SUPERSAWLEAD([0, 2, 4], dur=0.5, amp=0.4)",
        MAX_AMP,
        known_synths=KNOWN_SYNTHS,
    )
    assert result.quality_errors == ()


# ---------------------------------------------------------------------------
# Issue #2838 — подсказка «Возможно, имелся в виду» предложила 'sine',
# которого не было в scsynth (235 × "SynthDef sine not found").
# Подсказка обязана браться ТОЛЬКО из переданного known_synths —
# множества, реально подтверждённого сервером
# (см. MusicManager.known_synth_names).
# ---------------------------------------------------------------------------


SERVER_CONFIRMED = frozenset(
    {"epiano", "sinepad", "pianovel", "pluck", "blip"}
)


def test_suggestion_for_seepline_never_proposes_a_synth_missing_on_server():
    """RAW 23.09.2026: 'seepline' → подсказали 'sine'. Без 'sine' в
    подтверждённом множестве подсказка — ближайшее ИЗ него имя."""
    result = sanitize_renando(
        "p4 >> seepline([3, 5, 7, 5], dur=0.5, amp=0.18)",
        MAX_AMP,
        known_synths=SERVER_CONFIRMED,
    )
    assert len(result.quality_errors) == 1
    error = result.quality_errors[0]
    assert "'sine'" not in error
    suggested = error.split("имелся в виду ")[1].split("?")[0].strip("'")
    assert suggested in SERVER_CONFIRMED


def test_synth_missing_on_server_is_rejected_and_not_suggested():
    """Второй ход того же инцидента: LLM взяла подсказанный 'sine'. Если его
    нет на сервере — это HARD error, а не «успешно» и 235 отказов scsynth."""
    result = sanitize_renando(
        "d2 >> sine([3, 5, 7, 5], dur=0.5, oct=5, amp=0.18)",
        MAX_AMP,
        known_synths=SERVER_CONFIRMED,
    )
    errors = result.quality_errors
    assert errors
    assert any("Синта 'sine' не существует" in e for e in errors)
    assert not any("имелся в виду 'sine'" in e for e in errors)


# ---------------------------------------------------------------------------
# Issue #3173 — кап amplify=var(...) не трогает длительности
# ---------------------------------------------------------------------------


def test_cap_amplify_var_keeps_durations_untouched():
    """Репро из issue #3173: 0.875 — длительность шага, не уровень."""
    out = _cap_amp("p1 >> bass([0], amplify=var([1, 0.3], [0.875, 0.125]))", 0.85)
    assert out == "p1 >> bass([0], amplify=var([0.85, 0.3], [0.875, 0.125]))"


def test_cap_amplify_var_keeps_long_duck_cycle_of_the_arranger():
    """Огибающая дакинга аранжировщика: сумма длительностей = 4 доли."""
    code = (
        "p1 >> bass([0], amplify=var([0.3, 1, 0.3, 1, 0.3, 1, 0.3, 1], "
        "[0.125, 0.875, 0.125, 0.875, 0.125, 0.875, 0.125, 0.875]))"
    )
    out = _cap_amp(code, 0.85)
    assert "[0.125, 0.875, 0.125, 0.875, 0.125, 0.875, 0.125, 0.875]" in out
    assert "var([0.3, 0.85, 0.3, 0.85, 0.3, 0.85, 0.3, 0.85]," in out


def test_cap_amplify_var_keeps_scalar_duration():
    """``var(levels, 4)`` — скалярная длительность тоже не капается."""
    out = _cap_amp("p1 >> bass([0], amplify=var([1, 0.3], 4))", 0.85)
    assert out == "p1 >> bass([0], amplify=var([0.85, 0.3], 4))"


def test_cap_amplify_var_single_list_still_capped():
    out = _cap_amp("d1 >> play('X', amplify=var([1, 0.3]))", 0.7)
    assert out == "d1 >> play('X', amplify=var([0.7, 0.3]))"


def test_cap_amplify_plain_number_still_capped():
    out = _cap_amp("d1 >> play('X', amplify=0.9)", 0.7)
    assert out == "d1 >> play('X', amplify=0.7)"


@pytest.mark.parametrize(
    "code, expected",
    [
        # Вложенный var в уровнях: регэксп захватывает до первой ``)``,
        # верхнеуровневой запятой в захвате нет — капается всё, как и раньше.
        (
            "p1 >> bass([0], amplify=var([1, var([0.9, 0.2])], [2, 2]))",
            "p1 >> bass([0], amplify=var([0.85, var([0.85, 0.2])], [2, 2]))",
        ),
    ],
)
def test_cap_amplify_nested_var_behaviour_is_preserved(code, expected):
    assert _cap_amp(code, 0.85) == expected
