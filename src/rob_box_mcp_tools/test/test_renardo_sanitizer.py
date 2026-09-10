"""Тесты единого seam очистки Renardo-кода — ``sanitize_renando``.

Модуль ``core.renardo_sanitizer`` вынесен из ``MusicManager`` (архитектурный
обзор 2026-09-10, кандидат C). Контракт: один вызов прогоняет весь pipeline
(безопасность → музыкальный валидатор → перестановка слотов → pianovel→rhpiano
→ длина рисунка → кап amp) и возвращает структуру, из которой вызывающий
собирает тот же ответ тула, что и раньше.
"""

from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando

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
