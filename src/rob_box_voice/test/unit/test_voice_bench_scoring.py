"""Правила оценки бенчмарка — пришпилены тестом, а не соглашением.

Бенчмарк нужен, чтобы доказывать «стало лучше» цифрой (ADR-0018). Значит
сама шкала обязана быть проверяемой: если завтра кто-то поменяет вес или
условие, это должно быть видно падением теста, а не тихим сдвигом всех
исторических цифр.

Правила и обоснование — `docs/design/voice-quality-benchmark.md`.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_BENCH = Path(__file__).resolve().parents[4] / "scripts" / "voice_bench"
sys.path.insert(0, str(_BENCH))

from score import (  # noqa: E402
    PASS_THRESHOLD,
    WEIGHTS,
    pass_hat_k,
    report,
    score_turn,
)

_CASE = {
    "id": "composer-beat",
    "domain": "composer",
    "tools_any": ["execute_music_code", "compose_music"],
    "tools_none": ["load_track"],
}


def _turn(**over):
    base = {
        "case_id": "composer-beat",
        "repeat": 1,
        "spoken": "Ок, играю бит",
        "tools": ["execute_music_code"],
        "llm_requests": 2,
        "truncated": 0,
        "retry_no_music": 0,
        "provider_fallback": 0,
        "error": None,
    }
    base.update(over)
    return base


def test_weights_sum_to_one() -> None:
    """Иначе балл перестаёт быть долей и сравнивать прогоны нельзя."""
    assert abs(sum(WEIGHTS.values()) - 1.0) < 1e-9


def test_perfect_turn_scores_one() -> None:
    assert score_turn(_CASE, _turn()).score == 1.0


# ── главный дефект, ради которого всё затевалось ─────────────────────────


def test_words_without_any_tool_is_the_harshest_penalty() -> None:
    """«Красиво описал бит, который не играет» — провал хода, не стиль.

    Это инцидент из dialog_core.py:681: модель отвечала фразой и ни одним
    вызовом, продолжая few-shot из истории.
    """
    result = score_turn(_CASE, _turn(tools=[], spoken="Бит уже качает, лови грув!"))
    assert result.score == pytest.approx(1.0 - WEIGHTS["tool_choice"])
    assert any("НИ ОДНОГО вызова" in f for f in result.failures)


def test_wrong_tool_scores_between_nothing_and_right() -> None:
    """Домен нащупан, действие не то — это хуже правильного, но лучше молчания."""
    nothing = score_turn(_CASE, _turn(tools=[])).score
    wrong = score_turn(_CASE, _turn(tools=["get_music_state"])).score
    right = score_turn(_CASE, _turn()).score
    assert nothing < wrong < right


def test_forbidden_tool_costs_half_of_its_criterion() -> None:
    result = score_turn(_CASE, _turn(tools=["execute_music_code", "load_track"]))
    assert result.score == pytest.approx(1.0 - WEIGHTS["prohibitions"] * 0.5)


# ── форма ответа ─────────────────────────────────────────────────────────


def test_degenerate_text_is_penalised() -> None:
    """Живой инцидент 02.09: `[e~[` уехал в Telegram и заразил историю."""
    result = score_turn(_CASE, _turn(spoken="[e~["))
    assert result.score < 1.0
    assert any("вырожденный" in f for f in result.failures)


def test_normal_short_answer_is_not_degenerate() -> None:
    """«Ок.» — законный короткий accept, а не мусор."""
    assert score_turn(_CASE, _turn(spoken="Ок.")).score == 1.0


def test_code_in_speech_is_penalised() -> None:
    """Renardo-код в речи робот зачитает вслух вместо исполнения."""
    result = score_turn(_CASE, _turn(spoken="Играю: p1 >> blip([0,2,4], dur=0.5)"))
    assert any("код в речи" in f for f in result.failures)


def test_long_speech_is_penalised_but_not_fatal() -> None:
    result = score_turn(_CASE, _turn(spoken="а" * 400))
    assert 0.8 < result.score < 1.0


# ── эффективность и устойчивость ─────────────────────────────────────────


def test_more_iterations_score_less() -> None:
    scores = [score_turn(_CASE, _turn(llm_requests=n)).score for n in (2, 3, 5, 8)]
    assert scores == sorted(scores, reverse=True)
    assert scores[0] > scores[-1]


def test_retries_and_fallbacks_reduce_stability() -> None:
    clean = score_turn(_CASE, _turn()).score
    shaky = score_turn(_CASE, _turn(truncated=1, provider_fallback=1)).score
    assert shaky < clean


def test_turn_error_is_visible_in_score() -> None:
    assert score_turn(_CASE, _turn(error="TIMEOUT")).score < 1.0


# ── надёжность по повторам (pass^k из τ-bench) ───────────────────────────


def test_pass_hat_k_requires_every_repeat_to_pass() -> None:
    """Один удачный прогон из трёх — не «33% качества», а ненадёжный робот."""
    good = [score_turn(_CASE, _turn(repeat=i)) for i in (1, 2, 3)]
    assert pass_hat_k(good) == 1.0

    flaky = good[:2] + [score_turn(_CASE, _turn(repeat=3, tools=[]))]
    assert pass_hat_k(flaky) == 0.0


def test_pass_hat_k_counts_cases_not_turns() -> None:
    other = dict(_CASE, id="player-library", tools_any=["gen_list_library"])
    results = [
        score_turn(_CASE, _turn(repeat=1)),
        score_turn(_CASE, _turn(repeat=2)),
        score_turn(other, _turn(repeat=1, case_id="player-library", tools=[])),
    ]
    assert pass_hat_k(results) == 0.5


def test_threshold_is_below_a_single_forbidden_call() -> None:
    """Порог обязан пропускать ход с одной мелкой придиркой, но не с грубой.

    Иначе pass^k либо всегда 1.0, либо всегда 0.0 и ничего не различает.
    """
    minor = score_turn(_CASE, _turn(llm_requests=3)).score
    gross = score_turn(_CASE, _turn(tools=[])).score
    assert gross < PASS_THRESHOLD <= minor


# ── отчёт целиком ────────────────────────────────────────────────────────


def test_report_aggregates_by_domain_and_ignores_unknown_cases() -> None:
    run = {
        "config": {"label": "skills=on"},
        "turns": [
            _turn(repeat=1),
            _turn(repeat=2, tools=[]),
            {"case_id": "чего-то-не-было", "repeat": 1, "tools": []},
        ],
    }
    rep = report([_CASE], run)
    assert rep["turns"] == 2
    assert set(rep["by_domain"]) == {"composer"}
    assert 0.0 < rep["mean"] < 1.0
    assert rep["config"]["label"] == "skills=on"


# ── корреляция реплики и хода в раннере ──────────────────────────────────


def _runner():
    """Раннер импортируется отдельно: он тянет rclpy только внутри Speaker."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "voice_bench_runner", _BENCH / "run_bench.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write(path: Path, *lines: str) -> None:
    path.write_text(chr(10).join(lines) + chr(10), encoding="utf-8")


def _append(path: Path, *lines: str) -> None:
    with path.open("a", encoding="utf-8") as fh:
        fh.write(chr(10).join(lines) + chr(10))


def test_key_strips_the_wake_word() -> None:
    """Нода печатает user_input уже без wake-word — ключ обязан совпасть."""
    rb = _runner()
    assert rb._key_of("робот сыграй бит") == "сыграй бит"
    assert rb._key_of("какие точки ты знаешь") == "какие точки ты знаешь"


def test_collect_takes_its_own_turn_not_the_previous_one(tmp_path) -> None:
    """Регрессия первого прогона: ответы сдвинулись на кейс.

    В логе сначала лежит хвост ПРЕДЫДУЩЕГО хода, потом наша реплика и наш
    ответ. Раннер обязан вернуть наш, а не первый попавшийся.
    """
    rb = _runner()
    log = tmp_path / "node.log"
    _write(
        log,
        "[dialogue_node]: process_input returned: spoken='ЧУЖОЙ ОТВЕТ'[:60] "
        "tools=['set_dj_mode'] finish_reason='stop'",
        "[dialogue_node]: calling process_input: user_input='[Speaker:unknown] это кухня'",
        "[dialogue_node]: process_input returned: spoken='Запомнила кухню.'[:60] "
        "tools=['save_waypoint'] finish_reason='stop'",
    )
    turn = rb.collect(log, 0, timeout=1.0, phrase="робот это кухня")
    assert turn["spoken"] == "Запомнила кухню."
    assert turn["tools"] == ["save_waypoint"]


def test_collect_reports_undelivered_phrase(tmp_path) -> None:
    """Реплика не долетела до ноды — это не «робот промолчал», а обрыв связи."""
    rb = _runner()
    log = tmp_path / "node.log"
    _write(log, "[dialogue_node]: тишина")
    turn = rb.collect(log, 0, timeout=0.5, phrase="робот это кухня")
    assert turn["error"] == "NOT_DELIVERED"


def test_collect_reads_from_byte_offset(tmp_path) -> None:
    """Смещение — в байтах: текстовый seek уводил чтение в никуда.

    Кириллица занимает по два байта на символ, поэтому подмена байтового
    смещения символьным уезжает в середину строки и молча возвращает
    пустоту — ровно так первый прогон бенчмарка собрал нули.
    """
    rb = _runner()
    log = tmp_path / "node.log"
    _write(log, "старое: кириллица занимает два байта на символ")
    offset = log.stat().st_size
    _append(
        log,
        "calling process_input: user_input='[Speaker:unknown] это кухня'",
        "process_input returned: spoken='Ок'[:60] tools=['save_waypoint'] error=None",
    )
    turn = rb.collect(log, offset, timeout=1.0, phrase="робот это кухня")
    assert turn["tools"] == ["save_waypoint"]
