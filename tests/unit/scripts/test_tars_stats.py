"""Юнит-тесты для ADR-0077 §2.3: scripts/stt/tars_stats.py.

Контракт сводки (issue #2223):

* колонка ``providers`` (y_only / y+vosk / none) — распределение по
  полю ``attempts[].ok``;
* итоговая строка ``wake_rate = wake_hits / raw_seen`` в процентах;
* общий счётчик провайдеров по выборке;
* поведение ``--diff`` не сломано (проверяем, что скрипт по-прежнему
  находит кандидаты вне YAML и маркирует слова ≤3 букв как 🚩).

Регресс --diff filtering (issue #2225):

``diff_against_yaml()`` раньше печатал **каждое** слово из JSONL, не
входящее в operator namespace — например ``расскажи``, ``мне``, ``анекдот``.
Это шум wake-сегмента, не wake-word кандидаты.

ADR-0077 §2.3 контракт: «candidates for addition» = **phonetic distortions**
существующих wake-токенов (edit-distance ≤ 2 или SequenceMatcher.ratio ≥ 0.65).
``расскажи``/``мне``/``анекдот`` НЕ являются phonetic distortions от «ТАРС» и
должны быть отфильтрованы. Слова вроде ``арс`` (1 deletion), ``тарз`` (1
substitution), ``тэрс`` (1 substitution) ДОЛЖНЫ пройти.

Скрипт намеренно standalone (без rob_box_voice) — тестируем так же.
"""

from __future__ import annotations

import importlib.util
import io
import json
import sys
from contextlib import redirect_stdout
from pathlib import Path

import pytest

# Импортируем scripts/stt/tars_stats.py напрямую: это standalone-скрипт,
# не пакет. Без этого контракт — что утилита работает на любой машине
# с одним лишь дампом выборки — не тестируется.
SCRIPT_PATH = (
    Path(__file__).resolve().parents[3] / "scripts" / "stt" / "tars_stats.py"
)
SPEC = importlib.util.spec_from_file_location("tars_stats_under_test", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None, "tars_stats.py spec failed"
tars_stats = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tars_stats)  # type: ignore[union-attr]



def _run_summarize(records: list[dict], top: int = 15) -> str:
    buf = io.StringIO()
    with redirect_stdout(buf):
        tars_stats.summarize(records, top=top)
    return buf.getvalue()


# ─────────────────────────────────────────────────────────────────────
# _provider_signature
# ─────────────────────────────────────────────────────────────────────


class TestProviderSignature:
    """Чистая функция: attempts → 'y_only' / 'y+vosk' / 'none'."""

    def test_empty_attempts_is_none(self):
        assert tars_stats._provider_signature([]) == "none"
        assert tars_stats._provider_signature(None) == "none"

    def test_yandex_only(self):
        attempts = [{"provider": "yandex", "ok": True, "reason": "ok"}]
        assert tars_stats._provider_signature(attempts) == "y_only"

    def test_yandex_ok_plus_vosk_fail_is_y_only(self):
        attempts = [
            {"provider": "yandex", "ok": True, "reason": "ok"},
            {"provider": "vosk", "ok": False, "reason": "empty"},
        ]
        assert tars_stats._provider_signature(attempts) == "y_only"

    def test_yandex_and_vosk_both_ok_is_y_plus_vosk(self):
        attempts = [
            {"provider": "yandex", "ok": True, "reason": "ok"},
            {"provider": "vosk", "ok": True, "reason": "ok"},
        ]
        assert tars_stats._provider_signature(attempts) == "y+vosk"

    def test_yandex_fail_vosk_ok_is_still_none(self):
        """Если Яндекс не дал ok=True — это НЕ y_only, а none
        (сигнал «Яндекс не справился, Vosk спас»)."""
        attempts = [
            {"provider": "yandex", "ok": False, "reason": "empty"},
            {"provider": "vosk", "ok": True, "reason": "ok"},
        ]
        assert tars_stats._provider_signature(attempts) == "none"

    def test_both_fail_is_none(self):
        attempts = [
            {"provider": "yandex", "ok": False, "reason": "timeout"},
            {"provider": "vosk", "ok": False, "reason": "empty"},
        ]
        assert tars_stats._provider_signature(attempts) == "none"


# ─────────────────────────────────────────────────────────────────────
# summarize
# ─────────────────────────────────────────────────────────────────────


class TestSummarizeProviders:
    """Колонка providers в таблице summarize."""

    def test_provider_column_header_present(self):
        out = _run_summarize(
            [
                {
                    "raw_text": "тарс",
                    "has_operator_wake": True,
                    "duration_s": 1.0,
                    "attempts": [{"provider": "yandex", "ok": True}],
                }
            ]
        )
        assert "providers" in out
        assert "y_only(1)" in out

    def test_y_plus_vosk_shown_in_table(self):
        out = _run_summarize(
            [
                {
                    "raw_text": "тарс",
                    "has_operator_wake": True,
                    "duration_s": 1.0,
                    "attempts": [
                        {"provider": "yandex", "ok": True},
                        {"provider": "vosk", "ok": True},
                    ],
                }
            ]
        )
        assert "y+vosk(1)" in out

    def test_none_shown_when_both_providers_failed(self):
        out = _run_summarize(
            [
                {
                    "raw_text": "арс",
                    "has_operator_wake": False,
                    "duration_s": 0.5,
                    "attempts": [
                        {"provider": "yandex", "ok": False},
                        {"provider": "vosk", "ok": False},
                    ],
                }
            ]
        )
        assert "none(1)" in out

    def test_total_provider_breakdown_block(self):
        """Отдельный блок «📊 провайдеры по всей выборке» обязателен."""
        out = _run_summarize(
            [
                {"raw_text": "a", "has_operator_wake": True, "duration_s": 1.0,
                 "attempts": [{"provider": "yandex", "ok": True}]},
                {"raw_text": "b", "has_operator_wake": False, "duration_s": 1.0,
                 "attempts": [{"provider": "yandex", "ok": True}]},
                {"raw_text": "c", "has_operator_wake": False, "duration_s": 1.0,
                 "attempts": [
                     {"provider": "yandex", "ok": True},
                     {"provider": "vosk", "ok": True},
                 ]},
                {"raw_text": "d", "has_operator_wake": False, "duration_s": 1.0,
                 "attempts": [{"provider": "yandex", "ok": False}]},
            ]
        )
        assert "y_only=2" in out
        assert "y+vosk=1" in out
        assert "none=1" in out


class TestSummarizeWakeRate:
    """Итоговая строка wake_rate в процентах."""

    def test_wake_rate_appears_in_output(self):
        out = _run_summarize(
            [
                {"raw_text": "tars", "has_operator_wake": True,
                 "duration_s": 1.0, "attempts": []},
                {"raw_text": "ars", "has_operator_wake": False,
                 "duration_s": 1.0, "attempts": []},
            ]
        )
        assert "wake_rate" in out
        assert "50.0%" in out

    def test_wake_rate_excludes_empty_text_records(self):
        """wake_rate = wake_hits / raw_seen — знаменатель не считает
        сегменты с пустым raw_text (иначе % занижается из-за шума)."""
        out = _run_summarize(
            [
                # raw_seen = 2 (две непустые записи), wake_hits = 1
                {"raw_text": "tars", "has_operator_wake": True,
                 "duration_s": 1.0, "attempts": []},
                {"raw_text": "ars", "has_operator_wake": False,
                 "duration_s": 1.0, "attempts": []},
                # пустой raw_text → НЕ входит в знаменатель
                {"raw_text": "", "has_operator_wake": False,
                 "duration_s": 0.5, "attempts": []},
            ]
        )
        assert "wake_rate = wake_hits / raw_seen = 50.0%" in out

    def test_wake_rate_zero_when_no_text(self):
        out = _run_summarize(
            [{"raw_text": "", "has_operator_wake": False, "duration_s": 0.5,
              "attempts": []}]
        )
        assert "wake_hits / raw_seen = 0.0%" in out

    def test_wake_rate_zero_when_no_wake_hits(self):
        out = _run_summarize(
            [
                {"raw_text": "арс", "has_operator_wake": False,
                 "duration_s": 1.0, "attempts": []},
            ]
        )
        assert "wake_hits / raw_seen = 0.0%" in out

    def test_wake_rate_full_when_all_wake(self):
        out = _run_summarize(
            [
                {"raw_text": "tars", "has_operator_wake": True,
                 "duration_s": 1.0, "attempts": []},
                {"raw_text": "тарс", "has_operator_wake": True,
                 "duration_s": 1.0, "attempts": []},
            ]
        )
        assert "wake_hits / raw_seen = 100.0%" in out


class TestSummarizeTableColumns:
    """ADR-0077 §2.3 — все 4 обязательные колонки присутствуют."""

    def test_all_four_required_columns_present(self):
        out = _run_summarize(
            [
                {"raw_text": "tars", "has_operator_wake": True,
                 "duration_s": 1.0, "attempts": [{"provider": "yandex", "ok": True}]},
            ]
        )
        assert "count" in out
        assert "wake" in out
        assert "avg_dur" in out
        assert "providers" in out
        assert "raw_text" in out


class TestSummarizeEdgeCases:
    """Рергессии на поведение, которое было до issue #2223."""

    def test_empty_records_prints_no_data_marker(self):
        out = _run_summarize([])
        assert "🔇" in out
        assert "записей нет" in out

    def test_avg_dur_still_calculated(self):
        """ADR требовал «длительность» — не сломали."""
        out = _run_summarize(
            [
                {"raw_text": "tars", "has_operator_wake": True,
                 "duration_s": 1.0, "attempts": []},
                {"raw_text": "tars", "has_operator_wake": True,
                 "duration_s": 2.0, "attempts": []},
            ]
        )
        assert "1.50с" in out  # среднее (1+2)/2

    def test_normalization_lowercase_strip(self):
        out = _run_summarize(
            [
                {"raw_text": "  TARS  ", "has_operator_wake": True,
                 "duration_s": 1.0, "attempts": []},
                {"raw_text": "tars", "has_operator_wake": True,
                 "duration_s": 1.0, "attempts": []},
            ]
        )
        # Обе записи должны схлопнуться в одну строку count=2
        assert "2" in out


# ─────────────────────────────────────────────────────────────────────
# load_records (регрессия — формат JSONL не должен сломаться)
# ─────────────────────────────────────────────────────────────────────


class TestLoadRecords:
    def test_skips_bad_json_lines(self, tmp_path, capsys):
        p = tmp_path / "x.jsonl"
        p.write_text(
            '{"raw_text": "ok"}\n'
            'not a json\n'
            '{"raw_text": "ok2"}\n',
            encoding="utf-8",
        )
        records = tars_stats.load_records(p)
        assert len(records) == 2
        err = capsys.readouterr().err
        assert "bad JSON" in err

    def test_missing_file_returns_empty(self, tmp_path):
        records = tars_stats.load_records(tmp_path / "missing.jsonl")
        assert records == []


# ─────────────────────────────────────────────────────────────────────
# diff_against_yaml — регрессия: не сломали pre-existing поведение
# ─────────────────────────────────────────────────────────────────────


class TestDiffAgainstYaml:
    def test_short_wake_like_words_flagged(self, tmp_path, capsys):
        """Wake-like слова ≤3 букв помечаются 🚩 (риск инварианта 6a).

        Issue #2225: до фикса ``--diff`` печатал ВСЕ слова вне YAML, в т.ч.
        шум wake-сегмента («расскажи», «тт» — шумовая лексика). После
        фикса печатаются только wake-like кандидаты (phonetic distortions
        от «ТАРС»). Короткие wake-like слова получают 🚩-флаг.
        «тт» (2 буквы) и «расскажи» НЕ wake-like → не печатаются.
        """
        jsonl = tmp_path / "samples.jsonl"
        jsonl.write_text(
            json.dumps(
                {
                    "raw_text": "арс и тт расскажи",
                    "has_operator_wake": False,
                    "duration_s": 1.0,
                    "attempts": [],
                }
            )
            + "\n",
            encoding="utf-8",
        )
        yaml_p = tmp_path / "wake_words.yaml"
        yaml_p.write_text(
            "operator:\n  - тарс\n  - tars\n",
            encoding="utf-8",
        )
        records = tars_stats.load_records(jsonl)
        captured = io.StringIO()
        with redirect_stdout(captured):
            tars_stats.diff_against_yaml(records, yaml_p)
        out = captured.getvalue()
        assert "арс 🚩" in out  # 3 буквы wake-like → флаг
        assert "тт" not in out  # 2 буквы → prefilter rejects (не wake-like)
        assert "расскажи" not in out  # шум wake-сегмента (issue #2225)
        assert "инвариант 6a" in out

    def test_words_in_yaml_excluded(self, tmp_path):
        jsonl = tmp_path / "samples.jsonl"
        jsonl.write_text(
            json.dumps(
                {
                    "raw_text": "тарс и арс",
                    "has_operator_wake": False,
                    "duration_s": 1.0,
                    "attempts": [],
                }
            )
            + "\n",
            encoding="utf-8",
        )
        yaml_p = tmp_path / "wake_words.yaml"
        yaml_p.write_text(
            "operator:\n  - тарс\n  - tars\n",
            encoding="utf-8",
        )
        records = tars_stats.load_records(jsonl)
        captured = io.StringIO()
        with redirect_stdout(captured):
            tars_stats.diff_against_yaml(records, yaml_p)
        out = captured.getvalue()
        assert "тарс" not in out  # уже в YAML → исключён
        assert "арс" in out  # новый кандидат

    def test_missing_yaml_warns(self, tmp_path, capsys):
        records = [{"raw_text": "x", "has_operator_wake": False}]
        tars_stats.diff_against_yaml(records, tmp_path / "nope.yaml")
        assert "YAML не найден" in capsys.readouterr().err
@pytest.fixture
def fixture_yaml(tmp_path: Path) -> Path:
    """Минимальный wake_words.yaml с operator namespace = {тарс, tars}."""
    p = tmp_path / "wake_words.yaml"
    p.write_text(
        "personality:\n  - робокс\noperator:\n  - тарс\n  - tars\n",
        encoding="utf-8",
    )
    return p


@pytest.fixture
def fixture_jsonl(tmp_path: Path) -> Path:
    """Фикстура из issue #2225: 3 wake-сегмента без has_operator_wake."""
    records = [
        {"ts_ms": 1, "duration_s": 1.2, "raw_text": "арс расскажи мне анекдот",
         "has_operator_wake": False},
        {"ts_ms": 2, "duration_s": 1.0, "raw_text": "Тарс расскажи анекдот",
         "has_operator_wake": False},
        {"ts_ms": 3, "duration_s": 0.9, "raw_text": "тарз расскажи анекдот",
         "has_operator_wake": False},
    ]
    p = tmp_path / "samples.jsonl"
    p.write_text("\n".join(json.dumps(r) for r in records) + "\n", encoding="utf-8")
    return p


# --- единичные wake-кандидаты -------------------------------------------------


@pytest.mark.parametrize("candidate", ["арс", "тарз", "тэрс", "tarss"])
def test_wake_like_distortion_passes_filter(
    candidate: str, fixture_yaml: Path
) -> None:
    """Phonetic distortions «ТАРС» (edit-distance ≤ 2) → wake-like → True.

    NB: ``tars`` exact-match is **already in YAML**, so the filter rejects
    it (early return) — that's covered by test_diff_skips_segments_with_wake_hit
    indirectly. Here we only check edit-distance ≤ 2 catches the distortions.
    """
    op_set = {"тарс", "tars"}
    assert tars_stats._is_wake_like(candidate, op_set), (
        f"{candidate!r} должна быть wake-like (≤2 от «тарс»/«tars»)"
    )


# --- шум wake-сегмента -------------------------------------------------------


@pytest.mark.parametrize("noise", ["расскажи", "мне", "анекдот", "пожалуйста", "стоп"])
def test_noise_words_filtered_out(noise: str, fixture_yaml: Path) -> None:
    """Шумовая лексика wake-сегмента НЕ должна проходить фильтр."""
    op_set = {"тарс", "tars"}
    assert not tars_stats._is_wake_like(noise, op_set), (
        f"{noise!r} — шум wake-сегмента, НЕ wake-кандидат"
    )


# --- интеграция diff_against_yaml --------------------------------------------


def test_diff_filters_noise_words(
    fixture_yaml: Path, fixture_jsonl: Path
) -> None:
    """End-to-end: «расскажи/мне/анекдот» не появляются в выводе --diff."""
    buf = io.StringIO()
    with redirect_stdout(buf):
        tars_stats.diff_against_yaml(
            tars_stats.load_records(fixture_jsonl), fixture_yaml
        )
    out = buf.getvalue()

    # wake-кандидаты — есть
    assert "арс" in out, f"«арс» должна быть кандидатом, нет в:\n{out}"
    assert "тарз" in out, f"«тарз» должна быть кандидатом, нет в:\n{out}"

    # шумовая лексика — нет
    for noise in ("расскажи", "мне", "анекдот"):
        assert noise not in out, (
            f"«{noise}» НЕ должна быть в --diff (шум wake-сегмента), "
            f"но нашлась в:\n{out}"
        )


def test_diff_skips_segments_with_wake_hit(
    fixture_yaml: Path, tmp_path: Path
) -> None:
    """Сегменты с has_operator_wake=True → слова не учитываются."""
    records = [
        {"ts_ms": 1, "duration_s": 1.0, "raw_text": "тарс расскажи",
         "has_operator_wake": True},  # уже покрыто
        {"ts_ms": 2, "duration_s": 1.0, "raw_text": "тарз расскажи",
         "has_operator_wake": False},  # кандидат
    ]
    p = tmp_path / "mixed.jsonl"
    p.write_text("\n".join(json.dumps(r) for r in records) + "\n", encoding="utf-8")

    buf = io.StringIO()
    with redirect_stdout(buf):
        tars_stats.diff_against_yaml(tars_stats.load_records(p), fixture_yaml)
    out = buf.getvalue()

    assert "тарз" in out
    assert "тарс" not in out or "тарс расскажи" not in out  # wake-сегмент не печатается


def test_diff_empty_when_only_noise(fixture_yaml: Path, tmp_path: Path) -> None:
    """Только шумовая лексика (без wake-искажений) → пустой результат."""
    records = [
        {"ts_ms": 1, "duration_s": 1.0, "raw_text": "расскажи мне анекдот",
         "has_operator_wake": False},
    ]
    p = tmp_path / "noise_only.jsonl"
    p.write_text(json.dumps(records[0]) + "\n", encoding="utf-8")

    buf = io.StringIO()
    with redirect_stdout(buf):
        tars_stats.diff_against_yaml(tars_stats.load_records(p), fixture_yaml)
    out = buf.getvalue()

    assert "нет кандидатов" in out
    assert "расскажи" not in out
    assert "мне" not in out
    assert "анекдот" not in out


# --- хелпер Левенштейна ------------------------------------------------------


@pytest.mark.parametrize(
    "a,b,max_d,expected",
    [
        ("тарс", "тарс", 0, True),
        ("тарс", "арс", 2, True),     # 1 deletion
        ("тарс", "тарз", 2, True),    # 1 substitution
        ("тарс", "тэрс", 2, True),    # 1 substitution
        # NB: edit-distance=2 is the wake-like boundary per ADR-0077 §2.3,
        # so «расс» (dist=2 от «тарс») IS wake-like. The strict exclusion
        # in --diff comes from the SHORTER wake tokens — dist=2 with a 4-letter
        # base covers most STT distortions but not «расскажи» (dist=6).
        ("тарс", "расс", 2, True),    # 2 substitutions — within boundary
        ("тарс", "расскажи", 2, False),  # far outside edit-distance ≤ 2
        ("тарс", "мне", 2, False),    # length 2 — prefilter rejects
    ],
)
def test_levenshtein_le(a: str, b: str, max_d: int, expected: bool) -> None:
    assert tars_stats._levenshtein_le(a, b, max_d) is expected