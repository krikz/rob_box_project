"""
test_masterfilter_dynamics.py — радио-динамика мастер-шины (issue #3154, 29.09).

Жалоба товарища Шифу: «громкость скачет — то почти не слышно, то нормально».
Живой jack_rec DJ-сета: −35…−46 dB RMS за 40 с, провал фейда до −65. В
``masterfilter.scd`` добавлены выравниватель + компрессор + лимитер (см. файл).

История мастер-шины требует держать инварианты тестом, а не памятью:
``masterlimiter`` (Compander + Limiter) 31.08 глушил ВЕСЬ выход — NaN
защёлкивался в состоянии стоковых UGen'ов. Поэтому:

* стоковых динамических UGen'ов с состоянием в файле нет;
* вход чистится от NaN ДО всего остального;
* вход каждого детектора (Gate/Amplitude) проходит через ``clean``;
* выход динамики проверяется CheckBadValues ДО ``clip2`` (clip2 превращает
  NaN в ±потолок — офлайн-рендер NRT это показал: постоянка −7 dBFS), и
  порча уходит на прежний путь ``safe`` (tanh);
* фейдер ``gain`` (music_master_gain / set_music_volume) — ПОСЛЕ лимитера:
  шаг «громче» даёт ровно свой шаг и в лимитер не упирается;
* пик выхода динамики ≤ −1 dBFS (``ceiling``), ``dyn = 1`` по умолчанию.

Числа (уровни до/после) — офлайн-рендер ``scripts/music/club_loudness_nrt.py``
в PR, НЕ замер на роботе. Не требует ROS2, SuperCollider и робота.
"""

from pathlib import Path


def _repo_root(start: Path) -> Path:
    for parent in [start, *start.parents]:
        if (parent / "docker").is_dir() and (parent / "src").is_dir():
            return parent
    return start.parents[5]


MASTERFILTER = (
    _repo_root(Path(__file__).resolve())
    / "docker" / "vision" / "voice_assistant" / "custom_synthdefs" / "masterfilter.scd"
)


def _code() -> str:
    """Текст SynthDef без комментариев ``//`` (в них упоминаются Compander и т.п.)."""
    lines = MASTERFILTER.read_text(encoding="utf-8").splitlines()
    return "\n".join(line.split("//", 1)[0] for line in lines)


def _defaults() -> dict:
    """Аргументы SynthDef ``|a = 1, b = 2|`` → {имя: число}."""
    code = _code()
    head = code.index("SynthDef.new(\\masterfilter")
    start = code.index("|", head)
    end = code.index("|", start + 1)
    out = {}
    for item in code[start + 1:end].replace("\n", " ").split(","):
        name, _, value = item.partition("=")
        out[name.strip()] = float(value.strip())
    return out


def _pos(code: str, needle: str) -> int:
    assert needle in code, f"нет в masterfilter.scd: {needle!r}"
    return code.index(needle)


def test_no_stock_stateful_dynamics() -> None:
    """Compander/Limiter/Normalizer защёлкивались на NaN (31.08) — их здесь нет."""
    code = _code()
    for ugen in ("Compander.", "Limiter.", "Normalizer.", "CompanderD."):
        assert ugen not in code, ugen


def test_input_is_sanitized_before_filters_and_dynamics() -> None:
    code = _code()
    sanitize = _pos(code, "sig = Select.ar(bad,")
    assert sanitize < _pos(code, "HPF.ar(") < _pos(code, "Lag.ar(")


def test_every_detector_input_goes_through_clean() -> None:
    code = _code()
    assert "var clean = { |s| Select.ar(CheckBadValues.ar(s, 0, 0).min(1), [s, DC.ar(0)]) };" in code
    for line in code.splitlines():
        if "Amplitude.ar(" in line:
            assert "Amplitude.ar(clean.(" in line, line
        if "Gate.ar(" in line:
            assert "Gate.ar(clean.(" in line, line


def test_bad_dynamics_output_falls_back_before_clip() -> None:
    code = _code()
    check = _pos(code, "wetBad = CheckBadValues.ar(wet, 0, 0).min(1);")
    fallback = _pos(code, "wet = Select.ar(wetBad, [wet, safe]).clip2(ceilAmp);")
    assert check < fallback
    assert "safe = (sig * drive).tanh;" in code
    assert "out = Select.ar(dyn > 0.5, [safe, wet]);" in code


def test_fader_is_after_the_limiter() -> None:
    """music_master_gain / set_music_volume двигают готовый сигнал, а не вход лимитера."""
    code = _code()
    assert _pos(code, ".clip2(ceilAmp)") < _pos(code, "out = out * Lag.kr(gain, lag);") < _pos(code, "ReplaceOut.ar(")


def test_defaults_keep_headroom_and_dynamics_on() -> None:
    d = _defaults()
    assert d["dyn"] == 1
    assert d["ceiling"] <= -1.0
    assert d["gain"] == 0.5  # MusicManager.DEFAULT_MASTER_GAIN / music_master_gain
    assert d["lvlRatio"] >= 1 and d["cmpRatio"] >= 1
    assert d["lvlBoost"] > 0 and d["lvlCut"] > 0
    assert d["lvlGate"] < d["lvlTarget"]
    assert 0 < d["limLook"] <= 0.01  # DelayN максимум 0.01 с
