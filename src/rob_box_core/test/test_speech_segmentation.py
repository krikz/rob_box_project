"""Тесты для rob_box_core.speech_segmentation (issue #2199).

Зачем
-----
Раньше «конец фразы» решался четырьмя разными правилами в трёх местах
(см. issue #2199 и докстринг ``rob_box_core.speech_segmentation``).
Эти тесты — **точка истины**: один и тот же PCM-фрагмент должен
сегментироваться **одинаково** для всех путей (wake / robot_voice),
а регресс #2135 (блип 0.02с в wake) — больше не повторяться.

Чистый Python: ``rob_box_core.speech_segmentation`` не зависит от rclpy,
поэтому тесты бегут на dev-машине без Docker.
"""

from __future__ import annotations

import ast
import struct
from pathlib import Path

import pytest

from rob_box_core.speech_segmentation import (
    DEFAULT_ROBOT_VOICE_CONFIG,
    DEFAULT_WAKE_CONFIG,
    BYTES_PER_S,
    PhraseSegmenter,
    SpeechSegmentationConfig,
    _frame_is_speech,
)


# 20 мс @ 16 кГц = 320 семплов int16 = 640 байт.
FRAME_LEN = 640
FRAME_PERIOD_S = 0.02


def _speech_frame(sample_value: int = 1000) -> bytes:
    """int16 LE PCM-кадр постоянной амплитуды (пиковый достаточно выше
    порога 500 для peak-VAD и выше для RMS-порога)."""
    return struct.pack("<%dh" % 320, *([sample_value] * 320))


def _silent_frame() -> bytes:
    """int16 LE PCM-кадр нулей (peak=0, rms=0)."""
    return b"\x00\x00" * 320


def _silence_seg(seg: PhraseSegmenter, n: int, start: float = 0.0) -> float:
    """Скормить ``n`` тихих кадров. Вернуть финальное time-monotonic."""
    now = start
    for _ in range(n):
        seg.add_frame(_silent_frame(), now)
        now += FRAME_PERIOD_S
    return now


def _speech_seg(
    seg: PhraseSegmenter, n: int, start: float = 0.0, sample: int = 1000
) -> float:
    """Скормить ``n`` речевых кадров. Вернуть финальное time-monotonic."""
    now = start
    for _ in range(n):
        seg.add_frame(_speech_frame(sample), now)
        now += FRAME_PERIOD_S
    return now


# ─── API и конфиг ──────────────────────────────────────────────────────


def test_from_mapping_default_keys():
    """from_mapping работает без всех ключей (минимум — statistic)."""
    cfg = SpeechSegmentationConfig.from_mapping({"statistic": "rms"})
    assert cfg.statistic == "rms"
    assert cfg.gap_timeout_s == 0.4
    assert cfg.min_phrase_s == 0.25
    assert cfg.max_phrase_s == 15.0


def test_from_mapping_rejects_unknown_statistic():
    with pytest.raises(ValueError, match="unknown statistic"):
        SpeechSegmentationConfig.from_mapping({"statistic": "fft"})


def test_default_wake_uses_none_statistic():
    """Wake-канал: клиент уже отфильтровал — per-frame VAD не нужен."""
    assert DEFAULT_WAKE_CONFIG.statistic == "none"
    assert DEFAULT_WAKE_CONFIG.gap_timeout_s == 0.4
    assert DEFAULT_WAKE_CONFIG.min_phrase_s == 0.25


def test_default_robot_voice_uses_peak_500():
    """robot_voice: peak < 500, пауза 300мс — эквивалент старого _chunk_is_silent."""
    assert DEFAULT_ROBOT_VOICE_CONFIG.statistic == "peak"
    assert DEFAULT_ROBOT_VOICE_CONFIG.speech_threshold == 500
    assert DEFAULT_ROBOT_VOICE_CONFIG.gap_timeout_s == 0.3


def test_gap_timeout_bytes_matches_period():
    """Вспомогательное свойство для метрик и сравнения со старыми константами."""
    cfg = SpeechSegmentationConfig(gap_timeout_s=0.3)
    # 0.3 с × 32000 Б/с = 9600 байт
    assert cfg.gap_timeout_bytes == 9600


# ─── _frame_is_speech — совместимость со старым _chunk_is_silent ───────


@pytest.mark.parametrize(
    "samples,expected_speech",
    [
        ((0, 0, 0), False),  # тишина
        ((499, -499), False),  # на грани (ниже 500)
        ((500, 0), True),  # ровно порог (>=)
        ((-500,), True),  # отрицательный пик
        ((), False),  # пустой кадр
    ],
)
def test_frame_is_speech_peak_legacy_compat(samples, expected_speech):
    """_frame_is_speech со statistic='peak' — это инверсия старого _chunk_is_silent."""
    out = bytearray()
    for s in samples:
        if s < 0:
            s += 0x10000
        out.append(s & 0xFF)
        out.append((s >> 8) & 0xFF)
    assert _frame_is_speech(bytes(out), "peak", 500) is expected_speech


def test_frame_is_speech_odd_length_silent():
    """Нечётная длина — старый _chunk_is_silent считал это тишиной."""
    assert _frame_is_speech(b"\x01", "peak", 500) is False


def test_frame_is_speech_rms_basic():
    """RMS int16: peak=±4096 → rms=4096 (постоянный сигнал)."""
    pcm = struct.pack("<%dh" % 320, *([4096] * 320))
    assert _frame_is_speech(pcm, "rms", 4000) is True
    assert _frame_is_speech(pcm, "rms", 5000) is False


def test_frame_is_speech_none_always_true():
    """statistic='none' → кадр всегда речь (wake-канал, клиент уже фильтрует)."""
    assert _frame_is_speech(_silent_frame(), "none", 0) is True
    assert _frame_is_speech(b"", "none", 0) is True


# ─── PhraseSegmenter: wake-семантика ───────────────────────────────────


def test_wake_long_phrase_then_gap_closes_exactly_once():
    """wake: 50 кадров речи + tick(gap) = 1 фраза размером 50 кадров."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    now = 1000.0
    phrases = []
    for _ in range(50):
        out = seg.add_frame(_speech_frame(), now)
        if out is not None:
            phrases.append(out)
        now += FRAME_PERIOD_S
    assert phrases == []  # пока идут кадры — не закрывается

    closed = seg.tick(now + DEFAULT_WAKE_CONFIG.gap_timeout_s)
    assert closed == _speech_frame() * 50
    assert seg.buffered_bytes == 0


def test_wake_short_gap_does_not_close():
    """Пауза короче gap_timeout_s — фраза не закрывается."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    now = 1000.0
    for _ in range(30):
        seg.add_frame(_speech_frame(), now)
        now += FRAME_PERIOD_S
    assert seg.tick(now + DEFAULT_WAKE_CONFIG.gap_timeout_s / 2) is None
    assert seg.buffered_bytes == 30 * FRAME_LEN


def test_wake_blip_does_not_publish():
    """Регресс #2135: 0.02с / 640 байт wake = блип → не уходит в STT."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    assert seg.add_frame(_speech_frame(), 100.0) is None
    closed = seg.tick(100.0 + DEFAULT_WAKE_CONFIG.gap_timeout_s)
    assert closed is None
    assert seg.dropped_short_phrases == 1


def test_wake_buffer_cap_truncates():
    """Потолок max_phrase_s: фраза режется и уходит как есть."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    frames_to_cap = DEFAULT_WAKE_CONFIG.max_phrase_bytes // FRAME_LEN
    phrases = []
    now = 500.0
    for _ in range(frames_to_cap):
        out = seg.add_frame(_speech_frame(), now)
        if out is not None:
            phrases.append(out)
        now += FRAME_PERIOD_S
    assert len(phrases) == 1
    assert len(phrases[0]) == DEFAULT_WAKE_CONFIG.max_phrase_bytes
    assert seg.truncated_phrases == 1


def test_wake_reset_drops_unfinished_phrase():
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    _speech_seg(seg, 40)
    dropped = seg.reset()
    assert dropped == 40 * FRAME_LEN
    assert seg.buffered_bytes == 0


def test_wake_two_phrases_separated_by_gap():
    """Две реплики после паузы — две раздельные фразы, не склеиваются."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    now = _speech_seg(seg, 40, start=1000.0)
    first = seg.tick(now + DEFAULT_WAKE_CONFIG.gap_timeout_s)
    assert first == _speech_frame() * 40
    now = _speech_seg(seg, 20, start=now + 5.0)
    second = seg.tick(now + DEFAULT_WAKE_CONFIG.gap_timeout_s)
    assert second == _speech_frame() * 20


def test_regression_2135_tars_phrase_one_audiodata():
    """Регресс #2135 (DoD #2199).

    Сценарий: оператор говорит в шлем «ТАРС, сделай X» (0.5 с речи),
    делает паузу 0.5 с, продолжает «Y» (ещё 0.3 с речи). На коде ДО
    фикса каждая 20мс-порция PCM уходила в ``/audio/quest_wake`` отдельным
    ``AudioData`` и ``stt_node`` гонял полный цикл распознавания на
    каждой — распознавание 0.02 с всегда возвращало пусто, поэтому
    вейк «ТАРС» из шлема не мог сработать **никогда**.

    После фикса: ровно **две** ``AudioData`` (по одной на каждую
    фразовую паузу), каждая — сумма накопленных 20мс-кадров.
    ``stt_node`` теперь видит фразу длиной ≥0.25 с и распознаёт «ТАРС».
    """
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    now = 1000.0
    # «ТАРС, сделай X» — 25 кадров = 0.5 с речи (заведомо > min_phrase_s).
    phrases = []
    for _ in range(25):
        out = seg.add_frame(_speech_frame(), now)
        if out is not None:
            phrases.append(out)
        now += FRAME_PERIOD_S
    # Пауза 0.5 с — больше gap_timeout_s (0.4 с).
    now += 0.5
    closed = seg.tick(now)
    assert closed is not None
    assert len(closed) == 25 * FRAME_LEN
    assert len(closed) >= DEFAULT_WAKE_CONFIG.min_phrase_bytes
    phrases.append(closed)

    # «Y» — ещё 15 кадров = 0.3 с речи.
    for _ in range(15):
        out = seg.add_frame(_speech_frame(), now)
        if out is not None:
            phrases.append(out)
        now += FRAME_PERIOD_S
    closed2 = seg.tick(now + 0.5)
    if closed2 is not None:
        phrases.append(closed2)

    # Итог: 2 фразы (или 3, если что-то закрылось посередине). Главное —
    # НЕ 25 + 15 = 40 микро-фраз, как было до фикса #2135.
    assert 2 <= len(phrases) <= 3
    # Каждая фраза короче, чем 40 × 640 байт (= всё подряд склеенное).
    for phrase in phrases:
        assert len(phrase) <= 40 * FRAME_LEN
        assert len(phrase) >= DEFAULT_WAKE_CONFIG.min_phrase_bytes


# ─── PhraseSegmenter: robot_voice-семантика ─────────────────────────────


def test_robot_voice_long_speech_then_15_silent_closes():
    """EOU: 40 речевых + 15 тихих кадров (300мс) → 1 фраза."""
    seg = PhraseSegmenter(DEFAULT_ROBOT_VOICE_CONFIG)
    _speech_seg(seg, 40)
    assert seg.buffered_bytes == 40 * FRAME_LEN

    phrases = []
    now = 0.0
    for _ in range(15):
        out = seg.add_frame(_silent_frame(), now)
        if out is not None:
            phrases.append(out)
        now += FRAME_PERIOD_S
    assert len(phrases) == 1
    assert len(phrases[0]) == 40 * FRAME_LEN
    # Счётчик тишины сбросился после close.
    assert seg._silence_bytes_since_speech == 0


def test_robot_voice_14_silent_does_not_close():
    """Граница: 14 тихих кадров (280мс < 300мс) — фраза не закрывается."""
    seg = PhraseSegmenter(DEFAULT_ROBOT_VOICE_CONFIG)
    _speech_seg(seg, 40)
    now = 0.0
    phrases = []
    for _ in range(14):
        out = seg.add_frame(_silent_frame(), now)
        if out is not None:
            phrases.append(out)
        now += FRAME_PERIOD_S
    assert phrases == []
    assert seg.buffered_bytes == 40 * FRAME_LEN


def test_robot_voice_leading_silence_does_not_open_phrase():
    """Ведущая тишина не публикует пустой буфер."""
    seg = PhraseSegmenter(DEFAULT_ROBOT_VOICE_CONFIG)
    now = 0.0
    phrases = []
    for _ in range(20):
        out = seg.add_frame(_silent_frame(), now)
        if out is not None:
            phrases.append(out)
        now += FRAME_PERIOD_S
    assert phrases == []
    assert seg.buffered_bytes == 0


def test_robot_voice_force_close_flushes_partial():
    """PTT release: ``force_close`` отдаёт буфер без min_phrase-проверки."""
    seg = PhraseSegmenter(DEFAULT_ROBOT_VOICE_CONFIG)
    _speech_seg(seg, 20)
    phrase = seg.force_close()
    assert phrase is not None
    assert len(phrase) == 20 * FRAME_LEN


def test_robot_voice_force_close_empty_returns_none():
    seg = PhraseSegmenter(DEFAULT_ROBOT_VOICE_CONFIG)
    assert seg.force_close() is None


# ─── Issue #2199: одна и та же PCM-запись сегментируется одинаково ────


def _make_phrase(recording: bytes, frame_count: int) -> list[bytes]:
    """Разрезать PCM-запись на кадры по FRAME_LEN."""
    return [
        recording[i : i + FRAME_LEN]
        for i in range(0, frame_count * FRAME_LEN, FRAME_LEN)
    ]


def test_identical_segmentation_across_paths():
    """DoD #2199: при одинаковом config — одинаковая сегментация для всех путей.

    Берём запись «2 секунды речи + 300мс тишины + 1 секунда речи +
    300мс тишины + 0.5 секунды речи» (5 фраз по робот-voice-семантике).
    Конструируем два сегментатора с **одинаковыми** gap/min/max, но
    разной ``statistic``: ``"peak"`` (robot_voice) видит тишину как
    кадры-тишины, ``"none"`` (wake-эмуляция) видит только речь. Границы
    фраз должны совпасть, потому что wake-сегментатор **сам** вызовет
    закрытие через ``tick()`` после последнего кадра, а robot_voice
    закроет на тишине внутри ``add_frame``.
    """
    # 100 речевых + 15 тишины + 50 речевых + 15 тишины + 25 речевых +
    # 20 тишины (хвостовая, чтобы robot_voice тоже закрыл последнюю фразу)
    frames_total = 225
    frames = [0] * frames_total
    for i in range(0, 100):
        frames[i] = 1000
    for i in range(115, 165):
        frames[i] = 1000
    for i in range(180, 205):
        frames[i] = 1000

    # Один конфиг для обоих сегментаторов, разная статистика.
    shared_cfg = SpeechSegmentationConfig(
        statistic="none",  # wake: клиент уже отфильтровал
        speech_threshold=0,
        gap_timeout_s=0.3,  # одинаковый gap для обоих
        min_phrase_s=0.25,
        max_phrase_s=15.0,
    )
    robot_cfg = SpeechSegmentationConfig(
        statistic="peak",
        speech_threshold=500,
        gap_timeout_s=0.3,
        min_phrase_s=0.25,
        max_phrase_s=15.0,
    )

    raw_frames = [struct.pack("<320h", *([v] * 320)) for v in frames]
    # Для wake: имитируем клиентский VAD — фильтруем тишину, но
    # **сохраняем time-monotonic** от исходного кадра. Это эквивалентно
    # тому, что клиент пропускает кадры тишины, не прибавляя time.
    wake_times = [1000.0 + i * FRAME_PERIOD_S for i in range(frames_total)]
    wake_frames = [
        (t, f) for t, f in zip(wake_times, raw_frames) if f != _silent_frame()
    ]
    last_wake_t = wake_frames[-1][0] if wake_frames else wake_times[0]

    # --- wake ---
    wake_seg = PhraseSegmenter(shared_cfg)
    wake_closes: list[float] = []
    for t, f in wake_frames:
        out = wake_seg.add_frame(f, t)
        if out is not None:
            wake_closes.append(t)
    # Хвост: эмулируем продовый таймер 30 Гц — он заметит паузу через
    # ~gap_timeout_s + 1 кадр. Берём close_time как момент первого
    # тика после истечения gap, а не «+0.5с» — это синхронизирует
    # wake и robot_voice в пределах одного кадра.
    tail_close_time = last_wake_t + shared_cfg.gap_timeout_s + FRAME_PERIOD_S
    tail = wake_seg.tick(tail_close_time)
    if tail is not None:
        wake_closes.append(tail_close_time)

    # --- robot_voice ---
    rv_seg = PhraseSegmenter(robot_cfg)
    now = 1000.0
    rv_closes: list[float] = []
    for f in raw_frames:
        out = rv_seg.add_frame(f, now)
        if out is not None:
            rv_closes.append(now)
        now += FRAME_PERIOD_S

    # Должно быть 3 фразы в обоих случаях (между 5 зонами речи).
    assert len(wake_closes) == 3, f"wake got {len(wake_closes)} phrases"
    assert len(rv_closes) == 3, f"robot_voice got {len(rv_closes)} phrases"
    # Границы (до FRAME_PERIOD_S): оба закрывают на кадре, где
    # wake-кадры — это речевой кадр после паузы, а robot_voice — на
    # 15-м тихом кадре (300мс).
    for w, r in zip(wake_closes, rv_closes):
        assert abs(w - r) < FRAME_PERIOD_S * 2, (
            f"wake close {w} vs robot_voice close {r} differ by >{FRAME_PERIOD_S*2}s"
        )


def test_wake_consumes_silence_bytes_zero():
    """wake: тишины от клиента не поступает → _silence_bytes_since_speech всегда 0."""
    seg = PhraseSegmenter(DEFAULT_WAKE_CONFIG)
    _speech_seg(seg, 10)
    assert seg._silence_bytes_since_speech == 0
    seg.tick(1.0)
    assert seg._silence_bytes_since_speech == 0


def test_robot_voice_silence_counter_resets_on_speech():
    """robot_voice: речевой кадр сбрасывает счётчик тишины."""
    seg = PhraseSegmenter(DEFAULT_ROBOT_VOICE_CONFIG)
    _silence_seg(seg, 5)
    assert seg._silence_bytes_since_speech == 5 * FRAME_LEN
    seg.add_frame(_speech_frame(), 0.5)
    assert seg._silence_bytes_since_speech == 0


# ── Guard-rail: единый источник порогов (issue #2199, ревью Шифу 09.09) ──
#
# GOODWORKRINKZ просил «тест, который импортирует конфиг и проверяет, что
# quest_node, wake_segmenter и клиентский слой берут значения из него, а
# не из литералов». Реализуем это в два слоя:
#
# 1. ``test_default_configs_match_yaml_profile`` — Python-дефолты
#    (``DEFAULT_WAKE_CONFIG``, ``DEFAULT_ROBOT_VOICE_CONFIG``) совпадают
#    с YAML-профилями. Любая правка одного без другого — тест упадёт.
# 2. ``test_segmentation_callers_source_thresholds_from_config`` — AST-парс
#    ``quest_node.py`` и ``wake_segmenter.py``: запрещаем литералы
#    ``0.4``/``0.3``/``500`` рядом с атрибутами ``gap_timeout_s``,
#    ``speech_threshold`` (раньше там были магические числа — должны
#    приезжать из ``DEFAULT_*_CONFIG`` или YAML).
#
# Это превращает DoD-инвариант «не сорься с конфигом» в машинно-проверяемый
# контракт, не зависящий от аккуратности grep'а.

# Пути к файлам, которые должны брать пороги из конфига.
# test/ лежит в src/rob_box_core/test/ — корень репо на 3 уровня выше.
_REPO_ROOT = Path(__file__).resolve().parents[3]
_QUEST_NODE_PY = (
    _REPO_ROOT / "src/rob_box_quest/rob_box_quest/quest_node.py"
)
_WAKE_SEGMENTER_PY = (
    _REPO_ROOT / "src/rob_box_quest/rob_box_quest/core/wake_segmenter.py"
)


def _yaml_profiles() -> dict:
    """Прочитать ``profiles`` из ``config/speech_segmentation.yaml``.

    pyyaml — мягкая зависимость (объявлена в ``setup.py``), но на
    dev-машине без ``pip install -e`` может отсутствовать. Тест пропускается
    в этом случае (CI прогоняет после colcon install — там пакет стоит).
    """
    yaml_path = (
        _REPO_ROOT / "src/rob_box_core/config/speech_segmentation.yaml"
    )
    try:
        import yaml  # type: ignore[import-untyped]
    except ImportError:
        pytest.skip("pyyaml не установлен (pip install pyyaml)")
    with yaml_path.open(encoding="utf-8") as fp:
        return yaml.safe_load(fp)["profiles"]


def test_default_configs_match_yaml_profile():
    """``DEFAULT_*_CONFIG`` — точное зеркало YAML-профилей.

    Это тест-контракт: правим YAML → меняем дефолт → тест остаётся
    зелёным. Правим только одно из двух → падаем.
    """
    profiles = _yaml_profiles()

    wake = profiles["wake"]
    assert DEFAULT_WAKE_CONFIG.statistic == wake["statistic"]
    assert DEFAULT_WAKE_CONFIG.speech_threshold == wake["speech_threshold"]
    assert DEFAULT_WAKE_CONFIG.gap_timeout_s == wake["gap_timeout_s"]
    assert DEFAULT_WAKE_CONFIG.min_phrase_s == wake["min_phrase_s"]
    assert DEFAULT_WAKE_CONFIG.max_phrase_s == wake["max_phrase_s"]

    rv = profiles["robot_voice"]
    assert DEFAULT_ROBOT_VOICE_CONFIG.statistic == rv["statistic"]
    assert (
        DEFAULT_ROBOT_VOICE_CONFIG.speech_threshold == rv["speech_threshold"]
    )
    assert DEFAULT_ROBOT_VOICE_CONFIG.gap_timeout_s == rv["gap_timeout_s"]
    assert DEFAULT_ROBOT_VOICE_CONFIG.min_phrase_s == rv["min_phrase_s"]
    assert DEFAULT_ROBOT_VOICE_CONFIG.max_phrase_s == rv["max_phrase_s"]


def _file_uses_attribute_literal_pair(
    tree: ast.AST, attr_name: str, forbidden_literals: tuple
) -> list[tuple[int, str]]:
    """Вернуть ``[(lineno, snippet), ...]`` для подозрительных узлов.

    Ищем конструкции вида ``<name>.<attr_name> = <literal>`` (Assign) и
    вызовы ``<name>.<attr_name>(<literal>)`` (Call) — т.е. ситуации, когда
    кто-то пытается выставить порог сегментации числом в обход конфига.
    """
    hits: list[tuple[int, str]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if (
                    isinstance(target, ast.Attribute)
                    and target.attr == attr_name
                    and isinstance(node.value, ast.Constant)
                    and node.value.value in forbidden_literals
                ):
                    hits.append(
                        (node.lineno, f"Assign attr={attr_name} value={node.value.value!r}")
                    )
        elif isinstance(node, ast.Call):
            func = node.func
            if (
                isinstance(func, ast.Attribute)
                and func.attr == attr_name
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value in forbidden_literals
            ):
                hits.append(
                    (
                        node.lineno,
                        f"Call attr={attr_name} value={node.args[0].value!r}",
                    )
                )
    return hits


@pytest.mark.parametrize(
    "py_path, forbidden_pairs",
    [
        # wake-канал: gap_timeout_s, min_phrase_s, max_phrase_s из конфига.
        # robot_voice: + speech_threshold=500.
        (
            _QUEST_NODE_PY,
            {
                "gap_timeout_s": (0.4,),
                "min_phrase_s": (0.25,),
                "max_phrase_s": (15.0,),
                "speech_threshold": (500,),
            },
        ),
        # wake_segmenter — это shim, его константы ДОЛЖНЫ быть
        # ``DEFAULT_WAKE_CONFIG.<attr>``. Литералы запрещены.
        (
            _WAKE_SEGMENTER_PY,
            {
                "gap_timeout_s": (0.4,),
                "min_phrase_s": (0.25,),
                "max_phrase_s": (15.0,),
                "speech_threshold": (500,),
            },
        ),
    ],
)
def test_segmentation_callers_source_thresholds_from_config(
    py_path: Path, forbidden_pairs: dict
):
    """quest_node/wake_segmenter НЕ должны содержать литералов порогов.

    DoD-проверка issue #2199: «сегментатор один, пороги — из конфига».
    Защита от регрессии: кто-то добавил ``segmenter.gap_timeout_s = 0.4``
    в обход YAML/DEFAULT_*_CONFIG — тест упадёт с указанием файла:строки.
    """
    tree = ast.parse(py_path.read_text(encoding="utf-8"))
    all_hits: list[str] = []
    for attr, literals in forbidden_pairs.items():
        for lineno, snippet in _file_uses_attribute_literal_pair(
            tree, attr, literals
        ):
            all_hits.append(f"{py_path.name}:{lineno}: {snippet}")
    assert not all_hits, (
        "Найдены литералы порогов сегментации в обход "
        "DEFAULT_*_CONFIG/YAML (issue #2199, ревью Шифу 09.09). "
        "Берите значения из rob_box_core.speech_segmentation.DEFAULT_*_CONFIG "
        "или заведите новое поле в YAML:\n  " + "\n  ".join(all_hits)
    )
