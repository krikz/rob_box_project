"""
SSML prosody (pitch/volume) conversion helpers for TTS providers.

ADR-0145 §4 TTSNode step 1 — pure-Python module-level functions extracted
verbatim (no behaviour change) from ``tts_node.py``. Kept dependency-free
(no ROS/grpc/numpy) so it can be unit-tested standalone, same rationale as
the sibling ``tts_chunking.py`` module.

Covers two independent conversions:

* Issue #1780 — Yandex Cloud TTS v3 gRPC ``Hints`` only accepts
  ``pitch_shift`` (Hz-offset) and ``volume`` (absolute LUFS target), while
  SSML ``<prosody>`` uses relative multipliers/levels
  (``pitch="+10%"``, ``volume="loud"``). :func:`_ssml_pitch_to_hz` and
  :func:`_ssml_volume_to_lufs_target` bridge that gap.
* Issue #1064 — Silero v5 ``apply_tts`` only accepts a small whitelist of
  named pitch levels in ``<prosody pitch="...">``; LLM/MiniMax-style SSML
  emits numeric multipliers that crash Silero with ``Invalid <prosody>
  tag``. :func:`normalize_silero_pitch` maps arbitrary input to the
  nearest allowed level.

``tts_node.py`` re-exports every name here for backward compatibility with
existing imports/tests (``from rob_box_voice.tts_node import
_ssml_pitch_to_hz`` etc.).
"""

from typing import Optional

_SILERO_PITCH_LEVELS = ("x-low", "low", "medium", "high", "x-high", "robot")


# ── Issue #1780: Yandex gRPC v3 SSML → pitch/volume конвертация ────────────
# Yandex Cloud TTS v3 ``Hints`` API поддерживает только:
#   * ``pitch_shift`` — Hz-offset (range [-1000; 1000], default 0)
#   * ``volume``      — LUFS dB-offset (range [-145; 0), default -19)
#
# SSML `<prosody>` оперирует относительными множителями/уровнями
# (``pitch="+10%"``, ``volume="loud"``). Здесь мы приводим их к
# Yandex-формату без потери смысла: «на сколько Hz поднять голос» и
# «на сколько dB сделать громче/тише относительно дефолта».
YANDEX_BASELINE_PITCH_HZ: float = (
    130.0  # средняя основная частота голоса anton (~130 Hz)
)
YANDEX_BASELINE_VOLUME_LUFS: float = -19.0  # Yandex дефолт для LUFS-нормализации
_YANDEX_PITCH_SHIFT_MAX_HZ: float = 1000.0  # абсолютный предел API
_YANDEX_VOLUME_MIN_LUFS: float = -145.0  # нижний предел API


def _ssml_pitch_to_hz(pitch) -> Optional[float]:
    """SSML pitch → Hz-offset для Yandex gRPC v3 ``Hints.pitch_shift``.

    Принимает те же формы, что и ``_parse_ssml_attributes``:
    ``"+10%"``, ``"-25%"``, ``"1.2"``, ``"high"``, ``"low"``, ``"medium"``,
    ``"x-high"``, ``"x-low"``, ``"robot"``, ``1.2`` (float), ``None``.
    Возвращает число в ``[-1000; 1000]`` или ``None``, если вход не парсится.

    Эвристика: дефолтный голос anton ≈ 130 Hz baseline; ``+10%`` →
    ``+13 Hz``, ``high`` (~1.2×) → ``+26 Hz``, ``x-high`` (~1.5×) →
    ``+65 Hz``. Отрицательные аналоги.
    """
    if pitch is None:
        return None
    factor: Optional[float] = None
    if isinstance(pitch, (int, float)):
        factor = float(pitch)
    elif isinstance(pitch, str):
        value = pitch.strip().lower()
        # "robot" у Silero означает спец-эффект, не тон — для Yandex
        # не имеет однозначного Hz-маппинга → None.
        if value == "robot":
            return None
        if value in {"x-low", "low", "medium", "high", "x-high"}:
            mapping = {
                "x-low": 0.5,
                "low": 0.8,
                "medium": 1.0,
                "high": 1.2,
                "x-high": 1.5,
            }
            factor = mapping[value]
        elif value.endswith("%"):
            try:
                factor = 1.0 + float(value[:-1]) / 100.0
            except ValueError:
                return None
        else:
            try:
                factor = float(value)
            except ValueError:
                return None
    else:
        return None
    if factor is None:
        return None
    hz = (factor - 1.0) * YANDEX_BASELINE_PITCH_HZ
    # Clamp в валидный диапазон API.
    return max(-_YANDEX_PITCH_SHIFT_MAX_HZ, min(_YANDEX_PITCH_SHIFT_MAX_HZ, hz))


_SSML_NAMED_VOLUME_TO_DB: dict[str, float] = {
    # SSML стандарт (https://www.w3.org/TR/speech-synthesis/#S3.2.4):
    # silent (-∞, мы приравниваем к -145), x-soft (-12), soft (-6),
    # medium (0), loud (+6), x-loud (+12). Шаг ~6 dB.
    "silent": -145.0,
    "x-soft": -12.0,
    "soft": -6.0,
    "medium": 0.0,
    "loud": 6.0,
    "x-loud": 12.0,
}


def _ssml_volume_to_lufs_target(volume) -> Optional[float]:
    """SSML volume → абсолютная LUFS-цель для Yandex gRPC v3 ``Hints.volume``.

    Yandex ``volume`` — абсолютная LUFS-цель в диапазоне ``[-145; 0)``.
    SSML ``volume`` — относительный уровень (``"loud"`` = +6 dB относительно
    дефолта). Возвращаем абсолютную LUFS-цель, от которой Yandex будет
    нормализовать аудио (clamp в ``[-145; 0)``).

    Поддерживает:
    * числа в dB: ``"+5dB"``, ``"-3dB"``, ``"5"``, ``+5``, ``-3``;
    * проценты: ``"+50%"``, ``"-25%"`` (100% = +6 dB);
    * именованные уровни SSML: ``silent|x-soft|soft|medium|loud|x-loud``.
    """
    if volume is None:
        return None
    if isinstance(volume, (int, float)):
        # Числовое значение — трактуем как dB-offset относительно baseline.
        delta = float(volume)
    elif isinstance(volume, str):
        value = volume.strip().lower()
        if value in _SSML_NAMED_VOLUME_TO_DB:
            delta = _SSML_NAMED_VOLUME_TO_DB[value]
        elif value.endswith("db"):
            try:
                delta = float(value[:-2].strip())
            except ValueError:
                return None
        elif value.endswith("%"):
            try:
                pct = float(value[:-1])
            except ValueError:
                return None
            # 100% = +6 dB (один SSML-шаг «громче»). Логарифмически 6 dB
            # ≈ множитель 2× по амплитуде; для пользователя важнее
            # линейная интерполяция в стопе «loud/soft» шагов.
            delta = pct / 100.0 * 6.0
        else:
            try:
                delta = float(value)
            except ValueError:
                return None
    else:
        return None
    # Переводим смещение в абсолютную LUFS-цель.
    target = YANDEX_BASELINE_VOLUME_LUFS + delta
    # Clamp в валидный диапазон Yandex API: [-145; 0).
    return max(_YANDEX_VOLUME_MIN_LUFS, min(-1.0, target))


def normalize_silero_pitch(pitch) -> str:
    """Привести SSML pitch к уровню, который принимает Silero v5.

    Silero v5 ``apply_tts`` понимает в ``<prosody pitch="...">`` только
    ``x-low|low|medium|high|x-high|robot``. LLM/MiniMax-стиль SSML
    генерирует числовые множители (``1.2``, ``+10%``) — их прямая
    передача роняет Silero с ``Invalid <prosody> tag``, и fallback
    молчит (issue #1064). Здесь любой вход (число, процент, слово,
    мусор) приводится к ближайшему допустимому уровню; нераспознанное
    значение даёт безопасный дефолт ``medium``.

    Args:
        pitch: значение из ``ssml_attributes`` (float, int, str или None).

    Returns:
        Один из ``_SILERO_PITCH_LEVELS``.
    """
    if pitch is None:
        return "medium"
    if isinstance(pitch, str):
        value = pitch.strip().lower()
        if value in _SILERO_PITCH_LEVELS:
            return value
        if value.endswith("%"):
            try:
                factor = 1.0 + float(value[:-1]) / 100.0
            except ValueError:
                return "medium"
        else:
            try:
                factor = float(value)
            except ValueError:
                return "medium"
    elif isinstance(pitch, (int, float)):
        factor = float(pitch)
    else:
        return "medium"

    # Числовой множитель → ближайший уровень (симметрично _parse_ssml_attributes:
    # "high"→1.2, "low"→0.8, "medium"→1.0).
    if factor <= 0.6:
        return "x-low"
    if factor <= 0.85:
        return "low"
    if factor <= 1.15:
        return "medium"
    if factor <= 1.4:
        return "high"
    return "x-high"
