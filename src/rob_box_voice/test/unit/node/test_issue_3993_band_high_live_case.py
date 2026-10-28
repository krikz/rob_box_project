"""Issue #3993 (а) — «спорное» имя при score 0.944 и gap 0.283: классификация верна, спор дал форсированный порог.

Лог марафона 23.10 (``artifacts_live_check_2026-10-23/m210648/va_marathon.log``, время UTC):

    21:29:59.098 🔄 [2809/2828] name_confidence_band_high -> 0.99    <- node_params акта 2b
    21:30:14.461 identify candidates: best='Саша'(429eae2c) score=0.944 |
                 second='Борис'(6eb1a5c6) score=0.661 | gap=0.283
    21:30:14.463 Publishing: is_known=true name=None ... (name suppressed: contested, issue #2809)
    21:36:10.174 🔄 [2809/2828] name_confidence_band_high -> 0.8     <- акт 2b вернул порог
    21:52:56.316 identify candidates: best='Саша'(429eae2c) score=0.944 |
                 second='Борис'(6eb1a5c6) score=0.653 | gap=0.291
    21:52:56.317 Publishing: is_known=true name='Саша' epithet='Спутник' conf=0.944

0.944 при боевом ``band_high`` 0.80 - уверенное имя; ``contested`` только потому, что акт 2b поднял порог до 0.99
(после #2889 любой разноимённый конкурент в полосе - спор). Классификацию не трогаем, тест держит оба исхода.
Тест не поднимает ROS2 (как соседние тесты #2809/#2889).
"""

from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_voice import speaker_id_node as sid_node  # noqa: E402
from rob_box_voice.utils.speaker_embeddings import SpeakerMatch  # noqa: E402

DEFAULT_BAND_HIGH = (
    0.80  # declare_parameter name_confidence_band_high, config/speaker_id_node.yaml
)
ACT_2B_BAND_HIGH = 0.99  # node_params акта 2b марафона 23.10
MIN_GAP = 0.15


def _live_2b():
    """Цифры 21:30:14.461 из лога марафона 23.10."""
    best = SpeakerMatch(
        speaker_id="429eae2c", name="Саша", confidence=0.944, epithet="Спутник"
    )
    second = SpeakerMatch(speaker_id="6eb1a5c6", name="Борис", confidence=0.661)
    return best, [best, second]


def _classify(band_high):
    best, candidates = _live_2b()
    return sid_node.classify_name_confidence(
        best, candidates, band_high=band_high, min_gap=MIN_GAP
    )


def test_live_gap_is_the_one_from_the_log():
    best, candidates = _live_2b()
    assert sid_node._gap_to_other_name(best, candidates) == pytest.approx(0.283)


def test_score_094_is_a_confident_name_at_the_production_threshold():
    """Боевой порог: 0.944 - имя «Саша» звучит (так и было в 21:52:56 после возврата порога)."""
    assert _classify(DEFAULT_BAND_HIGH) == sid_node.NAME_CONFIDENT


def test_contested_came_from_the_forced_act_2b_threshold():
    """Порог 0.99 акта 2b: 0.944 в полосе, конкурент с другим именем - спор при любом разрыве (#2889)."""
    assert _classify(ACT_2B_BAND_HIGH) == sid_node.NAME_TENTATIVE_CONTESTED


def test_production_config_keeps_band_high_080():
    config = Path(__file__).resolve().parents[3] / "config" / "speaker_id_node.yaml"
    lines = [ln.strip() for ln in config.read_text(encoding="utf-8").splitlines()]
    assert f"name_confidence_band_high: {DEFAULT_BAND_HIGH:.2f}" in lines
