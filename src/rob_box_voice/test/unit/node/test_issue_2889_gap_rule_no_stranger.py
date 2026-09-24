"""Issue #2889 — правило разрыва не должно называть незнакомца чужим именем.

E2E акт 2b, run 35903232434, шаг n709 (незнакомец «Гена», голос zahar):

    identify candidates: best='Борис'(3d97f592) score=0.777 |
        second='Саша'(5baad325) score=0.611 | gap=0.166 | речь=3.72s
    📢 Publishing: is_known=true name='Борис' epithet='Гость' conf=0.777
    ACCEPTANCE[n709_gena_intro]: ❌ forbidden phrases spoken by robot: ['Борис']

``classify_name_confidence`` (PR #2818) делал имя уверенным при
``gap >= name_confidence_min_gap`` (0.15) для любого score в полосе
``[identify_threshold, band_high)``. Разрыв различает людей ИЗ галереи
между собой, но про незнакомца (которого в галерее нет) не говорит ничего:
0.166 значит «не Саша», а не «Борис».

Граница (выбрана по цифрам, identify_threshold 0.72 не тронут):
в полосе ниже band_high разноимённый конкурент → ``contested`` при ЛЮБОМ
разрыве (ни имени, ни гипотезы с именем). Не ``single``: сценарий акта 2b
запрещает «Борис» в речи робота на n709 целиком (must_not_say), а
``single`` = вопрос «Борис, это ты?» — то же имя вслух незнакомцу.

Почему это не отнимает имя у живого хозяина из #2809: ни одна живая или
синтетическая точка из калибровки 23.09 не опиралась на правило разрыва —
живой «Дэнчик» 0.856/0.812 уверен по band_high (0.80), 0.771/0.757 — single
(конкурентов с другим именем нет, второй профиль — дубль его же), синтетика
«своя» 0.836–0.961 — вся выше 0.80. Единственная точка «confident через
разрыв» в тестах была сконструированной (0.75 vs 0.55), не живой.

Тест не поднимает ROS2 (``object.__new__``, как в соседних тестах #2809).
"""

from __future__ import annotations

import json
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
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

DEFAULT_BAND_HIGH = 0.80  # declare_parameter name_confidence_band_high
ACT_2B_BAND_HIGH = 0.99  # node_params акта 2b
MIN_GAP = 0.15  # declare_parameter name_confidence_min_gap

BORIS_ID = "3d97f592-0000-0000-0000-000000000000"
SASHA_ID = "5baad325-0000-0000-0000-000000000000"


def _gena_candidates():
    """Цифры n709 из лога run 35903232434."""
    best = SpeakerMatch(speaker_id=BORIS_ID, name="Борис", confidence=0.777, epithet="Гость")
    second = SpeakerMatch(speaker_id=SASHA_ID, name="Саша", confidence=0.611)
    return best, [best, second]


def _classify(best, candidates, band_high, min_gap=MIN_GAP):
    return sid_node.classify_name_confidence(
        best, candidates, band_high=band_high, min_gap=min_gap
    )


# ---------------------------------------------------------------------------
# 1. Сам случай #2889 — падает на develop, проходит на ветке.
# ---------------------------------------------------------------------------


class TestStrangerGenaIsNotNamed:
    def test_gap_is_the_one_from_the_log(self):
        best, candidates = _gena_candidates()
        gap = sid_node._gap_to_other_name(best, candidates)
        assert gap == pytest.approx(0.166)
        assert gap >= MIN_GAP  # именно поэтому прежнее правило сработало

    @pytest.mark.parametrize("band_high", [ACT_2B_BAND_HIGH, DEFAULT_BAND_HIGH])
    def test_classified_as_contested_not_confident(self, band_high):
        best, candidates = _gena_candidates()
        assert _classify(best, candidates, band_high) == sid_node.NAME_TENTATIVE_CONTESTED

    @pytest.mark.parametrize("band_high", [ACT_2B_BAND_HIGH, DEFAULT_BAND_HIGH])
    def test_is_name_confident_false(self, band_high):
        best, candidates = _gena_candidates()
        assert (
            sid_node.is_name_confident(best, candidates, band_high=band_high, min_gap=MIN_GAP)
            is False
        )

    @pytest.mark.parametrize("min_gap", [0.05, 0.10, 0.15, 0.166, 0.20])
    def test_min_gap_does_not_promote_band_score(self, min_gap):
        """Разрыв в полосе больше не решает исход ни при каком min_gap."""
        best, candidates = _gena_candidates()
        assert (
            _classify(best, candidates, DEFAULT_BAND_HIGH, min_gap=min_gap)
            == sid_node.NAME_TENTATIVE_CONTESTED
        )


@pytest.fixture(params=[ACT_2B_BAND_HIGH, DEFAULT_BAND_HIGH], ids=["band_high_0.99", "band_high_0.80"])
def node(request):
    """Узел без ROS2; БД и эмбеддер подменены цифрами n709."""
    best, candidates = _gena_candidates()
    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._name_confidence_band_high = request.param
    instance._name_confidence_min_gap = MIN_GAP
    instance._sample_rate = 16000
    instance._recent_embeddings = []
    instance._pending_register_lock = MagicMock()
    instance._pending_register_name = None
    instance._result_pub = MagicMock()
    instance.get_logger = MagicMock(return_value=MagicMock())
    instance._log_identify_candidates = MagicMock()
    instance._apply_growth_session = MagicMock()
    instance._inconclusive_reason = MagicMock(return_value=None)
    embed = MagicMock()
    embed.embedding = np.ones(256, dtype=np.float32)
    embed.voiced_sec = 3.72
    instance._db = MagicMock()
    instance._db.embed_audio_ex.return_value = embed
    instance._db.identify.return_value = best
    instance._db.identify_candidates.return_value = candidates
    return instance


def test_process_utterance_publishes_no_name_for_gena(node):
    """Весь путь identify → classify → /voice/speaker/result: ни «Борис»,
    ни «Саша» в payload нет ни в каком поле (dialogue_node получает
    contested и задаёт максимум нейтральный «Как тебя зовут?»)."""
    node._process_utterance(b"\x00\x00" * 16000, utterance_id="utt-n709")

    assert node._result_pub.publish.call_count == 1
    payload = json.loads(node._result_pub.publish.call_args.args[0].data)
    assert payload["is_known"] is True
    assert payload["name"] is None
    assert payload["tentative_kind"] == sid_node.NAME_TENTATIVE_CONTESTED
    assert "tentative_name" not in payload
    raw = json.dumps(payload, ensure_ascii=False)
    assert "Борис" not in raw
    assert "Саш" not in raw


# ---------------------------------------------------------------------------
# 2. Кейсы #2809/#2818 — должны вести себя как задумано (зелёные и на
#    develop, и на ветке: фикс их не трогает).
# ---------------------------------------------------------------------------


class TestIssue2809CasesUnchanged:
    @pytest.mark.parametrize("score", [0.856, 0.812])
    def test_live_owner_confident_by_band_high(self, score):
        """Живой «Дэнчик» 23.09: 0.856/0.812 — имя (>= 0.80)."""
        best = SpeakerMatch(speaker_id="1ae4b0ac", name="Дэнчик", confidence=score)
        twin = SpeakerMatch(speaker_id="c9e981cb", name="Дэнчик", confidence=score - 0.1)
        assert _classify(best, [best, twin], DEFAULT_BAND_HIGH) == sid_node.NAME_CONFIDENT

    @pytest.mark.parametrize("score", [0.771, 0.757])
    def test_live_owner_in_band_keeps_named_question(self, score):
        """Живой «Дэнчик» 0.771/0.757: второй кандидат — его же дубль-профиль,
        разноимённого конкурента нет → single («Дэнчик, это ты?»), имя-гипотеза
        доезжает до dialogue_node."""
        best = SpeakerMatch(speaker_id="1ae4b0ac", name="Дэнчик", confidence=score)
        twin = SpeakerMatch(speaker_id="c9e981cb", name="Дэнчик", confidence=0.40)
        assert _classify(best, [best, twin], DEFAULT_BAND_HIGH) == sid_node.NAME_TENTATIVE_SINGLE

    def test_n210_impostor_stays_contested(self):
        """n210: Борис 0.780 vs Саша 0.653, gap 0.127 — contested, как было."""
        best = SpeakerMatch(speaker_id=BORIS_ID, name="Борис", confidence=0.780)
        other = SpeakerMatch(speaker_id=SASHA_ID, name="Саша", confidence=0.653)
        assert _classify(best, [best, other], DEFAULT_BAND_HIGH) == sid_node.NAME_TENTATIVE_CONTESTED

    @pytest.mark.parametrize("score", [0.836, 0.871, 0.909, 0.933, 0.961])
    def test_synthetic_own_voice_confident_even_with_competitor(self, score):
        """Синтетика «своя» 23.09: 0.836–0.961 — выше band_high, имя при любом конкуренте."""
        best = SpeakerMatch(speaker_id=BORIS_ID, name="Борис", confidence=score)
        other = SpeakerMatch(speaker_id=SASHA_ID, name="Саша", confidence=score - 0.13)
        assert _classify(best, [best, other], DEFAULT_BAND_HIGH) == sid_node.NAME_CONFIDENT

    def test_act_2b_sasha_before_boris_registered_is_single(self):
        """Акт 2b n702: Саша 0.928 при band_high 0.99, Бориса в базе ещё нет →
        single, вопрос «Саша, это ты?» (n703/n706) не пропадает."""
        best = SpeakerMatch(speaker_id=SASHA_ID, name="Саша", confidence=0.928)
        assert _classify(best, [best], ACT_2B_BAND_HIGH) == sid_node.NAME_TENTATIVE_SINGLE
