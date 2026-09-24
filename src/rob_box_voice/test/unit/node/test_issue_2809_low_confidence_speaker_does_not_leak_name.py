"""test_issue_2809_low_confidence_speaker_does_not_leak_name.py

Issue #2809 -- незнакомцу робот назвал имя другого диктора (Борис).

Диагноз (PR #2815, docs/plans/2026-09-23-issue-2809-persona-cluster-
diagnosis.md): must_not_say работает как detection (режет фразу с
именем в robot_speech()), но не как prevention -- имя всё равно
доезжает до LLM, потому что speaker_id_node публикует name уже при
confidence выше identify_threshold (0.72), а в полосе выше этого порога
match регулярно указывает на ДРУГОГО реального диктора (живой run
35788126541, шаг n210: confidence=0.763 -> "Борис").

История фикса (важно для ревью):

Первая версия (тот же PR #2818) вводила плоский порог
confident_identify_threshold=0.85: имя публикуется только при
confidence >= 0.85. ОТКАЧЕНА по ревью координатора -- живой хозяин 23.09
(develop 816013f92) даёт 0.708-0.856 за сессию, из них только 1 из 5
узнаваний выше 0.72, и то 0.812 < 0.85: с плоским порогом хозяина не
называли бы по имени никогда, а имя не доехало бы и до vision_face
(ADR-0123 п.6, issue #2771).

Текущая версия -- "зона сомнения + разрыв до другого человека"
(is_name_confident / _gap_to_other_name в speaker_id_node.py):

* confidence >= name_confidence_band_high (0.80) -> имя уверенно;
* иначе, если есть кандидат с ДРУГИМ именем и разрыв до него
  (best.confidence - candidate.confidence) >= name_confidence_min_gap
  (0.15) -> имя (явно не тот другой человек);
* иначе -> tentative (имя-гипотеза, не факт): не в user_input
  ([Speaker:tentative] вместо [Spkr:...]), не в system_context
  (payload name=None -> _build_dynamic_system_context сама уходит в
  существующую unknown-ветку с privacy_note, issue #2779).

Два и более профиля ОДНОГО человека под одним именем (issue #2747,
живой Дэнчик -- 1ae4b0ac + c9e981cb) -- не конкуренты: разрыв между ними
не в счёт, _gap_to_other_name их пропускает.

Источники калибровки чисел 0.80 / 0.15 (эмпирика):

* Resemblyzer/GE2E: свой голос обычно cos 0.8-0.95, чужой 0.3-0.6,
  0.6-0.8 -- зона неопределённости; надёжность падает на клипах короче
  ~2.6s (CEUR Vol-4164 paper7, https://ceur-ws.org/Vol-4164/paper7.pdf).
  Сокращение тестовой речи 3.6s -> 2.05s даёт +46% EER
  (https://arxiv.org/abs/1810.10884).
* Сырой косинус resemblyzer не калиброван (апстрим issue #42:
  разнополые голоса дали 0.88 --
  https://github.com/resemble-ai/Resemblyzer/issues/42).
* Open-set identification: ложные срабатывания растут с числом
  зарегистрированных, стандартное средство -- нормализация по когорте
  чужих голосов (AS-norm/top-norm, VoxWatch,
  https://arxiv.org/abs/2307.00169). В это решение НЕ включена --
  сложность признана избыточной для этого PR, см. следующий шаг в
  докстринге speaker_id_node._log_identify_candidates.

Тест не поднимает ROS2 (тот же приём, что test_epithet_wiring.py и
test_issue_1195_tg_source.py -- узлы собираются через object.__new__).
"""

from __future__ import annotations

import asyncio
import sys
import threading
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
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402
from rob_box_voice.utils.speaker_embeddings import SpeakerMatch  # noqa: E402

BAND_HIGH = 0.80
MIN_GAP = 0.15


def _match(speaker_id, name, confidence, epithet=None):
    return SpeakerMatch(
        speaker_id=speaker_id, name=name, confidence=confidence, epithet=epithet
    )


class TestGapToOtherName:
    def test_no_candidates_besides_best_returns_none(self):
        best = _match("boris-1", "Boris", 0.78)
        assert sid_node._gap_to_other_name(best, [best]) is None

    def test_same_name_duplicate_profile_is_not_a_competitor(self):
        best = _match("1ae4b0ac", "Denchik", 0.771)
        twin = _match("c9e981cb", "Denchik", 0.60)
        assert sid_node._gap_to_other_name(best, [best, twin]) is None

    def test_different_name_candidate_gives_gap(self):
        best = _match("boris-1", "Boris", 0.780)
        other = _match("sasha-1", "Sasha", 0.653)
        gap = sid_node._gap_to_other_name(best, [best, other])
        assert gap == pytest.approx(0.780 - 0.653)

    def test_skips_duplicate_before_finding_real_competitor(self):
        best = _match("1ae4b0ac", "Denchik", 0.80)
        twin = _match("c9e981cb", "Denchik", 0.75)
        other = _match("boris-1", "Boris", 0.60)
        gap = sid_node._gap_to_other_name(best, [best, twin, other])
        assert gap == pytest.approx(0.80 - 0.60)


REPLAY_DATASET = [
    (
        "n210_grisha_zahar_vs_boris",
        0.780, 0.653, False,
        "n210: best=Boris 0.780, second=Sasha 0.653, gap=0.127 < 0.15 -> tentative",
    ),
    (
        "n204_boris_vs_sasha_no_competitor",
        0.723, None, False,
        "n204: Sasha 0.723, no second candidate -> band, no competitor -> tentative",
    ),
    ("synth_boris_1", 0.909, 0.909 - 0.200, True, "band_high"),
    ("synth_boris_2", 0.871, 0.871 - 0.156, True, "band_high"),
    ("synth_boris_3", 0.933, 0.933 - 0.240, True, "band_high"),
    ("synth_boris_4", 0.904, 0.904 - 0.178, True, "band_high"),
    ("synth_boris_5", 0.886, 0.886 - 0.169, True, "band_high"),
    ("synth_boris_6", 0.871, 0.871 - 0.164, True, "band_high"),
    ("synth_boris_7", 0.836, 0.836 - 0.132, True, "band_high"),
    ("synth_sasha_1_alone", 0.929, None, True, "band_high, no competitor"),
    ("synth_sasha_2_alone", 0.931, None, True, "band_high, no competitor"),
    ("synth_sasha_3_alone", 0.894, None, True, "band_high, no competitor"),
    ("synth_sasha_4_alone", 0.961, None, True, "band_high, no competitor"),
    ("synth_sasha_5_alone", 0.920, None, True, "band_high, no competitor"),
    ("synth_sasha_6_alone", 0.932, None, True, "band_high, no competitor"),
    ("synth_sasha_7", 0.919, 0.919 - 0.204, True, "band_high"),
    ("synth_sasha_8", 0.911, 0.911 - 0.198, True, "band_high"),
    ("synth_sasha_9", 0.917, 0.917 - 0.217, True, "band_high"),
    (
        "live_denchik_0771_no_competitor",
        0.771, None, False,
        "band, competitor is same person (twin profile) -> tentative",
    ),
    (
        "live_denchik_0757_no_competitor",
        0.757, None, False,
        "same case, different utterance -> tentative",
    ),
    (
        "live_denchik_0856_short_utterance",
        0.856, None, True,
        "band_high; speech only 0.51s, duration is NOT part of this decision",
    ),
    (
        "live_denchik_0812_morning",
        0.812, None, True,
        "band_high (morning utterance, same session)",
    ),
    (
        "constructed_band_with_wide_gap",
        0.75, 0.55, False,
        "constructed (not in 23.09 live data): band, gap=0.20 -- issue #2889: "
        "gap no longer promotes to name (stranger Gena 0.777/0.611 was named)",
    ),
]


@pytest.mark.parametrize(
    "label,best_score,other_score,expected,note",
    REPLAY_DATASET,
    ids=[row[0] for row in REPLAY_DATASET],
)
def test_is_name_confident_replay(label, best_score, other_score, expected, note):
    best = _match("best-id", "Boris", best_score)
    candidates = [best]
    if other_score is not None:
        candidates.append(_match("other-id", "Sasha", other_score))
    result = sid_node.is_name_confident(
        best, candidates, band_high=BAND_HIGH, min_gap=MIN_GAP
    )
    assert result is expected, f"{label}: {note}"


class TestIsNameConfidentEdgeCases:
    def test_no_match_is_never_confident(self):
        assert (
            sid_node.is_name_confident(None, [], band_high=BAND_HIGH, min_gap=MIN_GAP)
            is False
        )

    def test_duplicate_profiles_of_same_person_do_not_count_as_gap(self):
        best = _match("1ae4b0ac", "Denchik", 0.75)
        twin = _match("c9e981cb", "Denchik", 0.40)
        assert (
            sid_node.is_name_confident(
                best, [best, twin], band_high=BAND_HIGH, min_gap=MIN_GAP
            )
            is False
        )


@pytest.fixture()
def sid():
    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._name_confidence_band_high = BAND_HIGH
    instance._name_confidence_min_gap = MIN_GAP
    instance._result_pub = MagicMock()
    instance.get_logger = MagicMock(return_value=MagicMock())
    return instance


def _published_payload(pub_mock):
    import json

    assert pub_mock.publish.call_count == 1
    msg = pub_mock.publish.call_args.args[0]
    return json.loads(msg.data)


class TestPublishResultNameDecisionWiring:
    def test_tentative_single_decision_suppresses_name(self, sid):
        match = SpeakerMatch(
            speaker_id="0ddc1ab9-9af0-4c00-8000-000000000000",
            name="Boris",
            confidence=0.780,
            epithet="Sobesednik",
        )
        sid._publish_result(match, name_decision=sid_node.NAME_TENTATIVE_SINGLE)

        payload = _published_payload(sid._result_pub)
        assert payload["is_known"] is True
        assert payload["name"] is None
        assert payload["speaker_id"] == match.speaker_id
        assert payload["confidence"] == pytest.approx(0.780)
        assert payload["epithet"] == "Sobesednik"

    def test_confident_decision_publishes_name(self, sid):
        match = SpeakerMatch(speaker_id="known-1", name="Boris", confidence=0.91)
        sid._publish_result(match, name_decision=sid_node.NAME_CONFIDENT)

        payload = _published_payload(sid._result_pub)
        assert payload["name"] == "Boris"

    def test_register_source_bypasses_name_decision(self, sid):
        match = SpeakerMatch(speaker_id="new-1", name="Grisha", confidence=0.5)
        sid._publish_result(match, source="register", name_decision=sid_node.NAME_TENTATIVE_SINGLE)

        payload = _published_payload(sid._result_pub)
        assert payload["name"] == "Grisha"
        assert payload["source"] == "register"

    def test_name_decision_none_falls_back_to_band_high_only(self, sid):
        below = SpeakerMatch(speaker_id="x", name="Boris", confidence=0.780)
        above = SpeakerMatch(speaker_id="y", name="Boris", confidence=0.91)

        sid._publish_result(below, name_decision=None)
        assert _published_payload(sid._result_pub)["name"] is None

        sid._result_pub.reset_mock()
        sid._publish_result(above, name_decision=None)
        assert _published_payload(sid._result_pub)["name"] == "Boris"

    def test_unknown_speaker_untouched(self, sid):
        sid._publish_result(None)
        payload = _published_payload(sid._result_pub)
        assert payload == {"is_known": False}


class TestPublishResultTentativeFields:
    """Issue #2809 (продолжение) -- tentative_name/tentative_conf/
    tentative_kind в payload /voice/speaker/result, для dialogue_node
    (переспрос) и как гарантия для vision_face_node (ADR-0123 п.6,
    issue #2771): ``name`` остаётся None, пока не подтверждено -- лицо
    не должно слиться по гипотезе."""

    def test_single_decision_carries_name_hypothesis(self, sid):
        match = SpeakerMatch(
            speaker_id="1ae4b0ac-0000-0000-0000-000000000000",
            name="Denchik",
            confidence=0.771,
        )
        sid._publish_result(match, name_decision=sid_node.NAME_TENTATIVE_SINGLE)

        payload = _published_payload(sid._result_pub)
        assert payload["is_known"] is True
        assert payload["name"] is None  # НИКОГДА не факт, пока не подтверждено
        assert payload["tentative_name"] == "Denchik"
        assert payload["tentative_conf"] == pytest.approx(0.771)
        assert payload["tentative_kind"] == "single"

    def test_contested_decision_never_carries_a_candidate_name(self, sid):
        """n210: голос похож на Бориса и Сашу одновременно -- ни одно из
        двух имён не должно уйти дальше даже как гипотеза."""
        match = SpeakerMatch(
            speaker_id="boris-id", name="Boris", confidence=0.780
        )
        sid._publish_result(match, name_decision=sid_node.NAME_TENTATIVE_CONTESTED)

        payload = _published_payload(sid._result_pub)
        assert payload["name"] is None
        assert "tentative_name" not in payload
        assert payload["tentative_kind"] == "contested"
        assert payload["tentative_conf"] == pytest.approx(0.780)
        # НИ имени "Boris" (реальное имя match), ни какого-либо другого
        # имени не должно быть НИГДЕ в payload -- сериализуем и проверяем
        # весь JSON целиком, не только известные ключи.
        import json as _json

        raw = _json.dumps(payload, ensure_ascii=False)
        assert "Boris" not in raw

    def test_confident_decision_has_no_tentative_fields(self, sid):
        match = SpeakerMatch(speaker_id="known-1", name="Boris", confidence=0.91)
        sid._publish_result(match, name_decision=sid_node.NAME_CONFIDENT)

        payload = _published_payload(sid._result_pub)
        assert "tentative_name" not in payload
        assert "tentative_conf" not in payload
        assert "tentative_kind" not in payload


class TestClassifyNameConfidenceThreeWay:
    """Issue #2809 (продолжение) -- classify_name_confidence различает
    single/contested (is_name_confident этого не делает, только да/нет)."""

    def test_single_when_no_competitor_with_different_name(self):
        best = SpeakerMatch(speaker_id="a", name="Denchik", confidence=0.75)
        result = sid_node.classify_name_confidence(
            best, [best], band_high=0.80, min_gap=0.15
        )
        assert result == sid_node.NAME_TENTATIVE_SINGLE

    def test_contested_when_competitor_gap_too_small(self):
        best = SpeakerMatch(speaker_id="a", name="Boris", confidence=0.780)
        other = SpeakerMatch(speaker_id="b", name="Sasha", confidence=0.653)
        result = sid_node.classify_name_confidence(
            best, [best, other], band_high=0.80, min_gap=0.15
        )
        assert result == sid_node.NAME_TENTATIVE_CONTESTED

    def test_confident_above_band_high_regardless_of_competitor(self):
        best = SpeakerMatch(speaker_id="a", name="Boris", confidence=0.91)
        other = SpeakerMatch(speaker_id="b", name="Sasha", confidence=0.85)
        result = sid_node.classify_name_confidence(
            best, [best, other], band_high=0.80, min_gap=0.15
        )
        assert result == sid_node.NAME_CONFIDENT

    def test_wide_gap_in_band_is_contested_not_confident(self):
        # Issue #2889: разрыв до другого имени в полосе больше не делает
        # имя уверенным (незнакомец Гена 0.777 vs 0.611 был назван Борисом).
        best = SpeakerMatch(speaker_id="a", name="Boris", confidence=0.75)
        other = SpeakerMatch(speaker_id="b", name="Sasha", confidence=0.55)
        result = sid_node.classify_name_confidence(
            best, [best, other], band_high=0.80, min_gap=0.15
        )
        assert result == sid_node.NAME_TENTATIVE_CONTESTED

    def test_duplicate_profile_of_same_person_yields_single_not_contested(self):
        best = SpeakerMatch(speaker_id="1ae4b0ac", name="Denchik", confidence=0.75)
        twin = SpeakerMatch(speaker_id="c9e981cb", name="Denchik", confidence=0.40)
        result = sid_node.classify_name_confidence(
            best, [best, twin], band_high=0.80, min_gap=0.15
        )
        assert result == sid_node.NAME_TENTATIVE_SINGLE


@pytest.fixture()
def node():
    n = object.__new__(DialogueNode)
    n.get_logger = MagicMock(return_value=MagicMock())
    n._speaker_lock = threading.Lock()
    n._publish_speaker_observation = MagicMock()
    return n


def _run(coro):
    return asyncio.run(coro)


class TestApplySpeakerIdentityTentative:
    def test_name_none_tentative_yields_tentative_tag_not_name(self, node):
        node._current_speaker = {
            "is_known": True,
            "speaker_id": "0ddc1ab9-9af0-4c00-8000-000000000000",
            "name": None,
            "confidence": 0.780,
            "epithet": "Sobesednik",
        }

        result = _run(
            node._apply_speaker_identity(
                "ya mimo shel. imya svoe ya tebe nazyvat ne budu.",
                speaker_context=None,
            )
        )

        assert "Boris" not in result
        assert "[Spkr:" not in result
        assert "[Speaker:tentative]" in result
        node._publish_speaker_observation.assert_called_once()

    def test_confident_known_speaker_still_gets_name_tag(self, node):
        node._current_speaker = {
            "is_known": True,
            "speaker_id": "known-1",
            "name": "Anton",
            "confidence": 0.93,
        }

        result = _run(
            node._apply_speaker_identity("privet", speaker_context=None)
        )

        assert "[Spkr:Anton]" in result

    def test_unknown_speaker_still_gets_unknown_tag(self, node):
        node._current_speaker = {"is_known": False}

        result = _run(
            node._apply_speaker_identity("privet", speaker_context=None)
        )

        assert "[Speaker:unknown]" in result
