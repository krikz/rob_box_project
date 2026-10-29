"""Стартовая фраза по готовности узлов (#3913; перенос v1 #1003 в #3912): вердикт и выбор фразы - код (ADR-0148)."""

from __future__ import annotations

import random
import types

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.startup import (LLM_PART, NodeReport, Readiness, ReadinessWatch, assess, enumerate_parts,
                                    llm_usable, pick_startup_phrase)

VOICE = ["/audio_node", "/stt_node", "/tts_node"]
REST = ["/context_aggregator", "/camera/camera", "/lslidar_driver_node", "/rtabmap/rtabmap"]


def _report(active=(), failed=(), missing=()) -> Readiness:
    return Readiness(NodeReport.from_lists(active, failed, missing), None)


def test_everything_up_is_ready_and_the_phrase_says_ready_to_work() -> None:
    r = _report(active=["/dialog_node", *VOICE, *REST])
    verdict = assess(r)
    assert verdict.kind == "ready" and verdict.down == ()
    for seed in range(20):
        text, v = pick_startup_phrase(r, rng=random.Random(seed))
        assert v.kind == "ready" and text in phrases.catalog()["startup.ready"]


def test_ready_catalog_has_explicit_ready_to_work_variants() -> None:
    assert 6 <= len(phrases.catalog()["startup.ready"]) <= 10
    assert sum("готов к работе" in p.lower() for p in phrases.catalog()["startup.ready"]) >= 3


def test_only_voice_up_is_voice_only_and_names_what_is_down() -> None:
    r = _report(active=["/dialog_node", *VOICE], missing=REST)
    verdict = assess(r)
    assert verdict.kind == "voice_only"
    assert verdict.down == ("context", "camera", "lidar", "slam")
    text, _ = pick_startup_phrase(r, rng=random.Random(1))
    assert "только голос" in text.lower()
    assert "сбор контекста, камера, лидар и карта и навигация" in text
    assert "готов к работе" not in text.lower()


def test_some_rest_up_is_partial_not_voice_only() -> None:
    r = _report(active=[*VOICE, "/camera/camera"], missing=["/lslidar_driver_node"], failed=["/rtabmap/rtabmap"])
    verdict = assess(r)
    assert verdict.kind == "partial" and verdict.down == ("slam", "lidar")  # failed раньше missing - порядок отчёта
    text, _ = pick_startup_phrase(r, rng=random.Random(2))
    assert "лидар" in text and "только голос" not in text.lower() and "готов к работе" not in text.lower()


def test_voice_node_down_is_partial_even_when_rest_is_up() -> None:
    verdict = assess(_report(active=["/audio_node", "/tts_node", *REST], missing=["/stt_node"]))
    assert verdict.kind == "partial" and verdict.down == ("stt",)


def test_nothing_up_is_partial_listing_even_voice_not_voice_only() -> None:
    verdict = assess(_report(missing=[*VOICE, *REST]))
    assert verdict.kind == "partial" and "stt" in verdict.down


def test_dialog_node_and_alternatives_label_are_skipped() -> None:
    r = _report(active=VOICE + REST, missing=["/dialog_node|/dialogue_node"])
    assert assess(r).kind == "ready"


def test_unknown_node_is_named_generically_not_by_ros_name() -> None:
    r = _report(active=VOICE, missing=["/some/new_node"])
    assert assess(r).down == ("other",)
    text, _ = pick_startup_phrase(r, rng=random.Random(0))
    assert "/some" not in text and "другие системы" in text


def test_no_report_is_unknown_and_honest() -> None:
    text, verdict = pick_startup_phrase(Readiness(None, True), rng=random.Random(3))
    assert verdict.kind == "unknown" and text in phrases.catalog()["startup.unknown"]
    assert "готов к работе" not in text.lower() and "в норме" not in text.lower()


def test_llm_down_blocks_ready_and_is_named() -> None:
    r = Readiness(NodeReport.from_lists(VOICE + REST, (), ()), llm_ok=False)
    verdict = assess(r)
    assert verdict.kind == "partial" and verdict.down == (LLM_PART,)
    text, _ = pick_startup_phrase(r, rng=random.Random(4))
    assert "языковая модель" in text


def test_llm_unknown_does_not_block_ready() -> None:
    assert assess(Readiness(NodeReport.from_lists(VOICE + REST, (), ()), llm_ok=None)).kind == "ready"


def test_llm_usable_reads_only_llm_kind_and_usable_health() -> None:
    def st(kind, health):
        return types.SimpleNamespace(kind=kind, health=health)

    assert llm_usable([]) is None
    assert llm_usable([st("tts", "down")]) is None
    assert llm_usable([st("llm", "quota"), st("llm", "degraded")]) is True
    assert llm_usable([st("llm", "quota"), st("llm", "auth")]) is False
    assert llm_usable([st("llm", "down"), st("tts", "ok")]) is False


def test_operator_text_wins_stripped_and_keeps_verdict() -> None:
    text, verdict = pick_startup_phrase(_report(missing=REST, active=VOICE), "  Привет!  ")
    assert text == "Привет!" and verdict.kind == "voice_only"
    text, _ = pick_startup_phrase(Readiness(None, None), "   ", rng=random.Random(0))
    assert text in phrases.catalog()["startup.unknown"]


def test_enumerate_parts() -> None:
    assert enumerate_parts(()) == ""
    assert enumerate_parts(("camera",)) == "камера"
    assert enumerate_parts(("camera", "lidar")) == "камера и лидар"
    assert enumerate_parts(("camera", "lidar", "llm")) == "камера, лидар и языковая модель"


def test_every_part_key_of_the_node_table_is_in_the_catalog() -> None:
    cat = phrases.catalog()
    for part in {*kn.STARTUP_NODE_PARTS.values(), LLM_PART, "other"}:
        assert f"startup.part.{part}" in cat
    assert set(kn.STARTUP_VOICE_NODES) <= set(kn.STARTUP_NODE_PARTS)


def test_watch_drops_a_stale_report_to_unknown() -> None:
    watch = ReadinessWatch(stale_s=15.0)
    assert watch.snapshot(0.0).nodes is None  # отчёта ещё нет
    report = NodeReport.from_lists(VOICE, (), ())
    watch.observe(report, now=100.0)
    assert watch.snapshot(110.0, True) == Readiness(report, True)
    assert watch.snapshot(116.0).nodes is None  # агрегатор замолчал - не факт


@pytest.mark.parametrize("kind", ["ready", "voice_only", "partial", "unknown"])
def test_all_verdict_keys_are_in_catalog(kind) -> None:
    variants = phrases.catalog()[f"startup.{kind}"]
    assert 1 <= len(variants) <= 10


def test_timings_are_v1_and_wait_is_bounded() -> None:
    assert kn.STARTUP_GREETING_DELAY_S == 12.0
    assert kn.STARTUP_GREETING_STEPS_S == (2.0, 1.5)
    assert kn.STARTUP_READY_TIMEOUT_S > kn.STARTUP_GREETING_DELAY_S
    assert kn.STARTUP_READY_POLL_S > 0
