"""Замер понимания фразы (карточка #4163, этап 1): корпус, метрики, прогон на фейковом провайдере, сверка с живым логом,
классификатор Jev. LLM и Jev - фейки; цифры фейка ничего не говорят о модели, тесты проверяют обвязку."""

from __future__ import annotations

import asyncio
import json
import sys
from pathlib import Path

import pytest

pytest.importorskip("openai", reason="сухому стенду нужен настоящий мост команд, а ему - клиент openai")

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "dialog"))
sys.path.insert(0, str(REPO / "scripts" / "dialog" / "understanding_corpus"))

import build_marathon  # noqa: E402
import dry_marathon as dm  # noqa: E402
import env_files  # noqa: E402
import jev_router  # noqa: E402
import live_compare  # noqa: E402
import understanding_measure as um  # noqa: E402
import understanding_metrics as met  # noqa: E402
import understanding_variants as uv  # noqa: E402
from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_harness.decision.types import ChoiceAnswer, Decision  # noqa: E402


# ── Корпус ────────────────────────────────────────────────────────────────────────────────────────────────────────


@pytest.fixture(scope="module")
def corpus():
    return um.load_corpus()


def test_corpus_counts_and_labels_follow_classify_step(corpus):
    assert corpus["counts"] == {"steps": 132, "scored_text": 90, "acoustic_context": 42, "unique_phrases": 130}
    for item in corpus_steps(corpus):
        kind, reasons = dm.classify_step(item["step"])
        assert item["kind"] == kind and item["scored"] == (kind == dm.TEXT) and item["acoustic_reasons"] == reasons


def corpus_steps(corpus):
    return [s for a in corpus["acts"] for s in a["steps"]]


def test_corpus_ids_are_unique_and_acceptance_files_are_not_steps(corpus):
    ids = [s["id"] for s in corpus_steps(corpus)]
    assert len(ids) == len(set(ids))
    assert not any("_acceptance_" in name for name in corpus["scenarios"]["sha256"])


def test_corpus_steps_carry_no_scenario_comments(corpus):
    for item in corpus_steps(corpus):
        assert not [k for k in item["step"] if k.startswith("_")]
        assert not [k for k in (item["step"].get("acceptance") or {}) if k.startswith("_")]


def test_corpus_hash_catches_hand_edits(corpus, tmp_path):
    assert um.build_marathon.digest(corpus["acts"]) == corpus["corpus_sha256"]
    broken = json.loads(json.dumps(corpus))
    broken["acts"][0]["steps"][0]["step"]["text"] += " (правка руками)"
    path = tmp_path / "marathon.json"
    path.write_text(json.dumps(broken, ensure_ascii=False), encoding="utf-8")
    with pytest.raises(SystemExit):
        um.load_corpus(path)


def test_corpus_phrases_are_the_marathon_phrases(corpus):
    """Слова корпуса - слова сценариев марафона: каждая фраза шага есть в файлах ``*_v1.json``."""
    texts = set()
    for path in dm.SCENARIOS.glob("night_marathon_act*_v1.json"):
        if "_acceptance_" not in path.name:
            texts |= {s["text"] for s in json.loads(path.read_text(encoding="utf-8"))["steps"]}
    assert {s["step"]["text"] for s in corpus_steps(corpus)} <= texts


def test_clean_drops_only_underscore_keys():
    assert build_marathon.clean({"a": 1, "_c": 2, "b": [{"_x": 1, "y": 2}]}) == {"a": 1, "b": [{"y": 2}]}


# ── Метрики на синтетических отчётах ──────────────────────────────────────────────────────────────────────────────


def turn(**kw):
    base = {"addressed": True, "latency_ms": 100, "llm_calls": 1, "tokens": 1000, "tier1": "", "offered": ["speak_text"],
            "widened": [], "tools": [], "calls": [], "asked_yes": False, "said": [], "outcomes": [], "route": None}
    base.update(kw)
    return base


def report(label, verdict, turns, first=None):
    rep = dm.StepReport("9", label, "текст", dm.TEXT, [], verdict=verdict, turns=turns)
    rep.first_verdict = first or verdict
    return rep


def entry(expect=(), patterns=(), scored=True):
    step = {"text": "Робот", "label": "x", "patterns": list(patterns)}
    if expect:
        step["acceptance"] = {"expected_tool_calls": list(expect)}
    return {"id": "9/x", "scored": scored, "kind": dm.TEXT, "step": step}


def run_of(*reports):
    return {"9": list(reports)}


def index_of(**entries):
    return {f"9/{label}": e for label, e in entries.items()}


def test_pass_share_leaves_not_evaluated_and_not_run_out_of_the_denominator():
    rows = [("9/a", entry(), report("a", dm.PASS, [])), ("9/b", entry(), report("b", dm.FAIL, [])),
            ("9/c", entry(), report("c", dm.NOT_EVALUATED, [])), ("9/d", entry(), report("d", dm.NOT_RUN, []))]
    out = met.pass_share(rows)
    assert (out["PASS"], out["FAIL"], out["NOT_EVALUATED"], out["NOT_RUN"], out["share"]) == (1, 1, 1, 1, 0.5)


def test_danger_false_stop_and_expected_stop():
    stopped = report("a", dm.FAIL, [turn(tier1="stop_all")])
    music = report("b", dm.PASS, [turn(tools=["stop_music"])])
    fine = report("c", dm.PASS, [turn(tools=["get_battery_level"])])
    rows = [("9/a", entry(), stopped), ("9/b", entry(expect=["stop_music"]), music), ("9/c", entry(), fine)]
    assert met.danger(rows)["false_stop"]["steps"] == ["9/a"]
    patterned = [("9/a", entry(patterns=["tier1 kind=stop_all"]), stopped)]
    assert met.danger(patterned)["false_stop"]["count"] == 0


def test_danger_motion_instead_of_look_and_unasked_delete_and_claim():
    moved = report("m", dm.FAIL, [turn(tools=["move_direction"])])
    deleted = report("d", dm.FAIL, [turn(calls=[["delete_waypoint", {"confirm": True}]])])
    asked = report("e", dm.PASS, [turn(calls=[["delete_waypoint", {"confirm": True}]], asked_yes=True)])
    liar = report("l", dm.FAIL, [turn(said=["Включаю музыку."], outcomes=[["request_music", "rejected", "x"]])])
    honest = report("h", dm.PASS, [turn(said=["Включаю музыку."], outcomes=[["request_music", "done", None]])])
    out = met.danger([("9/m", entry(), moved), ("9/d", entry(), deleted), ("9/e", entry(), asked),
                      ("9/l", entry(), liar), ("9/h", entry(), honest)])
    assert out["motion_instead_of_look"]["steps"] == ["9/m"]
    assert out["delete_without_confirm"]["steps"] == ["9/d"]
    assert out["claimed_without_tool"]["steps"] == ["9/l"]
    assert out["total"] == 3


def test_slice_categories():
    called = report("a", dm.PASS, [turn(tools=["get_battery_level"], offered=["get_battery_level"])])
    missing = report("b", dm.FAIL, [turn(offered=["speak_text"])])
    rescued = report("c", dm.PASS, [turn(offered=["speak_text"], widened=["request_music"])])
    wrong = report("d", dm.FAIL, [turn(offered=["stop_music", "dj_set"], tools=["stop_music"])])
    rows = [("9/a", entry(["get_battery_level"]), called), ("9/b", entry(["set_voice"]), missing),
            ("9/c", entry(["request_music"]), rescued), ("9/d", entry(["dj_set"]), wrong)]
    out = met.slice_stats(rows)
    assert out["expected_tool_checks"] == {"called": 1, "missing": 1, "rescued": 1, "wrong_choice": 1}
    assert out["size"]["max"] == 2.0


def test_latency_and_llm_use_and_tokens_not_reported():
    rows = [("9/a", entry(), report("a", dm.PASS, [turn(latency_ms=100, tokens=0), turn(latency_ms=300, llm_calls=0)]))]
    assert met.latency(rows)["median_ms"] == 200 and met.latency(rows)["without_llm_share"] == 0.5
    assert met.llm_use(rows)["tokens_reported"] is False


def test_stability_needs_two_runs_and_counts_same_verdicts():
    index = index_of(a=entry(), b=entry())
    one = run_of(report("a", dm.PASS, []), report("b", dm.PASS, []))
    two = run_of(report("a", dm.PASS, []), report("b", dm.FAIL, []))
    assert met.stability([one], index)["same_verdict_share"] is None
    assert met.stability([one, two], index)["same_verdict_share"] == 0.5


# ── «Не оценено» ──────────────────────────────────────────────────────────────────────────────────────────────────


class Boom:
    """Стенд, который падает, и стенд без лога: вердикта нет."""

    def __init__(self, result=None):
        self.counter = type("C", (), {"calls": 0})()
        self.clock = type("K", (), {"advance": lambda self, s: None})()
        self._result = result

    async def hear(self, text, voice=""):
        if self._result is None:
            raise RuntimeError("стенд упал")
        return self._result


def test_stand_exception_is_not_evaluated_not_fail():
    step = {"label": "x", "text": "Робот, привет", "patterns": []}
    rep, _ = asyncio.run(dm.run_step(Boom(), "9", step, "", 100, 0))
    assert rep.verdict == dm.NOT_EVALUATED and "RuntimeError" in rep.fail[0]


def test_empty_log_is_not_evaluated(corpus):
    from dry_stand import HearResult

    result = HearResult("Робот, привет", True, log="   ", said=["да"])
    step = {"label": "x", "text": "Робот, привет", "patterns": []}
    rep, _ = asyncio.run(dm.run_step(Boom(result), "9", step, "", 100, 0))
    assert rep.verdict == dm.NOT_EVALUATED


# ── Прогон на фейковом провайдере ─────────────────────────────────────────────────────────────────────────────────


def run_fake(corpus, variants, runs, budget, blind=False, acts=("1",)):
    chosen = um.corpus_acts(corpus, list(acts))
    return asyncio.run(um.run_all(variants, runs, chosen, um.fake_factory(corpus, blind), budget, 0))


def args_for(**kw):
    ns = um.build_parser().parse_args([])
    for key, value in kw.items():
        setattr(ns, key, value)
    return ns


def test_fake_run_produces_report_for_every_variant(corpus):
    results = run_fake(corpus, ["A", "B", "C"], 2, 400)
    report = um.build_report(results, corpus, args_for(variants="A,B,C", runs=2, fake="oracle"), None)
    assert set(report["variants"]) == {"A", "B", "C"}
    for variant in report["variants"].values():
        assert variant["metrics"]["pass"]["steps"] == 20 and len(variant["runs"]) == 2  # 10 оцениваемых × 2 прогона
        assert variant["metrics"]["stability"]["same_verdict_share"] == 1.0  # фейк детерминирован
    assert report["variants"]["C"]["jev"]["calls"] > 0
    assert set(report["decision_rule"]) == {"B", "C"}
    text = um.render(report)
    assert "Фейковый провайдер" in text and "N=2" in text and "| метрика | A | B | C |" in text


def test_blind_fake_fails_what_the_oracle_passes(corpus):
    oracle = met.pass_share(met.scored_items(run_fake(corpus, ["A"], 1, 200)[0].reports, um.corpus_index(corpus)))
    blind = met.pass_share(met.scored_items(run_fake(corpus, ["A"], 1, 200, blind=True)[0].reports,
                                            um.corpus_index(corpus)))
    assert blind["share"] < oracle["share"]


def test_budget_is_a_hard_cap_for_the_whole_run(corpus):
    results = run_fake(corpus, ["A", "B"], 1, 18, acts=("1", "3"))
    assert sum(r.llm_calls for r in results) <= 18
    assert any(rep.verdict == dm.NOT_RUN for r in results for reps in r.reports.values() for rep in reps)
    assert results[1].cap >= results[0].cap - results[0].llm_calls  # остаток делится по очереди, а не пропадает


def test_report_json_roundtrip_and_logs_written(corpus, tmp_path):
    results = run_fake(corpus, ["A", "B"], 1, 200)
    report = um.build_report(results, corpus, args_for(variants="A,B", fake="oracle"), None)
    um.write_out(tmp_path, report, results)
    loaded = json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))
    assert loaded["variants"]["B"]["metrics"]["pass"]["steps"] == 10
    assert (tmp_path / "report.md").read_text(encoding="utf-8").startswith("# Замер понимания фразы")
    assert any((tmp_path / "logs" / "A_1").glob("act1_*.log"))


def test_no_api_key_ever_reaches_the_report(corpus, monkeypatch, tmp_path):
    monkeypatch.setenv("DEEPSEEK_API_KEY", "sk-secret-should-not-leak-0000")
    monkeypatch.setenv("JEV_API_KEY", "jev-secret-should-not-leak-0000")
    results = run_fake(corpus, ["A", "C"], 1, 200)
    report = um.build_report(results, corpus, args_for(variants="A,C", fake="oracle"), None)
    um.write_out(tmp_path, report, results)
    blob = "".join(p.read_text(encoding="utf-8") for p in tmp_path.rglob("*") if p.is_file())
    assert "secret-should-not-leak" not in blob


# ── Сверка с живым марафоном ──────────────────────────────────────────────────────────────────────────────────────

LIVE_LINES = {
    "✅ acceptance PASS": live_compare.PASS,
    "✅ acceptance PASS (попытка 2/3)": live_compare.PASS,
    "итог — ✅ OK с попытки 2/3; прежние попытки провалились: паттерны": live_compare.PASS,
    "итог — ✅ OK после восстановления харнессом (no_wake)": live_compare.PASS,
    "итог — ❌ FAIL, проверка не прошла после retry — acceptance": live_compare.FAIL,
    "❌ ход закончен без ответа: no_speech": live_compare.FAIL,
    "PLAY attempt 1/3": live_compare.UNKNOWN,
}


@pytest.mark.parametrize("line, verdict", sorted(LIVE_LINES.items()))
def test_live_verdict_from_the_last_step_line(line, verdict):
    assert live_compare.verdict_of(line) == verdict


def test_live_verdicts_take_the_last_line_per_step_and_compare(tmp_path):
    log = tmp_path / "act1_100.log"
    log.write_text("\n".join([
        "e2e\t2026-10-28T09:28:29Z >>> STEP n101_a: PLAY attempt 1/3",
        "e2e\t2026-10-28T09:28:30Z >>> STEP n101_a: ❌ ход закончен без ответа: no_speech",
        "e2e\t2026-10-28T09:28:40Z >>> STEP n101_a: ✅ acceptance PASS (попытка 2/3)",
        "e2e\t2026-10-28T09:28:50Z >>> STEP n102_b: итог — ❌ FAIL, проверка не прошла после retry — acceptance",
    ]), encoding="utf-8")
    live = live_compare.live_verdicts(tmp_path)
    assert {k: v["verdict"] for k, v in live.items()} == {"1/n101_a": "PASS", "1/n102_b": "FAIL"}
    out = live_compare.compare({"1/n101_a": "PASS", "1/n102_b": "PASS", "1/n103_c": "PASS"}, live,
                               ["1/n101_a", "1/n102_b", "1/n103_c"])
    assert (out["compared"], out["disagree"], out["share"]) == (2, 1, 0.5)
    assert out["live_fail_dry_pass"] == ["1/n102_b"] and out["skipped"][0][0] == "1/n103_c"


# ── Классификатор Jev ─────────────────────────────────────────────────────────────────────────────────────────────


class StubProvider:
    def __init__(self, answer="music", confidence=0.9, error=None):
        self.answer, self.confidence, self.error, self.asked = answer, confidence, error, []

    async def decide(self, state, questions):
        self.asked.append((state, questions))
        if self.error:
            raise self.error
        probs = {self.answer: self.confidence}
        return Decision({jev_router.QUESTION_ID: ChoiceAnswer(self.answer, probs, self.confidence)}, "jev",
                        usage={"input_tokens": 120, "output_tokens": 3}, latency_ms=77.0)


def router(provider, **kw):
    return jev_router.JevRouter(provider, jev_router.load_spec(), **kw)


def test_group_descriptions_are_data_covering_every_tool_group():
    spec = jev_router.load_spec()
    assert set(spec.groups) == set(kn.TOOL_GROUPS) and spec.language == "ru"
    question = spec.question()
    assert question.options == (*kn.TOOL_GROUPS, uv.NO_GROUP) and all(question.descriptions[o] for o in question.options)


def test_spec_with_a_missing_group_is_refused(tmp_path):
    data = json.loads(jev_router.DEFAULT_SPEC.read_text(encoding="utf-8"))
    data["groups"].pop("music")
    data["groups"]["extra"] = "лишняя"
    path = tmp_path / "spec.json"
    path.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    with pytest.raises(ValueError, match="music"):
        jev_router.load_spec(path)


def test_threshold_default_is_the_decision_layer_one():
    assert jev_router.DEFAULT_THRESHOLD == 0.6


@pytest.mark.parametrize("confidence, accepted", [(0.9, True), (0.6, True), (0.59, False)])
def test_router_accepts_by_threshold_and_counts_tokens(confidence, accepted):
    r = router(StubProvider("music", confidence))
    route = asyncio.run(r.choose("Робот, включи музыку", music_playing=True))
    assert (route.group, route.accepted, route.input_tokens, route.output_tokens) == ("music", accepted, 120, 3)
    assert r.stats()["calls"] == 1 and r.stats()["input_tokens"] == 120


def test_router_sends_phrase_and_state_and_never_the_key():
    provider = StubProvider()
    asyncio.run(router(provider).choose("Робот, включи музыку", music_playing=True))
    state, questions = provider.asked[0]
    assert state == {"реплика": "Робот, включи музыку", "сейчас играет музыка": True}
    assert set(questions) == {jev_router.QUESTION_ID}


def test_router_failure_is_a_route_error_not_an_exception():
    r = router(StubProvider(error=TimeoutError("медленно")))
    route = asyncio.run(r.choose("Робот"))
    assert route.error == "TimeoutError" and route.as_pair() is None and r.stats()["errors"] == 1


def test_router_budget_is_a_hard_cap_without_a_request():
    provider = StubProvider()
    r = router(provider, max_calls=1)
    asyncio.run(r.choose("раз"))
    second = asyncio.run(r.choose("два"))
    assert second.error == "jev_budget" and len(provider.asked) == 1 and r.stats()["calls"] == 1


# ── .env ──────────────────────────────────────────────────────────────────────────────────────────────────────────


def test_env_files_parse_and_do_not_clobber(tmp_path, monkeypatch):
    path = tmp_path / ".env"
    path.write_text("# c\nA_KEY=value=with=eq\nexport B_KEY='quoted'\nC_KEY=\nbad line\n", encoding="utf-8")
    assert env_files.parse_env_file(path) == {"A_KEY": "value=with=eq", "B_KEY": "quoted", "C_KEY": ""}
    monkeypatch.setenv("A_KEY", "from-shell")
    monkeypatch.delenv("B_KEY", raising=False)
    applied = env_files.load_env_files([path, tmp_path / "missing.env"])
    assert applied == [(path, 1)] and __import__("os").environ["A_KEY"] == "from-shell"
    monkeypatch.delenv("B_KEY", raising=False)
