"""Замер понимания фразы, этап 2 (карточка #4163): части корпуса 2-4 (перефразы, искажения STT, английский), их
загрузка и наследование ожиданий, автопроверки «вейк, имена, числа на месте», метрики и правило решения по частям,
бюджет слотами. LLM и Jev - фейки; тесты проверяют обвязку, а не модель."""

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

import alternatives as alt  # noqa: E402
import build_english  # noqa: E402
import build_paraphrases  # noqa: E402
import build_stt  # noqa: E402
import dry_marathon as dm  # noqa: E402
import understanding_measure as um  # noqa: E402
import understanding_metrics as met  # noqa: E402
import understanding_report as ur  # noqa: E402

WAKES = build_paraphrases.wake_table()


@pytest.fixture(scope="module")
def base():
    return um.load_corpus()


@pytest.fixture(scope="module")
def parts(base):
    return um.load_parts(base, [1, 2, 3, 4])


# ── Файлы частей ──────────────────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("part", [2, 3, 4])
def test_part_is_frozen_against_part_one_and_hangs_only_on_scored_steps(base, parts, part):
    data = parts[part]
    index = alt.index_of(base)
    assert data["part"] == part and data["base"]["corpus_sha256"] == base["corpus_sha256"]
    assert data["corpus_sha256"] == alt.digest(data["alternatives"])
    assert all(index[sid]["scored"] and rows for sid, rows in data["alternatives"].items())
    assert data["counts"]["steps"] == len(data["alternatives"])
    assert data["counts"]["alternatives"] == sum(len(r) for r in data["alternatives"].values())


@pytest.mark.parametrize("part", [2, 3, 4])
def test_hand_edit_of_a_part_is_refused(base, parts, part, tmp_path):
    broken = json.loads(json.dumps(parts[part]))
    sid = next(iter(broken["alternatives"]))
    broken["alternatives"][sid][0]["text"] += " (правка руками)"
    path = tmp_path / "part.json"
    path.write_text(json.dumps(broken, ensure_ascii=False), encoding="utf-8")
    with pytest.raises(SystemExit, match="руками"):
        alt.load_part(part, base, path)


def test_part_built_against_another_part_one_is_refused(base, parts, tmp_path):
    stale = json.loads(json.dumps(parts[2]))
    stale["base"]["corpus_sha256"] = "0" * 64
    path = tmp_path / "part.json"
    path.write_text(json.dumps(stale, ensure_ascii=False), encoding="utf-8")
    with pytest.raises(SystemExit, match="пересобрать часть 2"):
        alt.load_part(2, base, path)


def test_alternative_on_an_acoustic_step_is_refused(base, parts, tmp_path):
    acoustic = next(s["id"] for a in base["acts"] for s in a["steps"] if not s["scored"])
    data = json.loads(json.dumps(parts[3]))
    data["alternatives"][acoustic] = [{"text": "Робот, привет"}]
    data["corpus_sha256"] = alt.digest(data["alternatives"])
    path = tmp_path / "part.json"
    path.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    with pytest.raises(SystemExit, match="неоцениваемых"):
        alt.load_part(3, base, path)


def test_unknown_part_is_refused(base):
    with pytest.raises(SystemExit, match="неизвестные части"):
        um.load_parts(base, [1, 7])


# ── Наследование ожиданий ─────────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("part", [2, 3, 4])
@pytest.mark.parametrize("run", [1, 2, 3])
def test_part_corpus_changes_only_the_text_and_keeps_act_context(base, parts, part, run):
    """Все шаги и акты марафона на месте и по порядку; у шага с альтернативой другой только ``text`` (альтернатива
    по кругу), ожидания те же; оцениваются только шаги части."""
    alts = parts[part]["alternatives"]
    view = alt.part_corpus(base, alts, run)
    assert [a["act"] for a in view["acts"]] == [a["act"] for a in base["acts"]]
    for act_view, act_base in zip(view["acts"], base["acts"]):
        assert [s["id"] for s in act_view["steps"]] == [s["id"] for s in act_base["steps"]]
        for got, src in zip(act_view["steps"], act_base["steps"]):
            rows = alts.get(src["id"])
            assert got["scored"] == bool(rows)
            assert {k: v for k, v in got["step"].items() if k != "text"} == \
                {k: v for k, v in src["step"].items() if k != "text"}
            assert got["step"]["text"] == (alt.alternative_for(rows, run)["text"] if rows else src["step"]["text"])
    assert base["acts"][0]["steps"][0]["step"]["text"].startswith("Робот")  # база не тронута подстановкой


def test_alternative_rotates_over_runs():
    rows = [{"text": "a"}, {"text": "b"}, {"text": "c"}]
    assert [alt.alternative_for(rows, r)["text"] for r in range(1, 7)] == list("abcabc")
    assert [alt.alternative_for(rows[:1], r)["text"] for r in (1, 2, 3)] == ["a", "a", "a"]


# ── Автопроверка перефраз ─────────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("original, paraphrase, problem", [
    ("Робот, который сейчас час?", "Робот, скажи, сколько времени?", None),
    ("Робот, который сейчас час?", "Скажи, сколько сейчас времени?", "вейк в начале"),
    ("Робот, который сейчас час?", "Робокс, сколько времени?", "вейк в начале"),
    ("Робот, который сейчас час?", "Робот, скажи время, робот.", "вейк-слов"),
    ("Слушай, включи музыку.", "Робот, включи музыку.", "вейк"),
    ("Робот, переключись на голос Джейн.", "Робот, давай голосом Джейн.", None),
    ("Робот, переключись на голос Джейн.", "Робот, смени голос на другой.", "пропали имена"),
    ("Борис, блок гудит.", "Слушай, Бориса блок гудит.", None),  # падеж имени можно менять
    ("Робот, поставь сет из трёх треков.", "Робот, включи сет на три трека.", None),
    ("Робот, поставь сет из трёх треков.", "Робот, включи сет на 3 трека.", None),
    ("Робот, поставь сет из трёх треков.", "Робот, включи сет на пять треков.", "числа"),
    ("Робот, скажи по-китайски «всем спокойной ночи».", "Робот, произнеси «всем спокойной ночи» по-китайски.", None),
    ("Робот, скажи по-китайски «всем спокойной ночи».", "Робот, пожелай всем доброй ночи по-китайски.", "цитата"),
    ("Робот, который сейчас час?", "Робот, который сейчас час.", "те же слова"),
])
def test_check_paraphrase_keeps_wake_names_numbers_quotes(original, paraphrase, problem):
    problems = alt.check_paraphrase(original, paraphrase, WAKES)
    assert (not problems) if problem is None else any(problem in p for p in problems), problems


def test_check_paraphrase_keeps_anchor_groups_and_refuses_answer_hints(base):
    groups, leaks = alt.anchors(base), alt.leak_groups(base)
    tea = "2/n203_sasha_memory_tea"
    assert "лук" in groups[tea] and any("зелён" in g for g in groups[tea])  # память для n1003/n1009 - на месте
    assert alt.check_paraphrase("Робот, запомни: пью зелёный чай и не ем лук.", "Робот, учти: не ем лук, пью чай.",
                                WAKES, groups[tea])
    search = "2/n213_memory_search_tea_after_reset"
    text = alt.index_of(base)[search]["step"]["text"]
    hint = alt.check_paraphrase(text, "Робот, найди в памяти, что я говорил про зелёный чай.", WAKES, groups[search],
                                leaks[search])
    assert any("подсказывает ответ" in p for p in hint)
    assert "космос" in groups["9/n907_search_space"]  # n1007 запрещает «космос»: тема n907 обязана остаться


def test_own_must_not_say_is_never_an_anchor(base):
    """Свой запрет ответу («сейчас включ» у n610) не держит перефразу: это не смысл реплики."""
    assert not any("сейчас включ" in g for g in alt.anchors(base)["6/n610_dj_state_honest"])


def test_name_stems_cover_every_capitalized_word_inside_phrases(base):
    for entry in (s for a in base["acts"] for s in a["steps"]):
        for word in alt.capitalized_inside(entry["step"]["text"]):
            assert alt.names_in(word), f"{entry['id']}: «{word}» - имя? добавить основу в NAME_STEMS"


def test_frozen_paraphrases_pass_every_check(base, parts):
    """То же, что ``build_paraphrases.py --check``: каждая перефраза проходит автопроверку и ручной отказ."""
    items = {item["id"]: item for item in build_paraphrases.plan(base, WAKES)[0]}
    for sid, rows in parts[2]["alternatives"].items():
        assert 2 <= len(rows) <= 3 and sid in items
        for row in rows:
            assert not build_paraphrases.problems_of(items[sid], row["text"], WAKES), (sid, row["text"])


def test_bare_wake_steps_are_excluded_with_a_reason(base, parts):
    excluded = {e["id"]: e["reason"] for e in parts[2]["excluded"]}
    assert set(excluded) == {"3/n305b_bare_robot_pickup", "3/n310_execute_backlog_lru"}
    assert all("голое вейк-слово" in r for r in excluded.values())


def test_manual_rejects_are_not_in_the_frozen_file(parts):
    texts = {(sid, r["text"]) for sid, rows in parts[2]["alternatives"].items() for r in rows}
    assert not texts & set(build_paraphrases.MANUAL_REJECT)


# ── Часть 3: только реальные искажения ────────────────────────────────────────────────────────────────────────────


def test_stt_rows_are_real_distortions_with_a_source(base, parts):
    index = alt.index_of(base)
    for sid, rows in parts[3]["alternatives"].items():
        original = index[sid]["step"]["text"]
        for row in rows:
            assert alt.norm(row["text"]) != alt.norm(original)
            assert build_stt.similarity(original, row["text"]) >= build_stt.MIN_SIMILARITY
            assert build_stt.edge_insertions(original, row["text"]) < build_stt.EDGE_WORDS
            src = row["source"]
            assert (src["kind"] == "artifact" and src["run_id"].isdigit() and src["file"].startswith("night_")) or \
                   (src["kind"] == "issue" and src["issue"] in {r[0] for r in build_stt.ISSUE_ROWS})


def test_stt_filter_counts_add_up_and_excluded_lines_carry_no_text(parts):
    method = parts[3]["method"]["filter"]
    unique = method.pop("transcript_lines_unique")
    method = {k: v for k, v in method.items() if not k.startswith("карточка") and k != "взято из карточек"}
    assert sum(method.values()) == unique
    assert all(set(e) == {"issue", "id", "reason"} for e in parts[3]["excluded"])  # без текста: там может быть чужая речь


@pytest.mark.parametrize("original, heard, why", [
    ("Робот, всё, глуши.", "робот всох в глуши", ""),
    ("Робот, всё, глуши.", "робот все глуши", "слова совпали"),
    ("Робот, продолжай развивать эту тему и добавь баса пожирнее.", "как раз", "мало общего"),
    ("Робот, ты меня слышишь? Ответь коротко.", "что нужно еще робот ты меня слышишь ответь коротко", "по краям"),
])
def test_stt_line_filter(original, heard, why):
    entry = {"step": {"text": original}, "scored": True}
    got = build_stt.verdict(entry, original, heard)
    assert (got == "") if not why else why in got


def test_stt_old_scenario_wording_is_not_taken():
    entry = {"step": {"text": "Робот, который час?"}, "scored": True}
    assert "старая редакция" in build_stt.verdict(entry, "Робот, который час, я спрашиваю!", "робот который час")


def test_stt_scan_reads_transcript_lines_with_run_id(tmp_path):
    night = tmp_path / "night_20261028_000000"
    night.mkdir()
    (night / "act7_40033620645.log").write_text(
        "x\t>>> TRANSCRIPT[n712_stop_all]: ожидалось «Робот, всё, глуши.», распознано «робот всох в глуши»\n"
        "y\t>>> TRANSCRIPT[n712_stop_all]: ожидалось «Робот, всё, глуши.», распознано «робот всох в глуши»\n",
        encoding="utf-8")
    seen = build_stt.scan(tmp_path)
    (key, row), = seen.items()
    assert key == ("n712_stop_all", "Робот, всё, глуши.", "робот всох в глуши")
    assert row["run_id"] == "40033620645" and row["line"] == 1 and row["runs"] == {"40033620645"}


# ── Часть 4: английский ───────────────────────────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("original, english, problem", [
    ("Робот, который сейчас час?", "Robbox, what time is it?", None),
    ("Робот, который сейчас час?", "Robot, what time is it?", "вейк в начале"),
    ("Робот.", "Robbox.", None),
    ("Слушай, включи музыку.", "Listen, put on some music.", None),
    ("Слушай, включи музыку.", "Robbox, put on some music.", "вейк"),
    ("Робот, переключись на голос Джейн.", "Robbox, switch to Jane's voice.", None),
    ("Робот, переключись на голос Джейн.", "Robbox, switch the voice.", "пропали имена"),
    ("Робот, поставь сет из трёх треков, объявляй каждый.", "Robbox, play a set of three tracks, announce each one.",
     None),
    ("Робот, опиши в двух словах.", "Robbox, describe it in a couple of words.", None),
    ("Робот, поставь сет из трёх треков.", "Robbox, play a set of tracks.", "числа"),
    ("Робот, который час?", "Robbox, который час?", "кириллица"),
])
def test_check_translation(original, english, problem):
    problems = alt.check_translation(original, english, WAKES)
    assert (not problems) if problem is None else any(problem in p for p in problems), problems


def test_english_steps_rest_on_tools_and_log_not_on_russian_answer_words(base, parts):
    chosen, refused = build_english.choose(base)
    index = alt.index_of(base)
    assert 40 <= len(chosen) <= 50 and set(parts[4]["alternatives"]) <= {c["id"] for c in chosen}
    for sid in parts[4]["alternatives"]:
        acc = index[sid]["step"].get("acceptance") or {}
        assert not acc.get("expected_keywords")
    reasons = {r["id"]: r["reason"] for r in refused}
    assert "10/n1007_no_leak" in reasons and "8/n805b_unsilence" in reasons and "1/n106_alt_wake_roboks" in reasons
    for sid, rows in parts[4]["alternatives"].items():
        assert not alt.check_translation(index[sid]["step"]["text"], rows[0]["text"], WAKES), (sid, rows[0]["text"])


def test_english_wake_is_in_the_robot_wake_table():
    assert alt.EN_WAKE.lower() in [w.lower() for w in WAKES] and "robot" not in [w.lower() for w in WAKES]


# ── Прогон по частям (фейк) ───────────────────────────────────────────────────────────────────────────────────────


def fake_plan(base, parts, variants, part_list, runs, budget, acts=("1",)):
    slots = um.plan_slots(base, parts, part_list, runs, list(acts))
    make = um.fake_factory(base, False, um.alternative_texts(base, parts))
    return asyncio.run(um.run_plan(variants, slots, make, budget, 0))


def args_for(**kw):
    ns = um.build_parser().parse_args([])
    for key, value in kw.items():
        setattr(ns, key, value)
    return ns


def test_slots_go_run_by_run_over_parts():
    slots = [(s.run, s.part) for s in um.plan_slots(um.load_corpus(), {}, [1], 3, ["1"])]
    assert slots == [(1, 1), (2, 1), (3, 1)]


def test_part_metrics_count_only_steps_of_the_part_and_use_alternative_texts(base, parts):
    results = fake_plan(base, parts, ["A", "C"], [1, 2, 4], 2, 2000)
    report = um.build_report(results, base, args_for(variants="A,C", runs=2, fake="oracle", parts="1,2,4"), None, parts)
    act1 = {s["id"] for s in base["acts"][0]["steps"]}
    for key, part in report["parts"].items():
        want = len([i for i in act1 if alt.index_of(base)[i]["scored"]]) if key == "1" else \
            len([i for i in act1 if i in parts[int(key)]["alternatives"]])
        for variant in part["variants"].values():
            assert variant["metrics"]["pass"]["steps"] == want * 2
    run2 = {s["id"]: s["text"] for s in report["parts"]["2"]["variants"]["A"]["runs"][1] if s["scored"]}
    for sid, text in run2.items():
        assert text == alt.alternative_for(parts[2]["alternatives"][sid], 2)["text"]
    english = [s["text"] for s in report["parts"]["4"]["variants"]["C"]["runs"][0] if s["scored"]]
    assert english and all(t.startswith("Robbox") for t in english)
    assert set(report["parts"]["2"]["decision_rule"]["C"]) >= {"better_than_A", "danger_ok", "median_latency_ok"}
    assert "better_than_A" not in report["parts"]["4"]["decision_rule"]["C"]  # английский в правиле не стоит
    text = um.render(report)
    assert "## Часть 2 - перефразы" in text and "## Главная таблица" in text and "этап 2" in text


def test_short_budget_skips_whole_slots_for_every_variant(base, parts):
    """Не хватает на слот всем вариантам - слот целиком NOT_RUN (видно в отчёте), а не обрезанный хвост у одного."""
    results = fake_plan(base, parts, ["A", "C"], [1, 2], 2, 50)
    assert sum(r.llm_calls for r in results) <= 50
    skipped = [(r.variant, r.part, r.run) for r in results if r.skipped]
    assert skipped and len(skipped) % 2 == 0
    for variant, part, run in skipped:
        assert (("A" if variant == "C" else "C"), part, run) in skipped
    rep = next(r for r in results if r.skipped)
    assert all(s.verdict == dm.NOT_RUN and "бюджет LLM" in s.fail[0] for reps in rep.reports.values() for s in reps)
    report = um.build_report(results, base, args_for(variants="A,C", runs=2, fake="oracle", parts="1,2"), None, parts)
    assert report["skipped"] and "Не запущено (бюджет LLM" in um.render(report)


def test_variants_of_one_slot_get_the_same_cap(base, parts):
    results = fake_plan(base, parts, ["A", "C"], [1, 2], 2, 600)
    by_slot = {}
    for r in results:
        by_slot.setdefault((r.part, r.run), set()).add(r.cap)
    assert all(len(caps) == 1 for caps in by_slot.values()), by_slot


def test_stability_ignores_not_run_and_not_evaluated():
    def rep(label, verdict):
        return dm.StepReport("9", label, "т", dm.TEXT, [], verdict=verdict)

    entry = {"id": "9/a", "scored": True, "kind": dm.TEXT, "step": {"text": "Робот", "label": "a"}}
    index = {"9/a": entry, "9/b": {**entry, "id": "9/b"}}
    runs = [{"9": [rep("a", dm.PASS), rep("b", dm.PASS)]}, {"9": [rep("a", dm.PASS), rep("b", dm.NOT_RUN)]},
            {"9": [rep("a", dm.NOT_EVALUATED), rep("b", dm.NOT_RUN)]}]
    out = met.stability(runs, index)
    assert out["steps"] == 1 and out["same_verdict_share"] == 1.0  # b судим один раз - не входит


# ── Правило решения по частям ─────────────────────────────────────────────────────────────────────────────────────


def metrics_of(share, danger=0, median_ms=1000.0):
    return {"metrics": {"pass": {"share": share}, "danger": {"total": danger}, "latency": {"median_ms": median_ms}}}


@pytest.mark.parametrize("a, c, better", [(0.80, 0.81, True), (0.80, 0.80, False), (0.80, 0.70, False)])
def test_paraphrases_and_stt_need_strictly_better_than_a(a, c, better):
    for part in (2, 3):
        rule = ur.decision_rule({"A": metrics_of(a), "C": metrics_of(c)}, 3.0, part)["C"]
        assert rule["better_than_A"] is better and "pass_drop_ok" not in rule


def test_overall_rule_needs_every_condition():
    parts = {1: {"decision_rule": ur.decision_rule({"A": metrics_of(0.9), "C": metrics_of(0.89)}, 3.0, 1)},
             2: {"decision_rule": ur.decision_rule({"A": metrics_of(0.7), "C": metrics_of(0.8)}, 3.0, 2)},
             3: {"decision_rule": ur.decision_rule({"A": metrics_of(0.5), "C": metrics_of(0.6)}, 3.0, 3)}}
    pooled = {"A": {"danger": {"total": 2}, "latency": {"median_ms": 1500.0}},
              "C": {"danger": {"total": 2}, "latency": {"median_ms": 3000.0}}}
    assert ur.overall_rule(parts, pooled, 3.0)["C"]["all_met"] is True
    worse = {**pooled, "C": {"danger": {"total": 3}, "latency": {"median_ms": 3000.0}}}
    assert ur.overall_rule(parts, worse, 3.0)["C"]["all_met"] is False
    no_stt = {k: v for k, v in parts.items() if k != 3}
    rule = ur.overall_rule(no_stt, pooled, 3.0)["C"]
    assert rule["part3_better"] is None and rule["all_met"] is False  # не измерено - не выполнено


def test_paired_counts_wins_and_losses_on_the_same_run_and_step():
    def runs(*verdicts):
        return [[{"id": f"9/s{i}", "scored": True, "verdict": v} for i, v in enumerate(verdicts)]]

    variants = {"A": {"runs": runs("PASS", "FAIL", "PASS", "NOT_RUN")},
                "C": {"runs": runs("PASS", "PASS", "FAIL", "PASS")}}
    assert ur.paired(variants)["C"] == {"won": 1, "lost": 1, "same": 1}


@pytest.mark.parametrize("before, after, calls, spent", [
    ({"USD": 6.46, "CNY": -1.02}, {"CNY": -1.02, "USD": 5.52}, 3823, "0.9400 USD"),  # порядок записей не важен
    ({"CNY": -1.02}, {"CNY": -1.02}, 125, None),  # не сдвинулся при вызовах - «не измерено», а не 0
    (None, {"USD": 1.0}, 10, None),
])
def test_deepseek_spend_by_currency(before, after, calls, spent):
    out = um.deepseek_spend(before, after, calls)
    assert out["spent"] == spent and ("не измерен" in out["note"]) == (spent is None)


def test_jev_budget_is_shared_over_all_passes(monkeypatch):
    import rob_box_harness.decision as decision

    monkeypatch.setattr(decision, "jev_from_env", lambda **kw: object())
    make = um.jev_factory(args_for(jev_budget=10))
    first = make()
    first.calls = 7
    second = make()
    assert second.max_calls == 3
    second.calls = 3
    assert make().max_calls == 0
