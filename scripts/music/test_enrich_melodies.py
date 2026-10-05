"""Тесты сборщика фактуры мелодий (issue #3428): правила приёма, цитата в сыром ответе, кэш, резюм, ретраи."""

from __future__ import annotations

import gzip
import json
import sys
import urllib.parse
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import enrich_melodies as em  # noqa: E402


def mb_item(mbid="r1", title="The Terminator", artist="Brad Fiedel", score=100, tags=("soundtrack",), dis=""):
    return {"id": mbid, "score": score, "title": title, "disambiguation": dis,
            "artist-credit": [{"name": artist, "artist": {"id": "a1", "name": artist}}],
            "tags": [{"count": 1, "name": t} for t in tags]}


def wd_entities(label="The Terminator", composer="Brad Fiedel", date="+1984-10-26T00:00:00Z"):
    ent = {"labels": {"en": {"value": label}}, "descriptions": {"en": {"value": "soundtrack"}},
           "claims": {"P31": [snak("Q1")], "P86": [snak("Q2")], "P136": [snak("Q3")],
                      "P577": [{"mainsnak": {"datavalue": {"type": "time", "value": {"time": date}}}}]}}
    return ent, {"Q1": "film score", "Q2": composer, "Q3": "synth-pop"}


def snak(qid):
    return {"mainsnak": {"datavalue": {"type": "wikibase-entityid", "value": {"id": qid}}}}


class FakeNet:
    """Отдаёт ответы по подстроке URL; считает обращения."""

    def __init__(self, mb=None, wd_label="The Terminator", wd_composer="Brad Fiedel"):
        self.calls = []
        self.mb = mb if mb is not None else [mb_item()]
        self.ent, self.labels = wd_entities(wd_label, wd_composer)
        self.script = {}

    def __call__(self, url):
        self.calls.append(url)
        if url in self.script and self.script[url]:
            status = self.script[url].pop(0)
            if status != 200:
                return status, ""
        if "musicbrainz" in url:
            return 200, json.dumps({"recordings": self.mb})
        if "wbsearchentities" in url:
            return 200, json.dumps({"search": [{"id": "Q10"}]})
        if "props=claims" in url:
            return 200, json.dumps({"entities": {"Q10": self.ent}})
        return 200, json.dumps({"entities": {k: {"labels": {"en": {"value": v}}} for k, v in self.labels.items()}})


def make_fetch(tmp_path, net, **kw):
    clock = {"t": 0.0}
    sleeps = []

    def sleep(sec):
        sleeps.append(sec)
        clock["t"] += sec
    fetch = em.Fetcher(em.State(tmp_path / "s.sqlite"), get=net, sleep=sleep, clock=lambda: clock["t"], **kw)
    fetch.sleeps = sleeps
    return fetch


REC = {"name": "terminat", "title": "The Terminator", "artist": "Brad Fiedel", "tags": []}


def fields(outcome):
    return {(f["field"], str(f["value"])) for f in outcome.facts}


def test_direct_accepts_fields_with_literal_evidence(tmp_path):
    fetch = make_fetch(tmp_path, FakeNet())
    out = em.enrich_record(REC, fetch, None)
    assert out.status == "done" and out.reason == "accepted"
    assert ("artist", "Brad Fiedel") in fields(out)
    assert ("year", "1984") in fields(out)
    assert ("composer", "Brad Fiedel") in fields(out) and ("work_type", "film_theme") in fields(out)
    hints = [f["value"] for f in out.facts if f["field"] == "style_hint"]
    assert len(hints) == len(set(hints))  # один ключ стиля — одна строка
    assert ("genre", "soundtrack") in fields(out) and ("style_hint", "soundtrack") in fields(out)
    for f in out.facts:
        assert f["evidence"] and f["verified"] in (True, False)
        assert set(f) == {"name", "field", "value", "source", "source_id", "source_url", "fetched_at", "confidence",
                          "verified", "license", "evidence"}
    year = next(f for f in out.facts if f["field"] == "year")
    assert year["source"] == "wikidata" and year["evidence"] == "+1984-10-26T00:00:00Z"
    tag = next(f for f in out.facts if f["field"] == "genre" and f["source"] == "musicbrainz")
    assert tag["license"] == "CC-BY-NC-SA-3.0"


def test_year_never_from_musicbrainz(tmp_path):
    doc = em.mb_docs({"recordings": [dict(mb_item(), **{"first-release-date": "1991"})]}, "t")[0]
    ok, why = em.accept_field({"field": "year", "value": 1991, "source_id": "r1", "quote": '"score":100'},
                              {"r1": doc}, em.library_anchor(REC))
    assert not ok and why == "year_only_wikidata_p577"


def test_remix_and_low_score_rejected(tmp_path):
    rec = {"name": "tetris", "title": "Tetris", "artist": "Mellow Sonic", "tags": []}
    net = FakeNet(mb=[mb_item("a", "Tetris Reset (Mellow Sonic remix)", "Mellow Sonic"),
                      mb_item("b", "Tetris", "Mellow Sonic", score=89),
                      mb_item("c", "Tetris", "Mellow Sonic", dis="live at the Forum")])
    out = em.enrich_record(rec, make_fetch(tmp_path, net), None)
    assert not [f for f in out.facts if f["source"] == "musicbrainz"]
    assert out.reason == "no_match"  # ремикс, score 89 и live отсеяны ДО предложений — фактов нет


@pytest.mark.parametrize("title,dis,expected", [
    ("Tetris (Mellow Sonic remix)", "", "variant"), ("Tetris", "karaoke version", "variant"),
    ("Tetris", "", "")])
def test_eligible_variants(title, dis, expected):
    doc = em.mb_docs({"recordings": [mb_item(title=title, artist="Mellow Sonic", dis=dis)]}, "t")[0]
    ok, why = em.eligible(doc, em.Anchor(["Tetris"], "Mellow Sonic", "library"))
    assert why == expected and ok == (expected == "")


def test_score_and_artist_rules():
    anchor = em.Anchor(["The Terminator"], "Brad Fiedel", "library")
    low = em.mb_docs({"recordings": [mb_item(score=89)]}, "t")[0]
    other = em.mb_docs({"recordings": [mb_item(artist="Someone Else")]}, "t")[0]
    assert em.eligible(low, anchor) == (False, "score_89")
    assert em.eligible(other, anchor) == (False, "artist_mismatch")


def test_fabricated_quote_rejected():
    doc = em.mb_docs({"recordings": [mb_item()]}, "t")[0]
    anchor = em.library_anchor(REC)
    bad = {"field": "artist", "value": "Brad Fiedel", "source_id": "r1", "quote": '"name":"Brad Fiedl"'}
    assert em.accept_field(bad, {"r1": doc}, anchor) == (False, "quote_not_in_raw")
    good = dict(bad, quote='"name":"Brad Fiedel"')
    assert em.accept_field(good, {"r1": doc}, anchor) == (True, "")
    lie = dict(good, value="John Williams")
    assert em.accept_field(lie, {"r1": doc}, anchor) == (False, "value_not_in_quote")
    unknown = dict(good, source_id="zzz")
    assert em.accept_field(unknown, {"r1": doc}, anchor)[0] is False


def test_bad_artist_without_llm_makes_no_requests(tmp_path):
    net = FakeNet()
    out = em.enrich_record({"name": "theme_169", "title": "Theme", "artist": "Studio Sport"}, make_fetch(tmp_path, net),
                           None)
    assert out.reason == "no_anchor" and not net.calls
    out = em.enrich_record({"name": "x", "title": "Terminator", "artist": "Films And Tv"}, make_fetch(tmp_path, net),
                           None)
    assert out.reason == "no_anchor" and not net.calls


class ScriptedLlm(em.Llm):
    def __init__(self, normalize, verify):
        self.normalize, self.verify, self.seen = normalize, verify, []

    def complete(self, system, user):
        self.seen.append(system)
        return self.normalize if system == em.PROMPT_NORMALIZE else self.verify(json.loads(user))


def test_llm_normalizes_bad_artist_and_verifies_with_quotes(tmp_path):
    def verify(payload):
        sid = next(c["source_id"] for c in payload["candidates"] if c["source"] == "mb")
        return {"fields": [
            {"field": "artist", "value": "Brad Fiedel", "source_id": sid, "quote": '"name":"Brad Fiedel"'},
            {"field": "genre", "value": "soundtrack", "source_id": sid, "quote": '"name":"soundtrack"'},
            {"field": "genre", "value": "synthwave", "source_id": sid, "quote": '"name":"synthwave"'},  # выдумка
            {"field": "canonical_title", "value": "The Terminator", "source_id": sid, "quote": "The Terminator"}]}
    llm = ScriptedLlm({"canonical_title": "The Terminator", "artist_or_composer": "Brad Fiedel",
                       "queries": ['recording:"The Terminator" AND artist:"Brad Fiedel"', "The Terminator"]}, verify)
    rec = {"name": "terminat", "title": "Terminator", "artist": "Film Theme", "tags": []}
    out = em.enrich_record(rec, make_fetch(tmp_path, FakeNet()), llm)
    assert llm.seen == [em.PROMPT_NORMALIZE, em.PROMPT_VERIFY]
    assert ("artist", "Brad Fiedel") in fields(out) and ("genre", "soundtrack") in fields(out)
    assert ("genre", "synthwave") not in fields(out)
    assert any("genre:quote_not_in_raw" in r for r in out.rejects)
    assert all(f["confidence"] == 0.7 for f in out.facts if f["field"] == "artist")  # якорь от LLM — ниже


def test_llm_unknown_is_honest_unknown(tmp_path):
    llm = ScriptedLlm({"canonical_title": "unknown", "artist_or_composer": "unknown", "queries": []}, None)
    out = em.enrich_record({"name": "t", "title": "Theme", "artist": "Studio Sport"}, make_fetch(tmp_path, FakeNet()),
                           llm)
    assert out.reason == "llm_unknown" and not out.facts


def test_not_called_when_direct_query_is_good(tmp_path):
    llm = ScriptedLlm({}, lambda p: {"fields": []})
    em.enrich_record(REC, make_fetch(tmp_path, FakeNet()), llm)
    assert em.PROMPT_NORMALIZE not in llm.seen


def test_genre_map_and_style_hint_verified_only_for_known_style():
    assert em.genre_to_style("synth-pop") == "club" and em.genre_to_style("hip hop") == "hiphop"
    assert em.genre_to_style("polka") is None
    rows = [{"field": "genre", "value": "hip hop", "confidence": 0.9, "verified": True},
            {"field": "genre", "value": "house", "confidence": 0.9, "verified": True}]
    hints = {r["value"]: r["verified"] for r in em.style_rows(rows)}
    assert hints["club"] is True and hints["hiphop"] is ("hiphop" in em.style_keys())


def test_http_cache_negative_result_and_pacing(tmp_path):
    net = FakeNet(mb=[])
    fetch = make_fetch(tmp_path, net)
    url = em.MB_URL + urllib.parse.quote("q")
    assert fetch.json(url)[0] == {"recordings": []}
    assert fetch.json(url)[0] == {"recordings": []}
    assert len(net.calls) == 1  # пустой ответ закэширован
    fetch.json(em.MB_URL + "second")
    assert fetch.sleeps and fetch.sleeps[-1] >= em.MB_GAP_S - 0.001  # >= 1.5 с между запросами к MB


def test_retry_on_503_then_ok_and_failure_not_cached(tmp_path):
    net = FakeNet()
    url = em.MB_URL + "x"
    net.script[url] = [503, 200]
    fetch = make_fetch(tmp_path, net)
    assert fetch.json(url)[0] is not None
    assert em.RETRY_PAUSES_S[0] in fetch.sleeps
    net.script[url] = [503, 503, 503]
    url2 = em.MB_URL + "y"
    net.script[url2] = [503, 503, 503]
    assert fetch.json(url2)[0] is None and fetch.state.cached(url2) is None


def test_budget_exceeded(tmp_path):
    fetch = make_fetch(tmp_path, FakeNet(), max_requests=1)
    fetch.json(em.MB_URL + "a")
    with pytest.raises(em.BudgetExceeded):
        fetch.json(em.MB_URL + "b")


def test_run_resumes_and_writes_artifact(tmp_path):
    net = FakeNet()
    recs = [REC, {"name": "theme_169", "title": "Theme", "artist": "Studio Sport"}]
    out = tmp_path / "out"
    summary = em.run(recs, out, fetch_factory=lambda st: em.Fetcher(st, get=net, sleep=lambda s: None), llm=None)
    first_calls = len(net.calls)
    assert summary["records"] == 2 and summary["by_status"]["done:no_anchor"] == 1
    rows = [json.loads(line) for line in gzip.open(out / em.ARTIFACT, "rt", encoding="utf8")]
    assert rows and {r["name"] for r in rows} == {"terminat"}
    em.run(recs, out, fetch_factory=lambda st: em.Fetcher(st, get=net, sleep=lambda s: None), llm=None)
    assert len(net.calls) == first_calls  # резюм: сделанное не перезапрашивается


def test_error_record_is_retried_on_resume(tmp_path):
    out = tmp_path / "out"
    dead = lambda url: (503, "")  # noqa: E731
    em.run([REC], out, fetch_factory=lambda st: em.Fetcher(st, get=dead, sleep=lambda s: None), llm=None)
    assert em.State(out / "enrich_state.sqlite").records()[0][1] == "error"
    net = FakeNet()
    em.run([REC], out, fetch_factory=lambda st: em.Fetcher(st, get=net, sleep=lambda s: None), llm=None)
    assert em.State(out / "enrich_state.sqlite").records()[0][1] == "done"


def test_pick_is_reproducible_by_seed():
    lib = [{"name": f"n{i}"} for i in range(100)]
    assert em.pick(lib, 5, 7, []) == em.pick(lib, 5, 7, [])
    assert em.pick(lib, 5, 7, []) != em.pick(lib, 5, 8, [])
    assert [r["name"] for r in em.pick(lib, 5, 7, ["n3"])] == ["n3"]


def test_parse_json_reply_tolerates_fences_and_think():
    assert em.parse_json_reply('<think>x</think>```json\n{"a": 1}\n```') == {"a": 1}
    assert em.parse_json_reply('Вот: {"a": 2} конец') == {"a": 2}


def test_work_type_must_follow_instance_of_label():
    ent, labels = wd_entities()
    doc = em.wd_doc("Q10", ent, labels, "t")
    anchor = em.library_anchor(REC)
    ok = {"field": "work_type", "value": "film_theme", "source_id": "Q10", "quote": '"instance_of":["film score"]'}
    assert em.accept_field(ok, {"Q10": doc}, anchor) == (True, "")
    assert em.accept_field(dict(ok, value="game_theme"), {"Q10": doc}, anchor) == (False, "work_type_not_supported")
