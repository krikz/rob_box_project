"""Тесты ``judge.py`` (ADR-0157 PR-4): схема ответа, провайдер по env, кеш по sha256 записи, бюджет J3.

``python -m pytest scripts/music/critic/test_judge.py -v``. Сеть и ключи не нужны: реальных вызовов API нет.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import judge as jd  # noqa: E402

ROW = {"t0": 1.0, "t1": 5.0, "kind": "hole", "claim": "лид молчит", "confidence": 0.7}


def test_schema_accepts_valid_row_and_args():
    claims = jd.parse_claims([ROW, {**ROW, "kind": "out_of_key", "args": {"role": "bass", "midi": [35]}}])
    assert [c.kind for c in claims] == ["hole", "out_of_key"] and claims[1].args["midi"] == [35]


@pytest.mark.parametrize("bad", [
    {**ROW, "kind": "vibes"},                     # вне перечня
    {k: v for k, v in ROW.items() if k != "t1"},  # нет поля
    {**ROW, "t0": 9.0},                           # t0 > t1
    {**ROW, "confidence": 1.5},                   # вне 0..1
    {**ROW, "claim": "  "},                       # пустой текст
    {**ROW, "extra": 1},                          # лишнее поле
    {**ROW, "args": {"colour": "red"}},           # неизвестный ключ args
    {**ROW, "t0": True},                          # bool — не число
])
def test_schema_rejects_bad_rows(bad):
    with pytest.raises(jd.JudgeSchemaError):
        jd.parse_claims([bad])


def test_schema_rejects_non_list():
    with pytest.raises(jd.JudgeSchemaError):
        jd.parse_claims({"claims": []})


def test_fake_judge_reads_fixture_and_missing_fixture_fails(tmp_path):
    fx = tmp_path / "f.json"
    fx.write_text(json.dumps({"claims": [ROW]}), encoding="utf-8")
    assert jd.FakeJudge(fx).judge(None, "p")[0].claim == "лид молчит"
    with pytest.raises(jd.JudgeUnavailable):
        jd.FakeJudge(tmp_path / "nope.json")


def test_provider_from_env_no_silent_fallback(tmp_path):
    with pytest.raises(jd.JudgeUnavailable, match="CRITIC_JUDGE"):
        jd.make_judge({})
    with pytest.raises(jd.JudgeUnavailable, match="CRITIC_JUDGE_FIXTURE"):
        jd.make_judge({"CRITIC_JUDGE": "fake"})
    with pytest.raises(jd.JudgeUnavailable, match="нет ключа"):
        jd.make_judge({"CRITIC_JUDGE": "qwen"})
    with pytest.raises(jd.JudgeUnavailable, match="нет ключа"):
        jd.make_judge({"CRITIC_JUDGE": "gemini"})
    fx = tmp_path / "f.json"
    fx.write_text("[]", encoding="utf-8")
    assert jd.make_judge({"CRITIC_JUDGE": "fake", "CRITIC_JUDGE_FIXTURE": str(fx)}).name == "fake"


def test_keyed_providers_are_honest_scaffolds():
    judge = jd.make_judge({"CRITIC_JUDGE": "qwen", "QWEN_API_KEY": "test-key"})
    assert isinstance(judge, jd.QwenOmniJudge)
    with pytest.raises(jd.JudgeNotImplemented):
        judge.judge(None, "p")
    with pytest.raises(jd.JudgeNotImplemented):
        jd.make_judge({"CRITIC_JUDGE": "gemini", "GEMINI_API_KEY": "test-key"}).judge(None, "p")


class Counting:
    name = "qwen"

    def __init__(self):
        self.calls = 0

    def judge(self, recording, prompt):
        self.calls += 1
        return jd.parse_claims([ROW])


def test_cache_next_to_recording_keyed_by_sha256_and_prompt(tmp_path):
    rec = tmp_path / "jazz.mp3"
    rec.write_bytes(b"audio-1")
    judge, budget = Counting(), jd.Budget(1.0)
    first = jd.run_judge(judge, rec, "prompt", budget)
    again = jd.run_judge(judge, rec, "prompt", budget)
    assert first == again and judge.calls == 1 and budget.cache_hits == 1 and budget.calls == 1
    cached = jd.cache_path(rec, "prompt", "qwen")
    assert cached.parent == rec.parent and jd.sha256_file(rec)[:16] in cached.name and cached.is_file()
    jd.run_judge(judge, rec, "other prompt", budget)  # другой промпт — другой ключ
    rec.write_bytes(b"audio-2")  # другая запись — другой ключ
    jd.run_judge(judge, rec, "prompt", budget)
    assert judge.calls == 3


def test_budget_limit_stops_before_the_call(tmp_path):
    rec = tmp_path / "a.mp3"
    rec.write_bytes(b"x")
    judge, budget = Counting(), jd.Budget(0.015)  # qwen: 0.01 за вызов — второй уже не влезает
    jd.run_judge(judge, rec, "p1", budget)
    with pytest.raises(jd.BudgetExceeded):
        jd.run_judge(judge, rec, "p2", budget)
    assert judge.calls == 1 and abs(budget.spent - 0.01) < 1e-9


def test_budget_limit_from_env():
    assert jd.Budget(env={"CRITIC_JUDGE_BUDGET_USD": "0.2"}).limit == 0.2
    assert jd.Budget(env={}).limit == jd.DEFAULT_BUDGET_USD
