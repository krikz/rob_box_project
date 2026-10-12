"""``scripts/research/jev_eval.py --domain music --provider jev|llm_harness`` (ADR-0157 PR-7, #3564): отчёт против
deterministic. Реальный API не вызывается — провайдер подставной (контракт ``DecisionProvider``)."""

from __future__ import annotations

import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from rob_box_harness.decision import ChoiceAnswer, Decision

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts" / "research"))
import jev_eval  # noqa: E402
import music_eval  # noqa: E402


class Stub:
    """Подставной провайдер: ``pick`` выбирает ответ по списку вариантов; ``usage`` — как у Jev."""

    name = "jev"

    def __init__(self, pick):
        self.pick, self.calls = pick, 0

    async def decide(self, state, questions):
        self.calls += 1
        answers = {}
        for qid, question in questions.items():
            choice = self.pick(question.options)
            answers[qid] = ChoiceAnswer(choice, {choice: 1.0}, 0.9)
        return Decision(answers=answers, provider=self.name, usage={"input_tokens": 100, "output_tokens": 4})


def args(**kw):
    return SimpleNamespace(scores=None, dataset=music_eval.DATASET, provider="jev", deadline_ms=500, **kw)


@pytest.fixture(scope="module")
def last_run():
    return jev_eval.run_music(args(), Stub(lambda options: options[-1]))


def test_report_compares_with_deterministic_and_counts_calls_and_tokens(last_run):
    rec, text = last_run["recorder"], jev_eval.music_markdown(last_run)
    assert rec.calls == len(last_run["deterministic"]["summary"]["ambiguous"]) > 0
    assert rec.input_tokens == 100 * rec.calls and rec.output_tokens == 4 * rec.calls
    assert rec.out_of_candidates == 0 and last_run["counters"]["jev_success"] == rec.calls
    for needle in ("| deterministic |", "| jev |", "верно на неоднозначном", "латентность p50 / p95",
                   "НЕ сверено", "## Критерии Go (§4)", "0 выборов вне кандидатов"):
        assert needle in text
    print(text)


def test_choice_outside_candidates_is_counted_and_never_executed():
    result = jev_eval.run_music(args(), Stub(lambda options: "no-such-candidate"))
    rec = result["recorder"]
    assert rec.out_of_candidates == rec.calls > 0
    assert result["counters"]["jev_invalid"] == rec.calls
    assert jev_eval.accuracy(result["model"]["rows"]) == jev_eval.accuracy(result["deterministic"]["rows"])
    verdict = {name: ok for name, ok, _ in jev_eval.go_verdict(result)}
    assert verdict["0 выборов вне кандидатов"] is False


def test_percentile_nearest_rank():
    assert jev_eval.percentile([], 95) is None
    assert jev_eval.percentile(list(range(1, 101)), 95) == 95 and jev_eval.percentile([7], 50) == 7


def test_env_file_fills_missing_keys_without_overriding_or_printing(tmp_path, monkeypatch, capsys):
    env = tmp_path / ".env"
    env.write_text("# комментарий\nJEV_API_KEY='from-file'\nexport LLM_HARNESS_MODEL=m-1\nбез равно\n"
                   "LLM_HARNESS_API_KEY=already\n", encoding="utf-8")
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    monkeypatch.delenv("LLM_HARNESS_MODEL", raising=False)
    monkeypatch.setenv("LLM_HARNESS_API_KEY", "set-before")
    jev_eval.load_env_file(env)
    assert os.environ["JEV_API_KEY"] == "from-file" and os.environ["LLM_HARNESS_MODEL"] == "m-1"
    assert os.environ["LLM_HARNESS_API_KEY"] == "set-before"
    assert capsys.readouterr().out == ""
    monkeypatch.delenv("JEV_API_KEY")
    monkeypatch.delenv("LLM_HARNESS_MODEL")


def test_missing_key_exits_with_names_not_values(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    with pytest.raises(SystemExit) as stop:
        jev_eval.run_music(args())
    assert "JEV_API_KEY" in str(stop.value)
