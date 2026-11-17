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
        jd.make_judge({"CRITIC_JUDGE": "qwen_omni"})
    with pytest.raises(jd.JudgeUnavailable, match="нет ключа"):
        jd.make_judge({"CRITIC_JUDGE": "gemini"})
    fx = tmp_path / "f.json"
    fx.write_text("[]", encoding="utf-8")
    assert jd.make_judge({"CRITIC_JUDGE": "fake", "CRITIC_JUDGE_FIXTURE": str(fx)}).name == "fake"


def test_qwen_text_judge_against_stub_transport(tmp_path):
    seen = {}

    def transport(url, headers, body):
        seen.update(url=url, body=body, auth=headers["Authorization"])
        return {"choices": [{"message": {"content": "```json\n" + json.dumps([ROW]) + "\n```"}}]}

    envf = tmp_path / ".env"
    envf.write_text("QWEN=stub-key\n", encoding="utf-8")
    judge = jd.QwenTextJudge({}, env_file=envf, transport=transport)
    assert judge.listens is False and judge.judge(None, "код трека")[0].kind == "hole"
    assert seen["url"].endswith("/compatible-mode/v1/chat/completions") and seen["body"]["model"] == "qwen3.8-max"
    assert "звука ты не слышал" in seen["body"]["messages"][0]["content"]
    bad = jd.QwenTextJudge({"QWEN": "k"}, transport=lambda *a: {"choices": [{"message": {"content": "не json"}}]})
    with pytest.raises(jd.JudgeSchemaError):
        bad.judge(None, "p")
    with pytest.raises(jd.JudgeUnavailable, match="нет ключа"):
        jd.make_judge({"CRITIC_JUDGE": "qwen_text"})


def test_gemini_is_honest_scaffold():
    with pytest.raises(jd.JudgeNotImplemented):
        jd.make_judge({"CRITIC_JUDGE": "gemini", "GEMINI_API_KEY": "test-key"}).judge(None, "p")


def sse(*chunks, done=True):
    lines = [f"data: {json.dumps(c, ensure_ascii=False)}" for c in chunks]
    return lines + (["data: [DONE]"] if done else [])


USAGE = {"prompt_tokens": 1500, "completion_tokens": 400, "prompt_tokens_details": {"audio_tokens": 72}}


def omni_stream(parts):
    chunks = [{"choices": [{"delta": {"content": x}}]} for x in parts] + [{"choices": [], "usage": USAGE}]
    return sse(*chunks)


def test_sse_parser_joins_deltas_and_takes_usage():
    text, usage = jd.parse_sse(["", ": ping"] + omni_stream(["[{", '"a":1}]']))
    assert text == '[{"a":1}]' and usage == USAGE
    with pytest.raises(jd.JudgeError, match="SSE"):
        jd.parse_sse(sse({"error": {"message": "quota"}}))


def test_omni_judge_sends_audio_and_code_streams_and_prices_usage(tmp_path):
    rec = tmp_path / "club.mp3"
    rec.write_bytes(b"ID3-fake")
    seen = {}

    def transport(url, headers, body):
        seen.update(url=url, body=body, auth=headers["Authorization"])
        return omni_stream(["```json\n", json.dumps([ROW]), "\n```"])

    judge = jd.QwenOmniJudge({"QWEN_PSG": "stub"}, transport=transport)
    assert judge.listens is True and judge.judge(rec, "КОД ТРЕКА")[0].kind == "hole"
    body = seen["body"]
    assert seen["url"] == jd.OMNI_BASE_URL + "/chat/completions" and seen["auth"] == "Bearer stub"
    assert body["model"] == "qwen3.8-omni-flash" and body["stream"] is True and body["modalities"] == ["text"]
    assert body["stream_options"] == {"include_usage": True}
    audio, text = body["messages"][1]["content"]
    assert audio["input_audio"]["format"] == "mp3" and audio["input_audio"]["data"].startswith("data:;base64,")
    assert text["text"] == "КОД ТРЕКА"  # запись и модель трека уходят вместе
    plain, audio_tok = 1500 - 72, 72
    price = jd.OMNI_PRICE_USD_PER_M["qwen3.8-omni-flash"]
    assert judge.last_usage == USAGE
    assert abs(judge.last_cost_usd - (plain * price[0] + audio_tok * price[1] + 400 * price[2]) / 1e6) < 1e-12


def test_omni_model_from_env_and_missing_key_or_recording(tmp_path):
    env = {"QWEN_PSG": "k", "CRITIC_OMNI_MODEL": "qwen3.5-omni-plus"}
    assert jd.QwenOmniJudge(env, transport=lambda *a: []).model == "qwen3.5-omni-plus"
    with pytest.raises(jd.JudgeUnavailable, match="нет ключа"):
        jd.QwenOmniJudge({})
    with pytest.raises(jd.JudgeUnavailable, match="нужна запись"):
        jd.QwenOmniJudge(env, transport=lambda *a: []).judge(tmp_path / "none.mp3", "p")


def test_omni_http_error_and_bad_answer_are_loud(tmp_path):
    rec = tmp_path / "a.mp3"
    rec.write_bytes(b"x")

    def boom(url, headers, body):
        raise jd.JudgeError("HTTP 400: incorrect modal")

    with pytest.raises(jd.JudgeError, match="HTTP 400"):
        jd.QwenOmniJudge({"QWEN_PSG": "k"}, transport=boom).judge(rec, "p")
    junk = jd.QwenOmniJudge({"QWEN_PSG": "k"}, transport=lambda *a: omni_stream(["не json"]))
    with pytest.raises(jd.JudgeSchemaError):
        junk.judge(rec, "p")


def test_run_judge_settles_budget_by_real_usage(tmp_path):
    rec = tmp_path / "a.mp3"
    rec.write_bytes(b"x")
    judge = jd.QwenOmniJudge({"QWEN_PSG": "k"}, transport=lambda *a: omni_stream([json.dumps([ROW])]))
    budget = jd.Budget(1.0)
    jd.run_judge(judge, rec, "p", budget)
    assert abs(budget.spent - judge.last_cost_usd) < 1e-12 and budget.spent != jd.COST_PER_CALL_USD["qwen_omni"]


def test_cli_one_style_writes_fixture_for_verify(tmp_path):
    style = tmp_path / "bundle" / "club"
    style.mkdir(parents=True)
    (style / "club.mp3").write_bytes(b"x")
    (style / "club.md").write_text(
        "# club\n\n`set1:01:A:abcd1234` · BPM 121 · дека A · встал на плеер на +4.44 с от старта записи (≈0.00 с в файле `club.mp3`)\n"
        "`set1:02:B:ef567890` · BPM 121 · дека B · встал (≈142.81 с в файле `club.mp3`)\n\n## Ответ без кода\nсекрет\n",
        encoding="utf-8")
    fx = tmp_path / "fx.json"
    fx.write_text(json.dumps({"claims": [ROW]}), encoding="utf-8")
    out = tmp_path / "out"
    code = jd.main(["--provider", "fake", "--fixture", str(fx), "--bundle-dir", str(tmp_path / "bundle"), "--style", "club",
                    "--out", str(out)])
    doc = json.loads((out / "club.json").read_text(encoding="utf-8"))
    assert code == 0 and doc["provider"] == "fake" and doc["style"] == "club"
    assert doc["tracks"] == [{"model": "set1_01_A_abcd1234.json", "start_s": 0.0},
                             {"model": "set1_02_B_ef567890.json", "start_s": 142.81}]
    assert jd.parse_claims(doc["claims"])[0].kind == "hole"
    _mp3, prompt, _t = jd.style_inputs(tmp_path / "bundle", "club")
    assert "секрет" not in prompt  # ответы прошлого судьи в промпт не попадают


class Counting:
    name = "qwen_omni"

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
    cached = jd.cache_path(rec, "prompt", "qwen_omni")
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


def test_args_normalized_int_midi_list_seconds_string_numbers():
    row = {**ROW, "kind": "out_of_key", "args": {"midi": 35, "seconds": [127.06, 141.18], "value": "0,5", "pcs": ["1", 2.0]}}
    args = jd.parse_claims([row])[0].args
    assert args == {"midi": [35], "seconds": pytest.approx(14.12), "value": 0.5, "pcs": [1, 2]}
    assert jd.parse_claims([{**ROW, "args": {"seconds": [7]}}])[0].args["seconds"] == 7.0
    assert jd.parse_claims([{**ROW, "args": {"midi": [35, 36]}}])[0].args["midi"] == [35, 36]


def test_unrecognized_args_are_marked_invalid_not_dropped():
    args = jd.parse_claims([{**ROW, "args": {"role": "pad", "midi": "high", "colour": "red", "seconds": [1, 2, 3]}}])[0].args
    assert args["role"] == "pad" and "midi" not in args
    assert set(args[jd.INVALID_KEY]) == {"midi", "colour", "seconds"}
    assert jd.parse_claims([{**ROW, "args": "pad"}])[0].args[jd.INVALID_KEY]["args"].startswith("ожидался объект")
    assert jd.Claim(1, 2, "hole", "x", 0.5, args).as_json()["args"][jd.INVALID_KEY]  # попадает в сохранённый JSON
