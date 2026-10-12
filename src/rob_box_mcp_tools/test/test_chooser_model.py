"""ADR-0157 PR-7 (#3564): модельные провайдеры выбора произведения за ``Chooser`` и shadow-режим.

Сети нет: транспорт — ``httpx.MockTransport``. Реальный ``jev``/``llm_harness`` не вызывается.
"""

from __future__ import annotations

import asyncio
import json
import time

import pytest

httpx = pytest.importorskip("httpx")

from rob_box_harness.decision import SystemOneHttpProvider  # noqa: E402
from rob_box_harness.decision.chat_choice import ChatChoiceProvider  # noqa: E402
from rob_box_mcp_tools.engine import chooser_model as cm  # noqa: E402
from rob_box_music.choice import Choice, ChoiceQuestion, DeterministicChooser, Option, Plays  # noqa: E402

SECRET = "sk-test-DO-NOT-LOG-0123456789"

#: Разрыв 0.2 < CHOOSER_CLEAR_GAP — неоднозначный вопрос: модель зовётся; детерминированный ответ — ``toccata``.
AMBIGUOUS = ChoiceQuestion(
    "токката баха",
    (Option("toccata", {"title_match": 1.0, "composer_match": 1.0}, "партитура «Toccata» Bach"),
     Option("fugue", {"title_match": 1.0, "composer_match": 1.0, "recent": 0.1}, "партитура «Fugue» Bach")),
    {"composer": [["bach"]], "recent": ["fugue"]})
CLEAR = ChoiceQuestion("моцарт", (Option("a", {"title_match": 1.0}), Option("b", {})), {})


class Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(("info", text))

    def warning(self, text):
        self.lines.append(("warning", text))


def client_for(handler):
    return lambda timeout_s: httpx.AsyncClient(transport=httpx.MockTransport(handler), timeout=timeout_s)


def jev(handler, **kw):
    return SystemOneHttpProvider("jev", base_url="https://api.test", model="jev-1.13.0", api_key_env="JEV_API_KEY",
                                 client_factory=client_for(handler), **kw)


def jev_answer(choice, confidence=0.8, other=None):
    other = other or ("toccata" if choice == "fugue" else "fugue")
    rest = round(1.0 - confidence, 6)
    return {"model": "jev-1.13.0", "usage": {"input_tokens": 90, "output_tokens": 5},
            "answers": {cm.QUESTION_ID: {"type": "choice", "choice": choice, "confidence": confidence,
                                         "probabilities": {choice: confidence, other: rest}}}}


@pytest.fixture(autouse=True)
def key(monkeypatch):
    monkeypatch.setenv("JEV_API_KEY", SECRET)
    monkeypatch.setenv("LLM_HARNESS_API_KEY", SECRET)
    monkeypatch.delenv(cm.SHADOW_ENV, raising=False)


def test_jev_adapter_sends_candidates_with_descriptions_and_returns_choice():
    seen = {}

    def handler(request):
        seen["auth"], seen["url"] = request.headers["authorization"], str(request.url)
        seen["body"] = json.loads(request.content)
        return httpx.Response(200, json=jev_answer("fugue", 0.77, other="toccata"))

    choice = cm.ModelChooser(jev(handler)).choose(AMBIGUOUS)
    assert choice == Choice("fugue", 0.77, "jev")
    assert seen["url"] == "https://api.test/v1/systemone" and seen["auth"] == f"Bearer {SECRET}"
    question = seen["body"]["questions"][cm.QUESTION_ID]
    assert question["type"] == "choice" and "Выбери один id" in question["instructions"]
    assert set(question["criteria"]) == {"toccata", "fugue"}
    assert "Toccata" in question["criteria"]["toccata"] and "композитор +2" in question["criteria"]["toccata"]
    assert "повтор −0.2" in question["criteria"]["fugue"]
    assert seen["body"]["state"] == "Заказ: «токката баха». За сессию сыграно: ничего"


def test_choice_outside_candidates_is_refused_and_chooser_answers_itself():
    model = cm.ModelChooser(jev(lambda request: httpx.Response(200, json=jev_answer("mozart", 0.9))))
    with pytest.raises(cm.ModelUnavailable):
        model.choose(AMBIGUOUS)
    assert model.orchestrator.metrics.counters()["jev_invalid"] == 1
    assert DeterministicChooser(model).choose(AMBIGUOUS) == Choice("toccata", pytest.approx(0.6), "deterministic")


def test_deadline_gives_up_in_time_and_deterministic_answers():
    async def slow(request):
        await asyncio.sleep(2.0)
        return httpx.Response(200, json=jev_answer("fugue"))

    model = cm.ModelChooser(jev(slow, timeout_s=5.0), deadline_ms=100)
    started = time.monotonic()
    answer = DeterministicChooser(model).choose(AMBIGUOUS, deadline_ms=100)
    assert time.monotonic() - started < 1.0
    assert answer.id == "toccata" and answer.provider == "deterministic"
    assert model.orchestrator.metrics.counters()["jev_timeout"] == 1


def test_circuit_opens_after_failures_and_stops_calling_the_provider():
    calls = []

    def handler(request):
        calls.append(1)
        return httpx.Response(503)

    model = cm.ModelChooser(jev(handler))
    for _ in range(5):
        with pytest.raises(cm.ModelUnavailable):
            model.choose(AMBIGUOUS)
    assert len(calls) == 3  # порог breaker'а спайка: 3 подряд
    assert model.orchestrator.metrics.counters()["jev_circuit_open"] == 2


def test_llm_harness_adapter_uses_strict_enum_schema_and_temperature_zero():
    seen = {}

    def handler(request):
        seen["url"], seen["body"] = str(request.url), json.loads(request.content)
        content = json.dumps({"choice": "fugue", "confidence": 0.7})
        return httpx.Response(200, json={"choices": [{"message": {"content": content}}],
                                         "usage": {"prompt_tokens": 120, "completion_tokens": 9}})

    provider = ChatChoiceProvider(base_url="https://llm.test/v1/", model="m-1", client_factory=client_for(handler))
    model = cm.ModelChooser(provider)
    assert model.choose(AMBIGUOUS) == Choice("fugue", 0.7, "llm_harness")
    body = seen["body"]
    assert seen["url"] == "https://llm.test/v1/chat/completions"
    assert body["temperature"] == 0 and body["model"] == "m-1"
    schema = body["response_format"]["json_schema"]
    assert schema["strict"] is True and schema["schema"]["properties"]["choice"]["enum"] == ["toccata", "fugue"]
    assert "Toccata" in body["messages"][1]["content"]


def test_llm_harness_choice_outside_enum_is_refused():
    def handler(request):
        content = json.dumps({"choice": "mozart", "confidence": 0.9})
        return httpx.Response(200, json={"choices": [{"message": {"content": content}}]})

    provider = ChatChoiceProvider(base_url="https://llm.test", model="m", client_factory=client_for(handler))
    model = cm.ModelChooser(provider)
    with pytest.raises(cm.ModelUnavailable):
        model.choose(AMBIGUOUS)
    assert model.orchestrator.metrics.counters()["llm_harness_invalid"] == 1


def shadow_model(handler, log):
    return DeterministicChooser(cm.ShadowModel(cm.ModelChooser(jev(handler)), lambda: log))


def test_shadow_executes_deterministic_and_logs_the_models_choice():
    log = Log()
    chooser = shadow_model(lambda request: httpx.Response(200, json=jev_answer("fugue", 0.9)), log)
    assert chooser.choose(AMBIGUOUS).id == "toccata"  # исполняется детерминированный, модель сказала «fugue»
    (level, line), = log.lines
    assert level == "info"
    assert line.startswith("🪞 [chooser-shadow] q='токката баха' det=toccata model=fugue agree=no ms=")
    assert SECRET not in line


def test_shadow_logs_agreement_and_provider_failure_without_changing_the_answer():
    log = Log()
    agree = shadow_model(lambda request: httpx.Response(200, json=jev_answer("toccata", 0.9, "fugue")), log)
    assert agree.choose(AMBIGUOUS).id == "toccata"
    down = shadow_model(lambda request: httpx.Response(500), log)
    assert down.choose(AMBIGUOUS).id == "toccata"
    assert "model=toccata agree=yes" in log.lines[0][1]
    assert "det=toccata model=ERR(ModelUnavailable: jev_error) agree=no" in log.lines[1][1]


def test_shadow_does_not_ask_the_model_when_the_deterministic_gap_is_clear():
    log, calls = Log(), []
    chooser = shadow_model(lambda request: calls.append(1) or httpx.Response(200, json=jev_answer("a")), log)
    assert chooser.choose(CLEAR).id == "a"
    assert calls == [] and log.lines == []


def test_shadow_is_off_by_default():
    assert cm.chooser_from_env({}, log=lambda: Log())._model is None


def test_shadow_without_key_is_off_with_one_log_line_and_no_crash(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY")
    log = Log()
    chooser = cm.chooser_from_env({cm.SHADOW_ENV: "jev"}, log=lambda: log)
    assert chooser._model is None and chooser.choose(AMBIGUOUS).id == "toccata"
    assert len(log.lines) == 1 and log.lines[0][0] == "warning" and "jev" in log.lines[0][1]
    assert SECRET not in log.lines[0][1]


def test_shadow_with_unknown_provider_name_is_off():
    log = Log()
    assert cm.chooser_from_env({cm.SHADOW_ENV: "gpt"}, log=lambda: log)._model is None
    assert len(log.lines) == 1 and log.lines[0][0] == "warning"


def test_shadow_enabled_by_env_with_key_builds_the_provider():
    log = Log()
    env = {cm.SHADOW_ENV: "jev", "JEV_API_KEY": SECRET}
    chooser = cm.chooser_from_env(env, log=lambda: log)
    assert isinstance(chooser._model, cm.ShadowModel)
    assert log.lines == [("info", "🪞 [chooser-shadow] включён: jev (исполняется детерминированный выбор)")]
    llm = cm.chooser_from_env({cm.SHADOW_ENV: "llm_harness", "LLM_HARNESS_API_KEY": SECRET,
                               "LLM_HARNESS_BASE_URL": "https://llm.test", "LLM_HARNESS_MODEL": "m"}, log=lambda: log)
    assert isinstance(llm._model, cm.ShadowModel)


def test_request_music_tool_builds_its_chooser_from_env(monkeypatch):
    """Потребитель шва: ``RequestMusicTool`` собирает ``PieceChain`` с выбиралкой из env."""
    from rob_box_mcp_tools.engine import tools_v2

    monkeypatch.setenv(cm.SHADOW_ENV, "jev")
    tool = tools_v2.RequestMusicTool(None, None, object(), lambda ids: {}, records=lambda ids: {},
                                     words=lambda w: None, classic=lambda *a, **k: None)
    assert isinstance(tool._pieces._chooser._model, cm.ShadowModel)
    monkeypatch.delenv(cm.SHADOW_ENV)
    plain = tools_v2.RequestMusicTool(None, None, object(), lambda ids: {}, records=lambda ids: {},
                                      words=lambda w: None, classic=lambda *a, **k: None)
    assert plain._pieces._chooser._model is None


PLAYED = ChoiceQuestion(
    "баха",
    (Option("toccata", {"title_match": 1.0, "recent": 1.0}, "партитура «Toccata» Bach", Plays(2, 3)),
     Option("fugue", {"title_match": 1.0}, "партитура «Fugue» Bach", Plays(0, None)),
     Option("minuet", {"title_match": 1.0}, "RTTTL «Minuet» Bach")),
    {"recent": ["toccata"]})


def test_jev_adapter_puts_play_counters_penalty_and_session_summary_into_criteria_and_state():
    seen = {}

    def handler(request):
        seen["body"] = json.loads(request.content)
        return httpx.Response(200, json=jev_answer("fugue", 0.8, other="toccata"))

    cm.ModelChooser(jev(handler)).choose(PLAYED)
    body = seen["body"]
    criteria = body["questions"][cm.QUESTION_ID]["criteria"]
    assert "уже играли 2 раза, последний раз 3 мин назад" in criteria["toccata"]
    assert "очки кода: название +3, повтор −2; итого 1;" in criteria["toccata"]
    assert "recent=" not in criteria["toccata"]
    assert "очки кода: название +3; итого 3; ещё не играли" in criteria["fugue"]
    assert "играли" not in criteria["minuet"]  # история не подключена — про повторы не говорим
    assert body["state"] == "Заказ: «баха». За сессию сыграно: партитура «Toccata» Bach ×2"
    assert body["questions"][cm.QUESTION_ID]["instructions"] == cm.INSTRUCTIONS


def test_shadow_line_carries_the_counters_the_model_saw():
    log = Log()
    shadow_model(lambda request: httpx.Response(200, json=jev_answer("fugue", 0.9, "toccata")), log).choose(PLAYED)
    assert log.lines[0][1].endswith("plays=[toccata=2/-2]")
    assert cm.plays_note(ChoiceQuestion("x", (Option("a", {}, "", Plays(1, 5)),))) == " plays=[a=1/0]"


def test_plural_forms():
    assert [cm._times(n) for n in (1, 2, 5, 11, 12, 21, 22)] == ["1 раз", "2 раза", "5 раз", "11 раз", "12 раз",
                                                                 "21 раз", "22 раза"]


def test_instructions_are_the_measured_v1_prompt_with_weights_from_the_code():
    assert cm.INSTRUCTIONS == (
        "Выбери, что сыграть роботу. У кандидатов - очки кода в одной шкале: совпадение названия +3, композитора +2, "
        "штраф за недавний повтор (до −2). Обычно лучший - с наибольшим итогом; отклоняйся, только если явно видишь, "
        "что другой кандидат лучше подходит к заказу. Выбери один id.")


def test_played_but_outside_the_penalty_window_says_so_instead_of_contradicting():
    option = Option("a", {"title_match": 1.0, "composer_match": 1.0, "fit": 1.0, "rating": 0.83}, "«A» Bach",
                    Plays(4, 90))
    text = cm.describe(option)
    assert "играли 4 раза за сутки, но не среди последних 3 исполнений - штрафа нет" in text
    assert "очки кода: название +3, композитор +2, подходит +1, рейтинг +0.415; итого 6.415" in text


def test_full_request_to_jev_for_bach_when_toccata_was_played_twice(capsys):
    """Полный запрос к Jev (state + instructions + criteria) - то, что уходит по проводу."""
    seen = {}

    def handler(request):
        seen["body"] = json.loads(request.content)
        return httpx.Response(200, json=jev_answer("brandenb", 0.5, other="toccata"))

    question = ChoiceQuestion(
        "баха",
        (Option("toccata", {"composer_match": 1.0, "fit": 0.9, "recent": 1.0, "rating": 0.83},
                "score «Toccata and Fugue» Bach",
                Plays(2, 3)),
         Option("brandenb", {"composer_match": 1.0, "fit": 0.8, "rating": 0.7}, "score «Brandenburg Concerto» Bach",
                Plays(0, None)),
         Option("jesujoyo", {"composer_match": 1.0, "fit": 0.7, "rating": 0.6}, "rtttl «Jesu Joy» Bach",
                Plays(0, None))),
        {"composer": [["bach"]], "recent": ["toccata"]})
    cm.ModelChooser(jev(handler)).choose(question)
    body = seen["body"]
    print(json.dumps(body, ensure_ascii=False, indent=1))
    crit = body["questions"][cm.QUESTION_ID]["criteria"]
    assert crit["toccata"] == ("score «Toccata and Fugue» Bach; очки кода: композитор +2, подходит +0.9, "
                               "рейтинг +0.415, повтор −2; итого 1.315; уже играли 2 раза, последний раз 3 мин назад")
    assert crit["brandenb"] == ("score «Brandenburg Concerto» Bach; очки кода: композитор +2, подходит +0.8, "
                                "рейтинг +0.35; итого 3.15; ещё не играли")
    assert body["state"] == "Заказ: «баха». За сессию сыграно: score «Toccata and Fugue» Bach ×2"
