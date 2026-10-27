"""Тесты сухого стенда марафона ``scripts/dialog/dry_marathon.py`` (эпик #3683, карточка #3906).

LLM — фейковый провайдер за НАСТОЯЩИМ мостом команд (``command_bridge.CommandModeLLM``: схема запроса, разбор,
проверка по срезу тулов, повтор по нарушениям). Для каждого класса багов, которые стенд обязан ловить без робота,
два теста: с исправленным кодом шаг сценария проходит, а с откатом исправления (``monkeypatch`` возвращает старое
поведение) тот же шаг красный. Это доказывает, что ожидание шага ловит регресс, а не проходит само по себе.

Классы: срез тулов при музыке (#3870), память за собеседником (#3881), бэклог с авторством (#3892), passed → invalid
(#3859), enum тулов среза (#3897), ``too_long`` у длинного ответа («перечисли голоса», #3883), фоновая фраза того же
собеседника (#3894).
"""

from __future__ import annotations

import asyncio
import json
import re
import sys
from pathlib import Path
from types import MappingProxyType

import pytest

# Мост команд и порты хода тянут клиент LLM-провайдеров (`openai`) через rob_box_harness.providers: в лёгком окружении
# «Python Code Quality» его нет, и сбор модуля падал ImportError. Нужен настоящий мост - модуль пропускаем.
pytest.importorskip("openai", reason="сухому стенду нужен настоящий мост команд, а ему - клиент openai")

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "dialog"))

import dry_marathon as dm  # noqa: E402
import dry_stand  # noqa: E402
from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog import memory as dialog_memory  # noqa: E402
from rob_box_dialog import phrases  # noqa: E402
from rob_box_dialog import tools_view  # noqa: E402
from rob_box_dialog import turn as turn_mod  # noqa: E402
from rob_box_llm.provider import LLMResponse, ProviderCapabilities, ToolCall  # noqa: E402
from rob_box_llm.providers.fake import FakeLLMProvider  # noqa: E402
from rob_box_voice.core import command_bridge  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import CallCounter, Counted  # noqa: E402
from rob_box_voice.tts_voice_registry import PROVIDER_LABELS, PROVIDER_VOICES  # noqa: E402


def command(args: dict) -> LLMResponse:
    return LLMResponse(content="", tool_calls=(ToolCall("c1", "command", args),), finish_reason="tool_calls")


def last(msgs) -> tuple:
    """``(роль, текст)`` последнего сообщения запроса к модели."""
    return msgs[-1].role, str(msgs[-1].content or "")


def user_text(msgs) -> str:
    """Реплика человека хода вместе со снимком: последнее сообщение роли ``user``."""
    return next((str(m.content) for m in reversed(msgs) if m.role == "user"), "")


def query_data(msgs) -> dict:
    """Данные ``query`` обратно модели (JSON в сообщении роли ``tool``); не данные (нарушения повтора) - ``{}``."""
    try:
        return json.loads(msgs[-1].content)["query_results"][0]["data"]
    except (ValueError, KeyError, TypeError):
        return {}


class Env:
    """Стенд с фейковым провайдером за мостом команд; ``rule(msgs, tools)`` решает, чем ответит «модель»."""

    def __init__(self, rule, tmp_path: Path) -> None:
        self.counter = CallCounter()

        async def on_complete(msgs, tools, settings):
            return command(rule(self, list(msgs), [t["function"]["name"] for t in tools]))

        self.inner = FakeLLMProvider(on_complete=on_complete, name="deepseek",
                                     capabilities=ProviderCapabilities(tools=True, tool_choice=True))
        relay = dm.LogRelay()
        llm = command_bridge.wrapper_for("deepseek", log=relay)(Counted(self.inner, self.counter))
        self.stand = dry_stand.DryStand(llm, self.counter, "Ты робот.", tmp_path)
        relay.target = self.stand
        self.stand.speakers.register("anton", "Саша")
        self.stand.speakers.register("ermil", "Борис")

    def run(self, step: dict, *, prev_speech: str = "", budget: int = 300):
        report, _ = asyncio.run(dm.run_step(self.stand, "t", step, prev_speech, budget, 0))
        return report

    def hear(self, text: str, voice: str = "anton"):
        return asyncio.run(self.stand.hear(text, voice))

    def close(self) -> None:
        self.stand.close()


@pytest.fixture
def make_env(tmp_path):
    envs = []

    def make(rule):
        env = Env(rule, tmp_path / f"env{len(envs)}")
        envs.append(env)
        return env

    yield make
    for env in envs:
        env.close()


def step(label: str, text: str, voice: str = "anton", **kw) -> dict:
    """Шаг сценария в форме ``gen_night_marathon.step``: acceptance — ``expect_tools`` / ``keywords`` / ``must_not``."""
    out = {"label": label, "voice": voice, "text": text, "patterns": kw.pop("patterns", [])}
    acc = {"expected_tool_calls": kw.pop("expect_tools", []), "must_not_call": kw.pop("must_not", [])}
    if "keywords" in kw:
        acc["expected_keywords"] = kw.pop("keywords")
    if "must_not_say" in kw:
        acc["must_not_say"] = kw.pop("must_not_say")
    if acc["expected_tool_calls"] or acc["must_not_call"] or "expected_keywords" in acc or "must_not_say" in acc:
        out["acceptance"] = acc
    out.update(kw)
    return out


# ── #3870: срез тулов при играющей музыке ─────────────────────────────────────────────────────────────────────────


def _battery_rule(env, msgs, tools):
    role, _ = last(msgs)
    if role == "tool" and query_data(msgs):
        return {"speech": f"Заряд {query_data(msgs)['battery_level']} процентов."}
    return {"actions": [{"tool": "get_battery_level", "args": {}}]}


def old_groups_for_music_first(real):
    """Старый порядок среза (#3857): играющая сцена вытесняет названную человеком группу."""
    def groups(snapshot, text, carry=(), asked=()):
        groups_now = real(snapshot, text, carry, asked)
        return tuple(g for g in groups_now if g == "music") + tuple(g for g in groups_now if g != "music")
    return groups


BATTERY = step("n_battery", "Робот, сколько у тебя заряда?", expect_tools=["get_battery_level"], keywords=["87"])


def test_3870_named_group_survives_playing_music(make_env):
    env = make_env(_battery_rule)
    env.stand.player.start("клубный трек")
    report = env.run(BATTERY)
    assert report.verdict == dm.PASS, report.fail


def test_3870_regression_music_scene_first_makes_step_red(make_env, monkeypatch):
    monkeypatch.setattr(tools_view, "groups_for", old_groups_for_music_first(tools_view.groups_for))
    # Страховка #3960 (тул вне среза → код открывает его группу и повторяет) прячет регресс среза: стенд проверяет
    # сам срез, поэтому откатываем и её — как было до #3960.
    monkeypatch.setattr(command_bridge, "widen_for", lambda missing, agent, have: ())
    env = make_env(_battery_rule)
    env.stand.player.start("клубный трек")
    report = env.run(BATTERY)
    assert report.verdict == dm.FAIL and "finished" in report.fail[0]  # тул вне среза: команда невалидна, ход провален


# ── #3881: память за собеседником ─────────────────────────────────────────────────────────────────────────────────


def _memory_rule(env, msgs, tools):
    role, text = last(msgs)
    if role == "tool":
        data = query_data(msgs)
        found = [r["content"] for r in data.get("results", [])]
        return {"speech": ("Помню: " + found[0]) if found else "Про тебя ничего не помню."}
    if "запомни" in user_text(msgs):
        fact = {"fact": "Борис болеет за Спартак", "category": "preference"}
        return {"actions": [{"tool": "memory_save", "args": fact}]}
    return {"actions": [{"tool": "memory_search", "args": {"query": "Спартак"}}]}


SAVE_BORIS = step("n206", "Робот, запомни про меня: я болею за Спартак.", "ermil", expect_tools=["memory_save"])
RECALL_BORIS = step("n209", "Робот, а про меня что помнишь?", "ermil", keywords=["Спартак"])


def test_3881_fact_is_bound_to_the_speaker_and_recalled(make_env):
    env = make_env(_memory_rule)
    assert env.run(SAVE_BORIS).verdict == dm.PASS
    report = env.run(RECALL_BORIS)
    assert report.verdict == dm.PASS, report.fail


def test_3881_regression_fact_bound_to_wrong_person_makes_recall_red(make_env, monkeypatch):
    env = make_env(_memory_rule)
    wrong = env.stand.speakers.register("anton", "Саша")
    real = dialog_memory.bind_person

    def bind_to_previous_person(call, snapshot):
        args = real(call, snapshot)
        if args is not None and call.tool == "memory_save":
            return MappingProxyType({**call.args, kn.PERSON_ARG: wrong})
        return args

    monkeypatch.setattr(dialog_memory, "bind_person", bind_to_previous_person)
    assert env.run(SAVE_BORIS).verdict == dm.PASS  # «вызван» ещё не «исполнен правильно» - красным станет вспоминание
    recall = env.run(RECALL_BORIS)
    assert recall.verdict == dm.FAIL and recall.said == ["Про тебя ничего не помню."]


def test_3881_speaker_changed_mid_turn_is_rejected_and_reported(make_env):
    """Собеседник сменился, пока ход ждал модель: факт не сохраняется ни за кем, а стенд показывает отказ."""
    def rule(env, msgs, tools):
        sasha = env.stand.speakers.result_for("anton")
        env.stand.session.set_speaker(dry_stand.speaker_ref(sasha, now=env.stand.clock.now()))
        return {"actions": [{"tool": "memory_save", "args": {"fact": "любит чай"}}]}

    env = make_env(rule)
    report = env.run(SAVE_BORIS)
    assert report.notes == ["memory_save: rejected(speaker_changed)"]
    assert env.stand.memory.get_facts() == []


# ── #3892: бэклог с авторством каждой фразы ───────────────────────────────────────────────────────────────────────


def _who_rule(env, msgs, tools):
    who = re.findall(r"\[говорит: ([^\]]+)\]", user_text(msgs))
    return {"speech": ("На фоне говорили: " + ", ".join(dict.fromkeys(who))) if who else "Не знаю, кто говорил."}


WHO = step("n306", "Робот.", "anton", keywords=["Борис"], expect="cycle", silent_ok=True)


def _backlog_then_pickup(env):
    assert not env.hear("Саша, блок питания гудит из-за земли, я вчера полночи его слушал.", "anton").addressed
    assert not env.hear("Саша, у тебя разводка виновата, надо было звездой разводить.", "ermil").addressed
    return env.run(WHO)


def test_3892_pickup_phrases_carry_their_own_author(make_env):
    env = make_env(_who_rule)
    report = _backlog_then_pickup(env)
    assert report.verdict == dm.PASS, report.fail
    assert "Борис" in "".join(report.said)


def test_3892_regression_all_phrases_signed_by_caller_makes_step_red(make_env, monkeypatch):
    monkeypatch.setattr(turn_mod, "picked_heard",
                        lambda addressed: tuple(("Саша", str(p.get("text") or "")) for p in addressed.picked))
    env = make_env(_who_rule)
    report = _backlog_then_pickup(env)
    assert report.verdict == dm.FAIL and report.said == ["На фоне говорили: Саша"]


# ── #3859: passed доходит до хода как passed ──────────────────────────────────────────────────────────────────────


PASSED = step("n_passed", "Робот, это я не тебе, это Борису.", "anton")


def test_3859_passed_on_addressed_phrase_finishes_the_turn(make_env):
    """Модель «молчит» на адресованную реплику дважды: мост один раз вернул ей нарушение ``passed_addressed`` (форма
    хода #4021 — ``passed`` на адресованное не положен), затем ход закрыт исходом ``passed`` — код мычит, finished."""
    env = make_env(lambda env, msgs, tools: {"passed": "not_addressed"})
    report = env.run(PASSED)
    assert report.verdict == dm.PASS, report.fail
    assert report.said and report.llm_calls == 1 + kn.COMMAND_RETRIES_MAX


def test_3859_regression_passed_as_empty_answer_makes_step_red(make_env, monkeypatch):
    real = command_bridge.to_legacy

    def passed_is_empty(result):
        # «промолчать» на адресованную реплику приходит к мосту нарушением формы хода (#4021) — его исход тоже теряем
        insisted = isinstance(result, command_bridge.Invalid) and result.reason == kn.VIOLATION_PASSED_ADDRESSED
        if insisted or (not isinstance(result, command_bridge.Invalid) and result.passed):
            return "", (), "empty"
        return real(result)

    monkeypatch.setattr(command_bridge, "to_legacy", passed_is_empty)
    env = make_env(lambda env, msgs, tools: {"passed": "not_addressed"})
    report = env.run(PASSED)
    assert report.verdict == dm.FAIL and "finished" in report.fail[0]


# ── #3897: enum тулов среза в схеме команды ───────────────────────────────────────────────────────────────────────


def _offered_enum(env) -> set:
    """Что модель могла выбрать в ``actions[].tool`` и что ей предложили тулами запроса."""
    call = env.inner.calls[0]
    schema = next(t["function"]["parameters"] for t in call.tools if t["function"]["name"] == kn.COMMAND_TOOL)
    enum = schema["properties"]["actions"]["items"]["properties"]["tool"].get("enum")
    allowed = None if enum is None else set(enum)
    offered = {t["function"]["name"] for t in call.tools} - {kn.COMMAND_TOOL}
    return allowed, offered


def test_3897_command_enum_is_the_slice_of_the_turn(make_env):
    env = make_env(lambda env, msgs, tools: {"speech": "Привет."})
    env.hear("Робот, привет!")
    allowed, offered = _offered_enum(env)
    assert "stop_music" not in allowed and "get_current_time" in allowed
    assert offered <= allowed | set(kn.SPEECH_CHANNEL_TOOLS)


def test_3897_regression_full_catalog_enum_is_visible(make_env, monkeypatch):
    monkeypatch.setattr(kn, "with_tool_enum", lambda schema, offered: schema)
    env = make_env(lambda env, msgs, tools: {"speech": "Привет."})
    env.hear("Робот, привет!")
    allowed, _ = _offered_enum(env)
    assert allowed is None  # схема не сужена срезом: модель может выбрать любой тул каталога


# ── #3883: слишком длинный ответ («перечисли голоса») ─────────────────────────────────────────────────────────────


LONG = "Я умею говорить голосами " + ", ".join(f"голос{n}" for n in range(60)) + "."
VOICES = step("n401", "Робот, какими голосами ты умеешь говорить? Перечисли, что у тебя есть.", expect_tools=[])


def test_3883_long_answer_is_retried_into_a_short_one(make_env):
    def rule(env, msgs, tools):
        return {"speech": "Алёна, Джейн, Захар, Ермил и другие." if "слишком длинно" in str(msgs[-1].content) or
                len(msgs) > 3 else LONG}

    env = make_env(rule)
    report = env.run(VOICES)
    assert report.verdict == dm.PASS, report.fail
    assert report.llm_calls == 2
    assert any("retry=1" in line and "too_long" in line for line in report.log.splitlines())


def test_3883_long_answer_that_stays_long_is_a_red_step(make_env):
    env = make_env(lambda env, msgs, tools: {"speech": LONG})
    report = env.run(VOICES)
    assert report.verdict == dm.FAIL and "finished" in report.fail[0]  # слишком длинно дважды: шаг красный (фича не выполнена)
    # ...но робот не молчит и не врёт «не понял»: честная фраза кода по причине too_long (#3953)
    assert report.said == [phrases.text(kn.INVALID_COMMAND_PHRASES["too_long"])] and report.llm_calls == 2


# ── #3883 (23.10): «какими голосами умеешь» - запрос выполнен, речь длинная, повтор честный, молчать нельзя ─────────


def _voices_rule(final_speech):
    """Модель кладёт ``list_tts_voices``; получив данные, пишет ``final_speech`` (``None`` - короткий ответ по
    данным, когда подсказка честная; иначе длинный). Старая подсказка («ничего не сделано») - повтор запроса."""
    def rule(env, msgs, tools):
        text = str(msgs[-1].content or "")
        if command_bridge.query_done(msgs):
            if text.startswith("invalid arguments") and "ничего не сделано" in text:
                return {"speech": LONG, "actions": [{"tool": "list_tts_voices", "args": {}}]}
            return {"speech": final_speech or "Голосов три набора: Яндекс, MiniMax и Silero. Назвать какие-нибудь?"}
        return {"actions": [{"tool": "list_tts_voices", "args": {}}]}
    return rule


VOICES_Q = step("n401", "Робот, какими голосами ты умеешь говорить?", expect_tools=["list_tts_voices"])


def test_3883_tool_gives_the_stand_a_brief_overview(make_env):
    seen = []

    def rule(env, msgs, tools):
        if command_bridge.query_done(msgs):
            seen.append(json.loads(msgs[-1].content)["query_results"][0]["data"])
            return {"speech": "Три провайдера."}
        return {"actions": [{"tool": "list_tts_voices", "args": {}}]}

    env = make_env(rule)
    report = env.run(VOICES_Q)
    assert report.verdict == dm.PASS, report.fail
    assert {p["provider"] for p in seen[0]["providers"]} == {"yandex", "minimax", "silero"} and "voices" not in seen[0]


def test_3883_long_speech_after_query_gets_honest_hint_and_turn_speaks(make_env):
    """Разворот из run 39878786968: длинная речь после запроса. Подсказка честная -> модель не повторяет запрос."""
    def long_first(env_, msgs, tools):
        if command_bridge.query_done(msgs) and "invalid arguments" not in str(msgs[-1].content or ""):
            return {"speech": LONG}
        return _voices_rule(None)(env_, msgs, tools)

    env = make_env(long_first)
    report = env.run(VOICES_Q)
    assert report.verdict == dm.PASS, report.fail
    assert report.said and "Назвать какие-нибудь" in report.said[-1]
    assert not any("speech_with_actions" in line for line in report.log.splitlines())
    assert any("retry=1" in line and "too_long" in line for line in report.log.splitlines())


def test_3883_still_long_after_retry_code_says_summary_not_silence(make_env):
    """Страховка (c): речь не прошла и после повтора - говорит код: число голосов по провайдерам + «назвать?»."""
    def rule(env, msgs, tools):
        if command_bridge.query_done(msgs):
            return {"speech": LONG}
        return {"actions": [{"tool": "list_tts_voices", "args": {}}]}

    env = make_env(rule)
    report = env.run(VOICES_Q)
    assert report.said, "робот промолчал"
    spoken = " ".join(report.said)
    # числа - из реестра (один источник, #4086: каст каталога растёт), не вписаны руками: новый голос не ломает тест
    for provider in ("yandex", "minimax", "silero"):
        assert f"{PROVIDER_LABELS[provider]} — {len(PROVIDER_VOICES[provider])}" in spoken, (provider, spoken)
    assert "Назвать какие-нибудь?" in spoken
    assert not any("invalid_command" in str(s) for s in env.stand.ports.stages)


# ── #3894: фоновая фраза того же собеседника в снимке адресованного хода ──────────────────────────────────────────


def _recent_rule(env, msgs, tools):
    text = user_text(msgs)
    return {"speech": "Понял, включу что-нибудь спокойное."} if "<recent_unaddressed>" in text and "спокойн" in text \
        else {"speech": "Что именно включить?"}


UNDERSTOOD = step("n_recent", "Робот, ты меня понял?", "anton", keywords=["спокойн"])


def _bg_then_ask(env):
    assert not env.hear("Включи что-нибудь спокойное, мне ещё ехать.", "anton").addressed
    return env.run(UNDERSTOOD)


def test_3894_recent_background_phrase_reaches_the_model(make_env):
    env = make_env(_recent_rule)
    report = _bg_then_ask(env)
    assert report.verdict == dm.PASS, report.fail


def test_3894_regression_no_recent_section_makes_step_red(make_env, monkeypatch):
    monkeypatch.setattr(turn_mod, "recent_section", lambda addressed: ())
    env = make_env(_recent_rule)
    report = _bg_then_ask(env)
    assert report.verdict == dm.FAIL and report.said == ["Что именно включить?"]


# ── сам стенд: разметка, бюджет, изоляция, форматы строк ──────────────────────────────────────────────────────────


def test_classification_of_real_acts_separates_acoustic_steps():
    kinds = {s["label"]: dm.classify_step(s)[0] for key in ("1", "2", "3", "4") for s in dm.load_act(key).steps}
    assert kinds["n103_time"] == dm.TEXT and kinds["n401_list_voices"] == dm.TEXT
    assert kinds["n201_sasha_intro_long"] == dm.ACOUSTIC  # регистрация голоса принимает speaker_id_node
    assert kinds["n210_grisha_no_name"] == dm.ACOUSTIC  # must_not_identify_as - биометрия
    assert kinds["n403_set_voice_alena"] == dm.ACOUSTIC  # voice_changed - владелец TTS
    assert kinds["n402_current_voice"] == dm.ACOUSTIC  # паттерн по строке SpeechEvent synthesized
    # 57: акт 4 получил n411b_prosody_normal (сброс скорости и тона, #3971 / разбор 26.10)
    assert sum(len(dm.load_act(k).steps) for k in ("1", "2", "3", "4")) == 57


@pytest.mark.parametrize("act,label", [
    ("4", "n409_speed_down"), ("4", "n410_speed_up"), ("4", "n411_pitch_bass"), ("4", "n411b_prosody_normal"),
    ("4", "n412_skazka_multivoice"), ("8", "n812_stop_music"), ("9", "n904_anim_sad"), ("9", "n905_anim_victory"),
    ("9", "n906_anekdot_anim"), ("5", "n502_start_renardo"), ("7", "n702_generate_raccoon"),
    ("7", "n711_renardo_not_broken"),
])
def test_steps_judged_by_owner_lines_the_stand_does_not_write_are_acoustic(act, label):
    """Ревью #4046: строк tts_node «изменён», ответа тула скорости, конца выступления, лица команды, was_playing и
    успеха play_animation фейки стенда не пишут -- такой шаг на стенде красный всегда, засчитывает его робот."""
    step = next(s for s in dm.load_act(act).steps if s["label"] == label)
    kind, reasons = dm.classify_step(step)
    assert kind == dm.ACOUSTIC, (label, step["patterns"])
    assert any(r.startswith("паттерн по строке") for r in reasons), reasons


def test_owner_line_fragments_match_the_generator_markers():
    """Фрагменты ``_ACOUSTIC_PATTERN`` сверены с ИСХОДНИКАМИ паттернов генератора, а не с текстом строк робота."""
    sys.path.insert(0, str(REPO / "scripts" / "e2e"))
    import gen_night_marathon as gen  # noqa: PLC0415

    for pattern in (gen.prosody_changed(gen.SPEED_PARAMS, gen.BELOW_NORMAL), gen.SPEED_RAISED, gen.SPEED_ALREADY_NORMAL,
                    gen.PITCH_ALREADY_NORMAL, gen.PERFORM_DONE, gen.FACE_DONE, gen.MUSIC_WAS_PLAYING,
                    gen.face_or_animation("sad"), gen.ROUTE_SYNTH, gen.ROUTE_GENERATE_ROBOT_SONG):
        assert dm._ACOUSTIC_PATTERN.search(pattern), pattern
    for pattern in (gen.TURN_DONE, gen.SPEECH_STARTED, gen.tool_done("play_sound"), gen.MUSIC_STARTED):
        assert not dm._ACOUSTIC_PATTERN.search(pattern), pattern


def test_acts_flags_follow_the_harness_isolation_rules():
    act2, act3 = dm.load_act("2"), dm.load_act("3")
    assert act2.registers_speakers and act2.writes_memory and not act2.inherits
    assert act3.inherits and not act3.registers_speakers


def test_budget_stops_before_the_step_that_would_overshoot(make_env):
    env = make_env(lambda env, msgs, tools: {"speech": "Привет."})
    env.counter.calls = 295
    report = env.run(step("n_b", "Робот, привет!"), budget=300)
    assert report.verdict == dm.NOT_RUN and env.inner.calls == []


def test_voice_owner_speaks_the_outcome_itself_and_the_log_line_is_checkable(make_env):
    """Смену голоса озвучивает сам тул (``OUTCOME_SPOKEN_ELSEWHERE``): ход не молчит, строка ``[set_voice]`` в логе."""
    args = {"actions": [{"tool": "set_voice", "args": {"voice": "alena", "provider": "yandex"}}]}
    env = make_env(lambda env, msgs, tools: args)
    report = env.run(step("n403", "Робот, говори голосом Алёны.", expect_tools=["set_voice"], voice_changed=True))
    assert report.verdict == dm.PASS, report.fail
    assert report.said == ["Говорю голосом alena."]
    assert "[set_voice] voice='alena' provider=yandex" in report.log and env.stand.voice_provider == "yandex"


def test_unknown_voice_is_refused_by_the_owner_and_not_applied(make_env):
    args = {"actions": [{"tool": "set_voice", "args": {"voice": "no-such-voice"}}]}
    env = make_env(lambda env, msgs, tools: args)
    heard = env.hear("Робот, говори голосом без имени.")
    assert heard.outcomes == [("set_voice", "rejected", "voice_unavailable")] and "недоступен" in heard.said[0]
    assert env.stand.session.snapshot.voice.applied is None


def test_cast_voice_named_by_catalog_name_is_applied_by_id_like_the_owner_does(make_env):
    """#4086: сухой стенд не расходится с владельцем голоса: «Весельчак» (имя каталога) -> id ``English_Jovialman``."""
    args = {"actions": [{"tool": "set_voice", "args": {"voice": "Весельчак", "provider": "minimax"}}]}
    env = make_env(lambda env, msgs, tools: args)
    heard = env.hear("Робот, говори голосом Весельчак.")
    assert heard.outcomes == [("set_voice", "done", None)]
    assert env.stand.voice_provider == "minimax" and env.stand.session.snapshot.voice.applied == "English_Jovialman"
    assert "English_Jovialman" in heard.said[0]


def test_expect_backlog_step_fails_when_the_phrase_is_addressed(make_env):
    env = make_env(lambda env, msgs, tools: {"speech": "Слушаю."})
    ok = env.run(step("n_bg", "Просто разговор без вейка.", expect="backlog", patterns=[r"\[backlog\] accumulated"]))
    bad = env.run(step("n_bg2", "Робот, слушай.", expect="backlog"))
    assert ok.verdict == dm.PASS and bad.verdict == dm.FAIL and env.counter.calls == 1


def test_stand_log_uses_the_formats_of_the_real_robot_lines():
    """Строки лога стенда сверены с источниками: поменяется формат на роботе - стенд разойдётся с ним, тест красный."""
    voice = "rob_box_voice/rob_box_voice"
    mcp = "rob_box_mcp_tools/rob_box_mcp_tools"
    pairs = [
        (f"{voice}/tts_node.py", '🔊 TTS: speech_id={speech_id[:8]}, '),
        (f"{voice}/core/speech_owner.py", "🗣️ SpeechEvent {kind} speech_id={rec.speech_id[:8]} "),
        (f"{voice}/speaker_id_node.py", "✅ Speaker '{name}' registered (id={sid[:8]})"),
        (f"{voice}/speaker_id_node.py", "👤 Speaker: '{match.name}' confidence={match.confidence:.3f} "),
        (f"{voice}/speaker_id_node.py", "👤 Speaker: unknown ({elapsed:.0f} ms)"),
        (f"{voice}/dialog_node.py", "[backlog] accumulated reason={decision.reason} entries="),
        (f"{mcp}/llm_adapter.py", "📤 Отправлен запрос {request_id[:8]}: {tool_name}"),
        (f"{mcp}/mcp_server.py", "📥 Запрос выполнения: {tool_name} с параметрами {parameters}"),
        (f"{mcp}/mcp_server.py", "📤 Публикую результат для {tool_name} (request_id: {request_id[:8]})"),
        (f"{mcp}/mcp_server.py", "✅ Инструмент {tool_name} выполнен успешно"),
        (f"{mcp}/tools/dialogue.py", "[set_voice] voice={(outcome.applied or voice_clean)!r}"),
    ]
    for rel, fragment in pairs:
        text = (REPO / "src" / rel).read_text(encoding="utf-8")
        assert fragment in text, f"{rel}: формат строки изменился: {fragment}"


def test_acceptance_check_is_one_function_for_the_robot_harness_and_the_dry_run():
    """Heredoc ``check_acceptance`` зовёт ``step_verdict``; своей копии правил в bash нет."""
    sh = (REPO / ".github/workflows/scripts/e2e_voice_test.sh").read_text(encoding="utf-8")
    assert "from e2e_tool_match import step_verdict" in sh
    assert "forbidden_said = [" not in sh and "missing_keywords = [" not in sh
