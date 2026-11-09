"""PR-8 ч.8e (#3702): транспорт хода v2 — LLM в режиме команд (``CommandResponse.command``), действия через исполнитель
тулов. Формы v1 между мостом и ходом нет (#4354 B2)."""

from __future__ import annotations

import asyncio
import json
import types

from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()

from rob_box_dialog import knowledge as kn, phrases  # noqa: E402
from rob_box_dialog.address import VOICE, Addressed  # noqa: E402
from rob_box_dialog.command import Command, Expression, Invalid, ToolCall as CommandCall, parse  # noqa: E402
from rob_box_dialog.expression import reflex  # noqa: E402
from rob_box_dialog.session import Session  # noqa: E402
from rob_box_dialog.turn import TurnEngine  # noqa: E402
import pytest  # noqa: E402
from rob_box_llm.provider import LLMResponse, ToolResult  # noqa: E402
from rob_box_voice.core import command_bridge  # noqa: E402
from rob_box_voice.core.command_bridge import CommandResponse  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import (  # noqa: E402
    EMPTY, CallCounter, Counted, V2Ports, counted_wrapper, follow_for, result_of, stage_plan, tool_schema,
)

TIME = type(M.TurnEvent().stamp)


class _Pub:
    def __init__(self, topic):
        self.topic, self.sent = topic, []

    def publish(self, msg):
        self.sent.append(msg)


class _Node:
    def __init__(self):
        self.pubs = {}

    def create_publisher(self, msg_type, topic, depth):
        return self.pubs.setdefault(topic, _Pub(topic))

    def get_clock(self):
        stamp = TIME(sec=1, nanosec=0)
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))


class _LLM:
    """Цепочка с одним ответом на вызов; ``answered_by`` — как у HealthAwareFallbackLLM."""

    def __init__(self, responses, counter):
        self.responses, self.counter, self.calls = list(responses), counter, []
        self.answered_by = "deepseek"

    async def complete(self, messages, tools=(), settings=None):
        self.counter.calls += 1
        self.calls.append((list(messages), list(tools)))
        return self.responses.pop(0)


class _Tools:
    def __init__(self, data=None):
        self.data, self.calls, self.turns = data or {}, [], 0

    def begin_turn(self):
        self.turns += 1

    async def execute(self, call):
        self.calls.append((call.name, dict(call.arguments)))
        status, data = self.data.get(call.name, ("done", {}))
        return ToolResult(tool_call_id=call.id, content="ok", is_error=status != "done",
                          data={"status": status, **data})


def _reply(args, **kw):
    """Ответ цепочки в режиме команд: команда хода из аргументов вызова ``command`` модели (разбор и вид для хода —
    как у моста, ``command_bridge.for_turn``)."""
    return CommandResponse(command=command_bridge.for_turn(parse(args)), **kw)


def test_result_of_takes_the_command_of_the_bridge_answer_as_is():
    """Ход получает ``command.Result`` моста без перевода: речь, команда с действиями, невалидная с причиной."""
    for result in (Command(speech="Привет!"), Command(actions=(CommandCall("stop_music", {}),)),
                   Invalid("bad_json", "аргументы command пусты или не JSON")):
        assert result_of(CommandResponse(command=result)) is result
    with pytest.raises(TypeError):  # провайдер цепочки без режима команд — ошибка сборки, а не тихая «речь»
        result_of(LLMResponse(content="Привет!"))


def test_bare_speech_and_empty_answer_for_the_turn():
    """Голая реплика — одной речью; ни слова — ``Invalid(EMPTY)`` (A14: пустой ответ по провайдеру)."""
    assert command_bridge.for_turn(Command(speech="Привет!")) == Command(speech="Привет!")
    assert command_bridge.for_turn(Command()) == Invalid(EMPTY, "ни речи, ни команды")
    original = Command(actions=(CommandCall("play_sound", {"sound": "boop"}),))
    assert command_bridge.for_turn(original) is original and command_bridge.verdict(original) == "command"


def test_invalid_with_salvaged_actions_reaches_the_turn_3953():
    """Невалидная после повтора команда несёт допустимые действия до хода (#3953)."""
    original = Invalid("speech_with_actions", salvage=(CommandCall("play_sound", {"sound": "boop"}),))
    assert command_bridge.verdict(original) == "invalid:speech_with_actions"
    assert command_bridge.for_turn(original) is original


def test_passed_goes_to_engine_as_passed_not_invalid():
    """#3859: «промолчать» доходит до TurnEngine как passed: ход finished/passed, не invalid; речь - фраза кода."""
    passed = Command(passed="nothing_to_say")
    assert command_bridge.verdict(passed) == "passed:nothing_to_say"
    assert command_bridge.for_turn(passed) == passed
    ports, node, _ = _ports([CommandResponse(command=passed)])
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    agg = asyncio.run(engine.run(Addressed("робот мы тут немного пошумим ты просто слушай", VOICE)))
    assert (agg.stage, agg.status) == ("done", "finished")
    mumble = [s.text for s in node.pubs["/voice/speech/request"].sent]  # адресованная реплика: код мычит (Шифу 22.10)
    assert len(mumble) == 1 and any(v in mumble[0] for v in phrases.catalog()[kn.PASSED_ADDRESSED_PHRASE])
    stages = [(e.stage, e.status, e.reason) for e in node.pubs["/dialog/turn_event"].sent]
    assert stages[-1] == ("done", "finished", "passed")


def test_follow_answers_the_command_call_with_tool_result():
    listed = Command(actions=(CommandCall("music_library", {"action": "list"}),))
    assistant, tool = follow_for(listed)('{"query_results": []}')
    (call,) = assistant.tool_calls
    assert assistant.role == "assistant" and call.name == kn.COMMAND_TOOL
    assert call.id.startswith(command_bridge.CALL_ID_PREFIX)
    assert call.arguments["actions"] == [{"tool": "music_library", "args": {"action": "list"}}]
    assert tool.role == "tool" and tool.tool_call_id == call.id and tool.content == '{"query_results": []}'
    assert follow_for(Command(speech="x"))("d") == ({"role": "user", "content": "d"},)


def test_counted_counts_every_provider_call_and_proxies_the_rest():
    counter = CallCounter()

    class Inner:
        name, capabilities = "minimax", "caps"

        async def complete(self, messages, **kw):
            return LLMResponse(content="ok")

    wrapped = counted_wrapper(lambda p: p, counter)(Inner())
    assert isinstance(wrapped, Counted) and wrapped.name == "minimax" and wrapped.capabilities == "caps"
    asyncio.run(wrapped.complete([]))
    asyncio.run(wrapped.complete([]))
    assert counter.calls == 2


def test_tool_schema_is_openai_function():
    entry = types.SimpleNamespace(name="stop_music", description="стоп", parameters={"type": "object"})
    assert tool_schema(entry) == {"type": "function", "function": {"name": "stop_music", "description": "стоп",
                                                                   "parameters": {"type": "object"}}}


def _ports(responses, tools=None):
    counter = CallCounter()
    node = _Node()
    intent_pub = _Pub("/dialog/intent")
    ports = V2Ports(node, _LLM(responses, counter), counter, tools or _Tools(), intent_pub=intent_pub)
    return ports, node, intent_pub


def test_full_turn_through_ports_speaks_outcome_and_publishes_turn_events():
    cmd = {"actions": [{"tool": "stop_music", "args": {}}]}
    tools = _Tools()
    ports, node, _ = _ports([_reply(cmd, usage={"prompt_tokens": 1234})], tools)
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    agg = asyncio.run(engine.run(Addressed("выключи музыку пожалуйста сейчас же", VOICE)))
    assert tools.calls == [("stop_music", {})] and tools.turns == 1
    speech = node.pubs["/voice/speech/request"].sent
    assert [s.text for s in speech] == ["Выключил музыку."]
    assert speech[0].agent == "personality" and speech[0].turn_id == agg.turn_id
    events = [(e.stage, e.status) for e in node.pubs["/dialog/turn_event"].sent]
    assert events == [("addressed", "started"), ("thinking", "started"), ("executing", "started"),
                      ("done", "finished")]
    assert node.pubs["/voice/expression/request"].sent, "рефлексы стадий без LLM"
    assert (agg.llm_calls, agg.llm_provider, agg.tokens) == (1, "deepseek", 1234)


def test_owner_rejection_reaches_phrase():
    cmd = {"actions": [{"tool": "stop_music", "args": {}}]}
    ports, node, _ = _ports([_reply(cmd)],
                            _Tools({"stop_music": ("rejected", {})}))
    engine = TurnEngine(Session("personality"), ports)
    asyncio.run(engine.run(Addressed("выключи музыку пожалуйста сейчас же", VOICE)))
    said = [s.text for s in node.pubs["/voice/speech/request"].sent]
    assert said == ["Не получилось выключить музыку."]


class _DeadLLM(_LLM):
    def __init__(self, error, counter):
        super().__init__([], counter)
        self.error = error

    async def complete(self, messages, tools=(), settings=None):
        raise self.error


def test_dead_provider_chain_is_provider_failed_and_turn_says_it_at_once():
    """В31: цепочка провайдеров отказала — не «провайдер жив», ход закрывается фразой с именем, а не ждёт потолка;
    ошибка кода (не провайдера) — не маскируется под отказ провайдера, ход падает громко."""
    import pytest
    from rob_box_dialog.deadline import ProviderFailed
    from rob_box_llm.errors import ProviderError

    counter = CallCounter()
    ports = V2Ports(_Node(), _DeadLLM(ProviderError("chain exhausted"), counter), counter, _Tools())
    with pytest.raises(ProviderFailed) as failed:
        asyncio.run(ports.ask([{"role": "user", "content": "x"}], [], 60.0))
    assert failed.value.provider == "deepseek" and ports.provider() == "deepseek"

    node = _Node()
    ports = V2Ports(node, _DeadLLM(ProviderError("chain exhausted"), counter), counter, _Tools())
    agg = asyncio.run(TurnEngine(Session("personality"), ports).run(Addressed("расскажи что-нибудь", VOICE)))
    said = [s.text for s in node.pubs["/voice/speech/request"].sent]
    assert said == [f"{phrases.text('understand.provider_failed', provider='deepseek')}"]
    assert agg.status == "failed"

    bug = V2Ports(_Node(), _DeadLLM(IndexError("pop from empty list"), counter), counter, _Tools())
    with pytest.raises(IndexError):
        asyncio.run(bug.ask([{"role": "user", "content": "x"}], [], 60.0))


def test_tier1_stop_publishes_stop_motion_intent_and_cancel():
    ports, node, intent_pub = _ports([])
    asyncio.run(TurnEngine(Session("personality"), ports).run(Addressed("стоп", VOICE)))
    (intent,) = intent_pub.sent
    assert intent.kind == "stop_motion" and json.loads(intent.args_json)["entities"] == {}
    assert node.pubs["/voice/speech/cancel"].sent


def test_assemble_llm_puts_every_provider_into_command_mode_and_counts_calls(monkeypatch):
    """#3906: одна сборка цепочки для ROS-хоста и сухого стенда: провайдер в режиме команд, основу промпта даёт хост."""
    from rob_box_voice.core.dialog_v2_ports import assemble_llm

    monkeypatch.setenv("DEEPSEEK_API_KEY", "dry-run-not-a-key")
    counter = CallCounter()
    llm, system = assemble_llm(["deepseek", " "], counter, prompt="Ты робот.\n")
    # промпт агента = основа + правила из knowledge.PROMPT_RULES (PR-15 15-rule, #3711)
    assert system == "Ты робот.\n\n" + kn.prompt_rules("personality")
    assert llm.name == "deepseek" and llm.mode in ("tool_choice", "json_mode", "tool_offered")
    assert counter.calls == 0


# -- команда из одной expression доходит до лица рефлексом done (#3997, хвост #3974 п.3) -----------------------------

N904 = "покажи как тебе грустно что смена такая длинная"  # n904 23.10 23:01:09 без вейка (его снимает адресация)
SAD_FACE = {"emotion": "sad", "face": {"eyes": "half", "gaze": "down", "mouth": "frown", "tint": "blue",
                                       "tempo": "slow", "duration_s": 6}}


def _done_request(node):
    """``ExpressionRequest`` итога хода: последний на топике (после него стадий нет)."""
    return node.pubs["/voice/expression/request"].sent[-1]


def test_expression_only_command_reaches_the_face_in_the_done_reflex():
    """n904: ``{expression: {emotion: sad, face}}`` без речи → ``ExpressionRequest`` итога ``done`` с эмоцией ``sad``,
    сценой лица LLM и earcon рефлекса; лицо просили — время показа (6 с сцены лица, #4083), а не 3 с рефлекса; речи
    нет, ход ``finished``."""
    ports, node, _ = _ports([_reply({"expression": SAD_FACE})])
    agg = asyncio.run(TurnEngine(Session("personality"), ports).run(Addressed(N904, VOICE)))
    assert (agg.stage, agg.status) == ("done", "finished") and agg.llm_calls == 1
    assert "/voice/speech/request" not in node.pubs, "лицо по прямой просьбе — молча"
    done = _done_request(node)
    assert (done.emotion, done.earcon, done.ttl_ms) == ("sad", reflex("done").earcon, 6000)
    assert done.ttl_ms > reflex("done").ttl_ms, "рефлекс done сцену по просьбе не обрезает (#4083)"
    assert json.loads(done.face_json) == {**SAD_FACE["face"], "duration_s": 6.0} and done.turn_id == agg.turn_id
    others = node.pubs["/voice/expression/request"].sent[:-1]
    assert [r.emotion for r in others] == [reflex(s).emotion for s in ("addressed", "thinking")], "отдельного нет"


def test_question_with_only_expression_gets_words_and_the_emotion_still_colours_done():
    """Вопрос: одна ``expression`` — нарушение ``silent_answer`` модели; второй ответ словами звучит, ``happy`` на
    итоге."""
    first = _reply({"expression": {"emotion": "happy"}})
    second = _reply({"speech": "Слышу тебя!", "expression": {"emotion": "happy"}})
    ports, node, _ = _ports([first, second])
    agg = asyncio.run(TurnEngine(Session("personality"), ports).run(Addressed("ты меня слышишь", VOICE)))
    assert agg.llm_calls == 2
    assert [s.text for s in node.pubs["/voice/speech/request"].sent] == ["Слышу тебя!"]
    assert _done_request(node).emotion == "happy"


class _SilencingLLM(_LLM):
    """Пока модель думает, человек сказал «замолчи» в другом ходе: кадр ``Silence`` на той же эпохе."""

    def __init__(self, responses, counter, session):
        super().__init__(responses, counter)
        self.session = session

    async def complete(self, messages, tools=(), settings=None):
        self.session.push_scene("Silence", "user")
        return await super().complete(messages, tools, settings)


def test_in_silence_the_command_emotion_is_not_sent():
    """#3984: в тишине лицо — заклеенный рот; эмоция и сцена команды в запрос итога не попадают."""
    session, counter = Session("personality"), CallCounter()
    node = _Node()
    llm = _SilencingLLM([_reply({"expression": SAD_FACE})], counter, session)
    ports = V2Ports(node, llm, counter, _Tools())
    agg = asyncio.run(TurnEngine(session, ports).run(Addressed(N904, VOICE)))
    assert (agg.stage, agg.status) == ("done", "finished")
    done = _done_request(node)
    assert (done.emotion, done.gesture, done.face_json) == (reflex("done").emotion, "", "")


def test_command_face_on_done_is_one_log_line_for_the_live_check():
    """Живая проверка видит окраску итога в логе ноды: одна строка ``🎭 [turn-v2]`` на ``done`` с эмоцией команды;
    стадия без экспрессии команды строки не пишет."""
    lines, node = [], _Node()
    ports = V2Ports(node, _LLM([], CallCounter()), CallCounter(), _Tools(), log=lines.append)
    ports.stage("t1", 0, "thinking", "started")
    ports.stage("t1", 0, "done", "finished", expression=Expression(emotion="sad"))
    assert lines == ["🎭 [turn-v2] лицо команды на done: emotion=sad gesture=- face=- turn=t1"]
    assert _done_request(node).emotion == "sad"


def test_stage_plan_merges_only_where_merge_allows():
    sad = Expression(emotion="sad", gesture="wink")
    assert (stage_plan("done", None, sad).emotion, stage_plan("done", None, sad).gesture) == ("sad", "wink")
    assert stage_plan("failed", None, sad) == reflex("failed"), "провал побеждает эмоцию команды"
    assert stage_plan("done") == reflex("done") and stage_plan("silenced", None, sad) is None


# -- #4049: исполненный play_animation доходит до лица ExpressionRequest итога (живая проверка 27.10 00:44) ---------

#: 00:44:39 LLM RESPONSE ``play_animation`` на «Робот, покажи молча, как тебе грустно» (аргументы как в логе).
SAD_ANIMATION = {"animation": "sad", "duration": 6,
                 "face": {"eyes": "half", "gaze": "down", "mouth": "frown", "tint": "blue"}}


def test_4049_play_animation_sad_is_the_done_expression_request_with_its_scene_face_and_duration():
    """Тул лицо не трогает (PR-3b) — до #4049 на лице был рефлекс ``happy``. Теперь ``ExpressionRequest`` итога:
    ``emotion=sad``, сцена лица LLM с 6 с вызова, ``ttl_ms=6000``; earcon — рефлекса; строка ``🎭`` с ``ttl_ms``."""
    lines = []
    args = {"actions": [{"tool": "play_animation", "args": SAD_ANIMATION}]}
    counter = CallCounter()
    node = _Node()
    ports = V2Ports(node, _LLM([_reply(args)], counter), counter, _Tools(), log=lines.append)
    agg = asyncio.run(TurnEngine(Session("personality"), ports).run(Addressed("покажи молча как тебе грустно", VOICE)))
    assert (agg.stage, agg.status) == ("done", "finished") and "/voice/speech/request" not in node.pubs
    done = _done_request(node)
    assert (done.emotion, done.gesture, done.ttl_ms, done.earcon) == ("sad", "", 6000, reflex("done").earcon)
    assert json.loads(done.face_json) == {**SAD_ANIMATION["face"], "duration_s": 6.0}
    assert done.emotion != reflex("done").emotion, "рефлекс done (happy) не перекрывает сцену вызова"
    assert lines[-1] == (f"🎭 [turn-v2] лицо команды на done: emotion=sad gesture=- face=llm ttl_ms=6000 "
                         f"turn={agg.turn_id}")


def test_4049_siren_animation_without_duration_holds_by_the_show_default():
    """«Включи сирену» → ``play_animation{police_lights}`` + речь: мигалка — сцена итога со временем показа по
    умолчанию (``FACE_SHOW_DEFAULT_S``, #4083: то же число, что по умолчанию у лица), а не 3 с рефлекса."""
    args = {"speech": "Сирена — включаю!",
            "actions": [{"tool": "play_animation", "args": {"animation": "police_lights"}}]}
    ports, node, _ = _ports([_reply(args)])
    asyncio.run(TurnEngine(Session("personality"), ports).run(Addressed("включи сирену", VOICE)))
    assert [s.text for s in node.pubs["/voice/speech/request"].sent] == ["Сирена — включаю!"]
    done = _done_request(node)
    assert (done.emotion, done.ttl_ms) == ("police_lights", int(kn.FACE_SHOW_DEFAULT_S * 1000))


# -- #4083: «молча» без слов; время показа доходит до ExpressionRequest; пачка — очередью ---------------------------


class _Later:
    """Таймер очереди лица под тестом: копит ``(через_с, fire)``, ``fire_all()`` — время прошло."""

    def __init__(self):
        self.calls, self.handles = [], []

    def __call__(self, delay_s, fire):
        handle = types.SimpleNamespace(cancelled=False)
        handle.cancel = lambda: setattr(handle, "cancelled", True)
        self.calls.append((delay_s, fire))
        self.handles.append(handle)
        return handle

    def fire_all(self):
        for (_, fire), handle in zip(self.calls, self.handles):
            if not handle.cancelled:
                fire()


def _queued_ports(responses, lines):
    counter, node, later = CallCounter(), _Node(), _Later()
    ports = V2Ports(node, _LLM(responses, counter), counter, _Tools(), log=lines.append, later=later)
    return ports, node, later


def _animations(*names):
    return [{"tool": "play_animation", "args": {"animation": n}} for n in names]


#: Пачка, которую кладёт МОДЕЛЬ (слова вне словаря анимаций: очередь кода Tier-1 её не берёт, #4103).
N_MODEL_BATCH = "покажи как ты радуешься потом грустишь и удивляешься"


def test_4083_batch_of_three_animations_reaches_the_face_one_after_another():
    """Модель кладёт три ``play_animation`` одной командой: на итоге ``done`` — первая сцена, остальные — своими
    ``ExpressionRequest`` в свой срок (8 с, 16 с), каждая ≥ 5 с; ни одна не перекрывает другую разом."""
    lines = []
    args = {"actions": _animations("happy", "sad", "surprised")}
    ports, node, later = _queued_ports([_reply(args)], lines)
    agg = asyncio.run(TurnEngine(Session("personality"), ports).run(Addressed(N_MODEL_BATCH, VOICE)))
    assert agg.llm_calls == 1
    assert (agg.stage, agg.status) == ("done", "finished") and "/voice/speech/request" not in node.pubs
    default = kn.FACE_SHOW_DEFAULT_S
    done = _done_request(node)
    assert (done.emotion, done.ttl_ms, done.earcon) == ("happy", int(default * 1000), reflex("done").earcon)
    assert [d for d, _ in later.calls] == [default, 2 * default], "вторая — когда кончилась первая, третья — за ней"
    before = len(node.pubs["/voice/expression/request"].sent)
    later.fire_all()
    queued = node.pubs["/voice/expression/request"].sent[before:]
    assert [(r.emotion, r.ttl_ms, r.earcon, r.gesture) for r in queued] == [
        ("sad", int(default * 1000), "", ""), ("surprised", int(default * 1000), "", "")]
    assert all(r.ttl_ms >= kn.FACE_SHOW_MIN_S * 1000 and r.turn_id == agg.turn_id for r in queued)
    assert any(line.startswith("🎭 [turn-v2] очередь лица: sad через 8 с на 8 с, surprised через 16 с на 8 с")
               for line in lines), lines
    assert sum(line.startswith("🎭 [turn-v2] лицо из очереди:") for line in lines) == 2


def test_4083_next_turn_drops_the_rest_of_the_queue():
    """Пока пачка показывается, человек снова обратился к роботу: новая стадия снимает недопоказанные сцены."""
    lines = []
    args = {"actions": _animations("happy", "sad", "surprised")}
    ports, node, later = _queued_ports([_reply(args)], lines)
    asyncio.run(TurnEngine(Session("personality"), ports).run(Addressed(N_MODEL_BATCH, VOICE)))
    ports.stage("next", 0, "addressed", "started")
    assert all(h.cancelled for h in later.handles)
    assert lines[-1] == "🎭 [turn-v2] очередь лица снята: 2 сцен (стадия addressed turn=next)"
    sent = len(node.pubs["/voice/expression/request"].sent)
    later.fire_all()
    assert len(node.pubs["/voice/expression/request"].sent) == sent, "снятая очередь лицу ничего не шлёт"


def test_4083_silent_ask_with_an_expression_is_one_call_and_no_speech():
    """08:20 «Робот, покажи молча, что ты задумался» → ``{expression: thinking}``: без нарушения ``silent_answer`` —
    один вызов LLM, речи нет («Молчу и думаю.» не звучит), ``thinking`` на итоге держится показом по просьбе."""
    args = {"expression": {"emotion": "thinking"}}
    ports, node, _ = _ports([_reply(args)])
    agg = asyncio.run(TurnEngine(Session("personality"), ports).run(Addressed("покажи молча что ты задумался", VOICE)))
    assert agg.llm_calls == 1 and (agg.stage, agg.status) == ("done", "finished")
    assert "/voice/speech/request" not in node.pubs
    done = _done_request(node)
    assert (done.emotion, done.ttl_ms) == ("thinking", int(kn.FACE_SHOW_DEFAULT_S * 1000))


def test_4083_expression_duration_from_the_model_reaches_the_expression_request():
    """Марафон 23.10 21:53:48: ``expression.duration_s`` модели раньше выбрасывался (``fields_dropped``). Теперь 8 с
    доходят до ``ttl_ms`` и до ``duration_s`` сцены лица."""
    lines = []
    expr = {"emotion": "sleepy", "face": {"eyes": "half", "mouth": "flat", "tint": "cold"}, "duration_s": 8}
    args = {"speech": "Тихо. Я тоже помолчу.", "expression": expr}
    counter, node = CallCounter(), _Node()
    ports = V2Ports(node, _LLM([_reply(args)], counter), counter, _Tools(), log=lines.append)
    agg = asyncio.run(TurnEngine(Session("personality"), ports).run(Addressed("все спасибо можно отдыхать", VOICE)))
    done = _done_request(node)
    assert (done.emotion, done.ttl_ms) == ("sleep", 8000)
    assert json.loads(done.face_json) == {**expr["face"], "duration_s": 8.0}
    assert lines[-1] == ("🎭 [turn-v2] лицо команды на done: emotion=sleep gesture=- face=llm ttl_ms=8000 "
                         f"turn={agg.turn_id}")


def test_4083_show_queue_only_for_done_with_a_batch():
    from rob_box_dialog.turn import Shown

    from rob_box_voice.core.dialog_v2_ports import show_queue

    batch = Shown(emotion="happy", duration_s=6.0, then=(Shown(emotion="sad", duration_s=7.0),))
    assert [(at, p.emotion, p.ttl_ms) for at, p in show_queue("done", batch)] == [(6.0, "sad", 7000)]
    assert show_queue("failed", batch) == () and show_queue("done", Expression(emotion="sad")) == ()
    assert show_queue("done", None) == ()


def _code_queue(text):
    """Ход «покажи по очереди …» через настоящий ``V2Ports``: LLM без ответов (вызов модели уронил бы тест)."""
    lines, counter, node, later, tools = [], CallCounter(), _Node(), _Later(), _Tools()
    ports = V2Ports(node, _LLM([], counter), counter, tools, log=lines.append, later=later)
    agg = asyncio.run(TurnEngine(Session("personality"), ports).run(Addressed(text, VOICE)))
    return agg, node, later, tools, lines


def test_4103_show_queue_by_code_reaches_the_face_one_after_another_without_llm():
    """Живая проверка 27.10 18:55: «Робот, покажи по очереди радость, грусть и удивление» → модель делала один
    ``play_animation thinking``. Теперь очередь собирает код: 0 вызовов LLM, три ``play_animation`` исполнителю по
    порядку реплики, речи нет; лицу — ``happy`` на итоге ``done``, ``sad`` и ``surprised`` своими ``ExpressionRequest``
    через 8 и 16 с, каждая на 8 с (≥ ``FACE_SHOW_MIN_S``)."""
    agg, node, later, tools, lines = _code_queue("покажи по очереди радость, грусть и удивление")
    assert agg.llm_calls == 0 and (agg.stage, agg.status) == ("done", "finished")
    assert tools.calls == [("play_animation", {"animation": n}) for n in ("happy", "sad", "surprised")]
    assert "/voice/speech/request" not in node.pubs
    default, ms = kn.FACE_SHOW_DEFAULT_S, int(kn.FACE_SHOW_DEFAULT_S * 1000)
    done = _done_request(node)
    assert (done.emotion, done.ttl_ms, done.earcon) == ("happy", ms, reflex("done").earcon)
    assert [d for d, _ in later.calls] == [default, 2 * default]
    before = len(node.pubs["/voice/expression/request"].sent)
    later.fire_all()
    queued = node.pubs["/voice/expression/request"].sent[before:]
    assert [(r.emotion, r.ttl_ms, r.turn_id) for r in queued] == [
        ("sad", ms, agg.turn_id), ("surprised", ms, agg.turn_id)]
    assert any(line.startswith("🎭 [turn-v2] очередь лица: sad через 8 с на 8 с, surprised через 16 с на 8 с")
               for line in lines), lines


def test_4103_neutral_first_scene_is_the_rest_face_not_the_happy_reflex():
    """Живая проверка 27.10 18:55:42: «покажи по очереди нейтральное лицо и грусть» → на ``done`` был рефлекс
    ``happy`` 5 с, грусти не было. Теперь: лицо покоя на итоге своё время (не ``happy``), ``sad`` — запросом из
    очереди, когда оно кончилось (8 с), на 8 с. Ревью #4088 (блокер «пачка с нейтралью первой теряет очередь») тоже
    держится: очередь доходит до лица."""
    agg, node, later, tools, _ = _code_queue("покажи по очереди нейтральное лицо и грусть")
    assert agg.llm_calls == 0 and tools.calls == [("play_animation", {"animation": n}) for n in ("idle", "sad")]
    default, ms = kn.FACE_SHOW_DEFAULT_S, int(kn.FACE_SHOW_DEFAULT_S * 1000)
    done = _done_request(node)
    assert (done.emotion, done.ttl_ms, done.earcon) == (kn.REFLEXES["idle"].face[0], ms, reflex("done").earcon)
    assert done.emotion != reflex("done").emotion
    assert [d for d, _ in later.calls] == [default]
    before = len(node.pubs["/voice/expression/request"].sent)
    later.fire_all()
    (queued,) = node.pubs["/voice/expression/request"].sent[before:]
    assert (queued.emotion, queued.ttl_ms, queued.turn_id) == ("sad", ms, agg.turn_id)


def test_4083_queue_timer_in_a_real_asyncio_loop_fires_on_time_and_the_next_stage_cancels_it():
    """Ревью #4088: таймер очереди без подмены (``_call_later``) в настоящем asyncio-loop — как в ``dialog_node``
    (``loop.run_forever`` в своём потоке, ход — его задача): сцена очереди приходит в свой срок, следующая стадия
    снимает недопоказанную, снятая лицу ничего не шлёт."""
    from rob_box_dialog.expression import ExpressionPlan

    lines, counter, node = [], CallCounter(), _Node()
    ports = V2Ports(node, _LLM([], counter), counter, _Tools(), log=lines.append)
    shows = ((0.01, ExpressionPlan(emotion="sad", ttl_ms=5000)),
             (0.3, ExpressionPlan(emotion="surprised", ttl_ms=5000)))

    async def scenario():
        ports._queue_shows("t1", 0, shows)
        await asyncio.sleep(0.1)
        fired = [r.emotion for r in node.pubs["/voice/expression/request"].sent if r.turn_id == "t1"]
        ports.stage("t2", 0, "addressed", "started")
        await asyncio.sleep(0.4)
        return fired

    fired = asyncio.run(scenario())
    assert fired == ["sad"], "первая сцена очереди пришла в свой срок"
    assert [r.emotion for r in node.pubs["/voice/expression/request"].sent if r.turn_id == "t1"] == ["sad"]
    assert "🎭 [turn-v2] очередь лица снята: 1 сцен (стадия addressed turn=t2)" in lines


def test_4083_queue_timer_outside_a_loop_is_a_thread_timer_that_cancels():
    """Вне asyncio-loop (стадия из ROS-потока) таймер очереди — поток-таймер с тем же ``cancel()``."""
    import threading

    from rob_box_voice.core.dialog_v2_ports import _call_later

    fired, late = threading.Event(), threading.Event()
    _call_later(0.01, fired.set)
    assert fired.wait(2.0)
    _call_later(0.05, late.set).cancel()
    assert not late.wait(0.3)


# -- #4050: спасённый query невалидной команды — данные модели, ответ звучит -----------------------------------------

LIBRARY_LIST = Invalid("speech_with_actions", salvage=(CommandCall("music_library", {"action": "list"}),))


def test_4050_follow_of_an_invalid_command_shows_the_model_only_its_salvaged_actions():
    assistant, tool = follow_for(LIBRARY_LIST)('{"query_results": []}')
    args = dict(assistant.tool_calls[0].arguments)
    assert args == {"actions": [{"tool": "music_library", "args": {"action": "list"}}]}
    assert tool.tool_call_id == assistant.tool_calls[0].id and tool.content == '{"query_results": []}'


def test_4050_salvaged_library_list_goes_back_to_the_model_and_the_answer_is_spoken():
    """00:44:54: ``invalid:speech_with_actions`` + ``music_library list`` → list исполнен, данные — модели, её слова
    звучат (до #4050 ход ``done`` без речи)."""
    tracks = {"tracks": [{"title": "Рассвет"}], "count": 1}
    tools = _Tools({"music_library": ("done", tracks)})
    answer = "В библиотеке один трек: «Рассвет»."
    ports, node, _ = _ports([CommandResponse(command=LIBRARY_LIST), _reply({"speech": answer})], tools)
    agg = asyncio.run(TurnEngine(Session("personality"), ports).run(
        Addressed("что у тебя в музыкальной библиотеке", VOICE)))
    assert tools.calls == [("music_library", {"action": "list"})]
    assert [s.text for s in node.pubs["/voice/speech/request"].sent] == [answer]
    second = ports._llm.calls[1][0]
    data = json.loads(second[-1].content)[kn.QUERY_RESULTS_FIELD]
    assert second[-1].role == "tool" and data[0]["data"]["tracks"] == tracks["tracks"]
    assert second[-2].tool_calls[0].arguments == {"actions": [{"tool": "music_library", "args": {"action": "list"}}]}
    assert (agg.stage, agg.status) == ("done", "finished") and agg.llm_calls == 2
