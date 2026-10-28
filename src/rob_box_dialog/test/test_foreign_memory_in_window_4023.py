"""#4023 (остаток после PR #4048): чужие факты не доходят до модели через ``recent_actions`` снимка (ADR-0148).

Марафон 27.10 (dev-d68a784): привязка памяти к кандидату уже закрыта (``memory.bind_person``), но исходы тулов памяти
прошлых ходов лежали в снимке как есть: в ходе Гриши 08:53:08.979 - результат ``memory_search`` Саши («Саша пьёт только
зелёный чай…»), в спорном ходе 09:05:38.034 - ``memory_save`` «Пользователя зовут Тамара». Теперь журнал помнит, чья
память была у хода при исполнении (``memory.memory_owner``), и исход тула памяти другого владельца в снимок не идёт
(``memory.foreign_result``). Тест проверяет поведение модулей и что уходит модели, а не текст промпта.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.memory import bind_person, foreign_result, memory_owner
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.turn import RECENT_ACTIONS, Answer, TurnEngine
from rob_box_dialog.turn_log import TOOL, TurnLog

SASHA = SpeakerRef(person_ref="spk-sasha", name="Саша", name_state="confirmed")
GRISHA = SpeakerRef(person_ref="spk-grisha", name="Гриша", name_state="confirmed")
CONTESTED = SpeakerRef(person_ref=None, name=None, name_state="contested")  # speaker_ref спора: профиля нет
TENTATIVE = SpeakerRef(person_ref="spk-sasha", name="Саша", name_state="tentative")
NAMELESS = SpeakerRef(person_ref="spk-guest-3", name=None, name_state="unknown")  # свой профиль без имени (#3858)
STRANGER = SpeakerRef()
TEA = "Саша пьёт только зелёный чай без сахара"


def _snap(speaker, agent="personality"):
    session = Session(agent)
    session.set_speaker(speaker)
    return session.snapshot


def _result(ref, tool, status="done", **data):
    return ActionResult(ref, 0, "personality", tool, kn.TOOL_CLASSES[tool], status, None, data)


# -- одно правило: чья память у хода -------------------------------------------------------------------------------


def test_memory_owner_is_the_confirmed_or_own_profile_only():
    assert memory_owner(_snap(SASHA)) == "spk-sasha"
    assert memory_owner(_snap(NAMELESS)) == "spk-guest-3", "свой профиль без имени - его память"
    for speaker in (CONTESTED, TENTATIVE, STRANGER):
        assert memory_owner(_snap(speaker)) is None, speaker.name_state
    assert memory_owner(Session("personality").snapshot) is None
    assert memory_owner(_snap(SASHA, agent="telegram")) is None, "у чата нет собеседника-голоса"


def test_bind_person_and_window_use_the_same_owner():
    """Привязка тула и окно - одно правило: кому тул не привязать, тому и чужой исход не показать."""
    call = ToolCall("memory_search", {"query": "чай"})
    for speaker in (SASHA, NAMELESS, CONTESTED, TENTATIVE, STRANGER):
        snap = _snap(speaker)
        bound = bind_person(call, snap)
        assert (bound[kn.PERSON_ARG] if bound is not None else None) == memory_owner(snap), speaker


def test_foreign_result_is_a_memory_tool_of_another_owner():
    search = {"tool": "memory_search", "status": "done", "data": {"results": [{"content": TEA}]}}
    assert foreign_result(search, "spk-sasha", "spk-grisha")
    assert foreign_result(search, "spk-sasha", None), "спорному и незнакомому чужие все исходы памяти"
    assert not foreign_result(search, "spk-sasha", "spk-sasha")
    assert not foreign_result(search, None, None), "отказ памяти без собеседника - без фактов, чужим не считается"
    for tool in kn.PERSON_SCOPED_TOOLS:
        assert foreign_result({"tool": tool}, "spk-sasha", "spk-grisha"), tool
    assert not foreign_result({"tool": "play_animation", "status": "done"}, "spk-sasha", "spk-grisha")


# -- журнал помнит владельца -------------------------------------------------------------------------------------


def test_turn_log_keeps_the_owner_and_late_outcome_keeps_it_too():
    log = TurnLog()
    log.begin("t1", 0, "что ты обо мне помнишь", "Саша")
    log.result("t1", _result("t1/0", "memory_search", status="timeout", query="чай"), "spk-sasha")
    log.result("t1", _result("t1/1", "play_animation", animation="happy"), "spk-sasha")
    assert [e.owner for e in log.window() if e.role == TOOL] == ["spk-sasha", "spk-sasha"]
    assert log.late(_result("t1/0", "memory_search", query="чай", results=[{"content": TEA}]), lambda _r: None)
    assert [e.owner for e in log.window() if e.role == TOOL] == ["spk-sasha", "spk-sasha"]


# -- ход целиком: что уходит модели (марафон 27.10, 08:53 и 09:05) ------------------------------------------------


class Ports:
    """Транспорт хода: ответы LLM по очереди; исход - аргументы вызова и данные владельца памяти."""

    def __init__(self, answers):
        self.answers, self.asked, self.said = list(answers), [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        out = []
        for n, a in enumerate(actions):
            data = dict(a.args)
            if a.tool == "memory_search":
                data["results"] = [{"kind": "fact", "content": TEA}]
            if a.tool == "memory_save":
                data["fact_id"] = 7
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool,
                                    kn.TOOL_CLASSES[a.tool], "done", None, data))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _recent(request):
    """Раздел ``recent_actions`` снимка хода - последнее сообщение запроса к модели."""
    last = request[-1]["content"]
    start = last.find(f"<{RECENT_ACTIONS}>")
    return last[start:last.find(f"</{RECENT_ACTIONS}>")] if start >= 0 else ""


def test_other_person_and_contested_voice_do_not_see_sashas_facts_but_sasha_does():
    session = Session("personality")
    reply = Answer(Command(speech="Ответ."), "deepseek")
    remembered = Answer(Command(actions=(ToolCall("memory_search", {"query": "чай"}),
                                         ToolCall("play_animation", {"animation": "happy"}))), "deepseek")
    ports = Ports([remembered, reply, reply, reply, reply, reply])  # query памяти - второй вызов модели в ходе Саши
    engine = TurnEngine(session, ports, system_prompt="Ты робот.")
    turns = ((SASHA, "что ты обо мне помнишь"), (GRISHA, "а что ты знаешь про меня"),
             (CONTESTED, "что ты обо мне помнишь"), (STRANGER, "привет"), (SASHA, "а что ещё помнишь"))
    for speaker, text in turns:
        session.set_speaker(speaker)
        asyncio.run(engine.run(Addressed(text, VOICE)))
    assert len(ports.asked) == 6
    grisha, contested, stranger, sasha_again = (_recent(r) for r in ports.asked[2:])
    for name, recent in (("Гриша", grisha), ("спорный голос", contested), ("незнакомец", stranger)):
        assert "зелёный чай" not in recent and "memory_search" not in recent, name
        assert '"tool":"play_animation"' in recent, f"{name}: прочие исходы в снимке - как были"
    assert "зелёный чай" in sasha_again and '"tool":"memory_search"' in sasha_again, "своя память - своему"
    assert all("зелёный чай" not in m["content"] for r in ports.asked[2:5] for m in r), "и нигде в запросе"


def test_saved_fact_of_a_named_person_does_not_reach_the_contested_turn():
    """09:05:38.034: в спорном снимке лежал ``memory_save`` «Пользователя зовут Тамара»."""
    session = Session("personality")
    save = Answer(Command(actions=(ToolCall("memory_save", {"fact": "Пользователя зовут Тамара"}),),
                          speech="Запомнил."), "deepseek")
    ports = Ports([save, Answer(Command(speech="Ответ."), "deepseek")])
    engine = TurnEngine(session, ports, system_prompt="Ты робот.")
    session.set_speaker(SpeakerRef(person_ref="spk-tamara", name="Тамара", name_state="confirmed"))
    asyncio.run(engine.run(Addressed("запомни, меня зовут Тамара", VOICE)))
    session.set_speaker(CONTESTED)
    asyncio.run(engine.run(Addressed("как дела", VOICE)))
    assert "Тамара" not in _recent(ports.asked[1]) and "memory_save" not in _recent(ports.asked[1])
