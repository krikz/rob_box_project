"""#4173, марафон-6 (29.10, dev-980c053, акт 6, run 40813763999, n608): модель крутит повтор исполненного Tier-1 стопа.

«робот хватит диджей выключай»: Tier-1 стоп исполнил `dj_set stop` + `stop_music` кодом (молча); остаток фразы «диджей
выключай» ушёл модели, и она трижды просила то же `dj_set {"action": "stop"}` (лог: 03:48:31 повтор не исполняется, ответ
2 - `speech_with_actions` и повтор моста, ответ 3 - снова повтор), а ход закрывался заглушкой `understand.silent_answer`
(«Ага, я на связи.») со `stage=done status=done llm_calls=3`. #4208 научил ход не падать на повторе; причина цикла -
лицо в команде: повтор без действий остаётся «одним лицом без слов» (`bare_answer`), и `owed_reason` возвращал модели
нарушение `silent_answer` вместо закрытия хода. Теперь повтор исполненного - ответа сверх сделанного нет: ход сразу
закрывается фразой кода по исходу стопа («Выключил музыку.»), без новых кругов LLM (ADR-0148).
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.turn import Answer, TurnEngine

N608 = "робот хватит диджей выключай"
OK = phrases.text("music.stop_ok")
SILENT = phrases.catalog()["understand.silent_answer"]
DJ_STOP = ToolCall("dj_set", {"action": "stop"})


class Ports:
    """Транспорт хода: идёт сет, первый стоп гасит музыку; сколько раз спросили модель и что прозвучало."""

    def __init__(self, answers):
        self.answers = list(answers)
        self.playing = True
        self.executed, self.said, self.asked = [], [], 0

    async def ask(self, messages, tools, timeout_s):
        self.asked += 1
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        out = []
        for n, call in enumerate(actions):
            data = dict(call.args)
            if call.tool in kn.MUSIC_STOP_TOOLS:
                data["was_playing"], self.playing = self.playing, False
            self.executed.append((call.tool, dict(call.args)))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, call.tool,
                                    kn.TOOL_CLASSES[call.tool], "done", None, data))
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


def _run(ports, text=N608):
    ids = iter(f"t{n}" for n in range(1, 100))
    session = Session("personality")
    session.set_music(MusicStateRef("playing", track_id="gen45685:01:A"))
    session.push_scene("DJ", "user")
    engine = TurnEngine(session, ports, system_prompt="Ты робот.", new_id=lambda: next(ids))
    return asyncio.run(engine.run(Addressed(text, VOICE)))


def _repeat(face=True):
    """Повтор исполненного стопа; с лицом - как отвечала модель в n608 (повтор остаётся «одним лицом без слов»)."""
    expression = Expression(emotion="happy") if face else Expression()
    return Answer(Command(actions=(DJ_STOP,), expression=expression), "deepseek")


def test_n608_repeat_with_a_face_closes_the_turn_at_once_with_the_stop_phrase():
    ports = Ports([_repeat(), _repeat()])
    agg = _run(ports)
    assert ports.executed == [("dj_set", {"action": "stop"}), ("stop_music", {})], "повтор не исполнен"
    assert ports.asked == 1, "после повтора исполненного модель больше не зовётся"
    assert ports.said == [OK], ports.said
    assert not set(ports.said) & set(SILENT), "заглушка «Ага, я на связи.» не звучит"
    assert (agg.stage, agg.status) == ("done", "finished") and agg.llm_calls == 1


def test_n608_three_repeats_by_log_make_one_model_call_and_one_phrase():
    """Лог n608: повтор, ответ моста с повтором (`calls=2`), повтор - модель крутит одно и то же; ход всё равно один
    вызов и одна фраза кода."""
    ports = Ports([_repeat(), Answer(_repeat().result, "deepseek", calls=2), _repeat()])
    agg = _run(ports)
    assert ports.asked == 1 and ports.said == [OK]
    assert len(ports.executed) == 2 and agg.llm_calls == 1


def test_repeat_without_a_face_still_closes_with_the_stop_phrase():
    ports = Ports([_repeat(face=False)])
    agg = _run(ports)
    assert ports.asked == 1 and ports.said == [OK] and (agg.stage, agg.status) == ("done", "finished")


def test_new_action_after_the_stop_still_runs_and_is_not_swallowed():
    """Модель просит не повтор, а новое (`dj_set` с другим действием): исполняется, фраза - по его исходу."""
    ports = Ports([Answer(Command(actions=(ToolCall("dj_set", {"action": "status"}),)), "deepseek")])
    _run(ports)
    assert ("dj_set", {"action": "status"}) in ports.executed
