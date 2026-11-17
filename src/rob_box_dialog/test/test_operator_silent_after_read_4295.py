"""#4295: ТАРС (оператор, dialog v2) на «статус контейнеров» прочитал ``container_status`` и ответил одним лицом без слов
(живой лог #4286, 03.11: ``verdict=command actions=[] fields_moved=['expression.mouth']``) - оператор в Telegram ответа
не получил. Класс К2 (#4157): у Личности ответ по данным закрывает ``turn.data_close``, потому что её голый ответ
«ждёт слов» (``words_owed`` - только агент с речью в динамики). У оператора (шлем/чат) слова после чтения не были
«должны»: ход ``done`` молча. Теперь прочитанное и не сказанное - всегда ответ кода по данным, кому бы ни.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import Addressed
from rob_box_dialog.command import Command, Expression, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine

#: Данные ``container_status`` живого прогона (4286_sup_live2.log, строка 24): имя и аптайм, без рестартов и CPU.
CONTAINERS = {"containers": [
    {"name": "cadvisor", "restart_count": None, "cpu_rate_per_sec": None, "memory_bytes": 0.0, "uptime_sec": 1014.14},
    {"name": "lslidar", "restart_count": None, "cpu_rate_per_sec": None, "memory_bytes": 0.0, "uptime_sec": 1016.14},
    {"name": "nav2", "restart_count": None, "cpu_rate_per_sec": None, "memory_bytes": 0.0, "uptime_sec": 1016.14}]}


class Ports:
    def __init__(self, answers, outcomes=None):
        self.answers, self.outcomes = list(answers), outcomes or {}
        self.said, self.asked = [], 0

    async def ask(self, messages, tools, timeout_s):
        self.asked += 1
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.call_class(a.tool, a.args),
                             "done", None, dict(self.outcomes.get(a.tool, {}))) for n, a in enumerate(actions)]

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


def _follow(data):
    return ({"role": "tool", "content": data},)


def _query():
    return Answer(Command(actions=(ToolCall("container_status", {}),)), "deepseek", follow=_follow)


def _face():
    """Второй ответ живого лога: ``command`` с одним лицом (``thinking``), без речи и действий."""
    return Answer(Command(expression=Expression(emotion="thinking")), "deepseek", follow=_follow)


def _run(ports, text, agent="operator"):
    engine = TurnEngine(Session(agent), ports, system_prompt="Ты ТАРС.")
    return asyncio.run(engine.run(Addressed(text=text, source="telegram")))


def test_operator_bare_face_after_read_is_answered_by_code():
    """Было: ``said == []`` (оператор не получил ответа). Стало: ответ кода по данным ``container_status``."""
    ports = Ports([_query(), _face(), _face(), _face(), _face(), _face()], {"container_status": CONTAINERS})
    _run(ports, "статус контейнеров")
    assert ports.said, "оператор не получил ни слова после чтения container_status (#4295)"
    assert ports.said == ["Контейнеров в отчёте — 3, работают от 17 мин до 17 мин."]


def test_operator_asked_for_face_only_stays_silent():
    """Лицо без слов по просьбе («покажи молча») - норма и у оператора, чтения не было."""
    ports = Ports([_face()])
    _run(ports, "покажи молча что ты задумался")
    assert ports.said == []


def test_operator_face_after_read_with_silent_ask_stays_silent():
    ports = Ports([_query(), _face()], {"container_status": CONTAINERS})
    _run(ports, "молча покажи статус контейнеров лицом")
    assert ports.said == []
