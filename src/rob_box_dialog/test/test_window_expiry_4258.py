"""#4258: окно диалога со сроком давности. Ходы вчерашней e2e-роли дожили до утра и робот назвал Шифу Сашей.

Сброс решает код: пауза между адресованными ходами дольше ``DIALOG_WINDOW_IDLE_RESET_S`` и смена режима e2e (в обе
стороны, хост зовёт ``TurnEngine.reset_window``). Проверяется то, что уходит модели на следующем ходу, не текст промпта.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.window_reset import REASON_E2E_OFF, REASON_E2E_ON, WindowGuard

OLD = "Привет, я Саша, ночной инженер"
NEW = "расскажи анекдот"


class Ports:
    def __init__(self):
        self.asked, self.n = [], 0

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return Answer(Command(speech="Ладно."), "deepseek")

    async def execute(self, actions, snapshot, turn_id):
        return []

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.n += 1
        return f"sp{self.n}"

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


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


def _setup():
    ports, clock = Ports(), Clock()
    return ports, clock, TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", clock=clock)


def _say(engine, text):
    asyncio.run(engine.run(Addressed(text, VOICE)))


def _in_last_request(ports, needle):
    return any(needle in str(m.get("content", "")) for m in ports.asked[-1])


def test_turn_inside_the_threshold_keeps_the_window():
    ports, clock, engine = _setup()
    _say(engine, OLD)
    clock.now += kn.DIALOG_WINDOW_IDLE_RESET_S - 1
    _say(engine, NEW)
    assert _in_last_request(ports, OLD)


def test_turn_after_a_long_pause_starts_with_an_empty_window():
    ports, clock, engine = _setup()
    _say(engine, OLD)
    epoch = engine.session.snapshot.epoch
    clock.now += kn.DIALOG_WINDOW_IDLE_RESET_S + 1  # ночь между ходами
    _say(engine, NEW)
    assert _in_last_request(ports, NEW) and not _in_last_request(ports, OLD)
    assert engine.session.snapshot.epoch == epoch + 1


def test_the_pause_is_counted_from_the_end_of_the_last_turn_not_its_start():
    ports, clock, engine = _setup()
    _say(engine, OLD)
    clock.now += 100
    _say(engine, "ещё раз")
    clock.now += kn.DIALOG_WINDOW_IDLE_RESET_S - 1
    _say(engine, NEW)
    assert _in_last_request(ports, OLD)


def test_first_turn_never_resets():
    ports, clock, engine = _setup()
    epoch = engine.session.snapshot.epoch
    _say(engine, NEW)
    assert engine.session.snapshot.epoch == epoch


def test_e2e_switch_either_way_empties_the_window():
    for reason in (REASON_E2E_OFF, REASON_E2E_ON):
        ports, clock, engine = _setup()
        _say(engine, OLD)
        clock.now += 60  # пауза короткая: сбрасывает именно смена режима
        engine.reset_window(reason)
        _say(engine, NEW)
        assert _in_last_request(ports, NEW) and not _in_last_request(ports, OLD), reason


def test_guard_without_turns_is_never_expired():
    guard = WindowGuard(10)
    assert guard.gap(5.0) is None and not guard.expired(1e9)
    guard.touch(0.0)
    assert not guard.expired(10.0) and guard.expired(10.1)
