"""Подхват с неузнанным автором: при «прогретом» голосе без имени код просит назваться (Шифу 05.11, после #4427).

Ответ Шифу на вопрос 2 PR #4427: «автор не узнан, вейк — уверенный Саша». Если незнакомый голос уже прогрелся
(свой профиль без имени, несколько реплик за сессию) — робот одной фразой кода просит представиться, а не отказывает
«Голос знакомый, но не уверен чей». Короткий разговор — прежнее поведение.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import decide
from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.execute import Executor, OwnerEvent
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.speaker_adapter import voice_tag
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.voice_warmth import WarmVoices

SASHA_ID, GUEST_ID = "2036e9af", "spk-guest-3"
SASHA = SpeakerRef(person_ref=SASHA_ID, name="Саша", name_state="confirmed")
NAMELESS = {"is_known": True, "speaker_id": GUEST_ID, "name": None, "epithet": "Гость", "confidence": 0.81}
UNKNOWN = {"is_known": False, "candidate_name": "Борис", "candidate_score": 0.41, "utterance_id": "u-x"}
TEA = "ребят поищи у себя в памяти что я говорил про чай"
ASK_READ = phrases.text(f"query.rejected.{kn.ASK_INTRO_REASON}")
ASK_SAVE = phrases.text(f"local.rejected.{kn.ASK_INTRO_REASON}")
REFUSAL = phrases.text(f"query.rejected.{kn.VOICE_UNCONFIRMED_REASON}")


class Clock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


def _warm_after(times, payload=NAMELESS):
    clock = Clock()
    warm = WarmVoices(clock)
    for t in times:
        clock.t = t
        warm.note(payload)
    return warm, clock


# -- критерий «прогрелся» ---------------------------------------------------------------------------------------


def test_short_talk_is_not_warm_long_talk_is():
    assert _warm_after([0.0, 10.0])[0].warm() == ()  # две реплики — мало
    assert _warm_after([0.0, 10.0, 20.0])[0].warm() == ()  # три, но за 20 с — беседа не долгая
    assert _warm_after([0.0, 40.0, 70.0])[0].warm() == (GUEST_ID,)


def test_only_nameless_profiles_with_a_verdict_count():
    named = {"is_known": True, "speaker_id": SASHA_ID, "name": "Саша", "confidence": 0.9}
    assert _warm_after([0.0, 40.0, 70.0], named)[0].warm() == ()
    assert _warm_after([0.0, 40.0, 70.0], {**NAMELESS, "inconclusive": True})[0].warm() == ()
    assert _warm_after([0.0, 40.0, 70.0], {"is_known": False})[0].warm() == ()


def test_warmth_fades_out_of_the_window_and_resets():
    warm, clock = _warm_after([0.0, 40.0, 70.0])
    clock.t = 70.0 + kn.WARM_VOICE_WINDOW_S + 1
    assert warm.warm() == ()
    warm, _ = _warm_after([0.0, 40.0, 70.0])
    warm.reset()
    assert warm.warm() == ()


# -- ход подхвата ---------------------------------------------------------------------------------------------


class MemoryPorts:
    def __init__(self, answers):
        self.answers, self.asked, self.said, self.requests = list(answers), [], [], []
        self.executor = Executor(self._owner)

    def _owner(self, request):
        self.requests.append(request)
        asyncio.get_running_loop().call_soon(
            self.executor.on_event, OwnerEvent(request.turn_id, "done", data={**request.args, "fact_id": 1}))

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        return await self.executor.run(actions, snapshot, turn_id)

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        return None

    def cancel_speech(self, *args, **kwargs):
        return None

    def intent(self, *args, **kwargs):
        return None


def _pickup(payload, text=TEA):
    backlog = Backlog()
    backlog.add(Heard(text, t=343.0, voice_s=4.0, utterance_id="u-a"))
    if payload:
        backlog.note_voice("u-a", voice_tag(payload))
    return decide(Heard("робот", t=350.0, voice_s=0.9), backlog, ["робот"])


def _run(addressed, warm, answers, initiator=SASHA):
    ports = MemoryPorts(answers)
    ids = iter(f"t{n}" for n in range(1, 50))
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids),
                        warm_voices=lambda: tuple(warm))
    engine.session.set_speaker(initiator)
    asyncio.run(engine.run(addressed))
    return ports


_SEARCH = Command(actions=(ToolCall("memory_search", {"query": "чай"}),))
_SAVE = Command(actions=(ToolCall("memory_save", {"fact": "пьёт только зелёный чай"}),))


def test_short_talk_keeps_the_old_refusal():
    """n213-подобный ход, автор не узнан, «Робот» — уверенный Саша, прогретых голосов нет: как в #4427 — отказ кодом."""
    ports = _run(_pickup(UNKNOWN), (), [Answer(_SEARCH, "x")])
    assert ports.requests == [] and ports.said == [REFUSAL]


def test_long_talk_with_a_nameless_voice_asks_to_introduce_instead_of_refusing():
    ports = _run(_pickup(UNKNOWN), (GUEST_ID,), [Answer(_SEARCH, "x")])
    assert ports.requests == []  # память Саши незнакомцу не уходит и сейчас
    assert ports.said == [ASK_READ] and REFUSAL not in ports.said
    assert '"ask_intro":true' in ports.asked[0][-1]["content"]  # модель видит, что код попросит назваться


def test_memory_save_asks_to_introduce_too():
    ports = _run(_pickup(UNKNOWN), (GUEST_ID,), [Answer(_SAVE, "x")])
    assert ports.requests == [] and ports.said == [ASK_SAVE]


def test_silent_biometry_author_with_warm_voice_also_asks():
    """Биометрия фразы не пришла — автор тоже не узнан."""
    ports = _run(_pickup({}), (GUEST_ID,), [Answer(_SEARCH, "x")])
    assert ports.said == [ASK_READ]


def test_warm_voice_that_said_robot_itself_is_not_asked():
    """Прогрет только сам инициатор (его профиль без имени): просить назваться некого сверх него — прежний отказ."""
    guest = SpeakerRef(person_ref=GUEST_ID, name=None, name_state="unknown")
    ports = _run(_pickup(UNKNOWN), (GUEST_ID,), [Answer(_SEARCH, "x")], initiator=guest)
    assert ASK_READ not in ports.said


def test_known_author_is_not_asked_even_with_a_warm_voice():
    """Автор узнан (n213: Саша 0.956) — права у него, просьбы нет."""
    sasha = {"is_known": True, "speaker_id": SASHA_ID, "name": "Саша", "confidence": 0.956, "utterance_id": "u-a"}
    contested = SpeakerRef(name_state="contested")
    ports = _run(_pickup(sasha), (GUEST_ID,), [Answer(_SEARCH, "x"), Answer(Command(speech="Зелёный."), "x")],
                 initiator=contested)
    assert [r.args[kn.PERSON_ARG] for r in ports.requests] == [SASHA_ID] and ASK_READ not in ports.said
