"""#4509: окно ответа на приветствие — ответ без лица, голое «робот» в окне, двойной ответ из бэклога.

Живой робот 06.11 ~16:20 UTC (Sound Pi, ``docker logs voice-assistant``):

    💬 [reply-window] greeting Денчик: принят source=bare_wake text='робот'
    🗒️ [backlog] pickup n=0 speakers=- turn=03b1a2b8051a
    📊 [turn] … stage=addressed status=finished latency_ms=4 llm_calls=0         ← голое «робот» в окне: тишина

    💬 [reply-window] greeting Денчик: не ответ → бэклог faces=0 text='Я не забыл про тебя, говорю.'   ← нет лица
    ✅ ПРИНЯТО: Ты что ли что-то, робот? Я сказал тебе, я не забыл.  → ход 'Принял, не забыл — слышу тебя. …'
    ✅ ПРИНЯТО: Роберт → 🗒️ [backlog] pickup n=1 → второй ответ 'И я тебя не забыл, …'           ← двойной ответ

Поведение модулей ``address``, ``backlog``, ``speaker_adapter`` и ``turn``.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import BARE_WAKE, GREETING_REPLY, GREETING_WAKE, VOICE, Addressed, NotAddressed, decide
from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.session import Session
from rob_box_dialog.speaker_adapter import answer_to_greeting, greeting_frame
from rob_box_dialog.turn import TurnEngine

WAKES = kn.DEFAULT_WAKE_WORDS
#: Конец приветствия (``SpeechEvent finished``).
GREETED_AT = 1000.0


def _window(clock_t: float = GREETED_AT):
    """Кадр ``GreetingReply`` с открытым окном: приветствие договорено в ``clock_t`` (как ``greeting_window``)."""
    session = Session("personality", clock=lambda: clock_t)
    session.push_scene("GreetingReply", "code", ttl_s=180.0, payload=greeting_frame("p1", "Денчик", "g1"))
    session.arm_scene(
        "GreetingReply", kn.IDENTITY_MIC_GRACE_S + kn.REPLY_WINDOW_S, {"asked_at": clock_t + kn.IDENTITY_MIC_GRACE_S}
    )
    return session.snapshot.top("GreetingReply")


# --- дефект 3: лицо в кадре не условие ---------------------------------------------------------------------------


def test_reply_without_any_face_in_frame_is_a_greeting_reply():
    said = "Я не забыл про тебя, говорю."
    heard = Heard(said, t=GREETED_AT + 6.0, faces=frozenset())
    assert decide(heard, Backlog(), WAKES, greeting=_window()) == Addressed(said, GREETING_REPLY)
    assert answer_to_greeting(_window(), heard) == "voice"


def test_reply_in_the_mic_grace_right_after_the_greeting_is_a_reply():
    """Дефект 1 со стороны диалога: окно идёт с конца приветствия — эхо в грейсе отсеивает ``stt_node`` словами."""
    said = "Добрый вечер, нет, не забыл про тебя."
    heard = Heard(said, t=GREETED_AT + 1.5)
    assert decide(heard, Backlog(), WAKES, greeting=_window()) == Addressed(said, GREETING_REPLY)


def test_after_the_window_the_reply_is_backlog_as_before():
    heard = Heard("Я не забыл про тебя, говорю.", t=GREETED_AT + kn.IDENTITY_MIC_GRACE_S + kn.REPLY_WINDOW_S + 1)
    assert decide(heard, Backlog(), WAKES, greeting=_window()) == NotAddressed("no_wake")


def test_greeting_reply_grammar_and_face_are_still_labels():
    frame = _window()
    assert answer_to_greeting(frame, Heard("привет", t=GREETED_AT + 3)) == "grammar"
    assert answer_to_greeting(frame, Heard("я тут работаю", t=GREETED_AT + 3, faces=frozenset({"денчик"}))) == "face"
    assert answer_to_greeting(frame, Heard("   ", t=GREETED_AT + 3)) == ""


# --- дефект 2: голое «робот» в окне приветствия → «Слушаю.» ----------------------------------------------------------


def test_bare_wake_in_the_greeting_window_is_greeting_wake_not_a_silent_reflex():
    decision = decide(Heard("робот", t=GREETED_AT + 8.0), Backlog(), WAKES, greeting=_window())
    assert decision == Addressed("", GREETING_WAKE) and decision.utterance_id == ""


def test_bare_wake_outside_the_greeting_window_is_the_usual_reflex():
    assert decide(Heard("робот", t=GREETED_AT + 8.0), Backlog(), WAKES) == Addressed("", BARE_WAKE)


def test_bare_wake_in_the_window_still_picks_up_a_fresh_backlog():
    backlog = Backlog()
    backlog.add(Heard("Включи музыку", t=GREETED_AT + 4.0))
    decision = decide(Heard("робот", t=GREETED_AT + 8.0), backlog, WAKES, greeting=_window())
    assert decision.pickup and decision.text == "Включи музыку"


class Ports:
    def __init__(self):
        self.asked, self.said = [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(messages)
        raise AssertionError("LLM не зовётся")

    async def execute(self, actions, snapshot, turn_id):
        return []

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


def _run(addressed):
    ports = Ports()
    agg = asyncio.run(TurnEngine(Session("personality"), ports, system_prompt="Ты робот.").run(addressed))
    return ports, agg


def test_greeting_wake_turn_says_listening_by_code_without_the_model():
    ports, agg = _run(Addressed("", GREETING_WAKE))
    assert ports.asked == [] and ports.said == [phrases.text(kn.LISTEN_ONLY_PHRASE)] == ["Слушаю."]
    assert agg.llm_calls == 0 and agg.status == "finished" and agg.stage == "done"


def test_plain_bare_wake_turn_stays_a_silent_reflex():
    ports, agg = _run(Addressed("", BARE_WAKE))
    assert ports.said == [] and ports.asked == [] and agg.llm_calls == 0


# --- дефект 4: фраза, на которую ход уже ответил, не подхватывается снова ----------------------------------------------


def test_backlog_phrase_seen_by_an_answered_turn_is_not_picked_up_again():
    """Лог: «Я не забыл…» в бэклог → «Ты что ли …, робот? …» (ход видит бэклог в снимке и отвечает) → «Роберт»
    (голое): подхват был n=1 и второй ответ. Теперь — n=0, обычный рефлекс."""
    backlog = Backlog()
    lost = Heard("Я не забыл про тебя, говорю.", t=100.0)
    assert decide(lost, backlog, WAKES) == NotAddressed("no_wake")
    backlog.add(lost)
    turn = decide(Heard("Ты что ли что-то, робот? Я сказал тебе, я не забыл.", t=103.0), backlog, WAKES)
    assert turn.source == VOICE and [p["text"] for p in turn.background] == [lost.text]
    again = decide(Heard("Роберт", t=108.0), backlog, WAKES)
    assert again == Addressed("", BARE_WAKE), "фраза уже учтена отвеченным ходом — второго ответа нет"


def test_phrase_after_the_answered_turn_is_still_picked_up():
    backlog = Backlog()
    backlog.add(Heard("старое", t=100.0))
    decide(Heard("Робот, привет", t=103.0), backlog, WAKES)
    backlog.add(Heard("Включи музыку", t=105.0))
    picked = decide(Heard("Робот", t=108.0), backlog, WAKES)
    assert picked.pickup and picked.text == "Включи музыку"
