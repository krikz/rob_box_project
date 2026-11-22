"""#5094 (марафон #3 акт 9 n906): «расскажи и покажи» не закрывается одной мигалкой.

Модель на «расскажи анекдот и покажи под него весёлую анимацию» вернула только ``play_animation``.
``play_animation`` входит в ``SILENT_DONE_TOOLS`` и ``INAUDIBLE_TOOLS``, и ход закрывался без речи.
Форма хода (``Form.needs_speech``, ADR-0148) решает по реплике человека: глагол говорения обязывает дать
текст или звук. «Покажи анимацию» без глагола говорения остаётся законным ответом без слов.
"""

import asyncio
import logging

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import (
    Busy,
    Command,
    Expression,
    Form,
    Invalid,
    ToolCall,
    current_form,
    explain,
    parse,
    validate,
)
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import speech_asked
from rob_box_dialog.turn import Answer, TurnEngine, _Turn, command_form

N906 = "робот расскажи анекдот и покажи под него веселую анимацию"
TELL = "расскажи анекдот и покажи под него весёлую анимацию"
SHOW = "покажи анимацию радости"
SAY = "скажи что-нибудь смешное"
ANSWER = "ответь и подмигни"
NEGATED = "не рассказывай, просто покажи анимацию"
TOOLS = ["play_animation", "play_sound", "memory_save"]
BLINK = {"actions": [{"tool": "play_animation", "args": {"animation": "happy"}}]}
SPEECH = {"speech": "Заходит робот в бар.", "actions": [{"tool": "play_animation", "args": {"animation": "happy"}}]}
SOUND = {"actions": [{"tool": "play_sound", "args": {"sound": "robot_happy"}}]}
FACE = {"expression": {"emotion": "happy"}}
TOLD = {"perform": {"kind": "poem", "segments": ["Шутка.", "Ещё одна."]}}
EDIT = {"edit_pending": True, "perform": {"kind": "poem", "segments": ["Шутка.", "Ещё одна."]}}
HEARD = {"actions": [{"tool": "memory_save", "args": {"fact": "анекдот про робота"}}]}


def _check(raw, form, tools=TOOLS):
    return validate(parse(raw), tools, form=form)


def _told():
    return Form(passed=False, needs_speech=True)


def test_n906_phrase_asks_for_speech_and_a_bare_show_does_not():
    assert speech_asked(N906) and speech_asked(TELL)
    assert speech_asked(SAY) and speech_asked(ANSWER)
    assert speech_asked("объясни, почему небо синее")
    assert speech_asked("прочти стих и покажи лицо")
    assert speech_asked("перескажи сюжет")
    assert not speech_asked(SHOW)
    assert not speech_asked("улыбнись")
    assert not speech_asked(NEGATED)
    assert not speech_asked("рассказчик молчит")  # основа, не форма глагола


def test_tell_and_only_play_animation_is_refused_with_the_hint():
    result = _check(BLINK, _told())
    assert isinstance(result, Invalid)
    assert [v.reason for v in result.all] == [kn.VIOLATION_SPEECH_ASKED]
    note = explain(result)
    assert "просили рассказать" in note and "мигалка" in note and "speech" in note


def test_show_animation_without_a_speech_verb_stays_ok():
    assert not speech_asked(SHOW)
    cmd = _check(BLINK, Form(passed=False))
    assert isinstance(cmd, Command) and cmd.actions[0].tool == "play_animation"


def test_tell_with_speech_or_a_perform_is_ok():
    said = _check(SPEECH, _told())
    assert isinstance(said, Command) and said.speech and said.actions[0].tool == "play_animation"
    assert isinstance(_check(TOLD, _told()), Command)


def test_tell_with_only_a_sound_or_a_face_is_refused():
    for raw in (SOUND, FACE, {"actions": [], "passed": None}):
        result = _check(raw, _told())
        assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_SPEECH_ASKED, raw


def test_say_and_answer_border_verbs_refuse_a_bare_blink():
    for text in (SAY, ANSWER, "ответьте на вопрос", "скажите анекдот"):
        assert speech_asked(text)
        result = _check(BLINK, Form(passed=False, needs_speech=speech_asked(text)))
        assert isinstance(result, Invalid) and result.reason == kn.VIOLATION_SPEECH_ASKED


def test_edit_pending_is_not_judged_as_a_new_speech_request():
    idle = _check(EDIT, _told())
    assert isinstance(idle, Invalid) and all(v.reason != kn.VIOLATION_SPEECH_ASKED for v in idle.all)
    running = validate(parse(EDIT), TOOLS, form=_told(), busy=Busy(active_segment=0))
    assert isinstance(running, Command) and running.edit_pending


class _Ports:
    def __init__(self):
        self.forms = []

    async def ask(self, messages, tools, timeout_s):
        self.forms.append(current_form())
        told = Command(speech="Заходит робот в бар.", actions=(ToolCall("play_animation", {"animation": "happy"}),))
        return Answer(told)

    async def execute(self, actions, snapshot, turn_id):
        return [
            ActionResult(
                f"{turn_id}/{n}",
                snapshot.epoch,
                snapshot.agent,
                a.tool,
                kn.TOOL_CLASSES[a.tool],
                "done",
                None,
                dict(a.args),
            )
            for n, a in enumerate(actions)
        ]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        return "sp"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass


def test_command_form_sets_needs_speech_and_the_turn_log_names_it(caplog):
    snap = Session("personality").snapshot
    told = command_form(_Turn("t1", 0, TELL, VOICE, deadline=None), snap)
    assert told.needs_speech
    shown = command_form(_Turn("t1", 0, SHOW, VOICE, deadline=None), snap)
    assert not shown.needs_speech
    assert "needs_speech" not in kn.FORM_FIELDS  # схему поле не закрывает
    ports = _Ports()
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    with caplog.at_level(logging.INFO):
        asyncio.run(engine.run(Addressed(TELL, VOICE)))
    assert ports.forms and ports.forms[0].needs_speech
    assert any("форма команды:" in r.getMessage() and "речь=нужна" in r.getMessage() for r in caplog.records)


def test_a_heard_tool_outside_the_silent_sets_answers_the_request():
    cmd = _check(HEARD, _told())
    assert isinstance(cmd, Command) and cmd.actions[0].tool == "memory_save"
    face = _check(FACE, Form(passed=False))  # без просьбы говорить лицо законно
    assert isinstance(face, Command) and face.expression == Expression(emotion="happy")


def test_speech_push_owed_ignores_a_show_beside_a_speech_verb():
    # «расскажи и покажи»: повтор и без запроса, и когда лицо просили. Голая анимация и тишина — нет.
    assert kn.speech_push_owed(True, False, True, False)
    assert kn.speech_push_owed(True, False, False, False)
    assert not kn.speech_push_owed(False, False, True, False)
    assert not kn.speech_push_owed(False, True, True, False)  # лицо просили, говорить — нет
    assert kn.speech_push_owed(False, True, False, False)  # запрос без лица, как раньше
    assert not kn.speech_push_owed(True, True, True, True)  # путь пения / не динамики


class _BlinkPorts:
    """Мост уже судил форму: просьба говорить — ``Invalid(speech_asked)``, голая «покажи» — законный Command.

    Ход сам Command не перепроверяет: Command ушёл бы в ``_act`` и мимо ``_close_invalid``.
    """

    def __init__(self, speech_asked=False):
        self.speech_asked = speech_asked
        self.asked, self.executed, self.said = [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        blink = ToolCall("play_animation", {"animation": "happy"})
        if self.speech_asked:
            return Answer(Invalid(kn.VIOLATION_SPEECH_ASKED, salvage=(blink,)))
        return Answer(Command(actions=(blink,)))

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        return [
            ActionResult(
                f"{turn_id}/{n}",
                snapshot.epoch,
                snapshot.agent,
                a.tool,
                kn.TOOL_CLASSES[a.tool],
                "done",
                None,
                dict(a.args),
            )
            for n, a in enumerate(actions)
        ]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return "sp"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass


def _run(text, speech_asked=False):
    ports = _BlinkPorts(speech_asked=speech_asked)
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.")
    asyncio.run(engine.run(Addressed(text, VOICE)))
    return ports


def test_n906_blink_only_is_pushed_then_the_code_says_it_could_not_tell(caplog):
    with caplog.at_level(logging.WARNING):
        ports = _run(N906, speech_asked=True)
    # первый вызов + повторы за словами; мост свой повтор не тратит (words_only)
    assert len(ports.asked) == 1 + kn.SPEECH_PUSHES_MAX
    # каждый повтор кладёт подсказку в следующий запрос; последний запрос видит обе
    hinted = [
        call
        for call in ports.asked
        if any(isinstance(m, dict) and "speech_asked" in str(m.get("content")) for m in call)
    ]
    assert len(hinted) == kn.SPEECH_PUSHES_MAX
    assert ports.executed == [[("play_animation", {"animation": "happy"})]]  # мигалка просьбы исполнена один раз
    heard = " ".join(ports.said)
    assert "Рассказать не вышло" in heard
    assert "Угу, я тут" not in heard
    assert any("speech_asked" in r.getMessage() and "повтор за словами" in r.getMessage() for r in caplog.records)


def test_a_bare_show_stays_a_silent_blink():
    ports = _run(SHOW)
    assert len(ports.asked) == 1  # говорить не просили — повтора за словами нет
    assert ports.executed == [[("play_animation", {"animation": "happy"})]]
    assert ports.said == []


def test_speech_asked_phrase_is_in_the_catalog():
    from rob_box_dialog import phrases

    assert kn.SPEECH_ASKED_PHRASE in kn.PHRASE_KEYS
    assert kn.INVALID_COMMAND_PHRASES[kn.VIOLATION_SPEECH_ASKED] == kn.SPEECH_ASKED_PHRASE
    assert phrases.text(kn.SPEECH_ASKED_PHRASE) == "Рассказать не вышло. Скажи ещё раз?"
