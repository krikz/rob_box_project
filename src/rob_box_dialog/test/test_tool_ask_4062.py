"""Вопрос тула как окно ответа (#4062; ADR-0148: фразу об успехе строит код из результата).

Живой робот 27.10 14:47: «Робот, удали из библиотеки трек X» → тул спросил «Удалить «X»? Скажи «да, удаляй».», а на
«Робот, да, удаляй» ``music_library`` не было в срезе хода, модель сказала «Удаляю все четыре…» без вызова тула -
треки остались. Здесь ход целиком на фейковых портах: владелец библиотеки ведёт себя как тул (без ``confirm``
спрашивает, ``confirm`` после вопроса удаляет), и проверяется ПОВЕДЕНИЕ: что удалено, что сказано, сколько вызовов LLM.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog import tool_ask
from rob_box_dialog.address import (BACKLOG_PICKUP, BARE_WAKE, GREETING_REPLY, IDENTITY_ANSWER, PERFORM_ANSWER,
                                    SOURCE_OPERATOR, SOURCE_TELEGRAM, VOICE, Addressed)
from rob_box_dialog.command import Ask, Command, Expression, ToolCall
from rob_box_dialog.respond import ActionResult, asks, failed, outcome_emotion, phrase_from_result, phrase_key
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools
from rob_box_dialog.turn import Answer, TurnEngine, command_face

QUESTION = "Удалить «Кот и ёж»? Скажи «да, удаляй»."  # литерал, а не сборка из каталога: ловит дрейф ru.yaml
DELETED = "Удалил «Кот и ёж» из библиотеки."
KEPT = "Не удаляю."
NEUTRAL = Expression().emotion  # лицо команды модели по умолчанию
ASKING = "neutral"  # эмоция речи вопроса тула - литерал: ловит дрейф ToolQuestion.emotion
ASK_DELETE = ToolCall("music_library", {"action": "delete", "track_id": "t-1"})
CONFIRM_DELETE = ("music_library", {"action": "delete", "track_id": "t-1", "confirm": True})


class Library:
    """Владелец библиотеки как тул ``music_library``: без ``confirm`` спрашивает (``needs_confirmation``), ``confirm``
    после вопроса о том же треке удаляет; ``forget`` - тул потерял вопрос (перезапуск) и спросит снова."""

    def __init__(self):
        self.tracks = {"t-1": "Кот и ёж", "t-2": "Дождь"}
        self.asked = set()

    def __call__(self, args):
        track_id, title = args.get("track_id"), self.tracks.get(args.get("track_id"), "")
        if args.get("action") != "delete":
            return "done", None, {}
        if args.get("confirm") is True and track_id in self.asked:
            self.asked.discard(track_id)
            del self.tracks[track_id]
            return "done", None, {"track_id": track_id, "title": title}
        self.asked.add(track_id)
        return "rejected", "needs_confirmation", {"track_id": track_id, "title": title}


class Ports:
    """Транспорт хода: ответы LLM по очереди; исполнитель отдаёт результат как настоящий (класс ТУЛА, данные =
    аргументы вызова поверх ответа тула); речь записывается с эмоцией. ``during_execute`` - что случится, пока тул
    отвечает (стоп)."""

    def __init__(self, library, answers=()):
        self.library = library
        self.answers = list(answers)
        self.asked, self.executed, self.said = [], [], []
        self.during_execute = None

    async def ask(self, messages, tools, timeout_s):
        self.asked.append([t.name for t in tools])
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        if self.during_execute is not None:
            self.during_execute()
        out = []
        for n, a in enumerate(actions):
            status, reason, data = self.library(dict(a.args))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                                    status, reason, {**dict(a.args), **data}))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append((text, emotion))
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


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


def _engine(ports, clock=None):
    ids = iter(f"t{n}" for n in range(1, 100))
    session = Session("personality", clock=clock or Clock())
    return TurnEngine(session, ports, system_prompt="Ты робот.", new_id=lambda: next(ids))


def _run(engine, text, source=VOICE):
    return asyncio.run(engine.run(Addressed(text, source, pickup=source == BACKLOG_PICKUP)))


def _confirmed(ports):
    """Подтверждённые вызовы удаления, что ушли исполнителю."""
    return [c for batch in ports.executed for c in batch if c[1].get("confirm")]


def _asked_to_delete(answers=(), expression=Expression()):
    """Шаг 1: модель зовёт ``delete`` без ``confirm``, тул спрашивает. Возвращает (движок, порты, владелец)."""
    library = Library()
    clock = Clock()
    ports = Ports(library, [Answer(Command(actions=(ASK_DELETE,), expression=expression), "deepseek"), *answers])
    engine = _engine(ports, clock)
    _run(engine, "Робот, удали из библиотеки трек «Кот и ёж»")
    engine.clock = clock
    return engine, ports, library


# ---------------------------------------------------------------------------------------------------------------
# шаг 1: вопрос тула - фраза из каталога, окно открыто, лицо не грустное
# ---------------------------------------------------------------------------------------------------------------

def test_tool_question_is_said_and_opens_the_answer_window():
    engine, ports, library = _asked_to_delete()
    assert [text for text, _ in ports.said] == [QUESTION]
    assert library.tracks["t-1"] == "Кот и ёж", "вопрос ничего не удаляет"
    asked = tool_ask.pending(engine.session)
    assert (asked.tool, asked.reason, dict(asked.args)) == ("music_library", "needs_confirmation",
                                                            {"action": "delete", "track_id": "t-1"})
    frame = engine.session.snapshot.top(kn.TOOL_ASK_SCENE)
    assert frame.ttl_until == pytest.approx(engine.clock.now + kn.TOOL_ASK_TTL_S)


QUESTION_RESULT = ActionResult("t1/0", 0, "personality", "music_library", "action.local", "rejected",
                               "needs_confirmation", {"action": "delete", "track_id": "t-1", "title": "Кот и ёж"})


@pytest.mark.parametrize("face", [Expression(), Expression(emotion="happy"), Expression(emotion="sad")])
def test_question_is_said_with_the_questions_emotion_not_the_models(face):
    """Вопрос на подтверждение - не провал и не настроение модели (ревью PR #4100: с ``happy`` модели вопрос звучал
    радостно): речь - эмоцией вопроса (нейтрально), лицо модели итог не красит (как у провала)."""
    engine, ports, _ = _asked_to_delete(expression=face)
    assert ports.said == [(QUESTION, ASKING)]
    assert command_face(face, [QUESTION_RESULT]) is None


def test_question_result_is_asking_and_a_real_refusal_is_still_sad():
    refusal = ActionResult("t1/0", 0, "personality", "music_library", "action.local", "rejected", "disk_on_fire",
                           {"action": "delete", "track_id": "t-1"})
    assert asks(QUESTION_RESULT) and not failed(QUESTION_RESULT)
    assert phrase_from_result(QUESTION_RESULT).text == QUESTION
    assert phrase_from_result(QUESTION_RESULT).emotion == ASKING == outcome_emotion([QUESTION_RESULT], "happy")
    assert not asks(refusal) and failed(refusal) and phrase_from_result(refusal).emotion == "sad"
    assert outcome_emotion([QUESTION_RESULT, refusal], "happy") == "sad", "провал рядом с вопросом - грустно"


def test_question_emotion_is_a_face_and_not_sadness():
    for question in kn.TOOL_QUESTIONS.values():
        assert question.emotion in kn.EMOTIONS and question.emotion != "sad"


# ---------------------------------------------------------------------------------------------------------------
# шаг 2: «да» исполняет код, «нет» - фраза кода; LLM не зовётся
# ---------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("text", [
    "Робот, да, удаляй", "да", "удаляй", "Робот, да, удаляй эту песню.", "ага, удали её насовсем", "Да, удаляй!",
    "давай, стирай", "да, удаляй трек",
])
def test_yes_deletes_by_code_and_the_phrase_comes_from_the_result(text):
    engine, ports, library = _asked_to_delete()
    agg = _run(engine, text)
    assert "t-1" not in library.tracks and library.tracks == {"t-2": "Дождь"}, "удалён ровно спрошенный трек"
    assert ports.executed[-1] == [("music_library", {"action": "delete", "track_id": "t-1", "confirm": True})]
    assert ports.said[-1] == (DELETED, None)
    assert agg.llm_calls == 0 and len(ports.asked) == 1, "подтверждение не идёт через LLM"
    assert tool_ask.pending(engine.session) is None, "ответ снимает окно"


@pytest.mark.parametrize("text", ["нет", "Робот, не надо", "нет, не удаляй", "отмена", "оставь", "нет, удали"])
def test_no_keeps_the_track_and_code_says_so(text):
    engine, ports, library = _asked_to_delete()
    agg = _run(engine, text)
    assert library.tracks["t-1"] == "Кот и ёж"
    assert len(ports.executed) == 1, "тул на «нет» не зовётся"
    assert ports.said[-1] == (KEPT, None) and agg.llm_calls == 0
    assert tool_ask.pending(engine.session) is None


def test_yes_after_the_window_expired_deletes_nothing():
    """Окно истекло: «да, удаляй» - обычная реплика к LLM без вопроса; модель удалять нечего не зовёт - трек цел."""
    engine, ports, library = _asked_to_delete([Answer(Command(speech="А что удалить?"), "deepseek")])
    engine.clock.now += kn.TOOL_ASK_TTL_S + 1
    agg = _run(engine, "Робот, да, удаляй")
    assert library.tracks["t-1"] == "Кот и ёж" and len(ports.executed) == 1
    assert agg.llm_calls == 1 and ports.said[-1] == ("А что удалить?", NEUTRAL)


def test_answer_does_not_carry_to_a_second_yes():
    """Окно - на один ответ: второе «да» (уже без вопроса) ничего не подтверждает кодом."""
    engine, ports, library = _asked_to_delete([Answer(Command(speech="Что удалить?"), "deepseek")])
    _run(engine, "да, удаляй")
    _run(engine, "да, удаляй")
    assert [c for batch in ports.executed for c in batch if c[1].get("confirm")] == [
        ("music_library", {"action": "delete", "track_id": "t-1", "confirm": True})]
    assert len(ports.asked) == 2


def test_tool_asking_again_on_the_confirmed_call_reopens_the_window():
    """Тул потерял вопрос (перезапуск) и спросил снова: звучит вопрос, окно открыто заново, ничего не удалено."""
    engine, ports, library = _asked_to_delete()
    library.asked.clear()
    _run(engine, "да, удаляй")
    assert library.tracks["t-1"] == "Кот и ёж"
    assert ports.said[-1] == (QUESTION, ASKING), "вопрос тула и на пути ответа - эмоцией вопроса"
    assert tool_ask.pending(engine.session) is not None
    _run(engine, "да, удаляй")
    assert "t-1" not in library.tracks and ports.said[-1] == (DELETED, None)


def test_yes_in_silence_confirms_nothing():
    engine, ports, library = _asked_to_delete()
    engine.session.push_scene("Silence", "user")
    assert tool_ask.pending(engine.session) is None, "тишина снимает окно"
    _run(engine, "да, удаляй")
    assert library.tracks["t-1"] == "Кот и ёж" and len(ports.executed) == 1 and ports.said == [(QUESTION, ASKING)]


# ---------------------------------------------------------------------------------------------------------------
# чужое «да» необратимое не подтверждает (ревью PR #4100, блокирующее)
# ---------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("source", [IDENTITY_ANSWER, GREETING_REPLY, PERFORM_ANSWER, BACKLOG_PICKUP])
def test_yes_from_another_window_or_the_backlog_deletes_nothing(source):
    """(a) «да» без вейка в окне ДРУГОГО вопроса робота («это ты, Саша?», приветствие, «выключить и зачитать?») или
    подхваченная из бэклога чужая фраза - ответ не тулу: трек цел, подтверждения нет, ход идёт своим путём."""
    engine, ports, library = _asked_to_delete([Answer(Command(speech="Угу."), "deepseek")])
    _run(engine, "да", source)
    assert library.tracks["t-1"] == "Кот и ёж" and _confirmed(ports) == []
    assert tool_ask.pending(engine.session) is not None, "окно ждёт ответа от того, кто может ответить"


@pytest.mark.parametrize("source, answers", [
    (VOICE, True), (SOURCE_TELEGRAM, True), (SOURCE_OPERATOR, True),
    (IDENTITY_ANSWER, False), (GREETING_REPLY, False), (PERFORM_ANSWER, False), (BACKLOG_PICKUP, False),
    (BARE_WAKE, False),
])
def test_who_may_answer_the_tools_question(source, answers):
    engine, _, _ = _asked_to_delete()
    assert (tool_ask.reply(engine.session, "да, удаляй", source) is not None) is answers


def test_answer_sources_are_the_addressed_sources_that_carry_a_wake_or_a_channel():
    assert set(tool_ask.ANSWER_SOURCES) == {VOICE, SOURCE_TELEGRAM, SOURCE_OPERATOR}


def test_models_new_question_takes_the_yes_away_from_the_tool():
    """(b) После вопроса тула модель спросила своё («Тогда, может, включить что-нибудь другое?», ``expects=yes_no``):
    «да» - ответ ей, а не подтверждение удаления."""
    other = Command(speech="Тогда, может, включить что-нибудь другое?",
                    ask=Ask(expects="yes_no", text="Тогда, может, включить что-нибудь другое?"))
    later = [Answer(Command(speech="Хорошо."), "deepseek") for _ in range(2)]
    engine, ports, library = _asked_to_delete([Answer(other, "deepseek"), *later])
    _run(engine, "а что ещё есть")
    assert engine.session.snapshot.pending_ask is not None and tool_ask.pending(engine.session) is None
    _run(engine, "да")
    _run(engine, "да, удаляй")
    assert library.tracks["t-1"] == "Кот и ёж" and _confirmed(ports) == []


def test_stop_closes_the_tools_question():
    """(c) «Робот, стоп» после вопроса: окно снято, следующее «да, удаляй» - обычная реплика к LLM, трек цел."""
    engine, ports, library = _asked_to_delete([Answer(Command(speech="Что удалить?"), "deepseek")])
    _run(engine, "стоп")
    assert tool_ask.pending(engine.session) is None
    agg = _run(engine, "да, удаляй")
    assert library.tracks["t-1"] == "Кот и ёж" and _confirmed(ports) == [] and agg.llm_calls == 1


def test_silence_turn_closes_the_tools_question_for_good():
    """«Робот, помолчи» после вопроса, затем «Робот, говори»: окно не вернулось, «да, удаляй» трек не удаляет."""
    engine, ports, library = _asked_to_delete([Answer(Command(speech="Что удалить?"), "deepseek")])
    _run(engine, "помолчи")
    assert engine.session.snapshot.top("Silence") is not None and tool_ask.pending(engine.session) is None
    _run(engine, "говори")
    assert engine.session.snapshot.top("Silence") is None
    _run(engine, "да, удаляй")
    assert library.tracks["t-1"] == "Кот и ёж" and _confirmed(ports) == []


@pytest.mark.parametrize("kind, set_by", [("Identity", "code"), ("GreetingReply", "code"), ("PerformAsk", "code"),
                                          ("Silence", "user"), ("OperatorHold", "operator")])
def test_robots_other_question_silence_or_operator_close_the_window(kind, set_by):
    """Кадр нового вопроса робота, тишины или оператора снимает окно, и снятие кадра его не возвращает."""
    assert kind in kn.TOOL_ASK_CLOSED_BY
    engine, ports, library = _asked_to_delete([Answer(Command(speech="Что удалить?"), "deepseek")])
    engine.session.push_scene(kind, set_by)
    assert tool_ask.pending(engine.session) is None
    engine.session.pop_scene(kind)
    assert tool_ask.pending(engine.session) is None
    _run(engine, "да, удаляй")
    assert library.tracks["t-1"] == "Кот и ёж" and _confirmed(ports) == []


def test_frames_that_are_not_questions_keep_the_window():
    """Сцена диджея или персоны - не вопрос: окно ждёт ответа дальше."""
    engine, _, _ = _asked_to_delete()
    engine.session.push_scene("DJ", "code")
    assert tool_ask.pending(engine.session) is not None
    assert not {"DJ", "Persona", kn.TOOL_ASK_SCENE} & set(kn.TOOL_ASK_CLOSED_BY)


def test_stop_while_the_tool_answers_opens_no_window():
    """Стоп пришёл, пока тул отвечал: вопрос не прозвучит (эпоха сменилась) - и окна под него нет."""
    library = Library()
    ports = Ports(library, [Answer(Command(actions=(ASK_DELETE,)), "deepseek")])
    engine = _engine(ports)
    ports.during_execute = engine.session.replace_epoch
    _run(engine, "удали из библиотеки трек «Кот и ёж»")
    assert ports.said == [] and tool_ask.pending(engine.session) is None


# ---------------------------------------------------------------------------------------------------------------
# другие слова в окне: к LLM с тулом вопроса в срезе (без круга tool_not_available → widen_for)
# ---------------------------------------------------------------------------------------------------------------

def test_other_words_go_to_llm_with_the_asked_tool_in_the_slice():
    engine, ports, library = _asked_to_delete([Answer(Command(speech="Каких двух?"), "deepseek")])
    agg = _run(engine, "Робот, да, удали оба")
    assert agg.llm_calls == 1 and "music_library" in ports.asked[-1]
    assert library.tracks["t-1"] == "Кот и ёж"
    assert tool_ask.pending(engine.session) is not None, "не ответ - окно ждёт ответа дальше"


def test_without_a_question_these_words_do_not_open_the_library():
    """Причина бага 27.10: в «да, удаляй» нет слов группы music - срез без ``music_library``. Открывает его вопрос."""
    snap = Session("personality").snapshot
    assert "music_library" not in [t.name for t in available_tools(snap, text="да, удаляй")]
    assert "music_library" in [t.name for t in available_tools(snap, text="да, удаляй", asked=("music_library",))]


def test_model_confirming_by_itself_closes_the_window():
    """Модель сама позвала тот же вызов с ``confirm`` (ответ не по грамматике) - исход закрывает окно: следующее «да»
    ничего не подтверждает кодом."""
    confirm = ToolCall("music_library", {"action": "delete", "track_id": "t-1", "confirm": True})
    engine, ports, library = _asked_to_delete([Answer(Command(actions=(confirm,)), "deepseek")])
    _run(engine, "удаляй его раз решили")
    assert "t-1" not in library.tracks and ports.said[-1] == (DELETED, NEUTRAL)
    assert tool_ask.pending(engine.session) is None


def test_new_session_drops_the_question():
    engine, ports, library = _asked_to_delete()
    engine.session.reset()
    assert tool_ask.pending(engine.session) is None


# ---------------------------------------------------------------------------------------------------------------
# грамматика ответа и знание в одном месте
# ---------------------------------------------------------------------------------------------------------------

QUESTION_RULE = kn.TOOL_QUESTIONS[("music_library", "needs_confirmation")]


@pytest.mark.parametrize("text, verdict", [
    ("да", True), ("Робот, да, удаляй", True), ("удаляй эту песню", True), ("да, давай, удаляй", True),
    ("да, удаляй трек", True), ("ага", True), ("угу", True), ("подтверждаю", True), ("да, уверен", True),
    ("нет", False), ("не надо", False), ("нет, не удаляй", False), ("отмена", False), ("да, но не эту", False),
    ("не уверен", False),
    ("да, удали все", None), ("удали два", None), ("да, удали оба", None), ("да, но потом", None), ("стой", None),
    ("хватит", None), ("", None), ("робот", None), ("эту песню", None), ("включи музыку", None),
])
def test_answer_grammar_covers_the_whole_phrase(text, verdict):
    assert tool_ask.answer(text, QUESTION_RULE) is verdict


@pytest.mark.parametrize("text", ["ладно", "ну ладно", "да ладно", "ок", "окей", "хорошо", "конечно", "точно",
                                  "давай", "ну давай"])
def test_neutral_agreement_does_not_confirm_an_irreversible_delete(text):
    """Ревью PR #4100: «ладно», «ок», «хорошо», «конечно», «точно», «давай» поодиночке удаляли трек. Это не явное
    «да» на «Удалить?» («ну ладно» - часто «забудь»): не ответ, решает LLM."""
    assert tool_ask.answer(text, QUESTION_RULE) is None


@pytest.mark.parametrize("text", ["ладно", "ок", "хорошо", "давай"])
def test_neutral_agreement_in_the_window_deletes_nothing_by_code(text):
    engine, ports, library = _asked_to_delete([Answer(Command(speech="Так удалить или нет?"), "deepseek")])
    agg = _run(engine, text)
    assert library.tracks["t-1"] == "Кот и ёж" and _confirmed(ports) == [] and agg.llm_calls == 1


def test_answer_words_do_not_swallow_stop_and_do_not_overlap():
    words = set(kn.TOOL_ANSWER_YES) | set(kn.TOOL_ANSWER_NO) | set(kn.TOOL_ANSWER_FILLER)
    assert not words & set(kn.STOP_WORDS), "стоп при открытом вопросе остаётся стопом Tier-1"
    assert not set(kn.TOOL_ANSWER_YES) & set(kn.TOOL_ANSWER_NO)
    assert not set(kn.TOOL_ANSWER_YES) & {"ладно", "ок", "окей", "хорошо", "конечно", "точно", "давай"}


@pytest.mark.parametrize("key", list(kn.TOOL_QUESTIONS))
def test_every_tool_question_speaks_its_own_phrase_and_has_a_decline(key):
    """Вопрос говорится ключом причины (не общим «не вышло»), отказ - своей фразой каталога; кадр окна - сцена
    сессии."""
    tool, reason = key
    result = ActionResult("t1/0", 0, "personality", tool, kn.call_class(tool, ASK_DELETE.args), "rejected", reason,
                          dict(ASK_DELETE.args))
    assert phrase_key(result).endswith(f".{reason}")
    assert kn.TOOL_QUESTIONS[key].declined in phrases.catalog()
    assert kn.TOOL_ASK_SCENE in kn.SCENE_KINDS and kn.TOOL_ASK_SCENE not in kn.ASK_SCENES
    assert 0 < kn.TOOL_ASK_TTL_S
