"""#4049, #4050 и хвосты ревью #4053 (живая проверка 27.10 00:44 UTC, dev-aa90837): решает код (ADR-0148).

* #4049: ``play_animation`` в v2 был пустой операцией для лица — тул лицо не трогает (ADR-0150 PR-3b), а ход не
  красил итог. «Робот, покажи молча, как тебе грустно» → ``play_animation{sad, duration 6, face}`` → на лице рефлекс
  ``happy``; «включи сирену» → ``play_animation{police_lights}`` → мигалки нет. Теперь сцена вызова, её сцена лица и
  время красят итог ``done`` (``turn.command_face`` → ``turn.Shown``);
* #4050: «что у тебя в музыкальной библиотеке?» → ``invalid:several_commands`` → повтор →
  ``invalid:speech_with_actions`` → ``music_library list`` исполнен → ход ``done`` молча. Спасённый ``query`` возвращает данные модели (итерации
  есть) или код говорит по данным (итераций нет);
* ревью #4053: отказ ЧТЕНИЯ памяти спорным голосом — своя фраза кода; ``passed`` рядом с ответом на адресованной
  реплике — лишнее поле, ответ не теряется;
* ревью PR #4055: фраза кода об отказе чтения — только на вопрос о памяти о самом человеке, на остальные реплики отказ
  — данные модели, и она отвечает («кто такой Гагарин?» неузнанным голосом); спасённый ``query`` с отказом — фраза
  отказа, а не «нашёл ответ».
"""

import asyncio
import json

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases, respond
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression, Form, Invalid, ToolCall, parse, validate
from rob_box_dialog.expression import held_face, hold_ms, merge, reflex
from rob_box_dialog.face_scene import FaceScene
from rob_box_dialog.memory import refused_read
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session, SpeakerRef
from rob_box_dialog.turn import SAD_FACE, Answer, Shown, TurnEngine, command_face, query_fallback, shown_face
from rob_box_dialog.words import asks_own_memory

#: Живой лог 27.10 00:44:37: «Робот, покажи молча, как тебе грустно» → вызов модели.
N_SAD = "покажи молча как тебе грустно"
SAD_CALL = ToolCall("play_animation", {"animation": "sad", "duration": 6,
                                       "face": {"eyes": "half", "gaze": "down", "mouth": "frown", "tint": "blue"}})
#: 00:45:45 «Робот, включи сирену» → ``play_animation{police_lights}`` без длительности.
N_SIREN = "включи сирену"
SIREN_CALL = ToolCall("play_animation", {"animation": "police_lights"})
#: 00:44:50 «Робот, что у тебя в музыкальной библиотеке?» → спасённый ``music_library list``.
N_LIBRARY = "что у тебя в музыкальной библиотеке"
LIST_CALL = ToolCall("music_library", {"action": "list"})
TRACKS = {"tracks": [{"title": "Ночной дозор"}, {"title": "Рассвет"}], "count": 2}
ALL_TOOLS = ("play_animation", "music_library", "memory_search", "memory_context", "memory_save", "play_sound")


class Ports:
    """Транспорт хода: ответы LLM по очереди; исход действия — как у ``owner_event``: аргументы вызова, данные тула
    поверх них (у ``play_animation`` тул отдаёт проверенные ``animation``/``duration``/``face``)."""

    def __init__(self, answers=(), outcomes=None):
        self.answers, self.outcomes = list(answers), outcomes or {}
        self.asked, self.executed, self.said, self.stages, self.faces = [], [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append((list(messages), [t.name for t in tools]))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        out = []
        for n, a in enumerate(actions):
            status, data, reason = self.outcomes.get(a.tool, ("done", {}, None))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool,
                                    kn.TOOL_CLASSES[a.tool], status, reason, {**dict(a.args), **data}))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append((text, voice, emotion))
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="",
              expression=None):
        self.stages.append((stage, status, reason))
        if expression is not None:
            self.faces.append((stage, expression))

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _run(ports, text, session=None):
    engine = TurnEngine(session or Session("personality"), ports, system_prompt="Ты робот.")
    return asyncio.run(engine.run(Addressed(text, VOICE)))


def _done_plan(expression):
    """Что уйдёт лицу на итоге ``done`` (то же, что ``dialog_v2_ports.stage_plan``): рефлекс, окрашенный экспрессией,
    со временем сцены, которое решил ход (``expression.duration_s``, #4083)."""
    seconds = expression.duration_s
    return merge(reflex("done"), "done", expression.emotion, expression.gesture, held_face(expression.face, seconds),
                 hold_ms(seconds))


# -- #4049: исполненный play_animation красит итог своей сценой --------------------------------------------------


def test_4049_play_animation_sad_is_the_done_scene_with_its_face_and_duration_not_the_happy_reflex():
    """00:44:39: ``play_animation{sad, 6 с, face}`` → итог ``done`` с ``sad``, сценой лица LLM и 6 с; не ``happy``."""
    ports = Ports([Answer(Command(actions=(SAD_CALL,)), "deepseek")])
    agg = _run(ports, N_SAD)
    assert (agg.stage, agg.status) == ("done", "finished") and ports.said == [], "лицо по прямой просьбе — молча"
    face = FaceScene(eyes="half", gaze="down", mouth="frown", tint="blue", duration_s=6.0)
    assert ports.faces == [("done", Shown(emotion="sad", face=face, duration_s=6.0))]
    plan = _done_plan(ports.faces[0][1])
    assert (plan.emotion, plan.ttl_ms, plan.face) == ("sad", 6000, face.to_dict())
    assert plan.emotion != reflex("done").emotion, "рефлекс done (happy) сцену вызова не перекрывает"
    assert plan.earcon == reflex("done").earcon, "earcon итога — рефлекса"


@pytest.mark.parametrize("scene", ["police_lights", "fire_truck", "ambulance"])
def test_4049_siren_animations_reach_the_face_without_duration_by_the_show_default(scene):
    """00:45:45 «включи сирену»: мигалка — сцена ``face_node`` (``SCENES``, сверка ``KNOWN_ANIMATIONS`` —
    ``tests/unit/docker/test_animation_assets.py``); без ``duration`` время — по умолчанию показа
    (``FACE_SHOW_DEFAULT_S``, #4083: то же число, что по умолчанию у лица), а не 3 с рефлекса ``done``."""
    cmd = Command(speech="Включаю!", actions=(ToolCall("play_animation", {"animation": scene}),))
    ports = Ports([Answer(cmd, "deepseek")])
    _run(ports, N_SIREN)
    shown = Shown(emotion=scene, duration_s=kn.FACE_SHOW_DEFAULT_S)
    assert ports.said == [("Включаю!", None, "neutral")] and ports.faces == [("done", shown)]
    plan = _done_plan(ports.faces[0][1])
    assert (plan.emotion, plan.ttl_ms, plan.gesture) == (scene, int(kn.FACE_SHOW_DEFAULT_S * 1000), "")


def test_4049_the_called_scene_beats_the_command_emotion_and_gesture():
    """Сцену показывает вызов: эмоция и жест команды её не перекрывают (жест ``face_node`` показал бы поверх)."""
    cmd = Command(speech="Сирена — включаю!", actions=(SIREN_CALL,),
                  expression=Expression(emotion="happy", gesture="wink"))
    ports = Ports([Answer(cmd, "deepseek")])
    _run(ports, N_SIREN)
    assert ports.said == [("Сирена — включаю!", None, "happy")], "play_animation тихий: речь рядом с ним звучит"
    assert ports.faces == [("done", Shown(emotion="police_lights", duration_s=kn.FACE_SHOW_DEFAULT_S))]


def test_4049_unknown_animation_is_an_honest_refusal_and_no_scene():
    """Анимации нет — тул отказал (``Неизвестная анимация``): код говорит отказ грустно, итог не красится вызовом —
    лицо итога по грустной фразе, а не рефлекс ``happy`` (#4069 п.8)."""
    ports = Ports([Answer(Command(actions=(ToolCall("play_animation", {"animation": "disco"}),)), "deepseek")],
                  outcomes={"play_animation": ("rejected", {}, None)})
    _run(ports, "покажи анимацию диско")
    assert ports.said == [(phrases.text("local.rejected"), None, "sad")] and ports.faces == [("done", SAD_FACE)]


def test_4049_shown_face_rules():
    sad = {"animation": "sad", "duration": 6, "face": {"eyes": "half", "duration_s": 2}}
    assert shown_face(None, sad) == Shown(emotion="sad", face=FaceScene(eyes="half", duration_s=6.0), duration_s=6.0)
    only_face = shown_face(Expression(emotion="happy", gesture="wink"), {"face": {"tint": "warm"}})
    held = kn.FACE_SHOW_DEFAULT_S  # сцена лица без своего времени (#4089): умолчание показа, одно на всех
    assert only_face == Expression(emotion="happy", gesture="wink", face=FaceScene(tint="warm", duration_s=held),
                                   duration_s=held)
    assert shown_face(Expression(emotion="sad"), {"face": {"eyes": "no-such"}}) == Expression(emotion="sad")
    assert shown_face(None, {"animation": "sad", "duration": True}).duration_s == kn.FACE_SHOW_DEFAULT_S, \
        "bool — не длительность: время по умолчанию показа"
    calls = [ActionResult(f"t/{n}", 0, "personality", "play_animation", "action.local", "done", None, {"animation": a})
             for n, a in enumerate(("sad", "victory"))]
    default = kn.FACE_SHOW_DEFAULT_S
    assert command_face(None, calls) == Shown(emotion="sad", duration_s=default,
                                              then=(Shown(emotion="victory", duration_s=default),)), \
        "#4083: пачка — по очереди, первая на итоге, а не «последний вызов главнее»"
    failed = [*calls, ActionResult("t/2", 0, "personality", "stop_music", "action.media", "rejected")]
    assert command_face(Expression(emotion="happy"), failed) is None, "«радостно» о провале не бывает"


def test_4049_merge_holds_the_called_scene_only_with_a_scene():
    plan = merge(reflex("done"), "done", "police_lights", None, None, 0)
    assert (plan.emotion, plan.ttl_ms) == ("police_lights", 0)
    calm = merge(reflex("done"), "done", "neutral", None, None, 9000)
    assert calm == reflex("done"), "без сцены вызова время рефлекса не меняется"
    assert merge(reflex("failed"), "failed", "sad", None, None, 6000) == reflex("failed"), "провал побеждает"


# -- #4050: спасённый query невалидной команды не молчит ---------------------------------------------------------


def _invalid_list(reason="speech_with_actions"):
    return Answer(Invalid(reason, salvage=(LIST_CALL,)), "deepseek", calls=2)


def test_4050_salvaged_query_returns_the_data_to_the_model_and_the_answer_is_spoken():
    """00:44:54: ``invalid:speech_with_actions`` + ``music_library list`` → данные модели, она отвечает словами."""
    answer = Command(speech="В библиотеке два трека: «Ночной дозор» и «Рассвет».")
    ports = Ports([_invalid_list(), Answer(answer, "deepseek")], outcomes={"music_library": ("done", TRACKS, None)})
    agg = _run(ports, N_LIBRARY)
    assert ports.executed == [[("music_library", {"action": "list"})]], "list исполнен один раз"
    assert [t for t, _, _ in ports.said] == [answer.speech] and agg.llm_calls == 3
    data = json.loads(ports.asked[1][0][-1]["content"])[kn.QUERY_RESULTS_FIELD]
    assert data == [{"tool": "music_library", "status": "done", "reason": None,
                     "data": {"action": "list", **TRACKS}}], "модель видит данные спасённого запроса"
    assert ports.stages[-1] == ("done", "finished", "")


def test_4050_salvaged_query_without_iterations_left_is_said_by_code_from_the_data(monkeypatch):
    """Бюджета итераций нет — читать данные некому: код говорит по ним (``query_fallback``), а не молчит."""
    monkeypatch.setattr(kn, "LLM_QUERY_ITERATIONS_MAX", 0)
    ports = Ports([_invalid_list()], outcomes={"music_library": ("done", TRACKS, None)})
    _run(ports, N_LIBRARY)
    assert len(ports.asked) == 1 and ports.said == [(phrases.text(kn.QUERY_FALLBACK_GENERIC), None, None)]
    assert ports.stages[-1] == ("done", "done", "invalid_salvaged:speech_with_actions")


def test_4050_model_invalid_again_after_the_salvaged_data_still_speaks():
    """И после данных модель ответила невалидно — код говорит по данным запроса (#3883), не молчит."""
    ports = Ports([_invalid_list(), _invalid_list(kn.VIOLATION_SPEECH_TOO_LONG)], outcomes={"music_library": ("done", TRACKS, None)})
    _run(ports, N_LIBRARY)
    assert [t for t, _, _ in ports.said] == [phrases.text(kn.QUERY_FALLBACK_GENERIC)]
    assert ports.executed == [[("music_library", {"action": "list"})]], "повторный list не исполняется"


def test_4050_salvaged_action_without_query_closes_the_turn_as_before():
    """Без ``query`` в спасённых — прежний путь #3953: исполнить, итог говорит код, второй вызов LLM не нужен."""
    ports = Ports([Answer(Invalid("speech_with_actions", salvage=(ToolCall("play_sound", {"sound": "ui_bell"}),)),
                          "deepseek", calls=2)])
    _run(ports, "позвони в колокольчик")
    assert len(ports.asked) == 1 and ports.stages[-1] == ("done", "done", "invalid_salvaged:speech_with_actions")


# -- ревью #4053: отказ чтения памяти — своя фраза кода -----------------------------------------------------------


def _contested_session():
    session = Session("personality")
    session.set_speaker(SpeakerRef(person_ref=None, name_state="contested"))
    return session


@pytest.mark.parametrize("tool", ["memory_search", "memory_context"])
def test_4053_refused_memory_read_has_its_own_phrase_not_the_write_one(tool):
    res = ActionResult("t/0", 0, "personality", tool, "query", "rejected", kn.VOICE_UNCONFIRMED_REASON)
    said = respond.phrase_from_result(res).text
    assert said == "Голос знакомый, но не уверен чей — не буду гадать, что помню."
    assert "запоминать" not in said and refused_read(res)
    write = ActionResult("t/1", 0, "personality", "memory_save", "action.local", "rejected",
                         kn.VOICE_UNCONFIRMED_REASON)
    assert respond.phrase_from_result(write).text == "Голос знакомый, но не уверен чей — запоминать не буду."
    assert not refused_read(write), "запись — фраза исхода действия, как и была"


def test_4053_what_do_you_remember_about_me_in_a_contested_voice_is_said_by_code():
    """«Что ты обо мне помнишь?» спорным голосом: чтение отклонено кодом до владельца — читать модели нечего, код
    говорит свою фразу (одну на причину), второго вызова LLM нет."""
    reads = (ToolCall("memory_context", {}), ToolCall("memory_search", {"query": "я"}))
    rejected = ("rejected", {}, kn.VOICE_UNCONFIRMED_REASON)
    ports = Ports([Answer(Command(actions=reads), "deepseek")],
                  outcomes={"memory_context": rejected, "memory_search": rejected})
    _run(ports, "что ты обо мне помнишь", _contested_session())
    assert ports.said == [("Голос знакомый, но не уверен чей — не буду гадать, что помню.", None, "sad")]
    assert len(ports.asked) == 1


def test_4053_refused_read_next_to_a_readable_query_says_the_refusal_and_returns_the_rest():
    reads = (ToolCall("memory_search", {"query": "чай"}), LIST_CALL)
    ports = Ports([Answer(Command(actions=reads), "deepseek"), Answer(Command(speech="Два трека."), "deepseek")],
                  outcomes={"memory_search": ("rejected", {}, kn.NO_PERSON_REASON),
                            "music_library": ("done", TRACKS, None)})
    _run(ports, "что ты обо мне помнишь и что в библиотеке")
    assert [t for t, _, _ in ports.said] == [phrases.text("query.rejected.no_person"), "Два трека."]
    rows = json.loads(ports.asked[1][0][-1]["content"])[kn.QUERY_RESULTS_FIELD]
    assert [r["tool"] for r in rows] == ["music_library"], "отказ чтения модели не уходит"


# -- ревью PR #4055: фраза кода об отказе чтения — только на вопрос о памяти о самом человеке ---------------------

#: Проба ревьюера: неузнанный голос спрашивает о постороннем, модель перед ответом зовёт ``memory_search``.
N_GAGARIN = "кто такой Гагарин"
GAGARIN_SEARCH = ToolCall("memory_search", {"query": "Гагарин"})
GAGARIN_ANSWER = "Гагарин — первый человек в космосе, полетел в 1961 году."
NO_PERSON = ("rejected", {}, kn.NO_PERSON_REASON)
UNCONFIRMED = ("rejected", {}, kn.VOICE_UNCONFIRMED_REASON)
OWN_PHRASES = {phrases.text(f"query.rejected.{reason}") for reason in kn.UNBOUND_REASONS}


def test_pr4055_unknown_voice_memory_search_on_a_foreign_question_goes_to_the_model_and_it_answers():
    """Регрессия PR #4055 (блокер): отказ чтения на вопрос НЕ о памяти о себе — данные модели, как до #4053; ход идёт
    дальше, модель отвечает про Гагарина. Раньше ход закрывался фразой «Не узнаю твой голос…» после одного вызова
    LLM."""
    ports = Ports([Answer(Command(actions=(GAGARIN_SEARCH,)), "deepseek"),
                   Answer(Command(speech=GAGARIN_ANSWER), "deepseek")], outcomes={"memory_search": NO_PERSON})
    agg = _run(ports, N_GAGARIN)
    assert agg.llm_calls >= 2 and len(ports.asked) == 2
    assert [t for t, _, _ in ports.said] == [GAGARIN_ANSWER], "звучит ответ модели, фразы кода об отказе нет"
    rows = json.loads(ports.asked[1][0][-1]["content"])[kn.QUERY_RESULTS_FIELD]
    assert rows == [{"tool": "memory_search", "status": "rejected", "reason": kn.NO_PERSON_REASON,
                     "data": {"query": "Гагарин"}}], "модель получила отказ данными"
    assert (agg.stage, agg.status) == ("done", "finished")


def test_pr4055_contested_voice_memory_read_on_a_foreign_question_also_goes_to_the_model():
    ports = Ports([Answer(Command(actions=(GAGARIN_SEARCH,)), "deepseek"),
                   Answer(Command(speech=GAGARIN_ANSWER), "deepseek")], outcomes={"memory_search": UNCONFIRMED})
    agg = _run(ports, N_GAGARIN, _contested_session())
    assert agg.llm_calls == 2 and [t for t, _, _ in ports.said] == [GAGARIN_ANSWER]
    assert not OWN_PHRASES & {t for t, _, _ in ports.said}


@pytest.mark.parametrize(("session", "outcome", "reason"), [
    (_contested_session, UNCONFIRMED, kn.VOICE_UNCONFIRMED_REASON),
    (lambda: Session("personality"), NO_PERSON, kn.NO_PERSON_REASON),
], ids=["contested_voice", "unknown_voice"])
def test_pr4055_what_do_you_remember_about_me_is_the_code_phrase_of_its_reason(session, outcome, reason):
    """«Что ты обо мне помнишь?» — ответ и есть отказ: фраза кода своей причины, второго вызова LLM нет."""
    ports = Ports([Answer(Command(actions=(ToolCall("memory_context", {}),)), "deepseek")],
                  outcomes={"memory_context": outcome})
    agg = _run(ports, "Робот, что ты обо мне помнишь?", session())
    assert ports.said == [(phrases.text(f"query.rejected.{reason}"), None, "sad")]
    assert len(ports.asked) == 1 and agg.llm_calls == 1


@pytest.mark.parametrize("text", [
    "что ты обо мне помнишь?", "Робот, а про меня что помнишь?", "как меня зовут и что ты про меня уже знаешь?",
    "ты ещё здесь, помнишь меня?", "помнишь, что я люблю зелёный чай?", "поищи в памяти, что я не ем",
    "а меня ты по голосу узнаёшь? Скажи, кто я такой и откуда ты меня знаешь.", "Робот, как меня зовут?",
    "а скажи-ка, ты же меня по голосу запомнил, как меня зовут?",
])
def test_pr4055_asks_own_memory_yes(text):
    assert asks_own_memory(text)


@pytest.mark.parametrize("text", [
    N_GAGARIN, "ты знаешь, кто такой Гагарин? расскажи мне", "знаешь, я вчера всю ночь паял блок питания",
    "Робот, а теперь найди мне рецепт борща, только учти то, что ты про меня знаешь.",
    "подведи итог ночи: кто к нам приходил, что мы слушали и что ты про нас запомнил?",
    "Меня зовут Саша, я твой ночной инженер", "спой песню про меня", "запомни: я пью только зелёный чай",
    "скажи, кто сейчас говорил на фоне: кого ты узнал по голосу, а кого нет?", "",
])
def test_pr4055_asks_own_memory_no(text):
    """Посторонний вопрос, память как условие другой просьбы (n908), самопредставление, запись — не вопрос о памяти."""
    assert not asks_own_memory(text)


def test_pr4055_borscht_with_memory_as_a_condition_keeps_the_turn_going_to_the_web_search():
    """Марафон n908 неузнанным голосом: «учти то, что ты про меня знаешь» — память условие, а не вопрос: отказ
    чтения — данные модели, она ищет рецепт; фразой об отказе ход не закрывается."""
    web = Answer(Command(actions=(ToolCall("search_web", {"query": "рецепт борща"}),)), "deepseek")
    ports = Ports([Answer(Command(actions=(ToolCall("memory_search", {"query": "еда"}),)), "deepseek"), web,
                   Answer(Command(speech="Борщ: свёкла, капуста, картошка."), "deepseek")],
                  outcomes={"memory_search": NO_PERSON, "search_web": ("done", {"results": ["борщ"]}, None)})
    _run(ports, "Робот, а теперь найди мне рецепт борща, только учти то, что ты про меня знаешь.")
    assert [a[0][0] for a in ports.executed] == ["memory_search", "search_web"]
    assert [t for t, _, _ in ports.said] == ["Борщ: свёкла, капуста, картошка."]


def test_pr4055_refusal_next_to_an_action_is_said_on_a_question_about_own_memory():
    """Смешанная команда ``[memory_search (отказ), play_sound]``: на вопрос о памяти о себе отказ говорит код одной
    репликой со звукоподражанием звонка, читать модели больше нечего — второго вызова нет; на посторонний вопрос отказ —
    данные модели (#4065: смешанная команда тоже возвращает ``query``), звонок сказан кодом, модель отвечает."""
    mixed = Command(actions=(ToolCall("memory_search", {"query": "чай"}), ToolCall("play_sound", {"sound": "ui_bell"})))
    ports = Ports([Answer(mixed, "deepseek")], outcomes={"memory_search": NO_PERSON})
    _run(ports, "помнишь, что я пью? и позвони в колокольчик")
    assert len(ports.said) == 1 and ports.said[0][2] == "sad" and len(ports.asked) == 1
    said = ports.said[0][0]
    assert said.startswith(phrases.text("query.rejected.no_person")), "отказ первым, затем звукоподражание звонка"
    assert ports.executed == [[("memory_search", {"query": "чай"}), ("play_sound", {"sound": "ui_bell"})]]
    foreign = Ports([Answer(mixed, "deepseek"), Answer(Command(speech=GAGARIN_ANSWER), "deepseek")],
                    outcomes={"memory_search": NO_PERSON})
    _run(foreign, "кто такой Гагарин? и позвони в колокольчик")
    assert not any(own in text for own in OWN_PHRASES for text, _, _ in foreign.said)
    assert foreign.said[-1][0] == GAGARIN_ANSWER and len(foreign.asked) == 2
    rows = json.loads(foreign.asked[1][0][-1]["content"])[kn.QUERY_RESULTS_FIELD]
    assert [r["tool"] for r in rows] == ["memory_search"], "модели — только данные query, не исход звонка"


# -- ревью PR #4055: спасённый query с отказом — не «нашёл ответ» ------------------------------------------------


@pytest.mark.parametrize(("status", "reason", "key"), [
    ("rejected", None, "query.rejected"), ("rejected", "bad_event", "query.rejected"),
    ("timeout", None, "query.timeout"),
])
def test_pr4055_salvaged_query_refused_on_the_last_iteration_says_its_outcome(monkeypatch, status, reason, key):
    """Спасённый ``music_library list`` отклонён (или не дождался ответа) на последней итерации: код говорит его
    исход — «Не нашёл ответа.», «Не дождался ответа…», грустно, — а не «Нашёл ответ, но не смог сложить…»."""
    monkeypatch.setattr(kn, "LLM_QUERY_ITERATIONS_MAX", 0)
    ports = Ports([_invalid_list()], outcomes={"music_library": (status, {}, reason)})
    _run(ports, N_LIBRARY)
    assert ports.said == [(phrases.text(key), None, "sad")]
    assert phrases.text(kn.QUERY_FALLBACK_GENERIC) not in {t for t, _, _ in ports.said}


def test_pr4055_query_fallback_says_found_only_when_something_was_found():
    done = ActionResult("t/0", 0, "personality", "music_library", "query", "done", None, {"action": "list", **TRACKS})
    refused = ActionResult("t/1", 0, "personality", "music_library", "query", "rejected", None, {"action": "list"})
    assert query_fallback([refused]) == phrases.text("query.rejected")
    assert query_fallback([refused, done]) == phrases.text(kn.QUERY_FALLBACK_GENERIC), "что-то найдено — «нашёл»"


# -- ревью #4053: passed рядом с ответом на адресованной реплике --------------------------------------------------

ADDRESSED = Form(passed=False, voice=False)


def test_4053_passed_with_speech_on_an_addressed_turn_is_a_dropped_field_not_a_lost_answer():
    res = validate(parse({"passed": "nothing_to_say", "speech": "Окей, молчу."}), ALL_TOOLS, form=ADDRESSED)
    assert isinstance(res, Command) and (res.passed, res.speech) == (None, "Окей, молчу.")
    assert "passed" in res.dropped
    bell = [{"tool": "play_sound", "args": {"sound": "ui_bell"}}]
    acts = validate(parse({"passed": "not_addressed", "actions": bell}), ALL_TOOLS, form=ADDRESSED)
    assert isinstance(acts, Command) and acts.passed is None and [a.tool for a in acts.actions] == ["play_sound"]


def test_4053_bare_passed_on_an_addressed_turn_is_still_the_violation_and_pickup_keeps_its_rule():
    assert validate(parse({"passed": "nothing_to_say"}), ALL_TOOLS, form=ADDRESSED).reason == \
        kn.VIOLATION_PASSED_ADDRESSED
    mixed = validate(parse({"passed": "not_addressed", "speech": "x"}), ALL_TOOLS, form=Form(passed=True, voice=False))
    assert isinstance(mixed, Invalid) and mixed.reason == "passed_with_content", "подхват бэклога — правило прежнее"


def test_4053_passed_with_speech_reaches_the_turn_as_speech_not_the_mumble():
    """Мост валидирует в форме хода: команда ``passed`` + речь доходит до хода речью — она звучит, мычания нет."""
    cmd = validate(parse({"passed": "nothing_to_say", "speech": "Слышу, слышу."}), ALL_TOOLS, form=ADDRESSED)
    ports = Ports([Answer(cmd, "deepseek")])
    agg = _run(ports, "ты тут")
    assert [t for t, _, _ in ports.said] == ["Слышу, слышу."] and (agg.stage, agg.status) == ("done", "finished")
    assert not any(t in phrases.catalog()[kn.PASSED_ADDRESSED_PHRASE] for t, _, _ in ports.said)
