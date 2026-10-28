"""#4074: шаг темпа и тона речи решает код (Tier-1 ``voice_step``), а не стоп и не LLM.

Марафон 27.10, акт 4: n410 «Робот, ладно, хватит тянуть, теперь наоборот — тараторь.» уходил в ``stop_all`` по
«хватит», остаток без слов темпа шёл в LLM без ``set_speed`` в срезе, и модель врала «Теперь я тараторю»; n411
«Робот, говори басом, как диктор в старом кино.» модель сказала одной фразой другим голосом вместо тона. Решение Шифу
27.10: «басом» - режим того же голоса с тоном ниже, снимают его «говори нормально» и «обычным голосом».
Тесты - на поведение модулей: грамматика, перебивание, ход на фейковых портах, срез тулов.
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.agents import agent_spec, allows_tier1
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.grammar import Tier1Command, match
from rob_box_dialog.interrupt import NO_INTERRUPT, on_interrupt
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import MusicStateRef, SceneFrame, Session, SessionSnapshot
from rob_box_dialog.tools_view import available_tools
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.words import STOP_CLEAR, STOP_DISPUTED, policy_word, stop_verdict


def _snap(**kw):
    return SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0, **kw)


IDLE = _snap()
SILENCED = _snap(scenes=(SceneFrame("Silence", "user"),))
VOICES = [("Russian_ReliableMan", "Надёжный мужчина"), ("Russian_BrightHeroine", "Яркая героиня")]
FASTER, SLOWER = ("set_speed", "faster"), ("set_speed", "slower")
LOWER, HIGHER = ("set_pitch", "lower"), ("set_pitch", "higher")
#: Реплики марафона 27.10 как их сказал сценарий и как их услышал STT (``TRANSCRIPT[...]`` акта 4, run 40478630310).
N410 = ("Робот, ладно, хватит тянуть, теперь наоборот — тараторь.", "ладно хватит тянуть теперь наоборот ораторь")
N411 = ("Робот, говори басом, как диктор в старом кино.", "робот говорит басом как диктор в старом кино")


def _step(*steps, rest=""):
    return Tier1Command(kn.VOICE_STEP_KIND, {"steps": tuple(steps)}, rest=rest)


# ---------------------------------------------------------------------------------------------------------------
# Грамматика: n410/n411 - шаг голоса, стоп остаётся стопом
# ---------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("text", N410)
def test_enough_dragging_now_rattle_on_is_a_faster_step_not_a_stop(text):
    """n410: «хватит тянуть» - манера речи, «наоборот» + слово (STT: «ораторь») - её противоположность."""
    assert match(text, IDLE, VOICES) == _step(FASTER)


@pytest.mark.parametrize("text", N411)
def test_speak_bass_like_an_announcer_is_a_lower_pitch_of_the_same_voice(text):
    """n411: «басом» - тон ниже, а не другой голос (решение Шифу 27.10); «как диктор в старом кино» - образ просьбы."""
    command = match(text, IDLE, VOICES)
    assert command == _step(LOWER)
    assert command.kind != "set_voice"


@pytest.mark.parametrize("text, kind, rest", [
    ("Робот, хватит", "stop_all", ""), ("хватит", "stop_all", ""), ("Робот, всё, хватит", "stop_all", ""),
    ("хватит, замолчи", "silence", ""), ("Робот, хватит, замолчи", "silence", ""),
    ("Робот, хватит говорить", "stop_speech", ""), ("робот хватит читать", "stop_speech", ""),
    ("хватит, говори быстрее", "stop_all", "говори быстрее"),  # стоп и новая просьба - стоп, как было
    ("робот хватит бубнить", "stop_all", "бубнить"),  # «бубнить» - не темп: стоп, как было
])
def test_enough_without_a_speech_manner_is_still_a_stop(text, kind, rest):
    command = match(text, IDLE, VOICES)
    assert (command.kind, command.rest, command.disputed) == (kind, rest, False), text


@pytest.mark.parametrize("text, steps", [
    ("тараторь", (FASTER,)), ("Робот, тараторь!", (FASTER,)), ("говори быстрее", (FASTER,)),
    ("быстрее говори", (FASTER,)), ("говори побыстрее, пожалуйста", (FASTER,)), ("говори помедленнее", (SLOWER,)),
    ("читай медленнее", (SLOWER,)), ("не тараторь", (SLOWER,)), ("хватит тараторить", (SLOWER,)),
    ("не тяни", (FASTER,)), ("хватит мямлить", (FASTER,)), ("не торопись, говори помедленнее", (SLOWER,)),
    ("говори пониже", (LOWER,)), ("говори низким голосом", (LOWER,)), ("говори как диктор", (LOWER,)),
    ("говори выше", (HIGHER,)), ("говори тоненьким голосом", (HIGHER,)),
    ("говори повыше и побыстрее", (FASTER, HIGHER)),
])
def test_pace_and_pitch_requests_are_voice_steps(text, steps):
    assert match(text, IDLE, VOICES) == _step(*steps), text


@pytest.mark.parametrize("text", [
    "быстрее", "басом", "как диктор", "скажи как диктор",  # без речевого глагола - не про речь (под музыку - трек)
    "говори как пират", "говори как обычно",  # образ без шага
    "не говори быстрее", "говори не так быстро",  # отрицание решает LLM
    "говори быстрее и медленнее", "говори выше и ниже",  # два действия одного тула
    "говори быстрее о погоде", "ты обычно говоришь быстро", "ускорь темп, а то мы тут засыпаем",
    "Робот, говори помедленнее я за тобой записываю",  # n409 из STT: лишние слова без границы части - LLM
    "хватит тянет", "хватит так медленно",  # манера - точной формой глагола, а не основой и не наречием
])
def test_not_a_voice_step(text):
    command = match(text, IDLE, VOICES)
    assert command is None or command.kind != kn.VOICE_STEP_KIND, text


@pytest.mark.parametrize("text", ["Робот, ты тараторишь", "ты тараторишь", "тараторишь", "Робот, ты тараторишь!"])
def test_you_rattle_on_is_a_complaint_not_a_request_to_speed_up(text):
    """Ревью #4113: основа «тарато» делала жалобу «ты тараторишь» просьбой ``set_speed faster`` - робот отвечал
    «Говорю быстрее.» человеку, которому и так быстро. Просьба - только повелительная форма («тараторь»)."""
    assert match(text, IDLE, VOICES) is None, text
    names = {t.name for t in available_tools(IDLE, text=text)}
    assert "set_speed" in names, text  # жалобу решает модель, и тул темпа у неё в срезе


@pytest.mark.parametrize("text", ["тараторь", "тараторьте", "Робот, тараторь!"])
def test_imperative_rattle_on_is_still_a_faster_step(text):
    assert match(text, IDLE, VOICES) == _step(FASTER), text


@pytest.mark.parametrize("text", [
    "ладно хватит тянуть теперь наоборот давай ораторь", "ладно хватит тянуть теперь наоборот ты ораторь",
    "ладно хватит тянуть теперь на оборот ораторь", "Робот, ладно, хватит тянуть, теперь наоборот, давай тараторь.",
    "ладно, хватит тянуть, теперь наоборот",
])
def test_n410_variants_from_stt_are_the_same_faster_step(text):
    """Ревью #4113: служебное слово перед пересказом («давай», «ты») и «на оборот» раздельно (STT) уводили n410 в
    спорный стоп с закрытым срезом тулов."""
    assert match(text, IDLE, VOICES) == _step(FASTER), text


@pytest.mark.parametrize("text, rest, follow", [
    ("хватит тянуть, теперь наоборот — замолчи", "замолчи", "silence"),
    ("хватит тянуть, теперь наоборот — стоп", "стоп", "stop_all"),
    ("хватит тянуть, теперь наоборот — остановись", "остановись", "stop_all"),
    ("хватит тянуть, теперь наоборот, погромче", "погромче", "volume"),
])
def test_another_tier1_word_after_on_the_contrary_is_not_swallowed(text, rest, follow):
    """Ревью #4113: слово после «наоборот» глоталось любое - стоп, тишина и громкость пропадали молча. Слово другой
    команды Tier-1 - не пересказ: оно остаётся остатком и исполняется своей грамматикой."""
    command = match(text, IDLE, VOICES)
    assert command == _step(FASTER, rest=rest), text
    assert match(command.rest, IDLE, VOICES).kind == follow


@pytest.mark.parametrize("text", ["хватит так тараторить", "Робот, хватит так тараторить!"])
def test_enough_with_so_before_the_manner_is_a_step(text):
    assert match(text, IDLE, VOICES) == _step(SLOWER), text


@pytest.mark.parametrize("text, command", [
    ("говори нормально", Tier1Command(kn.PROSODY_NORMAL_KIND)),
    ("Робот, всё, верни обычную скорость речи и обычный тон.", Tier1Command(kn.PROSODY_NORMAL_KIND)),
    ("говори обычным голосом", Tier1Command("set_voice", {"voice": kn.VOICE_DEFAULT_ARG})),
    ("говори голосом надёжного мужика", Tier1Command("set_voice", {"voice": "Russian_ReliableMan"})),
])
def test_neighbour_voice_commands_are_untouched(text, command):
    assert match(text, IDLE, VOICES) == command


@pytest.mark.parametrize("text, step, rest", [
    ("говори басом и расскажи сказку", LOWER, "расскажи сказку"),
    ("Робот, говори помедленнее, я за тобой записываю.", SLOWER, "я за тобой записываю"),
    ("хватит тянуть, давай рассказывай дальше", FASTER, "давай рассказывай дальше"),
    ("говори медленнее и тише", SLOWER, "тише"),
])
def test_a_new_request_after_a_clause_boundary_is_the_rest(text, step, rest):
    assert match(text, IDLE, VOICES) == _step(step, rest=rest)


def test_rest_after_a_step_runs_its_own_tier1():
    assert match("тише", IDLE, VOICES).kind == "volume"


def test_voice_step_speaks_so_silence_drops_it_and_only_personality_has_it():
    assert kn.TIER1_KINDS[kn.VOICE_STEP_KIND] is True
    command = match("говори басом", SILENCED, VOICES)  # «говори» в тишине - её снятие
    assert command is None or command.kind != kn.VOICE_STEP_KIND
    assert allows_tier1(agent_spec("personality"), kn.VOICE_STEP_KIND)
    assert not allows_tier1(agent_spec("operator"), kn.VOICE_STEP_KIND)
    assert not allows_tier1(agent_spec("telegram"), kn.VOICE_STEP_KIND)


# ---------------------------------------------------------------------------------------------------------------
# Стоп-слово с манерой речи - не ясный стоп: перебивание и политика планировщика это видят
# ---------------------------------------------------------------------------------------------------------------

def test_enough_dragging_is_not_a_clear_stop_anywhere():
    """Одно решение для грамматики, перебивания речи (``stop_verdict``) и политики планировщика (``policy_word``)."""
    text = "робот ладно хватит тянуть теперь наоборот ораторь"
    assert stop_verdict(text) == STOP_DISPUTED and policy_word(text) is None
    assert on_interrupt(text, IDLE) == NO_INTERRUPT
    assert stop_verdict("робот хватит") == STOP_CLEAR and policy_word("робот хватит") == "stop"


def test_enough_with_a_manner_the_grammar_does_not_cover_is_a_disputed_stop():
    """«хватит тянуть резину» - не шаг (лишнее слово) и не ясный стоп: спорный, решает модель, речь не рвётся, а тул
    темпа у модели в срезе (манера речи открывает группу голоса)."""
    command = match("хватит тянуть резину", IDLE, VOICES)
    assert command.kind == "stop_all" and command.disputed
    assert on_interrupt("Робот, хватит тянуть резину", IDLE) == NO_INTERRUPT
    assert "set_speed" in {t.name for t in available_tools(IDLE, text="хватит тянуть резину")}


@pytest.mark.parametrize("text", [
    "Робот, стой! Спешишь, врежешься", "стой, тороплюсь", "замолчи, тараторка", "стоп тяни", "хватит тянет",
])
def test_other_stop_words_before_a_manner_stem_are_a_clear_stop(text):
    """Ревью #4113: исключение «стоп-слово + основа манеры» делало спорным любой стоп («стой, тороплюсь»). Не ясный
    стоп - только ведущее слово манеры («хватит») перед её точной формой."""
    assert stop_verdict(text) == STOP_CLEAR, text
    command = match(text, IDLE, VOICES)
    assert command.kind in ("stop_all", "silence") and not command.disputed, text


def test_speech_manners_do_not_shadow_the_speech_stop_scope():
    """«хватит говорить/читать/петь» - стоп речи (#3994): ни одна манера темпа не начинает эти глаголы."""
    verbs = kn.STOP_SCOPE.objects["speech"].verbs
    assert not [(m, v) for m in kn.VOICE_STEP.manners for v in verbs if v.startswith(m) or m.startswith(v)]


def test_likeness_steps_are_steps_of_the_request_table():
    assert set(kn.VOICE_STEP.likeness) <= set(kn.VOICE_STEP.requests)
    assert set(kn.VOICE_STEP.manners.values()) <= set(kn.VOICE_STEP.requests)


# ---------------------------------------------------------------------------------------------------------------
# Ход на фейковых портах: тулы исполняет код, фразу строит код по результату
# ---------------------------------------------------------------------------------------------------------------

class Ports:
    """Транспорт хода: ответы LLM по очереди, исходы по тулу ``(статус, причина, данные тула)``."""

    def __init__(self, answers=(), outcomes=None):
        self.answers = list(answers)
        self.outcomes = outcomes or {}
        self.asked, self.executed, self.said, self.cancels = [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append((list(messages), [t.name for t in tools]))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        out = []
        for n, a in enumerate(actions):
            status, reason, data = self.outcomes.get(a.tool, ("done", None, {"status": "done"}))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                                    status, reason, {**dict(a.args), **data}))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append((text, voice))
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        self.cancels.append(reason)

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _engine(ports):
    ids = iter(f"t{n}" for n in range(1, 100))
    return TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids),
                      voices=lambda: VOICES)


def _run(ports, text):
    return asyncio.run(_engine(ports).run(Addressed(text, VOICE)))


def test_n410_from_stt_speeds_up_by_code_without_stop_and_without_llm():
    ports = Ports()
    agg = _run(ports, N410[1])
    assert ports.executed == [[("set_speed", {"action": "faster"})]]
    assert ports.asked == [] and agg.llm_calls == 0
    assert ports.cancels == []  # не стоп: речь не рвётся, эпоха не меняется
    assert ports.said == [(phrases.text("speed.faster"), None)]


def test_n411_from_stt_lowers_the_pitch_and_keeps_the_voice():
    """Тон ниже тулом ``set_pitch``; ``set_voice`` не зовётся, фраза кода - текущим голосом (``voice=None``)."""
    ports = Ports()
    _run(ports, N411[1])
    assert ports.executed == [[("set_pitch", {"action": "lower"})]]
    assert ports.asked == [] and ports.said == [(phrases.text("pitch.lower"), None)]


@pytest.mark.parametrize("tool, text, key", [
    ("set_speed", "тараторь", "speed.faster.at_limit"), ("set_pitch", "говори басом", "pitch.lower.at_limit"),
])
def test_step_at_the_end_of_the_axis_says_so_instead_of_claiming_a_change(tool, text, key):
    ports = Ports(outcomes={tool: ("done", kn.VOICE_STEP_AT_LIMIT, {"status": "done"})})
    _run(ports, text)
    assert ports.said == [(phrases.text(key), None)]
    assert phrases.text(key) != phrases.text(key.rsplit(".", 1)[0])


def test_step_refused_by_the_provider_says_the_refusal():
    ports = Ports(outcomes={"set_pitch": ("rejected", "pitch_unsupported", {"status": "rejected"})})
    _run(ports, "говори басом")
    assert ports.said == [(phrases.text("local.rejected.pitch_unsupported"), None)]


def test_llm_called_speed_tool_gets_the_code_phrase_from_the_result_not_done():
    """Путь LLM (реплику грамматика не покрыла): речь модели при действии не звучит, фраза - по результату тула."""
    claim = Command(speech="Теперь я тараторю!", actions=(ToolCall("set_speed", {"action": "faster"}),))
    ports = Ports([Answer(claim, "deepseek")])
    _run(ports, "ускорь темп, а то мы тут засыпаем")
    assert ports.executed == [[("set_speed", {"action": "faster"})]]
    assert ports.said == [(phrases.text("speed.faster"), None)]


def test_ordinary_voice_lifts_the_bass_mode_and_says_so_only_when_the_pitch_changed():
    """«Обычным голосом» после «басом»: голос говорит сам тул, сброс тона - фраза кода, только если тон менялся."""
    lifted = Ports()
    _run(lifted, "Робот, говори обычным голосом")
    assert lifted.executed == [[("set_voice", {"voice": kn.VOICE_DEFAULT_ARG}), ("set_pitch", {"action": "normal"})]]
    assert lifted.said == [(phrases.text("pitch.normal"), None)]
    already = Ports(outcomes={"set_pitch": ("done", kn.VOICE_STEP_AT_LIMIT, {"status": "done"})})
    _run(already, "Робот, говори обычным голосом")
    assert already.said == []  # тон и так обычный - о голосе сказал тул


def test_named_voice_does_not_touch_the_pitch():
    ports = Ports()
    _run(ports, "Робот, говори голосом надёжного мужика")
    assert ports.executed == [[("set_voice", {"voice": "Russian_ReliableMan"})]] and ports.said == []


def test_bass_and_then_a_tale_runs_the_step_by_code_and_gives_the_rest_to_the_llm():
    ports = Ports([Answer(Command(speech="Жила-была Красная Шапочка."), "deepseek")])
    agg = _run(ports, "Робот, говори басом и расскажи сказку")
    assert ports.executed == [[("set_pitch", {"action": "lower"})]]
    assert len(ports.asked) == 1 and agg.llm_calls == 1
    assert [text for text, _ in ports.said] == [phrases.text("pitch.lower"), "Жила-была Красная Шапочка."]


def test_complaint_you_rattle_on_goes_to_the_llm_with_the_speed_tool_and_the_phrase_comes_from_the_result():
    """«Ты тараторишь» - не Tier-1 (ревью #4113): решает модель, ``set_speed`` у неё в срезе, а фраза об исходе - по
    результату тула, не речь модели."""
    claim = Command(speech="Хорошо, буду медленнее.", actions=(ToolCall("set_speed", {"action": "slower"}),))
    ports = Ports([Answer(claim, "deepseek")])
    _run(ports, "Робот, ты тараторишь")
    assert "set_speed" in ports.asked[0][1]
    assert ports.executed == [[("set_speed", {"action": "slower"})]]
    assert ports.said == [(phrases.text("speed.slower"), None)]


def test_silence_after_on_the_contrary_runs_after_the_step_instead_of_being_swallowed():
    ports = Ports()
    engine = _engine(ports)
    agg = asyncio.run(engine.run(Addressed("Робот, хватит тянуть, теперь наоборот — замолчи", VOICE)))
    assert ports.executed == [[("set_speed", {"action": "faster"})]] and agg.llm_calls == 0
    assert engine.session.snapshot.top("Silence") is not None


# ---------------------------------------------------------------------------------------------------------------
# Срез тулов: слова темпа и тона открывают тулы голоса (если реплика всё же уходит к LLM)
# ---------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("text", [
    "тараторь", "быстрее говори", "медленнее", "говори басом", "говори пониже", "ускорь темп, а то засыпаем",
    "ладно хватит тянуть теперь наоборот тараторь",
])
def test_pace_and_pitch_words_open_the_voice_tools(text):
    names = {t.name for t in available_tools(IDLE, text=text)}
    assert {"set_speed", "set_pitch"} <= names, text


def test_pace_words_are_the_step_table():
    """Одна таблица: слова скорости среза и музыкальные двусмысленные - из ``VOICE_STEP.requests``; глаголы только о
    речи («тараторь») в двусмысленные не входят."""
    speed = {s for (tool, _), stems in kn.VOICE_STEP.requests.items() if tool == "set_speed" for s in stems}
    speech_only = {s for s in speed if s.startswith(kn.VOICE_STEP.speech_only)}
    assert speech_only and speed - speech_only <= set(kn.VOICE_PACE_STEMS) <= set(kn.TOOL_GROUPS["voice"].stems)
    assert not speech_only & set(kn.MUSIC_AMBIGUOUS_STEMS)
    assert set(kn.VOICE_STEP_STEMS) | set(kn.VOICE_STEP.speech_only) <= set(kn.TOOL_GROUPS["voice"].stems)


@pytest.mark.parametrize("text", ["ты тараторишь", "Робот, ты мямлишь"])
def test_speech_only_words_keep_the_voice_tools_under_playing_music(text):
    """Ревью #4113: «тарато» попало в слова скорости, а они под играющей музыкой двусмысленны (темп трека) - жалоба
    на речь теряла тулы голоса. Глагол только о речи голос не теряет."""
    playing = _snap(music=MusicStateRef("playing", title="Still Dre"))
    assert "set_speed" in {t.name for t in available_tools(playing, text=text)}, text
