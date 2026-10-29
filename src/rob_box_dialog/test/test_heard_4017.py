"""#4017 (марафон 23.10, решение Шифу 26.10): шум распознавания речи.

* шаг B — однозначные искажения команд чинит код по закрытой таблице до Tier-1 и до модели (``heard.repair``,
  ``knowledge.HEARD_FIXES``): n712 «Робот, всё, глуши.» → STT «робот всох в глуши» — модель «залипала на бите», стопа не
  было; n511 «Робот, всё, глуши музыку, у меня уши в трубочку.» → «робот все глушу музыку …»; «глуши» — стоп-слово;
* шаг A — строка о шуме STT в промпте формы ответа (``knowledge.STT_NOISE_NOTE``), временная до шага C.
"""

import asyncio
import logging

import pytest

from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed, decide
from rob_box_dialog.backlog import Backlog, Heard
from rob_box_dialog.command import Command, Form
from rob_box_dialog.grammar import match
from rob_box_dialog.heard import HEARD_MARK, Repaired, repair
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_dialog.words import STOP_CLEAR, STOP_DISPUTED, STOP_NONE, stop_verdict

WAKES = list(kn.DEFAULT_WAKE_WORDS)
#: Живые строки STT марафона 23.10 (va_marathon.log) и что было сказано.
N712 = "робот всох в глуши"  # 22:40:09.59, сценарий «Робот, всё, глуши.»
N511 = "робот все глушу музыку у меня уши в трубочку"  # 22:14:39.70, «Робот, всё, глуши музыку, у меня уши в трубочку.»


def _heard(text, source="respeaker"):
    return Heard(text=text, t=100.0, source=source, doa_deg=None, person_ref="", voice_s=0.0, faces=frozenset())


# -- heard.repair: только начало команды, только формы таблицы ---------------------------------------------------


@pytest.mark.parametrize("text, said, fix", [
    ("всох в глуши", "всё, глуши", "«всох в глуши» → «всё, глуши»"),
    ("все глушу музыку у меня уши в трубочку", "всё, глуши музыку у меня уши в трубочку", "«все глушу» → «всё, глуши»"),
    ("Всох, в глуши!", "всё, глуши!", "«Всох, в глуши» → «всё, глуши»"),
    ("всё глушу", "всё, глуши", "«всё глушу» → «всё, глуши»"),
])
def test_live_distortion_at_the_head_is_repaired_and_the_rest_kept(text, said, fix):
    assert repair(text) == Repaired(said, (fix,))


@pytest.mark.parametrize("text", [
    "мы живём в глуши", "расскажи как всох в глуши", "все глушат моторы", "всох", "глушу", "в глуши",
    "все глуши", "всё, глуши", "хватит", "", "   ",
])
def test_no_guessing_outside_the_table_or_away_from_the_head(text):
    assert repair(text) == Repaired(text)


def test_none_is_empty_text():
    assert repair(None) == Repaired("")


@pytest.mark.parametrize("heard, said", list(kn.HEARD_FIXES.items()))
def test_every_fix_turns_a_missed_stop_into_a_clear_stop(heard, said):
    """Строка таблицы нужна, только если она меняет решение: искажённая форма — не ясный стоп, починенная — ясный."""
    distorted = " ".join(heard)
    assert stop_verdict(f"робот {distorted}") != STOP_CLEAR
    assert stop_verdict(f"робот {repair(distorted).text}") == STOP_CLEAR
    assert heard == tuple(w.lower().replace("ё", "е") for w in heard), "ключ таблицы — как слова сверки"


# -- «глуши» — стоп-слово: ясный стоп, спорный, не стоп ------------------------------------------------------------


@pytest.mark.parametrize("text", ["Робот, всё, глуши", "робот глуши", "Робот, глуши музыку"])
def test_mute_is_a_clear_stop_of_all_sounds(text):
    command = match(text, Session("personality").snapshot)
    assert command is not None and command.kind == "stop_all" and not command.disputed


@pytest.mark.parametrize("text, verdict", [
    ("Робот, мы живём в глуши", STOP_DISPUTED),
    ("Робот, не глуши музыку", STOP_DISPUTED),
    ("Робот, где глушитель", STOP_NONE),
])
def test_mute_inside_other_words_is_not_a_clear_stop(text, verdict):
    assert stop_verdict(text) == verdict


def test_music_keep_clause_with_mute_still_keeps_music():
    """Оговорка «а музыку не глуши» (``STOP_SCOPE.keep``) не стала новым стопом."""
    command = match("Робот, хватит читать, а музыку не глуши", Session("personality").snapshot)
    assert command.kind == "stop_speech" and command.args["keep_music"] and not command.rest


# -- адресация: единственная точка починки ----------------------------------------------------------------------


def test_n712_addressed_text_is_repaired_with_a_log_note():
    assert decide(_heard(N712), Backlog(), WAKES) == Addressed(
        "всё, глуши", VOICE, stt_fixes=("«всох в глуши» → «всё, глуши»",))


def test_slipped_wake_is_repaired_too():
    out = decide(_heard("роботу всох в глуши"), Backlog(), WAKES)
    assert out.text == "всё, глуши" and out.stt_fixes


def test_echo_before_the_wake_repairs_only_the_command_after_it():
    """n811-образ: перед вейком эхо речи робота — голова фразы не искажение, команда после вейка — да."""
    out = decide(_heard("четыре недели робот всох в глуши"), Backlog(), WAKES)
    assert out.text == "четыре недели всох в глуши" and out.command == "всё, глуши"
    assert out.stt_fixes == ("«всох в глуши» → «всё, глуши»",)


def test_typed_text_is_not_repaired():
    out = decide(_heard("робот всох в глуши", source="telegram"), Backlog(), WAKES)
    assert out.text == "всох в глуши" and out.stt_fixes == ()


def test_ordinary_phrase_is_addressed_as_is():
    assert decide(_heard("робот мы живём в глуши"), Backlog(), WAKES) == Addressed("мы живём в глуши", VOICE)


# -- ход: стоп кодом, 0 вызовов модели ---------------------------------------------------------------------------


class Ports:
    """Записывающий транспорт хода: ответы модели по очереди, все действия — ``done``."""

    def __init__(self, answers=()):
        self.answers = list(answers)
        self.asked, self.executed, self.cancels, self.said = [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        from rob_box_dialog.respond import ActionResult

        self.executed.append([a.tool for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                             "done", None, {}) for n, a in enumerate(actions)]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        self.cancels.append(reason)

    def stop_perform(self, how, keep_backing=False):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _engine(ports, log):
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", log=log)
    engine.session._snap = engine.session.snapshot.__class__(
        **{**vars(engine.session.snapshot), "music": MusicStateRef("playing")})
    return engine


def test_n712_under_music_is_a_code_stop_with_zero_llm_calls(caplog):
    """n712: под играющей музыкой «робот всох в глуши» — ``stop_all`` Tier-1, ``stop_music``, модель не зовётся (у
    порта нет ответов: вызов модели упал бы)."""
    ports = Ports()
    log = logging.getLogger("test_heard_4017")
    with caplog.at_level(logging.INFO, logger=log.name):
        agg = asyncio.run(_engine(ports, log).run(decide(_heard(N712), Backlog(), WAKES)))
    assert ports.asked == [] and agg.llm_calls == 0
    assert ports.cancels == ["tier1 stop_all"] and ports.executed == [["stop_music"]]
    assert any(HEARD_MARK in r.getMessage() and "«всох в глуши» → «всё, глуши»" in r.getMessage()
               for r in caplog.records)


def test_n511_stop_by_code_then_the_rest_goes_to_the_model():
    """n511: «всё, глуши музыку у меня уши в трубочку» — стоп кодом, музыка гаснет до модели; непокрытый остаток —
    новая реплика того же хода (§6.2), как у любого ясного стопа с продолжением."""
    ports = Ports(answers=[Answer(Command(speech="Тише некуда."), "deepseek")])
    asyncio.run(_engine(ports, logging.getLogger("test_heard_4017")).run(decide(_heard(N511), Backlog(), WAKES)))
    assert ports.cancels == ["tier1 stop_all"] and ports.executed[0] == ["stop_music"]
    assert len(ports.asked) == 1 and ports.asked[0][-1]["content"].endswith("музыку у меня уши в трубочку")


# -- шаг A: строка о шуме STT не зависит от формы хода ----------------------------------------------------------


@pytest.mark.parametrize("schema", [
    None,
    cm.form_schema(kn.command_schema("json"), Form(passed=False, voice=False)),
    kn.with_sing(kn.command_schema("json"), "live:song"),
])
def test_stt_noise_note_opens_the_prompt_fragment_for_every_turn_form(schema):
    assert kn.prompt_fragment(schema).splitlines()[0] == kn.STT_NOISE_NOTE
