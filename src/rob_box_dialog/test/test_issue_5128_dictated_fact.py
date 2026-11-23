"""#5128: диктовку факта сохраняет код, не модель (ADR-0148).

Живой n206 (марафон №3, 22.11, run 45045210796): «робот запомни про меня борис болеет за
спартак…» — ``actual_tool_calls: []``, в ``voice_facts`` пусто, робот сказал «Запомнил».
Что человек продиктовал, код читает из его слов и кладёт обычный вызов ``memory_save``:
дальше тот же путь, что у вызова модели (``bind_person`` → ``VoiceMemory.save_fact``).
"""

import asyncio

from rob_box_dialog import fact_dictation
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine

N206 = "робот запомни про меня борис болеет за спартак и всегда приносит пиццу"
N206_FACT = "борис болеет за спартак и всегда приносит пиццу"
SAVE = ToolCall("memory_save", {"fact": "модель записала другое", "category": "general"})


class Ports:
    """Ответы модели по очереди; исполненные вызовы — доказательство, что запись дошла до тула."""

    def __init__(self, answers):
        self.answers = list(answers)
        self.executed = []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        from rob_box_dialog.respond import ActionResult

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
        return "sp"

    def stage(
        self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="", expression=None
    ):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _run(text, actions):
    ports = Ports([Answer(Command(speech="Запомнил!", actions=actions), "deepseek")])
    asyncio.run(TurnEngine(Session("personality"), ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))
    return ports.executed


def _saved(executed):
    return [args for batch in executed for tool, args in batch if tool == kn.MEMORY_SAVE_TOOL]


# --- детектор: ловит диктовку, слова человека как есть -------------------------------------------


def test_n206_live_text_without_colon_is_the_dictated_fact():
    """Дословный n206: STT без двоеточия, вейк «робот» в начале. Хвост — дословно, не пересказ."""
    assert fact_dictation.dictated_fact(N206) == N206_FACT


def test_colon_and_comma_forms_keep_the_tail_verbatim():
    """Обороты из задачи: двоеточие и запятая снимаются, хвост не пересказывается."""
    assert fact_dictation.dictated_fact("Робот, запомни про меня: Борис болеет за Спартак") == (
        "борис болеет за спартак"
    )
    assert fact_dictation.dictated_fact("запомни, что Саша любит чай") == "саша любит чай"
    assert fact_dictation.dictated_fact("запомни: Борис всегда приносит пиццу") == "борис всегда приносит пиццу"


# --- детектор: не ловит вопрос, представление, ответ и мету -------------------------------------


def test_a_question_about_memory_is_not_a_dictation():
    """«Что ты про меня помнишь?» — вопрос, глагола «запомни» нет."""
    assert fact_dictation.dictated_fact("что ты про меня помнишь?") is None


def test_remember_me_as_is_an_introduction_not_a_fact():
    """«Запомни меня как Саша» — самопредставление (граница ``self_intro``), не факт."""
    assert fact_dictation.dictated_fact("запомни меня как Саша") is None


def test_remember_me_without_a_fact_is_a_registration_not_a_dictation():
    """«Запомни меня» — просьба запомнить человека (#4184), не факт «меня».

    Имени в реплике нет: его называет ``register_speaker`` команды. Хвост «меня»
    фактом не становится — иначе фраза успеха диктовки глушит имя регистрации.
    «Запомни меня навсегда» — уже факт: хвост не одно слово «меня».
    """
    assert fact_dictation.dictated_fact("запомни меня") is None
    assert fact_dictation.dictated_fact("робот, запомни меня") is None
    assert fact_dictation.dictated_fact("запомни меня навсегда") == "меня навсегда"


def test_past_tense_remembered_is_an_answer_not_a_command():
    """«Запомнил, тебя зовут Саша» — прошедшее, ответ робота, не команда человека."""
    assert fact_dictation.dictated_fact("запомнил, тебя зовут Саша") is None
    assert fact_dictation.dictated_fact("запомнил двоих") is None


def test_keep_in_mind_has_nothing_to_store():
    """«Запомни, что я сказал» и «запомни это» — мета, «держи в уме»: факта в словах нет.

    Выбор: сохранять нечего, хвост — не факт. Спорный оборот покрыт явно, а не угадан.
    """
    assert fact_dictation.dictated_fact("запомни, что я сказал") is None
    assert fact_dictation.dictated_fact("запомни это") is None
    assert fact_dictation.dictated_fact("запомни то") is None


def test_remember_the_place_is_a_waypoint_not_a_fact():
    """«Запомни точку: это кухня» — точка навигации (``save_waypoint``), не факт о человеке."""
    assert fact_dictation.dictated_fact("запомни точку: это кухня") is None
    assert fact_dictation.dictated_fact("запомни место кухня") is None


def test_a_dictation_buried_after_other_words_is_not_taken():
    """«Запомни» не первое слово после вейка — не команда диктовки, код не выдёргивает хвост."""
    assert fact_dictation.dictated_fact("слушай запомни про меня борис болеет за спартак") is None


# --- ход: ровно одна запись, и ноль, если тул уже звали -----------------------------------------


def test_n206_turn_saves_the_dictated_words_once_when_the_model_called_nothing():
    """Модель сказала «Запомнил» и тул не вызвала: код добавляет ровно один ``memory_save``."""
    saved = _saved(_run(N206, ()))
    assert saved == [{"fact": N206_FACT, "category": "general"}]


def test_a_memory_save_already_in_the_turn_adds_nothing():
    """Тул уже в команде — ноль дополнительных записей, даже если текст модели другой."""
    saved = _saved(_run(N206, (SAVE,)))
    assert saved == [{"fact": "модель записала другое", "category": "general"}]


def test_a_memory_save_already_acted_this_turn_adds_nothing():
    """Тул исполнен раньше в ходе — код не пишет второй раз."""
    assert fact_dictation.owed(N206, (), (SAVE,)) is None


def test_question_introduction_and_meta_add_no_call():
    """Не-диктовка до тула не доходит."""
    for text in ("что ты про меня помнишь?", "запомни меня как Саша", "запомни, что я сказал", "привет"):
        assert _saved(_run(text, ())) == []


def test_with_dictated_names_the_tool_and_salvage_still_saves():
    """Имя добавленного тула — из общего модуля, не литерал. Повтор моста не помог — salvage всё равно пишет."""
    fresh, done = fact_dictation.with_dictated("запомни: Саша любит чай", Command(speech="Запомнил!"))
    assert done == kn.MEMORY_SAVE_TOOL
    assert [(c.tool, c.args["fact"]) for c in fresh.actions] == [(kn.MEMORY_SAVE_TOOL, "саша любит чай")]
    salvage = fact_dictation.salvage_with_dictated("запомни, что Саша любит чай", ())
    assert [(c.tool, c.args["fact"]) for c in salvage] == [(kn.MEMORY_SAVE_TOOL, "саша любит чай")]


# --- тот же писатель: save_fact пишет, read_voice_facts читает ----------------------------------


def test_the_saved_fact_comes_back_from_the_owner_reader(tmp_path):
    """Запись идёт ``VoiceMemory.save_fact`` — тем же путём, что ``MemorySaveTool.execute``.

    ``read_voice_facts`` читает ту же таблицу и возвращает продиктованные слова. Второй
    писатель не заводится: тест зовёт владельца напрямую, как зовёт его тул.
    """
    from rob_box_dialog_host.speaker_memory import read_voice_facts
    from rob_box_dialog_host.voice_memory import VoiceMemory

    db = str(tmp_path / "facts.db")
    memory = VoiceMemory(db_path=db, migrations=())
    call = fact_dictation.save_call(fact_dictation.dictated_fact(N206))
    fact_id = memory.save_fact(call.args["fact"], category=call.args["category"], speaker_id="person-7be0")
    memory.close()

    assert fact_id
    assert read_voice_facts(db, "person-7be0", limit=20) == [N206_FACT]
    assert read_voice_facts(db, "другой-человек", limit=20) == []
