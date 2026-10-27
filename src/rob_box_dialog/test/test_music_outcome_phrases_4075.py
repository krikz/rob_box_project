"""#4075: фраза исхода музыкальных тулов строится кодом из РЕЗУЛЬТАТА, а не общим «Включаю музыку.» (ADR-0148).

Живой робот 27.10 (dev-0d94634 с веткой PR #4071, Шифу рядом) и марафон 27.10 (акт 6, n608):

* «хватит диджеить, выключай» → ``dj_set action=stop`` остановил музыку, а робот сказал «Включаю музыку.»;
* «сгенерируй новую песню про енота своим голосом» → ``generate_music`` заказал платную генерацию и поставил мост
  (синт), робот сказал «Включаю музыку.»; Шифу: «надо было сказать: подожди, скоро будет, а пока послушай»;
* посреди рэпа (15:16:58) смена подложки через ``request_music`` прозвучала «Включаю музыку.» поверх выступления.

Ход целиком на фейковых портах: исполнитель отдаёт результат как настоящий (класс ТУЛА, данные = аргументы вызова
поверх ответа тула, ``command_turn.owner_event``).
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Performance, ToolCall
from rob_box_dialog.respond import ActionResult, phrase_from_result
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine, say_late
from rob_box_dialog.turn_log import PERFORM_STARTED, PERFORM_TOOL, TOOL, TurnLog

STARTED = phrases.text("media.done")  # «Включаю музыку.»
SONG = {"kind": "robot_song", "theme": "про енота"}
#: Что вернул бы тул при заказе песни с мостом (``song_order``): мост играет (``track_id``), песня пишется.
SONG_BRIDGED = {"ok": True, "pending": "song", "track_id": "bridge-7", "source": "synth", "style": "club",
                "source_message": "Пишу песню, пою голосом Алёна — минуты полторы. Пока играет своё."}
SONG_BARE = {"ok": True, "pending": "song", "reason": "music_source", "source_message": "Пишу песню."}


class Ports:
    """Транспорт хода: исходы действий по имени тула ``(статус, причина, данные тула)``."""

    def __init__(self, answers=(), outcomes=None):
        self.answers = list(answers)
        self.outcomes = outcomes or {}
        self.said = []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        out = []
        for n, a in enumerate(actions):
            status, reason, data = self.outcomes.get(a.tool, ("done", None, {}))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                                    status, reason, {**dict(a.args), **data}))
        return out

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


def _engine(ports):
    ids = iter(f"t{n}" for n in range(1, 100))
    return TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", new_id=lambda: next(ids))


def _run(engine, text):
    return asyncio.run(engine.run(Addressed(text, VOICE)))


def _said(call, outcome, text="включи", performing=False):
    """Что робот сказал после одного действия; ``performing`` - до хода уже идёт выступление."""
    ports = Ports([Answer(Command(actions=(call,)), "deepseek")], {call.tool: outcome})
    engine = _engine(ports)
    if performing:
        engine.log.begin("t0", 0, "прочитай рэп")
        engine.log.performed("t0", "rap", PERFORM_STARTED)
    _run(engine, text)
    return ports.said


# -- dj_set stop: фраза о стопе, а не «Включаю музыку.» ----------------------------------------------------------------


def test_dj_set_stop_says_the_music_is_off_not_that_it_is_on():
    """n608 (марафон 27.10): ``dj_set {'action': 'stop'}`` → ``[music v2] stop`` → «Включаю музыку.» - ложь об исходе."""
    stop = ToolCall("dj_set", {"action": "stop"})
    assert _said(stop, ("done", None, {"ok": True, "was_playing": True})) == [phrases.text("music.stop_ok")]
    assert _said(stop, ("done", None, {"ok": True})) == [phrases.text("music.stop_ok")], "тул не сообщил - как stop_music"
    assert STARTED not in _said(stop, ("done", None, {"was_playing": True}))


def test_dj_set_stop_with_nothing_playing_says_nothing_is_playing():
    """Музыку уже остановил Tier-1 («хватит»), сет гасит вхолостую (``stop track_id=None``): так и говорим."""
    stop = ToolCall("dj_set", {"action": "stop"})
    assert _said(stop, ("done", None, {"was_playing": False})) == [phrases.text("music.stop_idle")]


def test_dj_set_stop_that_failed_is_not_music_that_failed_to_start():
    stop = ToolCall("dj_set", {"action": "stop"})
    assert _said(stop, ("rejected", "error", {})) == [phrases.text("music.stop_fail")]
    assert phrases.text("music.stop_fail") != phrases.text("media.rejected")


def test_dj_set_start_keeps_the_phrase_of_the_class():
    """Старт сета - «Включаю музыку.» класса, как и был; таблица стопа его не трогает."""
    start = ToolCall("dj_set", {"action": "start", "theme": "космос"})
    assert _said(start, ("done", None, {"ok": True, "theme": "космос"})) == [STARTED]
    assert _said(start, ("rejected", "error", {})) == [phrases.text("media.rejected")]


# -- платная генерация: «подожди, скоро будет, а пока послушай» ---------------------------------------------------------


def _wait_variants(key):
    return set(phrases.catalog()[key])


@pytest.mark.parametrize("tool, args, data, key", [
    ("generate_music", SONG, SONG_BRIDGED, "music.gen_wait.song.bridge"),
    ("generate_music", SONG, SONG_BARE, "music.gen_wait.song"),
    ("generate_music", {"kind": "instrumental", "style": "rock"},
     {"ok": True, "pending": "instrumental", "track_id": "bridge-9", "route": "generate:instrumental:V6"},
     "music.gen_wait.track.bridge"),
    ("generate_music", {"kind": "instrumental"}, {"ok": True, "pending": "instrumental"}, "music.gen_wait.track"),
    ("request_music", {"intent": "track", "text": "сгенерируй песню"}, SONG_BRIDGED, "music.gen_wait.song.bridge"),
])
def test_started_generation_says_wait_a_bit_and_listen_meanwhile_from_the_phrase_table(tool, args, data, key):
    """15:12:39 «Включаю музыку.» после ``generate_music`` с мостом ``synth:club`` → фраза ожидания из таблицы; с мостом
    - «а пока послушай», без моста - только «подожди»."""
    said = _said(ToolCall(tool, args), ("done", None, data), "сгенерируй новую песню про енота своим голосом")
    assert len(said) == 1 and said[0] in _wait_variants(key), said
    assert STARTED not in said
    assert ("послушай" in said[0]) is key.endswith(".bridge"), "про мост - только когда мост играет"
    assert "через пару минут" in said[0] or "это пара минут" in said[0]


def test_wait_phrase_is_picked_by_code_by_turn_and_has_several_variants():
    """3-5 вариантов в каталоге, выбор - кодом по ``turn_id`` (повторяемо), модель фразу не выбирает."""
    for key in kn.GEN_WAIT.all_keys():
        assert 3 <= len(phrases.catalog()[key]) <= 5, key
    texts = set()
    for n in range(12):
        result = ActionResult(f"turn-{n}/0", 0, "personality", "generate_music", "action.media", "done", None,
                              {**SONG, **SONG_BRIDGED})
        again = phrase_from_result(result).text
        assert again == phrase_from_result(result).text, "один ход - один и тот же вариант"
        texts.add(again)
    assert texts == _wait_variants("music.gen_wait.song.bridge")


def test_music_started_without_pending_is_still_the_class_phrase():
    """Обычный запуск (``request_music`` без генерации) - как был; «подожди» говорится только про идущую генерацию."""
    request = ToolCall("request_music", {"intent": "track", "text": "поставь клубный трек"})
    assert _said(request, ("done", None, {"ok": True, "track_id": "t-1"})) == [STARTED]
    assert _said(request, ("done", None, {"ok": True, "pending": None, "track_id": "t-1"})) == [STARTED]
    assert _said(request, ("rejected", "music_source", {"ok": False})) == [phrases.text("media.rejected")]


def test_late_generation_result_says_the_wait_phrase_too():
    """Заказ закрылся по таймауту хода, итог пришёл позже (``say_late``): тот же код, та же фраза ожидания."""
    ports = Ports()
    engine = _engine(ports)
    say_late(engine, ActionResult("t1/0", 0, "personality", "generate_music", "action.media", "done", None,
                                  {**SONG, **SONG_BRIDGED}))
    assert len(ports.said) == 1 and ports.said[0] in _wait_variants("music.gen_wait.song.bridge")


def test_every_key_of_the_music_tool_tables_is_in_the_catalog():
    keys = [k for tool in ("dj_set", "request_music", "generate_music") for entry in kn.TOOL_PHRASES[tool].values()
            for k in entry.all_keys()]
    assert keys and [k for k in keys if k not in phrases.catalog()] == []
    assert set(kn.GEN_WAIT.keys) == {("song", True), ("song", False), ("instrumental", True), ("instrumental", False)}


def test_by_data_present_field_matches_by_having_a_value_not_by_the_value():
    table = kn.ByData(("kind", "id"), {("a", True): "yes", ("a", False): "no"}, present=("id",))
    assert table.key({"kind": "a", "id": "x-1"}) == table.key({"kind": "a", "id": 7}) == "yes"
    assert table.key({"kind": "a"}) == table.key({"kind": "a", "id": ""}) == table.key({"kind": "a", "id": None}) == "no"
    assert table.key({"kind": "b", "id": "x"}) is None


# -- выступление идёт: смена подложки не озвучивается отдельной фразой -------------------------------------------------


def test_music_started_during_a_performance_is_not_announced():
    """15:16:58 посреди рэпа: ``request_music`` сменил подложку - «Включаю музыку.» поверх выступления была шумом."""
    request = ToolCall("request_music", {"intent": "track", "text": "поставь другой бит"})
    assert _said(request, ("done", None, {"ok": True, "track_id": "t-2"}), performing=True) == []
    assert _said(request, ("done", None, {"ok": True, "track_id": "t-2"})) == [STARTED], "выступления нет - как было"


def test_announcement_returns_when_the_performance_ends():
    """Конец выступления (``block_end`` владельца, ``performance_ended``) снимает запрет: следующий запуск музыки слышно."""
    request = ToolCall("request_music", {"intent": "track", "text": "поставь трек"})
    ports = Ports([Answer(Command(actions=(request,)), "deepseek")], {"request_music": ("done", None, {"ok": True})})
    engine = _engine(ports)
    engine.log.begin("t0", 0, "прочитай рэп")
    engine.log.performed("t0", "rap", PERFORM_STARTED)
    assert engine.log.performing()
    engine.log.performance_ended("t0", "done", 3)
    assert not engine.log.performing()
    _run(engine, "поставь трек")
    assert ports.said == [STARTED]


def test_only_the_plain_success_is_quiet_during_a_performance():
    """Провал, ожидание генерации и стоп - данные об исходе, а не шум: говорятся и во время выступления."""
    request = ToolCall("request_music", {"intent": "track"})
    assert _said(request, ("rejected", "error", {}), performing=True) == [phrases.text("media.rejected")]
    assert _said(request, ("timeout", None, {}), performing=True) == [phrases.text("media.timeout")]
    gen = ToolCall("generate_music", SONG)
    assert len(_said(gen, ("done", None, SONG_BRIDGED), performing=True)) == 1
    stop = ToolCall("dj_set", {"action": "stop"})
    assert _said(stop, ("done", None, {"was_playing": True}), performing=True) == [phrases.text("music.stop_ok")]


def test_late_start_during_a_performance_is_quiet_too():
    ports = Ports()
    engine = _engine(ports)
    engine.log.begin("t0", 0, "прочитай рэп")
    engine.log.performed("t0", "rap", PERFORM_STARTED)
    late = ActionResult("t1/0", 0, "personality", "request_music", "action.media", "done", None, {"track_id": "t-2"})
    say_late(engine, late)
    assert ports.said == []
    engine.log.performance_ended("t0", "stopped")
    say_late(engine, late)
    assert ports.said == [STARTED]


def test_quiet_keys_are_the_class_success_phrase_only():
    assert kn.QUIET_IN_PERFORMANCE_KEYS == (kn.ACTION_CLASSES["action.media"].phrases["done"],) == ("media.done",)


# -- правка идущего выступления не оставляет «выступление идёт» после его конца (ревью #4104) ------------------------


class OwnerPorts(Ports):
    """Владелец выступления, как ``PerformHost``: старт - ``pending``; пока блок идёт, и ``edit_pending``, и новое
    выступление поверх (MERGE, «добавь бит») - правка (``done``); блок кончился - правке нечего править (отказ)."""

    def __init__(self, answers=(), outcomes=None):
        super().__init__(answers, outcomes)
        self.running = False

    def perform(self, turn_id, epoch, perf, edit):
        if self.running:
            return {"status": "done"}
        if edit:
            return {"status": "rejected", "reason": "edit_without_active_perform"}
        self.running = True
        return {"status": "pending", "block_id": f"perform-{turn_id}"}


RAP = Command(perform=Performance("rap", ("куплет раз", "куплет два")))
EDITS = {
    "edit_pending": Command(perform=Performance("rap", ("куплет три",)), edit_pending=True),
    "merge_over_running": Command(perform=Performance("rap", ("куплет три",))),
    "add_beat": Command(perform=Performance("rap", (), backing="beat")),
}
REQUEST = ToolCall("request_music", {"intent": "track", "text": "поставь трек"})


def _performs(engine):
    return [e.content["status"] for e in engine.log.window() if e.role == TOOL and e.content["tool"] == PERFORM_TOOL]


def _rap_with_edit(edit, extra_answers=()):
    """Рэп (ход t1), правка (ход t2), смена подложки во время выступления (t3); дальше - ответы ``extra_answers``."""
    answers = [Answer(RAP, "deepseek"), Answer(edit, "deepseek"), Answer(Command(actions=(REQUEST,)), "deepseek"),
               *(Answer(c, "deepseek") for c in extra_answers)]
    ports = OwnerPorts(answers, {"request_music": ("done", None, {"ok": True})})
    engine = _engine(ports)
    _run(engine, "прочитай рэп")
    _run(engine, "добавь куплет")
    assert engine.log.performing() and _performs(engine) == ["started", "started"], "оба хода взял владелец"
    _run(engine, "поставь другой бит")
    assert ports.said == [], "пока выступление идёт, смена подложки молчит"
    return ports, engine


@pytest.mark.parametrize("edit", EDITS.values(), ids=list(EDITS))
def test_end_of_a_performance_that_was_edited_makes_the_next_music_audible(edit):
    """Рэп → «добавь куплет» / «добавь бит» → ``block_end`` с ``turn_id`` ПЕРВОГО хода (блок хранит его, правка id не
    меняет): запись хода-правки тоже закрыта, и следующий «поставь трек» не выполняется молча."""
    ports, engine = _rap_with_edit(edit, extra_answers=(Command(actions=(REQUEST,)),))
    engine.log.performance_ended("t1", "done", 3)
    assert not engine.log.performing()
    assert _performs(engine) == ["done", "done"], "обе записи в окне закрыты исходом владельца"
    _run(engine, "поставь трек")
    assert ports.said == [STARTED]


def test_end_by_stop_closes_the_edit_record_with_the_same_status():
    ports, engine = _rap_with_edit(EDITS["edit_pending"])
    engine.log.performance_ended("t1", "halt", 2)
    assert not engine.log.performing() and _performs(engine) == ["stopped_by_user", "stopped_by_user"]


def test_next_performance_after_an_edited_one_is_tracked_again_and_old_records_stay_closed():
    ports, engine = _rap_with_edit(EDITS["edit_pending"], extra_answers=(RAP, Command(actions=(REQUEST,))))
    engine.log.performance_ended("t1", "done", 3)
    ports.running = False  # блок кончился и у владельца
    _run(engine, "прочитай ещё рэп")
    assert engine.log.performing() and _performs(engine) == ["done", "done", "started"]
    _run(engine, "поставь трек")
    assert ports.said == [], "второе выступление идёт - снова тихо"


def test_performance_end_closes_edit_records_even_when_the_block_turn_left_the_window():
    log = TurnLog(max_turns=2)
    log.begin("t1", 0, "прочитай рэп")
    log.performed("t1", "rap", PERFORM_STARTED)
    log.begin("t2", 0, "добавь куплет")
    log.performed("t2", "rap", PERFORM_STARTED)
    log.begin("t3", 0, "как дела")  # t1 выпал из журнала
    assert log.performing()
    assert log.performance_ended("t1", "done", 3), "закрыта запись правки"
    assert not log.performing()


def test_performance_end_without_any_started_record_changes_nothing():
    log = TurnLog()
    log.begin("t1", 0, "привет")
    assert not log.performance_ended("t9", "done") and not log.performing()
    assert log.window() == (log.window()[0],) and log.window()[0].role == "user"
