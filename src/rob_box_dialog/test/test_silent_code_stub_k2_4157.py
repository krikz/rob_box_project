"""#4157, класс К2 «заглушка кода вместо ответа» (эпик #4236, E-11): модель без слов - код отвечает по данным, а не
общей фразой «Данные получил, а ответ у меня не сложился» (решение Шифу 28.10: «главное что ответ не был шаблоном»).

Остаток после #4190 (память, звуки, сеть) и #4224 (повтор стопа первым ответом):

* М7 n608 (run 40832761532): Tier-1 стоп сета исполнен молча, модель прочла ``get_music_state``, затем повторила
  ``dj_set stop`` - ход закрылся общей заглушкой. Теперь сначала звучит несказанный исход стопа (``turn.data_close``), а
  у ``get_music_state`` есть слова - фраза, которую построил сам тул (``knowledge.TOOL_MESSAGE_FIELD``).
* n707 (run 40822008136): три поиска в библиотеке, итерации кончились - заглушка. Теперь у ``music_library``
  ``search``/``info`` свои фразы по данным, и ответ - по ПОСЛЕДНЕМУ результату хода.
* Любой ``query`` без своей фразы: фраза тула, иначе названия из данных; общая фраза - только когда в данных нет слов.
"""

import asyncio

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.turn import Answer, TurnEngine, query_answer, query_fallback

GENERIC = phrases.catalog()[kn.QUERY_FALLBACK_GENERIC]
STOP_OK = phrases.text("music.stop_ok")
N608 = "робот хватит диджей выключай"
#: М11 n608 (05.11): сам N608 модели больше не уходит - остаток «диджей выключай» тот же стоп, ход закрывает код
#: (``test_music_stop_honest_m11``). Повтор исполненного стопа моделью проверяется на остатке, что ей уходит.
MODEL_REST = "робот хватит диджей что дальше"
N707 = "расскажи про песню про енота"
DJ_STOP = ToolCall("dj_set", {"action": "stop"})
STATE = ToolCall("get_music_state", {})
NOTHING = "Сейчас ничего не играет."
FACE = Expression(emotion="happy")
TRACKS = [{"id": "gen1", "title": "Енот и луна", "genre": "фолк"}, {"id": "gen2", "title": "Енот-полоскун"},
          {"id": "gen3", "title": "Енот и луна"}]
INFO = {"id": "gen1", "title": "Енот и луна", "genre": "фолк", "mood": "спокойное", "tags": ["енот", "ночь"],
        "path": "/data/gen1/track.mp3", "exists_on_disk": True}


class Ports:
    """Транспорт хода: ответы LLM по очереди; исход - аргументы вызова и данные тула поверх них (как ``owner_event``);
    стоп гасит музыку."""

    def __init__(self, answers, outcomes=None):
        self.answers, self.outcomes = list(answers), outcomes or {}
        self.executed, self.said, self.asked = [], [], 0

    async def ask(self, messages, tools, timeout_s):
        self.asked += 1
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        out = []
        for n, call in enumerate(actions):
            data = {**dict(call.args), **self.outcomes.get((call.tool, call.args.get("action")),
                                                           self.outcomes.get(call.tool, {}))}
            if call.tool in kn.MUSIC_STOP_TOOLS:
                data["was_playing"] = True
            self.executed.append((call.tool, dict(call.args)))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, call.tool,
                                    kn.call_class(call.tool, call.args), "done", None, data))
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


def _query(*calls):
    return Answer(Command(actions=calls), "deepseek", follow=lambda data: ({"role": "tool", "content": data},))


def _run(ports, text, dj=False):
    session = Session("personality")
    if dj:
        session.set_music(MusicStateRef("playing", track_id="gen45685:01:A"))
        session.push_scene("DJ", "user")
    return asyncio.run(TurnEngine(session, ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))


def _result(tool, data, args=None, status="done"):
    return ActionResult("t1/0", 0, "personality", tool, kn.call_class(tool, args or {}), status, None,
                        {**(args or {}), **data})


def _search(query):
    return ToolCall(kn.LIBRARY_TOOL, {"action": "search", "query": query})


# -- n608: стоп сета, чтение состояния, повтор стопа ------------------------------------------------------------------


def test_n608_stop_then_state_then_repeat_says_the_stop_and_the_tool_phrase():
    """Лог М7 n608: Tier-1 стоп молча, ``get_music_state``, повтор ``dj_set stop`` - звучит исход стопа и фраза тула
    о состоянии, а не заглушка."""
    ports = Ports([_query(STATE), Answer(Command(actions=(DJ_STOP,), expression=FACE), "deepseek")],
                  {"get_music_state": {"playing": False, kn.TOOL_MESSAGE_FIELD: NOTHING}})
    agg = _run(ports, MODEL_REST, dj=True)
    assert ("dj_set", {"action": "stop"}) in ports.executed and ("get_music_state", {}) in ports.executed
    assert ports.executed.count(("dj_set", {"action": "stop"})) == 1, "повтор исполненного не исполняется"
    assert ports.said == [f"{STOP_OK} {NOTHING}"], ports.said
    assert not set(ports.said) & set(GENERIC)
    assert agg.stage == "done"


def test_n608_state_without_words_still_says_the_stop_not_the_stub():
    """Состояние прочитано, а слов в его данных нет (старый тул без фразы): звучит исход стопа один, без заглушки."""
    ports = Ports([_query(STATE), Answer(Command(actions=(DJ_STOP,), expression=FACE), "deepseek")],
                  {"get_music_state": {"playing": False}})
    _run(ports, MODEL_REST, dj=True)
    assert ports.said == [STOP_OK], ports.said


# -- n707: поиски в библиотеке, итерации кончились --------------------------------------------------------------------


def test_n707_searches_until_the_ceiling_answer_by_the_last_search():
    """Итерации ``query`` кончились на поисках по названию: код называет найденное последним поиском."""
    queries = [f"енот {n}" for n in range(kn.LLM_QUERY_ITERATIONS_MAX)]
    answers = [*(_query(_search(q)) for q in queries), _query(_search("енот ещё"))]  # последняя - снова query
    ports = Ports(answers, {(kn.LIBRARY_TOOL, "search"): {"tracks": TRACKS, "total": 3}})
    agg = _run(ports, N707)
    last = queries[-1]
    assert ports.said == [f"По запросу «{last}» нашёл треков: 3. Например: «Енот и луна», «Енот-полоскун»."]
    assert agg.stage == "done"


def test_library_info_names_the_track_and_its_card():
    phrase = query_answer([_result(kn.LIBRARY_TOOL, INFO, {"action": "info", "track_id": "gen1"})])
    assert phrase == "Это «Енот и луна»: фолк, спокойное, енот, ночь."


def test_library_search_found_nothing_says_so():
    phrase = query_answer([_result(kn.LIBRARY_TOOL, {"tracks": [], "total": 0}, {"action": "search", "query": "ёж"})])
    assert phrase == "По запросу «ёж» в библиотеке ничего нет."


def test_answer_is_by_the_last_result_of_the_turn():
    """Поиск, потом карточка найденного: ответ - по карточке (последний результат), не по поиску."""
    results = [_result(kn.LIBRARY_TOOL, {"tracks": TRACKS, "total": 3}, {"action": "search", "query": "енот"}),
               _result(kn.LIBRARY_TOOL, INFO, {"action": "info", "track_id": "gen1"})]
    assert query_answer(results).startswith("Это «Енот и луна»")


# -- любой query: фраза тула, иначе названия из данных ----------------------------------------------------------------


def test_tool_without_own_phrase_speaks_the_tool_message():
    status = _result("get_robot_status", {"battery_pct": 81, kn.TOOL_MESSAGE_FIELD: "Заряд 81 %, всё в порядке."})
    assert query_fallback([status]) == "Заряд 81 %, всё в порядке."


def test_tool_without_phrase_and_message_names_titles_from_data():
    voices = _result("get_perception_context", {"people": [{"name": "Денчик"}, {"name": "Саша"}, {"name": "Денчик"}]})
    assert query_fallback([voices]) == "Вот что нашлось: «Денчик», «Саша»."


def test_info_without_card_fields_names_the_title():
    phrase = query_answer([_result(kn.LIBRARY_TOOL, {"title": "Енот и луна"}, {"action": "info", "track_id": "g"})])
    assert phrase == "Вот что нашлось: «Енот и луна»."


def test_generic_stub_only_when_data_has_no_words():
    assert query_fallback([_result("get_current_time", {"ts": 1730000000})]) in GENERIC


def test_new_phrase_keys_are_in_the_catalog():
    catalog = phrases.catalog()
    keys = {*kn.QUERY_FALLBACK_PHRASES[kn.LIBRARY_TOOL].all_keys(), kn.QUERY_NAMES_PHRASE}
    assert keys <= set(kn.PHRASE_KEYS)
    assert all(key in catalog for key in keys)
