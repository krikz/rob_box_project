"""#4157: «скажи обо мне пять фактов» - на последней итерации после чтения памяти модель вернула одно лицо без слов,
а код сказал неправду «не смог сложить в короткую фразу» (ADR-0148: решает код, а не промпт). Марафон-2 28.10: n109
«какие звуки ты умеешь издавать», n908 «рецепт борща без лука» (``memory_search`` + ``search_web``, последняя - одно лицо).

1. Форма последней итерации после выполненного запроса (``turn.answer_form`` → ``Form.speech_required``): ответ без слов
   - нарушение валидатора ``VIOLATION_SPEECH_REQUIRED`` (повтор моста с подсказкой). Схему форма не меняет:
   ``required: [speech]`` спорил бы с ``speech_with_actions`` (звук после запроса законен без слов).
2. Модель всё равно без слов - код называет 2–3 пункта из данных (``turn.query_answer``): факты памяти (kind=fact, по
   score; только если ход читал одну память), категории звуков, выдача поиска. Слоты строит одно место; общая фраза
   причины не называет.
"""

import asyncio

import pytest

from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Expression, Form, Invalid, ToolCall, parse, validate
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine, _Turn, answer_form, query_answer, query_fallback

MAX = kn.LLM_QUERY_ITERATIONS_MAX
N_FACTS = "скажи обо мне пять фактов которые ты знаешь"
N_SOUNDS = "какие звуки ты вообще умеешь издавать"
FACE = Expression(emotion="happy", duration_s=12)  # ответ последнего вызова в живом логе: лицо без слов
SEARCH = ToolCall("memory_search", {"query": "факт", "limit": 10})
SEARCH_2 = ToolCall("memory_search", {"query": "Денчик"})
SEARCH_3 = ToolCall("memory_search", {"query": "любить"})
SOUNDS = ToolCall("get_sound_info", {})

#: Результат ``memory_search`` как его отдаёт тул (``MemorySearchTool``): факты и реплики прошлых разговоров, ``score``.
HITS = [
    {"kind": "turn", "role": "user", "content": "я люблю кататься на велосипеде", "score": 0.99, "source": "hybrid"},
    {"kind": "fact", "role": "fact", "content": "Денчик любит зелёный чай.", "score": 0.40, "category": "preference"},
    {"kind": "fact", "role": "fact", "content": "Денчик попросил называть его Шифу.", "score": 0.91,
     "category": "preference"},
    {"kind": "fact", "role": "fact", "content": "Денчик болеет за Спартак", "score": 0.75, "category": "general"},
    {"kind": "fact", "role": "fact", "content": "Денчик любит зелёный чай.", "score": 0.30, "category": "preference"},
    {"kind": "fact", "role": "fact", "content": "Денчик ест пиццу по пятницам", "score": 0.10, "category": "habit"},
]
FACTS_PHRASE = ("Вот что нашёл в памяти: Денчик попросил называть его Шифу; Денчик болеет за Спартак; "
                "Денчик любит зелёный чай.")
SOUND_DATA = {"total_sounds": 51, "categories": {"base": 21, "ui": 12, "robot": 18, "other": 0},
              "all_triggers": ["activation", "affirmation", "robot_happy"]}
SOUNDS_PHRASE = "Звуков у меня 51: эмоции — 21, интерфейс — 12, эффекты — 18. Назвать какие-нибудь?"


class Ports:
    """Транспорт хода: ответы LLM по очереди; на каждый вызов - форма хода и схема команды, какими их увидел бы мост;
    исход действия - аргументы вызова и данные тула поверх них (как у ``owner_event``)."""

    def __init__(self, answers=(), outcomes=None):
        self.answers, self.outcomes = list(answers), outcomes or {}
        self.asked, self.probes, self.executed, self.said, self.stages = [], [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        self.probes.append((cm.current_form(), cm.form_schema(kn.command_schema("json"))))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        out = []
        for n, a in enumerate(actions):
            status, data = self.outcomes.get((a.tool, a.args.get("query")), self.outcomes.get(a.tool, ("done", {})))
            out.append(ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool,
                                    kn.call_class(a.tool, a.args), status, None, {**dict(a.args), **data}))
        return out

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append((text, voice, emotion))
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="",
              expression=None):
        self.stages.append((stage, status, reason))

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


def _bare(expression=FACE):
    return Answer(Command(expression=expression), "deepseek")


def _run(ports, text):
    return asyncio.run(TurnEngine(Session("personality"), ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))


def _result(tool, data, status="done", args=None):
    return ActionResult("t1/0", 0, "personality", tool, kn.call_class(tool, args or {}), status, None,
                        {**(args or {}), **data})


def _turn(queried=(), text=N_FACTS, pickup=False):
    return _Turn("t1", 0, text, VOICE, deadline=None, pickup=pickup, queried=tuple(queried))


def _snap(agent="personality", silence=False):
    session = Session(agent)
    if silence:
        session.push_scene("Silence", "user")
    return session.snapshot


# -- 1. форма последней итерации требует слов --------------------------------------------------------------------------


def test_only_the_last_iteration_after_a_query_owes_words():
    """Зонд формы: две итерации чтения памяти - слова не обязательны; последняя (query-тулов в ней уже нет) - форма
    требует слов, валидатор моста видит это. После хода форма не течёт в чужие вызовы."""
    final = Answer(Command(speech="Помню, что ты зовёшь меня Шифу."), "deepseek")
    ports = Ports([_query(SEARCH), _query(SEARCH_2), final], {"memory_search": ("done", {"results": HITS})})
    agg = _run(ports, N_FACTS)
    assert len(ports.probes) == MAX + 1
    assert [form.speech_required for form, _ in ports.probes] == [False] * MAX + [True]
    assert ports.said == [("Помню, что ты зовёшь меня Шифу.", None, "neutral")] and agg.status == "finished"
    assert cm.current_form() == Form()


def test_the_schema_does_not_require_speech_it_would_contradict_speech_with_actions():
    """Схему форма не меняет: после запроса звук без слов - законный ответ («какие звуки» → ``play_sound``), а слова
    рядом со звуком - ``speech_with_actions``. ``required: [speech]`` толкал бы модель в это нарушение."""
    final = Answer(Command(speech="Звуков много."), "deepseek")
    ports = Ports([_query(SOUNDS), _query(SOUNDS), final], {"get_sound_info": ("done", SOUND_DATA)})
    _run(ports, N_SOUNDS)
    assert [schema for _, schema in ports.probes] == [ports.probes[0][1]] * (MAX + 1)
    assert "speech" not in ports.probes[-1][1]["required"]
    asked = Form(passed=False, voice=False, speech_required=True)
    sound = {"actions": [{"tool": "play_sound", "args": {"sound": "robot_happy"}}]}
    assert isinstance(validate(parse(sound), ALL, form=asked), Command)
    both = validate(parse({**sound, "speech": "Вот так."}), ALL, form=asked)
    assert isinstance(both, Invalid) and both.reason == kn.VIOLATION_SPEECH_WITH_ACTIONS


def test_form_is_open_when_the_query_was_never_done(monkeypatch):
    """Единственный вызов хода - последний, но запроса не было: читать нечего, форма открыта."""
    monkeypatch.setattr(kn, "LLM_QUERY_ITERATIONS_MAX", 0)
    ports = Ports([Answer(Command(speech="Привет."), "deepseek")])
    _run(ports, "привет, как дела")
    assert [form.speech_required for form, _ in ports.probes] == [False]


@pytest.mark.parametrize("kw, snap, expected", [
    ({}, {}, True),
    ({"pickup": True}, {}, False),                       # подхват бэклога: честное молчание открыто
    ({"text": "покажи как тебе грустно"}, {}, False),   # лицо по просьбе человека - ответ без слов законен
    ({"text": "покажи это молча"}, {}, False),
    ({"queried": ()}, {}, False),                        # запроса в ходе не было
    ({}, {"silence": True}, False),                      # тишина: в ней молчат (#3983)
    ({}, {"agent": "telegram"}, False),                  # не в динамики: как words_owed (#3995)
])
def test_answer_form_requires_words_where_words_owed_does(kw, snap, expected):
    turn = _turn(**{"queried": (_result("memory_search", {"results": HITS}),), **kw})
    assert answer_form(Form(), turn, True, _snap(**snap)).speech_required is expected
    assert answer_form(Form(), turn, False, _snap(**snap)).speech_required is False, "не последняя - ход повторит сам"


def test_answer_form_leaves_the_sing_route_alone():
    """Путь пения решён кодом (``perform`` или ``generate_music`` обязательны): требовать слов рядом с ними нельзя."""
    turn = _turn((_result("memory_search", {"results": HITS}),))
    live = answer_form(Form(passed=False, sing="live:song"), turn, True, _snap())
    assert live.speech_required is False and live.sing == "live:song"


def test_answer_form_keeps_everything_else_of_the_turn_form():
    base = Form(passed=False, voice=False)
    asked = answer_form(base, _turn((_result("memory_search", {"results": HITS}),)), True, _snap())
    assert (asked.passed, asked.voice, asked.sing) == (False, False, "") and asked.speech_required
    assert asked.closed() == base.closed()
    full = kn.command_schema("json")
    assert cm.form_schema(full, asked) == cm.form_schema(full, base), "схема та же"


# -- 1б. ответ без слов на этой форме - нарушение для моста ------------------------------------------------------------

ALL = ("memory_search", "play_animation", "play_sound", "memory_save")
ASKED = Form(passed=False, voice=False, speech_required=True)
ANIM = {"actions": [{"tool": "play_animation", "args": {"animation": "happy"}}]}


@pytest.mark.parametrize("raw", [{"expression": {"emotion": "happy", "duration_s": 12}}, ANIM,
                                 {"expression": {"earcon": "chime"}}])
def test_answer_without_words_on_the_asked_form_is_a_violation_with_a_hint(raw):
    res = validate(parse(raw), ALL, form=ASKED)
    assert isinstance(res, Invalid) and res.reason == kn.VIOLATION_SPEECH_REQUIRED
    note = cm.explain(res, after_query=True)
    assert kn.VIOLATION_SPEECH_REQUIRED in note and "в speech" in note and "данные запроса уже получены" in note
    opened = Form(passed=False, voice=False)
    assert isinstance(validate(parse(raw), ALL, form=opened), Command), "форма открыта - как было"


@pytest.mark.parametrize("raw", [
    {"speech": "Помню, что ты любишь чай."},
    {"ask": {"expects": "yes_no", "text": "Назвать ещё?"}},
    {"actions": [{"tool": "play_sound", "args": {"sound": "robot_happy"}}]},    # исход звука скажет код
    {"speech": "Запомнил.", "actions": [{"tool": "memory_save", "args": {"fact": "любит чай"}}]},
    {"speech": "Слова", "expression": {"emotion": "happy"}},
])
def test_answer_with_words_or_an_action_the_code_voices_passes_the_asked_form(raw):
    assert isinstance(validate(parse(raw), ALL, form=ASKED), Command)


# -- 2. модель всё равно без слов - код называет пункты из данных -----------------------------------------------------


def test_model_without_words_after_memory_search_code_names_the_facts_of_the_result():
    """Живой 28.10 13:40: два чтения памяти, на последней итерации - одно лицо. Код называет факты из результата
    (kind=fact, по score, без дублей, без реплик прошлых разговоров), не «не смог сложить в короткую фразу»."""
    ports = Ports([_query(SEARCH), _query(SEARCH_2), _bare()], {"memory_search": ("done", {"results": HITS})})
    agg = _run(ports, N_FACTS)
    assert len(ports.asked) == MAX + 1 and (agg.stage, agg.status) == ("done", "done")
    assert ports.said == [(FACTS_PHRASE, None, None)], "ответ по данным - без грусти"
    assert ports.stages[-1] == ("done", "done", f"query_fallback:{kn.VIOLATION_SILENT_ANSWER}")
    assert phrases.text(kn.QUERY_FALLBACK_GENERIC) not in {t for t, _, _ in ports.said}


def test_facts_read_on_an_earlier_iteration_survive_an_empty_last_search():
    """Факты прочитаны на первой итерации, а последний поиск вернул пусто: код называет то, что модель прочла за ход."""
    ports = Ports([_query(SEARCH), _query(SEARCH_3), _bare()],
                  {("memory_search", "факт"): ("done", {"results": HITS}),
                   ("memory_search", "любить"): ("done", {"results": []})})
    _run(ports, N_FACTS)
    assert ports.said == [(FACTS_PHRASE, None, None)]


def test_model_without_words_after_get_sound_info_code_names_the_categories():
    """Живой 28.10 15:22, n109: три итерации и та же заглушка на «какие звуки ты умеешь издавать»."""
    ports = Ports([_query(SOUNDS), _query(SOUNDS), _bare()], {"get_sound_info": ("done", SOUND_DATA)})
    agg = _run(ports, N_SOUNDS)
    assert ports.said == [(SOUNDS_PHRASE, None, None)] and (agg.stage, agg.status) == ("done", "done")


N_BORSCHT = "робот расскажи рецепт борща без лука пошагово"
WEB = {"query": "рецепт борща без лука пошагово", "backend": "ddgs",
       "results": [{"title": "Борщ без лука", "body": "Сварите бульон, добавьте свёклу и капусту...", "url": "u"}]}
FOOD = [{"kind": "fact", "content": "Денчик не ест лук.", "score": 1.0}]


def test_n908_memory_beside_a_web_search_the_code_reads_the_search_not_the_food_facts():
    """Марафон-2 n908: ``memory_search 'еда'`` + ``search_web``, затем ``memory_search 'лук'``, на последней - одно
    лицо. Данные ответа - выдача поиска; факты о еде рядом с ней - контекст, не ответ на «рецепт борща». Не общая
    заглушка."""
    first = _query(ToolCall("memory_search", {"query": "еда"}), ToolCall("search_web", {"query": WEB["query"]}))
    ports = Ports([first, _query(ToolCall("memory_search", {"query": "лук"})), _bare()],
                  {"memory_search": ("done", {"results": FOOD}), "search_web": ("done", WEB)})
    agg = _run(ports, N_BORSCHT)
    assert ports.said == [("Вот что нашёл в сети: Сварите бульон, добавьте свёклу и капусту.", None, None)]
    assert [form.speech_required for form, _ in ports.probes] == [False] * MAX + [True]
    assert (agg.stage, agg.status) == ("done", "done")


def test_web_search_slots_take_the_first_row_with_text():
    rows = {"results": [{"title": "пусто", "body": " "}, {"title": "t", "body": "Борщ   варят с\nсвёклой..."}]}
    assert query_answer([_result("search_web", rows)]) == "Вот что нашёл в сети: Борщ варят с свёклой."
    assert query_answer([_result("search_web", {"results": []})]) is None
    assert query_answer([_result("search_web", {"results": [{"body": "x"}]}, status="rejected")]) is None


def test_memory_facts_are_the_answer_only_when_the_turn_read_nothing_else():
    memory = _result("memory_search", {"results": HITS})
    assert query_answer([memory, _result("memory_search", {"results": []})]) == FACTS_PHRASE
    assert query_answer([memory, _result("get_sound_info", SOUND_DATA)]) == SOUNDS_PHRASE
    assert query_answer([memory, _result("search_web", {"results": []})]) is None, "контекст рядом с поиском - не ответ"
    assert query_answer([memory, _result("search_web", WEB, status="rejected")]) == FACTS_PHRASE, "поиск не выполнен"


def test_model_without_words_after_a_search_without_facts_says_the_honest_generic_phrase():
    turns_only = [h for h in HITS if h["kind"] == "turn"]
    ports = Ports([_query(SEARCH), _query(SEARCH_2), _bare()], {"memory_search": ("done", {"results": turns_only})})
    agg = _run(ports, N_FACTS)
    assert ports.said == [(phrases.text(kn.QUERY_FALLBACK_GENERIC), None, None)] and agg.status == "done"


def test_generic_phrase_names_no_false_reason():
    """Причина «слишком длинно» - только у ``command.too_long`` (#4137); слов по данным нет - ответ не сложился."""
    text = phrases.text(kn.QUERY_FALLBACK_GENERIC)
    assert "коротк" not in text and "длин" not in text and "не сложил" in text
    assert "длин" in phrases.text("command.too_long")


# -- слоты по данным: одно место ``turn.query_answer`` ----------------------------------------------------------------


def test_memory_search_names_up_to_three_distinct_facts_by_score():
    assert query_answer([_result("memory_search", {"results": HITS}, args={"query": "факт"})]) == FACTS_PHRASE
    one = [{"kind": "fact", "content": "Любит чай. ", "score": 0.2}]
    assert query_answer([_result("memory_search", {"results": one})]) == "Вот что нашёл в памяти: Любит чай."
    no_score = [{"kind": "fact", "content": "Первый"}, {"kind": "fact", "content": "Второй", "score": None}]
    assert query_answer([_result("memory_search", {"results": no_score})]) == "Вот что нашёл в памяти: Первый; Второй."


@pytest.mark.parametrize("data", [{}, {"results": []}, {"results": "нет"},
                                  {"results": [{"kind": "turn", "content": "x"}]},
                                  {"results": [{"kind": "fact", "content": "  "}]}])
def test_memory_search_without_facts_has_no_phrase_by_data(data):
    assert query_answer([_result("memory_search", data)]) is None


def test_memory_context_names_the_facts_in_the_owner_order():
    facts = [{"id": n, "fact": f"Факт {n}.", "category": "general", "speaker_id": "s1"} for n in range(1, 6)]
    data = {"facts": facts, "recent_turns": [{"role": "user", "content": "привет", "session": "s"}]}
    assert query_answer([_result("memory_context", data)]) == "Вот что нашёл в памяти: Факт 1; Факт 2; Факт 3."
    assert query_answer([_result("memory_context", {"facts": [], "recent_turns": []})]) is None


def test_memory_phrase_is_not_longer_than_the_speech_that_is_heard():
    long_fact = ("Очень длинный рассказ о том, как мы вместе собирали робота и спорили о синтезаторах. " * 3).strip()
    hits = [{"kind": "fact", "content": f"{n}. {long_fact}", "score": 1 - n / 10} for n in range(3)]
    text = query_answer([_result("memory_search", {"results": hits})])
    assert text is not None and len(text) <= kn.SPEECH_ACCEPT_MAX_CHARS
    assert text.startswith("Вот что нашёл в памяти: 0.")


def test_get_sound_info_overview_names_the_categories_and_the_rest_as_other():
    assert query_answer([_result("get_sound_info", SOUND_DATA)]) == SOUNDS_PHRASE
    odd = {"total_sounds": 5, "categories": {"base": 2, "ui": 0, "robot": 0, "weird": 3}}
    expected = "Звуков у меня 5: эмоции — 2, другие — 3. Назвать какие-нибудь?"
    assert query_answer([_result("get_sound_info", odd)]) == expected


@pytest.mark.parametrize("data", [
    {}, {"total_sounds": 0, "categories": {"base": 0}}, {"total_sounds": "много", "categories": {"base": 1}},
    {"sounds": ["robot_happy", "robot_sad"], "category": "base", "details": {}},      # список одной категории
    {"trigger": "robot_happy", "category": "base", "description_ru": "Радость"},        # один звук
])
def test_get_sound_info_other_shapes_have_no_phrase_by_data(data):
    assert query_answer([_result("get_sound_info", data)]) is None


def test_failed_query_has_no_phrase_by_data_and_keeps_the_outcome_phrase():
    """Запрос без ``done`` - фраза исхода, как раньше (ревью PR #4055): не «нашёл» и не факты."""
    refused = _result("memory_search", {"results": HITS}, status="rejected")
    assert query_answer([refused]) is None
    assert query_fallback([refused]) == phrases.text("query.rejected")
    assert query_fallback([refused, _result("memory_search", {"results": HITS})]) == FACTS_PHRASE


def test_every_query_slot_builder_has_a_phrase_and_back():
    from rob_box_dialog.turn import _QUERY_SLOTS

    assert set(_QUERY_SLOTS) == set(kn.QUERY_FALLBACK_PHRASES)
    assert {"memory_search", "memory_context", "get_sound_info", "search_web"} <= set(kn.QUERY_FALLBACK_PHRASES)
    assert set(kn.MEMORY_FACTS_TOOLS) <= set(kn.QUERY_FALLBACK_PHRASES)
