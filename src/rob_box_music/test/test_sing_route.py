"""#4079 — путь пения: робот поёт сам (``live:song``/``live:rap``) или генерирует трек (``generate:(robot_)song``).

Решение Шифу 27.10: «кандидат live:song в чойсере, явные слова сужают кодом, голое «спой» решает чойсер по контексту с
перевесом живому». Вилка Шифу 27.10 (2×2, без отказа): «либо на Ренардо поём, либо синтезируем песню; и ещё либо играем
без песни на Ренардо, или генерируем трек без песни» — названный синт с пением всегда живое пение, даже рядом с
«сгенерируй/сочини/запиши» («он должен на «Сгенерируй песню на Ренардо» сделать перформ»). Маркеры — одна таблица
``knowledge.SING_ROUTE_MARKERS`` по приоритету (+ слова движка ``MUSIC_ENGINES``), выбор — ``source.choose_sing`` до
основной модели диалога v2; пение — узко, словами целиком (ревью #4071 B1/B2). Тесты — на поведение модуля.
"""

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.choice import Choice, ChoiceQuestion, DeterministicChooser
from rob_box_music.source import (LIVE_TOOL_TEXT, CacheTrack, SingContext, SourceContext, choose_route, choose_sing,
                                  decline_text, sing_candidates, sing_kind, sing_markers, source_ask, synth_fit)

RICH = SourceContext(cache=(CacheTrack("k", "STAR WARS", "cover", "club", True, group="g1"),), gen_ready=True,
                     balance=900.0, day_left=120.0)


class ModelCalled(BaseException):
    """Модель чойсера позвана там, где путь решает код. ``BaseException``: ``DeterministicChooser`` глотает
    ``Exception`` модели (детерминированный ответ всегда есть) — ``AssertionError`` молча упал бы в него (ревью #4071
    круг 2 NIT-8), а это исключение доходит до теста."""


class NoModel:
    """Модель чойсера, которую звать нельзя: путь решает код. Вызов — :class:`ModelCalled`, тест краснеет."""

    def choose(self, question, *, deadline_ms=0):
        raise ModelCalled(f"модель чойсера позвана: {question.text}")


class Asked:
    """Модель чойсера: запоминает вопрос и выбирает ``pick``."""

    def __init__(self, pick):
        self.pick, self.questions = pick, []

    def choose(self, question, *, deadline_ms=0):
        self.questions.append(question)
        return Choice(self.pick, 0.9, "llm")


#: фраза → путь пения, решённый кодом (один кандидат, модель не зовётся).
BY_WORDS = [
    ("спой под синт про енота", "live:song"),
    ("Робот, спой под бит что-нибудь весёлое", "live:song"),
    ("спой вживую", "live:song"),
    ("спой на синтезаторе", "live:song"),
    ("спой на Ренардо", "live:song"),
    ("спой на рем дарда", "live:song"),  # искажение имени STT (#4070)
    ("Робот, спой живьём про пайку", "live:song"),
    ("спой сам про кота", "live:song"),
    ("спой прямо сейчас", "live:song"),
    ("зачитай рэп под бит", "live:rap"),
    ("сгенерируй песню про енота своим голосом", "generate:robot_song:V6"),
    ("сгенерируй песню про енота", "generate:song:V6"),
    ("запиши песню", "generate:song:V6"),
    ("спой нейросетью про енота", "generate:song:V6"),
    ("сделай песню про енота и спой", "generate:song:V6"),
    ("спой про енота, чтобы сохранилась", "generate:song:V6"),
    ("сгенерируй песню прямо сейчас", "generate:song:V6"),  # «прямо сейчас» слабее «сгенерируй»
    ("исполни песню про енота вживую", "live:song"),
    # вилка Шифу 27.10: синт назван + пение — живое пение, даже рядом с «сгенерируй/сочини/запиши» (N1)
    ("сгенерируй песню на Ренардо", "live:song"),
    ("Робот, сгенерируй песню на ренардо про енота", "live:song"),
    ("сочини песню на синтезаторе", "live:song"),
    ("запиши рэп на Ренардо", "live:rap"),
    ("сгенерируй песню под бит", "live:song"),
    ("сгенерируй песню вживую", "live:song"),
    ("сочини песню на синте", "live:song"),
    ("сгенерируй песню на рем дарда", "live:song"),
    # акт 8 n810: «под этот бит» — играющий бит назван, рэп вживую поверх него
    ("Робот, а теперь под этот бит зачитай короткий рэп про енотика.", "live:rap"),
]


@pytest.mark.parametrize("text,route", BY_WORDS)
def test_sing_route(text, route):
    """Маркер одного пути — кандидат только он, решает код: модель чойсера не зовётся, строка лога — ``(code)``."""
    chosen = choose_sing(text, SingContext(), DeterministicChooser(NoModel()))
    assert (chosen.id, chosen.by, chosen.candidates) == (route, "code", (route,))
    assert chosen.line().endswith(f"кандидаты=[{route}] → {route} (code)")


def test_bare_sing_goes_to_the_chooser_with_both_candidates_and_live_weighs_more():
    """Голое «спой про енота» — чойсер получает и живое, и генерацию; перевес живому (``sing_live``) — с разрывом
    ≥ ``CHOOSER_CLEAR_GAP``: модель без контекста не зовётся, выбирает код."""
    options = sing_candidates("спой про енота", SingContext())
    assert [o.id for o in options] == ["live:song", "generate:song:V6"]
    ranked = DeterministicChooser().rank(ChoiceQuestion("путь", options, weights=kn.SOURCE_WEIGHTS))
    assert ranked[0][1].id == "live:song" and ranked[0][0] - ranked[1][0] >= kn.CHOOSER_CLEAR_GAP
    chosen = choose_sing("спой про енота", SingContext(), DeterministicChooser(NoModel()))
    assert chosen.id == "live:song" and chosen.by == "chooser:deterministic"
    assert chosen.line() == ("[source] пение song слова=— кандидаты=[live:song, generate:song:V6] → live:song "
                             "(chooser:deterministic)")


def test_bare_sing_in_own_voice_offers_the_robot_song_of_the_generation():
    """«Своим голосом» не маркер пути: живое и генерация голосом робота — кандидаты чойсера."""
    assert [o.id for o in sing_candidates("спой своим голосом про енота", SingContext())] == [
        "live:song", "generate:robot_song:V6"]


def test_context_of_recent_replies_hands_the_choice_to_the_model():
    """«Сохрани для Бориса» минуту назад — довод за генерацию (``recent_asked``): разрыв меньше ``CHOOSER_CLEAR_GAP``,
    решает модель чойсера, и она видит фразу, последние реплики, играет ли музыка и здоровье генерации."""
    model = Asked("generate:song:V6")
    ctx = SingContext(recent=("старая реплика", "привет", "сохрани для Бориса"), playing=True, gen_health="ok")
    chosen = choose_sing("спой про енота", ctx, DeterministicChooser(model))
    assert chosen.id == "generate:song:V6" and chosen.by == "chooser:llm"
    question = model.questions[0]
    assert "сохрани для Бориса" in question.text and "старая реплика" not in question.text  # SING_CONTEXT_REPLIES
    assert "играет музыка: да" in question.text and "генерация: ok" in question.text and "«спой про енота»" in question.text
    gen = next(o for o in question.options if o.id.startswith("generate:"))
    assert gen.features["recent_asked"] == 1.0


def test_recent_live_words_do_not_hand_off_but_keep_the_live_lead():
    """Маркер живого в последних репликах («давай под бит») — признак живого: перевес только растёт, модель не нужна."""
    ctx = SingContext(recent=("давай под бит",))
    assert choose_sing("спой про кота", ctx, DeterministicChooser(NoModel())).id == "live:song"


def test_dead_generation_leaves_only_live_for_a_bare_sing_but_not_for_an_explicit_generation():
    """Генерация мертва (здоровье ``music_gen`` не годно): голое «спой» — живое кодом; «сгенерируй» — всё равно
    генерация: человек просил именно её, причину отказа скажет тул фразой кода (``gen_block``)."""
    dead = SingContext(gen_ok=False, gen_health="quota")
    assert choose_sing("спой про енота", dead, DeterministicChooser(NoModel())).id == "live:song"
    assert choose_sing("сгенерируй песню про енота", dead, DeterministicChooser(NoModel())).id == "generate:song:V6"


@pytest.mark.parametrize("text", [
    "сыграй на синтезаторе бит", "сыграй вживую", "поставь синтвейв", "включи рэп", "поставь песню Калинка",
    "сохрани песню, которую ты спел", "расскажи сказку про репку", "сгенерируй рок", "",
    "я прочитал статью про рэп", "исполни мою просьбу",
])
def test_not_a_request_to_sing_has_no_sing_route(text):
    """Не пение — пустой путь: форма хода диалога не меняется."""
    chosen = choose_sing(text, SingContext(), DeterministicChooser(NoModel()))
    assert sing_kind(text) == "" and chosen.id == "" and chosen.candidates == ()


#: Ревью #4071 B1: ложное «пение» — основа «спой» ловила «спойлеры», «вокал» — «без вокала», «свой/твой голос» без
#: глагола пения считался просьбой спеть. Форма живого пения убирала бы ``request_music``/``generate_music`` из среза и
#: требовала ``perform``.
NOT_SINGING_B1 = ["поставь музыку без вокала", "мне нравится твой голос", "что с твоим голосом",
                  "ответь своим голосом", "не рассказывай спойлеры", "Робот, спойлер: фильм кончится хорошо",
                  "сгенерируй трек без вокала", "говори своим голосом, а не чужим"]


@pytest.mark.parametrize("text", NOT_SINGING_B1)
def test_false_singing_of_the_review_b1_is_not_singing(text):
    ask = source_ask(text)
    assert sing_kind(text) == "" and not ask.vocal and not ask.robot_voice and ask.live == ""
    assert choose_sing(text, SingContext(), DeterministicChooser(NoModel())).id == ""


#: Ревью #4071 B2: ложная «генерация» — «сделай песню потише» (правка), «запиши, что …» (заметка), «песни, которые
#: написала нейросеть» (рассказ) давали ``generate:song`` кодом и принудительный платный ``generate_music``.
NOT_GENERATION_B2 = ["сделай песню потише", "сделай песню погромче", "сделай песню быстрее", "сделай песню помедленнее",
                     "запиши, что я люблю песни", "запиши что я люблю песни",
                     "расскажи про песни, которые написала нейросеть",
                     "расскажи про песни которые сгенерировала нейросеть", "песни нейросети мне нравятся"]


@pytest.mark.parametrize("text", NOT_GENERATION_B2)
def test_false_generation_of_the_review_b2_sets_no_generation_form(text):
    chosen = choose_sing(text, SingContext(), DeterministicChooser(NoModel()))
    assert sing_kind(text) == "" and chosen.id == "" and not source_ask(text).vocal


def test_a_volume_or_tempo_word_does_not_make_the_generation_marker_but_keeps_the_singing():
    """«Спой и сделай погромче» — пение (глагол пения правкой не отменяется), но «сделай» без объекта-песни и с правкой
    громкости — не маркер генерации: решает чойсер, а не принудительный ``generate_music``."""
    assert sing_kind("спой и сделай погромче") == "song" and sing_markers("спой и сделай погромче") == ""
    assert sing_kind("спой погромче про енота") == "song"
    assert sing_markers("спой, а потом сделай песню погромче") == ""


@pytest.mark.parametrize("text,kind", [
    ("спой", "song"), ("спойте", "song"), ("можешь спеть?", "song"), ("споёшь про кота?", "song"),
    ("давай споём", "song"), ("пропой припев", "song"), ("напой мелодию", "song"),
    ("исполни песню", "song"), ("исполни рэп", "rap"), ("зачитай рэп", "rap"), ("прочитай рэп про енота", "rap"),
    ("спой рэп", "rap"), ("сгенерируй трек с вокалом", "song"), ("сочини песню", "song"),
    ("придумай песенку про ежа", "song"), ("сгенерируй музон своим голосом", "song"),
    ("сыграй песню своим голосом", "song"), ("спой своим голосом", "song"), ("запиши новую песню про лето", "song"),
    ("сделай песню про енота", "song"), ("сгенерируй громкую песню", "song"),
])
def test_singing_is_a_singing_verb_or_a_song_with_a_verb_to_make_or_perform_it(text, kind):
    """Пение — глагол пения словом целиком, рэп с глаголом исполнения, или песня/вокал/«своим голосом» с глаголом
    создать или исполнить в той же части фразы. «Громкую песню» — прилагательное, не правка громкости."""
    assert sing_kind(text) == kind and source_ask(text).vocal


def _asked(verb):
    """Просьба с глаголом таблицы: инфинитив — со словом просьбы («можешь спеть»), 1-е лицо мн. — с «давай» («давай
    споём», #4090 B), иначе как есть."""
    if verb.endswith(kn.SING_JOINT_ENDINGS):
        return f"давай {verb}"
    return f"можешь {verb}" if verb.endswith(kn.SING_INFINITIVE_ENDING) else verb


def test_every_singing_verb_of_the_table_is_a_whole_word_not_a_stem():
    """Глаголы пения — точные слова: каждое узнаётся само, а слово, которое с него начинается, — нет («спойлер»);
    после «не» — не просьба («не пой больше»), кроме 2-го лица ед. ч.: «не споёшь?» — вежливая просьба (#4090 C)."""
    for verb in kn.SING_VERBS:
        assert verb == verb.lower() and "ё" not in verb, verb
        assert sing_kind(f"{_asked(verb)} про енота") == "song", verb
        assert sing_kind(f"{verb}лер про енота") == "", verb
        polite = verb.endswith(kn.SING_POLITE_ENDING)
        assert sing_kind(f"не {verb} про енота") == ("song" if polite else ""), verb
    assert kn.SOURCE_ASK_STEMS["vocal"] == kn.SING_VERBS  # одна таблица: грамматика v1 берёт те же слова


def test_play_on_the_synth_without_singing_stays_the_synth_of_4070():
    """«Сыграй на синтезаторе бит» без пения — путь синтеза, как в #4071 (движок ``synth``), не живое пение."""
    ask = source_ask("сыграй на синтезаторе бит")
    route = choose_route(ask, RICH, DeterministicChooser(NoModel()))
    assert ask.engine == "synth" and ask.live == "" and (route.kind, route.id) == ("synth", "synth:club")


@pytest.mark.parametrize("text", ["спой в стиле синтвейв", "спой на синт вейв", "спой что-нибудь синти-поп"])
def test_synth_style_words_are_not_the_live_marker(text):
    """Стиль «синтвейв»/«на синт вейв» — не движок и не маркер живого (#4071 не сломан): голое пение, решает чойсер."""
    assert sing_markers(text) == "" and source_ask(text).engine == ""
    assert [o.id for o in sing_candidates(text, SingContext())] == ["live:song", "generate:song:V6"]


@pytest.mark.parametrize("text", ["я сам не знаю, спой что-нибудь", "спой как сам захочешь", "самую весёлую спой"])
def test_bare_word_sam_is_not_the_live_marker(text):
    """«Сам» — маркер только рядом с глаголом пения («спой сам»): «я сам не знаю» — о человеке."""
    assert sing_markers(text) == ""


@pytest.mark.parametrize("text", ["сгенерируй песню вживую", "сгенерируй песню под бит", "сочини песню на синте",
                                  "сгенерируй песню на Ренардо", "запиши песню на синтезаторе, чтобы сохранилась"])
def test_named_synth_with_singing_beats_the_generation_words(text):
    """Ревью #4071 N1 и решение Шифу 27.10: названы оба пути — синт/живое побеждает «сгенерируй»: живое пение кодом,
    без чойсера и без отказа (KIE на Ренардо не играет)."""
    assert sing_markers(text) == "live"
    assert [o.id for o in sing_candidates(text, SingContext())] == ["live:song"]


def test_every_marker_of_the_table_names_its_route_by_priority():
    """Таблица не врёт сама себе: каждая запись узнаётся в просьбе спеть как свой путь (строка вилки ``synth`` —
    клетка «синт + пение» ``MUSIC_FORK``: живое); записи строчные, без «ё»; порядок — приоритет (синт назван →
    генерация → «спой сам/прямо сейчас»)."""
    for row, entries in kn.SING_ROUTE_MARKERS:
        route = kn.MUSIC_FORK[(row, True)].route if (row, True) in kn.MUSIC_FORK else row
        for entry in entries:
            assert entry == entry.lower() and "ё" not in entry, entry
            assert sing_markers(f"спой {entry} про енота") == route, entry
    assert [row for row, _entries in kn.SING_ROUTE_MARKERS] == ["synth", "generate", "live"]
    strong, gen, weak = (entries for _route, entries in kn.SING_ROUTE_MARKERS)
    assert sing_markers(f"спой {strong[0]} {gen[0]}") == "live"
    assert sing_markers(f"спой {weak[-1]} {gen[0]}") == "generate"


def test_every_verb_to_make_a_song_names_its_route():
    """Глаголы «создать песню» (``SING_CREATE_VERBS``): с объектом-песней — пение; значение ``generate`` — маркер
    генерации («сгенерируй/запиши/сделай песню»), «» — путь не назван («сочини/придумай/напиши» — решает чойсер).
    1-е лицо мн. — с «давай» (#4090 C: «давай сочиним песню»)."""
    for verb, route in kn.SING_CREATE_VERBS.items():
        assert verb == verb.lower() and "ё" not in verb, verb
        assert sing_kind(f"{_asked(verb)} песню про енота") == "song", verb
        assert sing_markers(f"{_asked(verb)} песню про енота") == route, verb
        assert sing_kind(f"{_asked(verb)} про енота") == "", verb  # без объекта-песни — не пение
    assert set(kn.SING_CREATE_VERBS.values()) == {"generate", ""}


@pytest.mark.parametrize("text", ["спой метал", "спой блюз про кота", "спой песню в стиле шансон", "спой рок"])
def test_a_genre_the_synth_does_not_play_gives_the_generation_the_lead_but_the_model_decides(text):
    """Ревью #4071 N2: жанр, который синт не играет (``synth_fit`` < ``SYNTH_FIT_MIN``), + голое «спой» — у генерации
    перевес (``synth_poor``), но разрыв меньше ``CHOOSER_CLEAR_GAP``: решает модель чойсера по контексту, без модели —
    генерация. Живое честно говорит, что подложка — свой бит."""
    live, gen = sing_candidates(text, SingContext())
    assert gen.features["synth_poor"] == 1.0 and "синт не сыграет — подложка своим битом" in live.label
    ranked = DeterministicChooser().rank(ChoiceQuestion("путь", (live, gen), weights=kn.SOURCE_WEIGHTS))
    assert ranked[0][1].id == gen.id and ranked[0][0] - ranked[1][0] < kn.CHOOSER_CLEAR_GAP
    model = Asked("live:song")  # модель видит контекст и может выбрать живое
    assert choose_sing(text, SingContext(), DeterministicChooser(model)).id == "live:song" and model.questions
    assert choose_sing(text, SingContext(), DeterministicChooser()).id == gen.id
    dead = SingContext(gen_ok=False, gen_health="quota")
    assert choose_sing(text, dead, DeterministicChooser(NoModel())).id == "live:song"


def test_a_genre_the_synth_plays_keeps_the_live_lead():
    """Жанр, который синт играет («клуб», «синтвейв»), — без ``synth_poor``: перевес живому, модель не зовётся."""
    for text in ("спой в стиле синтвейв", "спой клубную песню"):
        assert synth_fit(source_ask(text).genre) >= kn.SYNTH_FIT_MIN, text
        assert choose_sing(text, SingContext(), DeterministicChooser(NoModel())).id == "live:song", text


def test_live_candidate_carries_its_facts_for_the_chooser():
    """Факты кандидатов для чойсера: живое — сразу и бесплатно, не сохранится; генерация — минуты, кредиты, библиотека."""
    live, gen = sing_candidates("спой про енота", SingContext())
    assert "сразу и бесплатно" in live.label and "не сохранится" in live.label and live.features["wait_s"] <= 3
    assert "кредитов" in gen.label and "сохранится в библиотеке" in gen.label and gen.features["cost"] > 0


# -- ревью #4071 круг 2: пение — глагол и его ПРЯМОЕ дополнение, не слова фразы вразброс ------------------------------

#: BLOCK-1: «сделай/запиши + свой/твой голос» давало ``generate:robot_song`` кодом (платный ``generate_music``).
NOT_SINGING_VOICE = ["Робот, сделай свой голос помягче", "Робот, сделай свой голос повыше",
                     "Робот, сделай твой голос женским", "Робот, запиши свой голос", "Сделай голос потише"]
#: BLOCK-2: «запиши/сделай» + «песн…» не дополнением или правка звучащего — давало ``generate:song`` кодом.
NOT_GENERATION_OBJECT = ["Робот, запиши название песни", "Робот, запиши название этой песни",
                         "Робот, запиши слова песни", "Робот, запиши заметку про песню",
                         "Робот, сделай эту песню на повтор", "Робот, сделай песню на полную громкость",
                         "Робот, сделай песню потихоньку", "Робот, сделай песню не такой громкой",
                         "Робот, запиши песню в избранное", "Робот, запиши эту песню",
                         "Робот, запиши мне песню на флешку", "Робот, сделай песню, пожалуйста, потише",
                         "Робот, запиши песни Цоя", "Робот, сыграй песню Калинка", "Робот, сыграй песню"]
#: BLOCK-3: «создай/придумай/напиши» + «песн…» не дополнением — давало ``live:song`` (``request_music`` вне среза).
NOT_SINGING_OBJECT = ["Робот, создай плейлист песен", "Робот, создай плейлист из любимых песен",
                      "Робот, придумай плейлист из песен для вечеринки", "Робот, придумай название для песни",
                      "Робот, придумай название песни", "Робот, напиши письмо про песню", "Робот, напиши текст песни"]
#: BLOCK-4: голос и песня без глагола пения — давало ``live:song`` с обязательным ``perform``.
NOT_SINGING_NO_VERB = ["Мне нравится твой голос и твои песни", "Мне нравится твой голос в этой песне",
                       "Эта песня твоим голосом звучит классно", "Расскажи про песни своим голосом"]
#: BLOCK-5: «рэп» и глагол чтения не глагол с дополнением — давало ``live:rap`` (``request_music`` вне среза).
NOT_SINGING_RAP = ["Робот, включи рэп и прочитай новости", "Робот, прочитай мне новости про рэп",
                   "Робот, прочитай статью про рэп", "Робот, прочитай стих про рэп"]
#: NIT-2: инфинитив пения о себе или без просьбы — человек говорит о себе, а не просит робота.
NOT_SINGING_SELF = ["Я хочу спеть караоке", "Я не могу спеть эту песню", "Мы хотим спеть вместе",
                    "Я хочу сочинить песню", "Я мог бы спеть", "Расскажи, как спеть эту песню", "Где спеть караоке?",
                    "Не пой больше", "Не спой а сыграй"]


@pytest.mark.parametrize("text", NOT_SINGING_VOICE + NOT_GENERATION_OBJECT + NOT_SINGING_OBJECT + NOT_SINGING_NO_VERB
                         + NOT_SINGING_RAP + NOT_SINGING_SELF)
def test_false_singing_or_generation_of_the_review_round_2_is_not_singing(text):
    """Ни пения, ни генерации: путь пустой, форма хода не меняется (``request_music``/``generate_music`` в срезе,
    ``perform`` не обязателен), кредиты KIE не тратятся."""
    ask = source_ask(text)
    assert sing_kind(text) == "" and sing_markers(text) != "generate" and not ask.vocal and not ask.robot_voice
    assert choose_sing(text, SingContext(), DeterministicChooser(NoModel())).id == ""


#: Подлинные просьбы: фраза → путь пения (живое/генерация), без модели чойсера (перевес живому — разрыв ≥ порога).
GENUINE = [
    ("Спой мне колыбельную", "live:song"), ("Давай споём", "live:song"), ("Споешь что-нибудь?", "live:song"),
    ("Можешь спеть мне песню?", "live:song"), ("Спой-ка про енота", "live:song"), ("Пропой алфавит", "live:song"),
    ("Исполни песню про лето", "live:song"), ("Исполни эту песню", "live:song"), ("Исполни рэп про енота", "live:rap"),
    ("Зачитай рэп про кота", "live:rap"), ("Рэп мне зачитай", "live:rap"),
    ("Зачитай под бит рэп про енота", "live:rap"), ("Сочини мне ещё одну короткую весёлую песенку", "live:song"),
    ("Сочини и спой песню про енота", "live:song"), ("Попробуй сочинить песню про кота", "live:song"),
    ("Я прошу тебя спеть", "live:song"), ("Робот, пой", "live:song"), ("Робот, пой про енота", "live:song"),
    ("Пой со мной", "live:song"), ("Сыграй песню своим голосом", "live:song"),
    ("Сгенерируй мне песню про лето", "generate:song:V6"), ("Запиши новую песню про зиму", "generate:song:V6"),
    ("Сделай песню про дождь", "generate:song:V6"), ("Сгенерируй трек со словами про осень", "generate:song:V6"),
    ("Сгенерируй трек с вокалом", "generate:song:V6"), ("Можешь сгенерировать песню про лето?", "generate:song:V6"),
    ("Сгенерируй пиздатый музон своим голосом", "generate:robot_song:V6"),
    ("Сделай музыку своим голосом", "generate:robot_song:V6"), ("Запиши рэп-трек про енота", "generate:song:V6"),
]


@pytest.mark.parametrize("text,route", GENUINE)
def test_genuine_requests_keep_their_sing_route(text, route):
    assert choose_sing(text, SingContext(), DeterministicChooser(NoModel())).id == route


def test_every_object_of_the_table_is_a_direct_object_of_the_verb():
    """Дополнение — точное слово ``SING_OBJECTS`` рядом с глаголом: песня и рэп поют, музыка — только с голосом;
    то же слово не дополнением глагола («придумай название для …») — не пение."""
    for kind, words in kn.SING_OBJECTS.items():
        for word in words:
            assert word == word.lower() and "ё" not in word, word
            sung = sing_kind(f"сочини {word} про енота")
            assert sung == {"song": "song", "rap": "rap", "music": ""}[kind], word
            assert sing_kind(f"сгенерируй {word} своим голосом") in ("song", "rap"), word
            assert sing_kind(f"придумай название для {word}") == "", word


def test_only_the_instrumental_own_voice_is_the_voice_of_the_robot():
    """Голос робота — «своим/твоим голосом» фразой (``ROBOT_VOICE_PHRASES``): музыку с ним поют (``robot_song``);
    «свой голос», «твой голос» — сам голос, не голос песни (BLOCK-1/4)."""
    for phrase in kn.ROBOT_VOICE_PHRASES:
        assert sing_kind(f"сгенерируй трек {phrase}") == "song" and source_ask(f"спой {phrase}").robot_voice, phrase
    assert sing_kind("сгенерируй трек под свой голос") == ""
    assert sing_kind("сгенерируй трек, мне нравится твой голос") == ""
    assert not source_ask("спой, мне нравится твой голос").robot_voice


def test_every_word_of_editing_or_storing_the_song_is_not_a_request_to_make_one():
    """Правка или хранение звучащего (``SING_CREATE_MODIFIERS``) — не просьба создать: генерации нет; спеть можно."""
    for entry in kn.SING_CREATE_MODIFIERS:
        assert entry == entry.lower() and "ё" not in entry, entry
        assert sing_kind(f"сделай песню {entry}") == "" and sing_markers(f"сделай песню {entry}") == "", entry
        assert sing_kind(f"спой {entry}") == "song", entry


def test_a_sounding_or_someone_elses_song_is_not_made_but_can_be_sung():
    """«Эту/мою песню» (``SING_EXISTING_WORDS``) — звучащая или чужая: создать нельзя, исполнить можно."""
    for word in kn.SING_EXISTING_WORDS:
        assert sing_kind(f"запиши {word} песню") == "" and sing_kind(f"{word} песню запиши") == "", word
        assert sing_kind(f"исполни {word} песню") == "song", word


def test_an_infinitive_is_a_request_only_with_a_word_of_asking_and_not_about_oneself():
    """Инфинитив — просьба только со словом просьбы (``SING_INFINITIVE_ASKS``) и не о себе (``SING_SELF_WORDS`` без
    ``SING_ROBOT_AGENT_WORDS``)."""
    for ask in kn.SING_INFINITIVE_ASKS:
        assert sing_kind(f"{ask} спеть про енота") == "song", ask
    for me in kn.SING_SELF_WORDS:
        assert sing_kind(f"{me} хочу спеть") == "" and sing_kind(f"{me} прошу тебя спеть") == "song", me
    assert sing_kind("спеть про енота") == ""


NEGATED_OR_THEME_ENGINE = ["Робот, сгенерируй песню не на синтезаторе", "Робот, сгенерируй песню без синтезатора",
                           "Робот, сгенерируй мне настоящую песню, а не на синтезаторе",
                           "Робот, сгенерируй песню, но не вживую", "Робот, запиши песню, а не пой вживую",
                           "Робот, сгенерируй песню про синтезатор", "Робот, сгенерируй песню про Ренардо"]


@pytest.mark.parametrize("text", NEGATED_OR_THEME_ENGINE)
def test_a_negated_engine_or_an_engine_as_the_theme_does_not_name_the_synth(text):
    """Ревью #4071 круг 2 NIT-1: «не/без» перед словом движка и «про/о» (тема) движок не называют — код не делает
    обратное просьбе: генерация, как просили, а не живое пение; синт не назван и в пути тула."""
    assert source_ask(text).engine == "" and sing_markers(text) == "generate"
    assert choose_sing(text, SingContext(), DeterministicChooser(NoModel())).id == "generate:song:V6"


def test_every_negation_and_theme_word_of_the_table_unnames_the_engine():
    for neg in kn.ENGINE_NEGATIONS:
        assert source_ask(f"сыграй {neg} на синтезаторе").engine == "", neg
        assert sing_markers(f"сгенерируй песню, {neg} на Ренардо") == "generate", neg
    for lead in kn.ENGINE_TOPIC_LEADS:
        assert source_ask(f"сгенерируй песню {lead} Ренардо").engine == "", lead
    assert source_ask("сыграй на синтезаторе").engine == "synth"  # без отрицания — синт, как в #4070
    assert sing_markers("спой, но не нейросетью") == ""  # отрицание снимает и маркер таблицы


def test_tool_route_of_a_live_request_is_live_not_a_generation():
    """Тул музыки получил «спой под синт» (агент без формы пения: чат, v1) — путь ``live:song`` кодом, без кэша и
    генерации: исполняет диалог, тул генерацию не запускает."""
    route = choose_route(source_ask("спой под синт про енота"), RICH, DeterministicChooser(NoModel()))
    assert (route.kind, route.id) == ("live", "live:song")
    assert "live" in source_ask("спой под синт про енота").describe()
    # ревью #4071 круг 2 NIT-5: в агенте без формы пения выступления может не быть — фраза тула пения здесь не обещает
    # (#4090 G.2: «отсюда не выйдет»), называет, где поёт робот (вслух, из динамиков), и что сказать; генерацию не запускает
    assert decline_text(route) == LIVE_TOOL_TEXT and "отсюда не выйдет" in LIVE_TOOL_TEXT
    assert "вслух" in LIVE_TOOL_TEXT and "«Робот, спой под синт»" in LIVE_TOOL_TEXT
    assert "Генерацию для этого не запускаю" in LIVE_TOOL_TEXT


def test_a_named_voice_of_the_registry_narrows_the_bare_sing_to_live():
    """Ревью #4153: «спой голосом Алисы про кабана» — кандидатами были живое и генерация, чойсер (``music_chooser:
    chain`` на роботе) мог выбрать Suno, форма закрывала выступление, и голос молча терялся. Голос реестра назван
    (``SingContext.voiced`` — решает диалог) — только живое: этим голосом поёт робот сам, кодом, без модели."""
    voiced = SingContext(voiced=True)
    assert [o.id for o in sing_candidates("спой голосом Алисы про кабана", voiced)] == ["live:song"]
    assert choose_sing("спой голосом Алисы про кабана", voiced, DeterministicChooser(NoModel())).by == "code"
    assert [o.id for o in sing_candidates("спой голосом Алисы про кабана", SingContext())] == [
        "live:song", "generate:song:V6"], "без названного голоса — как раньше, решает чойсер"


def test_an_explicit_generation_keeps_its_route_next_to_a_named_voice():
    """Путь, названный словами («с Suno», «сгенерируй песню»), сильнее голоса: человек просил именно генерацию."""
    voiced = SingContext(voiced=True)
    assert [o.id for o in sing_candidates("спой с Suno голосом Алисы", voiced)] == ["generate:song:V6"]
    assert [o.id for o in sing_candidates("спой голосом Алисы на синтезаторе", voiced)] == ["live:song"]
