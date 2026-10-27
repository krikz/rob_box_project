"""#4079 — путь пения: робот поёт сам (``live:song``/``live:rap``) или генерирует трек (``generate:(robot_)song``).

Решение Шифу 27.10: «кандидат live:song в чойсере, явные слова сужают кодом, голое «спой» решает чойсер по контексту с
перевесом живому». Маркеры — одна таблица ``knowledge.SING_ROUTE_MARKERS`` (+ слова движка ``MUSIC_ENGINES``), выбор —
``source.choose_sing`` до основной модели диалога v2; тесты — на поведение модуля, не на текст промпта.
"""

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.choice import Choice, ChoiceQuestion, DeterministicChooser
from rob_box_music.source import (CacheTrack, SingContext, SourceContext, choose_route, choose_sing, sing_candidates,
                                  sing_kind, sing_markers, source_ask)

RICH = SourceContext(cache=(CacheTrack("k", "STAR WARS", "cover", "club", True, group="g1"),), gen_ready=True,
                     balance=900.0, day_left=120.0)


class NoModel:
    """Модель чойсера, которую звать нельзя: путь решает код."""

    def choose(self, question, *, deadline_ms=0):
        raise AssertionError(f"модель чойсера позвана: {question.text}")


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
])
def test_not_a_request_to_sing_has_no_sing_route(text):
    """Не пение — пустой путь: форма хода диалога не меняется."""
    chosen = choose_sing(text, SingContext(), DeterministicChooser(NoModel()))
    assert sing_kind(text) == "" and chosen.id == "" and chosen.candidates == ()


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


def test_both_routes_named_go_to_the_chooser():
    """Названы оба пути («сгенерируй песню вживую») — код не сужает, оба кандидата у чойсера."""
    assert sing_markers("сгенерируй песню вживую") == ""
    assert [o.id for o in sing_candidates("сгенерируй песню вживую", SingContext())] == [
        "live:song", "generate:song:V6"]


def test_every_marker_of_the_table_names_its_route():
    """Таблица не врёт сама себе: каждая запись узнаётся в просьбе спеть как свой путь; записи строчные, без «ё»."""
    for route, entries in kn.SING_ROUTE_MARKERS.items():
        for entry in entries:
            assert entry == entry.lower() and "ё" not in entry, entry
            assert sing_markers(f"спой {entry} про енота") == route, entry
    assert set(kn.SING_ROUTE_MARKERS) == {"live", "generate"}
    assert set(kn.SOURCE_ASK_STEMS["generated"]) <= set(kn.SING_ROUTE_MARKERS["generate"])  # одна таблица «сгенерируй»


def test_live_candidate_carries_its_facts_for_the_chooser():
    """Факты кандидатов для чойсера: живое — сразу и бесплатно, не сохранится; генерация — минуты, кредиты, библиотека."""
    live, gen = sing_candidates("спой про енота", SingContext())
    assert "сразу и бесплатно" in live.label and "не сохранится" in live.label and live.features["wait_s"] <= 3
    assert "кредитов" in gen.label and "сохранится в библиотеке" in gen.label and gen.features["cost"] > 0


def test_tool_route_of_a_live_request_is_live_not_a_generation():
    """Тул музыки получил «спой под синт» (агент без формы пения: чат, v1) — путь ``live:song`` кодом, без кэша и
    генерации: исполняет диалог, тул генерацию не запускает."""
    route = choose_route(source_ask("спой под синт про енота"), RICH, DeterministicChooser(NoModel()))
    assert (route.kind, route.id) == ("live", "live:song")
    assert "live" in source_ask("спой под синт про енота").describe()
