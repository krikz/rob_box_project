"""#4090 — хвост вилки пения после #4071: правка звучащего ≠ заказ песни, «мне/нам», ложные минусы, «на синтезаторе» =
«на Ренардо», вилка 2×2 одной таблицей (ADR-0148: решает структура и одна таблица ``knowledge``, тест — поведение).

Каждую фразу пунктов A–C ревьюер PR #4071 воспроизвёл пробником на head 3626aaa25; решение товарища Шифу по D —
Telegram 27.10 17:07 UTC: «на синтезаторе» = «на Ренардо» и для инструментала (синт ближайшим стилем, бесплатно, без
KIE).
"""

from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.choice import Choice, DeterministicChooser
from rob_box_music.source import (CacheTrack, SingContext, SourceContext, choose_route, choose_sing, fork_cell,
                                  route_note, settle_engine, sing_kind, sing_markers, source_ask, source_candidates)

RICH = SourceContext(cache=(CacheTrack("k", "STAR WARS", "cover", "club", True, group="g1"),), gen_ready=True,
                     balance=900.0, day_left=120.0)


class ModelCalled(BaseException):
    """Модель чойсера позвана там, где путь решает код (``BaseException`` проходит сквозь ``DeterministicChooser``)."""


class NoModel:
    def choose(self, question, *, deadline_ms=0):
        raise ModelCalled(f"модель чойсера позвана: {question.text}")


class Generates:
    """Модель чойсера, которая выбрала бы генерацию: если она позвана — путь решил не код."""

    def choose(self, question, *, deadline_ms=0):
        return Choice(next(o.id for o in question.options if o.id.startswith("generate:")), 0.9, "llm")


def _sing(text):
    return choose_sing(text, SingContext(), DeterministicChooser(NoModel()))


def _route(text):
    return choose_route(source_ask(text), RICH, DeterministicChooser(Generates()))


# ── A. Правка звучащего ≠ заказ песни ───────────────────────────────────────────────────────────────────────────────

#: Все фразы карточки #4090 A (давали ``generate:song:V6 (code)`` — принудительный платный ``generate_music``).
EDITS_OF_THE_CARD = [
    "сделай песню слышнее", "сделай песню короче", "сделай песню длиннее", "сделай песню веселее",
    "сделай песню лучше", "сделай песню чище", "сделай песню поярче", "сделай песню попроще", "сделай песню пониже",
    "сделай песню повыше", "сделай песню звонче", "сделай песню без шума", "сделай песню с эхом",
    "сделай песню в моно", "сделай песню на репите", "сделай песню на паузу", "сделай песню на 10 секунд короче",
    "сделай песню ещё раз", "сделай песню снова", "запиши песню на диктофон",
    "запиши песню, которую я сейчас спою",
]
#: Та же структура в других словах: степень перед сравнительной, «менее/более», правка до глагола, предлог между
#: глаголом и песней, повтор где угодно, служебные слова и запятые в хвосте.
EDITS_BY_STRUCTURE = [
    "Робот, сделай песню чуть громче", "сделай песню немного тише", "сделай песню менее громкой",
    "сделай песню более весёлой", "сделай песню, пожалуйста, побыстрее", "погромче сделай песню",
    "запиши на флешку песню", "снова сделай песню", "запиши мне песню ещё раз", "сделай песню на всю громкость",
    "запиши песню в избранное", "сделай песню потихоньку", "сделай песню не такой громкой",
    "запиши песню, которую мы споём", "сделай песню, которую я напою, на репите",
    "сделай песню на темпе повыше",  # «темпе» — не «тему» (ревью PR #4107: основа «тем» ловила «темп»)
]


@pytest.mark.parametrize("text", EDITS_OF_THE_CARD + EDITS_BY_STRUCTURE)
def test_editing_or_repeating_the_sounding_song_is_not_an_order_of_a_song(text):
    """Правка звучащего у «сделай/запиши» — не просьба создать: ни пения, ни маркера генерации, ни формы хода;
    кредиты KIE не тратятся, решает модель."""
    ask = source_ask(text)
    assert sing_kind(text) == "" and sing_markers(text) != "generate" and not ask.vocal, text
    assert _sing(text).id == "" and _sing(text).candidates == ()


#: Заказ остаётся заказом: хвост — слот заказа (тема, стиль, повод, голос, адресат), движок или другой путь.
ORDERS_WITH_A_TAIL = [
    ("сделай песню про енота", "generate:song:V6"), ("запиши новую песню про зиму", "generate:song:V6"),
    ("сделай песню в стиле рок", "generate:song:V6"), ("сделай песню на день рождения", "generate:song:V6"),
    ("сделай песню на тему лета", "generate:song:V6"), ("сделай песню с вокалом", "generate:song:V6"),
    ("сделай песню со словами про осень", "generate:song:V6"), ("сделай песню для мамы", "generate:song:V6"),
    ("сделай песню на английском", "generate:song:V6"),
    ("запиши песню, которая поднимет настроение", "generate:song:V6"),
    ("сделай песню без синтезатора", "generate:song:V6"),  # движок с отрицанием — слот заказа, генерация
    ("сделай песню на Ренардо", "live:song"), ("запиши песню на синтезаторе", "live:song"),
    ("сделай песню на рем дарда", "live:song"),
    # повтор с темой — новый заказ, не повтор звучащего (ревью PR #4107)
    ("сделай песню про енота ещё раз", "generate:song:V6"), ("ещё раз сделай песню про зиму", "generate:song:V6"),
    ("сделай песню на Новый год", "generate:song:V6"), ("сделай песню на новогодний праздник", "generate:song:V6"),
]


@pytest.mark.parametrize("text,route", ORDERS_WITH_A_TAIL)
def test_an_order_with_a_slot_of_the_order_stays_an_order(text, route):
    assert _sing(text).id == route


@pytest.mark.parametrize("text", ["сгенерируй песню короче", "сгенерируй песню с эхом", "сочини песню лучше прежней",
                                  "сочини песню, которую я спою маме", "сгенерируй песню на две минуты"])
def test_verbs_without_the_second_reading_take_any_tail_as_a_detail_of_the_order(text):
    """«Сгенерируй», «сочини», «напиши» не правят звучащее (``SING_EDIT_VERBS`` — только «сделай/запиши»): хвост —
    подробность заказа, пение остаётся."""
    assert sing_kind(text) == "song"


def test_every_comparative_ending_degree_word_and_repeat_of_the_tables_is_an_edit():
    """Таблицы правки не врут сами себе: каждое окончание сравнительной степени, слово степени, повтор, предлог правки
    и запись таблицы правки отменяют заказ у глагола второго чтения и не отменяют пение глаголом пения."""
    for ending in kn.SING_COMPARATIVE_ENDINGS:
        word = "звон" + ending
        assert sing_kind(f"сделай песню {word}") == "" and sing_kind(f"спой {word}") == "song", ending
    for degree in kn.SING_DEGREE_WORDS:
        assert sing_kind(f"сделай песню {degree} громче") == "", degree
    for repeat in kn.SING_REPEAT_PHRASES:
        assert sing_kind(f"сделай песню {repeat}") == "" and sing_kind(f"спой {repeat}") == "song", repeat
    for prep in kn.SING_EDIT_PREPOSITIONS:
        assert sing_kind(f"сделай песню {prep} шуме") == "", prep
    for entry in kn.SING_CREATE_MODIFIERS:
        assert sing_kind(f"сделай песню {entry}") == "" and sing_kind(f"спой {entry}") == "song", entry


def test_every_slot_of_the_order_after_an_edit_preposition_keeps_the_order():
    for prep, stems in kn.SING_ORDER_TAILS.items():
        for stem in stems:
            assert sing_kind(f"сделай песню {prep} {stem}ах") == "song", (prep, stem)


def test_edit_verbs_are_verbs_to_make_and_only_they_read_the_tail():
    assert set(kn.SING_EDIT_VERBS) <= set(kn.SING_CREATE_VERBS)
    edit_rules = [rule for rule in kn.SING_VERB_RULES if rule.edits]
    assert len(edit_rules) == 1 and set(edit_rules[0].verbs) == set(kn.SING_EDIT_VERBS)


# ── B. «мне/нам» — человек о себе ───────────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("text", ["Робот, мне надо спеть на конкурсе", "нам нужно спеть дуэтом, подскажи песню",
                                  "послушай, мы споём", "Мы споём вечером", "мне нужно сочинить песню к пятнице",
                                  "со мной надо спеть", "мною нужно спеть"])
def test_the_speaker_about_himself_is_not_a_request_to_the_robot(text):
    """Карточка #4090 B: давали ``live:song`` — форма live требовала ``perform``. Говорящий («мне/нам/мной»,
    ``SING_SELF_WORDS``) без исполнителя-робота и «споём» без «давай» — не просьба."""
    assert {"мне", "нам", "мной", "мною"} <= set(kn.SING_SELF_WORDS)
    assert sing_kind(text) == "" and _sing(text).id == ""


@pytest.mark.parametrize("text,route", [
    ("Можешь спеть мне песню?", "live:song"), ("Можете спеть нам?", "live:song"), ("Давай споём", "live:song"),
    ("давайте споём про енота", "live:song"), ("Спой мне колыбельную", "live:song"), ("Пой со мной", "live:song"),
    ("тебе надо спеть мне", "live:song"),
])
def test_the_robot_asked_to_sing_for_the_speaker_still_sings(text, route):
    """Исполнитель — робот: слово просьбы 2-го лица (``SING_INFINITIVE_ASKS`` ``True``), «тебе», повелительное,
    «давай»."""
    assert _sing(text).id == route


def test_second_person_asks_of_the_table_name_the_robot_as_the_singer():
    for ask, robot in kn.SING_INFINITIVE_ASKS.items():
        assert sing_kind(f"мне {ask} спеть") == ("song" if robot else ""), ask


# ── C. Ложные минусы → правильная клетка вилки ──────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("text,route", [
    ("ты не споёшь на синте?", "live:song"),  # «не» + 2-е лицо ед. ч. — вежливая просьба
    ("не сочинишь песню на Ренардо?", "live:song"),
    ("сгенерируй две песни на синтезаторе", "live:song"),  # вин. мн. «песни» после слова количества
    ("сгенерируй пару песен про лето", "generate:song:V6"),
    ("сгенерируй две новые песни про лето", "generate:song:V6"),  # между счётом и «песни» — прилагательное мн. ч.
    ("давай сочиним песню на синтезаторе", "live:song"), ("давай напишем песню на синтезаторе", "live:song"),
    ("давай сгенерируем песню про енота", "generate:song:V6"),
    ("рэп под бит", "live:rap"), ("рэп под бит, пожалуйста.", "live:rap"), ("порепь под бит", "live:rap"),
    ("песню на Ренардо", "live:song"), ("давай рэп на синте", "live:rap"), ("под бит рэп", "live:rap"),
])
def test_false_minus_of_the_card_goes_to_the_right_cell_of_the_fork(text, route):
    """Карточка #4090 C: форма не ставилась, а путь тула молча подменял клетку («сгенерируй две песни на синтезаторе»
    → платный ``generate:instrumental``). Теперь — клетка вилки кодом, и в пути пения, и в пути тула."""
    assert _sing(text).id == route
    tool = _route(text)
    assert tool.id == route or (route.startswith("generate:") and tool.kind == "generate"), (text, tool.id)


@pytest.mark.parametrize("text", ["Робот, запиши песни Цоя", "запиши слова песни", "включи рэп под бит",
                                  "мне нравится рэп под бит", "песню про енота", "не пой на синте",
                                  "сочиним песню на синтезаторе", "ты больше не пой",
                                  "сделай пару копий песни"])
def test_the_fixes_of_false_minus_do_not_make_new_false_plus(text):
    """Мн. ч. без слова количества — не дополнение (и через существительное: «пару копий песни» — «песни» —
    родительный «копий»); часть фразы с глаголом не-пения или без названного синта — не просьба спеть; «сочиним» без
    «давай» — о себе; «не пой» — отрицание (не 2-е лицо)."""
    assert sing_kind(text) == "" and _sing(text).id == ""


#: Ревью PR #4107 (пробник на 5692b0d4b давал ``live:*`` — форма, которая обязана петь): песня/рэп без глагола с
#: названным синтом, но предложение — не заказ: отрицание, оценка, сказуемое, слово-стоп.
NOT_AN_ELLIPTIC_ORDER = [
    "песню на синтезаторе не пой", "песню под бит не включай", "песню под бит не надо", "рэп под бит не люблю",
    "рэп под бит — это круто", "рэп под бит классный", "рэп на синтезаторе лучше", "рэп под бит мне нравится",
    "песню на синтезаторе мы слушали вчера", "хватит рэп под бит", "нет рэп под бит", "я не люблю, когда рэп под бит",
    "рэп под бит, это круто", "трек под бит",
]


@pytest.mark.parametrize("text", NOT_AN_ELLIPTIC_ORDER)
def test_song_or_rap_with_the_synth_but_no_verb_is_an_order_only_when_the_sentence_is_the_order(text):
    """Код не делает обратное сказанному (#4071, ADR-0148): клетка «синт + пение» (``live``: форма закрывает тулы
    музыки и требует ``perform``) — только когда предложение целиком заказ («рэп под бит», «рэп под бит,
    пожалуйста»). Любое другое слово — и пения кодом нет: ни путь пения, ни путь тула его не ставят, решает модель."""
    assert sing_kind(text) == "" and _sing(text).id == ""
    assert not _route(text).id.startswith("live:"), _route(text).id


def test_elliptic_order_reads_the_whole_sentence_not_the_clause():
    """Предложение кончается на ``.!?;``: заказ в своём предложении — заказ, оценка в том же — нет."""
    assert sing_kind("Привет! Рэп под бит.") == "rap"
    assert sing_kind("Рэп под бит, классный.") == ""
    assert sing_kind("Песню на Ренардо. Рэп под бит не люблю.") == "song"


# ── D. «На синтезаторе» = «на Ренардо» и для инструментала ──────────────────────────────────────────────────────────

@pytest.mark.parametrize("text,route_id", [
    ("сгенерируй трек на синтезаторе", "synth:club"), ("сгенерируй трек на синте", "synth:club"),
    ("Робот, сгенерируй музыку на синтезаторе", "synth:club"), ("сочини что-нибудь на синтезаторе", "synth:club"),
    ("сгенерируй метал на синтезаторе", "synth:breaks"), ("сгенерируй джаз на синте", "synth:synthwave"),
])
def test_generation_on_the_synthesizer_plays_the_synth_in_the_nearest_style_for_free(text, route_id):
    """Решение Шифу 27.10 17:07 UTC: «сгенерируй трек на синтезаторе» — синт ближайшим стилем, бесплатно; генерации и
    кэша в кандидатах нет (модель, выбравшая бы генерацию, не позвана)."""
    options, reason = source_candidates(source_ask(text), RICH)
    assert [o.id for o in options] == [route_id] and reason == ""
    route = _route(text)
    assert (route.kind, route.id, route.provider) == ("synth", route_id, "code")


def test_nearest_style_of_a_genre_the_synth_does_not_play_is_said_by_the_code():
    route = _route("сгенерируй метал на синтезаторе")
    assert route_note(route) == "Метал на синте играю ближайшим стилем — брейкбит."


@pytest.mark.parametrize("text", ["сгенерируй трек на синтезаторе", "сгенерируй рок на синте",
                                  "Робот, сочини трек на синтезаторе"])
def test_settle_engine_keeps_the_synth_slot_of_the_model_when_the_person_named_the_synth(text):
    ask, note = settle_engine(source_ask(text), "synth")
    assert ask.engine == "synth" and note == ""


def test_live_and_by_meaning_words_still_leave_every_path_of_a_generation_open():
    """«Вживую» — по смыслу, не слово движка: «сгенерируй трек вживую» — все пути (решение D — только про синт)."""
    ask = source_ask("сгенерируй трек вживую")
    assert ask.engine == "" and _route("сгенерируй трек вживую").kind == "generate"


# ── H. Вилка 2×2 — одна таблица, её читают путь пения, путь тула и вид генерации ───────────────────────────────────

@pytest.mark.parametrize("text,row,sung,route_id", [
    ("спой на Ренардо", "synth", True, "live:song"), ("сыграй на синтезаторе", "synth", False, "synth:club"),
    ("сгенерируй песню про енота", "generate", True, "generate:song:V6"),
    ("сгенерируй трек про енота", "generate", False, "generate:instrumental:V6"),
])
def test_each_cell_of_the_fork_is_its_row_of_the_table(text, row, sung, route_id):
    cell = fork_cell(row, sung)
    assert _route(text).id == route_id and route_id.startswith(cell.route)
    assert cell.kind in route_id or cell.route == "synth"


def test_the_fork_has_exactly_four_cells_with_one_path_each():
    assert set(kn.MUSIC_FORK) == {(row, sung) for row in ("synth", "generate") for sung in (True, False)}
    assert {cell.route for cell in kn.MUSIC_FORK.values()} == {"live", "synth", "generate"}


def test_sing_route_tool_route_and_kind_of_generation_read_the_table(monkeypatch):
    """Решение не продублировано: подмена клетки таблицы меняет и путь пения (``sing_markers``), и путь тула
    (``_named_cell``), и вид генерации (``_gen_option``)."""
    monkeypatch.setitem(kn.MUSIC_FORK, ("synth", True), kn.ForkCell("generate", "song"))
    assert sing_markers("спой на Ренардо") == "generate" and sing_markers("спой под бит") == "generate"
    monkeypatch.setitem(kn.MUSIC_FORK, ("synth", False), kn.ForkCell("live", "rap"))
    options, _reason = source_candidates(source_ask("сыграй на Ренардо"), RICH)
    assert [o.id for o in options] == ["live:rap"]
    monkeypatch.setitem(kn.MUSIC_FORK, ("generate", False), kn.ForkCell("generate", "song"))
    gen = [o.id for o in source_candidates(source_ask("сгенерируй трек"), RICH)[0] if o.id.startswith("generate:")]
    assert gen == ["generate:song:V6"]


def test_tool_route_and_sing_route_agree_on_every_named_cell():
    """Путь тула (``source_candidates``) и путь пения (``choose_sing``) на одной фразе с названным синтом дают одну
    клетку: живое пение — у обоих, синтез — только у тула (не пение)."""
    for text in ("спой на синтезаторе", "сгенерируй песню на Ренардо", "рэп под бит", "ты не споёшь на синте?",
                 "сгенерируй две песни на синтезаторе"):
        tool = [o.id for o in source_candidates(source_ask(text), RICH)[0]]
        assert tool == [_sing(text).id], text
    ask = replace(source_ask("сыграй на синтезаторе"), vocal=False)
    assert [o.id for o in source_candidates(ask, RICH)[0]] == ["synth:club"] and _sing("сыграй на синтезаторе").id == ""
