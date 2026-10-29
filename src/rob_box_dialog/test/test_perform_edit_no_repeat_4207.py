"""#4207: правка идущего выступления не повторяет сказанное (марафоны 4-6, акт 8, n807: «посчитай от одного до
двадцати»,
посреди счёта «и после каждого числа добавляй слово шаг»).

Модель присылала счёт С НАЧАЛА, код ставил его на места несказанных сегментов - числа 9-19 звучали дважды. Теперь
``repeats.drop_said`` снимает ведущие повторы, а блок контекста показывает модели ``last_said`` и ``pending``."""

from perform_sim import Sim

from rob_box_dialog.command import Performance
from rob_box_dialog.plan.perform import context_block
from rob_box_dialog.plan.repeats import drop_said, is_counting, numerals, words

# Марафон-5 (run 40801693422, 29.10 01:43): шесть сегментов счёта по логу (6-й в логе обрезан - «И двадцать…»).
COUNT = (
    "Один, два, три, четыре, пять —",
    "Шесть, семь, восемь, девять, десять —",
    "Одиннадцать, двенадцать, тринадцать,",
    "Четырнадцать, пятнадцать, шестнадцать,",
    "Семнадцать, восемнадцать, девятнадцать",
    "И двадцать.",
)
# Ответ модели на «после каждого числа добавляй слово шаг»: счёт С НАЧАЛА (в логе с «…», раскрыто по смыслу).
FROM_START = (
    "Один шаг, два шага, три шага, четыре шага, пять шагов, шесть шагов, семь шагов, восемь шагов.",
    "Девять шагов, десять шагов, одиннадцать шагов, двенадцать шагов, тринадцать шагов, четырнадцать шагов, "
    "пятнадцать шагов, шестнадцать шагов.",
    "Семнадцать шагов, восемнадцать шагов, девятнадцать шагов, двадцать шагов. Вот и всё, Саша — спокойно дошли.",
)


def _heard_numbers(sim):
    return [n for h in sim.segments_heard() for n in numerals(h.text)]


def test_n807_m5_log_verbatim_counting_is_not_repeated():
    sim = Sim(Performance("poem", COUNT, backing="none"))
    sim.start()
    sim.run(300, stop=lambda: sim.active_no() == 4)
    assert sim.block.active_segment == 4
    sim.apply(sim.block.edit(Performance("poem", FROM_START, backing="none", from_segment=5), sim.t))
    sim.run(300)
    heard = [h.text for h in sim.segments_heard()]
    assert heard == [*COUNT[:4], FROM_START[2]]
    # 1-16 как звучали, дальше «семнадцать шагов…», без возврата к «один»/«девять»
    numbers = _heard_numbers(sim)
    assert numbers == sorted(set(numbers)) == list(range(1, 21))  # каждое число ровно один раз, по порядку
    assert any("сказанное не повторяю" in ln for ln in sim.logs)
    assert sim.events("rejected") == []


# Марафон-6 (run 40815389578, 29.10 04:06): старые сегменты 1-3 в логе не приведены (в сегменте 4 - до «семнадцать»).
COUNT6 = (
    "Один. Два. Три. Четыре. Пять.",
    "Шесть. Семь. Восемь. Девять. Десять.",
    "Одиннадцать. Двенадцать. Тринадцать. Четырнадцать.",
    "Пятнадцать. Шестнадцать. Семнадцать.",
    "Восемнадцать. Девятнадцать.",
    "Двадцать.",
)
# Девять сегментов правки, как в логе («сегменты [6..14] заменены»): счёт с начала, «шаг» после числа, концовка.
REDO6 = (
    "Один — шаг. Два — шаг. Три — шаг.",
    "Четыре — шаг. Пять — шаг. Шесть — шаг.",
    "Семь — шаг. Восемь — шаг. Девять — шаг.",
    "Десять — шаг. Одиннадцать — шаг. Двенадцать — шаг.",
    "Тринадцать — шаг. Четырнадцать — шаг. Пятнадцать — шаг.",
    "Шестнадцать — шаг. Семнадцать — шаг. Восемнадцать — шаг.",
    "Девятнадцать — шаг.",
    "Двадцать — шаг. Все дошли.",
)


def test_n807_m6_log_verbatim_edit_while_the_old_count_is_nearly_over():
    """Правка пришла, когда досчитано до «девятнадцать», а «двадцать» ещё не отдано: вместо 9 сегментов повтора 1-19 -
    только «Двадцать — шаг. Все дошли.» на месте последнего."""
    sim = Sim(Performance("poem", COUNT6, backing="none"), synth_s=lambda text: 0.1)
    sim.start()
    sim.run(300, stop=lambda: sim.active_no() == 5)
    sim.apply(sim.block.edit(Performance("poem", REDO6, backing="none", from_segment=6), sim.t))
    sim.run(300)
    assert [h.text for h in sim.segments_heard()] == [*COUNT6[:5], REDO6[-1]]
    numbers = _heard_numbers(sim)
    assert numbers == sorted(set(numbers)) == list(range(1, 21))  # ни одного повтора числа


def test_n807_m6_same_edit_not_synthesized_by_the_boundary_still_has_no_repeats():
    """Как в логе: правка сегмента 6 не успела к границе (О7) - звучит прежнее «Двадцать.», повторов 4-20 нет."""
    sim = Sim(Performance("poem", COUNT6, backing="none"))
    sim.start()
    sim.run(300, stop=lambda: sim.active_no() == 5)
    sim.apply(sim.block.edit(Performance("poem", REDO6, backing="none", from_segment=6), sim.t))
    sim.run(300)
    assert [h.text for h in sim.segments_heard()] == list(COUNT6)
    numbers = _heard_numbers(sim)
    assert numbers == sorted(set(numbers)) == list(range(1, 21))


def test_n807_m6_edit_after_the_old_count_reached_twenty_adds_only_the_ending():
    """Старый счёт уже договорился до конца (последний сегмент отдан на звучание): вся правка - повтор 1-20, остаётся
    только «Все дошли.»; в начале счёта заново («Четыре — шаг…») не звучит."""
    sim = Sim(Performance("poem", COUNT6, backing="none"))
    sim.start()
    sim.run(300, stop=lambda: sim.active_no() == 6)
    sim.apply(sim.block.edit(Performance("poem", REDO6, backing="none", from_segment=7), sim.t))
    sim.run(300)
    assert [h.text for h in sim.segments_heard()] == [*COUNT6, "Все дошли."]
    numbers = _heard_numbers(sim)
    assert numbers == sorted(set(numbers)) == list(range(1, 21))
    assert not any("шаг" in h.text for h in sim.heard)


def test_n807_everything_said_and_nothing_new_is_rejected_old_tail_stays():
    count = COUNT6[:5] + ("Двадцать. Все дошли.",)
    sim = Sim(Performance("poem", count, backing="none"))
    sim.start()
    sim.run(300, stop=lambda: sim.active_no() == 5)
    sim.apply(sim.block.edit(Performance("poem", REDO6[:-1], backing="none", from_segment=6), sim.t))
    (rejected,) = sim.events("rejected")
    assert rejected.reason == "edit_repeats_said"
    sim.run(300)
    assert [h.text for h in sim.segments_heard()] == list(count)  # прежний хвост дочитан как был


def test_counting_row_with_new_numbers_loses_its_said_leading_sentences():
    said = ["Пятнадцать. Шестнадцать. Семнадцать.", "Восемнадцать. Девятнадцать."]
    rows = [("Четырнадцать — шаг. Пятнадцать — шаг.", "", ""),
            ("Шестнадцать — шаг. Семнадцать — шаг. Восемнадцать — шаг. Девятнадцать — шаг. Двадцать — шаг. Все дошли.",
             "voice", "happy"),
            ("Двадцать — шаг.", "", "")]
    assert drop_said(rows, ["Четырнадцать.", *said]) == [("Двадцать — шаг. Все дошли.", "voice", "happy"), rows[2]]


def test_exact_repeat_of_a_said_segment_is_dropped_only_when_leading():
    said = ["Припев: ля-ля-ля, тополя.", "Куплет про комара."]
    rows = [("Припев: ля-ля-ля, тополя!", "", ""), ("А теперь про енота.", "", ""),
            ("Припев: ля-ля-ля, тополя.", "", "")]
    assert drop_said(rows, said) == rows[1:]  # припев после нового - законный, остаётся


def test_ordinary_edit_is_untouched():
    """«Добавь про кота», «медленнее»: нового текста с числами и без - ни один сегмент не снимается."""
    said = ["Я комар, звеню над ухом. Один в поле не воин, но пою.", "Три дня я пел, пять ночей молчал."]
    rows = [("А вот кот пришёл, и он один.", "", ""), ("Три кота и пять мышей — счёт такой.", "", ""),
            ("Раз, два, три, четыре — кот пошёл гулять.", "", "")]
    assert drop_said(rows, said) == rows
    assert drop_said([], said) == []


def test_single_numeral_stories_are_not_counting():
    assert not is_counting("Один в поле не воин.") and not is_counting("Три богатыря вышли в поле.")
    assert is_counting("Один шаг, два шага.")
    assert words("Один — ЁЖ, и два!") == ("один", "еж", "и", "два")
    assert numerals("Девять шагов, 10 шагов, одна, две") == (9, 10, 1, 2)


def test_context_block_shows_what_was_said_and_what_is_left():
    sim = Sim(Performance("poem", COUNT, backing="none"))
    sim.start()
    sim.run(300, stop=lambda: sim.active_no() == 4)
    ctx = context_block(sim.block, sim.t)
    assert ctx["last_said"] == {"no": 4, "text": COUNT[3]}
    assert ctx["pending"] == list(COUNT[4:]) and ctx["first_pending"] == 5
    assert context_block(Sim(Performance("poem", COUNT, backing="none")).block, 0.0)["last_said"] is None
