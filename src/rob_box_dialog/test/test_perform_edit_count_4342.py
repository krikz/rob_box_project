"""#4342: правка счёта-выступления во время seg2 продолжает с первого непрозвучавшего числа."""

from perform_sim import Sim

from rob_box_dialog.command import Performance
from rob_box_dialog.plan.perform import Cancel
from rob_box_dialog.plan.repeats import numerals

COUNT = (
    "Один, два, три, четыре, пять, шесть, семь, восемь, девять, десять.",
    "Одиннадцать, двенадцать, тринадцать, четырнадцать, пятнадцать.",
    "Шестнадцать, семнадцать, восемнадцать, девятнадцать, двадцать.",
)
WITH_STEPS = (
    "Один шаг, два шага.",
    "Три шага, четыре шага, пять шагов.",
    "Шесть шагов, семь шагов, восемь шагов, девять шагов, десять шагов, одиннадцать шагов, двенадцать шагов, "
    "тринадцать шагов, четырнадцать шагов, пятнадцать шагов, шестнадцать шагов, семнадцать шагов, "
    "восемнадцать шагов, девятнадцать шагов, двадцать шагов.",
    "Вот и всё.",
)


def test_count_edit_during_seg2_drops_numbers_not_segments_and_cancels_old_seg3():
    sim = Sim(Performance("poem", COUNT, backing="none"), synth_s=lambda text: 0.1)
    sim.start()
    sim.run(300, stop=lambda: sim.active_no() == 2)
    old_seg3 = sim.block.lines.segments()[2]

    sim.apply(sim.block.edit(Performance("poem", WITH_STEPS, backing="none", from_segment=3), sim.t))
    sim.run(300)

    heard = sim.segments_heard()
    numbers = [number for line in heard for number in numerals(line.text)]
    assert numbers == list(range(1, 21))
    assert any(isinstance(effect, Cancel) and effect.speech_id == old_seg3.speech_id for _, effect in sim.effects)
    assert old_seg3.speech_id not in {line.speech_id for line in heard}
    assert all("шаг" in line.text for line in heard[2:-1])
    assert heard[-1].text == WITH_STEPS[-1]
