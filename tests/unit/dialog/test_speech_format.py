"""Оформление речи кодом в консоли (``scripts/dialog/speech_format.py``, эксперимент 29.10): экран и провайдеры голоса.

Пакеты нормализатора и ударений не нужны: тесты дают провайдерам фейковые функции (``Tools``).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
# Только то, что нужно speech_format: сам скрипт и пакет rob_box_dialog (rap_pauses — чистая stdlib).
# Не через dry_stand: он тянет мост команд и клиент openai, которого нет в job Offline Unit Tests.
for _p in (REPO / "scripts" / "dialog", REPO / "src" / "rob_box_dialog"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import speech_format as sf  # noqa: E402

VERSE = "Еду в 2024 году\nУ меня семьдесят восемь процентов заряда\nКод мой чистый и быстрый"


def tools(**kw):
    """Нормализатор пишет «2024» словами; ударения - «+» перед первой гласной каждого слова."""
    def stress(text):
        return " ".join(next((w[:i] + "+" + w[i:] for i, ch in enumerate(w) if ch in "аеёиоуыэюяАЕЁИОУЫЭЮЯ"), w)
                        for w in text.split(" "))
    base = dict(normalize=lambda t: t.replace("2024", "две тысячи двадцать четвёртом"), normalize_why="",
                normalizer_name="fake", stress=stress, stress_why="")
    return sf.Tools(**{**base, **kw})


def voiced(name, text, bar_s=0.0, **kw):
    return sf.providers(tools(**kw), [name])[0].voice(text, bar_s)


def test_screen_puts_the_dash_by_the_role_voice_not_by_the_text():
    assert sf.screen("Ты делал бэкап?", "Russian_ReliableMan") == "— Ты делал бэкап?"
    assert sf.screen("Приходит кодер к тимлиду.", "") == "Приходит кодер к тимлиду."


@pytest.mark.parametrize("name, tag", [("minimax", "<#"), ("yandex", "sil<["), ("silero", '<break time="')])
def test_each_provider_writes_the_beat_pause_in_its_own_markup(name, tag):
    out = voiced(name, VERSE, bar_s=2.0).text
    assert out.count(tag) == 2  # три строки - две паузы
    assert "<#" not in out or name == "minimax"


def test_beat_timing_spells_numbers_even_for_a_provider_that_reads_them():
    """Такт меряет строку гласными: «2024» без нормализатора дал бы строке ноль слогов и паузу не по такту."""
    with_beat = voiced("minimax", VERSE, bar_s=2.0)
    assert "две тысячи" in with_beat.text
    assert "2024" in voiced("minimax", VERSE).text  # без такта MiniMax читает цифры сам


def test_prose_gets_no_beat_pauses():
    out = voiced("minimax", "Приходит кодер на работу. В кармане ноутбук.", bar_s=2.0)
    assert "<#" not in out.text and "не куплет" in sf.steps_line(out.steps)


def test_only_yandex_gets_stress_marks_and_never_on_one_vowel_words():
    assert "+" not in voiced("minimax", "Белки на ветке").text
    assert "+" not in voiced("silero", "Белки на ветке").text  # Silero ставит ударения сам (put_accent)
    assert voiced("yandex", "Белки на ветке").text == "Б+елки на в+етке"


def test_silero_gets_escaped_ssml_with_its_breaks_intact():
    out = voiced("silero", "Чистый & быстрый\nКак вода", bar_s=2.0).text
    assert out.startswith("<speak>") and out.endswith("</speak>") and "&amp;" in out and '<break time="' in out


def test_a_missing_package_is_a_visible_skipped_step():
    out = voiced("silero", "В 2024 году", normalize=None, normalize_why="нет пакета (rutextnorm)")
    assert "2024" in out.text and "нет пакета (rutextnorm)" in sf.steps_line(out.steps)


def test_unknown_provider_and_normalizer_are_refused():
    with pytest.raises(ValueError):
        sf.providers(tools(), ["elevenlabs"])
    with pytest.raises(ValueError):
        sf.load_tools("num2words", stress=False)
