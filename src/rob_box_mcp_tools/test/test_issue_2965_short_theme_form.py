"""Регрессионный тест issue #2965 — «Drop It Like It's Hot».

Live 24.09.2026 (``docker logs voice-assistant`` на Vision Pi, LLM
minimax): RTTTL-тема ``dropitli`` — 2 такта — растянулась на полную
64-тактовую форму ``arc``: одна и та же 2-тактовая фраза без единой
вариации повторялась 32 раза подряд, а тул сообщил «Полная форма звучит
154 секунд», хотя в базе на неё было материала на 2 такта.

Системный фикс — :data:`core.arranger.MAX_THEME_REPEATS`, кап на бюджет
повторов в ``_snap_plan_to_theme`` (см. ``test_arranger.py::
TestShortThemeRepeatCap`` — проверка на синтетических длинах темы,
плюс прогон по всему RTTTL-архиву в описании PR). Этот файл — НЕ
проверка фикса (её делает ``TestShortThemeRepeatCap`` на любой длине
темы), а страховка регрессии конкретно на записи, из-за которой завели
issue: если однажды кто-то сделает кап зависимым от имени темы вместо
её длины, этот тест упадёт первым.
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock

for _mod in [
    "rclpy", "rclpy.node", "rclpy.action", "rclpy.qos",
    "std_msgs", "std_msgs.msg",
    "geometry_msgs", "geometry_msgs.msg",
    "nav2_msgs", "nav2_msgs.action",
    "action_msgs", "action_msgs.srv", "action_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.core.arranger import (  # noqa: E402
    FORMS,
    MAX_THEME_REPEATS,
    form_duration_seconds,
    form_summary,
    render,
    resolve_form,
    spec_from_flat,
)
from rob_box_mcp_tools.core.rtttl_compose import (  # noqa: E402
    melody_to_compose_params,
    rtttl_to_melody,
)

#: Дословно из живого архива (data/rtttl_melodies.jsonl.gz, ключ
#: "dropitli") — та самая запись из issue #2965, а не пересочинённая
#: строка.
DROPITLI_RTTTL = (
    "DropItLi:d=4,o=6,b=100:32a,32p,32g_,32p,16b,32p,32a,32p,32g_,32p,16g_,"
    "16p,16g,16p,32e,32p,32c_,32p,32c_,32p,32c_,32p,8e,16p,32e,32p,32c_,32p,"
    "32c_,32p,32c_,32p,8e,16p,32e,32p,32c_,32p,32c_,32p,32c_,32p,8e"
)


def _dropitli_spec():
    params = melody_to_compose_params(rtttl_to_melody(DROPITLI_RTTTL))
    return spec_from_flat(
        harmony=params["harmony"],
        bpm=float(params["bpm"]),
        root=str(params["root"]),
        scale=str(params["scale"]),
        lead_midi=str(params["lead_midi"]),
        lead_dur=str(params["lead_dur"]),
        form="arc",
        lead_synth="brass",
        bass_synth="dub",
        pad_synth="warmpad",
        repeat=True,
    ), params


class TestDropItLikeItsHotRegression:
    def test_theme_is_the_short_two_bar_phrase_from_the_issue(self):
        """Сама запись честно даёт 2 такта — на ней и завели issue."""
        _spec, params = _dropitli_spec()
        assert int(params["harmony"].bars) == 2

    def test_form_no_longer_loops_the_phrase_thirty_two_times(self):
        """Было: budget=round(64/2)=32 — фраза играла 32 раза подряд без
        единой вариации. Кап должен держать повтор в разумных пределах."""
        spec, _params = _dropitli_spec()
        plan = resolve_form(spec.form, spec.theme_bars)
        total_bars = sum(bars for _n, bars, _i in plan)
        repeats = total_bars // spec.theme_bars
        assert repeats <= MAX_THEME_REPEATS
        assert repeats < 32, "фраза снова повторяется почти без ограничения"

    def test_form_is_shorter_than_the_untouched_arc(self):
        """Короткая тема -> короткая форма, а не длинная форма той же фразы."""
        spec, _params = _dropitli_spec()
        plan = resolve_form(spec.form, spec.theme_bars)
        total_bars = sum(bars for _n, bars, _i in plan)
        default_total = sum(bars for _n, bars, _i in FORMS["arc"])
        assert total_bars < default_total / 2

    def test_reported_duration_is_honest_and_short(self):
        """Раньше тул сообщал «154 секунд» на 2 такта материала. Кап должен
        заметно сократить и реально сыгранную, и сообщаемую длительность —
        они обязаны совпадать (DJ ждёт перехода по этому числу)."""
        spec, _params = _dropitli_spec()
        duration_s = form_duration_seconds(spec.form, spec.bpm, spec.theme_bars)
        assert duration_s < 100.0, (
            f"форма всё ещё звучит {duration_s:.0f}с на 2-тактовой теме"
        )

    def test_summary_text_honestly_names_the_short_theme(self):
        """Текст результата тула обязан сказать «тема короткая», а не
        молчать о том, что в базе всего 2 такта материала."""
        spec, _params = _dropitli_spec()
        summary = form_summary(spec.form, spec.theme_bars)
        assert "тема в базе 2 такт" in summary
        assert "тема короткая" in summary

    def test_render_does_not_raise(self):
        """Дымовой прогон: капнутая форма всё ещё валидный Renardo-код."""
        spec, _params = _dropitli_spec()
        code = render(spec)
        assert "p2 >>" in code  # лид — тема dropitli
