"""Сеты темы × сид × трек, общие для ``test_pad_figures`` и ``test_lead_bass`` (#3940).

Оба файла строили один и тот же каталог — 4 темы из таблицы ``knowledge.THEME_TIMBRE`` × 30 сидов × 3 трека с историей
сета, — каждый у себя (≈ 12 с + ≈ 8 с на Windows). Теперь он строится один раз за сессию (фикстура ``club_theme_sets``
в ``conftest.py``). Тема не из таблицы семью берёт по сиду (``pick_timbre``, #3460), а ``test_lead_bass`` фиксирует её
на ``default_timbre`` — для неё каталоги разные, их строит каждый файл у себя.

Функция верхнего уровня модуля (для ``parallel.pmap``); состояния на диске и глобального нет — в ``pytest -n`` каждый
воркер строит каталог сам."""

from __future__ import annotations

from rob_box_music.arrange.compose import compose
from rob_box_music.diversity import track_history
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

#: Темы из таблицы тембров: семья сета у них одна на все сиды, поэтому каталог не зависит от файла-потребителя.
TABLE_THEMES = ("космос", "киберпанк", "детский праздник", "калинка")
SEEDS = range(30)
TRACKS = 3


def set_tracks(case):
    """Треки одного сета (тема, сид) с историей сета; сеты независимы."""
    theme, seed = case
    plan = seeded_plan(seeded_profile(theme), seed, set_id=f"p{seed}")
    history: list = []
    tracks = []
    for no in range(1, TRACKS + 1):
        track = compose(plan, no, history=history)
        history.insert(0, track_history(track, plan.set_id))
        tracks.append(track)
    return tracks
