"""Общая настройка тестов rob_box_music.

БЮДЖЕТ ПАКЕТА 150 с (CI: таймаут 240 с, #3539): тяжёлое (сиды × темы × треки, рендер) — через ``parallel.pmap``
по независимым единицам (сет, сид, комбинация) и module-фикстуры; лишнее внутри цикла (то, что не зависит от
переменной цикла) выноси наружу; перед PR смотри ``pytest --durations=30``.
"""

import pytest

from parallel import pmap


@pytest.fixture(scope="session")
def club_theme_sets():
    """{тема из таблицы тембров: [треки 30 сетов по 3 трека, с историей сета]} — один раз на сессию (на воркер при
    ``pytest -n``) для ``test_pad_figures`` и ``test_lead_bass`` (#3940). Сеты считаются параллельно (#3504, #3539: по
    сету, а не по теме). Тест файла дополняет каталог своей темой вне таблицы."""
    from shared_sets import SEEDS, TABLE_THEMES, set_tracks

    cases = [(theme, seed) for theme in TABLE_THEMES for seed in SEEDS]
    built = pmap(set_tracks, cases, chunksize=5)
    return {theme: [t for (th, _s), tracks in zip(cases, built) if th == theme for t in tracks]
            for theme in TABLE_THEMES}
