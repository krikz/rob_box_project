"""План состава формы (issue #2978): слои входят и уходят, не всей группой.

Статьи по аранжировке (инвентаризация issue #2978, 24.09.2026:
samesound.ru/write/81, moscowmusicschool.ru, danalex.livejournal.com/1650,
studioday.ru «15 советов») сходятся в одном: контраст секций строится
СОСТАВОМ, не только громкостью — тихая секция играет минимум партий, пик
играет максимум, а роли входят и уходят по одной, не разом. Живые треки
24.09 (гимн, Терминатор, DJ-сеты) звучали одним и тем же набором слоёв от
intro до outro — громче/тише, но не гуще/реже.

Этот тест не проверяет ни одну конкретную композицию (никаких имён песен и
ключей архива здесь нет — только таблица :data:`FORMS`, общая для любой
темы) — он гоняет :func:`core.arranger.form_role_plan_violations` по
КАЖДОЙ форме модуля, так что правка формы, ломающая правило, ловится тут,
а не на живом прогоне.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.arranger import (
    FORMS,
    MAX_ROLE_COUNT_STEP,
    MAX_SIMULTANEOUS_ROLES,
    SOLO_LEAD_INTENSITY,
    SOLO_MAX_ACCOMPANIMENT_ROLES,
    _section_role_set,
    form_role_plan_violations,
)


@pytest.mark.parametrize("form", sorted(FORMS))
def test_form_has_no_role_plan_violations(form):
    violations = form_role_plan_violations(FORMS[form])
    assert violations == [], f"форма {form!r}: {violations}"


@pytest.mark.parametrize("form", sorted(FORMS))
def test_no_section_exceeds_the_simultaneous_role_cap(form):
    for name, _bars, intensities in FORMS[form]:
        count = len(_section_role_set(intensities))
        assert count <= MAX_SIMULTANEOUS_ROLES, (
            f"{form}/{name}: {count} ролей одновременно"
        )


@pytest.mark.parametrize("form", sorted(FORMS))
def test_role_count_changes_gradually_between_sections(form):
    plan = FORMS[form]
    counts = [len(_section_role_set(i)) for _n, _b, i in plan]
    for (name_a, _ba, _ia), (name_b, _bb, _ib), count_a, count_b in zip(
        plan, plan[1:], counts, counts[1:]
    ):
        assert abs(count_b - count_a) <= MAX_ROLE_COUNT_STEP, (
            f"{form}: {name_a}({count_a})->{name_b}({count_b})"
        )


@pytest.mark.parametrize("form", sorted(FORMS))
def test_form_has_at_least_one_section_where_theme_solos(form):
    """Хотя бы одна секция, где тема солирует над минимальным аккомпанементом."""
    plan = FORMS[form]
    found = False
    for _name, _bars, intensities in plan:
        lead = float(intensities.get("lead", 0.0))
        if lead < SOLO_LEAD_INTENSITY:
            continue
        accompaniment = _section_role_set(intensities) - {"lead"}
        if len(accompaniment) <= SOLO_MAX_ACCOMPANIMENT_ROLES:
            found = True
            break
    assert found, f"форма {form!r}: нет соло-секции темы"


def test_peak_like_section_is_a_maximum_of_its_own_form():
    """«Пик — максимум» (issue #2978): секция с именем из ``FX_BOUNDARY_SECTIONS``,
    обозначающая кульминацию/дроп формы, не должна быть реже, чем предыдущая
    по счёту ролей — она обязана быть локальным максимумом состава.

    Проверяем только формы, где такая секция есть (``peak``/``drop``/``drop2``
    — имена см. :data:`FORMS`); форма без явной кульминации (``ambient``)
    не участвует.
    """
    climax_names = {"peak", "drop", "drop2"}
    for form, plan in FORMS.items():
        counts = [len(_section_role_set(i)) for _n, _b, i in plan]
        names = [name for name, _b, _i in plan]
        top = max(counts)
        climax_counts = [
            count for name, count in zip(names, counts) if name in climax_names
        ]
        if not climax_counts:
            continue
        assert max(climax_counts) == top, (
            f"форма {form!r}: кульминация не достигает максимума состава формы "
            f"({climax_counts} vs {top})"
        )
