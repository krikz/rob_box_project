"""Студия: ролик показывает тот же прогон, о котором текст, и «самый длинный заезд» — правило кода.

Разбор 14.11 (issue #4711). Пост `p20261114-559` звал «10 целей маршрута, 1561 с»
(факт `robot_run:depot4682_20261111T191857Z`), а приложенное видео было смонтировано из
корневого прогона `depot` (7 целей, окно sim 0…219 с). Три причины:

* `build_top_view()` брал `material.TOP_RUN` вместо прогона из факта поста;
* правила «самый длинный заезд с максимальным покрытием» в коде не было;
* `Gate.fetch()` не перезаписывал непустой файл, поэтому кэш `route.log` остался от 07.11.

Плюс разрыв в самом счёте целей: факт (`_route_goals`) считает ВСЕ `GOAL_START` — их у
`depot4682` десять, а `mapping_window()` считал только `M*` — пять. Читатель видел «10 целей»,
в ролике было пять.

Эталонные числа посчитаны с песочницы (10.1.1.250) 14.11, они в тестах как константы.
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import studio_material as material  # noqa: E402
import studio_motion as motion  # noqa: E402
import studio_video as video  # noqa: E402

# --- эталон с песочницы ------------------------------------------------------

RUN_LONG = "depot4682_20261111T191857Z"
RUN_SHORT = "depot4495_20261111T182530Z"
RUN_DEBUG = "depot4682_baseline_m1_20261111T224053Z"

LONG = {"goals": 10, "sim_s": 1561, "map_snapshots": 1193, "pose_rows": 15340}
SHORT = {"goals": 10, "sim_s": 1406, "map_snapshots": 915, "pose_rows": 13803}

# Покрытие траектории из poses.csv (span_x, span_y, длина пути).
COVER_LONG = (22.92, 13.86, 161.25)
COVER_SHORT = (8.15, 10.86, 137.22)

CATALOG = [
    {"id": "depot", "label": "склад Depot: вид сверху и камера", "kind": "frames", "topic": ""},
    {"id": RUN_SHORT, "label": f"прогон {RUN_SHORT}", "kind": "poses", "topic": "", "role": ""},
    {"id": RUN_LONG, "label": f"прогон {RUN_LONG}", "kind": "poses", "topic": "", "role": ""},
    {"id": RUN_DEBUG, "label": f"прогон {RUN_DEBUG}", "kind": "poses", "topic": "", "role": ""},
]


def fact(run_id: str, numbers: dict) -> dict:
    """Факт о прогоне — как его собирает studio_facts."""
    return {
        "key": f"robot_run:{run_id}",
        "kind": "robot_run",
        "run_id": run_id,
        "numbers": numbers,
        "visible": True,
    }


def post_for(run_id: str) -> dict:
    return {"title": "Робот прошёл склад", "fact_keys": [f"robot_run:{run_id}"]}


# --- выбор прогона по факту поста --------------------------------------------


def test_run_from_post_fact_is_not_top_run():
    """Прогон для ролика — из факта поста, а не из захардкоженного TOP_RUN."""
    facts = [fact(RUN_LONG, LONG)]
    chosen = video.run_of_post(post_for(RUN_LONG), facts)
    assert chosen == RUN_LONG


def test_run_from_post_fact_wins_over_catalog_order():
    """Порядок в каталоге не влияет: берётся прогон из факта."""
    facts = [fact(RUN_SHORT, SHORT), fact(RUN_LONG, LONG)]
    post = {"title": "t", "fact_keys": [f"robot_run:{RUN_LONG}"]}
    assert video.run_of_post(post, facts) == RUN_LONG


def test_no_fact_no_run():
    """Без факта о прогоне — нечего выбирать (дальше решает каталог)."""
    assert video.run_of_post({"title": "t", "fact_keys": []}, []) == ""


def test_plan_uses_run_from_fact_not_top_run():
    """План ролика по теме-факту содержит прогон факта, а не 'depot'."""
    facts = [fact(RUN_LONG, LONG)]
    post = post_for(RUN_LONG)
    plan = video.plan_by_code(post, "вид сверху", CATALOG, ["run_view", "top_view"], facts)
    assert plan is not None
    assert plan["runs"] == [RUN_LONG]
    assert material.TOP_RUN not in plan["runs"]


def test_plan_for_poses_run_picks_run_view():
    """У прогона без кадров камер ролик строится по позам (run_view), а не по чужому top_view."""
    facts = [fact(RUN_LONG, LONG)]
    options = video.feasible(post_for(RUN_LONG), facts, CATALOG)
    assert "run_view" in options
    plan = video.plan_by_code(post_for(RUN_LONG), "вид сверху", CATALOG, options, facts)
    assert plan["template"] == "run_view"
    assert plan["runs"] == [RUN_LONG]


def test_feasible_offers_run_view_for_own_run():
    """Шаблон ролика по позам предлагается, когда у темы есть прогон с позами."""
    facts = [fact(RUN_LONG, LONG)]
    assert "run_view" in video.feasible(post_for(RUN_LONG), facts, CATALOG)
    # Без факта о прогоне — такого шаблона нет.
    assert "run_view" not in video.feasible({"title": "t", "fact_keys": []}, [], CATALOG)


def test_plan_without_fact_keeps_general_warehouse_view():
    """Факта о прогоне нет — текст не про заезд, и остаётся складской вид из каталога.

    Это не «чужой прогон молча»: пост ни о каком прогоне не говорит, а вид сверху
    с камерами в каталоге один. Проверка чужого прогона — когда факт ЕСТЬ.
    """
    plan = video.plan_by_code({"title": "t", "fact_keys": []}, "вид сверху", CATALOG, ["top_view"])
    if plan is not None:
        assert plan["runs"] == [material.TOP_RUN]


def test_own_run_never_replaced_by_warehouse_view():
    """Есть факт о прогоне — план не берёт складской вид вместо прогона факта."""
    facts = [fact(RUN_LONG, LONG)]
    plan = video.plan_by_code(post_for(RUN_LONG), "вид сверху", CATALOG, ["run_view", "top_view"], facts)
    assert plan["runs"] == [RUN_LONG]
    assert material.TOP_RUN not in plan["runs"]


# --- «самый длинный заезд с максимальным покрытием» ---------------------------


def test_longest_run_by_coverage():
    """Покрытие больше — заезд длиннее: depot4682 побеждает depot4495."""
    runs = [
        {"id": RUN_SHORT, "cover": COVER_SHORT, "goals": 10},
        {"id": RUN_LONG, "cover": COVER_LONG, "goals": 10},
    ]
    best = material.best_run(runs)
    assert best == RUN_LONG


def test_longest_run_skips_debug():
    """Отладочный прогон (одна цель, покрытие ничтожное) в выдачу не попадает."""
    runs = [
        {"id": RUN_DEBUG, "cover": (0.4, 0.3, 1.2), "goals": 1},
        {"id": RUN_LONG, "cover": COVER_LONG, "goals": 10},
    ]
    assert material.best_run(runs) == RUN_LONG


def test_longest_run_empty():
    assert material.best_run([]) == ""


def test_run_measure_rejects_empty():
    """Прогон без измерений — не материал."""
    assert material.measure_run(None) is None
    assert material.measure_run([]) is None


# --- «самый длинный заезд» по данным фактов ----------------------------------


def test_longest_run_from_facts():
    """Самый длинный заезд — по числам фактов: дольше по времени и больше снимков карты."""
    facts = [
        fact(RUN_SHORT, SHORT),
        fact(RUN_LONG, LONG),
        fact(RUN_DEBUG, {"goals": 1, "sim_s": 222, "map_snapshots": 153, "pose_rows": 2197}),
    ]
    assert material.longest_run(facts) == RUN_LONG


def test_longest_run_skips_debug_fact():
    """Отладочный прогон (одна цель, 222 с) не выигрывает даже по времени."""
    facts = [
        fact(RUN_DEBUG, {"goals": 1, "sim_s": 222, "map_snapshots": 153, "pose_rows": 2197}),
        fact(RUN_SHORT, SHORT),
    ]
    assert material.longest_run(facts) == RUN_SHORT


def test_longest_run_empty_facts():
    assert material.longest_run([]) == ""


def test_longest_run_ignores_non_run_facts():
    """Факты не о прогоне в выдачу не попадают."""
    facts = [{"key": "merged_pr:1", "kind": "merged_pr", "numbers": {"sim_s": 9999}}]
    assert material.longest_run(facts) == ""


def test_longest_run_from_real_numbers():
    """Эталон с песочницы: depot4682 длиннее depot4495 по времени и по карте."""
    facts = [fact(RUN_SHORT, SHORT), fact(RUN_LONG, LONG)]
    assert material.longest_run(facts) == RUN_LONG
    assert LONG["sim_s"] > SHORT["sim_s"]
    assert LONG["map_snapshots"] > SHORT["map_snapshots"]


def test_request_for_longest_run_picks_longest_fact():
    """Просьба «самый длинный заезд» — правило кода: выбирается прогон по данным."""
    facts = [fact(RUN_SHORT, SHORT), fact(RUN_LONG, LONG)]
    chosen = video.run_for_request("возьми самый длинный заезд с максимальным покрытием", facts, "")
    assert chosen == RUN_LONG


def test_request_without_longest_words_keeps_post_fact():
    """Обычная просьба — прогон темы, а не «самый длинный»."""
    facts = [fact(RUN_SHORT, SHORT), fact(RUN_LONG, LONG)]
    chosen = video.run_for_request("сделай видео", facts, RUN_SHORT)
    assert chosen == RUN_SHORT


# --- материал, который уже уходил в канал, повторно не предлагается ------------


def state_with_published(runs: list, status: str = "approved") -> dict:
    """Состояние студии: один пост в статусе status с такими прогонами."""
    return {"posts": {"i20261106-1-post": {"status": status, "last_motion": {"runs": runs}}}}


def test_published_runs_collected():
    """Прогоны из опубликованного поста — «использованные»."""
    used = material.published_runs(state_with_published(["shisyun_s1", "shisyun_s2"]))
    assert used == {"shisyun_s1", "shisyun_s2"}


def test_proposed_post_does_not_count_as_published():
    """Черновик (proposed) ещё не уходил в канал — его материал можно использовать."""
    used = material.published_runs(state_with_published(["shisyun_s1"], status="proposed"))
    assert used == set()


def test_published_runs_empty_state():
    assert material.published_runs({}) == set()


def test_published_runs_ignores_rejected():
    """Отклонённый пост в канал не уходил."""
    used = material.published_runs(state_with_published(["shisyun_s1"], status="rejected"))
    assert used == set()


def test_catalog_excludes_published_runs():
    """Прогоны, чей материал уже в канале, из каталога убираются.

    Разбор 14.11 (issue #4711): пост 09.11 про PR #4555 получил ролик с
    прогонами shisyun_s1/s2 — материал уже уходил в канал 06.11, и файл был
    байт-идентичен тому, что опубликован. Студия брала первые два прогона из
    каталога, не глядя ни на тему, ни на то, что это уже показывали.
    """
    catalog = [
        {"id": "shisyun_s1", "kind": "poses"},
        {"id": "shisyun_s2", "kind": "poses"},
        {"id": RUN_LONG, "kind": "poses"},
    ]
    used = material.published_runs(state_with_published(["shisyun_s1", "shisyun_s2"]))
    fresh = material.fresh_catalog(catalog, used)
    assert [item["id"] for item in fresh] == [RUN_LONG]


def test_fresh_catalog_keeps_all_when_nothing_published():
    catalog = [{"id": "shisyun_s1", "kind": "poses"}, {"id": "shisyun_s2", "kind": "poses"}]
    assert material.fresh_catalog(catalog, set()) == catalog


# --- «ещё раз» — это не всегда «пришли то же самое» ----------------------------


def test_resend_request_wants_same_file():
    """Прямая просьба прислать ещё раз — тот же файл."""
    assert motion.wants_resend("пришли ещё раз")
    assert motion.wants_resend("покажи снова")
    assert motion.wants_resend("пришли повторно")


def test_repeated_search_is_not_resend():
    """«Ещё раз говорю найти самый длинный» — это повтор ПОИСКА, а не просьба прислать то же.

    Разбор 14.11 (issue #4711): Шифу написал «Это не самый длинный заезд, ещё раз
    говорю найти самый длинный», и студия прочитала «ещё раз» как «пришли тот же
    ролик» — и прислала старое видео. Слова поиска отменяют повторную отправку.
    """
    assert not motion.wants_resend("Это не самый длинный заезд, ещё раз говорю найти самый длинный")
    assert not motion.wants_resend("сделай заново, найди самый длинный заезд")


def test_resend_request_without_new_material_words():
    """Чистая просьба «пришли ещё раз» — повторная отправка."""
    assert motion.wants_resend("пришли ещё раз")
    assert motion.wants_resend("пришли, ещё раз")
    assert not motion.wants_resend("")


# --- единый счёт целей --------------------------------------------------------


def test_goals_count_all_goals():
    """Цели считаются все (M* и R*): факт и окно монтажа должны сойтись."""
    route = "\n".join(
        [
            "08:24:17.873 GOAL_START M1 target=(1,2,0) sim=10.0 frame=94",
            "08:24:18.873 GOAL_END M1 sim=20.0 frame=140",
            "08:24:19.873 GOAL_START R1 target=(3,4,0) sim=30.0 frame=200",
            "08:24:20.873 GOAL_END R1 sim=40.0 frame=260",
        ]
    )
    assert material.route_goals(route) == 2
    assert video.mapping_window(route) == (10.0, 40.0)


def test_window_includes_r_goals():
    """Окно монтажа — по всем целям, включая R*: иначе ролик короче текста."""
    route = "\n".join(
        [
            "08:24:17.873 GOAL_START M1 target=(1,2,0) sim=10.0 frame=94",
            "08:24:18.873 GOAL_END M1 sim=20.0 frame=140",
            "08:24:19.873 GOAL_START R1 target=(3,4,0) sim=900.0 frame=1200",
            "08:24:20.873 GOAL_END R1 sim=1000.0 frame=1400",
        ]
    )
    assert video.mapping_window(route) == (10.0, 1000.0)


def test_window_none_when_no_end():
    """Прерванный прогон (нет GOAL_END) окна не даёт — материалом не является."""
    route = "08:24:17.873 GOAL_START M1 target=(1,2,0) sim=10.0 frame=94"
    assert video.mapping_window(route) is None


# --- кэш материала освежается ------------------------------------------------


def test_fetch_refreshes_stale_file(tmp_path):
    """Старый файл в кэше перекачивается, свежий — нет."""
    calls = []

    class Gate:
        calls = 0
        key = "test-key"
        host = "test-host"

        def _run(self, cmd):
            calls.append(cmd)
            part = tmp_path / "route.log.part"
            part.write_text("свежий\n", encoding="utf-8")
            return True, "", ""

    gate = Gate()
    local = tmp_path / "route.log"
    local.write_text("старый\n", encoding="utf-8")
    # файл старше порога — должен обновиться
    import os
    import time

    old = time.time() - (material.CACHE_MAX_AGE_S + 600)
    os.utime(local, (old, old))
    reason = material.Gate.fetch(gate, f"{material.HIL_OUT}/route.log", local)
    assert reason == ""
    assert local.read_text(encoding="utf-8") == "свежий\n"
    assert calls, "файл обновлялся"


def test_fetch_keeps_fresh_file(tmp_path):
    """Свежий файл в кэше не перекачивается — лишних scp не делаем."""
    calls = []

    class Gate:
        calls = 0
        key = "test-key"
        host = "test-host"

        def _run(self, cmd):
            calls.append(cmd)
            return True, "", ""

    gate = Gate()
    local = tmp_path / "route.log"
    local.write_text("свежий\n", encoding="utf-8")
    reason = material.Gate.fetch(gate, f"{material.HIL_OUT}/route.log", local)
    assert reason == ""
    assert local.read_text(encoding="utf-8") == "свежий\n"
    assert not calls, "свежий файл не перекачивали"


def test_cache_max_age_is_sane():
    """Порог освежения — не ноль и не сутки: материал не должен протухать на неделю."""
    assert 0 < material.CACHE_MAX_AGE_S <= 24 * 3600
