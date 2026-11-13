"""Отбор темы поста и своё медиа к теме (issue #4702).

Разбор 13.11: сессия собрала 44 видимых факта, а черновик вышел один — и самый
скучный: мелкая правка голоса («fix(voice): keep rejected STT silent»). Причина
была не в сборе фактов, а в выборе темы: брали «первое в списке»
(studio_draft.KIND_PRIORITY + свежесть), поэтому свежий однострочный fix всегда
оказывался впереди. Прогонов робота по складу в фактах не было вовсе.

Здесь — регрессия на каждый пункт контракта:

  1. прогон робота побеждает две мелкие правки;
  2. свежесть сама по себе тему не выбирает — решают интерес и масштаб;
  3. показанная раньше тема опускается, но из ленты не уходит;
  4. ключ факта о прогоне устойчив и не путается с прогонами Actions;
  5. сборщик прогонов отдаёт факт с числами самого прогона и без его id;
  6. сборщик не падает, когда песочницы нет;
  7. карточка цифр темы идёт раньше общего таймлайна;
  8. карточка цифр собирается из чисел темы и безопасна для канала.

Плюс дыры, найденные на живой песочнице и на живых фактах 13.11:

  9. незавершённая правка («WIP …») не становится темой поста;
 10. отладочные повторы одного замера не плодят отдельные темы;
 11. в утверждении о прогоне есть и секунды, и минуты — иначе пост с
     «23 минуты симуляции» отклоняется проверкой чисел;
 12. у факта-прогона есть ключ, и карточка цифр строится по нему.
"""

import json
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import studio_facts as facts_mod  # noqa: E402
import studio_material as material  # noqa: E402
import studio_media as smedia  # noqa: E402
import studio_text as stext  # noqa: E402
import studio_topic as topic  # noqa: E402

RUN_ID = "depot4495_20261111T182530Z"
LONG_DIGITS = re.compile(r"\d{8,}")


# --------------------------------------------------------------------------- фикстуры


def _pr(number, title, paths=None, merged="2026-11-13T09:00:00Z", key=None):
    """Влитый PR как его видит сборщик фактов."""
    return {
        "kind": "merged_pr",
        "claim": f"PR #{number} влит в develop: {title}",
        "source": f"PR #{number} (https://github.com/krikz/rob_box_project/pull/{number})",
        "merged_at": merged,
        "paths": list(paths or []),
        "visible": True,
        "key": key or f"pr:{number}",
    }


def _robot_run(goals=7, sim_s=1406.23, snapshots=915, poses=13803, run_id=RUN_ID):
    """Прогон робота — факт нового вида."""
    return {
        "kind": "robot_run",
        "claim": f"Робот прошёл склад: {goals} целей маршрута, {int(sim_s)} с симуляции, {snapshots} снимков карты",
        "source": "журнал прогонов РОББОКС",
        "run_id": run_id,
        "run_label": f"прогон {run_id}",
        "numbers": {"goals": goals, "sim_s": int(sim_s), "map_snapshots": snapshots, "pose_rows": poses},
        "ended_at": "2026-11-11T18:56:32Z",
        "visible": True,
        "key": f"robot_run:{run_id}",
    }


class FakeGate:
    """Гейт с фейковой песочницей: сети нет, файлы отдаются из памяти."""

    def __init__(self, files: dict, dirs: dict):
        self.files = files
        self.dirs = dirs
        self.calls = 0

    def ls(self, path):
        self.calls += 1
        if path in self.dirs:
            return list(self.dirs[path]), ""
        return [], f"ls {path}: нет такого каталога"

    def fetch(self, remote, local):
        self.calls += 1
        if remote not in self.files:
            return f"scp {remote}: файла нет"
        local = Path(local)
        if local.is_file() and local.stat().st_size > 0:
            return ""
        local.parent.mkdir(parents=True, exist_ok=True)
        local.write_text(self.files[remote], encoding="utf-8")
        return ""


def _sandbox():
    """Фейковая песочница с одним прогоном: метаданные, маршрут и конец прогона."""
    base = material.HIL_OUT
    run_base = f"{base}/{RUN_ID}"
    route = "\n".join(
        [
            "08:24:17.873 GOAL_START M1 target=(4.5,3.0,0) sim=31.49 frame=94",
            "08:24:18.972 TRACK sim=31.60 x=0.000 y=0.000 yaw=0.0",
            "08:26:02.117 GOAL_START M2 target=(-6.0,4.0,0) sim=120.11 frame=210",
            "08:26:03.010 TRACK sim=120.22 x=-0.001 y=0.002 yaw=0.1",
            "08:28:44.551 GOAL_START M3 target=(-1.0,4.5,0) sim=281.90 frame=402",
            "08:28:45.220 TRACK sim=281.99 x=0.003 y=0.001 yaw=0.2",
            "08:31:02.900 GOAL_START M4 target=(-1.0,-1.0,0) sim=430.44 frame=640",
            "08:31:03.700 TRACK sim=430.55 x=0.000 y=0.000 yaw=0.0",
            "08:33:20.331 GOAL_START M5 target=(2.0,0.0,0) sim=560.02 frame=812",
            "08:33:21.010 TRACK sim=560.12 x=0.001 y=0.000 yaw=0.0",
            "08:35:40.882 GOAL_START M6 target=(3.5,1.5,0) sim=700.44 frame=980",
            "08:35:41.500 TRACK sim=700.55 x=0.000 y=0.001 yaw=0.0",
            "08:37:55.117 GOAL_START M7 target=(0.5,-2.0,0) sim=860.90 frame=1120",
            "08:37:55.900 TRACK sim=861.01 x=0.000 y=0.000 yaw=0.0",
        ]
    )
    files = {
        f"{run_base}/maprec/meta.json": json.dumps(
            {
                "map_topic": "/rtabmap/map",
                "map_snapshots": 915,
                "pose_rows": 13803,
                "sim_s": 1406.23,
                "wall_s": 1406.26,
                "rtf": 1.0,
            }
        ),
        f"{run_base}/route.log": route,
        f"{run_base}/run_end.txt": "2026-11-11T18:56:32Z\n",
    }
    dirs = {
        base: [RUN_ID],
        f"{run_base}/maprec": ["meta.json", "poses.csv"],
    }
    return FakeGate(files, dirs)


# Живая песочница 13.11: два настоящих прогона склада и пять отладочных
# повторов одного замера baseline_m1 (debug r3/r5/r7, diag, recur1).
# Время окончания — как на живой песочнице: по нему решается свежесть.
LIVE_RUNS = {
    "depot4495_20261111T182530Z": (10, 1406, 915, "2026-11-11T18:56:32Z"),
    "depot4682_20261111T191857Z": (10, 1561, 1193, "2026-11-11T19:18:57Z"),
    "depot4682_baseline_m1_20261111T224053Z": (1, 222, 153, "2026-11-11T22:40:53Z"),
    "depot4682_baseline_m1_debug_20261111T230136Z_r3": (1, 222, 160, "2026-11-11T23:01:36Z"),
    "depot4682_baseline_m1_debug_20261111T230136Z_r5": (1, 22, 1, "2026-11-11T23:01:36Z"),
    "depot4682_baseline_m1_debug_20261111T230136Z_r7": (1, 36, 8, "2026-11-11T23:01:36Z"),
    "depot4682_baseline_m1_diag_20261112T002656Z": (1, 149, 108, "2026-11-12T00:26:56Z"),
    "depot4682_baseline_m1_recur1_20261112T004740Z": (1, 150, 105, "2026-11-12T00:47:40Z"),
}


def _live_sandbox():
    """Фейковая песочница как живая 13.11: настоящие прогоны + отладочные повторы."""
    base = material.HIL_OUT
    files = {}
    dirs = {base: list(LIVE_RUNS)}
    for run_id, (goals, sim_s, snapshots, ended) in LIVE_RUNS.items():
        run_base = f"{base}/{run_id}"
        route = "\n".join(
            f"08:24:1{index}.873 GOAL_START M{index} target=(0,0,0) sim=1.0 frame={index}" for index in range(goals)
        )
        files[f"{run_base}/maprec/meta.json"] = json.dumps(
            {"map_snapshots": snapshots, "pose_rows": snapshots * 15, "sim_s": float(sim_s)}
        )
        files[f"{run_base}/route.log"] = route
        files[f"{run_base}/run_end.txt"] = f"{ended}\n"
        dirs[f"{run_base}/maprec"] = ["meta.json", "poses.csv"]
    return FakeGate(files, dirs)


# --------------------------------------------------------------------------- 1. тема по интересности


def test_robot_run_beats_two_small_fixes():
    """Регрессия #4702: прогон робота важнее двух мелких правок.

    Свежая однострочная правка голоса больше не должна выигрывать у прогона
    робота по складу — это и был разбор Шифу («один всратый пост»).
    """
    facts = [
        _pr(4700, "fix(voice): keep rejected STT silent", paths=["scripts/telegram/studio_text.py"]),
        _pr(4699, "fix(dialog): пустой ответ тула", paths=["scripts/telegram/studio_text.py"]),
        _robot_run(),
    ]

    assert topic.rank(facts)[0]["kind"] == "robot_run"


# --------------------------------------------------------------------------- 2. свежесть — не главное


def test_freshness_alone_does_not_win():
    """Свежая скучная правка проигрывает более старой, но крупной и robot-related."""
    facts = [
        _pr(4701, "обновление зависимостей", paths=["requirements.txt"], merged="2026-11-13T12:00:00Z"),
        _pr(
            4600,
            "навигация: карта склада и маршрут по целям",
            paths=[f"src/rob_box_nav/file{i}.py" for i in range(9)],
            merged="2026-11-10T12:00:00Z",
        ),
    ]

    ranked = topic.rank(facts)
    assert ranked[0]["key"] == "pr:4600"


# ------------------------------------------------- 3. показанное раньше — в конец


def test_shown_fact_is_demoted_but_stays_in_the_list():
    """Тему не предлагают дважды подряд, но из ленты факт не убирают."""
    paths = [f"src/rob_box_nav/file{i}.py" for i in range(5)]
    fresh = _pr(4601, "навигация: карта склада и маршрут", paths=paths)
    shown = dict(fresh, key="pr:4600", attempts=1)

    ranked = topic.rank([shown, fresh])

    assert [f["key"] for f in ranked] == ["pr:4601", "pr:4600"]
    assert len(ranked) == 2


# --------------------------------------------------------------------------- 4. ключ факта о прогоне


def test_fact_key_for_robot_run_is_stable():
    """Ключ прогона устойчив и не путается с прогонами Actions (префикс run:)."""
    fact = _robot_run()

    key = facts_mod.fact_key(fact)
    assert key.startswith("robot_run:")
    assert key == "robot_run:" + RUN_ID
    assert facts_mod.fact_key(dict(fact)) == key
    # Префикс прогона Actions занят другим видом факта.
    assert facts_mod.fact_key({"kind": "run", "source": "run_id 42219638788"}) == "run:42219638788"


# --------------------------------------------------------------------------- 5. сборщик прогонов


def test_robot_run_collector_uses_only_the_run_own_numbers(tmp_path):
    """Факт о прогоне: числа свои, идентификатора прогона в тексте нет.

    Идентификатор прогона — восемь цифр подряд: модель перепишет его в пост,
    и проверка отобьёт пост. Поэтому в claim/source/evidence его быть не должно.
    """
    facts = facts_mod.collect_robot_runs(gate=_sandbox(), cache_dir=tmp_path / "cache")

    assert len(facts) == 1
    fact = facts[0]
    assert fact["kind"] == "robot_run"
    assert fact["run_id"] == RUN_ID
    assert "7 целей маршрута" in fact["claim"]
    assert "1406 с симуляции" in fact["claim"]
    assert "915 снимков карты" in fact["claim"]

    shown = " ".join([fact["claim"], fact["source"], *(fact.get("evidence") or [])])
    assert RUN_ID not in shown
    assert not LONG_DIGITS.search(shown), shown


def test_robot_run_collector_survives_a_dead_sandbox(tmp_path):
    """Песочницы нет — сессия продолжает собирать остальные факты."""
    broken = FakeGate(files={}, dirs={})

    assert facts_mod.collect_robot_runs(gate=broken, cache_dir=tmp_path / "cache") == []


def test_robot_run_collector_survives_a_raising_gate(tmp_path):
    """Даже сломанный гейт не должен ронять сессию фактов."""

    class Broken:
        def ls(self, path):
            raise OSError("ssh: не отвечает")

        def fetch(self, remote, local):
            raise OSError("ssh: не отвечает")

    assert facts_mod.collect_robot_runs(gate=Broken(), cache_dir=tmp_path / "cache") == []


# --------------------------------------------------------------------------- 6. медиа по теме


def test_numbers_card_goes_before_the_generic_timeline():
    """Карточка цифр темы — раньше общего таймлайна влитых PR."""
    facts = [
        _robot_run(),
        _pr(4600, "навигация: карта склада", merged="2026-11-12T09:00:00Z"),
        _pr(4601, "голос: ответ тула", merged="2026-11-12T10:00:00Z"),
    ]
    post = {"id": "p1", "fact_keys": ["robot_run:" + RUN_ID]}

    variants = smedia.options(post, facts)

    assert "infographic:numbers" in variants
    assert "infographic:timeline" in variants
    assert variants.index("infographic:numbers") < variants.index("infographic:timeline")


def test_numbers_card_is_built_from_the_topic_own_numbers():
    """Карточка цифр — из чисел выбранной темы, и всё на ней — публичный текст."""
    facts = [_robot_run(), _pr(4600, "навигация: карта склада", merged="2026-11-12T09:00:00Z")]
    post = {"id": "p1", "fact_keys": ["robot_run:" + RUN_ID]}

    spec = smedia.numbers_spec(post, facts)

    assert spec is not None
    assert spec["template"] == "numbers"
    assert 2 <= len(spec["items"]) <= 4
    for item in spec["items"]:
        assert item["value"] != ""
        assert item["label"]
        assert item["source"]
    values = {item["value"] for item in spec["items"]}
    assert values == {7, 1406, 915}
    for text in smedia.visible_text(spec):
        assert smedia.is_public(text), text


def test_numbers_card_is_absent_when_the_topic_has_no_numbers():
    """Нет своих чисел — нет и карточки цифр: таймлайн остаётся последним."""
    facts = [_pr(4600, "навигация: карта склада", merged="2026-11-12T09:00:00Z")]
    post = {"id": "p1", "fact_keys": ["pr:4600"]}

    assert smedia.numbers_spec(post, facts) is None


# ------------------------------------------------- 9. незавершённая правка — не тема


def test_draft_pr_does_not_become_the_topic():
    """«WIP refactor(e2e): …» — черновик: он не должен обгонять живую тему.

    На живых фактах 13.11 такой PR набирал больше очков, чем правка про музыку
    в голосе: семь файлов против двух плюс лишний интерес от «e2e». Отсев WIP —
    на границе «что показывать» (ADR-0171, раздел 2а), а «e2e» — слово про
    стенд, а не про поведение робота, поэтому в интерес оно не входит.
    """
    wip = _pr(
        4646,
        "WIP refactor(e2e): tool_match без spoken= fallback",
        paths=[f"scripts/e2e/file{i}.py" for i in range(7)],
    )
    dialog = _pr(4679, "fix(dialog): настоящая причина отказа музыки в голосе", paths=["a.py", "b.py"])

    assert facts_mod.is_boring("WIP refactor(e2e): tool_match без spoken= fallback")
    assert not facts_mod.is_boring("fix(dialog): настоящая причина отказа музыки в голосе")
    assert "e2e" not in topic.INTEREST_WORDS
    assert topic.rank([wip, dialog])[0]["key"] == "pr:4679"


# --------------------------------------------- 10. отладочные повторы одного замера


def test_debug_variants_of_one_measurement_do_not_become_separate_facts(tmp_path):
    """Пять отладочных повторов baseline_m1 — это один замер, а не пять тем.

    Настоящие прогоны склада (depot4495 и depot4682, по 10 целей) остаются
    каждый своим фактом — это тот самый материал, ради которого всё затевалось.
    """
    facts = facts_mod.collect_robot_runs(gate=_live_sandbox(), cache_dir=tmp_path / "cache")

    run_ids = [fact["run_id"] for fact in facts]
    assert len(facts) == 3, run_ids
    assert "depot4495_20261111T182530Z" in run_ids
    assert "depot4682_20261111T191857Z" in run_ids
    # Из baseline_m1 остался один факт — и это обычный прогон, а не debug.
    baseline = [fact for fact in facts if "baseline_m1" in fact["run_id"]]
    assert len(baseline) == 1
    assert "debug" not in baseline[0]["run_id"]
    assert "diag" not in baseline[0]["run_id"]
    assert "recur" not in baseline[0]["run_id"]


def test_debug_variants_do_not_push_the_real_runs_out(tmp_path):
    """Настоящие прогоны склада выигрывают у отладочных повторов и по баллам."""
    facts = facts_mod.collect_robot_runs(gate=_live_sandbox(), cache_dir=tmp_path / "cache")

    ranked = topic.rank(facts)

    assert ranked[0]["run_id"] == "depot4682_20261111T191857Z"
    assert "10 целей маршрута" in ranked[0]["claim"]


# ------------------------------------------------------- 11. минуты в утверждении


def test_run_claim_allows_both_seconds_and_minutes(tmp_path):
    """В утверждении есть и секунды, и минуты: модель пишет «23 минуты» — и это проходит.

    Проверка чисел (studio_text.check_against_facts) разрешает только числа из
    claim: без минут пост «23 минуты симуляции» отклонялся бы, хотя по сути
    верен (1406 с — это 23 минуты).
    """
    facts = facts_mod.collect_robot_runs(gate=_sandbox(), cache_dir=tmp_path / "cache")

    claim = facts[0]["claim"]
    assert "1406 с симуляции" in claim
    assert "23 минуты" in claim

    ok = stext.check_against_facts("<p>Робот ездил по складу 23 минуты симуляции.</p>", facts)
    assert not [problem for problem in ok if "не найдено среди фактов" in problem], ok

    bad = stext.check_against_facts("<p>Робот ездил по складу 30 минут.</p>", facts)
    assert [problem for problem in bad if "30" in problem], bad


def test_minutes_plural_and_rounding():
    """Склонение и округление: 23 минуты, 22 минуты, 25 минут, 1406 с → 23."""
    assert facts_mod._sim_minutes(1406) == 23
    assert facts_mod._sim_minutes(1400) == 23
    assert facts_mod._sim_minutes(59) == 0

    assert facts_mod._plural(23, "минута", "минуты", "минут") == "минуты"
    assert facts_mod._plural(22, "минута", "минуты", "минут") == "минуты"
    assert facts_mod._plural(25, "минута", "минуты", "минут") == "минут"
    assert facts_mod._plural(21, "минута", "минуты", "минут") == "минута"
    assert facts_mod._plural(11, "минута", "минуты", "минут") == "минут"


# ------------------------------------------------ 12. ключ факта-прогона и карточка


def test_robot_run_fact_has_key_and_numbers_card_builds_by_it(tmp_path):
    """У факта-прогона есть ключ, и карточка цифр строится по нему."""
    facts = facts_mod.collect_robot_runs(gate=_sandbox(), cache_dir=tmp_path / "cache")

    fact = facts[0]
    assert fact["key"] == "robot_run:" + RUN_ID

    post = {"id": "p1", "fact_keys": [fact["key"]], "fact_dicts": facts}
    spec = smedia.numbers_spec(post, facts)

    assert spec is not None
    assert [item["value"] for item in spec["items"]] == [7, 1406, 915]
    assert smedia.chosen_fact(post, facts)["key"] == fact["key"]
