"""
test_issue_5031_first_mode_mid_series.py — first посреди серии не стирает каст.

Инцидент 21.11: часть 2 марафона запустили отдельным раннером с
``--e2e-marathon-phase first``. first — это переход e2e OFF->ON, а он стирает
обе e2e-БД (``speakers.e2e.db``, ``voice_memory.e2e.db``, issue #2890).
Каст и факты прошлой части пропали, 11 шагов покраснели ложно.

Гард ``refuse_first_mid_series`` стоит в ``e2e_voice_test.sh`` ДО часов
робота (``E2E_RUN_BEFORE``) и до ``enter_e2e_mode``:

  * ``first`` + непустой ``--prior-steps`` (или часть > 1, или профили уже
    в ``speakers.e2e.db``) — exit != 0 и текст про стирание, базы не тронуты;
  * ``middle`` — базы не трогает (md5 до и после совпадает);
  * ``first`` с нуля — отказ не срабатывает, стирание #2890 на месте.

Офлайн: временные файлы БД, робот не нужен. Стирание #2890 проверяется
настоящим ``enter_e2e_mode`` на фейковом роботе (как в #2890/#3972).

Run:
  python -m pytest tests/unit/e2e_scripts/test_issue_5031_first_mode_mid_series.py -v --no-cov
"""

from __future__ import annotations

import hashlib
import os
import sqlite3
import subprocess
import sys
from pathlib import Path

import pytest

from tests.unit.e2e_scripts.test_issue_2890_act_isolation import (
    BASH,
    NIGHT,
    REPO_ROOT,
    _bash_path,
    _copy_real,
    _require_bash_sees,
    _run_isolation,
    bash_required,
)

E2E_SCRIPT = REPO_ROOT / "scripts" / "ci" / "e2e" / "e2e_voice_test.sh"
ACT2 = NIGHT / "night_marathon_act2_acquaintance_v1.json"


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _seed_db(path: Path, rows: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(path)
    con.execute("create table speakers (speaker_id text primary key, name text, created_at real)")
    for i in range(rows):
        con.execute("insert into speakers values (?, ?, ?)", (f"id{i}", f"Саша{i}", 1.0))
    con.commit()
    con.close()


FAKE_ROBOT_SSH = Path(__file__).resolve().parent / "fake_robot_ssh.sh"


def _run_harness(tmp_path: Path, *args: str, env_extra=None, scenario: bool = True,
                 robot: str = "stub", stop_after_isolate: bool = False,
                 container_db: Path | None = None, stop_after_guard: bool = False):
    """Настоящий скрипт. ``robot="stub"`` — ssh печатает SSH_STUB_CALLED и
    падает (отказ #5031 обязан случиться раньше любого ssh). ``robot="fake"`` —
    общий фейк тестов харнесса: переход e2e OFF->ON удаляет файлы из
    FAKE_E2E_DBS, как узел стирает e2e-БД (#2890). ``sqlite3`` внутри
    ``docker exec`` фейка — настоящий, поэтому профиль каста можно положить
    в ``/data/speakers.e2e.db`` только через E2E_SPEAKERS_DB (локальный файл)."""
    _require_bash_sees(tmp_path)
    scn = tmp_path / "scenario.json"
    if scenario:
        scn.write_text(ACT2.read_text(encoding="utf-8"), encoding="utf-8")
    if robot == "stub":
        ssh = tmp_path / "ssh_stub.sh"
        ssh.write_text("#!/bin/bash\necho SSH_STUB_CALLED \"$@\" >&2\nexit 97\n", encoding="utf-8")
        ssh.chmod(0o755)
    else:
        ssh = FAKE_ROBOT_SSH
    script = E2E_SCRIPT
    mutated = stop_after_isolate or stop_after_guard or container_db is not None
    if mutated:
        # Доказательство про md5 нужно на границе isolate_act_start: именно там
        # middle решает «не стирать». Дальше идёт сценарий (музыка, прогрев) —
        # роботу он не нужен, а фейк на незнакомом вызове честно падает.
        # GATE-1 (acceptance) стоит РАНЬШЕ изоляции и честно валит прогон без
        # acceptance.json. Для доказательства про md5 он не нужен: решение
        # «стирать / не стирать» принимает isolate_act_start, и оно уже
        # отработало бы к этому месту, если бы мы его звали. Зову его здесь,
        # на том же сценарии, и выхожу — до музыки и прогрева.
        text = E2E_SCRIPT.read_text(encoding="utf-8")
        if container_db is not None:
            text = text.replace("/data/speakers.e2e.db", container_db.as_posix())
        if stop_after_guard:
            # Гард уже решил. Дальше часы робота и сценарий — к числу профилей
            # они отношения не имеют, а фейк на docker logs честно падает.
            text = text.replace(
                '\nrefuse_first_mid_series\n',
                '\nrefuse_first_mid_series\necho GUARD_DONE\nexit 0\n',
                1,
            )
        cut = text.replace(
            'if [ -n "$SCENARIO_FILE" ] && [ -z "$ACCEPTANCE_FILE" ] && [ "$ACCEPTANCE_SKIP" != "1" ]; then',
            'if [ -n "$SCENARIO_FILE" ]; then\n'
            '    isolate_act_start "$SCENARIO_FILE"\n'
            '    echo ISOLATE_DONE\n'
            '    exit 0\n'
            'fi\n'
            'if [ -n "$SCENARIO_FILE" ] && [ -z "$ACCEPTANCE_FILE" ] && [ "$ACCEPTANCE_SKIP" != "1" ]; then',
            1,
        )
        # Рядом с e2e_voice_lib.sh: скрипт source'ит соседей по BASH_SOURCE.
        script = E2E_SCRIPT.parent / f".e2e_voice_test_cut_{os.getpid()}.sh"
        script.write_text(cut, encoding="utf-8")
    env = {
        "PATH": os.environ.get("PATH", ""),
        "PYTHONIOENCODING": "utf-8",
        "ROBOT_SSH_OVERRIDE": str(ssh),
        "E2E_PRESYNTH": "0",
        "E2E_RECORDING": "0",
        "E2E_ACT_WARMUP_SEC": "0",
        "HOME": str(tmp_path),
    }
    if robot == "fake":
        env["FAKE_ROBOT_STATE"] = str(tmp_path / "fake_robot")
    if env_extra:
        env.update(env_extra)
    cmd = [BASH, str(script)]
    if scenario:
        cmd += ["--scenario", str(scn)]
    cmd += list(args)
    try:
        return subprocess.run(
            cmd, capture_output=True, text=True, encoding="utf-8", timeout=90, env=env,
        )
    finally:
        if mutated:
            script.unlink(missing_ok=True)


@bash_required
def test_first_with_prior_steps_refuses_and_names_the_wipe(tmp_path):
    """Сам инцидент: first + итоги прошлых актов — явная ошибка, не стирание."""
    speakers = tmp_path / "speakers.e2e.db"
    memory = tmp_path / "voice_memory.e2e.db"
    _seed_db(speakers, 2)
    memory.write_bytes(b"facts-of-the-previous-part")
    before = (_md5(speakers), _md5(memory))

    proc = _run_harness(
        tmp_path, "--e2e-marathon-phase", "first", "--prior-steps", "n201=OK,n211=OK",
        env_extra={"E2E_SPEAKERS_DB": str(speakers)},
    )

    assert proc.returncode != 0, proc.stdout
    assert proc.returncode == 2
    assert "E2E_FATAL" in proc.stderr
    assert "стирает каст" in proc.stderr
    assert "режимом middle" in proc.stderr
    assert "удалите e2e-БД осознанно" in proc.stderr
    assert "#5031" in proc.stderr
    assert "SSH_STUB_CALLED" not in proc.stderr, "отказ дошёл до робота — базы могли стереться"
    assert (_md5(speakers), _md5(memory)) == before, "first посреди серии тронул e2e-БД"


@bash_required
def test_first_on_part_above_one_refuses(tmp_path):
    speakers = tmp_path / "speakers.e2e.db"
    _seed_db(speakers, 1)
    before = _md5(speakers)

    proc = _run_harness(
        tmp_path, "--e2e-marathon-phase", "first", "--e2e-marathon-part", "2",
        env_extra={"E2E_SPEAKERS_DB": str(speakers)},
    )

    assert proc.returncode == 2
    assert "часть 2" in proc.stderr and "стирает каст" in proc.stderr
    assert _md5(speakers) == before


@bash_required
def test_first_with_profiles_already_in_db_refuses(tmp_path):
    """Каст в БД без признаков продолжения — остаток оборванной серии (#5077).

    До #5077 гард отказывал и здесь: профилей > 0 хватало. Инцидент 22.11
    (run 44973642247) показал, что именно эта ветка роняет следующий марафон
    после оборванного: таймер владельца снял режим, файлы остались, а
    признаков продолжения (--prior-steps, часть > 1) у нового прогона нет.
    Теперь остаток убирает маркер серии ДО гарда, и гард видит чистый старт.
    Живой каст гард по-прежнему защищает — но по признакам продолжения,
    не по числу профилей (см. два теста выше)."""
    speakers = tmp_path / "speakers.e2e.db"
    _seed_db(speakers, 2)

    proc = _run_harness(
        tmp_path, "--e2e-marathon-phase", "first",
        robot="stub", stop_after_guard=True,
        env_extra={"E2E_SPEAKERS_DB": str(speakers)},
    )

    assert "E2E_SERIES delete" in proc.stdout, proc.stderr
    assert not speakers.exists(), "остаток оборванной серии остался в e2e-БД (issue #5077)"
    assert "GUARD_DONE" in proc.stdout, proc.stderr
    assert proc.returncode == 0, proc.stderr


@bash_required
def test_profiles_are_counted_inside_the_container_read_only(tmp_path):
    """Без E2E_SPEAKERS_DB гард считает профили в контейнере, не на хосте.

    21.11 каст жил в /data/speakers.e2e.db контейнера voice-assistant. Файл на
    хосте харнесса гард видеть не должен — и читать обязан только на чтение:
    фейк на OFF->ON удалил бы FAKE_E2E_DBS, если бы SELECT превратился в wipe.

    С #5077 ветка «только профили» закрыта маркером серии: first без признаков
    продолжения и без живого маркера удаляет e2e-БД ДО гарда, и гард видит
    чистый старт. Поэтому здесь серия живая (маркер свежий) — иначе удаление
    #5077 унесло бы файл раньше, чем гард его прочитает.
    """
    speakers = tmp_path / "speakers.e2e.db"
    _seed_db(speakers, 3)
    before = _md5(speakers)
    data = tmp_path / "data"
    data.mkdir()
    # docker exec фейка исполняет команду локальным bash, но /data контейнера
    # подменяем: sqlite3 откроет файл по пути, который видит «контейнер».
    container_db = data / "speakers.e2e.db"
    container_db.write_bytes(speakers.read_bytes())

    marker = tmp_path / "series.id"
    # «opened» — серия прошла гард. Без второй строки first видит оборванный
    # прогон той же серии и стирает хвост до гарда (issue #5077), и до чтения
    # профилей дело не доходит.
    marker.write_text("night_live\nopened\n", encoding="utf-8")
    proc = _run_harness(
        tmp_path, "--e2e-marathon-phase", "first",
        robot="fake", container_db=container_db, stop_after_guard=True,
        env_extra={
            "FAKE_E2E_DBS": str(speakers),
            "E2E_SERIES_MARKER": str(marker),
            "E2E_SERIES_ID": "night_live",
        },
    )

    assert proc.returncode == 2, proc.stderr
    assert "профил" in proc.stderr and "стирает каст" in proc.stderr
    assert _md5(speakers) == before
    assert _md5(container_db) == before, "чтение профилей записало в e2e-БД"


@bash_required
def test_middle_keeps_both_e2e_dbs_byte_for_byte(tmp_path):
    """middle базы не трогает: md5 обеих до и после прогона режима совпадает.

    Фейк-робот на переходе OFF->ON удаляет обе БД (как узел, #2890) — если
    бы middle пошёл через enter_e2e_mode, md5 разошлись бы."""
    speakers = tmp_path / "speakers.e2e.db"
    memory = tmp_path / "voice_memory.e2e.db"
    _seed_db(speakers, 2)
    memory.write_bytes(b"voice-memory-cast-of-part-1")
    before = (_md5(speakers), _md5(memory))

    state = tmp_path / "fake_robot"
    state.mkdir()
    # Режим уже включён актом first — иначе middle честно падает на #3972,
    # не дойдя до решения «не стирать».
    (state / "param__speaker_id_nodee2e_mode").write_text("true\n", encoding="utf-8")
    proc = _run_harness(
        tmp_path, "--e2e-marathon-phase", "middle", "--prior-steps", "n201=OK",
        robot="fake", stop_after_isolate=True,
        env_extra={
            "E2E_SPEAKERS_DB": str(speakers),
            "FAKE_E2E_DBS": f"{speakers} {memory}",
            "FAKE_ROBOT_STATE": str(state),
        },
    )

    assert "ISOLATE_DONE" in proc.stdout, proc.stderr
    assert "каст прошлых актов на месте" in proc.stdout, proc.stderr
    assert "E2E_FATAL" not in proc.stderr, proc.stderr
    assert (_md5(speakers), _md5(memory)) == before, "middle стёр e2e-БД — каст прошлой части пропал"


@bash_required
def test_first_from_scratch_still_wipes(tmp_path):
    """#2890 не сломан: серия с нуля (нет prior-steps, часть 1, БД пуста)
    по-прежнему стирает e2e-БД на старте."""
    speakers = tmp_path / "speakers.e2e.db"
    _seed_db(speakers, 0)

    proc = _run_harness(
        tmp_path, "--e2e-marathon-phase", "first", "--e2e-marathon-part", "1",
        robot="fake", stop_after_isolate=True,
        env_extra={
            "E2E_SPEAKERS_DB": str(speakers),
            "FAKE_E2E_DBS": str(speakers),
        },
    )
    assert "ISOLATE_DONE" in proc.stdout, proc.stderr
    assert "#5031" not in proc.stderr, proc.stderr
    assert not speakers.exists(), "first с нуля обязан стереть speakers.e2e.db (#2890)"

    scenario = _copy_real(tmp_path, ACT2)
    proc, events = _run_isolation(tmp_path, scenario, phase="first")
    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert "WIPE /speaker_id_node" in events and "WIPE /mcp_server" in events


@bash_required
def test_single_run_without_phase_still_wipes(tmp_path):
    """Одиночный прогон (без фазы марафона) гард не касается — стирание #2890."""
    proc, events = _run_isolation(tmp_path, _copy_real(tmp_path, ACT2))
    assert "EXIT_OK" in proc.stdout, proc.stderr
    assert "WIPE /speaker_id_node" in events and "WIPE /mcp_server" in events


def test_guard_stands_before_robot_clock_and_wipe():
    """Отказ стоит до часов робота и до enter_e2e_mode.

    Дата сама ничего не стирает, но если она раньше гарда — отказ уже сходил
    на робота, и тест не отличит «базы целы» от «wipe мог случиться».
    """
    src = E2E_SCRIPT.read_text(encoding="utf-8")
    guard = src.index("refuse_first_mid_series\n")
    clock = src.index('E2E_RUN_BEFORE="$(${ROBOT_SSH} "date -u')
    enter = src.index("enter_e2e_mode() {")
    assert guard < clock < enter
