"""Признак серии решает, удалять ли e2e-БД (issue #5077).

Инцидент 22.11: марафон умер на акте 1 (phase=first, GHA run 44963710006),
каст остался в speakers.e2e.db. Следующий марафон (run 44973642247) упал на
гарде refuse_first_mid_series: «в speakers.e2e.db уже 1 профил(ей)». Таймер
владельца снял режим, но файлы БД не трогает — он внутри контейнера и маркера
серии не видит.

Удалять БД на каждом снятии режима нельзя: каст держится между актами одной
серии именно потому, что middle/last базы не трогают (#3972).

Правило (``e2e_series_should_delete <marker> <ttl> <current-id>``): удалять,
когда маркера нет, он пуст или битый; когда id в маркере не равен current-id
(чужая серия: её каст нам не принадлежит); когда current-id пуст (одиночный
прогон каст не держит); и когда id совпал, но маркер старше TTL. keep — только
совпавший id и свежий маркер. Сравнение id обязательно: TTL один не страхует
повтор. Инцидент 22.11 — оборванный прогон 23:50:59Z (run 44963710006), повтор
01:54:50Z, через 2 ч 04 мин, меньше TTL 10800 с. Свежий маркер прошлой серии
дал бы keep, и first упал бы на гарде, как run 44973642247. Живая серия
обновляет mtime маркера на каждом акте, поэтому посреди серии он не протухает.
SIGKILL раннера маркер не обновляет: mtime застывает, через TTL повтор той же
серии видит мёртвую серию и удаляет.

Гоняется настоящая функция из ``scripts/ci/e2e/e2e_series_marker.sh``.

Run:
    python3 -m pytest tests/unit/e2e_scripts/test_issue_5077_series_marker.py -v --no-cov
"""

from __future__ import annotations

import os
import shutil
import sqlite3
import subprocess
import time
from pathlib import Path

import pytest

from tests.unit.e2e_scripts.test_issue_2890_act_isolation import (
    BASH,
    NIGHT,
    REPO_ROOT,
    _require_bash_sees,
    bash_required,
)

E2E_SCRIPT = REPO_ROOT / "scripts" / "ci" / "e2e" / "e2e_voice_test.sh"
MARKER_SH = REPO_ROOT / "scripts" / "ci" / "e2e" / "e2e_series_marker.sh"
ACT2 = NIGHT / "night_marathon_act2_acquaintance_v1.json"
FAKE_ROBOT_SSH = Path(__file__).resolve().parent / "fake_robot_ssh.sh"

TTL = 10800


def decide(marker: Path | None, ttl: int = TTL, series_id: str = "night_20261122_010000"):
    """Решение настоящей функции: 'delete' | 'keep'.

    ``series_id`` — id текущей серии (третий аргумент). По умолчанию совпадает
    с id, который пишут TTL-кейсы: они проверяют возраст, а не чужую серию.
    Пустая строка — одиночный прогон, он каст не держит.
    """
    path = "" if marker is None else str(marker)
    script = "\n".join(
        [
            "set -u",
            f'. "{MARKER_SH}"',
            f'e2e_series_should_delete "{path}" "{ttl}" "{series_id}"',
        ]
    )
    p = subprocess.run([BASH, "-s"], input=script.encode("utf-8"), capture_output=True, timeout=60)
    return p.stdout.decode("utf-8", "replace").strip(), p.returncode


def _seed(path: Path, rows: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(path)
    con.execute("create table speakers (speaker_id text primary key, name text)")
    for i in range(rows):
        con.execute("insert into speakers values (?, ?)", (f"id{i}", f"cast{i}"))
    con.commit()
    con.close()


def _run(tmp_path: Path, *args: str, env_extra=None, robot: str = "stub"):
    """Настоящий скрипт до гарда. ``robot="stub"`` падает на первом ssh —
    решение по серии обязано случиться раньше."""
    _require_bash_sees(tmp_path)
    scn = tmp_path / "scenario.json"
    scn.write_text(ACT2.read_text(encoding="utf-8"), encoding="utf-8")
    if robot == "stub":
        ssh = tmp_path / "ssh_stub.sh"
        ssh.write_text("#!/bin/bash\necho SSH_STUB_CALLED >&2\nexit 97\n", encoding="utf-8")
        ssh.chmod(0o755)
    else:
        ssh = FAKE_ROBOT_SSH
    text = E2E_SCRIPT.read_text(encoding="utf-8")
    # Гард уже решил. Дальше часы робота и сценарий — к решению по серии
    # отношения не имеют, а фейк на незнакомом вызове честно падает.
    # Якорь — вызов, не определение: у определения за скобкой идёт комментарий.
    text = text.replace(
        "\nrefuse_first_mid_series\n",
        "\nrefuse_first_mid_series\necho GUARD_DONE\nexit 0\n",
        1,
    )
    script = E2E_SCRIPT.parent / f".e2e_voice_test_cut_{os.getpid()}.sh"
    script.write_text(text, encoding="utf-8")
    env = {
        "PATH": os.environ.get("PATH", ""),
        "PYTHONIOENCODING": "utf-8",
        "ROBOT_SSH_OVERRIDE": str(ssh),
        "HOME": str(tmp_path),
    }
    if robot == "fake":
        env["FAKE_ROBOT_STATE"] = str(tmp_path / "fake_robot")
    if env_extra:
        env.update(env_extra)
    try:
        return subprocess.run(
            [BASH, str(script), "--scenario", str(scn), *args],
            capture_output=True, text=True, encoding="utf-8", timeout=90, env=env,
        )
    finally:
        script.unlink(missing_ok=True)


@bash_required
class TestSeriesDecision:
    """Решение функции — по файлу маркера, без робота."""

    def test_no_marker_deletes(self, tmp_path):
        """Маркера нет — серии нет, e2e-БД удаляется (одиночный прогон)."""
        out, rc = decide(tmp_path / "absent.id")
        assert rc == 0 and out == "delete", out

    def test_empty_marker_deletes(self, tmp_path):
        marker = tmp_path / "series.id"
        marker.write_text("", encoding="utf-8")
        out, rc = decide(marker)
        assert rc == 0 and out == "delete", out

    def test_garbage_marker_deletes(self, tmp_path):
        """Битый маркер — не признак живой серии."""
        marker = tmp_path / "series.id"
        marker.write_text("not an id!\n", encoding="utf-8")
        out, rc = decide(marker)
        assert rc == 0 and out == "delete", out

    def test_fresh_marker_keeps(self, tmp_path):
        """Свежий маркер — серия жива, каст не трогать."""
        marker = tmp_path / "series.id"
        marker.write_text("night_20261122_010000\n", encoding="utf-8")
        out, rc = decide(marker)
        assert rc == 0 and out == "keep", out

    def test_expired_marker_deletes(self, tmp_path):
        """Маркер старше TTL — серия мертва (SIGKILL раннера), удалять."""
        marker = tmp_path / "series.id"
        marker.write_text("night_20261122_010000\n", encoding="utf-8")
        old = time.time() - (TTL + 60)
        os.utime(marker, (old, old))
        out, rc = decide(marker)
        assert rc == 0 and out == "delete", out

    def test_just_inside_ttl_keeps(self, tmp_path):
        """Секунда до TTL — серия ещё жива. Граница не должна съесть каст."""
        marker = tmp_path / "series.id"
        marker.write_text("night_20261122_010000\n", encoding="utf-8")
        fresh = time.time() - (TTL - 120)
        os.utime(marker, (fresh, fresh))
        out, rc = decide(marker)
        assert rc == 0 and out == "keep", out

    def test_fresh_marker_of_another_series_deletes(self, tmp_path):
        """Свежий маркер чужой серии — её каст нам не принадлежит.

        TTL это не страхует: инцидент 22.11, повтор через 2 ч 04 мин < TTL
        10800 с. Без сравнения id такой маркер дал бы keep, и следующий first
        упал бы на гарде, как run 44973642247.
        """
        marker = tmp_path / "series.id"
        marker.write_text("night_old\n", encoding="utf-8")
        out, rc = decide(marker, series_id="night_new")
        assert rc == 0 and out == "delete", out

    def test_fresh_marker_of_the_same_series_keeps(self, tmp_path):
        """Свежий маркер той же серии — каст между актами на месте."""
        marker = tmp_path / "series.id"
        marker.write_text("night_same\n", encoding="utf-8")
        out, rc = decide(marker, series_id="night_same")
        assert rc == 0 and out == "keep", out

    def test_empty_series_id_deletes_even_a_fresh_marker(self, tmp_path):
        """Одиночный прогон (current-id пуст) каст не держит — delete при любом маркере."""
        marker = tmp_path / "series.id"
        marker.write_text("night_live\n", encoding="utf-8")
        out, rc = decide(marker, series_id="")
        assert rc == 0 and out == "delete", out

    def test_missing_third_argument_is_empty_and_deletes(self, tmp_path):
        """Отсутствующий третий аргумент считается пустым — одиночный прогон, delete."""
        marker = tmp_path / "series.id"
        marker.write_text("night_live\n", encoding="utf-8")
        script = "\n".join(
            ["set -u", f'. "{MARKER_SH}"', f'e2e_series_should_delete "{marker}" "{TTL}"']
        )
        p = subprocess.run([BASH, "-s"], input=script.encode("utf-8"), capture_output=True, timeout=60)
        out = p.stdout.decode("utf-8", "replace").strip()
        assert p.returncode == 0 and out == "delete", out


@bash_required
class TestHarnessHonorsDecision:
    """Настоящий скрипт удаляет e2e-БД только по решению функции."""

    def test_dead_series_is_wiped_before_the_guard(self, tmp_path):
        """Оборванная серия: маркер протух, профили есть. Следующий first
        стартует с нуля, а не падает на гарде (run 44973642247)."""
        speakers = tmp_path / "speakers.e2e.db"
        _seed(speakers, 1)
        marker = tmp_path / "series.id"
        marker.write_text("night_dead\n", encoding="utf-8")
        os.utime(marker, (time.time() - (TTL + 60),) * 2)

        proc = _run(
            tmp_path, "--e2e-marathon-phase", "first",
            env_extra={
                "E2E_SPEAKERS_DB": str(speakers),
                "E2E_SERIES_MARKER": str(marker),
                "E2E_SERIES_ID": "night_new",
            },
        )

        assert "E2E_SERIES delete" in proc.stdout, proc.stderr
        assert not speakers.exists(), "мёртвая серия оставила e2e-БД"
        assert "GUARD_DONE" in proc.stdout, proc.stderr
        assert proc.returncode == 0, proc.stderr

    def test_fresh_foreign_series_is_wiped_before_the_guard(self, tmp_path):
        """Повтор через 2 часа: маркер прошлой серии ещё свежий (TTL 3 ч), но id
        другой. first новой серии стирает чужой каст ДО гарда, а не падает на нём
        (сценарий run 44973642247, который один TTL не ловит)."""
        speakers = tmp_path / "speakers.e2e.db"
        _seed(speakers, 1)
        marker = tmp_path / "series.id"
        marker.write_text("night_old\n", encoding="utf-8")

        proc = _run(
            tmp_path, "--e2e-marathon-phase", "first",
            env_extra={
                "E2E_SPEAKERS_DB": str(speakers),
                "E2E_SERIES_MARKER": str(marker),
                "E2E_SERIES_ID": "night_new",
            },
        )

        assert "E2E_SERIES delete" in proc.stdout, proc.stderr
        assert not speakers.exists(), "чужая серия оставила e2e-БД"
        assert "GUARD_DONE" in proc.stdout, proc.stderr
        assert proc.returncode == 0, proc.stderr

    def test_live_series_keeps_the_cast(self, tmp_path):
        """Живая серия: маркер свежий, first не трогает каст.

        middle/last до решения по маркеру не доходят вовсе: каст между актами
        держит #3972, и удаление до гарда для них запрещено. Здесь проверяется
        граница, где решение реально принимается, — first без признаков
        продолжения. Свежий маркер даёт keep, и профиль остаётся."""
        speakers = tmp_path / "speakers.e2e.db"
        _seed(speakers, 2)
        before = speakers.read_bytes()
        marker = tmp_path / "series.id"
        marker.write_text("night_live\n", encoding="utf-8")

        proc = _run(
            tmp_path, "--e2e-marathon-phase", "first",
            env_extra={
                "E2E_SPEAKERS_DB": str(speakers),
                "E2E_SERIES_MARKER": str(marker),
                "E2E_SERIES_ID": "night_live",
            },
        )

        assert "E2E_SERIES keep" in proc.stdout, proc.stderr
        assert speakers.read_bytes() == before, "живая серия стёрла каст"
        assert "SSH_STUB_CALLED" not in proc.stderr, "keep дошёл до робота"

    @pytest.mark.parametrize("phase", ["middle", "last"])
    def test_middle_and_last_never_delete_before_the_guard(self, tmp_path, phase):
        """middle/last до решения по маркеру не доходят (#3972): даже свежий
        маркер чужой серии и пустой id серии не дают повода стереть каст до гарда.
        Удаление посреди серии унесло бы каст следующему акту."""
        speakers = tmp_path / "speakers.e2e.db"
        _seed(speakers, 2)
        before = speakers.read_bytes()
        marker = tmp_path / "series.id"
        marker.write_text("night_old\n", encoding="utf-8")

        proc = _run(
            tmp_path, "--e2e-marathon-phase", phase,
            env_extra={
                "E2E_SPEAKERS_DB": str(speakers),
                "E2E_SERIES_MARKER": str(marker),
                "E2E_SERIES_ID": "night_new",
            },
        )

        assert "E2E_SERIES delete" not in proc.stdout, proc.stdout
        assert "E2E_SERIES keep" not in proc.stdout, proc.stdout
        assert speakers.read_bytes() == before, f"{phase} стёр каст до гарда"
        assert "GUARD_DONE" in proc.stdout, proc.stderr
        assert proc.returncode == 0, proc.stderr

    def test_single_run_without_series_deletes(self, tmp_path):
        """Прогон без серии (E2E_SERIES_ID пуст) маркера не пишет и удаляет."""
        speakers = tmp_path / "speakers.e2e.db"
        _seed(speakers, 1)
        marker = tmp_path / "series.id"

        proc = _run(
            tmp_path,
            env_extra={"E2E_SPEAKERS_DB": str(speakers), "E2E_SERIES_MARKER": str(marker)},
        )

        assert "E2E_SERIES delete" in proc.stdout, proc.stderr
        assert not speakers.exists()
        assert not marker.exists(), "прогон без серии не должен писать маркер"

    def test_single_run_deletes_a_fresh_foreign_marker(self, tmp_path):
        """Одиночный прогон (E2E_SERIES_ID пуст) каст не держит: свежий маркер
        чужой серии не повод оставить e2e-БД. Удаление — на старте, до гарда."""
        speakers = tmp_path / "speakers.e2e.db"
        _seed(speakers, 1)
        marker = tmp_path / "series.id"
        marker.write_text("night_old\n", encoding="utf-8")

        proc = _run(
            tmp_path,
            env_extra={
                "E2E_SPEAKERS_DB": str(speakers),
                "E2E_SERIES_MARKER": str(marker),
                "E2E_SERIES_ID": "",
            },
        )

        assert "E2E_SERIES delete" in proc.stdout, proc.stderr
        assert not speakers.exists(), "одиночный прогон оставил чужой каст"


def test_marker_file_is_deployed_with_the_harness():
    """Харнесс source'ит файл решения — workflow обязан копировать его на 249.

    Пропущенный файл даёт молчаливую деградацию: source падает, решение
    не определено, и прогон краснеет с диагнозом про робота (класс #1735).

    Отдельного input для id серии нет: у workflow_dispatch потолок 25 inputs,
    и он уже занят (тест #3972). Id едет в triggered_by_reason маркером
    series=<id>, который ставит только раннер марафона.
    """
    workflow = (REPO_ROOT / ".github" / "workflows" / "L-E2E Voice Test.yml").read_text(encoding="utf-8")
    assert "scp " in workflow and "e2e_series_marker.sh" in workflow
    assert "export E2E_SERIES_ID=" in workflow, "id серии не доходит до харнесса на 249"
    assert "series=[A-Za-z0-9_-]*" in workflow, "id серии не читается из причины диспатча"
    assert "e2e_series_id:" not in workflow, "26-й input упирается в потолок GitHub"
