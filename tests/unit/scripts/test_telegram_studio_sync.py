"""Студия обновляет свой код сама: синхронизация клона с develop (#4703).

Разбор 13.11 (ADR-0172): факты и медиа студия читала из worktree
`origin/develop` (он обновляется сам), а исполняемый код — из живого клона
`/home/builder/hermes-share/rob_box_project`, который обновлялся ТОЛЬКО
вручную. Итог: материал свежий, логика старая. 13.11 сессия в 03:00 UTC
отработала кодом от 11.11, хотя merge #4701 был в 08:58 того же дня.

Здесь — юнит-тесты на сам скрипт `scripts/telegram/studio_sync.sh`:
быстрый-forward, отказ при изменённом tracked-файле, выживание
untracked-файлов студии, no-op на актуальном клоне и — главное — случай,
когда обновление меняет САМ скрипт.

Тесты герметичны: origin — локальный bare-репозиторий, systemctl не
вызывается (команда рестарта подменяется через STUDIO_SYNC_RESTART_CMD),
сети нет.
"""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[3]
SCRIPT = REPO / "scripts" / "telegram" / "studio_sync.sh"

# Маркер новой версии скрипта: если при обновлении файла испортится
# исполнение, маркер всплывёт в stdout раньше времени.
NEW_VERSION_MARKER = "НОВАЯ_ВЕРСИЯ_СТУДИИ"


def _git(repo: Path, *args: str) -> str:
    """git в указанном репозитории; падение — это ошибка теста."""
    done = subprocess.run(
        ["git", *args],
        cwd=str(repo),
        capture_output=True,
        text=True,
        check=True,
    )
    return done.stdout.strip()


@pytest.fixture()
def studio(tmp_path: Path) -> SimpleNamespace:
    """Живой клон студии + origin (bare, локальный) + подменённый рестарт.

    Ни сети, ни systemd: origin — каталог на диске, рестарт — скрипт,
    который пишет строчку в лог.
    """
    bare = tmp_path / "origin.git"
    bare.mkdir()
    _git(bare, "init", "--bare", "--initial-branch=develop")

    seed = tmp_path / "seed"
    seed.mkdir()
    _git(seed, "init", "--initial-branch=develop")
    _git(seed, "config", "user.email", "studio@rob-box.local")
    _git(seed, "config", "user.name", "Studio")
    (seed / "README.md").write_text("база студии\n", encoding="utf-8")
    _git(seed, "add", "README.md")
    _git(seed, "commit", "--message=base")
    _git(seed, "remote", "add", "origin", str(bare))
    _git(seed, "push", "-u", "origin", "develop")

    clone = tmp_path / "live"
    _git(tmp_path, "clone", str(bare), str(clone), "--branch=develop")
    _git(clone, "config", "user.email", "studio@rob-box.local")
    _git(clone, "config", "user.name", "Studio")

    restart_log = tmp_path / "restart.log"
    fake_restart = tmp_path / "fake-restart.sh"
    fake_restart.write_text(
        "#!/bin/bash\nprintf 'restart %s\\n' \"$(date +%s)\" >> '" + str(restart_log) + "'\n",
        encoding="utf-8",
    )
    fake_restart.chmod(0o755)

    state_dir = tmp_path / "state"
    env = os.environ.copy()
    env.update(
        {
            "STUDIO_REPO": str(clone),
            "STUDIO_SYNC_RESTART_CMD": str(fake_restart),
            "STUDIO_SYNC_STATE_DIR": str(state_dir),
            "STUDIO_SYNC_LOCK_FILE": str(tmp_path / "studio-sync.lock"),
        }
    )
    return SimpleNamespace(
        bare=bare,
        seed=seed,
        clone=clone,
        env=env,
        restart_log=restart_log,
        state_dir=state_dir,
        tmp_path=tmp_path,
    )


def _commit_upstream(seed: Path, rel: str, content: str, message: str) -> str:
    """Новый коммит в origin/develop — как будто смержили PR."""
    path = seed / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    _git(seed, "add", rel)
    _git(seed, "commit", "--message", message)
    _git(seed, "push", "origin", "develop")
    return _git(seed, "rev-parse", "HEAD")


def _run(env: dict) -> subprocess.CompletedProcess:
    """Запуск скрипта синхронизации (без systemd и без сети)."""
    return subprocess.run(
        ["bash", str(SCRIPT)],
        capture_output=True,
        text=True,
        env=env,
        check=False,
    )


def _state(studio_env: SimpleNamespace) -> dict:
    """Состояние синхронизации, записанное скриптом рядом с состоянием студии."""
    state_file = studio_env.state_dir / "studio_sync.json"
    assert state_file.exists(), "скрипт не записал файл состояния"
    return json.loads(state_file.read_text(encoding="utf-8"))


def _restarts(studio_env: SimpleNamespace) -> int:
    """Сколько раз перезапускался studio-watch."""
    if not studio_env.restart_log.exists():
        return 0
    return len(studio_env.restart_log.read_text(encoding="utf-8").strip().splitlines())


# ------------------------------------------------------------------ 1. fast-forward


def test_fast_forward_pulls_new_code_and_restarts_service(studio: SimpleNamespace):
    """Клон отстал от develop → код обновлён, studio-watch перезапущен."""
    expected = _commit_upstream(studio.seed, "scripts/telegram/studio_facts.py", "новый код\n", "fix: facts")

    done = _run(studio.env)

    assert done.returncode == 0, done.stdout + done.stderr
    assert _git(studio.clone, "rev-parse", "HEAD") == expected
    assert _restarts(studio) == 1, "после реального обновления сервис обязан перезапуститься"

    state = _state(studio)
    assert state["result"] == "updated"
    assert state["behind_before"] == 1
    assert state["behind_after"] == 0
    assert state["restarted"] is True


# ------------------------------------------------------------------ 2. tracked-файл не затирается


def test_modified_tracked_file_is_not_overwritten(studio: SimpleNamespace):
    """В клоне изменён tracked-файл → обновления нет, работа не затерта."""
    _commit_upstream(studio.seed, "scripts/telegram/studio_facts.py", "версия из develop\n", "fix: facts")
    before = _git(studio.clone, "rev-parse", "HEAD")

    victim = studio.clone / "README.md"
    victim.write_text("ПРАВКА ЧЕЛОВЕКА\n", encoding="utf-8")

    done = _run(studio.env)

    assert done.returncode != 0, "изменённый tracked-файл обязан давать ненулевой код"
    assert victim.read_text(encoding="utf-8") == "ПРАВКА ЧЕЛОВЕКА\n"
    assert _git(studio.clone, "rev-parse", "HEAD") == before, "клон не должен двигаться"
    assert _restarts(studio) == 0

    state = _state(studio)
    assert state["result"] == "refused-dirty"
    assert state["behind_after"] == state["behind_before"], "отставание не изменилось — клон не тронут"


# ------------------------------------------------------------------ 3. untracked переживает


def test_untracked_studio_file_survives_update(studio: SimpleNamespace):
    """Файлы студии (очередь, инфографики) — untracked: их не трогаем."""
    _commit_upstream(studio.seed, "scripts/telegram/studio_facts.py", "новый код\n", "fix: facts")

    queue = studio.clone / "docs" / "channel" / "queue" / "900-post.md"
    queue.parent.mkdir(parents=True, exist_ok=True)
    queue.write_text("черновик Шифу\n", encoding="utf-8")
    infographic = studio.clone / "docs" / "channel" / "infographics" / "loc-drift.json"
    infographic.parent.mkdir(parents=True, exist_ok=True)
    infographic.write_text('{"loc": "drift"}\n', encoding="utf-8")

    done = _run(studio.env)

    assert done.returncode == 0, done.stdout + done.stderr
    assert queue.read_text(encoding="utf-8") == "черновик Шифу\n"
    assert infographic.read_text(encoding="utf-8") == '{"loc": "drift"}\n'


def test_untracked_conflict_stops_update_without_deleting_file(studio: SimpleNamespace):
    """develop принёс файл с тем же путём → стоп и причина, чужое НЕ удалено."""
    _commit_upstream(
        studio.seed,
        "docs/channel/queue/900-post.md",
        "версия из develop\n",
        "feat: post queue",
    )

    queue = studio.clone / "docs" / "channel" / "queue" / "900-post.md"
    queue.parent.mkdir(parents=True, exist_ok=True)
    queue.write_text("черновик Шифу\n", encoding="utf-8")
    before = _git(studio.clone, "rev-parse", "HEAD")

    done = _run(studio.env)

    assert done.returncode != 0, "конфликт с untracked-файлом обязан давать ненулевой код"
    assert queue.exists(), "untracked-файл студии нельзя удалять"
    assert queue.read_text(encoding="utf-8") == "черновик Шифу\n"
    assert _git(studio.clone, "rev-parse", "HEAD") == before
    assert _restarts(studio) == 0


# ------------------------------------------------------------------ 4. повторный запуск = no-op


def test_second_run_on_current_clone_does_not_restart_service(studio: SimpleNamespace):
    """Клон актуален → ничего не делаем, сервис НЕ рвём зря."""
    _commit_upstream(studio.seed, "scripts/telegram/studio_facts.py", "новый код\n", "fix: facts")
    assert _run(studio.env).returncode == 0
    assert _restarts(studio) == 1

    done = _run(studio.env)

    assert done.returncode == 0, done.stdout + done.stderr
    assert _restarts(studio) == 1, "на актуальном клоне рестарт не нужен"

    state = _state(studio)
    assert state["result"] == "up-to-date"
    assert state["behind_before"] == 0
    assert state["restarted"] is False


# ------------------------------------------------------------------ 5. скрипт обновляет сам себя


def test_update_that_changes_the_script_itself_does_not_corrupt_execution(
    studio: SimpleNamespace,
):
    """Главный регресс: merge меняет САМ studio_sync.sh — исполнение не портится.

    git при merge переписывает изменённые файлы НА МЕСТЕ (тот же inode), а bash
    читает скрипт по мере исполнения. Без защиты обновление рушит само себя:
    bash продолжает читать новый файл со старого смещения и выполняет мусор.

    Проверяем на живом поведении: новая версия скрипта лежит в origin/develop,
    скрипт из клона тянет её и обязан доработать нормально.
    """
    script_in_repo = studio.clone / "scripts" / "telegram" / "studio_sync.sh"
    script_in_repo.parent.mkdir(parents=True, exist_ok=True)
    script_in_repo.write_text(SCRIPT.read_text(encoding="utf-8"), encoding="utf-8")
    _git(studio.clone, "add", "scripts/telegram/studio_sync.sh")
    _git(studio.clone, "commit", "--message=feat: studio_sync.sh")

    # Та же версия, но с новой строкой в конце: при чтении «по старому
    # смещению» bash исполнил бы её вместо нормального завершения.
    newer = SCRIPT.read_text(encoding="utf-8") + (
        '\necho "[studio-sync] ' + NEW_VERSION_MARKER + '"\n'
    )
    expected = _commit_upstream(
        studio.seed,
        "scripts/telegram/studio_sync.sh",
        newer,
        "feat: studio_sync.sh (новее)",
    )

    done = _run(studio.env)

    assert done.returncode == 0, done.stdout + done.stderr
    assert NEW_VERSION_MARKER not in done.stdout, "исполнение скрипта пошло по мусору"
    assert _git(studio.clone, "rev-parse", "HEAD") == expected, "обновление не применилось"
    assert NEW_VERSION_MARKER in script_in_repo.read_text(encoding="utf-8"), "файл скрипта не обновлён"
    assert _restarts(studio) == 1, "код изменился — сервис надо перезапустить"


# ------------------------------------------------------------------ 6. параллельный запуск


def test_second_instance_reports_already_running(studio: SimpleNamespace):
    """Вторая копия не мешает первой: говорит «уже обновляется» и выходит."""
    lock_file = studio.tmp_path / "studio-sync.lock"
    lock_file.touch()
    with lock_file.open("r+") as handle:
        import fcntl

        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        done = _run(studio.env)

    assert done.returncode == 0, done.stdout + done.stderr
    assert "уже обновляется" in done.stdout
    assert _restarts(studio) == 0


# ------------------------------------------------------------------ 7. запреты ADR


def test_script_uses_neither_stash_nor_reset_hard(studio: SimpleNamespace):
    """ADR-0172: без stash и без reset --hard поверх несохранённого."""
    text = SCRIPT.read_text(encoding="utf-8")

    assert "git stash" not in text
    assert "reset --hard" not in text
    assert "git clean" not in text


# ------------------------------------------------------------------ 8. systemd-юниты


def test_sync_units_exist_and_use_repo_root_placeholder():
    """Юниты лежат рядом со studio-watch и подставляют корень клона."""
    service = (REPO / "docker" / "studio" / "studio-sync.service").read_text(encoding="utf-8")
    timer = (REPO / "docker" / "studio" / "studio-sync.timer").read_text(encoding="utf-8")

    assert "@REPO_ROOT@" in service
    assert "scripts/telegram/studio_sync.sh" in service
    assert "Type=oneshot" in service
    assert "OnUnitActiveSec=5min" in timer
    assert "WantedBy=timers.target" in timer


def test_install_script_installs_sync_units():
    """studio_install.sh ставит sync-юнит и таймер так же, как studio-watch."""
    install = (REPO / "scripts" / "telegram" / "studio_install.sh").read_text(encoding="utf-8")

    assert "studio-sync.service" in install
    assert "studio-sync.timer" in install
    assert "@REPO_ROOT@" in install
