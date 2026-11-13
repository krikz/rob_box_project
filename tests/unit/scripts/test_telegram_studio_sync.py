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


def _run(env: dict, script: Path | None = None) -> subprocess.CompletedProcess:
    """Запуск скрипта синхронизации (без systemd и без сети).

    По умолчанию — скрипт из репозитория. Для теста «обновление меняет сам
    скрипт» нужен запуск ИЗ КЛОНА: именно его файл переписывает merge.
    """
    return subprocess.run(
        ["bash", str(script or SCRIPT)],
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


def test_stale_submodule_does_not_block_later_updates(studio: SimpleNamespace):
    """Сдвинутый указатель сабмодуля не должен залипнуть синхронизацию.

    merge обновляет указатель сабмодуля, но не его рабочее дерево: после
    fast-forward `git status` показывает " M src/...". Если считать это
    «изменённым tracked-файлом», то каждый следующий запуск видел бы грязь
    и отказывался обновлять клон — синхронизация залипала бы навсегда.
    У боевого клона студии сабмодули есть (src/ros2leds, src/vesc_nexus),
    так что это не теория.
    """
    # Настоящий сабмодуль: отдельный bare-репозиторий и рабочее дерево.
    sub_bare = studio.tmp_path / "suborigin.git"
    _git(studio.tmp_path, "init", "--bare", "--initial-branch=develop", str(sub_bare))
    sub_seed = studio.tmp_path / "subseed"
    sub_seed.mkdir()
    _git(sub_seed, "init", "--initial-branch=develop")
    _git(sub_seed, "config", "user.email", "studio@rob-box.local")
    _git(sub_seed, "config", "user.name", "Studio")
    (sub_seed / "s.txt").write_text("v1\n", encoding="utf-8")
    _git(sub_seed, "add", "s.txt")
    _git(sub_seed, "commit", "--message=sub v1")
    _git(sub_seed, "remote", "add", "origin", str(sub_bare))
    _git(sub_seed, "push", "-u", "origin", "develop")
    sub_sha_v1 = _git(sub_seed, "rev-parse", "HEAD")

    # Сабмодуль в origin/develop.
    _git(studio.seed, "-c", "protocol.file.allow=always", "submodule", "add", "-q", str(sub_bare), "vendor/sub")
    _git(studio.seed, "commit", "--message=feat: vendor/sub")
    _git(studio.seed, "push", "origin", "develop")
    _git(studio.clone, "fetch", "origin", "develop")
    _git(studio.clone, "merge", "--ff-only", "origin/develop")
    _git(studio.clone, "-c", "protocol.file.allow=always", "submodule", "update", "--init", "-q", "vendor/sub")

    # В develop указатель сабмодуля сдвинут — рабочее дерево в клоне остаётся
    # на старом коммите, и status показывает " M vendor/sub".
    (sub_seed / "s.txt").write_text("v2\n", encoding="utf-8")
    _git(sub_seed, "add", "s.txt")
    _git(sub_seed, "commit", "--message=sub v2")
    _git(sub_seed, "push", "origin", "develop")
    sub_sha_v2 = _git(sub_seed, "rev-parse", "HEAD")
    # Указатель сабмодуля ставим явно (не через update --remote): так тест не
    # зависит от того, как git выбирает ветку сабмодуля.
    _git(studio.seed, "update-index", "--add", "--cacheinfo", f"160000,{sub_sha_v2},vendor/sub")
    _git(studio.seed, "commit", "--message=feat: vendor/sub moved")
    _git(studio.seed, "push", "origin", "develop")
    _git(studio.clone, "fetch", "origin", "develop")
    _git(studio.clone, "merge", "--ff-only", "origin/develop")

    # Проверяем предпосылку: клон теперь «грязный» по мнению git.
    # (вывод git status с ведущим пробелом — helper его срезает, поэтому
    # сверяемся по имени и по признаку изменения)
    assert sub_sha_v1 != sub_sha_v2
    dirty_line = _git(studio.clone, "status", "--porcelain", "--untracked-files=no")
    assert "vendor/sub" in dirty_line, "сабмодуль должен быть виден как изменённый"
    assert dirty_line.startswith("M"), f"ожидался изменённый сабмодуль, получено: {dirty_line!r}"

    # А теперь синхронизация: грязный сабмодуль не должен блокировать обновление.
    _commit_upstream(studio.seed, "scripts/telegram/studio_facts.py", "код\n", "fix: facts")
    done = _run(studio.env)

    assert done.returncode == 0, "грязный сабмодуль не должен блокировать обновление:\n" + done.stdout
    assert _state(studio)["behind_after"] == 0
    assert _restarts(studio) == 1, "обновление должно было примениться"





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
    читает скрипт по мере исполнения, по смещению в файле. Если бы скрипт
    исполнялся прямо из файла в репозитории, обновление рушило бы само себя:
    bash продолжил бы читать переписанный файл со старого смещения и выполнил
    бы мусор вместо своего хвоста.

    Поэтому скрипт обязан работать из ПРИВАТНОЙ КОПИИ во временном каталоге:
    её merge не касается. Проверяем это прямо по журналу — скрипт печатает
    путь, из которого реально работает.

    Версии скрипта лежат в истории origin:
      A — та, что запускается из клона;
      B — новая (с маркерной строкой), её приносит merge.
    Запуск идёт ИЗ КЛОНА: именно его файл переписывает merge.
    """
    version_a = SCRIPT.read_text(encoding="utf-8")
    script_in_clone = studio.clone / "scripts" / "telegram" / "studio_sync.sh"

    # Версия A уезжает в origin/develop — клон её потом получит fast-forward'ом.
    (studio.seed / "scripts" / "telegram").mkdir(parents=True, exist_ok=True)
    (studio.seed / "scripts" / "telegram" / "studio_sync.sh").write_text(version_a, encoding="utf-8")
    _git(studio.seed, "add", "scripts/telegram/studio_sync.sh")
    _git(studio.seed, "commit", "--message=feat: studio_sync.sh (версия A)")
    _git(studio.seed, "push", "origin", "develop")
    _git(studio.clone, "fetch", "origin", "develop")
    _git(studio.clone, "merge", "--ff-only", "origin/develop")
    assert script_in_clone.read_text(encoding="utf-8") == version_a

    # Версия B: маркерная строка в начало файла. Если бы bash читал файл в
    # клоне со старого смещения, содержимое сдвинулось бы и исполнился мусор.
    version_b = version_a.replace(
        "set -euo pipefail",
        'set -euo pipefail\necho "[studio-sync] ' + NEW_VERSION_MARKER + '"',
        1,
    )
    assert NEW_VERSION_MARKER in version_b
    expected = _commit_upstream(
        studio.seed,
        "scripts/telegram/studio_sync.sh",
        version_b,
        "feat: studio_sync.sh (версия B)",
    )

    done = _run(studio.env, script=script_in_clone)

    assert done.returncode == 0, done.stdout + done.stderr
    assert NEW_VERSION_MARKER not in done.stdout, "исполнение скрипта пошло по мусору"
    assert _git(studio.clone, "rev-parse", "HEAD") == expected, "обновление не применилось"
    assert NEW_VERSION_MARKER in script_in_clone.read_text(encoding="utf-8"), "файл скрипта не обновлён"
    assert _restarts(studio) == 1, "код изменился — сервис надо перезапустить"

    # Работать — из приватной копии, а не из файла в репозитории: merge
    # переписывает файл в клоне на месте, и bash, читающий его со старого
    # смещения, исполнил бы чужой код.
    working_from = _working_copy_path(done.stdout)
    assert working_from is not None, "скрипт не сообщил, из чего он работает"
    assert working_from != str(script_in_clone), "скрипт работает прямо из файла в репозитории"
    assert not Path(working_from).exists(), "приватная копия должна быть удалена на выходе"


def _working_copy_path(stdout: str) -> str | None:
    """Путь, из которого скрипт реально работает (из его же журнала)."""
    for line in stdout.splitlines():
        marker = "работаю из копии: "
        if marker in line:
            return line.split(marker, 1)[1].strip()
    return None


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
    """ADR-0172: без stash и без reset --hard поверх несохранённого.

    Проверяем КОМАНДЫ, а не весь текст: в комментариях скрипт эти слова
    упоминает, объясняя, почему они запрещены.
    """
    commands = [
        line.strip()
        for line in SCRIPT.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]
    body = "\n".join(commands)

    assert "stash" not in body
    assert "reset --hard" not in body
    assert "git clean" not in body
    assert "--ff-only" in body, "обновление должно быть fast-forward'ом"


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
