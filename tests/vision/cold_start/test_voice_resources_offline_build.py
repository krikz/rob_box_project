"""Regression test: voice-resources Dockerfile.offline собирается без сети.

Cold-start при недоступном registry (issue #2610) предполагает, что
даже init-контейнер voice-resources-init может стартануть с образа,
собранного локально БЕЗ доступа в интернет (иначе после ребута
Pi первый же ``docker compose up`` зависнет на init-pull).

Тест проверяет: ``docker build --network=none -f Dockerfile.offline``
проходит успешно. Если это не так — кто-то изменил Dockerfile и
добавил сетевую зависимость (pip install, curl, download_samples.py),
что ломает cold-start сценарий.

⚠️ Этот тест требует Docker. Пропускается там, где его нет.
"""
from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
DOCKERFILE_OFFLINE = (
    REPO_ROOT / "docker" / "vision" / "voice_resources" / "Dockerfile.offline"
)
INIT_SCRIPT = REPO_ROOT / "docker" / "vision" / "voice_resources" / "init_resources.sh"


def _docker_available() -> bool:
    return shutil.which("docker") is not None and subprocess.run(
        ["docker", "version"],
        capture_output=True,
        text=True,
        timeout=10,
    ).returncode == 0


pytestmark = pytest.mark.skipif(
    not _docker_available(),
    reason="docker CLI недоступен (типично для CI без docker)",
)


def test_dockerfile_offline_exists() -> None:
    """Dockerfile.offline должен существовать (иначе e2e cold-start сломан)."""
    assert DOCKERFILE_OFFLINE.exists(), (
        f"{DOCKERFILE_OFFLINE} не найден. Этот Dockerfile нужен для "
        f"cold-start e2e на железе (issue #2610 acceptance #3). "
        f"Без него init-контейнер не сможет стартовать после ребута "
        f"при недоступном registry."
    )


def test_dockerfile_offline_has_no_network_calls() -> None:
    """В Dockerfile.offline не должно быть ``pip install``, ``curl``, ``wget``.

    Если кто-то добавит — cold-start e2e сломается: build не пройдёт
    при ``--network=none``. Проверяем только реальные RUN/COPY строки,
    не комментарии (в комментариях можно ссылаться на ``download_samples``
    для контекста, главное — не вызывать).
    """
    text = DOCKERFILE_OFFLINE.read_text(encoding="utf-8")

    forbidden_substrings = [
        "pip install",
        "pip3 install",
        "apt-get install",  # Тоже сеть, даже если пакеты уже в кэше
        "curl ",
        "wget ",
    ]

    bad: list[str] = []
    for raw_line in text.splitlines():
        # Убираем комментарии и пустые строки
        stripped = raw_line.split("#", 1)[0].strip()
        if not stripped:
            continue
        # Только команды RUN/COPY/ADD — это активные шаги Dockerfile.
        if not re.match(r"^(RUN|COPY|ADD)\s+", stripped):
            continue
        for pattern in forbidden_substrings:
            if pattern in stripped:
                bad.append(f"{raw_line.strip()!r} содержит {pattern!r}")

    assert not bad, (
        f"Dockerfile.offline содержит сетевые вызовы в активных шагах: "
        f"{bad}. Это сломает cold-start сценарий (build с --network=none "
        f"упадёт). Если добавление samples критично — вынесите их в "
        f"отдельный build-stage с явным комментарием 'ОЖИДАЕТ СЕТЬ'."
    )


def test_dockerfile_offline_builds_without_network() -> None:
    """``docker build --network=none -f Dockerfile.offline`` — exit 0.

    Это e2e-проверка cold-start сценария в миниатюре: build должен
    пройти без единого сетевого обращения.
    """
    result = subprocess.run(
        [
            "docker", "build",
            "--network=none",
            "-f", str(DOCKERFILE_OFFLINE),
            "-t", "voice-resources-cold-start:test",
            str(REPO_ROOT),
        ],
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert result.returncode == 0, (
        f"docker build --network=none Dockerfile.offline exit={result.returncode}.\n"
        f"STDOUT (tail 2000):\n{result.stdout[-2000:]}\n"
        f"STDERR (tail 2000):\n{result.stderr[-2000:]}"
    )


def test_dockerfile_offline_references_init_script() -> None:
    """Dockerfile.offline должен копировать init_resources.sh и делать его entrypoint."""
    text = DOCKERFILE_OFFLINE.read_text(encoding="utf-8")
    assert "init_resources.sh" in text, (
        "Dockerfile.offline не ссылается на init_resources.sh — "
        "entrypoint будет пустой/сломанный. Cold-start сценарий не "
        "сможет корректно отработать (volume останется не-initialized)."
    )
    assert "ENTRYPOINT" in text, (
        "Dockerfile.offline без ENTRYPOINT — контейнер не сможет "
        "выполнить init_resources.sh при старте."
    )
