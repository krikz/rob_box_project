"""Регресс: прибитый SHA256 в заглавном регистре не должен ронять скачивание.

Дефект (15.09.2026, issue #2599): ``download_retinaface_hef.sh`` хранит
эталонный хэш заглавными буквами, а ``sha256sum`` печатает hex строчными.
Прямое сравнение строк не совпадало НИКОГДА — скрипт скачивал корректный
HEF, печатал "SHA256 mismatch" (одинаковый hex, разный регистр), удалял
файл и выходил с 1. Следствие на роботе: ``vision_face`` стартовала
``mode=real`` и сразу уходила в degraded с ``FileNotFoundError``, ни одного
события на ``/vision/hailo/events``.

Тест гоняет реальные скрипты против локального файла (``file://`` URL),
без сети.
"""

from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
from pathlib import Path
from typing import List, Optional

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / "docker" / "vision" / "scripts" / "vision-hailo"

DOWNLOADERS = (
    ("download_retinaface_hef.sh", "retinaface_mobilenet_v1.hef"),
    ("download_yolov8n_hef.sh", "yolov8n.hef"),
)


def _candidate_forms(path: Path) -> List[str]:
    """Как записать путь, чтобы его понял конкретный bash.

    На Windows рядом живут несколько bash (MSYS из Git, WSL), и диски они
    видят по-разному: ``/d/...`` против ``D:/...``. Форму не угадываем —
    подбираем пробой (см. ``_detect_path_form``).
    """
    posix = path.resolve().as_posix()
    forms = [posix]
    if len(posix) > 1 and posix[1] == ":":
        forms.insert(0, "/" + posix[0].lower() + posix[2:])
    return forms


def _detect_path_form():
    """Рабочий конвертер путей или ``None``, если bash не видит проект.

    MSYS-bash, запущенный из Python напрямую, приходит с поломанной таблицей
    монтирования и не находит файлы ни в одной форме. Тогда тест честно
    скипается (в CI на Linux он полноценный), а не врёт про сломанный скрипт.
    """
    if shutil.which("bash") is None:
        return None
    probe = REPO_ROOT / "Makefile"
    for index, probe_text in enumerate(_candidate_forms(probe)):
        check = subprocess.run(
            ["bash", "-c", 'test -f "$1"', "_", probe_text],
            capture_output=True,
            text=True,
        )
        if check.returncode == 0:
            return lambda path, i=index: _candidate_forms(path)[i]
    return None


PATH_FORM: Optional[object] = _detect_path_form()


@pytest.mark.skipif(
    PATH_FORM is None, reason="bash в этой среде не видит файлы проекта"
)
@pytest.mark.parametrize("script_name,hef_name", DOWNLOADERS)
def test_uppercase_pinned_sha_still_installs_hef(
    tmp_path: Path, script_name: str, hef_name: str
) -> None:
    script = SCRIPTS_DIR / script_name
    assert script.is_file(), "нет скрипта " + str(script)

    source = tmp_path / "source.hef"
    source.write_bytes(b"fake HEF payload for sha-case regression")
    digest = hashlib.sha256(source.read_bytes()).hexdigest()

    hef_dir = tmp_path / "models"
    env = dict(os.environ)
    env.update(
        HEF_DIR=PATH_FORM(hef_dir),
        HEF_URL="file://" + PATH_FORM(source),
        # Эталон заглавными — ровно так хэш прибит в скрипте.
        HEF_SHA256=digest.upper(),
        HEF_FORCE_DOWNLOAD="1",
    )

    result = subprocess.run(
        ["bash", PATH_FORM(script)],
        env=env,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, (
        "скрипт упал: rc=%s\nstdout: %s\nstderr: %s"
        % (result.returncode, result.stdout, result.stderr)
    )
    assert "mismatch" not in (result.stdout + result.stderr).lower()
    assert (hef_dir / hef_name).is_file(), "HEF не установлен"
