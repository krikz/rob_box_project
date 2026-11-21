"""Гард #5045: мёртвый ``mapping_state.json`` не возвращается в репозиторий.

``/maps/mapping_state.json`` лежал в томе карт Main Pi (bind-mount
``docker/main/maps`` → ``/maps``) и называл карту «квартира». Его никто не
читает с коммита ``4f5baba18`` (``fix(rtabmap): always start in localization
mode, remove state file dependency``, 2026-03-06): ``start_rtabmap.sh`` больше
не смотрит в файл, MCP-тулы держат состояние в памяти (``mapping_state.py``),
имя активной карты — строка ``maps.is_active`` в БД инструментов
(``WaypointStore``, #4392).

Самого файла в git никогда не было: каталог ``docker/main/maps/`` — данные
робота и закрыт ``.gitignore`` (#4371), поэтому «удалить из репо» нечего.
Опасность обратная. Если файл вернётся в дерево (сняли строку gitignore,
закоммитили JSON как конфиг), появится третий источник правды об активной
карте, и робот снова ответит «квартира», стоя в мастерской.

Тест падает, если git отслеживает путь, оканчивающийся на
``mapping_state.json``. На роботе файл убирает
``scripts/deploy/remove_dead_mapping_state.sh`` шагом деплоя Main Pi — это
проверяет ``scripts/deploy/tests/test_remove_dead_mapping_state.sh``.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
DEAD_NAME = "mapping_state.json"


def _tracked_dead_files() -> list[str]:
    """Пути в индексе git, чей последний компонент — ``mapping_state.json``."""
    out = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
        timeout=60,
    )
    tracked = out.stdout.decode("utf-8", errors="replace").split("\0")
    suffix = "/" + DEAD_NAME

    def is_dead(path: str) -> bool:
        return path.endswith(suffix) or path == DEAD_NAME

    return [path for path in tracked if is_dead(path)]


def test_git_does_not_track_mapping_state_json():
    """Ни один отслеживаемый файл не называется mapping_state.json (#5045)."""
    tracked = _tracked_dead_files()
    assert tracked == [], (
        "git снова отслеживает mapping_state.json (#5045). Файл мёртв с "
        "4f5baba18 и называет чужую карту: имя активной карты — "
        f"maps.is_active в БД (#4392), не файл. Вернувшиеся пути: {tracked}"
    )


def test_maps_volume_is_gitignored():
    """Том карт остаётся данными робота: docker/main/maps/ закрыт .gitignore.

    Иначе mapping_state.json с робота попадёт в коммит вместе с rtabmap.db.
    """
    out = subprocess.run(
        ["git", "check-ignore", "-v", "--", f"docker/main/maps/{DEAD_NAME}"],
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        encoding="utf-8",
        text=True,
        timeout=60,
    )
    assert out.returncode == 0, (
        "docker/main/maps/ больше не в .gitignore (#4371/#5045): мёртвый "
        "mapping_state.json сможет вернуться в репо. "
        f"stderr: {out.stderr.strip()}"
    )
    assert "docker/main/maps/" in out.stdout, out.stdout


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
