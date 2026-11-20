#!/usr/bin/env python3
"""Гард пина ядра: импорт из site-packages тега Dockerfile, а не из src/ (#4932, #4891).

pytest монорепо кладёт ``src/<пакет>`` в ``sys.path`` раньше site-packages. Зелёный прогон тогда
смотрит локальную копию, а образ — ``rob-box-dialog-core`` из pip. Этот скрипт запускает чистый
интерпретатор без ``src`` на пути и требует, чтобы импорт шёл из site-packages пина Dockerfile.

Подмена ловится так: если каталог ``src/<пакет>`` оказывается на ``sys.path`` раньше установленного
пакета, импорт резолвится в чекаут. Проба это детектит и падает, а не зеленеет. Без префикса с
тегом скрипт падает, а не пропускается: штатные unit-джобы этот каталог не собирают (он вне
``src/*/test`` и вне ``tests/unit``), а джоб ``G: Dialog Core Pin Guard`` ставит тег в
изолированный venv и передаёт ``ROB_BOX_DIALOG_CORE_PREFIX``.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_PIN_RE = re.compile(r"rob_box_dialog_core@([\w.]+)")
_CORE_TOPS = (
    "rob_box_core",
    "rob_box_dialog",
    "rob_box_dialog_host",
    "rob_box_harness",
    "rob_box_llm",
    "rob_box_music",
)


def pin_in(dockerfile: Path) -> str:
    """Тег ``rob_box_dialog_core@`` из одного Dockerfile. Нет строки — ошибка, не догадка."""
    match = _PIN_RE.search(dockerfile.read_text(encoding="utf-8"))
    if not match:
        raise SystemExit(f"пин rob_box_dialog_core@ не найден в {dockerfile}")
    return match.group(1)


def pin_from_dockerfile(repo: Path = _REPO) -> str:
    """Тег ядра. voice-assistant и telegram_bot обязаны нести один и тот же.

    Расхождение — дырка: образ телеграма молча останется на старом теге. Источник
    истины — voice Dockerfile; telegram сверяется с ним, второго хардкода нет.
    """
    voice = repo / "docker" / "sound" / "voice_assistant" / "Dockerfile"
    telegram = repo / "docker" / "sound" / "telegram_bot" / "Dockerfile"
    pin = pin_in(voice)
    other = pin_in(telegram)
    if other != pin:
        raise SystemExit(
            f"пины ядра разошлись: voice-assistant {pin}, telegram_bot {other}"
        )
    return pin


def src_dirs(repo: Path = _REPO) -> list[Path]:
    return [repo / "src" / name for name in _CORE_TOPS]


def probe(
    python: str, pin: str, planted: list[Path] | None = None
) -> subprocess.CompletedProcess[str]:
    """Чистый интерпретатор: ни cwd, ни PYTHONPATH. Подмену передаём аргументом.

    Флаг ``-P`` на 3.10 нет, ``-I`` выкидывает PYTHONPATH — поэтому путь явный.
    ``-c`` несёт только вызов функции этого файла: логика пробы одна, её же гоняет тест.
    """
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    call = "import runpy, sys; runpy.run_path(sys.argv[1], run_name='probe')['run_probe']()"
    return subprocess.run(
        [
            python,
            "-I",
            "-c",
            call,
            str(Path(__file__).resolve()),
            pin,
            *(str(path) for path in (planted or [])),
        ],
        capture_output=True,
        text=True,
        env=env,
        timeout=60,
        check=False,
    )


def run_probe() -> None:
    """Точка входа чистого интерпретатора. Печатает IMPORT/DIRECT и падает на подмене."""
    pin = sys.argv[2]
    planted = [entry for entry in sys.argv[3:] if entry]
    for entry in reversed(planted):
        sys.path.insert(0, entry)
    import rob_box_dialog

    origin = Path(rob_box_dialog.__file__).resolve()
    print(f"IMPORT {origin}")
    print("PATH")
    for entry in sys.path:
        print(entry)
    parts = origin.parts
    pkg = parts[parts.index("src") + 1] if "src" in parts else ""
    if pkg in _CORE_TOPS:
        raise SystemExit(f"подмена: импорт из чекаута {origin}")
    if "site-packages" not in origin.parts:
        raise SystemExit(f"импорт не из site-packages: {origin}")
    planted_resolved = {Path(entry).resolve() for entry in planted}
    ahead = [
        entry
        for entry in sys.path
        if entry and Path(entry).resolve() in planted_resolved
    ]
    if ahead:
        raise SystemExit("подмена: src на sys.path раньше пакета: " + ", ".join(ahead))
    # Версия в metadata тега может не совпасть с именем тега. Сверяем direct_url: pip пишет туда тег.
    from importlib.metadata import distribution

    direct = distribution("rob-box-dialog-core").read_text("direct_url.json") or ""
    print(f"DIRECT {direct}")
    if pin not in direct:
        raise SystemExit(f"установлен не тег {pin}: {direct}")
    print(f"PIN {pin}")


def require_prefix() -> Path:
    prefix = os.environ.get("ROB_BOX_DIALOG_CORE_PREFIX")
    if not prefix:
        raise SystemExit(
            "нет ROB_BOX_DIALOG_CORE_PREFIX: нужен venv с pip install "
            "rob-box-dialog-core@<тег из Dockerfile>, без src на PYTHONPATH"
        )
    python = Path(prefix) / "bin" / "python"
    if not python.is_file():
        raise SystemExit(f"в префиксе нет {python}")
    return python


def main(argv: list[str] | None = None) -> int:
    """``--substitution`` — отрицательный контроль; иначе проверка установленного пина."""
    args = list(sys.argv[1:] if argv is None else argv)
    pin = pin_from_dockerfile()
    if "--substitution" in args:
        got = probe(sys.executable, "local-src", planted=src_dirs())
        sys.stdout.write(got.stdout)
        sys.stderr.write(got.stderr)
        if got.returncode == 0 or "подмена" not in (got.stdout + got.stderr):
            print("отрицательный контроль не поймал подмену", file=sys.stderr)
            return 1
        print("SUBSTITUTION detected")
        return 0
    got = probe(str(require_prefix()), pin)
    sys.stdout.write(got.stdout)
    sys.stderr.write(got.stderr)
    if got.returncode != 0:
        return got.returncode or 1
    if "site-packages" not in got.stdout or "подмена" in got.stdout:
        print("пин не доказан: нет site-packages или есть подмена", file=sys.stderr)
        return 1
    print(f"PIN_OK {pin}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
