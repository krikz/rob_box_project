"""Синхронизация YAML-конфига сегментации и его TS-зеркала (issue #2199).

Что делает
----------
Сейчас профиль ``wake.client_vad`` живёт в двух местах:

  * ``src/rob_box_core/config/speech_segmentation.yaml`` — источник истины.
  * ``src/rob_box_quest/webxr_client/src/input/voice_segmentation_constants.ts``
    — ручное зеркало (TS не умеет импортировать YAML напрямую).

Этот скрипт проверяет, что числа в обоих местах совпадают. В перспективе
его заменит кодогенерация ``tools/generate_ts_voice_constants.py``
(voice-vr 08, отдельная карточка); пока — guard-rail.

Запуск
------

    cd <repo>
    python3 -m rob_box_core.tools.check_ts_sync

Коды выхода: 0 — синхронно, 1 — расхождение, 2 — не найден файл.

Зачем это нужно
----------------
Без guard'а правка ``rms_threshold`` или ``hangover_ms`` в YAML без
зеркала ломает wake-канал: клиент шлёт фразу, пока сервер ещё не
считает её завершённой (issue #2135 уже стрелял по этой причине).
"""  # noqa: D401 — docstring is intentionally verbose.

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# YAML — единственная мягкая зависимость. setup.py её уже объявил
# (install_requires=["setuptools", "pyyaml"]), но на dev-машине без
# ``pip install -e`` пакет может быть не установлен. Пытаемся импортнуть;
# если не вышло — сообщаем и выходим.
try:
    import yaml  # type: ignore[import-untyped]
except ImportError as exc:  # pragma: no cover - окружение
    sys.stderr.write(
        "check_ts_sync: требуется pyyaml. Установите: pip install pyyaml\n"
    )
    raise SystemExit(2) from exc


# Пути по умолчанию — относительно корня репо (где запускается
# ``python3 -m rob_box_core.tools.check_ts_sync``).
_DEFAULT_YAML = Path("src/rob_box_core/config/speech_segmentation.yaml")
_DEFAULT_TS = Path(
    "src/rob_box_quest/webxr_client/src/input/voice_segmentation_constants.ts"
)

# Маппинг «имя в YAML» → «имя константы в TS». Менять согласованно
# с обоими файлами. Если добавляете новое поле — расширьте маппинг и
# добавьте регекс в ``_ts_extract_constants``.
_FIELDS: tuple[tuple[str, str], ...] = (
    ("client_vad.rms_threshold", "VOICE_RMS_THRESHOLD_DEFAULT"),
    ("client_vad.hangover_ms", "VOICE_HANGOVER_MS_DEFAULT"),
)

_TS_CONST_RE = re.compile(
    r"export\s+const\s+(?P<name>[A-Z0-9_]+)\s*=\s*(?P<value>-?\d+(?:\.\d+)?)\s*;"
)


def _yaml_get(d: dict, dotted: str) -> int | float | None:
    """Безопасный доступ к вложенному ключу через точку."""
    cur: object = d
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    if isinstance(cur, (int, float)):
        return cur
    return None


def _yaml_load(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fp:
        data = yaml.safe_load(fp)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: корень YAML не dict (получен {type(data).__name__})")
    return data


def _ts_extract_constants(path: Path) -> dict[str, float | int]:
    """Парсим TS-зеркало: ``export const NAME = 200;`` → {NAME: 200}.

    Строковые/boolean-значения игнорируются (мы синхронизируем только
    численные пороги). Многострочные декларации поддерживаются.
    """
    text = path.read_text(encoding="utf-8")
    found: dict[str, float | int] = {}
    for match in _TS_CONST_RE.finditer(text):
        name = match.group("name")
        raw = match.group("value")
        if "." in raw:
            found[name] = float(raw)
        else:
            found[name] = int(raw)
    return found


def _check(yaml_path: Path, ts_path: Path) -> list[str]:
    """Вернуть список расхождений (пустой — ОК)."""
    diffs: list[str] = []
    yaml_data = _yaml_load(yaml_path)
    ts_data = _ts_extract_constants(ts_path)
    for yaml_key, ts_name in _FIELDS:
        yaml_val = _yaml_get(yaml_data["profiles"]["wake"], yaml_key)  # type: ignore[index]
        ts_val = ts_data.get(ts_name)
        if yaml_val is None:
            diffs.append(
                f"{yaml_path}: ключ profiles.wake.{yaml_key} не найден в YAML"
            )
            continue
        if ts_val is None:
            diffs.append(
                f"{ts_path}: константа {ts_name} не найдена (должна быть "
                f"{yaml_val} — иначе клиент и сервер расходятся)"
            )
            continue
        if float(yaml_val) != float(ts_val):
            diffs.append(
                f"{yaml_key}: YAML={yaml_val!r} vs TS({ts_name})={ts_val!r}"
            )
    return diffs


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Проверяет, что TS-зеркало порогов сегментации речи "
            "синхронно с YAML-конфигом."
        )
    )
    parser.add_argument("--yaml", type=Path, default=_DEFAULT_YAML)
    parser.add_argument("--ts", type=Path, default=_DEFAULT_TS)
    args = parser.parse_args(argv)

    if not args.yaml.exists():
        sys.stderr.write(f"check_ts_sync: не найден YAML: {args.yaml}\n")
        return 2
    if not args.ts.exists():
        sys.stderr.write(f"check_ts_sync: не найден TS: {args.ts}\n")
        return 2

    diffs = _check(args.yaml, args.ts)
    if diffs:
        sys.stderr.write("check_ts_sync: расхождение YAML ↔ TS-зеркало:\n")
        for line in diffs:
            sys.stderr.write(f"  - {line}\n")
        sys.stderr.write(
            "\nФикс: править ОБА файла согласованно "
            "(в перспективе — генератор TS из YAML).\n"
        )
        return 1
    print(
        f"check_ts_sync: OK — {len(_FIELDS)} пары "
        f"значений YAML ↔ TS синхронны ({args.yaml} ↔ {args.ts})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())