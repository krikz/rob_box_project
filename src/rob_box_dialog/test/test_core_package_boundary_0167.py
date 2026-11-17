"""Граница ядра: код пакета ``rob-box-dialog-core`` не читает файлы вне пакета (ADR-0167 §2.1, шаг Ш1а).

Ядро, поставленное пакетом без чекаута (колонка, чужой продукт), видит только себя: путь вверх от своего файла
(``parents[N]``, ``"..", ".."``), поиск от текущего каталога (``Path.cwd()``), каталог чужого ROS-пакета
(``get_package_share_directory``), монтирование контейнера робота (``/migrations``, ``docker/…``) там ведут в пустоту.
Свои данные ядро читает через ``importlib.resources``; что даёт продукт - передаёт хост.

Состав - из ``packaging/dialog-core/pyproject.toml`` (каталоги пакетов и ``py-modules``), исходники - из ``src/``
чекаута, поэтому тест идёт в любом окружении. Комментарии и строки документации не считаются.

Известные дыры - :data:`KNOWN_LEAKS`, с числом мест. Число можно только понижать: закрыл дыру - уменьши, новая
дыра - красный тест, а не строчка в списке.
"""

from __future__ import annotations

import io
import re
import tokenize
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[2]
PYPROJECT = SRC.parent / "packaging" / "dialog-core" / "pyproject.toml"

#: ``файл относительно src/<пакет>/`` → (число мест, почему ещё здесь и где закрывается).
#: Пусто с Ш1а-2: каталог пресетов стиля ушёл в профиль продукта (env ``ROB_BOX_VOICE_PRESETS_YAML``), описания
#: сообщений заглушки - в данные пакета (``rob_box_core/dialog_msgs``).
KNOWN_LEAKS: dict = {}

_WALK_UP = re.compile(r"""^[rbu]*['"]\.\.['"]$""")
_ROBOT_PATH = re.compile(r"""(^[rbu]*['"]/(migrations|config|ws|opt)\b)|docker/|/docker\b""")


def _core_files():
    text = PYPROJECT.read_text(encoding="utf-8")
    roots = re.findall(r'"\.\./\.\./src/([a-z_]+)"', text)
    for root in roots:
        pkg = SRC / root / root
        yield from (p for p in pkg.rglob("*.py") if "test" not in p.relative_to(pkg).parts)
    for dotted in re.findall(r'^\s*"(rob_box_(?:voice|mcp_tools)\.[a-z_.]+)",\s*$', text, re.M):
        top = dotted.split(".")[0]
        yield SRC / top / top / Path(*dotted.split(".")[1:]).with_suffix(".py")


def _leaks(path: Path):
    """Места чтения вне пакета: ``(строка, что)`` по токенам кода (без комментариев и докстрингов)."""
    toks = [
        t
        for t in tokenize.generate_tokens(io.StringIO(path.read_text(encoding="utf-8-sig")).readline)
        if t.type not in (tokenize.COMMENT, tokenize.NL, tokenize.NEWLINE, tokenize.INDENT, tokenize.DEDENT)
    ]
    for i, tok in enumerate(toks):
        nxt = toks[i + 1].string if i + 1 < len(toks) else ""
        prev = toks[i - 1].string if i else ""
        if tok.type == tokenize.NAME and tok.string == "parents" and prev == "." and nxt == "[":
            yield tok.start[0], "parents[…]"
        elif (
            tok.type == tokenize.NAME
            and tok.string == "parent"
            and prev == "."
            and nxt == "."
            and i + 2 < len(toks)
            and toks[i + 2].string == "parent"
        ):
            yield tok.start[0], ".parent.parent"
        elif tok.type == tokenize.NAME and tok.string in ("cwd", "getcwd") and prev == "." and nxt == "(":
            yield tok.start[0], tok.string
        elif tok.type == tokenize.NAME and tok.string == "get_package_share_directory" and nxt == "(":
            yield tok.start[0], "share другого пакета"
        elif tok.type == tokenize.STRING and (_WALK_UP.match(tok.string) or _ROBOT_PATH.search(tok.string)):
            if prev not in ("", "INDENT") and not tok.string.startswith(('"""', "'''")):
                yield tok.start[0], tok.string


pytestmark = pytest.mark.skipif(not PYPROJECT.is_file(), reason="нет packaging/ (colcon test_ws копирует только src/)")


def test_core_files_are_found():
    files = list(_core_files())
    assert len(files) > 100 and all(p.is_file() for p in files), [str(p) for p in files if not p.is_file()]


def test_core_reads_no_files_outside_the_package():
    found = {}
    for path in _core_files():
        hits = list(_leaks(path))
        if hits:
            found[path.relative_to(SRC).as_posix().split("/", 1)[1]] = hits
    counts = {name: len(hits) for name, hits in found.items()}
    assert counts == KNOWN_LEAKS, found


def test_detector_sees_each_kind_of_leak(tmp_path):
    sample = tmp_path / "leaky.py"
    sample.write_text(
        "from pathlib import Path\nimport os\n"
        'A = Path(__file__).resolve().parents[3] / "migrations"\n'
        "B = Path(__file__).parent.parent\n"
        'C = os.path.join(os.path.dirname(__file__), "..", "x")\n'
        "D = Path.cwd()\n"
        'E = "/migrations"\n'
        'F = REPO / "docker/sound/config/wake_words.yaml"\n'
        "# parents[2] в комментарии - не место\n",
        encoding="utf-8",
    )
    assert [line for line, _ in _leaks(sample)] == [3, 4, 5, 6, 7, 8]
