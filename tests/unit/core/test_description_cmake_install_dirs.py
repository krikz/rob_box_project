"""CMake install(DIRECTORY ...) в rob_box_description ссылается только на существующие каталоги.

Ловит до сборки образов: удалённый каталог (напр. models/ в #4445, ADR-0166)
роняет `colcon build` с «file INSTALL cannot find».
"""
import re
from pathlib import Path

PKG = Path(__file__).resolve().parents[3] / "src" / "rob_box_description"


def _install_dirs():
    text = (PKG / "CMakeLists.txt").read_text(encoding="utf-8")
    dirs = []
    for m in re.finditer(r"install\s*\(\s*DIRECTORY\s+([^)]*?)\s+DESTINATION", text, re.S | re.I):
        dirs += m.group(1).split()
    return dirs


def test_install_directories_exist():
    dirs = _install_dirs()
    assert dirs, "install(DIRECTORY ...) не найден"
    missing = [d for d in dirs if not (PKG / d).is_dir()]
    assert not missing, f"install(DIRECTORY) ссылается на отсутствующие каталоги: {missing}"
