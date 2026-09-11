"""Регрессия: ``slice_policy.yaml`` обязан доезжать до install-дерева.

Issue #1998 §6.2 (срез на транспорте) положил политику
``sender → slice → tool`` в ``rob_box_mcp_tools/data/slice_policy.yaml`` и
читает её через ``importlib.resources.files("rob_box_mcp_tools.data")``.

Это **ресурс пакета**, а не ament-share. Значит его доставку обеспечивает
``package_data`` в ``setup.py`` — и ничто другое: ``find_packages()``
собирает только ``*.py``, а ``data_files`` кладёт файлы в
``share/<pkg>/``, откуда ``importlib.resources`` их не видит.

Прод-образ (``docker/vision/voice_assistant/Dockerfile``) собирается
**без** ``--symlink-install`` — это записано там явным комментарием, — то
есть пакет реально устанавливается, а не линкуется на исходники. Поэтому
пропуск ``package_data`` не ловится ни одним обычным тестом: из дерева
исходников (где живёт CI) YAML читается всегда.

Цена пропуска: ``load_default_authority()`` бросает ``ConfigError``,
``mcp_server`` уходит в fail-closed с пустой политикой и блокирует **все**
инструменты у **всех** sender'ов — весь tool-слой агента молча умирает.

Ровно этот класс поломки в проекте уже стрелял дважды: ``prompts/`` у
супервизора (коммит ``6bb0f999``) и ``wake_words.yaml`` (issue #2022).
Тест — сторож, чтобы третьего раза не было.
"""

from __future__ import annotations

import ast
import pathlib

import pytest

# test/unit/test_x.py → parents[2] == src/rob_box_mcp_tools
_PKG_ROOT = pathlib.Path(__file__).resolve().parents[2]
_SETUP_PY = _PKG_ROOT / "setup.py"
_DATA_DIR = _PKG_ROOT / "rob_box_mcp_tools" / "data"


def _setup_kwargs() -> dict:
    """Достать kwargs вызова ``setup(...)`` из setup.py, не исполняя его.

    Исполнять setup.py в тесте нельзя (он дёрнет setuptools и попробует
    разобрать argv pytest'а), поэтому разбираем AST и возвращаем узлы
    аргументов как есть — вызывающему нужен только их литеральный разбор.
    """
    tree = ast.parse(_SETUP_PY.read_text(encoding="utf-8"), filename=str(_SETUP_PY))
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "setup"
        ):
            return {kw.arg: kw.value for kw in node.keywords if kw.arg}
    pytest.fail(f"вызов setup(...) не найден в {_SETUP_PY}")


class TestSlicePolicyPackaging:
    def test_data_dir_is_a_real_package(self):
        """``importlib.resources.files`` требует пакет, а не просто папку."""
        assert (_DATA_DIR / "__init__.py").is_file(), (
            "rob_box_mcp_tools/data/__init__.py пропал — importlib.resources "
            "не сможет найти пакет rob_box_mcp_tools.data"
        )

    def test_slice_policy_yaml_exists(self):
        assert (_DATA_DIR / "slice_policy.yaml").is_file()

    def test_setup_py_declares_package_data_for_data_pkg(self):
        """Главный сторож: без этой строки YAML не доедет до робота."""
        kwargs = _setup_kwargs()
        assert "package_data" in kwargs, (
            "setup.py не объявляет package_data — slice_policy.yaml не "
            "попадёт в install-дерево, и mcp_server уйдёт в fail-closed "
            "(все инструменты заблокированы). См. docstring этого модуля."
        )
        package_data = ast.literal_eval(kwargs["package_data"])
        assert "rob_box_mcp_tools.data" in package_data, (
            "package_data не покрывает пакет rob_box_mcp_tools.data; "
            f"объявлено: {sorted(package_data)}"
        )
        patterns = package_data["rob_box_mcp_tools.data"]
        assert any(p.endswith(".yaml") for p in patterns), (
            f"package_data['rob_box_mcp_tools.data'] = {patterns} — "
            "ни один шаблон не покрывает *.yaml"
        )

    def test_every_yaml_in_data_dir_is_covered(self):
        """Новый YAML рядом с политикой не должен тихо остаться за бортом."""
        kwargs = _setup_kwargs()
        patterns = ast.literal_eval(kwargs["package_data"])["rob_box_mcp_tools.data"]
        uncovered = [
            f.name
            for f in _DATA_DIR.glob("*.yaml")
            if not any(pathlib.PurePath(f.name).match(p) for p in patterns)
        ]
        assert not uncovered, (
            f"YAML-ресурсы {uncovered} не покрыты ни одним шаблоном "
            f"package_data {patterns}"
        )

    def test_rtttl_archive_exists_and_is_packaged(self):
        """Архив мелодий (10460 RTTTL) обязан доезжать до install-дерева."""
        assert (_DATA_DIR / "rtttl_melodies.jsonl.gz").is_file(), (
            "rtttl_melodies.jsonl.gz пропал из data/ — compose_music и "
            "search_melody не найдут ни одной мелодии"
        )
        kwargs = _setup_kwargs()
        patterns = ast.literal_eval(kwargs["package_data"])["rob_box_mcp_tools.data"]
        assert any(
            p.endswith(".jsonl.gz") or p.endswith("*.jsonl.gz") for p in patterns
        ), (
            f"package_data['rob_box_mcp_tools.data'] = {patterns} — "
            "архив rtttl_melodies.jsonl.gz не покрыт"
        )

    def test_not_zip_safe(self):
        """zip-egg ломает чтение ресурсов с диска на части путей установки."""
        kwargs = _setup_kwargs()
        assert "zip_safe" in kwargs, "setup.py должен явно объявлять zip_safe"
        assert ast.literal_eval(kwargs["zip_safe"]) is False, (
            "пакет читает slice_policy.yaml с диска — zip_safe должен быть False"
        )

    def test_default_authority_loads(self):
        """Сам ресурс парсится и даёт непустую политику."""
        from rob_box_mcp_tools.slice_authority import load_default_authority

        authority = load_default_authority()
        assert authority.known_senders, (
            "в slice_policy.yaml не объявлено ни одного sender'а"
        )
        assert authority.known_slices, (
            "в slice_policy.yaml не объявлено ни одного среза"
        )
