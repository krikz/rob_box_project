"""declare_parameter() type must match the YAML that actually feeds it.

Живой инцидент 31.08/01.09.2026: tts_node падал в рестарт-цикле, робот
вообще не мог говорить. Лог с робота, 123 таких падения за прогон::

    tts_node.py, line 748, in __init__
        self.declare_parameter("minimax_pitch", "")  # semitones; "" → не задан
    rclpy.exceptions.InvalidParameterTypeException:
        Trying to set parameter 'minimax_pitch' to '0' of type 'INTEGER',
        expecting type 'STRING'
    [ERROR] [tts_node-5]: process has died [exit code 1] → перезапуск → снова падает

Код объявляет ``minimax_pitch`` строкой (``""`` = «не задан»; прямой
``int("")`` уронил бы код, поэтому приведение типа — отдельная функция).
А в ``tts_node.yaml`` этот ключ был задан ДВАЖДЫ в одном YAML-документе:
один раз как ``0`` (int), один раз как ``''`` (str) — под одним и тем же
именем ноды. ``yaml.safe_load`` молча берёт последнее вхождение (тут это
случайно была правильная строка), но живой crash показывает, что
``rcl_yaml_param_parser`` (C-парсер ROS2, не PyYAML) взял ПЕРВОЕ —
внутренне непротиворечивого поведения для дублирующихся YAML-ключей
попросту не существует, это undefined по духу YAML-спеки. Значит сам
факт дублирования ключа в ``ros__parameters`` — уже баг, независимо от
того, что нашёл бы ``yaml.safe_load``.

Этот тест закрывает ДВА инварианта одним и тем же классом ошибки:

1. Ни у одной ноды нет задвоенного ключа в одном YAML-документе
   (обнаруживает именно тот баг, что уронил tts_node — ``yaml.safe_load``
   его прячет, тест — нет).
2. Тип default-значения в ``declare_parameter(name, default)`` совпадает
   с типом значения в YAML, когда параметр там задан (иначе ROS2 роняет
   ноду той же ``InvalidParameterTypeException`` при старте).

Сверяются только литеральные объявления (``declare_parameter("x", 0)``)
и литеральные YAML-скаляры — динамические имена (``f"{provider}.model"``)
и вычисляемые дефолты (``CHUNK_LIMITS[...]``) пропускаются: их нельзя
сверить статическим анализом без импорта модуля (а модули тянут rclpy).

Не требует ROS2 — только файлы репозитория (ast + PyYAML).
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest
import yaml


def _repo_root(start: Path) -> Path:
    for parent in [start, *start.parents]:
        if (parent / "docker").is_dir() and (parent / "src").is_dir():
            return parent
    return start.parents[5]


REPO_ROOT = _repo_root(Path(__file__).resolve())
PKG_DIR = REPO_ROOT / "src" / "rob_box_voice" / "rob_box_voice"
CONFIG_DIRS = [
    REPO_ROOT / "src" / "rob_box_voice" / "config",
    REPO_ROOT / "docker" / "vision" / "config" / "voice_assistant",
]

#: Ноды, у которых есть и declare_parameter(), и YAML-конфиг с тем же
#: базовым именем в обеих директориях выше.
NODE_NAMES = [
    "audio_node",
    "command_node",
    "dialogue_node",
    "led_node",
    "sound_node",
    "speaker_id_node",
    "stt_node",
    "tts_node",
]


def _type_name(value: object) -> str:
    # bool — подкласс int в Python, проверяем ПЕРВЫМ, иначе True/False
    # ошибочно сойдёт за int.
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "str"
    if isinstance(value, list):
        return "list"
    if value is None:
        return "null"
    return type(value).__name__


def _declared_param_types(pyfile: Path) -> dict:
    """{param_name: (type_name, default, lineno)} для литеральных declare_parameter().

    Пропускает объявления с нелитеральным именем (f-строки, например
    ``f"{provider}.model"`` в dialogue_node.py — для них имя параметра
    неизвестно статически) и с нелитеральным дефолтом (константы уровня
    модуля вроде ``CHUNK_LIMITS["minimax"]`` — их пришлось бы импортировать
    модуль, а он тянет rclpy).
    """
    tree = ast.parse(pyfile.read_text(encoding="utf-8"), filename=str(pyfile))
    result: dict = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not (isinstance(func, ast.Attribute) and func.attr == "declare_parameter"):
            continue
        if len(node.args) < 2:
            continue
        key_node, val_node = node.args[0], node.args[1]
        if not (isinstance(key_node, ast.Constant) and isinstance(key_node.value, str)):
            continue  # динамическое имя параметра — не сверяем
        try:
            default = ast.literal_eval(val_node)
        except Exception:
            continue  # нелитеральный дефолт — не сверяем
        result[key_node.value] = (_type_name(default), default, node.lineno)
    return result


class _DuplicateKeyLoader(yaml.SafeLoader):
    """SafeLoader, который сообщает о задвоенных ключах вместо того, чтобы
    молча взять последний (как это делает штатный safe_load)."""


def _construct_mapping_reporting_dups(loader, node, deep=False):
    mapping: dict = {}
    dups: list = []
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            dups.append(key)
        mapping[key] = loader.construct_object(value_node, deep=deep)
    if dups:
        raise ValueError(f"duplicate keys: {dups}")
    return mapping


_DuplicateKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping_reporting_dups
)


def _yaml_files_for(node_name: str):
    for cfg_dir in CONFIG_DIRS:
        path = cfg_dir / f"{node_name}.yaml"
        if path.exists():
            yield path


def _ros_params_block(data: dict, node_name: str) -> dict:
    """Достаёт плоский словарь ros__parameters для данной ноды."""
    if node_name in data:
        block = data[node_name]
    elif len(data) == 1:
        block = next(iter(data.values()))
    else:
        block = next((v for k, v in data.items() if node_name in k), None)
    if not isinstance(block, dict):
        return {}
    params = block.get("ros__parameters", {})
    return params if isinstance(params, dict) else {}


def _cases():
    """(node_name, yaml_path) для каждой существующей пары ноды/конфига."""
    cases = []
    for node_name in NODE_NAMES:
        for yaml_path in _yaml_files_for(node_name):
            cases.append(pytest.param(node_name, yaml_path, id=f"{node_name}:{yaml_path.parent.name}"))
    return cases


_CASES = _cases()


def test_setup_actually_found_config_pairs() -> None:
    """Smoke: если пары ноды/YAML не находятся, весь остальной тест пуст."""
    assert len(_CASES) >= len(NODE_NAMES), (
        "найдено подозрительно мало пар нода/YAML — сверь NODE_NAMES "
        "с реальными файлами в config/ и docker/vision/config/voice_assistant/"
    )


@pytest.mark.parametrize("node_name, yaml_path", _CASES)
def test_no_duplicate_keys_in_ros_parameters(node_name: str, yaml_path: Path) -> None:
    """Ни один YAML-параметр-файл не должен объявлять один и тот же ключ
    дважды в одном документе.

    Это ровно тот баг, что уронил tts_node: ``minimax_pitch`` был задан
    и как ``0``, и как ``''`` в одном ``ros__parameters``. Какое из двух
    значений «победит» — решает не Python-код, а внутренности
    ``rcl_yaml_param_parser`` на роботе; полагаться на это нельзя,
    единственный безопасный инвариант — задвоений не быть вовсе.
    """
    try:
        yaml.load(yaml_path.read_text(encoding="utf-8"), Loader=_DuplicateKeyLoader)
    except ValueError as exc:
        pytest.fail(f"{yaml_path}: {exc} — один и тот же ключ задан дважды в ros__parameters")


@pytest.mark.parametrize("node_name, yaml_path", _CASES)
def test_declared_param_type_matches_yaml_value_type(node_name: str, yaml_path: Path) -> None:
    """Тип default в declare_parameter() обязан совпадать с типом значения
    в YAML, когда параметр там реально задан.

    ROS2 не приводит типы параметров сам: если нода объявила параметр
    строкой, а YAML отдаёт число (или наоборот), ``rclpy`` кидает
    ``InvalidParameterTypeException`` прямо из ``__init__`` ноды — процесс
    умирает раньше, чем успевает что-либо сделать, и супервизор ROS2
    перезапускает его по кругу (см. crash-лог tts_node в докстринге
    модуля).
    """
    pyfile = PKG_DIR / f"{node_name}.py"
    declared = _declared_param_types(pyfile)

    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    yaml_params = _ros_params_block(data, node_name)

    mismatches = []
    for pname, (py_type, py_default, lineno) in declared.items():
        if pname not in yaml_params:
            continue  # не задан в YAML → используется дефолт кода, ок
        yaml_type = _type_name(yaml_params[pname])
        if py_type != yaml_type:
            mismatches.append(
                f"  {pname}: declare_parameter (line {lineno}) default={py_default!r} "
                f"({py_type}) vs YAML {yaml_params[pname]!r} ({yaml_type})"
            )

    assert not mismatches, (
        f"{pyfile.name} vs {yaml_path}: тип параметра в declare_parameter() "
        "не совпадает с YAML — ROS2 уронит ноду при старте:\n" + "\n".join(mismatches)
    )
