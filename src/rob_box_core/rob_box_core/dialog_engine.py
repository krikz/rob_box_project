"""dialog_engine - флаг движка диалога ``v1 | v2`` (ADR-0150 §11, PR-8 #3702).

Один файл ``dialog_engine.yaml`` (секция ``/**``) на все ноды, которым флаг что-то меняет: launch решает, какая нода
диалога стартует (``v1`` - ``dialogue_node``, ``v2`` - ``dialog_node``), ``tts_node`` подключает вход
``/voice/dialogue/response`` адаптера ``speech_compat`` только при ``v1``, ``telegram_node`` и ``supervisor_node`` при
``v2`` не трогают топики ``dialogue_node`` (их замену дают PR-10/PR-12a/PR-13). Значения и чтение - только здесь
(``rob_box_core`` есть во всех образах Sound Pi, ``rob_box_dialog`` - не во всех).
Дефолт ``v2`` (решение Шифу 23.10.2026).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, List, Mapping, Tuple, Union

PARAM = "dialog_engine"
V1, V2 = "v1", "v2"
ENGINES: Tuple[str, ...] = (V1, V2)
DEFAULT = V2
#: Имя файла флага в каталоге конфигов ``voice_assistant`` (обе копии сверяет тест).
FILE_NAME = "dialog_engine.yaml"


def engine(value: Any) -> str:
    """Проверенное значение флага; чужое - ``ValueError`` (опечатка в yaml не должна молча включить v1)."""
    text = str(value or "").strip().lower()
    if text not in ENGINES:
        raise ValueError(f"{PARAM}: ожидается одно из {ENGINES}, получено {value!r}")
    return text


def declare(node: Any) -> str:
    """Объявить параметр ноды и вернуть проверенное значение."""
    node.declare_parameter(PARAM, DEFAULT)
    return engine(node.get_parameter(PARAM).value)


def from_yaml(path: Union[str, Path]) -> str:
    """Значение из ``dialog_engine.yaml`` (для launch, до старта нод). Файла нет - :data:`DEFAULT`
    (каталог конфигов без флага ведёт себя как дефолт); ключа нет или значение чужое - ``ValueError``."""
    import yaml  # noqa: PLC0415 — нужен только launch-у

    file = Path(path)
    if not file.is_file():
        return DEFAULT
    data = yaml.safe_load(file.read_text(encoding="utf-8")) or {}
    params = (data.get("/**") or {}).get("ros__parameters") or {}
    if PARAM not in params:
        raise ValueError(f"{file}: нет ключа /**.ros__parameters.{PARAM}")
    return engine(params[PARAM])


def launch_nodes(context: Any, dialogue_node: Any, tts_node_args: Mapping[str, Any]) -> List[Any]:
    """``OpaqueFunction`` обоих launch-файлов voice_assistant: ноды, которым важен флаг, в прежнем порядке.

    Флаг читается из ``<config_dir>/dialog_engine.yaml``: ``v1`` — ``dialogue_node`` и ``tts_node``; ``v2`` —
    ``dialog_node`` (свой ``dialog_node.yaml``) на месте ``dialogue_node``, тот не стартует. ``tts_node`` и
    ``dialog_node`` получают значение флага параметром (файла может не быть в старом каталоге конфигов — тогда дефолт).
    """
    from launch.substitutions import LaunchConfiguration  # noqa: PLC0415 — launch есть только в образе
    from launch_ros.actions import Node  # noqa: PLC0415

    config_dir = LaunchConfiguration("config_dir").perform(context)
    path = os.path.join(config_dir, FILE_NAME)
    value = from_yaml(path)
    print(f"[voice_assistant launch] {PARAM}={value} ({path})")
    tts_node = Node(**{**tts_node_args, "parameters": [*tts_node_args["parameters"], {PARAM: value}]})
    if value == V1:
        return [dialogue_node, tts_node]
    dialog_node = Node(**{**tts_node_args, "executable": "dialog_node", "name": "dialog_node",
                          "parameters": [os.path.join(config_dir, "dialog_node.yaml"), {PARAM: value}]})
    return [dialog_node, tts_node]


__all__ = ["DEFAULT", "ENGINES", "FILE_NAME", "PARAM", "V1", "V2", "declare", "engine",
           "from_yaml", "launch_nodes"]
