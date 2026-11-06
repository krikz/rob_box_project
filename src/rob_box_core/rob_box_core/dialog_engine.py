"""dialog_engine - ноды диалога в launch voice_assistant (ADR-0150 §11, PR-8 #3702).

Флага ``v1 | v2`` больше нет (#4354 A3): нода v1 удалена в PR-15 (#3711), ветки v1 у ``supervisor_node`` и
``telegram_node`` — в #4354 A3, файла ``dialog_engine.yaml`` нет. Launch стартует ``dialog_node`` и ``tts_node``
(:func:`launch_nodes`). Метки-параметра ``dialog_engine`` у ``dialog_node`` тоже нет: её читала только ветка v1
e2e-харнесса (#4354 A4).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List, Mapping, Tuple, Union

#: Параметры ``dialog_node``, значение которых живёт в yaml другой ноды (одно на обе, launch передаёт его
#: ``dialog_node``): ``(файл, нода, ключ)``. Режим выбиратели пути музыки ``mcp_server.music_chooser`` — путь пения хода
#: Личности (#4079, ``build_sing``) выбирает тот же выбиратель, что путь тула музыки (ревью #4071 N4).
SHARED_PARAMS: Tuple[Tuple[str, str, str], ...] = (("mcp_server.yaml", "mcp_server", "music_chooser"),)


def shared_params(config_dir: Union[str, Path]) -> Dict[str, Any]:
    """Значения :data:`SHARED_PARAMS` из yaml их нод в ``config_dir`` (для launch); файла или ключа нет — параметра нет
    (у ``dialog_node`` — его умолчание)."""
    import yaml  # noqa: PLC0415 — нужен только launch-у

    out: Dict[str, Any] = {}
    for file_name, node, key in SHARED_PARAMS:
        file = Path(config_dir) / file_name
        data = (yaml.safe_load(file.read_text(encoding="utf-8")) or {}) if file.is_file() else {}
        params = (data.get(node) or {}).get("ros__parameters") or {}
        if key in params:
            out[key] = params[key]
    return out


def launch_nodes(context: Any, tts_node_args: Mapping[str, Any]) -> List[Any]:
    """``OpaqueFunction`` обоих launch-файлов voice_assistant: ``dialog_node`` (свой ``dialog_node.yaml`` и параметры,
    общие с другими нодами, :func:`shared_params`) и ``tts_node`` — в прежнем порядке."""
    from launch.substitutions import LaunchConfiguration  # noqa: PLC0415 — launch есть только в образе
    from launch_ros.actions import Node  # noqa: PLC0415

    config_dir = LaunchConfiguration("config_dir").perform(context)
    tts_node = Node(**tts_node_args)
    dialog_node = Node(**{**tts_node_args, "executable": "dialog_node", "name": "dialog_node",
                          "parameters": [os.path.join(config_dir, "dialog_node.yaml"), shared_params(config_dir)]})
    return [dialog_node, tts_node]


__all__ = ["SHARED_PARAMS", "launch_nodes", "shared_params"]
