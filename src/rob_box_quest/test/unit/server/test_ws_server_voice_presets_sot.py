"""Regression-тест для issue #2240: ws_server не должен объявлять
свою копию whitelist'а пресетов/языков, а импортировать из
rob_box_core.bridge_protocol (single source of truth).

Этот guard — против «третьего раунда»: раньше копия-tuple жила
в ws_server.py и supervisor_node.py. Когда кто-то добавлял новый
пресет в voice_presets.yaml + bridge_protocol, но забывал одну из
копий, ws_server отвечал voice_set_nack (`invalid_voice_preset`).
UI получал «применилось», а оператор получал молчаливый отказ.

DoD карточки: «git grep VOICE_PRESET_IDS src/ → объявление ровно одно».
Дополнительно — runtime-проверка: ws_server.VOICE_PRESET_IDS is
catalog.VOICE_PRESET_IDS (а не просто set-equal: re-export гарантирует,
что добавление в каталог мгновенно подхватывается в ws_server).

Запуск: cd src/rob_box_quest && PYTHONPATH=../rob_box_core:. \\
    python3 -m pytest test/unit/server/test_ws_server_voice_presets_sot.py -v
"""

from __future__ import annotations

import ast
from pathlib import Path

from rob_box_core.bridge_protocol import (
    VOICE_LANGUAGES as CATALOG_LANGUAGES,
    VOICE_PRESET_IDS as CATALOG_PRESETS,
)
from rob_box_quest.server.ws_server import (
    VOICE_LANGUAGES,
    VOICE_PRESET_IDS,
)


WS_SERVER_PY = (
    Path(__file__).resolve().parents[4]
    / "rob_box_quest"
    / "rob_box_quest"
    / "server"
    / "ws_server.py"
)


class TestVoicePresetsSingleSourceOfTruth:
    """Issue #2240: ws_server импортирует whitelist из канона, а не дублирует."""

    def test_ws_server_preset_ids_are_catalog_object(self) -> None:
        """ws_server.VOICE_PRESET_IDS — это тот же объект, что в каталоге.

        Если когда-нибудь кто-то вернёт локальное объявление-tuple,
        этот тест сломается (другой объект → добавление в каталог
        не подхватывается ws_server'ом автоматически).
        """
        assert VOICE_PRESET_IDS is CATALOG_PRESETS, (
            "ws_server.VOICE_PRESET_IDS должен быть re-export из "
            "rob_box_core.bridge_protocol.VOICE_PRESET_IDS, а не копией."
        )

    def test_ws_server_languages_are_catalog_object(self) -> None:
        assert VOICE_LANGUAGES is CATALOG_LANGUAGES, (
            "ws_server.VOICE_LANGUAGES должен быть re-export из "
            "rob_box_core.bridge_protocol.VOICE_LANGUAGES, а не копией."
        )

    def test_ws_server_no_local_tuple_definition(self) -> None:
        """AST-проверка: в ws_server.py НЕТ литерала-tuple с именем
        VOICE_PRESET_IDS / VOICE_LANGUAGES. Разрешён ТОЛЬКО import."""
        src = WS_SERVER_PY.read_text(encoding="utf-8")
        tree = ast.parse(src)
        for node in tree.body:
            if not isinstance(node, ast.Assign):
                continue
            for target in node.targets:
                if not isinstance(target, ast.Name):
                    continue
                if target.id not in {"VOICE_PRESET_IDS", "VOICE_LANGUAGES"}:
                    continue
                # Module-level assignment с правой частью-Tuple → это
                # локальное объявление копии, нарушение SoT-инварианта.
                assert not isinstance(node.value, ast.Tuple), (
                    f"{WS_SERVER_PY.name} локально объявляет "
                    f"{target.id} = (...): это нарушение SoT-инварианта "
                    "issue #2240. Импортируйте из rob_box_core.bridge_protocol."
                )
