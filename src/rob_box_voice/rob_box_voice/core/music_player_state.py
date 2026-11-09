"""Временный псевдоним (ADR-0167 Ш1г-1): модуль переехал в ``rob_box_dialog_host.music_player_state``.

Старый путь отдаёт тот же объект модуля (``sys.modules``): импорт и ``mock.patch("rob_box_voice.core.music_player_state.X")`` попадают в настоящий
модуль, второй копии кода нет. Удаляет Ш1г-2 - импорты потребителей переходят на ``rob_box_dialog_host``; список
псевдонимов сверяет ``src/rob_box_dialog_host/test/test_moved_from_voice_0167.py``.
"""

import importlib
import sys

sys.modules[__name__] = importlib.import_module("rob_box_dialog_host.music_player_state")
