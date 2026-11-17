"""#4082: склейка профилей в ``speaker_id_node`` переносит факты ``memory_save`` в той же БД инструментов, куда их
пишет ``mcp_server`` - путь из ``rob_box_core.tools_db``, а не своё умолчание ноды.

Приём - как в ``test_e2e_db_isolation.py``: ``SpeakerIdNode`` через ``object.__new__`` (без ROS, resemblyzer),
только поля, которые трогает ``_merge_identity``.
"""

from __future__ import annotations

import sqlite3
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)
for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_core import tools_db  # noqa: E402
from rob_box_voice import speaker_id_node as sid_node  # noqa: E402
from rob_box_voice.utils.speaker_embeddings import SpeakerDatabase  # noqa: E402


def _embedding(seed: int):
    import numpy as np

    v = np.random.default_rng(seed).standard_normal(256).astype("float32")
    return v / np.linalg.norm(v)


def _facts_db(path: Path, speaker_id: str) -> None:
    conn = sqlite3.connect(path)
    conn.execute(
        "CREATE TABLE voice_facts (id INTEGER PRIMARY KEY, fact TEXT, category TEXT, speaker_id TEXT, "
        "created_at REAL, updated_at REAL)"
    )
    conn.execute(
        "INSERT INTO voice_facts (fact, category, speaker_id, created_at, updated_at) "
        "VALUES ('не ест лук', 'preference', ?, 0, 0)",
        (speaker_id,),
    )
    conn.commit()
    conn.close()


@pytest.fixture()
def node(tmp_path):
    instance = object.__new__(sid_node.SpeakerIdNode)
    instance._db = SpeakerDatabase(str(tmp_path / "speakers.db"))
    instance.get_logger = MagicMock(return_value=MagicMock())
    yield instance
    instance._db.close()


@pytest.mark.parametrize("param_value", ["", None])
def test_merge_moves_memory_save_facts_in_the_tools_db(
    node, tmp_path, monkeypatch, param_value
):
    """Параметр не задан (YAML его больше не несёт) - склейка берёт файл ``mcp_server``."""
    src = node._db.register("Денчик", _embedding(1))
    dst = node._db.register("Деньчик", _embedding(2))
    facts = tmp_path / "tools.db"
    _facts_db(facts, src)
    monkeypatch.setenv(tools_db.TOOLS_DB_ENV, str(facts))
    params = {
        "memory_db_path": str(tmp_path / "harness_voice.db"),
        "voice_facts_db_path": param_value,
    }
    node.get_parameter = lambda name: types.SimpleNamespace(value=params[name])

    _, facts_moved = node._merge_identity(src, dst)

    conn = sqlite3.connect(facts)
    owners = [row[0] for row in conn.execute("SELECT speaker_id FROM voice_facts")]
    conn.close()
    assert facts_moved == 1
    assert owners == [dst]
