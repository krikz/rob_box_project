"""БД памяти инструментов голоса: путь к файлу знает только этот модуль (#4082, ADR-0055, ADR-0128).

Что в файле. Один SQLite на томе ``/data`` голосового контейнера, владелец - ``mcp_server``: факты о людях
``voice_facts`` (пишет ``memory_save``, читают ``memory_search``/``memory_context``, раздел памяти собеседника
``dialog_node`` и склейка профилей ``speaker_id_node``), карты и вейпоинты, FAQ мероприятия, архив RTTTL и реестр
произведений, история сыгранного (``music_history``). Профили собеседников (``SQLiteVoiceMemory``) живут в другом
файле, ``harness_voice.db``: слить два файла заменой пути нельзя - у ``waypoints`` и ``faq_items`` там другие
схемы, ``CREATE TABLE IF NOT EXISTS`` промолчит, а вставки упадут (ADR-0055 §1.3, ADR-0128 §1.2).

Откуда путь. Env :data:`TOOLS_DB_ENV` (стенд, тесты, реплей) - иначе :data:`TOOLS_DB_DEFAULT`. Раньше умолчание
было вписано в пятнадцати местах трёх пакетов и сверялось тестом-грепом; теперь каждый, кому нужен файл, зовёт
:func:`tools_db_path` (или получает путь от того, кто её позвал - ``rob_box_music`` пакетов не импортирует).

Почему имя файла прежнее. На роботе файл живой (факты людей мастерской, вейпоинты); переименование - это перенос
данных на боевой ``/data``, а не правка кода. Миграции нет, пока имя не меняется; понадобится - она живёт здесь.

E2E. Пока владелец режима (``rob_box_core.e2e_mode``) держит e2e, ``mcp_server`` пишет факты в
:data:`TOOLS_E2E_DB_DEFAULT`, и ``dialog_node`` читает их оттуда же - одно умолчание на обоих.
"""

from __future__ import annotations

import os

TOOLS_DB_ENV = "VOICE_MEMORY_DB_PATH"
TOOLS_DB_DEFAULT = "/data/voice_memory.db"
TOOLS_E2E_DB_DEFAULT = "/data/voice_memory.e2e.db"


def tools_db_path() -> str:
    """Боевой путь БД инструментов: :data:`TOOLS_DB_ENV`, если задан и не пуст, иначе :data:`TOOLS_DB_DEFAULT`."""
    return os.environ.get(TOOLS_DB_ENV, "").strip() or TOOLS_DB_DEFAULT


__all__ = ["TOOLS_DB_DEFAULT", "TOOLS_DB_ENV", "TOOLS_E2E_DB_DEFAULT", "tools_db_path"]
