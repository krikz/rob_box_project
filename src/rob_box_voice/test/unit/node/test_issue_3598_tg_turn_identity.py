"""Issue #3598 — TG-ход владельца: личность по chat_id, голос комнаты не применяется.

Живой лог 13.10 08:38 UTC (Vision Pi, develop ``0ee1952ba``): модель сочла сообщение владельца «гостевой
репликой, не директорским приказом» — в профиль TG-хода попал голос комнаты.

Здесь — таблица владельцев и строки профиля (``core.turn_identity``). PR-15 (#3711, часть 15p): сквозные проверки
сборки контекста хода в ноде диалога v1 удалены вместе с ней.
"""

from __future__ import annotations

from rob_box_voice.core.turn_identity import (
    DEFAULT_OWNER_CHAT_IDS,
    OWNER_NAME,
    owner_chat_ids,
    telegram_profile_lines,
    user_profile_lines,
    voice_profile_lines,
)

OWNER_CHAT = 495039871

#: Снимок голоса из живого лога: последний опознанный голос комнаты.
LITVIN = {
    "is_known": True,
    "name": "Литвин",
    "confidence": 0.82,
    "speaker_id": "3103dcf7-0467-4f80-bef9-3277eb858c0f",
    "epithet": "Спутник",
}


class TestVoiceProfile:
    def test_unknown_voice_keeps_privacy_note(self):
        lines = voice_profile_lines("", {"confidence": 0.0})
        assert lines[0] == "    <name>unknown</name>"
        assert "privacy_note" in lines[1]


class TestOwnerTable:
    def test_default_owner_is_shifu_chat(self):
        assert owner_chat_ids({}) == DEFAULT_OWNER_CHAT_IDS == frozenset({OWNER_CHAT})

    def test_env_overrides_owner_list(self):
        assert owner_chat_ids({"TELEGRAM_OWNER_CHAT_IDS": " 1, 2 ,x"}) == {1, 2}

    def test_garbage_env_falls_back_to_default(self):
        assert owner_chat_ids({"TELEGRAM_OWNER_CHAT_IDS": "x,"}) == DEFAULT_OWNER_CHAT_IDS

    def test_explicit_owner_set(self):
        assert "<role>director</role>" in "".join(telegram_profile_lines(7, owners=frozenset({7})))
        assert "<role>guest</role>" in "".join(telegram_profile_lines(8, owners=frozenset({7})))

    def test_user_profile_lines_dispatch(self):
        assert user_profile_lines("Литвин", LITVIN, OWNER_CHAT)[0] == f"    <name>{OWNER_NAME}</name>"
        assert user_profile_lines("Литвин", LITVIN, None)[0] == "    <name>Литвин</name>"
