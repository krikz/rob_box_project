"""Issue #3598 — TG-ход владельца: личность по chat_id, голос комнаты не применяется.

Живой лог 13.10 08:38 UTC (Vision Pi, develop ``0ee1952ba``)::

    [speaker_id_node] 👤 Speaker: 'Литвин' confidence=0.825
    🎤 STT: [TG:495039871] продолжай играть вагнера
    [4] user: '<system_context>\\n  <user_profile>\\n    <name>Литвин</name>
               <voice_confidence>0.82</voice_confidence> …'

Модель сочла сообщение владельца «гостевой репликой, не директорским
приказом». Тест воспроизводит то же состояние ноды: голос Литвина
«горячий», сообщение пришло из чата владельца.
"""

from __future__ import annotations

import asyncio

from rob_box_voice.core.turn_identity import (
    DEFAULT_OWNER_CHAT_IDS,
    OWNER_NAME,
    owner_chat_ids,
    telegram_profile_lines,
    user_profile_lines,
    voice_profile_lines,
)

from .test_dialogue_node import _make_node

OWNER_CHAT = 495039871

#: Снимок _current_speaker из живого лога: последний опознанный голос комнаты.
LITVIN = {
    "is_known": True,
    "name": "Литвин",
    "confidence": 0.82,
    "speaker_id": "3103dcf7-0467-4f80-bef9-3277eb858c0f",
    "epithet": "Спутник",
}


def _profile(ctx: str) -> str:
    return ctx.split("<user_profile>", 1)[1].split("</user_profile>", 1)[0]


def _prepare(node, *, from_tg: bool) -> tuple[str, str]:
    return asyncio.run(
        node._prepare_user_input_context(
            user_input="продолжай играть вагнера",
            from_tg=from_tg,
            speaker_context=None,
        )
    )


def _tg_node():
    n = _make_node({"provider": "minimax"})
    n._current_speaker = dict(LITVIN)
    n._active_tg_chat_id = OWNER_CHAT
    n._speaker_id_enabled = True
    return n


class TestOwnerTelegramTurn:
    def test_owner_turn_gets_owner_profile_not_room_voice(self):
        user_input, ctx = _prepare(_tg_node(), from_tg=True)
        profile = _profile(ctx)
        assert user_input == "[TG] продолжай играть вагнера"
        assert f"<name>{OWNER_NAME}</name>" in profile
        assert "<role>director</role>" in profile
        assert "<channel>telegram</channel>" in profile
        for leaked in ("Литвин", "voice_confidence", "3103dcf7", "Спутник", "epithet"):
            assert leaked not in profile, f"голос комнаты в TG-профиле: {leaked}"

    def test_other_chat_is_guest_without_room_voice(self):
        n = _tg_node()
        n._active_tg_chat_id = 6059238600
        _, ctx = _prepare(n, from_tg=True)
        profile = _profile(ctx)
        assert "<name>unknown</name>" in profile
        assert "<role>guest</role>" in profile
        assert "Литвин" not in profile

    def test_tg_turn_does_not_spend_voice_identity_hints(self):
        """Подсказка-гипотеза голоса (#2809) ждёт голосовую реплику, а не TG."""
        n = _tg_node()
        n._pending_identity_hint = {"kind": "contested"}
        _, ctx = _prepare(n, from_tg=True)
        assert "identity_question_rule" not in ctx
        assert n._pending_identity_hint == {"kind": "contested"}


class TestVoiceTurnUnchanged:
    def test_voice_profile_still_from_biometry(self):
        n = _make_node({"provider": "minimax"})
        n._current_speaker = dict(LITVIN)
        profile = _profile(n._build_dynamic_system_context())
        assert "<name>Литвин</name>" in profile
        assert "<voice_confidence>0.82</voice_confidence>" in profile
        assert "<speaker_id>3103dcf7-0467-4f80-bef9-3277eb858c0f</speaker_id>" in profile
        assert '<epithet internal="true">Спутник</epithet>' in profile
        assert "telegram" not in profile

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
