"""Пробы окна ответа после приветствия (слот 9, Refs #4694).

Таблица входов для ``greeting_window`` + ``address.decide`` / ``answer_to_greeting``.
Живой микрофон и VAD-порог не трогаем: только то, что код уже решает.

Честный счёт (не добивать фиктивными «шумами»):

* **10 положительных** — окно/адресация принимает (грамматика, голос в окне, вейк).
* **7 отрицательных** — код реально отсекает (пусто, вне TTL, речь не кончилась,
  отмена, флаг выключен, не-respeaker).
* **Шум / STT-мусор без вейка в открытом окне код не отличает от реплики**
  (``answer_to_greeting`` → ``"voice"``) — отдельного порога VAD нет; такие входы
  **не** кладём в отрицательные пробы.
"""

from __future__ import annotations

import types
from typing import Any, Optional, Tuple
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

install_dialog_msgs_stub()

from voice_robot_wake import ROBOT_WAKE  # noqa: E402

from rob_box_dialog import address  # noqa: E402
from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog.address import Addressed, NotAddressed, decide  # noqa: E402
from rob_box_dialog.backlog import Heard  # noqa: E402
from rob_box_dialog.session import Session  # noqa: E402
from rob_box_dialog.speaker_adapter import answer_to_greeting  # noqa: E402
from rob_box_voice.core import greeting_window as gw  # noqa: E402
from rob_box_voice.core.identity_question import publish  # noqa: E402

_FINISHED, _CANCELLED = 4, 5

# Сколько проб реально отличает код (см. docstring и PR).
POSITIVE_PROBE_COUNT = 10
NEGATIVE_PROBE_COUNT = 7
# Шум без нового VAD-порога код не режет — фиктивных «10 шумов» нет.
NOISE_REJECT_PROBE_COUNT = 0


class _Clock:
    t = 1000.0

    def __call__(self) -> float:
        return self.t


def _node(clock: _Clock) -> Any:
    node = types.SimpleNamespace(_session_pub=MagicMock())
    node.get_logger = lambda: MagicMock()
    node._session = Session("personality", clock=clock)
    node._session.subscribe(lambda event: publish(node, event))
    return node


def _greeted(clock: _Clock, name: str = "Денис") -> Any:
    node = _node(clock)
    gw.open_window(node, "p1", name, "g1")
    gw.on_speech_event(node, _FINISHED, "g1", "personality")
    return node


def _heard(
    text: str,
    t: float,
    *,
    source: str = "respeaker",
    faces: Tuple[str, ...] = (),
) -> Heard:
    return Heard(text=text, t=t, source=source, doa_deg=10.0, faces=frozenset(faces))


def _decide(clock: _Clock, node: Any, text: str, after_s: float, faces: Tuple[str, ...] = ()):
    clock.t += after_s
    return decide(_heard(text, clock.t, faces=faces), None, ROBOT_WAKE, None, gw.window(node))


# --- положительные: окно/адресация принимает -------------------------------------------------

# (id, text, after_s, faces, expected_source, answer_tag_or_None)
# answer_tag — метка answer_to_greeting для безвейковых; None — путь с вейком.
_POSITIVE = (
    ("p01_grammar_privet", "привет", 5.0, (), address.GREETING_REPLY, "grammar"),
    ("p02_grammar_privet_bang", "Привет!", 5.0, (), address.GREETING_REPLY, "grammar"),
    ("p03_grammar_dobroe_utro", "доброе утро", 5.0, (), address.GREETING_REPLY, "grammar"),
    ("p04_grammar_kak_dela", "как дела", 5.0, (), address.GREETING_REPLY, "grammar"),
    ("p05_grammar_spasibo", "спасибо", 5.0, (), address.GREETING_REPLY, "grammar"),
    ("p06_grammar_i_tebe", "и тебе", 5.0, (), address.GREETING_REPLY, "grammar"),
    ("p07_grammar_aga", "ага", 5.0, (), address.GREETING_REPLY, "grammar"),
    # Любой непустой текст в окне — ход (метка voice), лицо не нужно (#4509).
    ("p08_voice_any_phrase", "я тут просто мимо шёл", 5.0, (), address.GREETING_REPLY, "voice"),
    # Голое «робот» в окне — GREETING_WAKE, окно переоткроется take_reply (#4509).
    ("p09_greeting_wake", "робот", 5.0, (), address.GREETING_WAKE, None),
    # Вейк + слова — обычная адресация VOICE (окно открыто, фраза с вейком).
    ("p10_wake_command", "робот, как дела", 5.0, (), address.VOICE, None),
)


@pytest.mark.parametrize("probe_id, text, after_s, faces, source, tag", _POSITIVE, ids=[p[0] for p in _POSITIVE])
def test_positive_probe_window_accepts(probe_id, text, after_s, faces, source, tag):
    assert len(_POSITIVE) == POSITIVE_PROBE_COUNT
    clock = _Clock()
    node = _greeted(clock)
    decision = _decide(clock, node, text, after_s, faces=faces)
    assert isinstance(decision, Addressed), f"{probe_id}: expected Addressed, got {decision!r}"
    assert decision.source == source, f"{probe_id}: source={decision.source!r}"
    if tag is not None:
        frame = gw.window(node)
        assert frame is not None, f"{probe_id}: window must stay open before take"
        assert answer_to_greeting(frame, _heard(text, clock.t, faces=faces)) == tag
        taken = gw.take_reply(node, decision.source, _heard(text, clock.t, faces=faces))
        assert taken is not None, f"{probe_id}: take_reply must consume/rearm"
    elif source == address.GREETING_WAKE:
        rearmed = gw.take_reply(node, address.GREETING_WAKE, _heard(text, clock.t))
        assert rearmed is not None and gw.window(node) is not None, f"{probe_id}: wake rearms window"


# --- отрицательные: код реально отсекает (без выдуманного VAD) -------------------------------

# (id, setup, text, after_s, faces, expected_reason_or_None, note)
# setup: "open" | "speaking" | "cancelled" | "flag_off"
# expected_reason — NotAddressed.reason; None — отсев на take_reply (не decide).


def _setup(kind: str, clock: _Clock, monkeypatch: Optional[Any] = None) -> Any:
    if kind == "open":
        return _greeted(clock)
    if kind == "speaking":
        node = _node(clock)
        gw.open_window(node, "p1", "Денис", "g1")
        return node
    if kind == "cancelled":
        node = _node(clock)
        gw.open_window(node, "p1", "Денис", "g1")
        gw.on_speech_event(node, _CANCELLED, "g1", "personality")
        return node
    if kind == "flag_off":
        assert monkeypatch is not None
        monkeypatch.setattr(kn, "GREETING_REPLY_WITHOUT_WAKE", False)
        return _greeted(clock)
    raise AssertionError(kind)


_NEGATIVE = (
    ("n01_empty_text", "open", "", 5.0, (), address.NO_WAKE, "decide"),
    ("n02_whitespace_only", "open", "   ", 5.0, (), address.NO_WAKE, "decide"),
    ("n03_after_ttl", "open", "привет", 20.0, (), address.NO_WAKE, "decide"),
    ("n04_still_speaking", "speaking", "привет", 3.0, (), address.NO_WAKE, "decide"),
    ("n05_cancelled_greeting", "cancelled", "привет", 5.0, (), address.NO_WAKE, "decide"),
    ("n06_flag_off_no_wake", "flag_off", "привет", 5.0, (), address.NO_WAKE, "decide"),
    # take_reply: только respeaker; telegram-источник окно не снимает.
    ("n07_non_respeaker_take", "open", "привет", 5.0, (), None, "take_reply"),
)


@pytest.mark.parametrize(
    "probe_id, setup, text, after_s, faces, reason, path",
    _NEGATIVE,
    ids=[p[0] for p in _NEGATIVE],
)
def test_negative_probe_window_rejects(probe_id, setup, text, after_s, faces, reason, path, monkeypatch):
    assert len(_NEGATIVE) == NEGATIVE_PROBE_COUNT
    assert NOISE_REJECT_PROBE_COUNT == 0, "шум без VAD-порога код не отсекает — не раздувать таблицу"
    clock = _Clock()
    node = _setup(setup, clock, monkeypatch)
    if path == "decide":
        decision = _decide(clock, node, text, after_s, faces=faces)
        assert decision == NotAddressed(reason), f"{probe_id}: got {decision!r}"
        assert (
            gw.window(node) is None
            or setup == "flag_off"
            or (setup == "open" and after_s < 15.0 and not (text or "").strip())
            or setup == "speaking"
        )
        # Пустой текст: окно может остаться открытым (нечего принимать).
        if not (text or "").strip() and setup == "open" and after_s < 15.0:
            assert gw.window(node) is not None
        elif setup in ("speaking", "cancelled") or after_s >= 20.0:
            assert gw.window(node) is None
        return
    # take_reply: окно открыто, фраза не с respeaker — кадр не снимаем.
    assert path == "take_reply"
    clock.t += after_s
    heard = _heard(text, clock.t, source="telegram", faces=faces)
    before = gw.window(node)
    assert before is not None
    assert gw.take_reply(node, address.GREETING_REPLY, heard) is None
    assert gw.window(node) is before


def test_probe_inventory_is_honest():
    """Счётчик для PR: сколько проб реально, без добивки шумом."""
    assert len(_POSITIVE) == POSITIVE_PROBE_COUNT == 10
    assert len(_NEGATIVE) == NEGATIVE_PROBE_COUNT == 7
    assert NOISE_REJECT_PROBE_COUNT == 0
    # Контроль: типичный «шум» STT в открытом окне сейчас — ход (voice), не отсев.
    clock = _Clock()
    node = _greeted(clock)
    noise = "ааа шшш м-м-м"
    decision = _decide(clock, node, noise, 5.0)
    assert decision == Addressed(noise, address.GREETING_REPLY)
    assert answer_to_greeting(gw.window(node), _heard(noise, clock.t)) == "voice"
