"""Регрессия: нажатие кнопки не должно теряться.

Три бага, из-за которых кнопка «Обсудить» молчала:

  1. Ретрай на 409 не срабатывал: Telegram пишет в описании «Conflict: terminated by
     other getUpdates request», и цифры «409» там нет. Смотрели описание — ретрая не было,
     опрос возвращал «новых апдейтов нет».
  2. Смещение не сохранялось при падении обработки: апдейт уже забрали у Telegram,
     и нажатие пропадало навсегда.
  3. Смещение сохранялось только в конце цикла: если агент зависал или ронял опрос,
     нажатие терялось так же.
"""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_studio as studio  # noqa: E402
import studio_state as st  # noqa: E402


class _Response:
    def __init__(self, payload):
        self._payload = payload

    def json(self):
        return self._payload


class _Session:
    """Подставка requests: отдаёт заранее заданные ответы."""

    def __init__(self, payloads):
        self.payloads = list(payloads)
        self.calls = 0

    def post(self, url, json=None, timeout=None):
        self.calls += 1
        payload = self.payloads[min(self.calls - 1, len(self.payloads) - 1)]
        return _Response(payload)


def test_conflict_409_is_retried(monkeypatch):
    """409 — конфликт long-poll: его надо переждать, а не считать «новых апдейтов нет»."""
    session = _Session(
        [
            {"ok": False, "error_code": 409, "description": "Conflict: terminated by other getUpdates request"},
            {"ok": True, "result": [{"update_id": 5, "message": {"text": "привет"}}]},
        ]
    )
    bot = studio.StudioBot(token="x", session=session)
    monkeypatch.setattr("time.sleep", lambda _s: None)
    updates = bot.get_updates(0, timeout=0)
    assert session.calls == 2, "после 409 должен быть повтор"
    assert len(updates) == 1


def test_too_many_requests_429_is_retried(monkeypatch):
    session = _Session(
        [
            {"ok": False, "error_code": 429, "description": "Too Many Requests: retry after 5"},
            {"ok": True, "result": []},
        ]
    )
    bot = studio.StudioBot(token="x", session=session)
    monkeypatch.setattr("time.sleep", lambda _s: None)
    bot.get_updates(0, timeout=0)
    assert session.calls == 2


def test_other_error_is_not_retried():
    """401/400 — повтор не поможет, лишние запросы только вредят."""
    session = _Session([{"ok": False, "error_code": 401, "description": "Unauthorized"}])
    bot = studio.StudioBot(token="x", session=session)
    bot.get_updates(0, timeout=0)
    assert session.calls == 1


def test_offset_is_saved_even_if_handling_fails(tmp_path):
    """Апдейт забрали у Telegram — нажатие нельзя потерять из-за ошибки обработки."""
    path = tmp_path / "state.json"

    class _BoomBot:
        def get_updates(self, offset, timeout=0):
            return [{"update_id": 7, "callback_query": {"id": "cb", "data": "studio:idea:i1:take"}}]

        def answer_callback(self, *a, **k):
            raise RuntimeError("Telegram недоступен")

    with pytest.raises(RuntimeError):
        with st.state_transaction(path) as state:
            state["offset"] = 8
            _BoomBot().answer_callback("cb", "x")
    # Смещение обязано уехать: Telegram этот апдейт больше не отдаст.
    assert st.load_state(path)["offset"] == 8


def test_offset_is_saved_right_after_fetch(tmp_path):
    """Смещение двигается сразу при получении апдейта, а не в конце цикла."""
    path = tmp_path / "state.json"
    saved = []

    class _Bot:
        def get_updates(self, offset, timeout=0):
            return [{"update_id": 42, "callback_query": {"id": "cb", "data": "studio:idea:i1:skip"}}]

        def answer_callback(self, *a, **k):
            return {"ok": True}

        def send_message(self, *a, **k):
            return {"ok": True}

    def _spy(path_):
        original = st.save_state

        def spy(p, state):
            saved.append(state.get("offset"))
            return original(p, state)

        return spy

    original_save = st.save_state
    st.save_state = _spy(path)
    try:
        studio.poll_once(_Bot(), path, timeout=0)
    finally:
        st.save_state = original_save

    assert 43 in saved, "смещение должно писаться сразу после получения апдейта"
    assert st.load_state(path)["offset"] == 43
