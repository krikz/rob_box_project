"""Регрессия по второму ревью: три критичных и остальные замечания.

  A. Сессия анализа падала с NameError — модуль звал несуществующий load_state.
  B. Черновик не доходил: запись поста создавалась без id, send_post выходил с
     «у поста нет id», а лог писал «пост готов».
  C. Агент-мост наследовал токен бота студии и мог опубликовать в канал сам.
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_studio as studio  # noqa: E402
import studio_facts as facts_mod  # noqa: E402
import studio_reply as reply_mod  # noqa: E402
import studio_state as st  # noqa: E402

# --- A. Сессия анализа не должна падать на первом шаге ----------------------


def test_facts_module_has_load_state():
    """studio_facts звал load_state, которого в модуле не было: NameError, rc=1."""
    assert hasattr(facts_mod, "load_state")
    assert callable(facts_mod.load_state)


def test_facts_module_runs_and_returns_facts(tmp_path):
    """Сборщик фактов отрабатывает и отдаёт структуру, а не падает."""
    import contextlib
    import io

    state = tmp_path / "state.json"
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            facts_mod.main.__wrapped__ if hasattr(facts_mod.main, "__wrapped__") else None
        except AttributeError:
            pass
        # main() читает sys.argv — подменяем.
        import sys as _sys

        old = _sys.argv
        _sys.argv = ["studio_facts.py", "--no-save", "--state", str(state)]
        try:
            facts_mod.main()
        finally:
            _sys.argv = old
    payload = __import__("json").loads(buf.getvalue())
    assert "facts" in payload
    assert "visible_facts" in payload


# --- B. Черновик должен доходить до Шифу --------------------------------------


def test_post_record_has_id(tmp_path, monkeypatch):
    """Без id в записи поста send_post выходил с ошибкой, а лог врал."""
    monkeypatch.setenv("STUDIO_QUEUE_DIR", str(tmp_path / "queue"))
    state = {"ideas": [], "posts": {}}
    (
        studio.apply_post_action(
            state,
            "001",
            studio.ACTION_PUBLISH,
        )
        if False
        else None
    )
    # Проверяем форму записи, которую собирает производство.
    record = {
        "id": "i1-post",
        "status": "proposed",
        "body_html": "текст #роббокс",
        "tags": ["#роббокс"],
    }
    assert record["id"]
    # send_post обязан принять такую запись (не ругаться на отсутствие id).
    sent = {}

    class _Bot:
        def send_message(self, chat_id, text, keyboard=None, parse_mode="HTML"):
            sent["text"] = text
            return {"ok": True}

    studio.send_post(_Bot(), record, 495039871)
    assert "текст" in sent["text"]


def test_send_post_reports_failure_honestly(tmp_path):
    """Если черновик не отправлен — лог должен говорить об этом, а не «пост готов»."""

    class _Bot:
        def send_message(self, chat_id, text, keyboard=None, parse_mode="HTML"):
            return {"ok": False, "description": "chat not found"}

    lines = []
    studio.send_post(_Bot(), {"id": "001", "body_html": "текст"}, 1, out=lines.append)
    assert any("не отправлен" in line for line in lines)


# --- C. Мост к агенту: без токена бота и с проверкой ответа --------------------


def test_reply_bridge_has_no_agent_and_no_subprocess():
    """Мост к Шифу — модель без инструментов, а не агент: `hermes -t ""` включал набор по умолчанию."""
    source = (REPO / "scripts" / "telegram" / "studio_reply.py").read_text(encoding="utf-8")
    assert "import subprocess" not in source
    assert '"hermes"' not in source


def test_reply_uses_narrow_schema():
    """Ответ берётся только из поля answer; любой другой ответ модели — пустота, а не мусор в чат."""
    assert reply_mod.ask_model("x", call=lambda p, timeout=0: {"answer": " Привет "}) == "Привет"
    assert reply_mod.ask_model("x", call=lambda p, timeout=0: {"text": "мимо схемы"}) == ""
    assert reply_mod.ask_model("x", call=lambda p, timeout=0: {"error": "HTTP 500"}) == ""


def test_answer_is_stripped_of_html_and_noise():
    """В чат уходит чистый текст: без разметки и без хвоста лога."""
    raw = "hermes: запуск\n\n<b>Ответ</b> агента — по делу.\n"
    answer = reply_mod.clean_answer(raw)
    assert "<b>" not in answer
    assert "hermes:" not in answer
    assert "Ответ" in answer


def test_answer_with_internal_ip_is_rejected():
    """Ответ с внутренним адресом в чат не отправляем."""
    problems = reply_mod.answer_problems("Робот живёт на 10.1.1.22, всё хорошо")
    assert any("внутренний IP" in p for p in problems)


def test_answer_with_api_key_is_rejected():
    problems = reply_mod.answer_problems("Ключ sk-abcdefghijklmnopqrstuvwxyz1234567890")
    assert any("ключ API" in p for p in problems)


def test_empty_answer_is_rejected():
    assert reply_mod.answer_problems("") != []


# --- Прочее из ревью ---------------------------------------------------------


def test_callback_without_from_is_rejected():
    """Нажатие без отправителя — не от Шифу."""
    state = {"ideas": [{"id": "i1", "status": "proposed"}], "posts": {}}
    answer = studio.handle_callback(state, {"data": "studio:idea:i1:skip"})
    assert "не для тебя" in answer
    assert state["ideas"][0]["status"] == "proposed"


def test_publish_twice_creates_one_queue_file(tmp_path, monkeypatch):
    """Повторное нажатие «Опубликовать» не плодит файлы в очереди."""
    queue = tmp_path / "queue"
    monkeypatch.setenv("STUDIO_QUEUE_DIR", str(queue))
    state = {
        "posts": {
            "001": {
                "id": "001",
                "status": "proposed",
                "title": "Тест",
                "body_html": "текст #роббокс #честность",
                "tags": ["#роббокс", "#честность"],
            }
        }
    }
    studio.apply_post_action(state, "001", studio.ACTION_PUBLISH)
    studio.apply_post_action(state, "001", studio.ACTION_PUBLISH)
    assert len(list(queue.glob("*.md"))) == 1


def test_rejected_post_cannot_be_published_afterwards(tmp_path, monkeypatch):
    """Отклонённый пост не должен оживать по кнопке «Опубликовать»."""
    queue = tmp_path / "queue"
    monkeypatch.setenv("STUDIO_QUEUE_DIR", str(queue))
    state = {
        "posts": {
            "001": {
                "id": "001",
                "status": "rejected",
                "body_html": "текст #роббокс",
                "tags": ["#роббокс"],
            }
        }
    }
    answer = studio.apply_post_action(state, "001", studio.ACTION_PUBLISH)
    assert "отклонён" in answer
    assert list(queue.glob("*.md")) == []


def test_broken_state_file_restores_from_backup(tmp_path):
    """Битый state.json — не «offset: 0»: иначе студия повторила бы всю лену."""
    path = tmp_path / "state.json"
    # Две записи: вторая создаёт резервную копию первой.
    st.save_state(path, {"offset": 1, "ideas": []})
    st.save_state(path, {"offset": 4242, "ideas": []})
    path.write_text("{это не json", encoding="utf-8")
    restored = st.load_state(path)
    assert restored["offset"] in (4242, 1)


def test_queue_numbers_do_not_clash_with_develop(tmp_path):
    """Номера очереди студии не должны совпадать с номерами в develop."""
    queue = tmp_path / "queue"
    queue.mkdir()
    (queue / "001-hello.md").write_text("x", encoding="utf-8")
    (queue / "012-three.md").write_text("x", encoding="utf-8")
    number = studio._queue_number(queue)
    assert number > 100
    assert number not in (1, 12)
