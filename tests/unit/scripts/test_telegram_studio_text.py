"""Студия канала: сбор фактов и генерация текста. Сеть замокана."""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import studio_facts as facts_mod  # noqa: E402
import studio_text as text_mod  # noqa: E402


# --------------------------------------------------------------------------- выбор модели


def test_m3_thinking_can_be_disabled():
    assert text_mod.thinking_params("MiniMax-M3") == {"thinking": {"type": "disabled"}}


def test_flash_gets_reasoning_effort_not_disabled():
    """M3.1-Flash на thinking=disabled отвечает HTTP 400, а без reasoning_effort
    размышления съедают весь max_tokens и content приходит пустым."""
    params = text_mod.thinking_params("MiniMax-M3.1-Flash")
    assert "thinking" not in params
    assert params["reasoning_effort"] == "low"


def test_default_model_is_flash():
    assert text_mod.DEFAULT_MODEL == "MiniMax-M3.1-Flash"


# --------------------------------------------------------------------------- теги


def test_extract_tags_from_body():
    body = "<b>Привет</b> #навигация и ещё #честность #навигация"
    assert text_mod.extract_tags(body) == ["#навигация", "#честность"]


def test_extract_tags_ignores_unknown_tags():
    assert text_mod.extract_tags("текст #неизвестныйтег") == []


def test_tags_come_from_text_not_from_model():
    """Модель часто не возвращает поле tags — теги берём из текста."""
    idea = {"body_html": "пост #роббокс #дневник", "tags": []}
    tags = text_mod.extract_tags(idea["body_html"])
    assert tags == ["#роббокс", "#дневник"]


# --------------------------------------------------------------------------- проверка по фактам


def make_facts():
    return [
        {"claim": "PR #4502 влит в develop: сторож локализации", "source": "PR #4502"},
        {"claim": "RTAB-Map сбрасывает одометрию после потери", "source": "PR #4503"},
    ]


def test_numbers_must_come_from_facts():
    body = "Робот проехал 48 часов работы"
    problems = text_mod.check_against_facts(body, make_facts())
    assert any("48" in p for p in problems)


def test_numbers_from_facts_pass():
    body = "В PR #4502 сторож локализации не «приехал»"
    problems = text_mod.check_against_facts(body, make_facts())
    assert problems == []


def test_proper_names_must_come_from_facts():
    body = "Мы переписали <code>nav_bench</code> целиком"
    problems = text_mod.check_against_facts(body, make_facts())
    assert any("nav_bench" in p for p in problems)


def test_russian_capitals_are_not_proper_names():
    """Русские слова с заглавной — это начало предложения, а не имя собственное."""
    body = "Проверили. Иначе робот ответил бы неправдой."
    problems = text_mod.check_against_facts(body, make_facts())
    assert problems == []


def test_html_attributes_are_not_proper_names():
    body = '<a href="https://github.com/krikz/rob_box_project/pull/4502">PR #4502</a>'
    problems = text_mod.check_against_facts(body, make_facts())
    assert problems == []


def test_br_tag_is_rejected():
    """Telegram HTML не знает <br> — абзацы разделяют переводом строки."""
    problems = text_mod.validate_post("первый<br>второй", ["#роббокс"], make_facts())
    assert any("<br>" in p for p in problems)


def test_internal_ip_is_rejected():
    problems = text_mod.validate_post("робот живёт на 10.1.1.22 #роббокс", ["#роббокс"], make_facts())
    assert any("внутренний IP" in p for p in problems)


def test_valid_draft_passes():
    problems = text_mod.validate_post(
        "В PR #4502 сторож локализации не считает потерю опоры успехом. #навигация #честность",
        ["#навигация", "#честность"],
        make_facts(),
    )
    assert problems == []


def test_tag_not_in_dictionary_is_rejected():
    problems = text_mod.validate_post("текст #выдуманныйтег", ["#выдуманныйтег"], make_facts())
    assert any("словаре" in p for p in problems)


# --------------------------------------------------------------------------- фильтр «интересного»


def test_technical_pr_is_not_visible():
    assert facts_mod.is_boring("chore(deps): bump the python-deps group")
    assert facts_mod.is_boring("ci: sound SHA tags")
    assert facts_mod.is_boring("docs(channel): инструкция")


def test_product_pr_is_visible():
    assert not facts_mod.is_boring("fix(nav): сторож локализации")
    assert not facts_mod.is_boring("feat(e2e): до 3 попыток на вейк")


def test_visible_topics_detect_navigation():
    assert facts_mod.visible("fix(nav): сторож локализации")
    assert facts_mod.visible("feat(face): one panel_image message")
    assert not facts_mod.visible("chore(deps): bump packages")


# --------------------------------------------------------------------------- вызов MiniMax


def test_call_minimax_without_key_returns_error(monkeypatch):
    monkeypatch.delenv("MINIMAX_API_KEY", raising=False)
    result = text_mod.call_minimax("тест", api_key="")
    assert "error" in result


def test_call_minimax_parses_json(monkeypatch):
    class FakeResponse:
        status_code = 200

        @staticmethod
        def json():
            return {"choices": [{"message": {"content": '{"ideas": [{"id": "i1"}]}'}, "finish_reason": "stop"}]}

    class FakeRequests:
        @staticmethod
        def post(url, json=None, headers=None, timeout=None):
            return FakeResponse()

    monkeypatch.setitem(sys.modules, "requests", FakeRequests)
    result = text_mod.call_minimax("тест", api_key="k")
    assert result["ideas"][0]["id"] == "i1"


def test_call_minimax_reports_empty_content(monkeypatch):
    """Пустой content = размышления съели max_tokens: это должен быть понятный отчёт."""

    class FakeResponse:
        status_code = 200

        @staticmethod
        def json():
            return {
                "choices": [{"message": {"content": ""}, "finish_reason": "length"}],
                "usage": {"completion_tokens": 8000},
            }

    class FakeRequests:
        @staticmethod
        def post(url, json=None, headers=None, timeout=None):
            return FakeResponse()

    monkeypatch.setitem(sys.modules, "requests", FakeRequests)
    result = text_mod.call_minimax("тест", api_key="k")
    assert "error" in result
    assert "пустой" in result["error"]


def test_call_minimax_sends_reasoning_effort_for_flash(monkeypatch):
    captured = {}

    class FakeResponse:
        status_code = 200

        @staticmethod
        def json():
            return {"choices": [{"message": {"content": "{}"}}]}

    class FakeRequests:
        @staticmethod
        def post(url, json=None, headers=None, timeout=None):
            captured.update(json or {})
            return FakeResponse()

    monkeypatch.setitem(sys.modules, "requests", FakeRequests)
    text_mod.call_minimax("тест", model="MiniMax-M3.1-Flash", api_key="k")
    assert captured["reasoning_effort"] == "low"
    assert "thinking" not in captured


# --- проверки на выдумку (главный блокер ревью) ------------------------------


def test_invented_russian_name_is_rejected():
    """Имя, которого нет в фактах, — выдумка модели, а не факт."""
    body = "В работе над навигацией Саша проверил потерю опоры"
    problems = text_mod.check_against_facts(body, make_facts())
    assert any("Саша" in p for p in problems)


def test_invented_number_is_rejected():
    """Числа нет в фактах — даже если похожее число есть в номере прогона."""
    facts = [
        {
            "claim": "Сторож локализации не считает потерю опоры успехом",
            "source": "PR #4502",
            "evidence": ["run_id 39985229892"],
        }
    ]
    body = "Сторож локализации работает 12 минут"
    problems = text_mod.check_against_facts(body, facts)
    assert any("12" in p for p in problems)


def test_run_id_is_not_allowed_in_text():
    """Идентификатор прогона в тексте поста — лишнее и потенциально внутреннее."""
    facts = [{"claim": "Сторож локализации работает", "source": "PR #4502", "evidence": []}]
    body = "Прогон 39985229892 прошёл успешно"
    problems = text_mod.check_against_facts(body, facts)
    assert any("39985229892" in p for p in problems)


def test_api_key_in_text_is_rejected():
    """Ключ в тексте — стоп: публиковать такое нельзя."""
    body = "Ключ sk-abcdefghijklmnopqrstuvwxyz1234567890 в конфиге"
    problems = text_mod.validate_post(body, ["#роббокс"], make_facts())
    assert any("ключ API" in p for p in problems)


def test_idea_without_show_or_deadline_is_rejected():
    """По брифу у идеи есть «что показать» и «примерный срок»."""
    idea = {"title": "Тема", "facts": ["Сторож локализации не считает потерю опоры успехом"], "show": "", "deadline": ""}
    problems = text_mod.validate_idea(idea, make_facts())
    assert any("что показать" in p for p in problems)
    assert any("срок" in p for p in problems)


def test_idea_with_invented_source_is_rejected():
    """Поле facts модели проверяется: выдуманный источник — не источник."""
    idea = {
        "title": "Тема",
        "facts": ["Андрей переписал навигацию"],
        "show": "скриншот",
        "deadline": "на следующей неделе",
    }
    problems = text_mod.validate_idea(idea, make_facts())
    assert any("не найден среди фактов" in p for p in problems)
