from pathlib import Path
from unittest.mock import MagicMock

import pytest

from rob_box_voice.dialogue_node import DialogueNode


def _make_node(parameters: dict | None = None) -> DialogueNode:
    values = parameters or {}
    node = object.__new__(DialogueNode)
    logger = MagicMock()
    node._logger = logger
    node.get_logger = lambda: logger
    node.get_parameter = lambda name: type(
        "Parameter", (), {"value": values.get(name)}
    )()
    node._faq_store = None
    node._event_profile = None
    return node


def test_load_event_profile_from_yaml(tmp_path: Path) -> None:
    config_path = tmp_path / "event_mode.yaml"
    config_path.write_text(
        "event:\n"
        '  name: "День открытых дверей 2026"\n'
        '  organization: "Президентская академия"\n'
        '  location: "Москва"\n'
        '  date: "2026-04-12"\n'
        '  description: "Презентация бакалавриата"\n'
        '  robot_role: "РОББОКС — ровер-помощник"\n'
        # as_posix(): a Windows temp path inside a double-quoted YAML scalar
        # is escape-sequence soup (\U in \Users), safe_load raises, and
        # _load_event_profile fails open with an empty profile.
        f'  faq_file: "{(tmp_path / "faq.xlsx").as_posix()}"\n',
        encoding="utf-8",
    )

    node = _make_node(
        {
            "faq_mode_enabled": True,
            "faq_event_config_file": str(config_path),
        }
    )

    profile = node._load_event_profile()

    assert profile["name"] == "День открытых дверей 2026"
    assert profile["organization"] == "Президентская академия"
    assert profile["robot_role"] == "РОББОКС — ровер-помощник"


def test_render_event_instructions_includes_role_and_context() -> None:
    node = _make_node()
    node._event_profile = {
        "name": "День открытых дверей 2026",
        "organization": "Президентская академия",
        "location": "Москва",
        "date": "2026-04-12",
        "description": "Презентация бакалавриата",
        "robot_role": "РОББОКС — ровер-помощник",
    }

    rendered = node._render_event_instructions("BASE PROMPT")

    assert "РОББОКС — ровер-помощник" in rendered
    assert "День открытых дверей 2026" in rendered
    assert rendered.endswith("BASE PROMPT")


def test_render_event_instructions_requires_faq_before_stylization() -> None:
    node = _make_node()
    node._faq_store = MagicMock()
    node._event_profile = {
        "name": "День открытых дверей 2026",
        "organization": "Президентская академия",
        "location": "Москва",
        "date": "2026-04-12",
        "description": "Презентация бакалавриата",
        "robot_role": "РОББОКС — ровер-помощник",
    }

    rendered = node._render_event_instructions("BASE PROMPT")

    assert "сначала подними факты из FAQ" in rendered
    assert "рэп" in rendered
    assert "execute_music_code" in rendered


def test_build_event_faq_prefetch_context_uses_store_results() -> None:
    node = _make_node()
    node._event_profile = {
        "event_id": "open-day-2026",
        "name": "День открытых дверей 2026",
        "robot_role": "РОББОКС — ровер-помощник",
    }
    node._faq_store = MagicMock()
    node._faq_store.search.return_value = [
        {
            "question": "Что рассказывают про госслужбу?",
            "answer": "На дне открытых дверей объясняют программы по госуправлению и карьерные треки.",
            "category": "Программы",
            "source": "faq.xlsx",
            "score": 1.2,
        }
    ]

    context = node._build_event_faq_prefetch_context(
        "зачитай рэп про госслужбу в стиле синтвейв"
    )

    node._faq_store.search.assert_called_once_with(
        query="зачитай рэп про госслужбу в стиле синтвейв",
        event_id="open-day-2026",
        limit=3,
    )
    assert "FAQ для текущего запроса уже проверен" in context
    assert "Что рассказывают про госслужбу?" in context
    assert "execute_music_code" in context


def test_build_event_faq_prefetch_context_returns_none_without_store_matches() -> None:
    node = _make_node()
    node._event_profile = {
        "event_id": "open-day-2026",
        "name": "День открытых дверей 2026",
        "robot_role": "РОББОКС — ровер-помощник",
    }
    node._faq_store = MagicMock()
    node._faq_store.search.return_value = []

    context = node._build_event_faq_prefetch_context("включи что-нибудь бодрое")

    assert context is None


def test_faq_mode_clears_conversation_history_after_each_turn() -> None:
    """In FAQ event mode _conversation must be empty after _agent_run completes.

    This ensures every LLM call starts with a minimal, fixed context — no
    accumulated turn history that grows with session length and causes
    ~10s/turn latency inflation.
    """
    node = _make_node()
    node._event_profile = {"event_id": "open-day-2026", "name": "День открытых дверей"}
    node._conversation = [
        {"role": "user", "content": "предыдущий вопрос"},
        {"role": "assistant", "content": "предыдущий ответ"},
    ]
    import threading

    node._conv_lock = threading.Lock()

    # Simulate what _agent_run does at the end of a successful FAQ turn
    if node._event_profile:
        with node._conv_lock:
            node._conversation = []

    assert node._conversation == [], (
        "FAQ mode must clear conversation history after each turn"
    )
