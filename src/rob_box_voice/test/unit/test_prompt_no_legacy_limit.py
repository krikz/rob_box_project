"""Длина реплики — одна цифра, из ``knowledge`` (issue #1377, перевод на dialog v2 — #4354 B1).

#1377: промпт учил «не больше 150 символов» на вызов ``speak_text``, а рантайм резал по другой цифре — модель
дробила ответ. В v2 тула ``speak_text`` нет: речь — поле ``speech`` команды, потолок — ``knowledge.SPEECH_MAX_CHARS``,
и модели его называет фрагмент формы команды (``knowledge.prompt_fragment``), который мост дописывает к системному
промпту (``command_bridge.mode_note``). Файл промпта Личности своей цифры не держит — второй цифре негде разойтись
с первой (ADR-0148: знание живёт в одном модуле).

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/test_prompt_no_legacy_limit.py
"""

from __future__ import annotations

import re
from pathlib import Path

from rob_box_dialog import knowledge as kn
from rob_box_dialog import prompts
from rob_box_dialog.speech_channel import ModeRequest
from rob_box_dialog_host import command_bridge as cb

#: Промпт Личности - основа ядра (ADR-0167 Ш1б-3) с персоной профиля; правила хода - в основе.
MASTER_PROMPT = Path(prompts.__file__).with_name(prompts.BASE_FILE)

#: Цифра-потолок символов в тексте («150 characters», «max 200 символов»): паузы вида «150ms» не считаются.
_CHAR_CAP = re.compile(r"\b\d{2,4}\s*(characters?|chars?|символ\w*|знак\w*)", flags=re.IGNORECASE)


def test_personality_prompt_file_states_no_character_cap_of_its_own() -> None:
    """Потолок длины реплики модели называет только фрагмент knowledge, не файл промпта."""
    hits = _CHAR_CAP.findall(MASTER_PROMPT.read_text(encoding="utf-8"))
    assert hits == [], f"{prompts.BASE_FILE} держит свою цифру длины: {hits!r}"


def test_request_note_names_the_speech_cap_from_knowledge() -> None:
    """Добавка моста к системному промпту называет ``SPEECH_MAX_CHARS`` — ту же цифру, что проверяет валидатор."""
    note = cb.mode_note("tool_offered", ModeRequest(mode="tool_offered"))
    speech = next(line for line in note.splitlines() if line.startswith("- speech:"))
    assert f"≤ {kn.SPEECH_MAX_CHARS} символов" in speech
