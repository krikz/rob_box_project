"""ADR-0132 PR-5 — composer.txt получает партитуру/ручки/превью-дисциплину.

Проверяет, что ``prompts/skills/composer.txt`` действительно содержит
(не только тул зарегистрирован, а именно ПРОМПТ обучает пользоваться им):

* правило «читай партитуру, прежде чем сказать готово» (ADR-0132 §3.6);
* таблицу «жалоба → ручка» с однозначным действием на «бас гудит»
  (эксперимент показал разброс интерпретаций между моделями — ADR §3.6);
* дисциплину вызова ``preview_arrangement`` (§3.5): без превью на простое
  «сыграй X», максимум один перед повторной игрой, ноль на DJ-переходе;
* сам тул ``preview_arrangement`` назван по имени (skill-контракт из
  ``test_skill_prompt_contract.py`` требует этого для каждого тула
  скилла, но здесь проверяется КОНКРЕТНО дисциплина превью, а не просто
  факт упоминания слова).
* ``dj.txt`` по-прежнему не называет инструменты composer (контракт
  между скиллами не меняется — ADR §3.6).
"""

from __future__ import annotations

from pathlib import Path

_PROMPTS = Path(__file__).resolve().parents[2] / "prompts"
_COMPOSER = _PROMPTS / "skills" / "composer.txt"
_DJ = _PROMPTS / "skills" / "dj.txt"


def _composer_text() -> str:
    return _COMPOSER.read_text(encoding="utf-8")


def test_composer_prompt_names_preview_arrangement() -> None:
    text = _composer_text()
    assert "preview_arrangement" in text


def test_composer_prompt_tells_to_read_score_before_done() -> None:
    text = _composer_text()
    assert "партитур" in text.lower()
    assert "готово" in text.lower()


def test_composer_prompt_has_complaint_to_knob_table() -> None:
    text = _composer_text()
    assert "ЖАЛОБА" in text and "РУЧКА" in text
    # Эксперимент (ADR §3.6): «бас гудит» без уточнения раньше выключал
    # бас целиком у части моделей — правило обязано требовать СНАЧАЛА
    # root, а не off.
    assert "бас гудит" in text
    assert "bass_style=root" in text
    assert "bass_style=off" in text


def test_composer_prompt_has_preview_discipline() -> None:
    text = _composer_text()
    assert "ДИСЦИПЛИНА ПРЕВЬЮ" in text
    # «сыграй X» — сразу играть, без превью.
    assert "БЕЗ `preview_arrangement`" in text or "БЕЗ preview_arrangement" in text
    # DJ-переход — превью не вызывать вовсе.
    assert "DJ-переход" in text and "НЕ вызывай" in text


def test_composer_prompt_keeps_pr2_root_scale_rule() -> None:
    """Ручки root/scale задаются ТОЛЬКО если юзер сам назвал тональность
    (правило из PR-2) — PR-5 не должен было его ослабить."""
    text = _composer_text()
    assert "только если юзер" in text.lower()


def test_dj_prompt_does_not_name_composer_tools() -> None:
    """Контракт из ADR §3.6: dj.txt не называет инструменты composer."""
    text = _DJ.read_text(encoding="utf-8")
    for tool in ("compose_music", "preview_arrangement", "execute_music_code"):
        assert tool not in text, f"dj.txt называет чужой инструмент {tool!r}"
