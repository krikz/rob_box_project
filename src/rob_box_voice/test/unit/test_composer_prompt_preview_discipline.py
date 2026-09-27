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


def test_composer_prompt_has_user_speech_rule() -> None:
    """Issue #2943: живая проверка 24.09.2026 — робот зачитал юзеру
    партитуру целиком («Партитура чистая: пэд стоит над басом, бас
    держит только корни (bass_style=root), контрмелодия снята…»),
    прочитав правило «не "звучит хорошо", а "по партитуре: …"» как
    указание озвучивать партитуру. Партитура — материал ДЛЯ РЕШЕНИЙ
    модели, не текст для TTS; юзеру — короткая человеческая фраза без
    жаргона ручек/синтов/нот."""
    text = _composer_text()
    flat = text.replace("\n  ", " ").replace("\n", " ")
    assert "ЧТО ГОВОРИШЬ ЮЗЕРУ" in text
    # Старый шаблон озвучки должен быть явно запрещён, а не просто исчезнуть
    # молча — иначе следующий агент случайно вернёт его как "улучшение".
    assert "«по партитуре:" in flat and "ЗАПРЕЩЕНА" in flat
    # Запрет жаргона в речи — конкретные слова, которые нельзя произносить.
    for jargon in ("bass_style", "pad_style", "«партитура»", "«пэд»", "«контрмелодия»", "«ручка»"):
        assert jargon in flat, f"{jargon!r} не упомянут как запрещённое слово в речи"
    # Честность не должна была потеряться при переписывании правила.
    assert "ADR-0018" in text
    assert "звучит отлично" in flat


def test_composer_prompt_speech_example_has_no_jargon() -> None:
    """Примеры человеческой фразы («убрал гул в басе», «сделал легче») не
    должны сами содержать жаргон, который правило запрещает — иначе
    промпт учит одному, а показывает другое."""
    text = _composer_text()
    start = text.index("🗣️ ЧТО ГОВОРИШЬ ЮЗЕРУ")
    end = text.index("🎛️ ЖАЛОБА", start)
    section = text[start:end]
    for forbidden in ("bass_style=root", "bass_style=off", "pad_style=sustain"):
        assert forbidden not in section, f"пример фразы юзеру содержит жаргон {forbidden!r}"
    assert "убрал гул в басе" in section
    assert "сделал легче" in section


def test_dj_prompt_does_not_name_composer_tools() -> None:
    """Контракт из ADR §3.6: dj.txt не называет инструменты composer."""
    text = _DJ.read_text(encoding="utf-8")
    for tool in ("compose_music", "preview_arrangement", "execute_music_code"):
        assert tool not in text, f"dj.txt называет чужой инструмент {tool!r}"
