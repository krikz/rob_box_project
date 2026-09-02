"""Smoke test for the dramaturgy fix in build_auto_prompt (issue #1016 follow-up).

Verifies that:
  * n=1 prompt BANS load_track/list_tracks (library auto-play) and carries
    the stage_line markers
  * n=2 prompt contains the "С РАЗВИТИЕМ" requirement + the listed patterns
  * Neither prompt leaks task-tracking identifiers (#NNNN, "Refs:", "issue #")
    that carry no semantic value for the LLM and may trigger hallucinated
    "fix #NNNN" comments in generated code.
"""
from __future__ import annotations

import json
import logging
import sys
from pathlib import Path

# Make the in-tree package importable without `pip install -e`.
ROOT = Path(__file__).resolve().parents[4]  # .../test/unit/core/dj_mode/
sys.path.insert(0, str(ROOT / "src" / "rob_box_voice"))

from rob_box_voice.core.dj_mode import DJModeController


class _StubHook:
    """Minimal DJHook-shaped stub — only the attributes the controller reads."""

    def __init__(self) -> None:
        self.persona_default = "Роббокс"
        self.dispatch_calls: list[str] = []

    def dispatch(self, prompt: str, is_auto: bool) -> None:  # noqa: ARG002
        self.dispatch_calls.append(prompt)

    def is_active(self) -> bool:
        return False

    def is_dialogue_active(self) -> bool:
        return False


def _build_controller() -> DJModeController:
    return DJModeController(hook=_StubHook(), logger=logging.getLogger("test"))


def test_n1_contains_library_and_stage_lines() -> None:
    ctrl = _build_controller()
    ctrl.state.theme = "летняя дискотека"
    prompt = ctrl.build_auto_prompt(1)

    # Library line: DJ transitions must NOT load stored tracks (load_track
    # auto-plays them → harsh cut between tracks).
    assert "НЕ вызывай load_track" in prompt, "load_track ban missing from n=1 prompt"
    # Issue #1811: DJ mode plays through compose_music, not hand-written code.
    assert "compose_music" in prompt, "compose_music instruction missing from n=1 prompt"
    assert "execute_music_code" not in prompt, (
        "n=1 prompt must not tell the model to hand-write Renardo code"
    )

    # Stage line: progress indicator for the LLM.
    assert "переход #1" in prompt, f"stage_line missing — got: {prompt[:200]!r}"

    # Persona + theme still reach the prompt (regression guard).
    assert "Роббокс" in prompt
    assert "летняя дискотека" in prompt

    # Start-of-party marker preserved.
    assert "СТАРТ ВЕЧЕРИНКИ" in prompt


def test_n2_requires_dramaturgy() -> None:
    """Issue #1811: development is now the arranger's job (compose_music's
    ``form``), not hand-written ``.every()``/``Pvar``/``Clock.future`` code."""
    ctrl = _build_controller()
    ctrl.state.theme = "тёмный техно"
    prompt = ctrl.build_auto_prompt(2)

    assert "DJ_AUTO переход #2" in prompt
    assert "РАЗВИВАТЬСЯ" in prompt, "transition prompt must still demand development"
    assert "compose_music" in prompt, "transition must go through compose_music"
    assert "form=" in prompt, "form is what carries development now"

    # The old hand-written-code dramaturgy techniques are no longer required.
    for pattern in (".every()", "Pvar", "linvar", "Clock.future"):
        assert pattern not in prompt, (
            f"{pattern!r} is a hand-written-code technique — compose_music "
            "should not need it"
        )
    assert "execute_music_code" not in prompt

    # Library line still present on transitions — as a BAN (no auto-play).
    assert "НЕ вызывай load_track" in prompt


def test_no_task_ids_leak_into_prompt() -> None:
    """Bug fix follow-up: do NOT pollute prompts with issue numbers / 'Refs:'.

    The LLM has no use for issue IDs and may echo them into generated code
    as a fake 'fix reference'. Such identifiers belong in git commit
    messages and PR descriptions, not in the prompt itself.
    """
    for n in (1, 2, 3, 5):
        ctrl = _build_controller()
        ctrl.state.theme = "хаус"
        prompt = ctrl.build_auto_prompt(n)

        for forbidden in ("#1016", "#990", "#1000", "Refs:", "issue #"):
            assert forbidden not in prompt, (
                f"n={n} prompt leaked {forbidden!r}: {prompt[:300]!r}"
            )


def test_n1_contains_research_and_plan_instructions() -> None:
    """Переход #1 должен: сначала исследовать персону/музыку, затем план."""
    ctrl = _build_controller()
    ctrl.state.theme = "панк"
    prompt = ctrl.build_auto_prompt(1)

    # research-first: поиск персоны, сэмплов и готовых композиций.
    assert "search_web" in prompt, "research-first: search_web missing"
    assert "search_samples" in prompt, "research-first: search_samples missing"
    assert "gen_search_library" in prompt, "research-first: gen_search_library missing"
    # план сета через set_dj_mode(plan=...).
    assert "ПЛАН СЕТА" in prompt, "plan instruction missing from n=1"
    assert "set_dj_mode(enabled=true, plan=" in prompt


def test_tech_guardrails_present_on_transition() -> None:
    """Issue #1811: аранжировщик и мастер-фильтр держат амплитуды/паттерны —
    переход обязан нести НЕ ручные лимиты, а инструкцию разнообразить
    compose_music-параметры между треками."""
    ctrl = _build_controller()
    ctrl.state.theme = "техно"
    prompt = ctrl.build_auto_prompt(2)

    assert "compose_music" in prompt
    assert "hats_sample" in prompt
    assert "perc" in prompt
    assert "swing" in prompt
    assert "search_samples" in prompt

    # Hand-managed limits from the execute_music_code era are gone — the
    # arranger and master filter own this now, the model shouldn't compute it.
    for stale in ("Clock.clear()", "СУММА amp", "d4/d5/p4/p5", "chop="):
        assert stale not in prompt, f"{stale!r} is a pre-#1811 hand-coded limit"


def test_chop_is_never_mentioned_in_dj_transitions() -> None:
    """compose_music has no chop= parameter — DJ prompts should not mention it
    at all anymore (issue #1811), unlike the old hand-written-code era."""
    ctrl = _build_controller()
    ctrl.state.theme = "техно"
    prompt = ctrl.build_auto_prompt(2)

    assert "chop" not in prompt
    assert ".every()" not in prompt


def test_final_track_when_plan_set() -> None:
    """При достижении последнего трека плана — финальный промпт с выключением DJ."""
    ctrl = _build_controller()
    ctrl.state.theme = "диско"
    ctrl.state.set_plan = "Трек 1: старт\nТрек 2: пик"

    prompt = ctrl.build_auto_prompt(2)

    assert "ФИНАЛЬНЫЙ ТРЕК" in prompt
    assert "set_dj_mode(enabled=false)" in prompt


def test_first_plan_does_not_reset_transition_count() -> None:
    """Первый план приходит на переходе #1 — счётчик НЕ должен сброситься,
    иначе следующий тик снова запустит переход #1."""
    ctrl = _build_controller()
    ctrl.state.enabled = True
    ctrl.state.transition_count = 1

    ctrl.handle_message(json.dumps({
        "enabled": True,
        "plan": "Трек 1: a\nТрек 2: b",
        "next_transition_sec": 45,
    }))

    assert ctrl.state.set_plan == "Трек 1: a\nТрек 2: b"
    assert ctrl.state.transition_count == 1, "первый план не должен сбрасывать счётчик"


def test_mid_session_plan_rewrite_keeps_transition_count() -> None:
    """Перезапись плана ВНУТРИ идущей сессии — счётчик НЕ сбрасывается.

    Контракт изменён сознательно в ``ef525468e`` ("fix(dj): keep transition
    progress on mid-session plan updates", live 01.09, issue #992). До него
    сигналом «новый сет» была строка плана: ``is_rewrite = bool(set_plan)``.
    На практике модель переписывает план своими словами почти на КАЖДОМ
    переходе (промпт этого не запрещает) → строка отличается → счётчик
    обнулялся → ``build_auto_prompt(1)`` снова выдавал «СТАРТ ВЕЧЕРИНКИ» →
    модель снова сочиняла план → бесконечный цикл. И финальный трек, и
    auto-stop гейтятся на ``transition_count`` vs длину плана (см.
    ``tick()``), поэтому сет НИКОГДА не завершался. Живой прогон 01.09
    («вечеринка у черепашек-ниндзя»): 8 «rewrite» подряд за 12 минут, ни
    одного перехода дальше #1.

    Теперь единственный сигнал генуинного старта — DJ был ВЫКЛЮЧЕН
    (``is_fresh_start = not was_enabled``), см.
    ``test_fresh_start_resets_transition_count`` ниже.
    """
    ctrl = _build_controller()
    ctrl.state.enabled = True
    ctrl.state.set_plan = "Трек 1: old"
    ctrl.state.transition_count = 5

    ctrl.handle_message(json.dumps({
        "enabled": True,
        "plan": "Трек 1: new\nТрек 2: new2",
    }))

    assert ctrl.state.set_plan == "Трек 1: new\nТрек 2: new2", (
        "текст плана обновляется даже без сброса счётчика"
    )
    assert ctrl.state.transition_count == 5, (
        "перезапись плана в идущей сессии НЕ должна сбрасывать счётчик "
        "(ef525468e, issue #992) — иначе сет зацикливается на переходе #1"
    )


def test_fresh_start_resets_transition_count() -> None:
    """Генуинный старт (DJ был выключен) — счётчик с нуля.

    Обратная сторона ``ef525468e``: сброс прогресса остался, но привязан к
    переходу enabled False→True, а не к тексту плана.
    """
    ctrl = _build_controller()
    ctrl.state.enabled = False
    ctrl.state.set_plan = "Трек 1: old"
    ctrl.state.transition_count = 5

    ctrl.handle_message(json.dumps({
        "enabled": True,
        "plan": "Трек 1: new\nТрек 2: new2",
    }))

    assert ctrl.state.set_plan == "Трек 1: new\nТрек 2: new2"
    assert ctrl.state.transition_count == 0, (
        "при генуинном старте (enabled False→True) счётчик сбрасывается "
        "в 0, даже если план уже был — is_fresh_start = not was_enabled"
    )


def test_middle_transitions_are_music_only() -> None:
    """🔴 Живой лог 02.09: на КАЖДОМ переходе поверх бита звучало
    «Переход номер два отыгран — нарастание с дропом в ре миноре
    фригийском, сто сорок ударов!». Промпт запрещал только ``speak_text``,
    а свободный текст ответа тоже уходит в TTS. Юзер: «во время сочинения
    музыки LLM много говорит».
    """
    ctrl = _build_controller()
    ctrl.state.set_plan = "\n".join(
        f"Трек {i}: {c}" for i, c in enumerate("abcd", start=1)
    )

    # #1 — представление диджея, говорить можно.
    assert ctrl.is_music_only_transition(1) is False
    # Середина сета — только музыка.
    assert ctrl.is_music_only_transition(2) is True
    assert ctrl.is_music_only_transition(3) is True
    # Финальный трек плана — прощание, говорить можно.
    assert ctrl.is_music_only_transition(4) is False


def test_music_only_without_a_plan() -> None:
    """Без плана финального трека не существует — молчат все переходы,
    кроме первого."""
    ctrl = _build_controller()
    assert ctrl.is_music_only_transition(1) is False
    assert ctrl.is_music_only_transition(7) is True


def test_transition_prompt_asks_for_the_form_length() -> None:
    """Форма compose_music играет 96-190 секунд, а промпт просил
    ``next_transition_sec=45`` — дроп не звучал ни разу за 30 часов лога."""
    ctrl = _build_controller()
    for n in (1, 2):
        prompt = ctrl.build_auto_prompt(n)
        assert "next_transition_sec=45" not in prompt
        assert "длительность" in prompt.lower()
