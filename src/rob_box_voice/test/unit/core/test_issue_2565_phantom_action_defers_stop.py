"""Issue #2565 — phantom-action must defer ``stop_command_guard`` (issue #935).

Live repro (vision-pi, 2026-09-15 12:16 MSK, DJ Oakenfold case): юзер сказал
«Ты диджей PAUL OAKENFOLD...», LLM ответила spoken='Запускаю Oakenfold-сессию
— стартуем с акт I, 124 BPM, погружение в разгон.' при ``tools=[]``.
Сразу же сработал ``stop_command_guard`` (issue #935) → активная
музыка потушена → юзер слышит тишину после «Запускаю…».

Гипотеза (issue #935 vs phantom-action): guard смотрит только на юзер-интент
и на наличие stop-тула. Но если LLM только что ПООБЕЩАЛА запустить новый
трек (action-claim без тула — уже отлавливается issue #992 Bug E /
:func:`detect_unbacked_action_claim`), глушить активную музыку нельзя —
промпт-сценарий «новый трек заменит старый» уже сломан тем, что модель
не вызвала тул, и CRITICAL-retry на :func:`_check_unbacked_action_claim_and_retry`
это починит. Если же мы сначала потушим старую музыку, то получится
«юзер слышит тишину после „Запускаю…"» — то, что и воспроизвело issue #2565.

Минимальный фикс (вариант 1 из task body): ``MusicGuard.evaluate``
получает ``spoken`` и перед FORCE_STOP проверяет, не было ли action-claim
на запуск/смену трека. Если было — возвращает ``SKIP_NOT_APPLICABLE``
с reason ``"phantom_action_defers_stop"`` (music НЕ тушится), и оставляет
возможность ``_check_unbacked_action_claim_and_retry`` сделать свою работу
в :meth:`DialogueNode._handle_result` (он уже вызывается там же).

Acceptance criteria (из задачи):

* phantom-action + stop_command_guard → НЕ должна тушить активную музыку,
  должна сработать retry (это проверяет ``_check_unbacked_action_claim_and_retry``
  — отдельный тест-инвариант в dialogue_node).
* Реальный stop (spoken="Музыка выключена." + tools=[stop_music]) —
  guard НЕ останавливает сам (потому что stop_music уже был).
* При tools=[] без phantom-action — старое поведение FORCE_STOP
  сохраняется (back-compat с live 30.08 vision-pi 12:33).

Все тесты тут — pure-Python: ``MusicGuard`` не требует ROS2.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.music_guard import (
    MusicGuard,
    MusicGuardVerdictKind,
)


def _music_prompt(user_input: str) -> str:
    """Stub for ``DialogueNode._build_music_retry_prompt``."""
    return f"[CRITICAL] user_input={user_input}"


def _dj_prompt() -> str:
    """Stub for ``DialogueNode._build_dj_retry_prompt``."""
    return "[CRITICAL] DJ retry — call execute_music_code"


# ---------------------------------------------------------------------------
# Issue #2565 — phantom-action deferral (the actual fix).
# ---------------------------------------------------------------------------


class TestPhantomActionDefersForceStop:
    """Phantom-action (issue #2559) must defer ``FORCE_STOP``.

    Until issue #2565, :meth:`MusicGuard.evaluate` returned
    ``FORCE_STOP`` whenever ``is_music_stop_command(user_input)`` was true
    AND no ``stop_music`` tool was in ``tools_called``. That logic is
    correct for a real stop («останови музыку» → LLM промолчала), but
    WRONG for a phantom-action on a NEW track: «Ты диджей PAUL OAKENFOLD»
    → LLM сказала «Запускаю Oakenfold-сессию…» без тула, и при этом
    активная музыка ещё играет — гасить её нельзя, иначе юзер слышит
    тишину после «Запускаю…».
    """

    def test_live_oakenfold_phantom_action_defers_force_stop(self) -> None:
        """Дословный кейс из живого лога issue #2565.

        Юзер: «Ты диджей PAUL OAKENFOLD и у нас сегодня вечеринка в наливайке
        в Батайске для местных алконавтов» (фраза матчит music-старт).
        LLM: «Запускаю Oakenfold-сессию — стартуем с акт I, 124 BPM…»,
        tools=[]. До фикса guard сразу же Force-Stop'ал активную музыку;
        после фикса — deferral, чтобы CRITICAL-retry успел сработать.
        """
        guard = MusicGuard()
        # Live scenario: previous track was playing, user asked for a new
        # one, LLM promised it but did not call load_track / gen_play_…
        verdict = guard.evaluate(
            was_dj_auto=False,
            user_input="поставь диджея PAUL OAKENFOLD на вечеринку",
            tools_called=(),
            dj_enabled=False,
            spoken=(
                "Запускаю Oakenfold-сессию — стартуем с акт I, 124 BPM, "
                "погружение в разгон. И профиль сохранил."
            ),
            build_music_retry_prompt=_music_prompt,
        )
        assert verdict.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE, (
            f"phantom-action must DEFER force-stop, got {verdict.kind} "
            f"reason={verdict.reason!r}"
        )
        assert verdict.reason == "phantom_action_defers_stop"
        # Stop-guard budget must NOT be touched — мы вообще ничего не делаем.
        assert guard.user_retry_count == 0

    def test_phantom_action_track_load_claim(self) -> None:
        """Минимальный phantom-action на запуск трека.

        Юзер: «запусти трек тисбит». LLM: «Трек играет.», tools=[] —
        ``detect_unbacked_action_claim`` ловит правило ``track_load``.
        Guard должен пропустить stop-ветку.
        """
        guard = MusicGuard()
        verdict = guard.evaluate(
            was_dj_auto=False,
            user_input="запусти трек тисбит",
            tools_called=(),
            dj_enabled=False,
            spoken="Трек играет.",
            build_music_retry_prompt=_music_prompt,
        )
        assert verdict.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE
        assert verdict.reason == "phantom_action_defers_stop"

    def test_real_stop_with_stop_music_tool_still_skips(self) -> None:
        """Реальный stop с ``stop_music`` в tools — guard НЕ работает (нечего
        останавливать). Поведение back-compat с TestEvaluateStopCommand.
        """
        guard = MusicGuard()
        verdict = guard.evaluate(
            was_dj_auto=False,
            user_input="останови музыку",
            tools_called=("stop_music",),
            dj_enabled=False,
            spoken="Музыка выключена.",
            build_music_retry_prompt=_music_prompt,
        )
        assert verdict.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE
        assert verdict.reason == "executed"  # short-circuit на stop_music

    def test_real_stop_without_phantom_still_force_stops(self) -> None:
        """Реальный stop без tool и БЕЗ phantom-action — старое поведение
        FORCE_STOP сохраняется (live 30.08, vision-pi 12:33).
        """
        guard = MusicGuard()
        verdict = guard.evaluate(
            was_dj_auto=False,
            user_input="останови музыку",
            tools_called=(),
            dj_enabled=False,
            spoken="Хорошо.",  # без action-claim claim_re
            build_music_retry_prompt=_music_prompt,
        )
        assert verdict.kind is MusicGuardVerdictKind.FORCE_STOP
        assert verdict.reason == "stop_command_unbacked"

    def test_no_spoken_param_does_not_crash_and_preserves_force_stop(self) -> None:
        """Back-compat: ``spoken`` опционален (default ``None``).

        Если caller (например, _dispatch_dj_turn path, где spoken ещё не
        известен) вызывает guard без spoken, поведение — ровно как до
        фикса. FORCE_STOP срабатывает, фантом не отлавливается.
        """
        guard = MusicGuard()
        verdict = guard.evaluate(
            was_dj_auto=False,
            user_input="останови музыку",
            tools_called=(),
            dj_enabled=False,
            # spoken НЕ передаём — default None
            build_music_retry_prompt=_music_prompt,
        )
        assert verdict.kind is MusicGuardVerdictKind.FORCE_STOP
        assert verdict.reason == "stop_command_unbacked"

    def test_phantom_action_does_not_consume_user_retry_budget(self) -> None:
        """Deferral НЕ должен списывать user-retry budget — мы вообще ничего
        не делаем (CRITICAL-retry сам разберётся в :func:`_check_unbacked_action_claim_and_retry`).
        """
        guard = MusicGuard()
        # Prime the budget to verify it's not consumed.
        guard._user_retry_count = 0
        guard.evaluate(
            was_dj_auto=False,
            user_input="поставь диджея",
            tools_called=(),
            dj_enabled=False,
            spoken="Запускаю расслабленную лаундж-композицию через compose_music.",
            build_music_retry_prompt=_music_prompt,
        )
        assert guard.user_retry_count == 0, (
            "phantom-action deferral must not consume the user-retry budget "
            "— it's not a music request, it's an action-claim"
        )


# ---------------------------------------------------------------------------
# Phantom-action deferral must NOT mask other guards.
# ---------------------------------------------------------------------------


class TestPhantomActionDoesNotMaskOtherGuards:
    """Sanity: deferral активен ТОЛЬКО в стоп-ветке (FORCE_STOP), чтобы не
    ломать Bug B / Bug C / Bug F happy-paths.
    """

    def test_bug_c_still_reprompts_on_real_music_skip(self) -> None:
        """«сыграй трек» + tools=[] + spoken без action-claim → USER_RETRY
        (Bug C), как и до фикса. Phantom-action НЕ ловит эту ветку.
        """
        guard = MusicGuard()
        verdict = guard.evaluate(
            was_dj_auto=False,
            user_input="сыграй трек про космос",
            tools_called=(),
            dj_enabled=False,
            spoken="Окей.",  # нет claim_re
            build_music_retry_prompt=_music_prompt,
        )
        assert verdict.kind is MusicGuardVerdictKind.USER_RETRY
        assert verdict.reason == "bug_c"

    def test_bug_b_dj_retry_unaffected(self) -> None:
        """DJ auto-tick + tools=[speak_text] → DJ_RETRY (Bug B). Phantom
        deferral не должен вмешиваться.
        """
        guard = MusicGuard()
        verdict = guard.evaluate(
            was_dj_auto=True,
            user_input="DJ auto prompt",
            tools_called=("speak_text",),
            dj_enabled=True,
            spoken="Сочиняю бит через compose_music.",  # не action-claim для DJ
            build_dj_retry_prompt=_dj_prompt,
        )
        assert verdict.kind is MusicGuardVerdictKind.DJ_RETRY


# ---------------------------------------------------------------------------
# Parametrised smoke — что classify-функция понимает «запускаю …»
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "spoken",
    [
        "Запускаю Oakenfold-сессию — стартуем с акт I, 124 BPM.",
        "Трек играет.",
        "Композиция пошла.",
        "Загрузил трек тисбит, наслаждайся.",
    ],
)
def test_phantom_action_claims_for_music_are_detected(spoken: str) -> None:
    """Покрывает правило ``track_load`` из :data:`ACTION_CLAIM_RULES`."""
    guard = MusicGuard()
    verdict = guard.evaluate(
        was_dj_auto=False,
        user_input="поставь диджея" if "дидж" in spoken.lower() else "запусти трек",
        tools_called=(),
        dj_enabled=False,
        spoken=spoken,
        build_music_retry_prompt=_music_prompt,
    )
    assert verdict.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE
    assert verdict.reason == "phantom_action_defers_stop"
