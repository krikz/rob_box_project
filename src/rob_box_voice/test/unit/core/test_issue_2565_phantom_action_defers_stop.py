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
        """Сценарий из живого лога issue #2565 (с правильными матчами).

        Юзер сказал «останови музыку, загрузи трек OAKENFOLD» — фраза
        содержит стоп-слово («останови музыку» — ``MUSIC_STOP_OVERRIDES``)
        и просьбу нового трека (правило ``track_load`` в
        :data:`ACTION_CLAIM_RULES`). LLM ответила «Загрузил трек
        OAKENFOLD, наслаждайся.» при tools=[] — action-claim на запуск.
        До фикса guard Force-Stop'ал активную музыку; после фикса —
        deferral, чтобы CRITICAL-retry в
        :func:`_check_unbacked_action_claim_and_retry` сначала дожал
        модель до реального ``load_track`` / ``gen_play_from_library``,
        и только потом музыка сменилась (а не потухла совсем).
        """
        guard = MusicGuard()
        verdict = guard.evaluate(
            was_dj_auto=False,
            user_input="останови музыку, загрузи трек OAKENFOLD",
            tools_called=(),
            dj_enabled=False,
            spoken=(
                "Загрузил трек OAKENFOLD — стартуем с акт I, 124 BPM. "
                "Погружение в разгон, профиль сохранил."
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

    def test_phantom_action_track_load_claim_with_stop_word(self) -> None:
        """Минимальный phantom-action на запуск трека + стоп-фраза.

        Юзер: «выключи музыку, загрузи трек тисбит».
        LLM: «Трек играет.», tools=[] — ``detect_unbacked_action_claim``
        ловит правило ``track_load``. Guard должен пропустить stop-ветку.
        """
        guard = MusicGuard()
        verdict = guard.evaluate(
            was_dj_auto=False,
            user_input="выключи музыку, загрузи трек тисбит",
            tools_called=(),
            dj_enabled=False,
            spoken="Трек играет.",
            build_music_retry_prompt=_music_prompt,
        )
        assert verdict.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE
        assert verdict.reason == "phantom_action_defers_stop"

    def test_real_stop_with_stop_music_tool_still_skips(self) -> None:
        """Реальный stop с ``stop_music`` в tools — guard пропускает (нечего
        останавливать). ``spoken`` не влияет на back-compat, даже если он
        похож на phantom-action (на случай когда LLM после ``stop_music``
        ещё и сказала «Готово.»).
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
        assert verdict.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE, (
            f"real stop with stop_music tool must skip the guard, "
            f"got {verdict.kind} reason={verdict.reason!r}"
        )

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

        Issue #2971 регресс: раньше тут стоял ``user_input="поставь
        диджея"`` и попадал в FORCE_STOP-ветку ТОЛЬКО из-за той же
        голой-подстроки-«диджея» бага, что чинит #2971 (``поставь``
        не стоп-глагол — юзер просит ЗАПУСТИТЬ, а не остановить). После
        фикса ``is_music_stop_command("поставь диджея")`` корректно
        ``False``, и этот кейс больше не проверяет phantom-action
        deferral. Заменено на «выключи музыку, поставь диджея» — явный
        стоп-глагол («выключи») + муз. объект даёт настоящую
        стоп-команду, которая вместе с action-claim в ``spoken`` (см.
        :func:`detect_unbacked_action_claim`) действительно проверяет
        deferral-ветку, как и было задумано.
        """
        guard = MusicGuard()
        # Prime the budget to verify it's not consumed.
        guard._user_retry_count = 0
        guard.evaluate(
            was_dj_auto=False,
            user_input="выключи музыку, поставь диджея",
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
    "user_input,spoken",
    [
        # Каждый кейс — стоп-фраза из MUSIC_STOP_OVERRIDES + phantom-action
        # claim в LLM-ответе, попадающий в правило ``track_load`` из
        # :data:`ACTION_CLAIM_RULES` (user_re: «загрузи/включи/поставь/запусти
        # ... трек/композиц/мелоди»; claim_re: «игра/звучит/запустил/
        # включил/поставил/загрузил»).
        (
            "останови музыку, загрузи трек тисбит",
            "Загрузил трек тисбит, наслаждайся.",
        ),
        (
            "выключи музыку, включи трек про весну",
            "Трек играет.",
        ),
        (
            "останови музыку, поставь трек джаз",
            "Поставил, погнали.",
        ),
    ],
)
def test_phantom_action_claims_for_music_are_detected(user_input: str, spoken: str) -> None:
    """Покрывает правило ``track_load`` из :data:`ACTION_CLAIM_RULES` в
    сочетании со стоп-фразой из :data:`MUSIC_STOP_OVERRIDES`. Deferral
    срабатывает ТОЛЬКО в стоп-ветке guard'а — иначе action-claim уже
    ловится в :func:`_check_unbacked_action_claim_and_retry`.
    """
    guard = MusicGuard()
    verdict = guard.evaluate(
        was_dj_auto=False,
        user_input=user_input,
        tools_called=(),
        dj_enabled=False,
        spoken=spoken,
        build_music_retry_prompt=_music_prompt,
    )
    assert verdict.kind is MusicGuardVerdictKind.SKIP_NOT_APPLICABLE
    assert verdict.reason == "phantom_action_defers_stop"
