"""Music-stack health helpers extracted from ``MusicManager``.

Phase 2 of the ``MusicManager`` decomposition (ADR-0134 §5): the 5
methods that look after the sclang startup snapshot and the
"known SynthDef" truth-table now live in :class:`MusicStackHealth`.
The host class :class:`MusicManager` keeps the same public API via
thin delegating shims (see ``# SHIM-remove-after-#3014-phase-6``).

What lives here
===============

* :meth:`MusicStackHealth.known_synth_names` — frozenset of SynthDefs
  Python-side sent + server-confirmed in scsynth (issue #2838).
* :meth:`MusicStackHealth._log_synth_truth_discrepancy` — one-shot
  warning emitted at the end of successful ``_initialize_renardo``
  (issue #2838).
* :meth:`MusicStackHealth._evaluate_music_stack_health` — boot-time
  read of ``/tmp/sclang.log`` → :class:`MusicStackStatus`, plus the
  degraded-mode side-effect that clears ``_renardo_available``.
* :meth:`MusicStackHealth.is_music_stack_healthy` — boolean view of
  the latest status.
* :meth:`MusicStackHealth.music_stack_unavailable_error` — the single
  stable error payload returned by ``execute_code`` /
  ``set_vibe_preset`` / ``stop_music`` when the stack is degraded.

Why a separate class
====================

The five members share the same dependency surface (config flags,
synth-truth set, the ``_log_warning`` sink) but their call-sites in
``tools/music.py`` are spread across the boot sequence, the
SynthDef-validation guard, and the tool dispatch path. Lifting them
into one place lets the next phases of ADR-0134 tighten each method
in isolation without touching the rest of ``MusicManager``.

The host class still owns the mutated state (status snapshot,
confirmed-synths set, ``_renardo_available`` flag) — that contract
is documented on :class:`MusicStackHealth` itself.
"""

from __future__ import annotations

import os
from typing import Any, Dict, FrozenSet, List, Optional

from rob_box_voice.core.music_stack_validation import (
    MusicStackStatus,
    load_confirmed_synths,
    load_sclang_health,
)
from rob_box_voice.core.sc_only_custom_synthdefs import (
    CUSTOM_SC_ONLY_SYNTH_NAMES,
)


# ADR-0021 R1 / ADR-0134 §3.1: every method here is CC <= 4. Easy to keep
# under the 12-budget from ADR-0134 §5 Phase 2 because all 5 methods are
# short reads over the host's state.
# Single source of truth for the Python-wrapped SynthDef set — used by
# both ``known_synth_names`` and ``_log_synth_truth_discrepancy``.
_WRAPPED_FALLBACK: FrozenSet[str] = frozenset(CUSTOM_SC_ONLY_SYNTH_NAMES)


class MusicStackHealth:
    """Snapshot + display layer for the sclang/Renardo startup health.

    The class holds no state of its own: it reads and writes five
    attributes on the host ``MusicManager`` (passed as ``manager``):

    * ``_music_stack_status`` (:class:`MusicStackStatus`) — last
      snapshot produced by :func:`load_sclang_health`.
    * ``_server_confirmed_synths`` (:class:`frozenset` of ``str`` /
      ``None``) — names sclang confirmed in scsynth after
      ``Server.sync``; ``None`` until preload finishes.
    * ``_synthdefs_added`` (:class:`set` of ``str``) — names Python
      sent via ``sdef.add()``.
    * ``_renardo_available`` (``bool`` / ``None``) — flipped to
      ``False`` (without clearing ``_renardo_last_error``) when the
      stack is degraded AND ``_require_healthy`` is set.
    * ``_critical_synths`` (:class:`tuple` of ``str``) — passed to
      :func:`load_sclang_health`.

    Logging goes through ``manager._log_warning`` so the existing
    ``_logger`` / stderr fallback in ``MusicManager`` continues to
    apply without duplication.

    Args:
        manager: The owning ``MusicManager`` (or a duck-typed test
            double that exposes the attributes listed above and a
            ``_log_warning(message: str) -> None`` callable).
    """

    __slots__ = ("_manager",)

    def __init__(self, manager: Any) -> None:
        self._manager = manager

    # ------------------------------------------------------------------
    # Issue #2838 — known SynthDef set
    # ------------------------------------------------------------------

    def known_synth_names(self) -> Optional[frozenset]:
        """Множество SynthDef-имён, реально загруженных в scsynth.

        Issue #2838 (живой прогон 23.09.2026): раньше сюда шёл весь
        ``self._synthdefs_added`` — то, что Python-сторона renardo
        ОТПРАВИЛА через ``sdef.add()`` (UDP ``/foxdot`` → sclang, без
        подтверждения). Часть этих пакетов теряется на порту sclang
        (drops в ``/proc/net/udp``), и потерянный ``sine`` остался
        «известным»: валидатор сам подсказал его LLM, та им сыграла —
        235 × "SynthDef sine not found".

        Теперь источник истины — ``self._server_confirmed_synths``:
        имена, которые sclang подтвердил в scsynth строкой прелоада
        "SynthDef in scsynth: X" (печатается после ``Server.sync``,
        см. ``foxdot_init.sc``). Пересекаем его с тем, для чего есть
        Python-обёртка (``_synthdefs_added`` ∪
        ``CUSTOM_SC_ONLY_SYNTH_NAMES``): синт без обёртки код всё
        равно не вызовет. Это же отсекает служебные шины
        ``masterlimiter``/``masterfilter`` — они есть на сервере, но
        не тембры для ``lead_synth``/``bass_synth``/``pad_synth``.

        Если подтверждения нет (sclang-лог недоступен / прелоад не
        завершён) — прежнее поведение:
        ``_synthdefs_added`` ∪ ``CUSTOM_SC_ONLY_SYNTH_NAMES``. Оно НЕ
        проверено сервером; на старте это логируется
        (:meth:`_log_synth_truth_discrepancy`).

        Returns:
            ``None``, пока ``_synthdefs_added`` пуст (Renardo ещё не
            инициализирован, или тест создал ``MusicManager`` через
            ``__new__`` в обход ``__init__``) — вызывающая сторона
            должна трактовать это как «набор неизвестен», а не
            «ничего не разрешено», иначе валидатор блокировал бы
            ЛЮБОЙ синт до завершения инициализации. Иначе — frozenset
            имён (нижний регистр — как их печатает sclang).
        """
        manager = self._manager
        added = getattr(manager, "_synthdefs_added", None)
        if not added:
            return None
        wrapped = frozenset(added) | _WRAPPED_FALLBACK
        confirmed = getattr(manager, "_server_confirmed_synths", None)
        if confirmed is None:
            return wrapped
        return wrapped & confirmed

    def _log_synth_truth_discrepancy(self) -> None:
        """Issue #2838: залогировать расхождение «отправлено» vs «на сервере».

        Вызывается один раз в конце успешного ``_initialize_renardo``.
        Ничего не меняет — только делает видимым, какие синты
        Python-сторона считает добавленными, но sclang не подтвердил
        в scsynth (валидатор их отклоняет и не подсказывает).
        """
        manager = self._manager
        confirmed = getattr(manager, "_server_confirmed_synths", None)
        if confirmed is None:
            manager._log_warning(
                "[music #2838] нет подтверждения прелоада SynthDef-ов в "
                "sclang-логе — валидатор синтов работает по списку "
                "ОТПРАВЛЕННЫХ (sdef.add()), он не проверен сервером"
            )
            return
        unconfirmed = sorted(set(manager._synthdefs_added) - confirmed)
        wrapped = set(manager._synthdefs_added) | _WRAPPED_FALLBACK
        no_wrapper = sorted(confirmed - wrapped)
        sent = len(manager._synthdefs_added)
        allowed = len(self.known_synth_names() or ())
        manager._log_warning(
            f"[music #2838] SynthDef truth: подтверждено в scsynth "
            f"{len(confirmed)}, отправлено renardo {sent}, "
            f"разрешено валидатору {allowed}; "
            f"без подтверждения ({len(unconfirmed)}, отклоняются): "
            f"{unconfirmed}; на сервере без Python-обёртки: {no_wrapper}"
        )

    # ------------------------------------------------------------------
    # Music-stack health (issue G-MUSIC, architect review v3)
    # ------------------------------------------------------------------

    def _evaluate_music_stack_health(
        self,
        sclang_log_path: Optional[str] = None,
    ) -> MusicStackStatus:
        """Snapshot sclang health from the startup log and mark the manager.

        When ``is_healthy is False`` AND ``_require_healthy`` is True,
        this will also clear ``_renardo_available`` (without touching
        ``_renardo_last_error``) so downstream tools see consistent
        state.

        Args:
            sclang_log_path: Override log location. Falls back to
                ``SCLANG_LOG_PATH`` env var, then ``/tmp/sclang.log``.

        Returns:
            The :class:`MusicStackStatus` that was applied.
        """
        manager = self._manager
        status = load_sclang_health(
            sclang_log_path,
            critical_synths=list(manager._critical_synths),
        )
        manager._music_stack_status = status
        manager._server_confirmed_synths = load_confirmed_synths(sclang_log_path)

        if not status.is_healthy and manager._require_healthy:
            # Mark Renardo as unavailable WITHOUT clearing the existing
            # last_error (which might be informative for diagnostics).
            # The operator should see both "music stack degraded" AND
            # any subsequent renardo init failure that follows.
            manager._renardo_available = False

        return status

    def is_music_stack_healthy(self) -> bool:
        """True if the sclang startup log was healthy at the last check."""
        manager = self._manager
        return bool(manager._music_stack_status.is_healthy)

    def music_stack_unavailable_error(self) -> Dict[str, str]:
        """Build a stable error payload for ``music unavailable`` replies.

        Used by ``execute_code`` / ``set_vibe_preset`` / ``stop_music``
        so the LLM gets a single, recognizable error message rather
        than a different string for each entry-point.
        """
        manager = self._manager
        status = manager._music_stack_status
        details: List[str] = []
        if status.fatal_errors:
            details.append("; ".join(status.fatal_errors[:3]))
        if status.missing_synths:
            details.append(
                f"missing SynthDefs: {', '.join(status.missing_synths)}"
            )
        detail_str = (" — " + "; ".join(details)) if details else ""
        log_path = os.environ.get("SCLANG_LOG_PATH", "/tmp/sclang.log")
        return {
            "success": False,
            "error": (
                "Музыка недоступна: sclang стартовал в degraded-режиме "
                "(syntax error в startup-логе Renardo/FoxDot)"
                f"{detail_str}. См. {log_path}."
            ),
        }