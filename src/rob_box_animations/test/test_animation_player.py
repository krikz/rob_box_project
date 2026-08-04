# Copyright (c) 2024-2026 Rob Box Project. See LICENSE file for details.
"""Unit tests for AnimationPlayer.play_animation() (gap G-ANIM fix).

The bug being tested: before this fix the node called ``load_animation()``
followed by ``play()``. When the previous playback was still running,
``play()`` logged ``'Animation already playing'`` and returned False
without cancelling the previous thread — so two playback loops ended up
racing over ``current_animation`` and spamming the same WARN ~21 times
per dialog (see ``voice_v3_postdeploy.log``).

Acceptance (per 06-ARCHITECT-REVIEW-V3.md §2.G-ANIM):

* ``play_animation()`` on a running player replaces the previous thread
  cleanly — no ``'Animation already playing'`` WARN.
* Reload is skipped when the same manifest is requested twice in a row
  (no needless publisher churn).
* State is consistent under the lock (no half-written
  ``current_animation``).

All tests run against an in-process FakeNode (see ``conftest.py``) so
they need neither a live ROS2 daemon nor the full animation asset tree.
"""

from __future__ import annotations

import threading
from unittest.mock import patch

import pytest  # noqa: F401  # needed for conftest fixtures


# ---------------------------------------------------------------------------
# TestAnimationPlayerPlayAnimation
# ---------------------------------------------------------------------------


class TestAnimationPlayerPlayAnimation:
    """Behavioral tests for the new atomic load+play entry point."""

    def test_play_animation_starts_when_idle(self, player) -> None:
        """First call must succeed and set ``is_playing=True``."""
        ok = player.play_animation('idle.yaml')

        assert ok is True
        assert player.is_playing is True
        assert player.current_animation is not None
        assert player.current_animation.name == 'idle'
        # And a fresh playback thread was spawned. With the no-op
        # _playback_loop fixture body it may have already returned by
        # the time we check, so we only assert that one was started.
        assert player.playback_thread is not None

    def test_play_animation_cancels_previous_playback(
        self, player,
    ) -> None:
        """Second call must terminate the previous thread, not coexist.

        Before the fix, the second call hit ``play()``'s
        ``is_playing`` guard and returned ``False`` while the old
        thread kept running — exactly the production race we are
        closing.
        """
        assert player.play_animation('idle.yaml') is True
        first_thread = player.playback_thread
        assert first_thread is not None

        # Replace the no-op playback body with one that sleeps so we
        # can observe cancellation actually interrupting it.
        cancelled = threading.Event()
        with patch.object(
            player,
            '_playback_loop',
            side_effect=lambda: (
                player.stop_event.wait(timeout=5.0),
                cancelled.set(),
            ),
        ):
            ok = player.play_animation('talking.yaml')

        assert ok is True
        assert player.is_playing is True
        assert player.current_animation is not None
        assert player.current_animation.name == 'talking'

        # The old thread should be gone and a new one in its place.
        # First thread's stop_event was set — its loop returned.
        assert cancelled.is_set() or not first_thread.is_alive()
        # The new playback_thread is the freshly-spawned one.
        assert player.playback_thread is not first_thread

    def test_play_animation_does_not_warn_already_playing(
        self, player,
    ) -> None:
        """The exact WARN string that spammed the log must not appear.

        Production log evidence:
            ``voice_v3_postdeploy.log:278,318,...,520`` — 21
            occurrences of ``'Animation already playing'`` in 145s
            of dialog.
        """
        # FakeNode.get_logger returns the underlying MagicMock logger
        # directly, so .warn is the warn *method* on it.
        warn_mock = player.node._logger.warn
        warn_mock.reset_mock()

        player.play_animation('idle.yaml')
        player.play_animation('talking.yaml')
        player.play_animation('happy.yaml')

        for call in warn_mock.call_args_list:
            args, _ = call
            assert 'Animation already playing' not in (args[0] if args else '')

        # Stronger assertion: zero such warnings across the run.
        offending = [
            call for call in warn_mock.call_args_list
            if 'Animation already playing' in (
                call.args[0] if call.args else ''
            )
        ]
        assert offending == []

    def test_play_animation_skip_reload_when_same_manifest(
        self, player,
    ) -> None:
        """Reload-skip path: same manifest twice = 1 disk load."""
        with patch.object(
            player.loader,
            'load_manifest',
            wraps=player.loader.load_manifest,
        ) as spy:
            player.play_animation('idle.yaml')
            player.play_animation('idle.yaml')  # second call should skip

        # Loader only consulted once — the cached manifest is reused.
        assert spy.call_count == 1
        # And the state still points at the same animation.
        assert player.current_animation.name == 'idle'

    def test_play_animation_reloads_when_manifest_changes(
        self, player,
    ) -> None:
        """Different manifest = fresh load — regression guard."""
        with patch.object(
            player.loader,
            'load_manifest',
            wraps=player.loader.load_manifest,
        ) as spy:
            player.play_animation('idle.yaml')
            player.play_animation('talking.yaml')

        assert spy.call_count == 2

    def test_play_animation_returns_false_on_missing_manifest(
        self, player,
    ) -> None:
        """Failure path: loader raises → returns ``False``, state preserved."""
        # First load the idle animation cleanly.
        assert player.play_animation('idle.yaml') is True

        with patch.object(
            player.loader,
            'load_manifest',
            side_effect=FileNotFoundError('nope.yaml'),
        ):
            ok = player.play_animation('nope.yaml')

        assert ok is False
        # We did NOT clobber the previously-loaded animation.
        assert player.current_animation is not None
        assert player.current_animation.name == 'idle'

    def test_play_animation_thread_safety_under_concurrent_calls(
        self, player,
    ) -> None:
        """21 concurrent ``play_animation()`` calls must leave clean state.

        Reproduces the production race: many parallel requests
        coming in (TTS state + voice requests + idle-fallback timer
        firing at once). Before the fix this left multiple playback
        threads alive; after the fix exactly one
        ``playback_thread`` is tracked.
        """
        names = ['idle.yaml', 'talking.yaml', 'happy.yaml'] * 7  # 21
        results: list[bool] = []
        results_lock = threading.Lock()

        def worker(name: str) -> None:
            ok = player.play_animation(name)
            with results_lock:
                results.append(ok)

        threads = [
            threading.Thread(target=worker, args=(n,))
            for n in names
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10.0)
            assert not t.is_alive(), 'worker thread hung'

        # All calls succeeded.
        assert results == [True] * len(names)

        # Exactly one playback thread is tracked. The actual thread
        # may have already finished (no-op loop body), but the
        # bookkeeping invariant is what matters here: at any moment
        # the player tracks at most one playback_thread.
        assert player.playback_thread is not None
        assert player.is_playing is True
        assert player.current_animation is not None

        # And the worker threads didn't leak any "Animation already
        # playing" warnings — the load+play path never reaches the
        # old play() method's guard.
        warn_mock = player.node._logger.warn
        offending = [
            call for call in warn_mock.call_args_list
            if 'Animation already playing' in (
                call.args[0] if call.args else ''
            )
        ]
        assert offending == []


# ---------------------------------------------------------------------------
# TestLoadAnimationLockContract
# ---------------------------------------------------------------------------


class TestLoadAnimationLockContract:
    """Pin down the contract that load_animation is safe to call alone."""

    def test_load_animation_succeeds_on_first_call(self, player) -> None:
        assert player.load_animation('idle.yaml') is True
        assert player.current_animation.name == 'idle'

    def test_load_animation_skips_reload_of_same_manifest(
        self, player,
    ) -> None:
        player.load_animation('idle.yaml')
        with patch.object(
            player.loader, 'load_manifest', wraps=player.loader.load_manifest,
        ) as spy:
            ok = player.load_animation('idle.yaml')

        assert ok is True
        assert spy.call_count == 0

    def test_load_animation_does_not_require_playback_state(
        self, player,
    ) -> None:
        """load_animation() alone must not touch is_playing.

        External callers (e.g. ROS service handlers) may load without
        immediately playing. The lock must not flip playback state.
        """
        player.load_animation('idle.yaml')
        assert player.is_playing is False
        assert player.playback_thread is None


# ---------------------------------------------------------------------------
# TestManifestNameHelper
# ---------------------------------------------------------------------------


class TestManifestNameHelper:
    """Direct coverage for the small helper that drives skip-reload."""

    def test_bare_name(self) -> None:
        from rob_box_animations.animation_player import AnimationPlayer
        assert AnimationPlayer._manifest_name('idle.yaml') == 'idle'

    def test_name_without_yaml_suffix(self) -> None:
        from rob_box_animations.animation_player import AnimationPlayer
        assert AnimationPlayer._manifest_name('idle') == 'idle'

    def test_absolute_path(self) -> None:
        from rob_box_animations.animation_player import AnimationPlayer
        assert (
            AnimationPlayer._manifest_name('/abs/path/to/happy.yaml')
            == 'happy'
        )

    def test_relative_path(self) -> None:
        from rob_box_animations.animation_player import AnimationPlayer
        assert (
            AnimationPlayer._manifest_name('./manifests/happy.yaml')
            == 'happy'
        )