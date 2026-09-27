"""sclang must resync the SynthDef palette itself after scsynth restarts alone.

Live incident, issue #1807. The palette (~300 compiled SynthDefs — startup
preload plus everything Python renardo sends later via ``/foxdot``) lives
only in scsynth's memory. It is loaded by sclang, which runs in a DIFFERENT
container (voice-assistant) and only does this once, at its own startup.

``docker restart supercollider`` alone brings scsynth back with nothing but
the RootNode. sclang never restarted, so nothing tells it to reload — and it
has no way to notice on its own unless it is watching for it. From outside
everything still looks healthy: both containers report healthy, the tool
that executes music code reports success, the logs are clean. The robot is
completely mute. Verified live: after ``docker restart supercollider``,
``makeSound``, ``startSound``, ``volume``, ``play1``, ``play2`` were all gone
from the server — without ``makeSound`` no note reaches the output at all.

Same shape of failure as the group-1 incident (commit 8c4079a7, "scsynth did
not create group 1") — there the *fix* lived in the side that survives a
restart of its partner (scsynth recreates its own group). Here it is sclang
that survives a restart of scsynth, so the same principle puts the fix in
sclang: track every SynthDef path ever loaded, watch ``serverRunning`` for a
true→false→true transition (a restart, not a cold boot), and replay the
whole known palette plus re-arm the master filter when it happens.

Not exercised live here (no ROS2/SuperCollider available in this test) — the
checks are structural, confirming the resync machinery is actually wired
into the script rather than merely described in a comment.
"""
from __future__ import annotations

import os
import re
from pathlib import Path


def _repo_root(start: Path) -> Path:
    """Locate the rob_box_project repo root, robust to dev vs CI layouts.

    Three layouts we must support:

    1. **Dev / krikz worktree** — test file lives under ``src/rob_box_voice/test/...``.
       Walking up: ``core → unit → test → rob_box_voice → src → <repo>``.
       The ``<repo>`` directory has both ``src/`` and ``docker/`` next to it.

    2. **GitHub Actions ``test_ws``** — CI mirrors ``src/`` into
       ``test_ws/src/`` and ``docker/`` into ``test_ws/docker/`` (see
       ``.github/workflows/G-Run Tests.yml``). Same walk lands at ``test_ws``
       and the parent's ``src/`` + ``docker/`` check matches.

    3. **``colcon test`` build tree** — when integration tests run under
       ``colcon test``, pytest may discover the test file from
       ``test_ws/build/<pkg>/...`` or from ``test_ws/install/...`` where the
       ``src/`` and ``docker/`` siblings are absent. In that case the env
       var ``ROB_BOX_REPO_ROOT`` (set by the workflow) is the only reliable
       anchor.

    Search order:

    * explicit override via ``ROB_BOX_REPO_ROOT`` env var (set in CI);
    * any ancestor whose direct children are both ``src/`` and ``docker/``;
    * any ancestor that contains a ``src/rob_box_voice`` subdir (dev-repo);
    * as a last resort, walk all the way up looking for a sibling pair.

    A hard ``RuntimeError`` is kept ONLY if nothing matches at all — that
    means the file is genuinely outside the repo, which is a configuration
    problem, not a CI vs dev mismatch.
    """
    override = os.environ.get("ROB_BOX_REPO_ROOT")
    if override:
        candidate = Path(override).expanduser().resolve()
        if (candidate / "src").is_dir():
            return candidate

    for parent in [start, *start.parents]:
        if (parent / "src").is_dir() and (parent / "docker").is_dir():
            return parent

    for parent in [start, *start.parents]:
        if (parent / "src" / "rob_box_voice").is_dir():
            return parent

    raise RuntimeError(
        f"repo root not found for {start!s}; set ROB_BOX_REPO_ROOT or run "
        "from inside rob_box_project"
    )


REPO_ROOT = _repo_root(Path(__file__).resolve())
FOXDOT_INIT = (
    REPO_ROOT / "docker" / "vision" / "voice_assistant" / "foxdot_init.sc"
)


def _script() -> str:
    return FOXDOT_INIT.read_text(encoding="utf-8")


def _executable_lines(text: str) -> str:
    """Drop full-line comments so incident write-ups don't fake a match."""
    return "\n".join(
        line for line in text.splitlines() if not line.strip().startswith("//")
    )


def test_script_exists() -> None:
    assert FOXDOT_INIT.is_file(), f"not found: {FOXDOT_INIT}"


def test_every_loaded_synthdef_path_is_remembered() -> None:
    """Both the startup preload and runtime ``/foxdot`` loads must be tracked.

    Without a full history there is nothing to replay after a restart is
    detected — sclang would only know about the 53 startup names and lose
    everything Python renardo pushed later at runtime.
    """
    code = _executable_lines(_script())
    assert re.search(r"\bloadedSynthPaths\s*=\s*Set\.new", code), (
        "нет коллекции, накапливающей пути ко всем загруженным .scd"
    )
    # Стартовый прелоад (startupSynths/customSynths) должен класть путь в неё.
    assert code.count("loadedSynthPaths.add(path)") >= 2, (
        "прелоад не запоминает свои пути — при рестарте scsynth "
        "перезаливать будет нечего"
    )
    # Рантайм-загрузка через /foxdot (то, что шлёт Python renardo) тоже
    # обязана попадать в историю — иначе после первого /foxdot-сообщения
    # список расходится с реальной палитрой scsynth.
    oscdef_block = code[code.index("'/foxdot'") - 400 : code.index("'/foxdot'")]
    assert "loadedSynthPaths.add(path)" in oscdef_block, (
        "обработчик /foxdot не запоминает путь — рантайм-палитра "
        "(то, что Python renardo шлёт после старта) не переживёт рестарт scsynth"
    )


def test_watches_server_running_for_a_restart_not_a_cold_boot() -> None:
    """Must react to true→false→true, and only that — not the initial boot.

    A watcher that fires on every ``serverRunning`` becoming true would also
    fire during the very first connection and double up with the normal
    startup preload below it in the file.
    """
    code = _executable_lines(_script())
    assert "Server.default.serverRunning" in code, (
        "нет обращения к serverRunning — нечем детектировать рестарт scsynth"
    )
    assert re.search(r"var\s+serverWasRunning\s*=\s*false", code), (
        "флаг предыдущего состояния сервера должен стартовать c false — "
        "иначе холодный старт спутается с восстановлением после дропа"
    )
    assert re.search(r"var\s+serverDropped\s*=\s*false", code), (
        "нет отдельного флага 'сервер уже пропадал' — без него первый "
        "коннект при холодном старте будет ошибочно принят за рестарт"
    )
    # Переход в true выставляется только когда сервер УЖЕ был живым.
    assert re.search(
        r"if\s*\(\s*serverWasRunning\s*&&\s*running\.not\s*\)", code
    ), "нет проверки перехода true→false (сервер пропал)"
    assert re.search(
        r"if\s*\(\s*serverDropped\s*&&\s*running\s*\)", code
    ), "нет проверки перехода false→true ПОСЛЕ дропа (сервер вернулся)"


def test_restart_recovery_replays_history_and_rearms_master_filter() -> None:
    """On recovery: reload every remembered path, then re-arm the master bus.

    The master bus synth (node 999, ``masterfilter``) is gone too after a
    scsynth restart — skipping this step would leave the mix without its
    anti-aliasing ceiling even though every instrument plays again.
    """
    code = _executable_lines(_script())
    assert "loadedSynthPaths.copy.do" in code, (
        "восстановление должно перебирать именно накопленную историю путей"
    )
    assert code.count("armMasterFilter.value") >= 2, (
        "armMasterFilter должен вызываться и на обычном старте, и при "
        "восстановлении после рестарта scsynth"
    )


def test_recovery_reuses_the_synced_loader_not_a_fire_and_forget_load() -> None:
    """Replaying the palette must wait for scsynth after each file.

    ``path.load`` only compiles inside sclang; ``/d_recv`` goes out over UDP
    unacknowledged. A bulk replay without ``Server.sync`` between files can
    drop definitions silently, exactly like the original 30.08 preload bug
    this same file already fixed once.
    """
    code = _executable_lines(_script())
    assert re.search(
        r"reloadKnownPath\s*=\s*\{[^}]*Server\.default\.sync", code, re.S
    ), "reloadKnownPath должен ждать Server.default.sync после path.load"
    assert "loadedSynthPaths.copy.do({ |path| reloadKnownPath.value(path) })" in code, (
        "перезаливка при восстановлении обязана идти через reloadKnownPath "
        "(с sync), а не голым path.load"
    )
