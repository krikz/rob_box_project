from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[2] / "docker/vision/scripts/supercollider/start_supercollider.sh"


def test_supercollider_cleans_host_jack_registry_and_db_dirs() -> None:
    script = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "jack-shm-registry" in script
    assert "jack_db-" in script


def test_supercollider_scsynth_pins_maxlogins_to_32() -> None:
    """Issue #1363: scsynth defaults to -l 64 (maxLogins), but sclang
    reserves default-group IDs as ``client_id * (2**26) + 1`` and assumes
    ≤32 clients. With server=64 / client=32 the group ID is 0x80000001,
    which as signed int32 = -2147483647 → ``/g_new negative node IDs
    are reserved`` and (downstream) a stuck synth that whistles through
    JACK.

    The script MUST pin ``-l 32`` on the scsynth invocation so the
    server's maxLogins matches what sclang expects.
    """

    script = SCRIPT_PATH.read_text(encoding="utf-8")
    scsynth_block = _extract_scsynth_invocation(script)

    assert "-l 32" in scsynth_block, (
        "scsynth must be started with `-l 32` (issue #1363); "
        "without it sclang gets negative node IDs and the music stack "
        "logs `FAILURE IN SERVER /g_new negative node IDs are reserved` "
        "after every `Clock.clear()` transition. Got invocation:\n"
        f"{scsynth_block}"
    )


def test_supercollider_scsynth_options_use_jack_backend() -> None:
    """Sanity check: ensure the scsynth invocation still uses JACK and
    the 16 kHz rate that the rest of the audio pipeline (dmix_respeaker,
    ReSpeaker UAC1.0) expects. Pinning -l 32 must not regress these.
    """

    script = SCRIPT_PATH.read_text(encoding="utf-8")
    scsynth_block = _extract_scsynth_invocation(script)

    assert "-H jack" in scsynth_block
    assert "-S 16000" in scsynth_block
    assert "-u 57110" in scsynth_block
    assert "-D 0" in scsynth_block  # no realtime in Docker


def _extract_scsynth_invocation(script: str) -> str:
    """Return the textual block that starts a `scsynth` invocation,
    terminated at the first `&` (background) line.
    """

    lines = script.splitlines()
    out: list[str] = []
    started = False
    for line in lines:
        stripped = line.strip()
        if not started:
            if stripped.startswith("scsynth"):
                started = True
                out.append(line)
            continue
        if stripped.endswith("&"):
            out.append(line)
            break
        out.append(line)
    return "\n".join(out)
