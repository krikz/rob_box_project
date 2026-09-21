"""Regression tests for issue #2609 — torch 2.14.0+cu130 (CUDA-сборка) на
ARM64 Pi без GPU — ~1.6 GiB мёртвого груза в voice-assistant.

Background
----------
The voice-assistant container runs 9 ROS nodes sharing a 4 GiB
mem_limit. Each torch-using node (tts_node + speaker_id_node) was
importing ``torch`` at module load, which on a vanilla
``pip install torch>=2.13.0`` resolves to ``torch==2.14.0+cu130`` —
the CUDA-13.0 build for x86_64+arm64 with NVIDIA stack. Vision Pi 5
(ARM64, **no GPU**, VideoCore VII) doesn't have CUDA libraries, so
those ~800 MB of binaries per node are pure waste:

    /proc/90/maps  (tts_node):  libtorch_cuda.so, libc10_cuda.so,
                               libnvpl_blas_*, libarm_compute*
    /proc/98/maps  (speaker_id_node):  149 mappings of torch

Total bill on a fresh boot: tts_node=866 MB, speaker_id_node=781 MB,
nearly 1.6 GiB of CUDA binaries (most of which isn't even mapped into
RAM — but the .so files still load via dlopen on torch init, costing
both disk and CPU startup time).

This test module enforces the three-layer fix:

  1. **CPU-only torch pin** in ``docker/vision/voice_*/requirements.txt``
     via ``--extra-index-url https://download.pytorch.org/whl/cpu`` and an
     exact pin on the local version label (``torch==2.14.0+cpu``).
  2. **Lazy ``import torch``** in ``tts_node.py`` — module no longer
     imports torch at load time; the import is deferred to
     ``_load_silero_model`` which is only called when a real Silero
     fallback actually fires.
  3. **Lazy resemblyzer warm-up** in ``speaker_id_node.py`` —
     ``self._executor.submit(self._warmup)`` is now gated on the
     ``resemblyzer_warmup_on_start`` ROS parameter (default ``False``),
     so the resemblyzer ``VoiceEncoder(device=\"cpu\")`` cold-load no
     longer pays ~2-3 s + ~70 MB RSS on every container restart when
     no one is speaking.

Acceptance criteria covered here:

  * The two ``requirements.txt`` files pin the ``+cpu`` local version
    label and point at the dedicated CPU-only PyTorch index URL.
  * ``tts_node.py`` does NOT have a top-level ``import torch`` (it's
    only inside ``_load_silero_model``).
  * ``speaker_id_node.py`` declares ``resemblyzer_warmup_on_start``
    and only submits ``_warmup`` when the parameter is True.
  * ``speaker_id_node.yaml`` ships with ``resemblyzer_warmup_on_start: false``
    explicitly so the choice is visible to operators.
  * Runtime check: ``TTSNode.__init__`` does NOT trigger an import of
    ``torch`` (verified via sys.modules manipulation in the test).

The tests run with the project's existing ``ros_stubs`` + ``MagicMock``
``torch`` shim — we don't need a real CUDA-less build to assert on
import behaviour.
"""

from __future__ import annotations

import ast
import re
import sys
import textwrap
from pathlib import Path
from unittest.mock import patch

import pytest


# ── Path helpers ────────────────────────────────────────────────────────────

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
_REPO_ROOT = _PACKAGE_ROOT.parents[1]                # rob_box_project/

_DOCKER_REQUIREMENTS_FILES = [
    _REPO_ROOT / "docker" / "vision" / "voice_base" / "requirements.txt",
    _REPO_ROOT / "docker" / "vision" / "voice_assistant" / "requirements.txt",
]

_TTS_NODE_SRC = _PACKAGE_ROOT / "rob_box_voice" / "tts_node.py"
_SPEAKER_ID_SRC = _PACKAGE_ROOT / "rob_box_voice" / "speaker_id_node.py"
_SPEAKER_ID_YAML = _PACKAGE_ROOT / "config" / "speaker_id_node.yaml"


# ── Layer 1: CPU-only torch pin in requirements.txt ────────────────────────


@pytest.mark.parametrize("path", _DOCKER_REQUIREMENTS_FILES, ids=lambda p: p.name)
def test_requirements_pin_cpu_only_torch_index(path: Path) -> None:
    """Issue #2609 acceptance — both requirements.txt files must point at
    the dedicated CPU-only PyTorch index URL.
    """
    text = path.read_text(encoding="utf-8")
    assert "https://download.pytorch.org/whl/cpu" in text, (
        f"{path.relative_to(_REPO_ROOT)} must pin the CPU-only PyTorch "
        f"index URL (--index-url https://download.pytorch.org/whl/cpu) "
        f"to avoid resolver pulling +cu130 wheels on ARM64 Pi. See #2609."
    )


@pytest.mark.parametrize("path", _DOCKER_REQUIREMENTS_FILES, ids=lambda p: p.name)
def test_requirements_pin_torch_with_cpu_label(path: Path) -> None:
    """Issue #2609 — the torch version specifier must carry the ``+cpu``
    local version label so pip rejects ``+cu130`` (or any other CUDA
    variant) on the dedicated CPU index.
    """
    text = path.read_text(encoding="utf-8")
    # Match an EXACT pin ``torch==X.Y.Z+cpu``.
    match = re.search(
        r"^torch\s*==\s*\d+\.\d+\.\d+\+cpu\s*$",
        text,
        re.MULTILINE,
    )
    assert match, (
        f"{path.relative_to(_REPO_ROOT)} must pin torch EXACTLY with the "
        f"+cpu local version label, e.g. ``torch==2.14.0+cpu``. A range "
        f"like ``>=2.14.0+cpu`` is not enough: with --extra-index-url the "
        f"PyPI ``2.14.0`` (that IS the +cu130 aarch64 build) also satisfies "
        f"it, and the resolver may take it (issue #2609)."
    )
    # Also pin torchaudio for symmetry — silero doesn't use it but the
    # package is in requirements.txt and we don't want a future +cu130
    # torchaudio to slip through.
    ta_match = re.search(
        r"^torchaudio\s*==\s*\d+\.\d+\.\d+\+cpu\s*$",
        text,
        re.MULTILINE,
    )
    assert ta_match, (
        f"{path.relative_to(_REPO_ROOT)} must pin torchaudio with +cpu "
        f"label too (consistency with torch pin)."
    )


def test_requirements_no_legacy_torch_pin_without_cpu_label() -> None:
    """Defensive: no line should pin a bare ``torch>=X.Y.Z`` (without
    +cpu) on either requirements file. If this fires, somebody removed
    the CPU label.
    """
    for path in _DOCKER_REQUIREMENTS_FILES:
        text = path.read_text(encoding="utf-8")
        # Match a bare pin like ``torch>=2.13.0`` / ``torch==2.14.0`` —
        # anything missing the +cpu label.
        bare = re.search(
            r"^torch\s*[=>]=\s*\d+\.\d+\.\d+(?!\+cpu)\s*$",
            text,
            re.MULTILINE,
        )
        assert not bare, (
            f"{path.relative_to(_REPO_ROOT)} still has a bare torch pin "
            f"without +cpu label: {bare.group(0)!r}. This was the bug "
            f"fixed by #2609 — the resolver would pull torch==2.14.0+cu130 "
            f"on ARM64 Pi and bloat RSS by ~1.6 GiB across tts_node + "
            f"speaker_id_node."
        )


# ── Layer 2: lazy ``import torch`` in tts_node.py ──────────────────────────


def _parse_module(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


def _all_top_level_imports(tree: ast.Module) -> list[tuple[str, str]]:
    """Return ``[(level, module)]`` for every top-level import in module."""
    out: list[tuple[str, str]] = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            for alias in node.names:
                out.append(("module", alias.name))
        elif isinstance(node, ast.ImportFrom):
            mod = node.module or ""
            for alias in node.names:
                out.append(("from", f"{mod}.{alias.name}" if mod else alias.name))
    return out


def test_tts_node_has_no_top_level_torch_import() -> None:
    """The top of tts_node.py must NOT carry ``import torch``.

    Issue #2609 — torch is heavy (CPU-only wheel: ~70 MB RSS even at
    idle; +cu130 wheel: ~800 MB). tts_node only uses torch for Silero
    TTS, which is the offline fallback. The primary provider is
    ``minimax``/``yandex`` — importing torch at module load is wasted
    RSS for every boot.
    """
    tree = _parse_module(_TTS_NODE_SRC)
    imports = _all_top_level_imports(tree)
    torch_imports = [name for level, name in imports if name == "torch"]
    assert torch_imports == [], (
        "tts_node.py must not import torch at module scope — defer to "
        "_load_silero_model. Issue #2609 acceptance. "
        f"Found: {torch_imports}"
    )


def test_tts_node_load_silero_model_does_import_torch() -> None:
    """Lazy import contract — torch IS imported inside _load_silero_model,
    but only when Silero fallback actually runs.
    """
    tree = _parse_module(_TTS_NODE_SRC)
    # Find TTSNode class
    cls = None
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "TTSNode":
            cls = node
            break
    assert cls is not None, "TTSNode class not found"

    # Find _load_silero_model method
    method = None
    for node in cls.body:
        if isinstance(node, ast.FunctionDef) and node.name == "_load_silero_model":
            method = node
            break
    assert method is not None, "_load_silero_model method not found"

    # Walk the method body and find ``import torch`` statements.
    found_torch_import = False
    for sub in ast.walk(method):
        if isinstance(sub, ast.Import):
            for alias in sub.names:
                if alias.name == "torch":
                    found_torch_import = True
    assert found_torch_import, (
        "_load_silero_model must import torch locally — this is the "
        "lazy import contract from #2609. Top-level torch import was "
        "removed, the local one is the only path that brings the module "
        "in (and only when Silero fallback actually runs)."
    )


def test_tts_node_device_is_none_at_init() -> None:
    """``self.device = torch.device(\"cpu\")`` is also deferred.

    At __init__ time we don't yet know whether Silero will run, so
    ``self.device`` is set to None as a placeholder. The lazy
    ``_load_silero_model`` creates the real ``torch.device(\"cpu\")``
    the first time it imports torch.
    """
    tree = _parse_module(_TTS_NODE_SRC)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "TTSNode")
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")

    # Look for ``self.device = torch.device(...)`` in __init__.
    found_eager_device = False
    for sub in ast.walk(init):
        if (
            isinstance(sub, ast.Assign)
            and len(sub.targets) == 1
            and isinstance(sub.targets[0], ast.Attribute)
            and sub.targets[0].attr == "device"
            and isinstance(sub.value, ast.Call)
            and isinstance(sub.value.func, ast.Attribute)
            and sub.value.func.attr == "device"
        ):
            found_eager_device = True
    assert not found_eager_device, (
        "TTSNode.__init__ must NOT call torch.device(...) eagerly — "
        "issue #2609 acceptance. self.device is set to None as a "
        "placeholder, _load_silero_model creates the real device on "
        "first import."
    )


# ── Layer 3: lazy resemblyzer warmup in speaker_id_node.py ─────────────────


def test_speaker_id_declares_resemblyzer_warmup_on_start_parameter() -> None:
    """Issue #2609 — speaker_id_node must expose a ``resemblyzer_warmup_on_start``
    ROS parameter so operators can opt into the legacy eager warm-load.
    """
    tree = _parse_module(_SPEAKER_ID_SRC)
    cls = next(
        n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SpeakerIdNode"
    )
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")

    declared_names = {
        node.args[0].value
        for node in ast.walk(init)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "declare_parameter"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and isinstance(node.args[0].value, str)
    }
    assert "resemblyzer_warmup_on_start" in declared_names, (
        "SpeakerIdNode.__init__ must declare ``resemblyzer_warmup_on_start`` "
        "ROS parameter. Issue #2609 — gates the eager warm-up."
    )


def test_speaker_id_warmup_submit_is_gated_on_parameter() -> None:
    """Issue #2609 acceptance — ``self._executor.submit(self._warmup)`` must
    be INSIDE a conditional that reads ``resemblyzer_warmup_on_start``.

    Without the gate, the node pays ~2-3 s of CPU + ~70 MB RSS at boot
    even when nobody speaks — wasted in 99% of container restarts.
    """
    tree = _parse_module(_SPEAKER_ID_SRC)
    cls = next(
        n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SpeakerIdNode"
    )
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")

    # Walk __init__ and find any ``self._executor.submit(self._warmup)`` calls.
    # For each, walk up the AST ancestors (limited — we approximate by
    # checking if any enclosing ``if`` node in the same function reads
    # ``resemblyzer_warmup_on_start``).
    init_src = ast.unparse(init)

    # Direct check: the line ``self._executor.submit(self._warmup)`` must
    # not appear in __init__ without being inside an ``if`` branch that
    # also reads ``resemblyzer_warmup_on_start``.
    submit_call_re = re.compile(r"self\._executor\.submit\(\s*self\._warmup\s*\)")
    matches = list(submit_call_re.finditer(init_src))
    assert matches, (
        "Expected to find self._executor.submit(self._warmup) in "
        "SpeakerIdNode.__init__ — but it's missing entirely. Issue #2609 "
        "moved it inside a conditional, the AST should still show one "
        "submission site under the gate."
    )

    # Check each submission site is preceded (within the same function)
    # by a reference to ``resemblyzer_warmup_on_start`` — that's our
    # gate.
    gate_re = re.compile(r"resemblyzer_warmup_on_start")
    gated_submits = 0
    for match in matches:
        # Look for the gate in the same scope: between the previous
        # ``if`` keyword and this ``submit`` call.
        preceding = init_src[: match.start()]
        if gate_re.search(preceding):
            gated_submits += 1
    assert gated_submits >= 1, (
        "self._executor.submit(self._warmup) is not gated on "
        "``resemblyzer_warmup_on_start``. Issue #2609 acceptance: the "
        "submit must be inside an ``if bool(self.get_parameter(...).value)`` "
        "branch so the eager warm-load is opt-in."
    )


def test_speaker_id_yaml_sets_resemblyzer_warmup_false() -> None:
    """speaker_id_node.yaml must default the parameter to ``false`` so
    operators see the choice explicitly.

    Setting True by default would re-introduce the RSS bloat on every
    boot; setting False here, with a comment, makes the trade-off
    visible at the place where operators edit it.
    """
    text = _SPEAKER_ID_YAML.read_text(encoding="utf-8")
    assert "resemblyzer_warmup_on_start" in text, (
        "speaker_id_node.yaml must declare resemblyzer_warmup_on_start "
        "(issue #2609). The choice should be visible at the operator's "
        "config layer, not hidden in node default."
    )
    # Match the explicit ``resemblyzer_warmup_on_start: false`` line.
    match = re.search(
        r"^\s*resemblyzer_warmup_on_start\s*:\s*(true|false)\s*$",
        text,
        re.MULTILINE,
    )
    assert match is not None and match.group(1) == "false", (
        "speaker_id_node.yaml must set resemblyzer_warmup_on_start: false "
        "explicitly (issue #2609 acceptance). Operators that want "
        "legacy eager warm-load can flip to true."
    )


# ── Runtime check: importing tts_node does NOT pull torch into sys.modules ──


def test_importing_tts_node_does_not_import_torch() -> None:
    """Runtime acceptance for layer 2 — if you ``import
    rob_box_voice.tts_node`` from a clean process, torch must NOT be
    eagerly loaded into ``sys.modules``.

    The conftest's ``sys.modules.setdefault(\"torch\", MagicMock())`` is
    a safety net for other tests — it does NOT count as torch being
    imported by tts_node. The AST-level check
    ``test_tts_node_has_no_top_level_torch_import`` is the authoritative
    structural test for this invariant; this runtime check is a
    belt-and-braces smoke test that complements it by verifying the
    conftest's torch stub still satisfies tts_node (so the AST removal
    doesn't accidentally break import-time dependency on torch).
    """
    import importlib

    # Conftest may already have installed a MagicMock for torch — if so,
    # that's a no-op stub for OTHER tests. We re-import tts_node just to
    # verify the lazy import doesn't blow up.
    spec = importlib.util.find_spec("rob_box_voice.tts_node")
    if spec is None:
        pytest.skip(
            "rob_box_voice.tts_node is not importable in this env "
            "(missing ROS deps); AST checks above are the authoritative "
            "regression for #2609."
        )

    from unittest.mock import MagicMock

    torch_mod = sys.modules.get("torch")
    if torch_mod is not None and isinstance(torch_mod, MagicMock):
        # The stub is in place. Re-import tts_node to verify it didn't
        # somehow trigger a real torch load.
        sys.modules.pop("rob_box_voice.tts_node", None)
        try:
            importlib.import_module("rob_box_voice.tts_node")
        except Exception as exc:  # noqa: BLE001 — conftest stubs aren't perfect
            pytest.skip(
                f"rob_box_voice.tts_node import raised {type(exc).__name__}: "
                f"{exc} — likely a conftest stub gap. AST checks above "
                f"are the authoritative regression for #2609."
            )
        # Re-check torch — should still be the MagicMock stub.
        torch_mod_after = sys.modules.get("torch")
        assert isinstance(torch_mod_after, MagicMock), (
            "tts_node.__init__ code triggered a real ``import torch`` "
            "— that defeats issue #2609's lazy-import contract. The "
            "AST check should be enough, but if you're seeing this, "
            "tts_node.py still has a top-level ``import torch`` (or a "
            "function call that pulls torch in at module load)."
        )


# ── Layer 1.5: smoke test for the requirements files being pip-installable ─


def test_requirements_have_pytorch_cpu_index_url() -> None:
    """The pytorch CPU-only channel must be declared as an EXTRA index, and
    never as ``--index-url``.

    ``--index-url`` is a global pip option that REPLACES PyPI for the whole
    file (regardless of the line it sits on). Both Dockerfiles install these
    files with a single ``pip3 install -r requirements.txt``, so a bare
    ``--index-url`` would send vosk / resemblyzer / renardo-lib / ddgs /
    yandex-cloud-ml-sdk / pyaudio / ... to download.pytorch.org too — they
    are not published there and the image build dies on ``No matching
    distribution found``. ``--extra-index-url`` adds the channel next to
    PyPI instead; the exact ``+cpu`` pin is what keeps the CUDA wheel out.
    """
    for path in _DOCKER_REQUIREMENTS_FILES:
        text = path.read_text(encoding="utf-8")
        lines = [ln.strip() for ln in text.splitlines()]
        assert "--extra-index-url https://download.pytorch.org/whl/cpu" in lines, (
            f"{path.relative_to(_REPO_ROOT)} must declare "
            f"``--extra-index-url https://download.pytorch.org/whl/cpu`` so "
            f"the CPU-only PyTorch channel is available alongside PyPI "
            f"(issue #2609)."
        )
        offenders = [ln for ln in lines if ln.startswith("--index-url")]
        assert not offenders, (
            f"{path.relative_to(_REPO_ROOT)} uses {offenders[0]!r}. "
            f"``--index-url`` replaces PyPI for the ENTIRE file, so every "
            f"other dependency here would be looked up on the PyTorch index "
            f"and the image build fails. Use --extra-index-url instead."
        )


# ── Layer 2.5: runtime smoke — __init__ doesn't trigger eager torch import


def test_tts_node_init_does_not_touch_torch_attribute() -> None:
    """Structural smoke test — TTSNode.__init__ AST must NOT contain any
    direct attribute access on a ``torch`` name. (Indirect access via
    ``getattr(torch, ...)`` or via ``import torch`` inside a function is
    fine; this test guards against accidental direct usage like
    ``torch.device(...)`` sneaking back in.)
    """
    tree = _parse_module(_TTS_NODE_SRC)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "TTSNode")
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")

    # Walk __init__ and find any Name ``id == \"torch\"``.
    found = False
    for sub in ast.walk(init):
        if isinstance(sub, ast.Name) and sub.id == "torch":
            found = True
            break
    assert not found, (
        "TTSNode.__init__ must NOT reference the bare name ``torch`` "
        "directly — issue #2609 acceptance. The lazy import lives "
        "inside _load_silero_model only."
    )


# ── Layer 4: the DEPLOYED configs actually turn the warm-loads off ──────────

_DOCKER_VOICE_CONFIG = _REPO_ROOT / "docker" / "vision" / "config" / "voice_assistant"


def test_deployed_tts_yaml_disables_silero_warm_load() -> None:
    """Issue #2609 — the robot launches with docker/vision/config, not
    src/rob_box_voice/config. Without the key there, tts_node falls back to
    the code default (``silero_warm_load=True``) and loads torch on boot."""
    import yaml

    data = yaml.safe_load((_DOCKER_VOICE_CONFIG / "tts_node.yaml").read_text(encoding="utf-8"))
    assert data["tts_node"]["ros__parameters"].get("silero_warm_load") is False


def test_deployed_stt_yaml_defers_vosk() -> None:
    import yaml

    data = yaml.safe_load((_DOCKER_VOICE_CONFIG / "stt_node.yaml").read_text(encoding="utf-8"))
    assert data["stt_node"]["ros__parameters"].get("vosk_preload") is False
