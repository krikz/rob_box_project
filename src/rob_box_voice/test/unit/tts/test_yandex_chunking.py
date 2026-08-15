"""Regression tests for kanban task ``t_c38e0128`` (issue #931).

The Yandex Cloud TTS gRPC API v3 enforces a per-request text-length cap
(см. https://cloud.yandex.ru/docs/speechkit/tts/limits — 2500 символов на
одну ``UtteranceSynthesis`` RPC). LLM-ответы длиннее 2400 символов
(анекдоты, рассказы) приводили к ``INVALID_ARGUMENT - Too long text`` от
Yandex и фолбэку на Silero (с 7-сек задержкой lazy-load после #929).

This test enforces:

1. ``TTSNode._chunk_text`` splits text по границам предложений
   (``.``, ``!``, ``?``, ``\\n``), гарантируя, что каждый чанк ≤
   :data:`YANDEX_MAX_CHUNK_CHARS` (default 2400).
2. Если одно предложение длиннее лимита — split по whitespace.
3. ``_synthesize_yandex`` для длинного текста делает несколько gRPC
   вызовов (по одному на чанк) и склеивает аудио. Один ``speech_id``
   обслуживает все чанки.
4. Если chunk-вызов падает, исключение пробрасывается наверх — caller
   переключается на Silero fallback для всего текста (старое
   поведение #929 сохранено).
5. Короткий текст (≤ лимита) → ровно 1 gRPC-вызов, никаких накладных
   расходов на чанкинг.

Tests use AST inspection where possible (no rclpy/grpc required) and
pytest's parameterization for compact enumeration. The runtime test for
``_synthesize_yandex`` uses a stub gRPC stub + helper fakes imported
from the project ``conftest.py`` for the heavy ROS surface.
"""
from __future__ import annotations

import ast
import io
import struct
import wave
from pathlib import Path

import numpy as np
import pytest


# ── AST-level invariants (no rclpy/grpc needed) ───────────────────────────────


_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
_TTS_NODE = _PACKAGE_ROOT / "rob_box_voice" / "tts_node.py"


def _parse_module(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _find_class_member(src: str, class_name: str, member_name: str) -> ast.FunctionDef:
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for member in node.body:
                if isinstance(member, ast.FunctionDef) and member.name == member_name:
                    return member
    raise LookupError(f"{class_name}.{member_name} not found")


def _find_module_assign(src: str, target_id: str) -> ast.AnnAssign | ast.Assign:
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id == target_id:
            return node
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == target_id:
                    return node
    raise LookupError(f"{target_id} not found at module scope")


def test_yandex_max_chunk_chars_constant_exists() -> None:
    """Module declares YANDEX_MAX_CHUNK_CHARS with safe Yandex limit."""
    src = _parse_module(_TTS_NODE)
    node = _find_module_assign(src, "YANDEX_MAX_CHUNK_CHARS")
    # Expect int annotation or constant
    code = ast.unparse(node)
    assert "YANDEX_MAX_CHUNK_CHARS" in code
    # Either an annotated int or a provider limit expression; runtime value is
    # asserted by test_tts_chunking.py.
    assert isinstance(node, (ast.AnnAssign, ast.Assign))


def test_chunk_text_is_static_method() -> None:
    """``_chunk_text`` must be a @staticmethod on TTSNode (so it can be unit-tested
    without instantiating the ROS node)."""
    fn = _find_class_member(_parse_module(_TTS_NODE), "TTSNode", "_chunk_text")
    decos = [ast.unparse(d) for d in fn.decorator_list]
    assert "staticmethod" in decos, f"_chunk_text must be @staticmethod, got {decos}"


def test_synthesize_yandex_uses_retry_helper() -> None:
    """Yandex hot-path must use retry-halving around its single-RPC helper."""
    src = _parse_module(_TTS_NODE)
    yandex = _find_class_member(src, "TTSNode", "_synthesize_yandex")
    yandex_src = ast.unparse(yandex)

    assert "synthesize_with_retry(" in yandex_src
    assert "_synthesize_yandex_single" in yandex_src
    assert "chunk_max_chars_yandex" in yandex_src
    assert "np.concatenate" in yandex_src or "concatenate" in yandex_src

    single = _find_class_member(src, "TTSNode", "_synthesize_yandex_single")
    single_src = ast.unparse(single)
    assert "synthesize_with_retry" not in single_src
    assert "UtteranceSynthesis(" in single_src


def test_synthesize_yandex_logs_per_chunk_latency() -> None:
    """Issue #931 acceptance: latency per chunk must be logged.

    ``_synthesize_yandex`` (multi-chunk path) and
    ``_synthesize_yandex_single_with_latency`` (retry-halve path) both
    measure synthesis time and log it, so operators can see per-chunk
    latency without instrumenting gRPC.
    """
    src = _parse_module(_TTS_NODE)
    yandex = _find_class_member(src, "TTSNode", "_synthesize_yandex")
    yandex_src = ast.unparse(yandex)
    assert "time.monotonic()" in yandex_src
    assert "Yandex chunk" in yandex_src
    assert "ms" in yandex_src

    latency_wrapper = _find_class_member(
        src, "TTSNode", "_synthesize_yandex_single_with_latency"
    )
    latency_src = ast.unparse(latency_wrapper)
    assert "time.monotonic()" in latency_src
    assert "Yandex synth" in latency_src
    assert "_synthesize_yandex_single(" in latency_src
    assert "ms" in latency_src


def test_fallback_to_silero_preserved() -> None:
    """Когда Yandex chunk падает, caller (_synthesize_and_play) переключается
    на Silero для всего текста (старое поведение #929)."""
    src = _parse_module(_TTS_NODE)
    fn = _find_class_member(src, "TTSNode", "_synthesize_and_play")
    fn_src = ast.unparse(fn)
    # The except clause must set audio_np=None and the subsequent code
    # must switch to silero_model.apply_tts(...)
    assert "silero_model.apply_tts" in fn_src, "Silero fallback path must be intact"
    assert "Yandex gRPC отвалился" in fn_src, "Yandex failure log message must be present"


# ── Behavioural tests for _chunk_text (pure Python) ─────────────────────────


@pytest.fixture(autouse=True)
def _import_chunk_text():
    """Import ``_chunk_text`` without instantiating TTSNode (no rclpy)."""
    sys_modules_backup = {}
    try:
        from test.unit.tts import conftest as _cf

        _cf._install_all_mocks()
    except Exception:
        pass
    import sys

    # The project conftest stubs ``grpc`` and ``yandex.cloud.ai.tts.v3``
    # via MagicMock — but ``tts_node._synthesize_yandex_single`` calls
    # ``tts_pb2.UtteranceSynthesisRequest(...)`` as a real constructor.
    # Provide a tiny stand-in so multi-chunk behavioural tests can
    # exercise the full code path without the real yandex SDK.
    import types

    yandex_v3 = sys.modules.get("yandex.cloud.ai.tts.v3")
    if yandex_v3 is None or not hasattr(yandex_v3, "tts_pb2"):

        class _Container:
            WAV = 0

            def __init__(self, container_audio_type=None):
                self.container_audio_type = container_audio_type

        class _AudioFormatOptions:
            def __init__(self, container_audio=None):
                self.container_audio = container_audio

        class _Hints:
            def __init__(self, voice=None, speed=None):
                self.voice = voice
                self.speed = speed

        class _UtteranceSynthesisRequest:
            LUFS = 0

            def __init__(self, text=None, output_audio_spec=None, hints=None,
                         loudness_normalization_type=None):
                self.text = text
                self.output_audio_spec = output_audio_spec
                self.hints = hints
                self.loudness_normalization_type = loudness_normalization_type

        class _TTS_pb2:
            ContainerAudio = _Container
            AudioFormatOptions = _AudioFormatOptions
            Hints = _Hints
            UtteranceSynthesisRequest = _UtteranceSynthesisRequest

        if yandex_v3 is None:
            # Build the full stub chain (matching the import path).
            yandex_root = types.ModuleType("yandex")
            yc = types.ModuleType("yandex.cloud")
            yc_ai = types.ModuleType("yandex.cloud.ai")
            yc_ai_tts = types.ModuleType("yandex.cloud.ai.tts")
            yc_ai_tts_v3 = types.ModuleType("yandex.cloud.ai.tts.v3")
            yc_ai_tts_v3.tts_pb2 = _TTS_pb2()
            yc_ai_tts_v3.tts_service_pb2_grpc = types.SimpleNamespace(
                SynthesizerStub=lambda *a, **kw: None
            )
            sys.modules["yandex"] = yandex_root
            sys.modules["yandex.cloud"] = yc
            sys.modules["yandex.cloud.ai"] = yc_ai
            sys.modules["yandex.cloud.ai.tts"] = yc_ai_tts
            sys.modules["yandex.cloud.ai.tts.v3"] = yc_ai_tts_v3
        else:
            yandex_v3.tts_pb2 = _TTS_pb2()
            yandex_v3.tts_service_pb2_grpc = types.SimpleNamespace(
                SynthesizerStub=lambda *a, **kw: None
            )

    sys.path.insert(0, str(_PACKAGE_ROOT))
    # Force re-import in case test order matters
    if "rob_box_voice.tts_node" in sys.modules:
        del sys.modules["rob_box_voice.tts_node"]
    from rob_box_voice.tts_node import TTSNode, YANDEX_MAX_CHUNK_CHARS

    yield TTSNode, YANDEX_MAX_CHUNK_CHARS

    sys.modules.pop("rob_box_voice.tts_node", None)


def test_short_text_no_split(_import_chunk_text):
    TTSNode, MAX = _import_chunk_text
    text = "Hello world. " * 5  # ~65 chars
    chunks = TTSNode._chunk_text(text, max_chars=MAX)
    assert len(chunks) == 1
    assert chunks[0] == text.strip()


def test_empty_text_returns_empty_list(_import_chunk_text):
    TTSNode, MAX = _import_chunk_text
    assert TTSNode._chunk_text("", max_chars=MAX) == []
    assert TTSNode._chunk_text("   ", max_chars=MAX) == []


@pytest.mark.parametrize(
    "repeat",
    [300, 500, 800, 1200],
)
def test_sentence_split_stays_within_limit(_import_chunk_text, repeat):
    TTSNode, MAX = _import_chunk_text
    text = ("Sentence. " * repeat).strip()
    assert len(text) > MAX, f"need text > {MAX}, got {len(text)}"  # pre-condition
    chunks = TTSNode._chunk_text(text, max_chars=MAX)
    # Each chunk ≤ limit
    assert all(len(c) <= MAX for c in chunks), [len(c) for c in chunks]
    # Content round-trips (modulo whitespace)
    assert "".join(text.split()) == "".join("".join(chunks).split())
    # At least 2 chunks
    assert len(chunks) >= 2


def test_word_split_when_single_sentence_exceeds_limit(_import_chunk_text):
    TTSNode, MAX = _import_chunk_text
    text = " ".join(["word"] * 1000) + "."  # ~6001 chars, one giant sentence
    chunks = TTSNode._chunk_text(text, max_chars=MAX)
    assert all(len(c) <= MAX for c in chunks), max(len(c) for c in chunks)
    assert len(chunks) >= 2


def test_exactly_at_limit_no_split(_import_chunk_text):
    TTSNode, MAX = _import_chunk_text
    text = "a" * MAX
    chunks = TTSNode._chunk_text(text, max_chars=MAX)
    assert chunks == [text]


def test_over_by_one(_import_chunk_text):
    TTSNode, MAX = _import_chunk_text
    text = "a" * (MAX + 1)
    chunks = TTSNode._chunk_text(text, max_chars=MAX)
    assert all(len(c) <= MAX for c in chunks)
    assert len(chunks) >= 2


def test_paragraph_break_used_as_separator(_import_chunk_text):
    TTSNode, MAX = _import_chunk_text
    text = ("Hello.\nWorld!\nHow are you?\n" * 200).strip()
    chunks = TTSNode._chunk_text(text, max_chars=MAX)
    assert all(len(c) <= MAX for c in chunks)
    # Round-trip preserves content
    assert "".join(text.split()) == "".join("".join(chunks).split())


def test_long_anecdote_realistic(_import_chunk_text):
    """Realistic LLM anecdote ~5k chars."""
    TTSNode, MAX = _import_chunk_text
    text = (
        "Once upon a time in a small village, there lived a brave robot. "
        "It helped everyone in need. "
    ) * 100
    assert len(text) > 5000
    chunks = TTSNode._chunk_text(text, max_chars=MAX)
    assert all(len(c) <= MAX for c in chunks)
    assert len(chunks) >= 2


def test_small_custom_limit(_import_chunk_text):
    TTSNode, _ = _import_chunk_text
    chunks = TTSNode._chunk_text("a. b. c. d. e. f.", max_chars=5)
    assert all(len(c) <= 5 for c in chunks), chunks


# ── Multi-chunk _synthesize_yandex behavioural test ──────────────────────────


def _make_wav_bytes(num_samples: int = 2205, sample_rate: int = 22050) -> bytes:
    """Construct a minimal valid mono PCM16 WAV."""
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        frames = b""
        for i in range(num_samples):
            v = int(8000 * np.sin(i / 10))
            frames += struct.pack("<h", v)
        w.writeframes(frames)
    return buf.getvalue()


class _FakeResponse:
    def __init__(self, audio_data: bytes):
        self.audio_chunk = type("Chunk", (), {"data": audio_data})()


class _FakeYandexStub:
    """Records each UtteranceSynthesis call; returns a canned WAV."""

    def __init__(self):
        self.calls: list[tuple[str, tuple]] = []

    def UtteranceSynthesis(self, request, metadata=None):
        self.calls.append((request.text, metadata or ()))
        # 2205 samples ≈ 100ms @ 22050Hz — small but real audio.
        return iter([_FakeResponse(_make_wav_bytes(num_samples=2205))])


def _build_node(stub):
    """Build a TTSNode instance bypassing __init__ (no rclpy available)."""
    from rob_box_voice.tts_node import (
        DEFAULT_MAX_RETRIES,
        MIN_CHUNK_CHARS,
        TTSNode,
        YANDEX_MAX_CHUNK_CHARS,
    )

    node = TTSNode.__new__(TTSNode)
    node.yandex_stub = stub
    node.yandex_voice = "anton"
    node.yandex_speed = 1.0
    node.yandex_api_key = "test-key"
    node.chunk_max_chars_yandex = YANDEX_MAX_CHUNK_CHARS
    node.chunk_max_retries = DEFAULT_MAX_RETRIES
    node.chunk_min_chars = MIN_CHUNK_CHARS

    class _Logger:
        def info(self, *a, **kw):
            pass

        def warn(self, *a, **kw):
            pass

        def error(self, *a, **kw):
            pass

        def debug(self, *a, **kw):
            pass

    node.get_logger = lambda: _Logger()
    return node


def test_short_text_one_grpc_call(_import_chunk_text):
    """Текст ≤ лимита → 1 gRPC-вызов, 1 np.ndarray, dtype float32."""
    _, MAX = _import_chunk_text
    from rob_box_voice.tts_node import TTSNode

    stub = _FakeYandexStub()
    node = _build_node(stub)
    audio = TTSNode._synthesize_yandex(node, "Short text.")  # type: ignore[arg-type]
    assert len(stub.calls) == 1
    assert isinstance(audio, np.ndarray)
    assert audio.dtype == np.float32
    assert audio.ndim == 1


def test_long_text_multi_chunk_concatenated(_import_chunk_text):
    """Текст >2400 chars → N gRPC-вызовов, склеенный аудио-массив."""
    _, MAX = _import_chunk_text
    from rob_box_voice.tts_node import TTSNode

    stub = _FakeYandexStub()
    node = _build_node(stub)

    text = ("This is a sentence about robots. " * 220).strip()
    assert len(text) > MAX

    audio = TTSNode._synthesize_yandex(node, text)  # type: ignore[arg-type]

    assert len(stub.calls) >= 2, f"expected ≥2 chunks, got {len(stub.calls)}"
    # Every chunk ≤ MAX
    for i, (chunk_text, _) in enumerate(stub.calls):
        assert len(chunk_text) <= MAX, f"chunk {i} has {len(chunk_text)} > {MAX}"
    # Concatenated audio is a single 1-D float32 array
    assert isinstance(audio, np.ndarray)
    assert audio.ndim == 1
    assert audio.dtype == np.float32
    assert len(audio) > 0


def test_synthesize_yandex_retries_too_long_chunk(_import_chunk_text):
    """Yandex INVALID_ARGUMENT must halve the rejected chunk before fallback."""
    from rob_box_voice.tts_node import TTSNode

    class ThresholdStub(_FakeYandexStub):
        def __init__(self, max_chars: int):
            super().__init__()
            self.max_chars = max_chars
            self.attempts: list[str] = []

        def UtteranceSynthesis(self, request, metadata=None):
            self.attempts.append(request.text)
            if len(request.text) > self.max_chars:
                raise Exception(
                    "Yandex gRPC error: INVALID_ARGUMENT - Too long text"
                )
            return super().UtteranceSynthesis(request, metadata)

    stub = ThresholdStub(max_chars=180)
    node = _build_node(stub)
    node.chunk_max_chars_yandex = 700
    node.chunk_max_retries = 3
    node.chunk_min_chars = 50
    text = ("Retry this phrase. " * 15).strip()
    assert 180 < len(text) < node.chunk_max_chars_yandex

    audio = TTSNode._synthesize_yandex(node, text)  # type: ignore[arg-type]

    assert len(stub.attempts) == 3
    assert stub.attempts[0] == text
    assert all(len(chunk) <= 180 for chunk in stub.attempts[1:])
    assert isinstance(audio, np.ndarray)
    assert audio.dtype == np.float32


def test_chunk_failure_propagates_for_silero_fallback(_import_chunk_text):
    """Если chunk gRPC падает, исключение пробрасывается — caller фолбэчит
    на Silero для всего текста."""
    _, MAX = _import_chunk_text
    from rob_box_voice.tts_node import TTSNode

    stub = _FakeYandexStub()
    call_count = {"n": 0}
    original = stub.UtteranceSynthesis

    def flaky(request, metadata=None):
        call_count["n"] += 1
        if call_count["n"] == 2:
            # Simulate Yandex INVALID_ARGUMENT — Too long text.
            # Use a generic Exception (matches real wrapper behaviour).
            raise Exception("Yandex gRPC error: INVALID_ARGUMENT - Too long text")
        return original(request, metadata)

    stub.UtteranceSynthesis = flaky
    node = _build_node(stub)
    text = ("OK sentence. " * 300).strip()
    assert len(text) > MAX

    with pytest.raises(Exception, match=r"(Too long text|Yandex (gRPC|synthesis) error)"):
        TTSNode._synthesize_yandex(node, text)  # type: ignore[arg-type]