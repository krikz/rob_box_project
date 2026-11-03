"""Regression tests for kanban task ``t_c38e0128`` (issue #931).

The Yandex Cloud TTS gRPC API v3 enforces a per-request text-length cap
(см. https://cloud.yandex.ru/docs/speechkit/tts/limits — 2500 символов на
одну ``UtteranceSynthesis`` RPC). LLM-ответы длиннее 2400 символов
(анекдоты, рассказы) приводили к ``INVALID_ARGUMENT - Too long text`` от
Yandex и фолбэку на Silero (с 7-сек задержкой lazy-load после #929).

This test enforces:

Синтез Yandex живёт в адаптере ``rob_box_voice.tts_providers.yandex`` (#4240, ADR-0160 §2.5).

1. ``yandex.chunk_text`` splits text по границам предложений
   (``.``, ``!``, ``?``, ``\\n``), гарантируя, что каждый чанк ≤
   :data:`YANDEX_MAX_CHUNK_CHARS` (default 2400).
2. Если одно предложение длиннее лимита — split по whitespace.
3. ``YandexTTSProvider.synthesize_np`` для длинного текста делает несколько gRPC
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
_YANDEX = _PACKAGE_ROOT / "rob_box_voice" / "tts_providers" / "yandex.py"
_SILERO = _PACKAGE_ROOT / "rob_box_voice" / "tts_providers" / "silero.py"


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


def test_chunk_text_is_module_function_of_the_adapter() -> None:
    """``chunk_text`` — функция модуля адаптера (тестируется без ROS-ноды); в ``tts_node`` копии нет (#4240)."""
    tree = ast.parse(_parse_module(_YANDEX))
    assert any(isinstance(n, ast.FunctionDef) and n.name == "chunk_text" for n in tree.body)
    with pytest.raises(LookupError):
        _find_class_member(_parse_module(_TTS_NODE), "TTSNode", "_chunk_text")


def test_synthesize_yandex_uses_retry_helper() -> None:
    """Yandex hot-path must use retry-halving around its single-RPC helper."""
    src = _parse_module(_YANDEX)
    yandex = _find_class_member(src, "YandexTTSProvider", "synthesize_np")
    yandex_src = ast.unparse(yandex)

    assert "synthesize_with_retry(" in yandex_src
    assert "_synthesize_one" in yandex_src
    assert "_chunk_max_chars" in yandex_src
    assert "np.concatenate" in yandex_src or "concatenate" in yandex_src

    single = _find_class_member(src, "YandexTTSProvider", "_synthesize_one")
    single_src = ast.unparse(single)
    assert "synthesize_with_retry" not in single_src
    # ADR-0158 PR-6: сам RPC вынесен в tts_engines.yandex_collect_audio (общий с sing_synth)
    assert "yandex_collect_audio(" in single_src


def test_synthesize_yandex_logs_per_chunk_latency() -> None:
    """Issue #931 acceptance: latency per chunk must be logged.

    ``YandexTTSProvider.synthesize_np`` (multi-chunk path) and
    ``_synthesize_one_logged`` (retry-halve path) both
    measure synthesis time and log it, so operators can see per-chunk
    latency without instrumenting gRPC.
    """
    src = _parse_module(_YANDEX)
    yandex = _find_class_member(src, "YandexTTSProvider", "synthesize_np")
    yandex_src = ast.unparse(yandex)
    assert "time.monotonic()" in yandex_src
    assert "Yandex chunk" in yandex_src
    assert "ms" in yandex_src

    latency_wrapper = _find_class_member(src, "YandexTTSProvider", "_synthesize_one_logged")
    latency_src = ast.unparse(latency_wrapper)
    assert "time.monotonic()" in latency_src
    assert "Yandex synth" in latency_src
    assert "_synthesize_one(" in latency_src
    assert "ms" in latency_src


def test_fallback_to_silero_preserved() -> None:
    """Когда Yandex chunk падает, caller (_synthesize_and_play) переключается
    на Silero для всего текста (старое поведение #929)."""
    src = _parse_module(_TTS_NODE)
    fn = _find_class_member(src, "TTSNode", "_synthesize_and_play")
    fn_src = ast.unparse(fn)
    # Issue #2078 decomposition: silero fallback helper now owns the
    # ``silero_model.apply_tts(...)`` call (was inline in this method).
    # Search across the whole module so the contract survives extraction.
    module_src = _TTS_NODE.read_text(encoding="utf-8")
    # ADR-0158 PR-6: сам вызов apply_tts вынесен в ``tts_engines.silero_apply_tts``
    # (общий с sing_synth); #4240 — его зовёт адаптер Silero, нода зовёт адаптер.
    silero_src = _SILERO.read_text(encoding="utf-8")
    assert "silero_apply_tts(" in silero_src and "silero_tts.synthesize_np(" in module_src, (
        "Silero fallback path must be intact (search across module after "
        "issue #2078 decomposition)"
    )
    # The Yandex-failure log message MUST stay somewhere in the high-level
    # orchestrator or its helpers (per-issue #1083 acceptance:
    # «переключаюсь на Silero»).  Issue #2078 moved it to
    # ``_sap_synthesize_yandex``; search across the module.
    assert "Yandex gRPC отвалился" in module_src, (
        "Yandex failure log message must be present (module-wide after "
        "issue #2078 decomposition)"
    )


# ── Behavioural tests for _chunk_text (pure Python) ─────────────────────────


@pytest.fixture(autouse=True)
def _import_chunk_text():
    """Import ``chunk_text`` адаптера Yandex (no rclpy)."""
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

    def _needs_pb2_stand_in(module) -> bool:
        """True unless ``tts_pb2`` already builds a request that keeps its text.

        ``hasattr`` is useless here: the project conftest registers
        ``yandex.cloud.ai.tts.v3`` as a ``MagicMock``, which answers yes to
        every attribute. The guard passed, the stand-in below was skipped, and
        ``UtteranceSynthesisRequest(text=...)`` returned a MagicMock whose
        ``.text`` is another MagicMock — ``len()`` of which is 0, so the
        "too long text" branch never fired and the retry test saw one attempt
        instead of three. It passed alone (no MagicMock in ``sys.modules``
        yet) and failed in a directory run.
        """
        if module is None:
            return True
        pb2 = getattr(module, "tts_pb2", None)
        if pb2 is None:
            return True
        try:
            probe = pb2.UtteranceSynthesisRequest(text="probe")
        except Exception:  # noqa: BLE001 — anything unusable needs the stub
            return True
        return getattr(probe, "text", None) != "probe"

    yandex_v3 = sys.modules.get("yandex.cloud.ai.tts.v3")
    if _needs_pb2_stand_in(yandex_v3):

        class _Container:
            WAV = 0

            def __init__(self, container_audio_type=None):
                self.container_audio_type = container_audio_type

        class _AudioFormatOptions:
            def __init__(self, container_audio=None):
                self.container_audio = container_audio

        class _Hints:
            def __init__(self, voice=None, speed=None, pitch_shift=None,
                         volume=None):
                self.voice = voice
                self.speed = speed
                self.pitch_shift = pitch_shift
                self.volume = volume

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
    from rob_box_voice.tts_node import YANDEX_MAX_CHUNK_CHARS
    from rob_box_voice.tts_providers.yandex import chunk_text

    yield chunk_text, YANDEX_MAX_CHUNK_CHARS

    sys.modules.pop("rob_box_voice.tts_node", None)


def test_short_text_no_split(_import_chunk_text):
    chunk_text, MAX = _import_chunk_text
    text = "Hello world. " * 5  # ~65 chars
    chunks = chunk_text(text, max_chars=MAX)
    assert len(chunks) == 1
    assert chunks[0] == text.strip()


def test_empty_text_returns_empty_list(_import_chunk_text):
    chunk_text, MAX = _import_chunk_text
    assert chunk_text("", max_chars=MAX) == []
    assert chunk_text("   ", max_chars=MAX) == []


@pytest.mark.parametrize(
    "repeat",
    [300, 500, 800, 1200],
)
def test_sentence_split_stays_within_limit(_import_chunk_text, repeat):
    chunk_text, MAX = _import_chunk_text
    text = ("Sentence. " * repeat).strip()
    assert len(text) > MAX, f"need text > {MAX}, got {len(text)}"  # pre-condition
    chunks = chunk_text(text, max_chars=MAX)
    # Each chunk ≤ limit
    assert all(len(c) <= MAX for c in chunks), [len(c) for c in chunks]
    # Content round-trips (modulo whitespace)
    assert "".join(text.split()) == "".join("".join(chunks).split())
    # At least 2 chunks
    assert len(chunks) >= 2


def test_word_split_when_single_sentence_exceeds_limit(_import_chunk_text):
    chunk_text, MAX = _import_chunk_text
    text = " ".join(["word"] * 1000) + "."  # ~6001 chars, one giant sentence
    chunks = chunk_text(text, max_chars=MAX)
    assert all(len(c) <= MAX for c in chunks), max(len(c) for c in chunks)
    assert len(chunks) >= 2


def test_exactly_at_limit_no_split(_import_chunk_text):
    chunk_text, MAX = _import_chunk_text
    text = "a" * MAX
    chunks = chunk_text(text, max_chars=MAX)
    assert chunks == [text]


def test_over_by_one(_import_chunk_text):
    chunk_text, MAX = _import_chunk_text
    text = "a" * (MAX + 1)
    chunks = chunk_text(text, max_chars=MAX)
    assert all(len(c) <= MAX for c in chunks)
    assert len(chunks) >= 2


def test_paragraph_break_used_as_separator(_import_chunk_text):
    chunk_text, MAX = _import_chunk_text
    text = ("Hello.\nWorld!\nHow are you?\n" * 200).strip()
    chunks = chunk_text(text, max_chars=MAX)
    assert all(len(c) <= MAX for c in chunks)
    # Round-trip preserves content
    assert "".join(text.split()) == "".join("".join(chunks).split())


def test_long_anecdote_realistic(_import_chunk_text):
    """Realistic LLM anecdote ~5k chars."""
    chunk_text, MAX = _import_chunk_text
    text = (
        "Once upon a time in a small village, there lived a brave robot. "
        "It helped everyone in need. "
    ) * 100
    assert len(text) > 5000
    chunks = chunk_text(text, max_chars=MAX)
    assert all(len(c) <= MAX for c in chunks)
    assert len(chunks) >= 2


def test_small_custom_limit(_import_chunk_text):
    chunk_text, _ = _import_chunk_text
    chunks = chunk_text("a. b. c. d. e. f.", max_chars=5)
    assert all(len(c) <= 5 for c in chunks), chunks


# ── Multi-chunk YandexTTSProvider.synthesize_np behavioural test ──────────────────────────


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


def _build_node(stub, **limits):
    """Адаптер Yandex с фейковым gRPC-стабом (no rclpy available)."""
    from rob_box_voice.tts_providers.yandex import YandexTTSProvider

    class _Logger:
        def info(self, *a, **kw):
            pass

        def warn(self, *a, **kw):
            pass

        def error(self, *a, **kw):
            pass

        def debug(self, *a, **kw):
            pass

    return YandexTTSProvider("test-key", stub, logger=_Logger(), **limits)


def test_short_text_one_grpc_call(_import_chunk_text):
    """Текст ≤ лимита → 1 gRPC-вызов, 1 np.ndarray, dtype float32."""
    _, MAX = _import_chunk_text
    stub = _FakeYandexStub()
    node = _build_node(stub)
    audio = node.synthesize_np("Short text.")
    assert len(stub.calls) == 1
    assert isinstance(audio, np.ndarray)
    assert audio.dtype == np.float32
    assert audio.ndim == 1


def test_long_text_multi_chunk_concatenated(_import_chunk_text):
    """Текст >2400 chars → N gRPC-вызовов, склеенный аудио-массив."""
    _, MAX = _import_chunk_text
    stub = _FakeYandexStub()
    node = _build_node(stub)

    text = ("This is a sentence about robots. " * 220).strip()
    assert len(text) > MAX

    audio = node.synthesize_np(text)

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
    node = _build_node(stub, chunk_max_chars=700, chunk_max_retries=3, chunk_min_chars=50)
    text = ("Retry this phrase. " * 15).strip()
    assert 180 < len(text) < 700

    audio = node.synthesize_np(text)

    assert len(stub.attempts) == 3
    assert stub.attempts[0] == text
    assert all(len(chunk) <= 180 for chunk in stub.attempts[1:])
    assert isinstance(audio, np.ndarray)
    assert audio.dtype == np.float32


def test_chunk_failure_propagates_for_silero_fallback(_import_chunk_text):
    """Если chunk gRPC падает, исключение пробрасывается — caller фолбэчит
    на Silero для всего текста."""
    _, MAX = _import_chunk_text
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
        node.synthesize_np(text)

# ── chunk_text не должен быть копией split_text ────────────────────────────


@pytest.mark.parametrize(
    "text",
    [
        "Он подумал… Потом ушёл… И вернулся… Снова подумал… Ушёл опять…",
        "Раз. Два. Три. Четыре. Пять. Шесть. Семь. Восемь.",
        "Ага! Точно? Ну да! Конечно? Ага! Точно? Ну да! Конечно?",
        "Первый абзац тут.\nВторой абзац тут.\nТретий абзац тут.",
    ],
)
@pytest.mark.parametrize("max_chars", [20, 40, 120])
def test_chunk_text_agrees_with_shared_chunker(
    _import_chunk_text, text: str, max_chars: int
) -> None:
    """``yandex.chunk_text`` (бывший ``TTSNode._chunk_text``) обязан резать так же, как общий ``split_text``.

    Тело ``_chunk_text`` было построчной копией
    ``rob_box_voice.tts_chunking.split_text`` (сходство нормализованного
    исходника 0.91, ~100 строк на копию), при том что ``tts_node`` и так
    импортирует ``split_text``. Единственное, чем копия отличалась по
    поведению, — набор разделителей: у неё ``".!?\n"``, у общего
    чанкера ``".!?…\n"``.

    Из-за этого многоточие не считалось границей предложения, и
    Yandex-путь резал русскую речь с «…» по словам посреди фразы, ломая
    паузу, — тогда как все остальные провайдеры резали по предложениям.
    """
    from rob_box_voice.tts_chunking import split_text

    chunk_text, _ = _import_chunk_text
    assert chunk_text(text, max_chars=max_chars) == split_text(
        text, max_chars=max_chars
    )
