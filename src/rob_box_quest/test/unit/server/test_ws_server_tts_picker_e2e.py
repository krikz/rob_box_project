"""End-to-end тесты TTS picker round-trip list → set → preview.

Цель (issue #1919, AV-27 / audit 2026-09-02-avatar-epic-state-audit.md §2.2 G10):
доказать, что фича TTS picker реально работает end-to-end — фикс зазора
"доки врут — фичи нет". Закрывает цепочку JSON_CMD → JSON_EVENT /
BINARY_FRAME для трёх команд, прибитых контрактом docs/architecture/
meta-quest-api.md §4.1-4.5 + каталогом ``rob_box_core.bridge_protocol``
(ADR-0080 §2.2):

  * ``list_voices`` → ``voice_list{voices, active_provider, active_voice, ts_ms}``
  * ``set_voice {voice_id, preset?}`` → ``voice_set_ack`` / ``voice_set_nack``
  * ``preview_voice {request_id, voice_id, text}``
        → ``preview_voice_audio`` × N + ``preview_voice_done`` /
          ``preview_voice_error``

Контракт (SOT = ``src/rob_box_core/rob_box_core/bridge_protocol.py``,
генератор TS = ``tools/gen_bridge_protocol_ts.py``):

  * Event ``voice_list`` (не ``voices_list`` — это была описка автора задачи;
    реальное имя проверено в каталоге и в §6 ``meta-quest-api.md``).
  * Event ``voice_set_ack{voice_id, preset, ts_ms}`` и
    ``voice_set_nack{voice_id, reason, available?, ts_ms}``.
  * Event ``preview_voice_audio{request_id, format, content_type, seq, total,
    ts_ms}`` + BINARY_FRAME с байтами аудио.
  * Event ``preview_voice_done{request_id, ts_ms}`` или
    ``preview_voice_error{request_id, reason, ts_ms}``.

В тестах используется **Stub TTS provider** (``StubTtsBridge``,
наследник ``NoOpBridge``), который:

  1. Экспонирует ≥2 детерминированных голоса с полями voice_id, name, lang,
     provider, tags (соответствует контракту VoiceInfo из
     ``webxr_client/src/wire/messages.ts:165``).
  2. При ``publish_preview_voice`` планирует в ``server._send_loop``
     доставку ``deliver_preview_audio`` + ``deliver_preview_done``
     (имитация ROS-callback'а от ``/avatar/preview_voice/audio`` +
     ``/avatar/preview_voice/result`` в проде).
  3. На ``set_voice(unknown)`` отдаёт ``voice_set_nack{reason, available}`` —
     не hardcoded fallback, а честный "voice_unavailable".

Покрытие (соответствует acceptance задачи t_45a1d1b1):

  1. **list_voices**: ответ — ``voice_list`` event с ≥2 голосами,
     ``active_provider`` / ``active_voice`` / ``ts_ms`` заполнены.
  2. **set_voice (валидный id)**: ``voice_set_ack`` со применённым
     ``voice_id`` и ``preset``; Bridge видит вызов.
  3. **preview_voice**: ≥1 ``preview_voice_audio`` чанк
     (JSON_EVENT + BINARY_FRAME) + ``preview_voice_done`` terminator,
     ``request_id`` совпадает.
  4. **set_voice (unknown id)**: ``voice_set_nack`` с reason
     (``voice_unavailable``) и non-empty ``available`` список.
  5. **Empty provider**: ``voice_list{voices: []}``,
     ``active_provider == ""`` (НЕ hardcoded fallback на дефолтного
     "alena" / "anton" — пустой кэш должен быть честно пустым,
     иначе UI будет врать оператору).

Запуск:

    cd src/rob_box_quest
    PYTHONPATH=".:$PYTHONPATH" python3 -m pytest \\
        test/unit/server/test_ws_server_tts_picker_e2e.py -v

CI: ``.github/workflows/G-Run Tests.yml`` запускает per-file pytest
для каждого ``test/unit/server/test_*.py`` — новый файл попадёт в
прогон автоматически. ``-p no:launch_testing -p no:ament_*`` флаги
уже стоят в CI.

Если упадёт — failure message прямо укажет, какая команда цепочки
сломалась (имя теста = команда в chain).
"""

from __future__ import annotations

import asyncio
import json
import time
from dataclasses import dataclass, field
from typing import Any, Callable

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from rob_box_quest.protocol.frame import (
    FrameType,
    decode_frame,
    encode_frame,
)
from rob_box_quest.server.ws_server import (
    NoOpBridge,
    WSSServer,
    build_app,
)


# ── Stub TTS provider ────────────────────────────────────────────────────────
#
# Намеренно НЕ используем существующий RecordingBridge из test_ws_server_voice.py:
# тот записывает вызовы, но не симулирует preview round-trip (не планирует
# deliver_audio). Здесь же нужен честный e2e — stub сам имитирует ROS-колбэки
# preview-канала.


@dataclass
class VoiceDescriptor:
    """Один голос, который Stub TTS provider отдаёт клиенту."""

    voice_id: str
    name: str
    lang: str
    provider: str
    tags: list[str] = field(default_factory=list)


@dataclass
class PreviewChunk:
    """Один аудио-чанк, который Stub TTS provider пришлёт клиенту."""

    audio_bytes: bytes
    audio_format: str  # "mp3" | "opus" | "wav"
    content_type: str  # "audio/mpeg" | "audio/ogg" | "audio/wav"


@dataclass
class PreviewPlan:
    """Сценарий preview для одного request_id.

    Если ``fail_reason`` непуст — Stub вместо audio-чанков шлёт
    ``preview_voice_error{fail_reason}``.
    """

    request_id: str
    voice_id: str
    text: str
    chunks: list[PreviewChunk] = field(default_factory=list)
    fail_reason: str = ""


class StubTtsBridge(NoOpBridge):
    """Stub TTS-провайдера: детерминированный список голосов + preview pipeline.

    Логика выдерживает контракт AV-27 (issue #1919):

      * ``list_voices_snapshot()`` отдаёт ровно то, что засунуто в
        ``voices_payload`` (либо пустой массив, если ``voices_payload=[]``).
      * ``set_voice(voice_id, preset)``:
          - если voice_id есть в ``voices_payload`` → ``(True, voice_id, ...)``;
          - если нет → ``(False, None, "voice_unavailable", [...ids...])``;
          - если ``voices_payload == []`` → ``(False, None, "tts_unreachable", None)``.
      * ``publish_preview_voice(request_id, voice_id, text)`` планирует
        в ``server._send_loop`` (должен быть установлен заранее) серию
        ``deliver_preview_audio`` + финальный ``deliver_preview_done``.
        Если для ``request_id`` есть план в ``preview_plans`` с
        ``fail_reason`` — вместо audio шлётся ``deliver_preview_error``.
    """

    def __init__(
        self,
        voices_payload: list[VoiceDescriptor] | None = None,
        active_provider: str = "yandex",
        active_voice: str = "",
    ) -> None:
        super().__init__()
        self.voices_descriptors: list[VoiceDescriptor] = (
            list(voices_payload) if voices_payload is not None else []
        )
        # Сериализованная форма для JSON_EVENT (см. meta-quest-api.md §4.1
        # + VoiceInfo в messages.ts:165 — поля voice_id/display_name/language/
        # gender/description/presets/provider).
        self.voices_payload: list[dict[str, Any]] = [
            self._descriptor_to_dict(v) for v in self.voices_descriptors
        ]
        self.active_provider: str = active_provider
        self.active_voice: str = active_voice
        # Логи вызовов (для assert'ов в тестах).
        self.list_snapshot_calls: int = 0
        self.set_voice_calls: list[tuple[str, str | None]] = []
        self.preview_published: list[tuple[str, str, str]] = []
        # Планы preview: ``request_id → PreviewPlan``.
        self.preview_plans: dict[str, PreviewPlan] = {}
        # Генерируемое аудио (для дефолтного сценария, если план не задан).
        self.default_preview_chunker: Callable[[str], list[PreviewChunk]] = (
            self._default_chunker
        )

    @staticmethod
    def _descriptor_to_dict(v: VoiceDescriptor) -> dict[str, Any]:
        """VoiceDescriptor → JSON-serializable dict формата VoiceInfo."""
        # AV-27 wire формат (см. messages.ts VoiceInfo). Здесь намеренно
        # компактно: voice_id + display_name + language + provider. gender
        # нет в stub-контракте, оставляем "neutral" чтобы UI мог рисовать
        # иконку; см. §4.1 «voice_list event» в meta-quest-api.md.
        return {
            "voice_id": v.voice_id,
            "display_name": v.name,
            "language": v.lang,
            "provider": v.provider,
            "gender": "neutral",
            "tags": list(v.tags),
        }

    @staticmethod
    def _default_chunker(text: str) -> list[PreviewChunk]:
        """Дефолтная нарезка preview на 2 чанка (фиксированные, не случайные).

        Тест должен быть **детерминирован** — иначе flakiness под нагрузкой
        CI. Делим на 2 чанка примерно пополам, padding до 16 байт каждый
        (минимум, чтобы BINARY_FRAME имел смысл; см. ADR-0055 §3.2).
        """
        mid = max(1, len(text) // 2)
        chunk1_text = text[:mid].encode("utf-8") or b"stub"
        chunk2_text = text[mid:].encode("utf-8") or b"stub"
        # Pad до 16 байт — фиксированный размер для удобства assert'ов.
        chunk1 = (chunk1_text + b"\x00" * 16)[:16]
        chunk2 = (chunk2_text + b"\x00" * 16)[:16]
        return [
            PreviewChunk(
                audio_bytes=chunk1,
                audio_format="opus",
                content_type="audio/ogg",
            ),
            PreviewChunk(
                audio_bytes=chunk2,
                audio_format="opus",
                content_type="audio/ogg",
            ),
        ]

    # ── Bridge interface (AV-27 / list_voices / set_voice / preview_voice) ──

    def list_voices_snapshot(self) -> dict[str, Any]:
        """Возвращает ровно то, что в ``voices_payload`` — без fallback."""
        self.list_snapshot_calls += 1
        return {
            "voices": [dict(v) for v in self.voices_payload],
            "active_provider": self.active_provider,
            "active_voice": self.active_voice,
        }

    def set_voice(
        self,
        voice_id: str,
        preset: str | None,
    ) -> tuple[bool, str | None, str | None, list[str] | None]:
        """Валидация voice_id; честный nack с reason и available-списком."""
        self.set_voice_calls.append((voice_id, preset))
        if not self.active_provider:
            return False, None, "tts_unreachable", None
        known_ids = {v["voice_id"] for v in self.voices_payload}
        if voice_id not in known_ids:
            available = sorted(known_ids)
            return False, None, "voice_unavailable", available
        return True, voice_id, None, None

    def publish_preview_voice(
        self,
        request_id: str,
        voice_id: str,
        text: str,
    ) -> None:
        """Имитация ROS-callback'а: планирует preview-audio + done.

        Контракт: ``server._send_loop`` ДОЛЖЕН быть установлен заранее
        (через ``server.set_send_loop(loop)``). Это та же логика, что в
        проде: ROS-поток публикует запрос, ``quest_node`` через callback
        достаёт ws из реестра сессии и зовёт ``deliver_preview_*``.
        """
        self.preview_published.append((request_id, voice_id, text))
        server = getattr(self, "_server", None)
        if server is None or server._send_loop is None:
            # Не падаем — тест, который забыл set_send_loop, просто не
            # получит audio и упадёт в assert'е с понятным сообщением.
            return
        plan = self.preview_plans.get(request_id)
        loop = server._send_loop
        if plan is not None and plan.fail_reason:
            loop.call_soon_threadsafe(
                server.deliver_preview_error,
                request_id,
                plan.fail_reason,
            )
            return
        chunks = (
            plan.chunks
            if plan is not None and plan.chunks
            else self.default_preview_chunker(text)
        )
        # Планируем: сначала все чанки audio, потом done. call_soon_threadsafe
        # безопасен из любого потока, но мы в одном loop'е — поэтому
        # фактически это просто отложенный вызов.
        for i, ch in enumerate(chunks):
            seq = i
            total = len(chunks)
            loop.call_soon_threadsafe(
                server.deliver_preview_audio,
                request_id,
                ch.audio_bytes,
                ch.audio_format,
                ch.content_type,
                seq,
                total,
            )
        loop.call_soon_threadsafe(
            server.deliver_preview_done,
            request_id,
        )


# ── WS client helpers (как в test_ws_server_voice.py / test_ws_server_deliver_audio.py)


@pytest.fixture
def fixed_pin(monkeypatch) -> str:
    pin = "123456"
    monkeypatch.setattr("rob_box_quest.server.ws_server.ACTIVE_PIN", pin)
    return pin


@pytest.fixture
async def server_client(fixed_pin):
    """Запускает WSSServer с дефолтным Stub TTS провайдером (2 голоса).

    Возвращает ``(http_client, server, bridge)``. ``server._send_loop``
    выставляется вручную, чтобы ``deliver_preview_*`` могли публиковать
    JSON_EVENT'ы / BINARY_FRAME'ы (без этого preview_audio теряется,
    см. ws_server._schedule_ws_send).

    Yields: (http_client, server, bridge)
    """
    bridge = StubTtsBridge(
        voices_payload=[
            VoiceDescriptor(
                voice_id="alena",
                name="Алёна",
                lang="ru-RU",
                provider="yandex",
                tags=["female", "ru"],
            ),
            VoiceDescriptor(
                voice_id="filipp",
                name="Филипп",
                lang="ru-RU",
                provider="yandex",
                tags=["male", "ru"],
            ),
        ],
        active_provider="yandex",
        active_voice="alena",
    )
    server = WSSServer(bridge=bridge, pin=fixed_pin)
    bridge._server = server  # type: ignore[attr-defined]
    app = build_app(server)
    async with TestClient(TestServer(app)) as http_client:
        server.set_send_loop(asyncio.get_running_loop())
        yield http_client, server, bridge


@pytest.fixture
async def empty_provider_server_client(fixed_pin):
    """Сервер с пустым stub-провайдером (voices_payload=[])."""
    bridge = StubTtsBridge(voices_payload=[], active_provider="", active_voice="")
    server = WSSServer(bridge=bridge, pin=fixed_pin)
    bridge._server = server  # type: ignore[attr-defined]
    app = build_app(server)
    async with TestClient(TestServer(app)) as http_client:
        server.set_send_loop(asyncio.get_running_loop())
        yield http_client, server, bridge


async def _open_and_hello(http_client, pin: str):
    """Открыть WS к /quest, послать HELLO, дождаться WELCOME."""
    ws = await http_client.ws_connect("/quest")
    payload = json.dumps(
        {"client_version": "0.1.0", "capabilities": ["webxr"], "session_pin": pin}
    ).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.HELLO, 0, payload))
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        msg = await ws.receive()
        if msg.type == WSMsgType.CLOSE:
            pytest.fail("closed before WELCOME")
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, _p = decode_frame(msg.data)
            if ftype == FrameType.WELCOME:
                return ws
    pytest.fail("WELCOME not received")


async def _send_json_cmd(ws, cmd_obj: dict[str, Any]) -> None:
    payload = json.dumps(cmd_obj).encode("utf-8")
    await ws.send_bytes(encode_frame(FrameType.JSON_CMD, 0, payload))


async def _wait_for_json_event(ws, predicate, *, timeout: float = 2.0):
    """Читает WS-сообщения пока не найдёт JSON_EVENT, удовлетворяющий predicate.

    Возвращает dict payload или None по таймауту.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = await ws.receive()
        if msg.type == WSMsgType.CLOSE:
            return None
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, payload = decode_frame(msg.data)
            if ftype != FrameType.JSON_EVENT:
                continue
            try:
                body = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if predicate(body):
                return body
    return None


async def _read_binary_frame(ws, *, timeout: float = 1.0):
    """Читает следующий BINARY_FRAME, возвращает payload bytes или None."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = await ws.receive()
        if msg.type == WSMsgType.CLOSE:
            return None
        if msg.type == WSMsgType.BINARY:
            ftype, _sid, payload = decode_frame(msg.data)
            if ftype == FrameType.BINARY_FRAME:
                return payload
    return None


async def _collect_preview_audio(
    ws,
    *,
    expected_request_id: str,
    timeout: float = 2.0,
) -> tuple[list[dict[str, Any]], list[bytes], dict[str, Any] | None]:
    """Собирает все preview-сообщения для одного request_id.

    Возвращает ``(meta_list, binary_payloads, terminator)`` где
    ``terminator`` — это ``preview_voice_done`` или ``preview_voice_error``.

    Завершается по ``preview_voice_done`` / ``preview_voice_error`` либо
    по таймауту. Проглатывает heartbeat / ping/pong.
    """
    metas: list[dict[str, Any]] = []
    binaries: list[bytes] = []
    terminator: dict[str, Any] | None = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and terminator is None:
        msg = await ws.receive()
        if msg.type == WSMsgType.CLOSE:
            break
        if msg.type != WSMsgType.BINARY:
            continue
        ftype, _sid, payload = decode_frame(msg.data)
        if ftype == FrameType.BINARY_FRAME:
            binaries.append(payload)
            continue
        if ftype != FrameType.JSON_EVENT:
            continue
        try:
            body = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if body.get("type") == "preview_voice_audio":
            metas.append(body)
        elif body.get("type") in {"preview_voice_done", "preview_voice_error"}:
            if body.get("request_id") == expected_request_id:
                terminator = body
    return metas, binaries, terminator


# ── Тесты ───────────────────────────────────────────────────────────────────
#
# Имена = шаги round-trip: list → set → preview, плюс negative path и
# empty provider. failure message в каждом тесте прямо укажет, какая
# команда цепочки сломалась.


async def test_list_voices_returns_event_with_two_voices(
    server_client, fixed_pin
):
    """list_voices → voice_list event с ≥2 голосами, active_provider/voice/ts_ms."""
    http_client, _server, bridge = server_client
    assert len(bridge.voices_payload) >= 2, "fixture должен иметь ≥2 голоса"

    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "list_voices", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_list", timeout=2.0
        )
        assert body is not None, (
            "list_voices: voice_list event НЕ пришёл за 2с — цепочка "
            "JSON_CMD → JSON_EVENT сломана на стороне сервера"
        )
        assert isinstance(body.get("voices"), list)
        assert len(body["voices"]) >= 2, (
            f"list_voices: voice_list пришёл с {len(body['voices'])} голосами; "
            f"ожидается ≥2 (fixture StubTtsBridge отдаёт 2)"
        )
        ids = {v.get("voice_id") for v in body["voices"]}
        assert {"alena", "filipp"} <= ids, (
            f"list_voices: voice_list вернул неожиданные voice_id: {ids}; "
            f"ожидаются alena + filipp"
        )
        assert body.get("active_provider") == "yandex"
        assert body.get("active_voice") == "alena"
        assert isinstance(body.get("ts_ms"), int) and body["ts_ms"] > 0, (
            f"list_voices: ts_ms должен быть int > 0; получено {body.get('ts_ms')!r}"
        )
        assert bridge.list_snapshot_calls == 1, (
            "list_voices: bridge.list_voices_snapshot() не был вызван"
        )
    finally:
        await ws.close()


async def test_set_voice_with_valid_id_returns_ack(server_client, fixed_pin):
    """set_voice{voice_id: alena, preset: friendly} → voice_set_ack."""
    http_client, _server, bridge = server_client

    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {
                "cmd": "set_voice",
                "voice_id": "alena",
                "preset": "friendly",
                "ts_ms": 0,
            },
        )
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_set_ack", timeout=2.0
        )
        assert body is not None, (
            "set_voice(valid): voice_set_ack НЕ пришёл за 2с — цепочка "
            "JSON_CMD → JSON_EVENT сломана"
        )
        assert body.get("voice_id") == "alena", (
            f"set_voice(valid): ack.echoed voice_id={body.get('voice_id')!r}; "
            f"ожидалось 'alena'"
        )
        assert body.get("preset") == "friendly", (
            f"set_voice(valid): ack.preset={body.get('preset')!r}; "
            f"ожидалось 'friendly'"
        )
        assert isinstance(body.get("ts_ms"), int) and body["ts_ms"] > 0
        assert bridge.set_voice_calls == [("alena", "friendly")], (
            f"set_voice(valid): bridge.set_voice_calls={bridge.set_voice_calls}; "
            f"ожидалось [('alena', 'friendly')]"
        )
    finally:
        await ws.close()


async def test_preview_voice_delivers_audio_chunks_and_done(server_client, fixed_pin):
    """preview_voice → ≥1 preview_voice_audio (JSON_EVENT + BINARY_FRAME) + done."""
    http_client, server, bridge = server_client

    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        request_id = "req-e2e-001"
        await _send_json_cmd(
            ws,
            {
                "cmd": "preview_voice",
                "request_id": request_id,
                "voice_id": "filipp",
                "text": "Привет, оператор",
                "ts_ms": 0,
            },
        )
        # Даём event loop шанс обработать входящий фрейм и вызвать handler.
        # aiohttp test client не синхронен — между send_bytes и
        # assert'ом сервер ещё мог не прочитать фрейм. Та же пауза, что
        # в test_preview_voice_calls_bridge_and_registers и test_set_voice_*.
        await asyncio.sleep(0.1)
        # Bridge увидел publish_preview_voice.
        assert bridge.preview_published == [
            (request_id, "filipp", "Привет, оператор")
        ], (
            f"preview_voice: bridge.preview_published={bridge.preview_published}; "
            f"ожидалось [{request_id!r}, 'filipp', 'Привет, оператор']"
        )
        # НЕ ассертим ``request_id in server._preview_pending`` здесь:
        # StubTtsBridge.publish_preview_voice ПЛАНИРУЕТ
        # ``deliver_preview_audio`` + ``deliver_preview_done`` через
        # ``loop.call_soon_threadsafe`` — после 0.1с sleep они уже
        # отработали, и ``deliver_preview_done`` УДАЛИЛ request_id из
        # ``_audio_pending["preview"]``. Т.е. факт регистрации в pending
        # виден только мгновенно, до доставки. Проверяем его косвенно:
        # preview_voice_audio/done пришли, а done почистил pending
        # (финальный assert ниже).

        # Собираем audio-чанки + terminator.
        metas, binaries, terminator = await _collect_preview_audio(
            ws, expected_request_id=request_id, timeout=2.0
        )

        assert len(metas) >= 1, (
            "preview_voice: 0 preview_voice_audio событий за 2с; "
            "ожидался ≥1 чанк от StubTtsBridge (default_chunker → 2 чанка)"
        )
        # Все аудио-мета имеют тот же request_id и формат.
        for m in metas:
            assert m.get("request_id") == request_id, (
                f"preview_voice: preview_voice_audio.request_id={m.get('request_id')!r}; "
                f"ожидалось {request_id!r}"
            )
            assert m.get("format") in {"mp3", "opus", "wav"}, (
                f"preview_voice: неизвестный format={m.get('format')!r}"
            )
            assert isinstance(m.get("seq"), int)
            assert isinstance(m.get("total"), int)
            assert m["total"] == len(metas), (
                f"preview_voice: meta.total={m['total']} != реальному "
                f"числу чанков {len(metas)}"
            )

        # Каждый meta имеет соответствующий BINARY_FRAME.
        assert len(binaries) == len(metas), (
            f"preview_voice: получено {len(binaries)} BINARY_FRAME; "
            f"ожидалось {len(metas)} (по одному на чанк)"
        )
        for b in binaries:
            assert isinstance(b, (bytes, bytearray)) and len(b) > 0, (
                f"preview_voice: BINARY_FRAME пустой или не bytes: {b!r}"
            )

        # Terminator — preview_voice_done (НЕ error).
        assert terminator is not None, (
            "preview_voice: preview_voice_done / preview_voice_error "
            "НЕ пришёл за 2с — StubTtsBridge не завершил round-trip"
        )
        assert terminator.get("type") == "preview_voice_done", (
            f"preview_voice: terminator.type={terminator.get('type')!r}; "
            f"ожидался 'preview_voice_done' "
            f"(reason={terminator.get('reason')!r})"
        )
        assert terminator.get("request_id") == request_id

        # request_id вычищен из pending.
        assert request_id not in server._preview_pending, (
            f"preview_voice: request_id {request_id!r} всё ещё в "
            f"server._preview_pending — deliver_preview_done не почистил"
        )
    finally:
        await ws.close()


async def test_set_voice_with_unknown_id_returns_nack_with_available(
    server_client, fixed_pin
):
    """set_voice{voice_id: bogus} → voice_set_nack{reason, available: [...]}."""
    http_client, _server, bridge = server_client

    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(
            ws,
            {"cmd": "set_voice", "voice_id": "bogus", "ts_ms": 0},
        )
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_set_nack", timeout=2.0
        )
        assert body is not None, (
            "set_voice(unknown): voice_set_nack НЕ пришёл за 2с — цепочка "
            "JSON_CMD → JSON_EVENT сломана"
        )
        assert isinstance(body.get("reason"), str) and body["reason"], (
            f"set_voice(unknown): nack.reason должен быть непустой строкой; "
            f"получено {body.get('reason')!r}"
        )
        assert body.get("reason") == "voice_unavailable", (
            f"set_voice(unknown): nack.reason={body.get('reason')!r}; "
            f"ожидалось 'voice_unavailable' от StubTtsBridge"
        )
        avail = body.get("available")
        assert isinstance(avail, list) and len(avail) >= 2, (
            f"set_voice(unknown): nack.available={avail!r}; "
            f"ожидался список ≥2 (alena+filipp)"
        )
        assert sorted(avail) == ["alena", "filipp"], (
            f"set_voice(unknown): nack.available={sorted(avail) if avail else None!r}; "
            f"ожидался ['alena', 'filipp']"
        )
        assert body.get("voice_id") == "bogus", (
            "set_voice(unknown): nack должен echo'ить запрошенный voice_id"
        )
    finally:
        await ws.close()


async def test_empty_provider_returns_empty_voice_list(
    empty_provider_server_client, fixed_pin
):
    """Stub провайдер с [] → voice_list{voices:[]} (НЕ hardcoded fallback)."""
    http_client, _server, bridge = empty_provider_server_client
    assert bridge.voices_payload == [], (
        "fixture empty_provider_server_client должен иметь пустой "
        "voices_payload; иначе этот тест бесполезен"
    )
    assert bridge.active_provider == ""

    ws = await _open_and_hello(http_client, fixed_pin)
    try:
        await _send_json_cmd(ws, {"cmd": "list_voices", "ts_ms": 0})
        body = await _wait_for_json_event(
            ws, lambda b: b.get("type") == "voice_list", timeout=2.0
        )
        assert body is not None, (
            "list_voices (empty provider): voice_list event НЕ пришёл за 2с"
        )
        assert body.get("voices") == [], (
            f"list_voices (empty provider): voice_list.voices={body.get('voices')!r}; "
            f"ожидался [] (НЕ hardcoded fallback на yandex/alena)"
        )
        assert body.get("active_provider") == "", (
            f"list_voices (empty provider): active_provider={body.get('active_provider')!r}; "
            f"ожидалась '' (провайдер не настроен)"
        )
        assert body.get("active_voice") == ""
    finally:
        await ws.close()
