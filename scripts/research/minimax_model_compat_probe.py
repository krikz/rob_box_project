#!/usr/bin/env python3
"""minimax_model_compat_probe.py — совместимость НАШЕГО кода с моделью MiniMax.

Голый curl про модель говорит мало: робот молчит не от того, что API ответил
200, а от того, как наш провайдер разобрал ответ. Проба строит провайдер так
же, как прод (``rob_box_harness.providers.catalog.build_provider`` — путь
``assembly.build_agent`` личности и ТАРС), и гоняет через него сценарии:

  a1  complete, thinking выключен (как голосовой ход);
  a2  complete, thinking включён (поле ``thinking`` не шлём — DJ/сочинение);
  a3  один и тот же вопрос с выключенным и включённым thinking — цифры;
  b1  stream, thinking выключен: когда пришёл первый чанк, не сработал ли
      first-chunk-гуард (#2718/#3662), что в сыром потоке;
  b2  stream, thinking включён, тяжёлый запрос (гуард на reasoning-чанках);
  c1  complete + тулы (set_voice, speak_text) и второй вызов с результатом тула;
  c2  stream + те же тулы;
  c3  tool_choice: auto / required (DJ-путь шлёт ``required``, ADR-0143);
  d   картинка на вход (``image_input``);
  e   рассуждение не попадает в речь (strip/markup_recovery);
  r   повторы (PROBE_REPS, по умолчанию 5 реплик): разброс задержек, доля
      настоящих tool_calls, срабатывания first-chunk-гуарда, ``finish_reason=length``.

Запуск ВНУТРИ образа voice-assistant (там пакеты ``rob_box_*`` и ``openai``),
ключ только из окружения — скрипт его не печатает и не пишет::

    docker run --rm --memory 512m --env-file docker/sound/.env.secrets \\
        -v /tmp/probe.py:/probe.py:ro --entrypoint bash <image> \\
        -c 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; \\
            python3 /probe.py'

``PROBE_MODEL`` — модель (по умолчанию ``DEFAULT_MODEL`` из кода — сейчас M3;
для Flash: ``PROBE_MODEL=MiniMax-M3.1-Flash``),
``PROBE_ONLY`` — список случаев через запятую (``a1,b1``).
Вывод — строки ``PROBE[<случай>] {json}``: их можно вставлять в карточку как
есть.
"""

from __future__ import annotations

import asyncio
import io
import json
import os
import struct
import sys
import time
import traceback
import zlib
from typing import Any
from unittest.mock import MagicMock

KEY = os.environ.get("MINIMAX_API_KEY", "")
ONLY = {c for c in os.environ.get("PROBE_ONLY", "").split(",") if c}

PROMPT_PATHS = (
    "/ws/install/rob_box_voice/share/rob_box_voice/prompts/master_prompt_compact.txt",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src",
                 "rob_box_voice", "prompts", "master_prompt_compact.txt"),
)
SYSTEM_FALLBACK = "Ты — робот Робби. Отвечай коротко, по-русски."

#: Слот на ход, как в проде: personality minimax -> temperature 0.7, max_tokens 3000.
PROD_TEMPERATURE = 0.7
PROD_MAX_TOKENS = 3000

HEAVY_PROMPT = (
    "Робот, составь мне DJ-сет из пяти треков от спокойного к энергичному: "
    "для каждого назови BPM, тональность и объясни, почему он идёт после предыдущего."
)


def red(value: Any) -> str:
    """Строка для печати без ключа."""
    text = value if isinstance(value, str) else json.dumps(value, ensure_ascii=False, default=str)
    return text.replace(KEY, "***") if KEY else text


def emit(tag: str, **fields: Any) -> None:
    print(f"PROBE[{tag}] {red(fields)}", flush=True)


def load_system_prompt() -> tuple[str, str]:
    for path in PROMPT_PATHS:
        if os.path.exists(path):
            with open(path, encoding="utf-8") as fh:
                return fh.read(), os.path.basename(path)
    return SYSTEM_FALLBACK, "fallback"


# ---------------------------------------------------------------------------
# Запись того, что реально ушло в SDK и пришло из него
# ---------------------------------------------------------------------------


class StreamProxy:
    """Обёртка над потоком SDK: время каждого сырого события и что в дельте."""

    def __init__(self, inner: Any, call: dict[str, Any], t0: float) -> None:
        self._inner = inner
        self._call = call
        self._t0 = t0

    def __aiter__(self) -> "StreamProxy":
        return self

    async def __anext__(self) -> Any:
        event = await self._inner.__anext__()
        choice = event.choices[0] if event.choices else None
        delta = choice.delta if choice else None
        keys = sorted(delta.model_dump(exclude_none=True)) if delta is not None else []
        extra = dict(getattr(delta, "model_extra", None) or {}) if delta is not None else {}
        self._call["events"].append({
            "t": round(time.monotonic() - self._t0, 3),
            "keys": keys,
            "content": bool(getattr(delta, "content", None)),
            "reasoning": bool(extra.get("reasoning_content") or extra.get("reasoning_details")),
            "tool_calls": bool(getattr(delta, "tool_calls", None)),
            "finish": getattr(choice, "finish_reason", None) if choice else None,
            "usage": bool(getattr(event, "usage", None)),
        })
        return event

    async def aclose(self) -> None:
        close = getattr(self._inner, "aclose", None) or getattr(self._inner, "close", None)
        if close is not None:
            await close()


class Recorder:
    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []

    def install(self, provider: Any) -> None:
        completions = provider._inner._client.chat.completions
        original = completions.create
        recorder = self

        async def create(**kwargs: Any) -> Any:
            t0 = time.monotonic()
            call: dict[str, Any] = {
                "request": {k: v for k, v in kwargs.items() if k not in ("messages", "tools")},
                "messages": len(kwargs.get("messages", ())),
                "tools": [t["function"]["name"] for t in kwargs.get("tools", ())],
                "events": [],
            }
            recorder.calls.append(call)
            result = await original(**kwargs)
            call["t_response_headers"] = round(time.monotonic() - t0, 3)
            if kwargs.get("stream"):
                return StreamProxy(result, call, t0)
            return result

        completions.create = create

    def last(self) -> dict[str, Any]:
        return self.calls[-1] if self.calls else {}


def stream_timeline(call: dict[str, Any]) -> dict[str, Any]:
    events = call.get("events", [])

    def first(pred: Any) -> float | None:
        for ev in events:
            if pred(ev):
                return ev["t"]
        return None

    return {
        "raw_events": len(events),
        "t_first_raw_event": events[0]["t"] if events else None,
        "t_first_reasoning_event": first(lambda e: e["reasoning"]),
        "t_first_content_event": first(lambda e: e["content"]),
        "t_first_tool_event": first(lambda e: e["tool_calls"]),
        "t_last_event": events[-1]["t"] if events else None,
        "reasoning_events": sum(1 for e in events if e["reasoning"]),
        "content_events": sum(1 for e in events if e["content"]),
        "delta_keys": sorted({k for e in events for k in e["keys"]}),
    }


# ---------------------------------------------------------------------------
# Сборка провайдера и данных
# ---------------------------------------------------------------------------


def build(model: str) -> tuple[Any, Recorder]:
    from rob_box_harness.providers.catalog import build_provider

    provider = build_provider("minimax", model=model)
    recorder = Recorder()
    recorder.install(provider)
    return provider, recorder


def real_tools() -> tuple[list[dict[str, Any]], str]:
    """set_voice + speak_text из каталога тулов; нет пакета — схемы вручную."""
    try:
        from rob_box_mcp_tools.tools.dialogue import SetVoiceTool, SpeakTextTool

        node = MagicMock()
        return [SetVoiceTool(node).to_openai_tool_format(),
                SpeakTextTool(node).to_openai_tool_format()], "rob_box_mcp_tools"
    except Exception as exc:  # noqa: BLE001 — запасной путь
        specs = [
            ("set_voice", "Установить голос TTS на диалог.",
             {"voice": {"type": "string", "description": "Имя голоса, например alena"}}, ["voice"]),
            ("speak_text", "Произнести текст голосом.",
             {"text": {"type": "string", "description": "Что сказать"}}, ["text"]),
        ]
        tools = [{"type": "function", "function": {
            "name": n, "description": d,
            "parameters": {"type": "object", "properties": p, "required": r,
                           "additionalProperties": False}}} for n, d, p, r in specs]
        return tools, f"inline ({type(exc).__name__})"


def png_bytes(width: int = 160, height: int = 120) -> bytes:
    """PNG без PIL: слева красный блок, справа синий."""
    rows = bytearray()
    for _ in range(height):
        rows.append(0)
        rows += bytes((220, 20, 20)) * (width // 2) + bytes((20, 40, 220)) * (width - width // 2)

    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(bytes(rows))) + chunk(b"IEND", b""))


def make_image() -> tuple[bytes, str, str]:
    try:
        from PIL import Image, ImageDraw

        im = Image.new("RGB", (320, 240), "white")
        draw = ImageDraw.Draw(im)
        draw.ellipse((30, 60, 150, 180), fill=(220, 20, 20))
        draw.rectangle((190, 60, 290, 180), fill=(20, 40, 220))
        buf = io.BytesIO()
        im.save(buf, "JPEG", quality=90)
        return buf.getvalue(), "image/jpeg", "PIL: красный круг слева, синий квадрат справа"
    except Exception:  # noqa: BLE001
        return png_bytes(), "image/png", "PNG: левая половина красная, правая синяя"


def msgs(system: str, user: Any) -> list[Any]:
    from rob_box_llm.provider import LLMMessage

    return [LLMMessage(role="system", content=system), LLMMessage(role="user", content=user)]


def prod_settings(**extra: Any) -> Any:
    from rob_box_llm.provider import LLMSettings

    return LLMSettings(temperature=PROD_TEMPERATURE, max_tokens=PROD_MAX_TOKENS, **extra)


def describe_response(resp: Any) -> dict[str, Any]:
    from rob_box_harness.providers.reasoning import without_reasoning

    raw = resp.raw
    message = raw.choices[0].message if raw is not None and raw.choices else None
    extra = dict(getattr(message, "model_extra", None) or {})
    reasoning = extra.get("reasoning_content")
    reasoning = reasoning if isinstance(reasoning, str) else ""
    raw_usage = raw.usage.model_dump() if raw is not None and getattr(raw, "usage", None) else {}
    from rob_box_harness.core.tool_loop.text_classify import is_tool_call_markup

    cleaned = without_reasoning(resp).content
    return {
        "content": resp.content,
        "content_cleaned_by_without_reasoning": cleaned,
        "markup_detected_by_text_classify": is_tool_call_markup(cleaned),
        "finish_reason": resp.finish_reason,
        "usage": dict(resp.usage),
        "raw_usage": raw_usage,
        "message_extra_keys": sorted(extra),
        "reasoning_content_len": len(reasoning),
        "reasoning_content_head": reasoning[:100],
        "tool_calls": [(tc.name, dict(tc.arguments), tc.id) for tc in resp.tool_calls],
        "truncated_tool_args": resp.truncated_tool_args,
        "think_tag_in_content": "<think>" in resp.content or "</think>" in resp.content,
        "reasoning_text_in_content": bool(reasoning) and reasoning.strip()[:40] in resp.content,
    }


async def aggregate_stream(provider: Any, messages: Any, **kwargs: Any) -> dict[str, Any]:
    """Тот же сбор потока, что в ``AgentCore._stream_response``."""
    from rob_box_harness.providers.reasoning import without_reasoning
    from rob_box_llm.provider import LLMResponse

    parts: list[str] = []
    calls: list[Any] = []
    finish = usage = None
    truncated = False
    chunks = 0
    empty = 0
    t0 = time.monotonic()
    t_first = None
    stream = provider.stream(messages, **kwargs)
    try:
        async for chunk in stream:
            if t_first is None:
                t_first = round(time.monotonic() - t0, 3)
            chunks += 1
            if chunk.content_delta:
                parts.append(chunk.content_delta)
            elif chunk.tool_call_delta is None and not chunk.finish_reason:
                empty += 1
            if chunk.tool_call_delta is not None:
                calls.append(chunk.tool_call_delta)
            finish = chunk.finish_reason or finish
            usage = chunk.usage or usage
            truncated = truncated or chunk.truncated_tool_args
    finally:
        close = getattr(stream, "aclose", None)
        if close is not None:
            await close()
    resp = without_reasoning(LLMResponse(content="".join(parts), tool_calls=tuple(calls),
                                         finish_reason=finish, usage=usage or {},
                                         truncated_tool_args=truncated))
    return {
        "t_first_llmchunk": t_first,
        "t_total": round(time.monotonic() - t0, 3),
        "llm_chunks": chunks,
        "empty_chunks_reasoning_liveness": empty,
        "content": resp.content,
        "finish_reason": resp.finish_reason,
        "usage": dict(resp.usage),
        "tool_calls": [(tc.name, dict(tc.arguments), tc.id) for tc in resp.tool_calls],
        "truncated_tool_args": resp.truncated_tool_args,
        "think_tag_in_content": "<think>" in resp.content or "</think>" in resp.content,
    }


def error_fields(exc: BaseException) -> dict[str, Any]:
    cause = exc.__cause__
    return {
        "error": type(exc).__name__,
        "message": str(exc)[:500],
        "http_status": getattr(cause, "status_code", None),
        "http_body": getattr(cause, "body", None),
        "traceback_tail": traceback.format_exc().strip().splitlines()[-3:],
    }


# ---------------------------------------------------------------------------
# Случаи
# ---------------------------------------------------------------------------


async def case_a1(model: str, system: str) -> None:
    provider, rec = build(model)
    try:
        t0 = time.monotonic()
        resp = await provider.complete(msgs(system, "Робот, привет! Как тебя зовут?"), settings=prod_settings())
        emit("a1_complete_thinking_disabled", seconds=round(time.monotonic() - t0, 3),
             request=rec.last()["request"], **describe_response(resp))
    finally:
        await provider.aclose()


async def case_a2(model: str, system: str) -> None:
    from rob_box_harness.providers.reasoning import TURN_REASONING

    provider, rec = build(model)
    token = TURN_REASONING.set(True)
    try:
        t0 = time.monotonic()
        resp = await provider.complete(msgs(system, "Робот, привет! Как тебя зовут?"), settings=prod_settings())
        request = rec.last()["request"]
        emit("a2_complete_thinking_on", seconds=round(time.monotonic() - t0, 3), request=request,
             thinking_field_sent="thinking" in (request.get("extra_body") or {}), **describe_response(resp))
    finally:
        TURN_REASONING.reset(token)
        await provider.aclose()


async def case_a3(model: str, system: str) -> None:
    from rob_box_harness.providers.reasoning import TURN_REASONING

    question = "Сколько будет 17 умножить на 23, минус 48? Ответь только числом."
    for label, thinking in (("disabled", False), ("omitted", True)):
        provider, rec = build(model)
        token = TURN_REASONING.set(thinking)
        try:
            t0 = time.monotonic()
            resp = await provider.complete(msgs(system, question), settings=prod_settings())
            info = describe_response(resp)
            emit(f"a3_arith_thinking_{label}", seconds=round(time.monotonic() - t0, 3),
                 thinking_in_request=(rec.last()["request"].get("extra_body") or {}).get("thinking", "<absent>"),
                 content=info["content"], reasoning_content_len=info["reasoning_content_len"],
                 usage=info["usage"], raw_usage=info["raw_usage"], finish_reason=info["finish_reason"])
        finally:
            TURN_REASONING.reset(token)
            await provider.aclose()


async def case_b(tag: str, model: str, system: str, user: str, thinking: bool) -> None:
    from rob_box_harness.providers.reasoning import TURN_REASONING

    provider, rec = build(model)
    token = TURN_REASONING.set(thinking)
    try:
        result = await aggregate_stream(provider, msgs(system, user), settings=prod_settings())
        emit(tag, guard_s=provider._first_chunk_timeout_s, thinking_on=thinking,
             request=rec.last()["request"], timeline=stream_timeline(rec.last()), **result)
    except Exception as exc:  # noqa: BLE001
        emit(tag, guard_s=provider._first_chunk_timeout_s, thinking_on=thinking,
             timeline=stream_timeline(rec.last()), **error_fields(exc))
    finally:
        TURN_REASONING.reset(token)
        await provider.aclose()


TOOL_USER = "Робот, дальше говори голосом Алёна. Скажи: привет, товарищ Шифу!"


def tool_followup(messages: list[Any], resp: Any) -> list[Any]:
    """Вторая итерация агент-цикла: ответ ассистента с вызовами + результаты тулов."""
    from rob_box_llm.provider import LLMMessage, ToolResult

    out = list(messages)
    out.append(LLMMessage(role="assistant", content=resp.content, tool_calls=resp.tool_calls))
    for call in resp.tool_calls:
        result = ToolResult(tool_call_id=call.id, content='{"success": true}')
        out.append(LLMMessage(role="tool", content=result.content, tool_call_id=call.id, tool_result=result))
    return out


async def case_c1(model: str, system: str) -> None:
    tools, source = real_tools()
    provider, rec = build(model)
    try:
        messages = msgs(system, TOOL_USER)
        t0 = time.monotonic()
        resp = await provider.complete(messages, tools=tools, settings=prod_settings())
        emit("c1_complete_tools_turn1", tools_source=source, seconds=round(time.monotonic() - t0, 3),
             **describe_response(resp))
        if resp.tool_calls:
            t1 = time.monotonic()
            follow = await provider.complete(tool_followup(messages, resp), tools=tools, settings=prod_settings())
            emit("c1_complete_tools_turn2_after_tool_result", seconds=round(time.monotonic() - t1, 3),
                 **describe_response(follow))
    finally:
        await provider.aclose()


async def case_c2(model: str, system: str) -> None:
    tools, source = real_tools()
    provider, rec = build(model)
    try:
        result = await aggregate_stream(provider, msgs(system, TOOL_USER), tools=tools, settings=prod_settings())
        emit("c2_stream_tools", tools_source=source, timeline=stream_timeline(rec.last()), **result)
    except Exception as exc:  # noqa: BLE001
        emit("c2_stream_tools", timeline=stream_timeline(rec.last()), **error_fields(exc))
    finally:
        await provider.aclose()


async def case_c3(model: str, system: str) -> None:
    tools, _ = real_tools()
    for choice in ("auto", "required", "none"):
        provider, rec = build(model)
        try:
            t0 = time.monotonic()
            resp = await provider.complete(msgs(system, "Привет, как дела?"), tools=tools,
                                           settings=prod_settings(tool_choice=choice))
            info = describe_response(resp)
            emit(f"c3_tool_choice_{choice}", seconds=round(time.monotonic() - t0, 3),
                 sent=rec.last()["request"].get("tool_choice"), tool_calls=info["tool_calls"],
                 content=info["content"][:200], finish_reason=info["finish_reason"])
        except Exception as exc:  # noqa: BLE001
            emit(f"c3_tool_choice_{choice}", sent=choice, **error_fields(exc))
        finally:
            await provider.aclose()


IMAGE_SYSTEM = "Ты видишь камеру робота. Ответь по-русски одним предложением, без вызова инструментов."


async def case_d(model: str, system: str) -> None:
    """Картинка на вход: 3 попытки с простым системным промптом (мастер-промпт зовёт к тулам, а не к описанию)."""
    from rob_box_llm.provider import ImagePart, TextPart

    data, media_type, what = make_image()
    provider, rec = build(model)
    try:
        caps = provider.capabilities_for(model)
        emit("d_image_capability_in_code", model=model, image_input=caps.image_input)
        content = (TextPart("Что на этой картинке? Назови фигуры и их цвета."),
                   ImagePart(source=data, media_type=media_type))
        for attempt in (1, 2, 3):
            t0 = time.monotonic()
            try:
                resp = await provider.complete(msgs(IMAGE_SYSTEM, content), settings=prod_settings())
            except Exception as exc:  # noqa: BLE001
                emit(f"d_image_input_{attempt}", image=what, bytes=len(data), **error_fields(exc))
                continue
            info = describe_response(resp)
            text = info["content"].lower()
            emit(f"d_image_input_{attempt}", image=what, bytes=len(data), seconds=round(time.monotonic() - t0, 3),
                 mentions_red="красн" in text, mentions_blue="син" in text or "голуб" in text, **info)
    finally:
        await provider.aclose()


REPEAT_USERS = (
    "Робот, привет! Как дела?",
    "Робот, который час?",
    "Робот, расскажи короткий анекдот.",
    "Робот, говори голосом Захар.",
    "Робот, спасибо, ты молодец.",
)


def _percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(q * (len(ordered) - 1))))]


async def case_r(model: str, system: str) -> None:
    """Повторы с тулами: stream и complete на разных репликах, thinking как в голосовом ходе."""
    tools, _ = real_tools()
    reps = int(os.environ.get("PROBE_REPS", "5"))
    users = (REPEAT_USERS * ((reps // len(REPEAT_USERS)) + 1))[:reps]
    provider, rec = build(model)
    rows: list[dict[str, Any]] = []
    try:
        for user in users:
            row: dict[str, Any] = {"user": user}
            try:
                streamed = await aggregate_stream(provider, msgs(system, user), tools=tools, settings=prod_settings())
                row.update(stream_first=streamed["t_first_llmchunk"], stream_total=streamed["t_total"],
                           stream_calls=[c[0] for c in streamed["tool_calls"]],
                           stream_finish=streamed["finish_reason"], stream_text=streamed["content"][:60])
            except Exception as exc:  # noqa: BLE001
                row["stream_error"] = f"{type(exc).__name__}: {str(exc)[:120]}"
            try:
                t0 = time.monotonic()
                resp = await provider.complete(msgs(system, user), tools=tools, settings=prod_settings())
                info = describe_response(resp)
                row.update(complete_s=round(time.monotonic() - t0, 3), complete_calls=[c[0] for c in info["tool_calls"]],
                           complete_finish=info["finish_reason"], complete_text=info["content"][:60],
                           reasoning_tokens=(info["raw_usage"].get("completion_tokens_details") or {}).get("reasoning_tokens"),
                           completion_tokens=info["usage"].get("completion_tokens"))
            except Exception as exc:  # noqa: BLE001
                row["complete_error"] = f"{type(exc).__name__}: {str(exc)[:120]}"
            rows.append(row)
        firsts = [r["stream_first"] for r in rows if "stream_first" in r]
        totals = [r["complete_s"] for r in rows if "complete_s" in r]
        emit("r_repeats_rows", rows=rows)
        emit("r_repeats_summary", n=len(rows), stream_first_p50=_percentile(firsts, 0.5),
             stream_first_max=max(firsts) if firsts else None, complete_p50=_percentile(totals, 0.5),
             complete_max=max(totals) if totals else None,
             errors=sum(1 for r in rows if "stream_error" in r or "complete_error" in r),
             stream_without_tool_call=sum(1 for r in rows if "stream_calls" in r and not r["stream_calls"]),
             complete_without_tool_call=sum(1 for r in rows if "complete_calls" in r and not r["complete_calls"]),
             finish_length=sum(1 for r in rows if "length" in (r.get("stream_finish"), r.get("complete_finish"))),
             markup_text=sum(1 for r in rows if "<tool_call>" in (r.get("stream_text", "") + r.get("complete_text", ""))))
    finally:
        await provider.aclose()


async def case_r2(model: str, system: str) -> None:
    """Ход с thinking (DJ/сочинение) с тулами: поле глубины не шлём — дефолт модели."""
    from rob_box_harness.providers.reasoning import TURN_REASONING

    tools, _ = real_tools()
    provider, rec = build(model)
    token = TURN_REASONING.set(True)
    try:
        result = await aggregate_stream(provider, msgs(system, "Робот, расскажи стих голосом Захар."),
                                        tools=tools, settings=prod_settings())
        emit("r2_stream_thinking_on_with_tools", request=rec.last()["request"],
             timeline=stream_timeline(rec.last()), **result)
    except Exception as exc:  # noqa: BLE001
        emit("r2_stream_thinking_on_with_tools", **error_fields(exc))
    finally:
        TURN_REASONING.reset(token)
        await provider.aclose()


def case_e() -> None:
    """Рассуждение не речь: вырезание и разбор разметки вызова (озвучка content удалена, #3698 ч.7d-3)."""
    from rob_box_harness.core.tool_loop.markup_recovery import parse_tool_call_markup
    from rob_box_harness.providers.reasoning import strip_reasoning

    tools, _ = real_tools()
    samples = {
        "think_in_content": "<think>надо поздороваться</think>Привет, Шифу!",
        "clean_content": "Привет, Шифу!",
        "think_cut_by_max_tokens": "<think>длинное рассуждение без конца",
        "markup_after_think": ("<think>хм</think><invoke name=\"set_voice\">"
                               "<parameter name=\"voice\">alena</parameter></invoke>"),
    }
    out = {}
    for name, text in samples.items():
        stripped = strip_reasoning(text)
        out[name] = {
            "stripped": stripped,
            "recovered_calls": [(n, dict(a)) for n, a in parse_tool_call_markup(stripped, tools=tools)],
        }
    emit("e_reasoning_not_speech_synthetic", **out)


CASES = (
    ("a1", lambda m, s: case_a1(m, s)),
    ("a2", lambda m, s: case_a2(m, s)),
    ("a3", lambda m, s: case_a3(m, s)),
    ("b1", lambda m, s: case_b("b1_stream_thinking_disabled", m, s, "Робот, расскажи коротко, что ты умеешь.", False)),
    ("b2", lambda m, s: case_b("b2_stream_thinking_on_heavy", m, s, HEAVY_PROMPT, True)),
    ("c1", lambda m, s: case_c1(m, s)),
    ("c2", lambda m, s: case_c2(m, s)),
    ("c3", lambda m, s: case_c3(m, s)),
    ("d", lambda m, s: case_d(m, s)),
    ("r", lambda m, s: case_r(m, s)),
    ("r2", lambda m, s: case_r2(m, s)),
)


async def main() -> int:
    if not KEY:
        print("MINIMAX_API_KEY не задан в окружении", file=sys.stderr)
        return 2
    from rob_box_harness.providers.catalog import LLM_PROVIDER_REGISTRY

    model = os.environ.get("PROBE_MODEL") or LLM_PROVIDER_REGISTRY["minimax"]["default_model"]
    system, prompt_name = load_system_prompt()
    emit("setup", model=model, default_model_in_code=LLM_PROVIDER_REGISTRY["minimax"]["default_model"],
         system_prompt=prompt_name, system_prompt_chars=len(system), python=sys.version.split()[0],
         only=sorted(ONLY) or "all")
    for name, run in CASES:
        if ONLY and name not in ONLY:
            continue
        try:
            await run(model, system)
        except Exception as exc:  # noqa: BLE001
            emit(f"{name}_FAILED", **error_fields(exc))
    if not ONLY or "e" in ONLY:
        case_e()
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
