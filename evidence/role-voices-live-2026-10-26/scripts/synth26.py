"""synth26.py spec.json outdir : синтез ВНУТРИ сервисного контейнера voice-assistant (ключи YANDEX/MINIMAX берутся
из env контейнера и нигде не печатаются). Запуск: docker exec -i voice-assistant python3 - spec.json outdir < synth26.py

spec: [{id, provider: yandex|minimax, voice, pitch, speed, text}] ; существующие wav пропускает.
Запросы повторяют synth_yandex / synth_minimax из .github/workflows/scripts/e2e_voice_test.sh (те же поля).
Яндекс: три подряд отказа PERMISSION_DENIED / UNAUTHENTICATED / RESOURCE_EXHAUSTED -> стоп (по решению Шифу:
не долбить, сырой ответ в отчёт). Каждая попытка (и ретраи) считается вызовом: итог CALLS yandex=N minimax=M."""
import json, os, sys, time

spec = json.load(open(sys.argv[1], encoding="utf-8"))
out = sys.argv[2]
os.makedirs(out, exist_ok=True)
calls = {"yandex": 0, "minimax": 0}
yfail_streak = 0
stop_yandex = False


def synth_yandex(s, fn):
    import grpc
    from yandex.cloud.ai.tts.v3 import tts_pb2, tts_service_pb2_grpc
    key = os.environ["YANDEX_API_KEY"]
    ch = grpc.secure_channel("tts.api.cloud.yandex.net:443", grpc.ssl_channel_credentials())
    stub = tts_service_pb2_grpc.SynthesizerStub(ch)
    req = tts_pb2.UtteranceSynthesisRequest(
        text=s["text"],
        output_audio_spec=tts_pb2.AudioFormatOptions(
            container_audio=tts_pb2.ContainerAudio(container_audio_type=tts_pb2.ContainerAudio.WAV)),
        hints=[tts_pb2.Hints(voice=s["voice"]), tts_pb2.Hints(speed=float(s.get("speed", 1.0)))],
        loudness_normalization_type=tts_pb2.UtteranceSynthesisRequest.LUFS,
    )
    resp = stub.UtteranceSynthesis(req, metadata=(("authorization", "Api-Key " + key),))
    data = b""
    for r in resp:
        data += r.audio_chunk.data
    if not data:
        raise RuntimeError("EMPTY")
    open(fn, "wb").write(data)


def synth_minimax(s, fn):
    import requests
    key = os.environ["MINIMAX_API_KEY"]
    base = os.environ.get("MINIMAX_TTS_BASE_URL", "https://api.minimax.io").rstrip("/")
    payload = {
        "model": os.environ.get("MINIMAX_TTS_MODEL", "speech-02-hd"),
        "text": s["text"], "stream": False,
        "voice_setting": {"voice_id": s["voice"], "speed": round(float(s.get("speed", 1.0)), 4), "pitch": int(s.get("pitch", 0))},
        "audio_setting": {"sample_rate": 32000, "bitrate": 128000, "format": "wav", "channel": 1},
    }
    r = requests.post(base + "/v1/t2a_v2", headers={"Authorization": "Bearer " + key, "Content-Type": "application/json"},
                      json=payload, timeout=60)
    body = r.json()
    br = body.get("base_resp") or {}
    if r.status_code < 400 and int(br.get("status_code", 0) or 0) == 0 and (body.get("data") or {}).get("audio"):
        open(fn, "wb").write(bytes.fromhex(body["data"]["audio"]))
        return
    raise RuntimeError("MINIMAX http=%s base_resp=%s" % (r.status_code, {k: br.get(k) for k in ("status_code", "status_msg")}))


for s in spec:
    fn = os.path.join(out, s["id"] + ".wav")
    if os.path.exists(fn) and os.path.getsize(fn) > 44:
        continue
    prov = s["provider"]
    if prov == "yandex" and stop_yandex:
        print("SKIP(yandex stopped)", s["id"], flush=True)
        continue
    for attempt in range(2):
        calls[prov] += 1
        try:
            (synth_yandex if prov == "yandex" else synth_minimax)(s, fn)
            print("ok", s["id"], os.path.getsize(fn), flush=True)
            if prov == "yandex":
                yfail_streak = 0
            break
        except Exception as e:
            code = ""
            if hasattr(e, "code") and callable(e.code):
                try:
                    code = e.code().name
                except Exception:
                    pass
            det = ""
            if hasattr(e, "details") and callable(e.details):
                try:
                    det = str(e.details())[:200]
                except Exception:
                    pass
            print("FAIL", s["id"], prov, s["voice"], type(e).__name__, code, det or str(e)[:200], flush=True)
            if prov == "yandex":
                if code in ("PERMISSION_DENIED", "UNAUTHENTICATED", "RESOURCE_EXHAUSTED"):
                    yfail_streak += 1
                    if yfail_streak >= 3:
                        stop_yandex = True
                        print("YANDEX_STOP: 3 подряд", code, flush=True)
                    break
                if code in ("INVALID_ARGUMENT", "NOT_FOUND"):
                    break
            else:
                if "http=4" in str(e):
                    break
            time.sleep(2)
print("CALLS yandex=%d minimax=%d" % (calls["yandex"], calls["minimax"]), flush=True)
