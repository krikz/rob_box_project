# usage: synth_batch.py spec.json outdir ; spec: [{id,voice,pitch,speed,text}] ; существующие пропускает
# Запрос повторяет synth_minimax из .github/workflows/scripts/e2e_voice_test.sh
import json, os, sys, time, requests

spec = json.load(open(sys.argv[1], encoding="utf-8"))
out = sys.argv[2]
os.makedirs(out, exist_ok=True)
key = os.environ["MINIMAX_API_KEY"]
base = os.environ.get("MINIMAX_TTS_BASE_URL", "https://api.minimax.io").rstrip("/")
calls = 0
for s in spec:
    fn = os.path.join(out, s["id"] + ".wav")
    if os.path.exists(fn) and os.path.getsize(fn) > 44:
        continue
    payload = {
        "model": os.environ.get("MINIMAX_TTS_MODEL", "speech-02-hd"),
        "text": s["text"],
        "stream": False,
        "voice_setting": {
            "voice_id": s["voice"],
            "speed": round(float(s["speed"]) * float(os.environ.get("YANDEX_SPEED", "1.0")), 4),
            "pitch": int(s["pitch"]),
        },
        "audio_setting": {"sample_rate": 32000, "bitrate": 128000, "format": "wav", "channel": 1},
    }
    for attempt in range(3):
        calls += 1
        try:
            r = requests.post(
                base + "/v1/t2a_v2",
                headers={"Authorization": "Bearer " + key, "Content-Type": "application/json"},
                json=payload, timeout=60)
            body = r.json()
            br = body.get("base_resp") or {}
            if r.status_code < 400 and int(br.get("status_code", 0) or 0) == 0 and (body.get("data") or {}).get("audio"):
                open(fn, "wb").write(bytes.fromhex(body["data"]["audio"]))
                print("ok", s["id"], flush=True)
                break
            print("FAIL", s["id"], r.status_code, br, flush=True)
            if r.status_code < 500:
                break
        except Exception as e:
            print("EXC", s["id"], e, flush=True)
        time.sleep(2)
print("API_CALLS", calls, flush=True)
