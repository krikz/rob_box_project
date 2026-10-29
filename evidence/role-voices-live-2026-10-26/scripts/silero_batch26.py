# на katana: python3 silero_batch26.py spec.json : синтез Silero v4 как synth_silero харнесса + EQ харнесса -> ~/vp26/eq/<id>.wav
# spec: [{id, provider:"silero", voice, text}] ; существующие пропускает. Silero бесплатный (локально).
import sys, os, json, wave, subprocess
import torch, numpy as np

torch.set_grad_enabled(False)
torch.set_num_threads(4)
spec = json.load(open(sys.argv[1], encoding="utf-8"))
home = os.path.expanduser("~")
model = torch.package.PackageImporter(home + "/.cache/rob_box_voice/tts_models/v4_ru.pt").load_pickle("tts_models", "model")
model.to(torch.device("cpu"))
os.makedirs(home + "/vp26/syn_s", exist_ok=True)
os.makedirs(home + "/vp26/eq", exist_ok=True)
n = 0
for s in spec:
    out = "%s/vp26/eq/%s.wav" % (home, s["id"])
    if os.path.exists(out) and os.path.getsize(out) > 44:
        continue
    raw = "%s/vp26/syn_s/%s.wav" % (home, s["id"])
    audio = model.apply_tts(ssml_text='<speak><prosody pitch="medium">' + s["text"] + "</prosody></speak>",
                            speaker=s["voice"], sample_rate=48000, put_accent=True, put_yo=True)
    pcm = (np.clip(audio.numpy(), -1.0, 1.0) * 32767).astype(np.int16)
    with wave.open(raw, "wb") as h:
        h.setnchannels(1); h.setsampwidth(2); h.setframerate(48000); h.writeframes(pcm.tobytes())
    subprocess.run(["ffmpeg", "-nostdin", "-y", "-loglevel", "error", "-i", raw, "-af",
                    "highpass=f=100,volume=3.0,alimiter=limit=0.98,adelay=1500|all=1", "-ac", "1", "-ar", "16000", out], check=True)
    n += 1
    print("ok", s["id"], flush=True)
print("SILERO_DONE new=%d" % n, flush=True)
