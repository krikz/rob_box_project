# на katana: silero_batch.py phrases.json n_phr  -> ~/vp/eq/<S_xx>_pNN.wav (синтез как synth_silero харнесса + его EQ)
import sys, os, json, wave, subprocess
import torch, numpy as np

torch.set_grad_enabled(False)
torch.set_num_threads(4)
phr = json.load(open(sys.argv[1], encoding="utf-8"))
n = int(sys.argv[2])
home = os.path.expanduser("~")
model = torch.package.PackageImporter(home + "/.cache/rob_box_voice/tts_models/v4_ru.pt").load_pickle("tts_models", "model")
model.to(torch.device("cpu"))
V = {"SEU": "eugene", "SAI": "aidar", "SKS": "kseniya", "SXE": "xenia", "SBA": "baya"}
os.makedirs(home + "/vp/syn_s", exist_ok=True)
os.makedirs(home + "/vp/eq", exist_ok=True)
for cfg, voice in V.items():
    for i in range(n):
        raw = "%s/vp/syn_s/%s_p%02d.wav" % (home, cfg, i + 1)
        out = "%s/vp/eq/%s_p%02d.wav" % (home, cfg, i + 1)
        audio = model.apply_tts(ssml_text='<speak><prosody pitch="medium">' + phr[i] + "</prosody></speak>",
                                speaker=voice, sample_rate=48000, put_accent=True, put_yo=True)
        pcm = (np.clip(audio.numpy(), -1.0, 1.0) * 32767).astype(np.int16)
        with wave.open(raw, "wb") as h:
            h.setnchannels(1); h.setsampwidth(2); h.setframerate(48000); h.writeframes(pcm.tobytes())
        subprocess.run(["ffmpeg", "-nostdin", "-y", "-loglevel", "error", "-i", raw, "-af",
                        "highpass=f=100,volume=3.0,alimiter=limit=0.98,adelay=1500|all=1", "-ac", "1", "-ar", "16000", out], check=True)
        print("ok", cfg, i + 1, flush=True)
