# emb.py indir out.json : resemblyzer-эмбеддинг каждого *.raw (int16 16 кГц) — как SpeakerDatabase.embed_audio_ex
import glob, json, os, sys
import numpy as np
from resemblyzer import VoiceEncoder, preprocess_wav

indir, out = sys.argv[1], sys.argv[2]
done = json.load(open(out)) if os.path.exists(out) else {}
enc = VoiceEncoder(device="cpu")
for f in sorted(glob.glob(os.path.join(indir, "*.raw"))):
    k = os.path.basename(f)
    if k in done:
        continue
    pcm = np.frombuffer(open(f, "rb").read(), dtype=np.int16).astype(np.float32) / 32768.0
    dur = len(pcm) / 16000.0
    if dur < 0.5:
        done[k] = None
        continue
    wav = preprocess_wav(pcm, source_sr=16000)
    e = enc.embed_utterance(wav)
    done[k] = {"dur": dur, "voiced": len(wav) / 16000.0, "emb": [float(x) for x in e]}
json.dump(done, open(out, "w"))
print("embedded", sum(1 for v in done.values() if v), "of", len(done))
