"""mkspec.py name:voice:pitch:speed[,..] first last out.json  -> spec для synth_batch.py"""
import json, sys, os

HERE = os.path.dirname(os.path.abspath(__file__))
phr = json.load(open(os.path.join(HERE, "phrases.json"), encoding="utf-8"))
cfgs, a, b, out = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
rep = int(sys.argv[5]) if len(sys.argv) > 5 else 1
sfx = "" if rep == 1 else "r%d" % rep
append = os.path.exists(out) and len(sys.argv) > 6
spec = json.load(open(out, encoding="utf-8")) if append else []
for c in cfgs.split(","):
    name, voice, pitch, speed = c.split(":")
    for i in range(a, b):
        spec.append({"id": "%s_p%02d%s" % (name, i + 1, sfx), "voice": voice, "pitch": int(pitch),
                     "speed": float(speed), "text": phr[i]})
json.dump(spec, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=0)
print(len(spec), "items")
