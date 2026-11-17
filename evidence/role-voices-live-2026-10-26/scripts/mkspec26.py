"""mkspec26.py out.json first last name:provider:voice:pitch:speed[,..] [--append] [--phr=phrases_nw.json] [--off=20]
-> spec для synth26.py / silero_batch26.py. Фразы [first,last) (индекс с нуля) из phrases.json (с вейком «Робот,») или
из файла --phr (phrases_nw.json - те же фразы без вейка). Номер в id = индекс + 1 + off, так что наборы не пересекаются
по именам файлов: с вейком p01..p20, без вейка p21..p40 (--off=20)."""
import json, sys, os

HERE = os.path.dirname(os.path.abspath(__file__))
opts = {a.split("=")[0]: a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--") and "=" in a}
phr = json.load(open(os.path.join(HERE, "..", opts.get("--phr", "phrases.json")), encoding="utf-8"))
off = int(opts.get("--off", "0"))
args = [a for a in sys.argv[1:] if not a.startswith("--")]
out, a, b, cfgs = args[0], int(args[1]), int(args[2]), args[3]
append = "--append" in sys.argv and os.path.exists(out)
spec = json.load(open(out, encoding="utf-8")) if append else []
for c in cfgs.split(","):
    name, prov, voice, pitch, speed = c.split(":")
    for i in range(a, b):
        spec.append({"id": "%s_p%02d" % (name, i + 1 + off), "provider": prov, "voice": voice, "pitch": int(pitch),
                     "speed": float(speed), "text": phr[i]})
json.dump(spec, open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=0)
print(len(spec), "items")
