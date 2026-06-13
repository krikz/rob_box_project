import sys, time
sys.path.insert(0, "/ws/install/rob_box_mcp_tools/lib/python3.10/site-packages")
from rob_box_mcp_tools.tools.music import MusicManager

mm = MusicManager()

tests = [
    ("1: chop=4 (TEST FOR CLICKS)", """Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=1, amp=0.35)
d2 >> play("--.-", dur=1, sample=3, amp=0.15)
p1 >> dub([0,0,4,5], dur=4, oct=3, amp=0.4, chop=4)
p2 >> arpy([0,2,4,2,3,1], dur=1, oct=5, amp=0.5)"""),
    ("2: chop=8 (MORE EXTREME)", """Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=1, amp=0.35)
p1 >> dub([0,0,4,5], dur=4, oct=3, amp=0.4, chop=8)
p2 >> arpy([0,2,4,2,3,1], dur=1, oct=5, amp=0.5)"""),
    ("3: coarse=4 on drums (TEST FOR CLICKS)", """Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=1, amp=0.35, coarse=4)
d2 >> play("--.-", dur=1, sample=3, amp=0.15)
p1 >> dub([0,0,4,5], dur=4, oct=3, amp=0.4)
p2 >> arpy([0,2,4,2,3,1], dur=1, oct=5, amp=0.5)"""),
    ("4: coarse=8 on drums (MORE EXTREME)", """Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=1, amp=0.35, coarse=8)
d2 >> play("--.-", dur=1, sample=3, amp=0.15)
p1 >> dub([0,0,4,5], dur=4, oct=3, amp=0.4)
p2 >> arpy([0,2,4,2,3,1], dur=1, oct=5, amp=0.5)"""),
    ("5: coarse=4 on synth (TEST FOR CLICKS)", """Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=1, amp=0.35)
p1 >> dub([0,0,4,5], dur=4, oct=3, amp=0.4, coarse=4)
p2 >> arpy([0,2,4,2,3,1], dur=1, oct=5, amp=0.5)"""),
    ("6: chop + coarse combo", """Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=1, amp=0.35, coarse=4)
d2 >> play("--.-", dur=1, sample=3, amp=0.15)
p1 >> dub([0,0,4,5], dur=4, oct=3, amp=0.4, chop=4)
p2 >> arpy([0,2,4,2,3,1], dur=1, oct=5, amp=0.5)"""),
]

tn = int(sys.argv[1]) if len(sys.argv) > 1 else 0
if tn > 0:
    tests = [tests[tn - 1]]

for name, code in tests:
    print(f"\n{'='*50}")
    print(f"  {name}")
    print(f"{'='*50}")
    r = mm.execute_code(code)
    print(f"  Result: {str(r)[:100]}")
    print(f"  Playing 10s... listen!")
    time.sleep(10)
    mm.stop_all()
    time.sleep(1)
    print(f"  STOPPED")

print("\nALL DONE!")
