import sys, time
sys.path.insert(0, "/ws/install/rob_box_mcp_tools/lib/python3.10/site-packages")
from rob_box_mcp_tools.tools.music import MusicManager

mm = MusicManager()

tests = [
    ("1: delay=PRand (EXPECT CLICKS)", """Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=1, amp=0.35)
d2 >> play("--.-", dur=1, sample=3, amp=0.15, delay=PRand([0, PWhite(-0.1, 0.1)]))
p1 >> dub([0,0,4,5], dur=4, oct=3, amp=0.4)"""),
    ("2: sample=PRand (SAFE)", """Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=PRand([1,2,3,4]), amp=0.35)
d2 >> play("--.-", dur=1, sample=PRand([3,4,5]), amp=0.15)
p1 >> dub([0,0,4,5], dur=4, oct=3, amp=0.4)"""),
    ("3: dur=PRand (SAFE)", """Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=1, amp=0.35)
p1 >> dub([0,0,4,5], dur=4, oct=3, amp=0.4)
p2 >> saw(PRand([0,1,2,3,4]), dur=PRand([0.5,1,2]), oct=3, amp=0.5, attack=0.02, decay=0.1, sus=0.1)"""),
    ("4: PRand notes (SAFE)", """Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=1, amp=0.35)
p1 >> saw(PRand([0,1,2,3,4]), dur=0.5, oct=3, amp=0.5, attack=0.02, decay=0.1, sus=0.1)
p2 >> karp(PRand([0,2,4,3,5]), dur=1, oct=2, amp=0.35)"""),
    ("5: FULL combo (no delay!)", """Clock.clear(); Clock.bpm=136; Root.default="F#"; Scale.default="phrygian"
d1 >> play("X...X...", sample=PRand([4,5,6]), amp=0.35)
d2 >> play("--.-", dur=1, sample=PRand([3,4,5]), amp=0.15)
p1 >> saw(PRand([0,1,2,3,4]), dur=0.5, oct=3, amp=0.5, attack=0.02, decay=0.1, sus=0.1)
p2 >> karp(PRand([0,1,2,3]), dur=PRand([0.5,1,2]), oct=2, amp=0.35)"""),
    ("6: Acid recipe", """Clock.clear(); Clock.bpm=130; Root.default="F#"; Scale.default="minor"
p1 >> saw(PRand([0,1,2,3,4]), dur=0.25, oct=3, amp=0.5, attack=0.01, decay=0.08, sus=0.05, room=0.2)
p2 >> bass(PRand([0,1,2,3,4]), dur=PRand([0.5,1,2]), oct=2, amp=0.4)
d1 >> play("X...X...", sample=5, amp=0.35)
d2 >> play("..o...o.", sample=3, amp=0.15)"""),
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
