"""Test each layer of track 'Первая рюмка' individually to find clicking."""
import time
from rob_box_mcp_tools.tools.music import MusicManager

mm = MusicManager()
mm.stop_all()
time.sleep(0.5)

layers = [
    ("d1 kick+snare", 'd1 >> play("X..X..o.", sample=3, amp=0.18)'),
    ("d2 hi-hat", 'd2 >> play("--.-", dur=1, sample=4, amp=0.08)'),
    ("d3 choir spack=1", 'd3 >> play("c", spack=1, sample=var([6,10,14], 8), dur=4, sus=4, amp=0.12, room=0.4)'),
    ("p1 fuzz bass", 'p1 >> fuzz([0,0,3,5], dur=2, oct=3, amp=0.25)'),
    ("p2 blip melody", 'p2 >> blip([0,2,4,3,2], dur=1, oct=5, amp=0.3, room=0.3)'),
    ("p3 ambi pad", 'p1 >> fuzz([0,0,3,5], dur=2, oct=3, amp=0.25)\np3 >> ambi(dur=6, amp=0.1, sus=6, room=0.7).follow(p1) + (0,2,4)'),
]

preamble = 'Clock.clear()\nClock.bpm = 122\nRoot.default = "G"\nScale.default = "minor"\n'

for i, (name, code) in enumerate(layers, 1):
    print(f"\n=== Тест {i}/6: {name} ===")
    full_code = preamble + code
    r = mm.execute_code(full_code)
    success = r.get("success", False)
    msg = r.get("message", r.get("error", ""))
    print(f"  success={success}  msg={msg[:100]}")
    time.sleep(4)
    mm.stop_all()
    time.sleep(0.5)

print("\n=== Тест 7: ВСЕ ВМЕСТЕ (полный трек) ===")
full_track = preamble + """d1 >> play("X..X..o.", sample=3, amp=0.18)
d2 >> play("--.-", dur=1, sample=4, amp=0.08)
d3 >> play("c", spack=1, sample=var([6,10,14], 8), dur=4, sus=4, amp=0.12, room=0.4)
p1 >> fuzz([0,0,3,5], dur=2, oct=3, amp=0.25)
p2 >> blip([0,2,4,3,2], dur=1, oct=5, amp=0.3, room=0.3)
p3 >> ambi(dur=6, amp=0.1, sus=6, room=0.7).follow(p1) + (0,2,4)
"""
r = mm.execute_code(full_track)
success = r.get("success", False)
msg = r.get("message", r.get("error", ""))
print(f"  success={success}  msg={msg[:100]}")
time.sleep(5)
mm.stop_all()

print("\n=== ВСЕ ТЕСТЫ ЗАВЕРШЕНЫ ===")
