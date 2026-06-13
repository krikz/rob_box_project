"""Test transition with the FIXED execute_code (gate=0 before freeAll)."""
import time
import sys
sys.path.insert(0, "/ws/src/rob_box_mcp_tools")
sys.path.insert(0, "/ws/install/rob_box_mcp_tools/lib/python3.10/site-packages")

# Re-import to get the patched version
import importlib
import rob_box_mcp_tools.tools.music as music_mod
importlib.reload(music_mod)
MusicManager = music_mod.MusicManager

mm = MusicManager()
mm.stop_all()
time.sleep(0.5)

# === TRACK 1 ===
print("=== TRACK 1: Drunk House 118 BPM ===")
track1 = """Clock.clear()
Clock.bpm = 118
Root.default = "E"
Scale.default = "minor"

d1 >> play("X..X..o.", sample=3, amp=0.15)
d2 >> play("--.-", dur=1, sample=4, amp=0.08)
d3 >> play("c", spack=1, sample=var([2,6,10], 8), dur=4, sus=4, amp=0.12)
p1 >> wobblebass([0,-2,0,4], dur=2, oct=3, amp=0.25)
p2 >> pianovel([0,2,4,5,4,2], dur=1, oct=5, amp=0.3)
p3 >> ambi(dur=8, amp=0.1, sus=8)
"""
r = mm.execute_code(track1)
print(f"  track1: success={r.get('success')}")
print("  Playing track 1 for 5 seconds...")
time.sleep(5)

# === TRANSITION via execute_code (with gate=0 fix) ===
print("\n=== TRANSITION -> TRACK 2 (via execute_code with gate=0 fix) ===")
track2 = """Clock.clear()
Clock.bpm = 122
Root.default = "G"
Scale.default = "minor"

d1 >> play("X..X..o.", sample=3, amp=0.18)
d2 >> play("--.-", dur=1, sample=4, amp=0.08)
d3 >> play("c", spack=1, sample=var([6,10,14], 8), dur=4, sus=4, amp=0.12, room=0.4)
p1 >> fuzz([0,0,3,5], dur=2, oct=3, amp=0.25)
p2 >> blip([0,2,4,3,2], dur=1, oct=5, amp=0.3, room=0.3)
p3 >> ambi(dur=6, amp=0.1, sus=6, room=0.7).follow(p1) + (0,2,4)
"""
r = mm.execute_code(track2)
print(f"  track2: success={r.get('success')}")
print("  Playing track 2 for 8 seconds - LISTEN FOR CLICKS...")
time.sleep(8)

mm.stop_all()
print("\n=== DONE ===")
