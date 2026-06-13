"""Test smooth transition: gate=0 (release) before /g_freeAll."""
import time
import socket
import struct

from rob_box_mcp_tools.tools.music import MusicManager

SC_HOST = "127.0.0.1"
SC_PORT = 57110

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

# === SMOOTH TRANSITION ===
print("\n=== SMOOTH TRANSITION -> TRACK 2 ===")

# Step 1: Send gate=0 to all synths (triggers ADSR release)
# /n_set -1 "gate" 0  → set gate=0 on ALL nodes (node ID -1 = all)
msg_gate = b"/n_set\x00\x00"
msg_gate += b",isf\x00\x00"  # int, string, float
msg_gate += struct.pack(">i", -1)  # node ID -1 = all
msg_gate += b"gate\x00\x00\x00\x00"  # parameter name
msg_gate += struct.pack(">f", 0.0)  # value = 0

try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(msg_gate, (SC_HOST, SC_PORT))
    print("  Sent gate=0 (release envelope)")
except Exception as e:
    print(f"  gate=0 failed: {e}")

# Step 2: Wait for release (50ms)
time.sleep(0.05)

# Step 3: /g_freeAll (now synths are in release phase, much quieter)
osc_freeall = b"/g_freeAll\x00\x00,i\x00\x00\x00\x00\x00\x01"
try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(osc_freeall, (SC_HOST, SC_PORT))
    print("  Sent /g_freeAll")
except Exception as e:
    print(f"  freeAll failed: {e}")

# Step 4: /g_new (create new group)
msg_gnew = bytearray()
for part in [b"/g_new\x00\x00", b",iii\x00\x00\x00\x00"]:
    msg_gnew.extend(part)
for v in [1, 0, 0]:
    msg_gnew.extend(struct.pack(">i", v))
try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(bytes(msg_gnew), (SC_HOST, SC_PORT))
    print("  Sent /g_new")
except Exception as e:
    print(f"  g_new failed: {e}")

# Step 5: Register new patterns (AFTER cleanup)
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
