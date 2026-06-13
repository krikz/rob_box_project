#!/bin/bash
# test_prand.sh — Test PRand usages on robot
# Usage: bash test_prand.sh [test_number]

ROBOT="ros2@10.1.1.11"
SSH="sshpass -p open ssh -o StrictHostKeyChecking=no $ROBOT"
EXEC='docker exec voice-assistant python3 -c'

stop() {
    $SSH "$EXEC \"from qirky.tools.music import MusicManager; MusicManager().stop_music('all')\"" 2>/dev/null
    sleep 1
}

play() {
    local code="$1"
    local desc="$2"
    echo ""
    echo "=========================================="
    echo "TEST: $desc"
    echo "=========================================="
    stop
    $SSH "$EXEC \"from qirky.tools.music import MusicManager; mm=MusicManager(); print(mm.execute_music_code('$code'))\"" 2>/dev/null
    echo "Playing 10s... listen for clicks!"
    sleep 10
    stop
    echo "DONE"
}

trap stop EXIT

echo "PRand Test Suite"
echo "================"
echo "Each test: 10 seconds, listen for CLICKS"
echo ""

case "${1:-all}" in
  1|all)
    # TEST 1: delay=PRand — EXPECT CLICKS
    play 'Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=1, amp=0.35)
d2 >> play("--.-", dur=1, sample=3, amp=0.15, delay=PRand([0, PWhite(-0.1, 0.1)]))
p1 >> dub([0,0,4,5], dur=4, oct=3, amp=0.4)' \
    "1: delay=PRand (EXPECT CLICKS)"
    ;;&
  2|all)
    # TEST 2: sample=PRand — SAFE
    play 'Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=PRand([1,2,3,4]), amp=0.35)
d2 >> play("--.-", dur=1, sample=PRand([3,4,5]), amp=0.15)
p1 >> dub([0,0,4,5], dur=4, oct=3, amp=0.4)' \
    "2: sample=PRand (SAFE — random samples)"
    ;;&
  3|all)
    # TEST 3: dur=PRand — SAFE
    play 'Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=1, amp=0.35)
d2 >> play("--.-", dur=1, sample=3, amp=0.15)
p1 >> dub([0,0,4,5], dur=4, oct=3, amp=0.4)
p2 >> saw(PRand([0,1,2,3,4]), dur=PRand([0.5,1,2]), oct=3, amp=0.5, attack=0.02, decay=0.1, sus=0.1)' \
    "3: dur=PRand on synth (SAFE)"
    ;;&
  4|all)
    # TEST 4: PRand notes — SAFE
    play 'Clock.clear(); Clock.bpm=125; Root.default="F"; Scale.default="minor"
d1 >> play("X..X.o..", sample=1, amp=0.35)
p1 >> saw(PRand([0,1,2,3,4]), dur=0.5, oct=3, amp=0.5, attack=0.02, decay=0.1, sus=0.1)
p2 >> karp(PRand([0,2,4,3,5]), dur=1, oct=2, amp=0.35)' \
    "4: PRand for notes only (SAFE)"
    ;;&
  5|all)
    # TEST 5: Full combo (no delay!)
    play 'Clock.clear(); Clock.bpm=136; Root.default="F#"; Scale.default="phrygian"
d1 >> play("X...X...", sample=PRand([4,5,6]), amp=0.35)
d2 >> play("--.-", dur=1, sample=PRand([3,4,5]), amp=0.15)
p1 >> saw(PRand([0,1,2,3,4]), dur=0.5, oct=3, amp=0.5, attack=0.02, decay=0.1, sus=0.1)
p2 >> karp(PRand([0,1,2,3]), dur=PRand([0.5,1,2]), oct=2, amp=0.35)' \
    "5: FULL PRand combo (sample+dur+notes, NO delay)"
    ;;&
  6|all)
    # TEST 6: Acid recipe
    play 'Clock.clear(); Clock.bpm=130; Root.default="F#"; Scale.default="minor"
p1 >> saw(PRand([0,1,2,3,4]), dur=0.25, oct=3, amp=0.5, attack=0.01, decay=0.08, sus=0.05)
p2 >> bass(PRand([0,1,2,3,4]), dur=PRand([0.5,1,2]), oct=2, amp=0.4)
d1 >> play("X...X...", sample=5, amp=0.35)
d2 >> play("..o...o.", sample=3, amp=0.15)' \
    "6: Acid recipe with PRand (full speed)"
    ;;&
esac

echo ""
echo "===================="
echo "ALL TESTS COMPLETE!"
echo "Report: which had clicks?"
echo "===================="
