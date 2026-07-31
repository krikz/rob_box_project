#!/bin/bash
# E2E Voice Test - runs on build machine via SSH
R="$1"; D="$2"; V="$3"
ffmpeg -y -i /tmp/voice_new.ogg -af "highpass=f=200,volume=1.5,alimiter=limit=0.95" -ac 1 -ar 16000 /tmp/voice_eq.wav 2>/dev/null
pactl set-sink-volume @DEFAULT_SINK@ ${V}%
timeout ${D} parec --format=s16le --channels=1 --rate=16000 "${R}" &
RPID=$!
sleep 3
echo ">>> PLAYING"
paplay /tmp/voice_eq.wav && echo ">>> PLAY_DONE" || echo ">>> PLAY_FAIL"
wait $RPID 2>/dev/null
echo ">>> RECORDING_DONE"
