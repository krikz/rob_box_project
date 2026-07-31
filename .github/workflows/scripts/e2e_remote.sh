#!/bin/bash
# E2E Voice Test - runs on build machine via SSH
R="$1"; D="$2"; V="$3"
ffmpeg -y -i /tmp/voice_new.ogg -af "highpass=f=200,volume=1.5,alimiter=limit=0.95" -ac 1 -ar 16000 /tmp/voice_eq.wav 2>/dev/null
pactl set-sink-volume @DEFAULT_SINK@ ${V}%
# Record raw PCM then convert to proper WAV
timeout ${D} parec --format=s16le --channels=1 --rate=16000 /tmp/e2e_raw.pcm &
RPID=$!
sleep 3
echo ">>> PLAYING"
paplay /tmp/voice_eq.wav && echo ">>> PLAY_DONE" || echo ">>> PLAY_FAIL"
wait $RPID 2>/dev/null
# Convert raw PCM to proper WAV
ffmpeg -y -f s16le -ar 16000 -ac 1 -i /tmp/e2e_raw.pcm "${R}" 2>/dev/null
rm -f /tmp/e2e_raw.pcm
echo ">>> RECORDING_DONE"
