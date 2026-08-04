#!/bin/bash
# E2E voice test: ensure voice command file exists on build machine.
# If missing — generate via Yandex TTS on robot (10.1.1.21), push to build machine (10.1.1.249).
# Usage: ensure_voice_file.sh <voice_text> <voice_file_path>
set -e

VOICE_TEXT="$1"
VFILE="$2"

echo "Checking ${VFILE} on build machine..."
if sshpass -e ssh -o StrictHostKeyChecking=no ros2@10.1.1.249 "ls -la ${VFILE}" 2>/dev/null; then
  echo "VOICE_FILE_PRESENT"
  exit 0
fi

echo "VOICE_FILE_MISSING — generating via Yandex TTS on robot..."
# Ключи Yandex живут в voice-assistant контейнере на роботе
sshpass -e ssh -o StrictHostKeyChecking=no ros2@10.1.1.21 "cat > /tmp/gen_voice_e2e.py << 'PYEOF'
import os, json, urllib.request, base64
text = \"${VOICE_TEXT}\"
key = os.environ['YANDEX_API_KEY']
folder = os.environ['YANDEX_FOLDER_ID']
body = {
    'text': text,
    'lang': 'ru-RU',
    'voice': 'anton',
    'folderId': folder,
    'format': 'lpcm',
    'sampleRateHertz': 22050
}
req = urllib.request.Request(
    'https://tts.api.cloud.yandex.net/tts/v3/utteranceSynthesis',
    data=json.dumps(body).encode('utf-8'),
    headers={'Authorization': 'Api-Key ' + key, 'Content-Type': 'application/json'}
)
with urllib.request.urlopen(req, timeout=30) as r:
    resp = json.loads(r.read())
raw = base64.b64decode(resp['result']['audioChunk']['data'])
with open('/tmp/voice_cmd.wav', 'wb') as f:
    f.write(raw)
print('TTS_OK', len(raw))
PYEOF
docker cp /tmp/gen_voice_e2e.py voice-assistant:/tmp/ && \
docker exec voice-assistant python3 /tmp/gen_voice_e2e.py && \
docker cp voice-assistant:/tmp/voice_cmd.wav /tmp/voice_cmd.wav"

echo "Fetching WAV + converting to OGG..."
sshpass -e scp -o StrictHostKeyChecking=no ros2@10.1.1.21:/tmp/voice_cmd.wav /tmp/voice_cmd.wav
ffmpeg -y -i /tmp/voice_cmd.wav -c:a libopus -b:a 48k /tmp/voice_gen.ogg 2>/dev/null

echo "Pushing to build machine..."
sshpass -e scp -o StrictHostKeyChecking=no /tmp/voice_gen.ogg "ros2@10.1.1.249:${VFILE}"
sshpass -e ssh -o StrictHostKeyChecking=no ros2@10.1.1.249 "ls -la ${VFILE} && echo VOICE_FILE_GENERATED"
