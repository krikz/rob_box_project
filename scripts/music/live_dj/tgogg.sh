# usage: TG_CHAT_ID=<chat> bash tgogg.sh <wav on host> "<caption>"
# на Vision Pi; токен берётся из окружения контейнера telegram-bot. Моно не форсируем (без -ac 1).
W="$1"; C="$2"; B=$(basename "$W" .wav)
docker cp "$W" telegram-bot:/tmp/$B.wav
docker exec telegram-bot ffmpeg -y -loglevel error -i /tmp/$B.wav -c:a libopus -b:a 48k /tmp/$B.ogg
docker exec -i -e C="$C" -e B="$B" -e CHAT="${TG_CHAT_ID:?set TG_CHAT_ID}" telegram-bot python3 - <<'PY'
import os, requests
t=os.environ["TELEGRAM_BOT_TOKEN"]; b=os.environ["B"]
r=requests.post(f"https://api.telegram.org/bot{t}/sendVoice",data={"chat_id":os.environ["CHAT"],"caption":os.environ["C"]},files={"voice":open(f"/tmp/{b}.ogg","rb")},timeout=120)
print(r.status_code, r.text[:120].replace(t,"<tok>"))
PY
