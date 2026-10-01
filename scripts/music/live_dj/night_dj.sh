#!/bin/bash
# на Vision Pi: bash /tmp/cc_night_dj.sh  (запускать через nohup)
D=/tmp/night_dj; mkdir -p $D
THEMES=("космос" "пираты" "дождливый ночной город" "детский праздник" "киберпанк" "море и чайки" "ретро восьмидесятые денди" "весенний лес")
i=0
for T in "${THEMES[@]}"; do
  i=$((i+1)); S=$(date -u +%Y-%m-%dT%H:%M:%SZ)
  echo "=== SET $i '$T' start $S" >> $D/index.txt
  bash /tmp/cc_say.sh "Робот останови музыку" 6 1 >/dev/null 2>&1
  bash /tmp/cc_say.sh "Робот включи диджей сет на тему $T" 5 1 >/dev/null 2>&1
  ( sleep 200; bash /tmp/cc_rec.sh 45 s${i}_a.wav; mv /tmp/s${i}_a.wav $D/ 2>/dev/null
    sleep 150; bash /tmp/cc_rec.sh 45 s${i}_b.wav; mv /tmp/s${i}_b.wav $D/ 2>/dev/null ) &
  sleep 600; wait
  docker logs voice-assistant --since "$S" 2>&1 | grep -vE '^\[dialogue_node-4\]   \[|tools\(61\)|heartbeat|diagnostic|HTTP Request|VoskAPI' > $D/set${i}.log
  echo "=== SET $i done $(date -u +%H:%M:%SZ) log=$(wc -l < $D/set${i}.log)" >> $D/index.txt
done
bash /tmp/cc_say.sh "Робот останови музыку" 6 1 >/dev/null 2>&1
echo "=== ALL DONE $(date -u +%H:%M:%SZ)" >> $D/index.txt
