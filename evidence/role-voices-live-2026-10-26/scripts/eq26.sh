#!/bin/bash
# eq26.sh (внутри voice-assistant): EQ харнесса для новых syn/*.wav -> eq/ ; новые файлы дублируются в eq_new/
# Фильтр и параметры - строка 2141 .github/workflows/scripts/e2e_voice_test.sh.
mkdir -p /tmp/vp26/eq /tmp/vp26/eq_new
rm -f /tmp/vp26/eq_new/*.wav
for f in /tmp/vp26/syn/*.wav; do
  b=$(basename "$f")
  [ -s "/tmp/vp26/eq/$b" ] && continue
  ffmpeg -nostdin -y -loglevel error -i "$f" -af "highpass=f=100,volume=3.0,alimiter=limit=0.98,adelay=1500|all=1" -ac 1 -ar 16000 "/tmp/vp26/eq/$b" && cp "/tmp/vp26/eq/$b" "/tmp/vp26/eq_new/$b"
done
ls /tmp/vp26/eq_new | wc -l
