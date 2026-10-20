#!/usr/bin/env bash
# ADR-0154 PR-6: из манифеста батча (pdmx_batch.py run) — таблицы §3.4, серия M1–M3, версия пака. Запускать на katana.
#
#   scripts/music/pdmx_finalize.sh <версия ГГГГ.ММ.ДД.N> [--publish]
#
# Каталог работы — $WORK (по умолчанию ~/pr6): lib/ (JSON материалов), manifest.jsonl, selection.jsonl. Батч можно не
# останавливать: берётся МГНОВЕННЫЙ СНИМОК принятых материалов (манифест по строкам на момент старта + их JSON), поэтому
# и таблицы, и серия, и пак описывают один и тот же набор; продолжение батча на них не влияет.
# Результат в $WORK/out/<версия>/: progression_transitions.json, bass_tones.json, report.txt (raw), пак и манифест пака.
set -euo pipefail
VERSION="${1:?версия ГГГГ.ММ.ДД.N}"
PUBLISH="${2:-}"
WORK="${WORK:-$HOME/pr6}"
SEED="${SEED:-20261007}"
JOBS="${JOBS:-4}"
REPO="${REPO:-$WORK/repo}"
LOCAL_SRC="${LOCAL_SRC:-$HOME/scores_src/local}"
OUT="$WORK/out/$VERSION"
SNAP="$OUT/snapshot"
mkdir -p "$SNAP/lib"
cd "$WORK"

# 1. снимок: первые N строк манифеста (N фиксируем до копирования) и JSON принятых из них
N=$(wc -l < manifest.jsonl)
head -n "$N" manifest.jsonl > "$SNAP/manifest.jsonl"
python3 - "$SNAP/manifest.jsonl" "$WORK/lib" "$SNAP/lib" <<'PY'
import json, os, sys
manifest, src, dst = sys.argv[1:4]
good = bad = 0
with open(manifest, encoding="utf-8") as fh:
    rows = [json.loads(line) for line in fh if line.strip().endswith("}")]
with open(manifest, "w", encoding="utf-8") as fh:
    for m in rows:
        fh.write(json.dumps(m, ensure_ascii=False) + "\n")
        if m["status"] == "ok":
            name = m["material_id"].replace(":", "_") + ".json"
            if not os.path.exists(os.path.join(dst, name)):
                os.link(os.path.join(src, name), os.path.join(dst, name))
            good += 1
print(f"снимок: строк манифеста {len(rows)}, принято {good}")
PY
python3 "$REPO/scripts/music/pdmx_batch.py" index --manifest "$SNAP/manifest.jsonl" --lib "$SNAP/lib"

{
  echo "== отчёт батча (M1/M7) =="
  python3 "$REPO/scripts/music/pdmx_batch.py" report --manifest "$SNAP/manifest.jsonl"

  # 2. таблицы §3.4
  echo; echo "== корпус таблиц =="
  python3 "$REPO/scripts/music/research/score_corpus_stats.py" "$SNAP/lib" --out "$OUT/stats.json" --jobs "$JOBS"
  LABEL="PDMX no_license_conflict (ADR-0154 PR-6): снимок батча $VERSION, ранг — байесовский рейтинг; без стоп-листа ADR-0155"
  echo; echo "== PROGRESSION_TRANSITIONS =="
  python3 "$REPO/scripts/music/research/score_markov_harmony.py" "$OUT/stats.json" \
      --write-table "$OUT/progression_transitions.json" --corpus-label "$LABEL"
  echo; echo "== BASS_TONES =="
  python3 "$REPO/scripts/music/research/score_bass_tones.py" "$OUT/stats.json" \
      --write-table "$OUT/bass_tones.json" --corpus-label "$LABEL"

  # 3. серия M1–M3 (тот же снимок, один сид)
  echo; echo "== серия M1–M3, сид $SEED =="
  python3 "$REPO/scripts/music/research/score_series_m1_m3.py" "$SNAP/lib" --manifest "$SNAP/manifest.jsonl" \
      --stats "$OUT/stats.json" --seed "$SEED" -n 100
} 2>&1 | tee "$OUT/report.txt"

# 4. пак: каталог снимка + локальные партитуры Шифу (лицензия «private-local»)
python3 "$REPO/scripts/music/build_score_library.py" build --version "$VERSION" --out "$OUT/pack" \
    --lib-dir "$SNAP/lib" --local "$LOCAL_SRC" --local-license "private-local (Shifu only, not PD)" \
    --local-source "local scores (Shifu)" --jobs 2 2>&1 | tee -a "$OUT/report.txt"
ls -l "$OUT/pack" | tee -a "$OUT/report.txt"

if [ "$PUBLISH" = "--publish" ]; then
  python3 "$REPO/scripts/music/build_score_library.py" publish --manifest "$OUT/pack/scores-$VERSION.manifest.json" \
      --lock-out "$OUT/score_library.lock.json" 2>&1 | tee -a "$OUT/report.txt"
fi
