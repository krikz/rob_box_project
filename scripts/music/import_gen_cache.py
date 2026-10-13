#!/usr/bin/env python3
"""Одноразовый импорт первых записей кэша генераций (ADR-0158 §2.10).

Каталог с пробами 13.10 (``artifacts_live_check_2026-10-13/suno/``: mp3 + JSON
ответов KIE) → библиотека ``/data/music_library`` (``engine.gen.library``) и
``gen_ledger``. Импортируются ТОЛЬКО перечисленные в ``FIRST_ENTRIES`` группы;
всё остальное в каталоге (``valkyries_la_*`` — отвергнуто товарищем Шифу) не
трогается.

Идемпотентно: запись есть, если совпал ``task_id`` + ``variant``, а при
неизвестном ``task_id`` — ``sha256`` файла. Повторный запуск ничего не меняет.

    # запускать там, где /data/music_library виден (контейнер voice-assistant)
    python scripts/music/import_gen_cache.py --src /tmp/suno [--library /data/music_library] [--dry-run]

Копирование каталога на робота — шаг деплоя PR-1 (координатор), не этот скрипт.
Нужен ``rob_box_mcp_tools`` на ``PYTHONPATH`` (в контейнере он уже установлен).
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

from rob_box_mcp_tools.engine.gen.keys import pool_key, request_key, sha256_file
from rob_box_mcp_tools.engine.gen.kie_suno import tracks_from_record
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary, utc_iso

GEN_PROVIDER = "kie_suno"


@dataclass(frozen=True)
class Entry:
    """Группа файлов одной задачи KIE (оба варианта)."""

    prefix: str                         # <prefix>_0.mp3, <prefix>_1.mp3
    kind: str
    style_key: str
    record: Optional[str] = None        # JSON recordInfo (taskId, кредиты, треки)
    lyrics_file: Optional[str] = None
    tts_provider: str = ""
    voice: str = ""
    # Только когда записи ответа нет (кавер): метаданные из параметров запроса.
    task_id: Optional[str] = None
    title: str = ""
    model: str = ""
    style_tags: str = ""
    duration_s: float = 0.0
    credits: float = 0.0
    completed_at: str = ""
    work_id: str = ""
    instrument: str = ""
    source_audio_file: Optional[str] = None
    meta_source: str = "api_record"
    ledger_id: str = ""                 # когда taskId неизвестен
    audio_weight: Optional[float] = None


#: ADR-0158 §2.10, таблица. ``valkyries_la_*`` сюда НЕ входит намеренно.
FIRST_ENTRIES = (
    Entry(
        prefix="valkyries_cover", kind="cover", style_key="orchestral",
        meta_source="request_params", ledger_id="legacy:valkyries_cover_2026-10-13",
        title="Ride of the Valkyries (RoBBox cover)", model="V6", duration_s=89.0, credits=12.0,
        style_tags=("epic symphonic orchestra, Wagner opera, Ride of the Valkyries, thundering brass "
                    "and horns, galloping strings, timpani, cinematic"),
        completed_at="2026-10-13T11:00:00+00:00", work_id="Ride Of The Valkyries",
        source_audio_file="valkyries_ours.wav", audio_weight=0.65,
    ),
    Entry(
        prefix="robbox_song", kind="robot_song", style_key="grunge", record="add_instr_record.json",
        lyrics_file="lyrics.txt", tts_provider="minimax", voice="Russian_ReliableMan",
    ),
    Entry(prefix="suno", kind="instrumental", style_key="grunge", record="record.json"),
)


def _utc_from_ms(ms: object) -> str:
    return utc_iso(datetime.fromtimestamp(float(ms) / 1000.0, timezone.utc))


def _load_record(src: Path, name: str) -> dict:
    doc = json.loads((src / name).read_text(encoding="utf-8"))
    return doc.get("data") or {}


def _param_input(rec: dict) -> dict:
    """``input`` запроса из ``paramJson`` (там же теги, лирика, модель)."""
    try:
        outer = json.loads(rec.get("paramJson") or rec.get("param") or "{}")
        inner = outer.get("input")
        return json.loads(inner) if isinstance(inner, str) else (inner or {})
    except ValueError:
        return {}


def import_entry(entry: Entry, src: Path, lib: GeneratedMusicLibrary) -> List[Dict[str, object]]:
    """Импортировать одну группу. Возвращает отчёт по вариантам."""
    rec = _load_record(src, entry.record) if entry.record else {}
    inp = _param_input(rec)
    task_id = str(rec.get("taskId") or "") or entry.task_id
    title = entry.title or str(inp.get("title") or "")
    model = entry.model or str(inp.get("model") or "")
    style_tags = entry.style_tags or str(inp.get("tags") or inp.get("style") or title)
    lyrics = (src / entry.lyrics_file).read_text(encoding="utf-8") if entry.lyrics_file else ""
    credits = float(rec.get("creditsConsumed") or entry.credits)
    done_at = _utc_from_ms(rec["completeTime"]) if rec.get("completeTime") else entry.completed_at
    src_sha = sha256_file(src / entry.source_audio_file) if entry.source_audio_file else ""
    instrumental = entry.kind in ("instrumental", "cover")
    api_tracks = {t.variant: t for t in tracks_from_record(rec)}
    audio_weight = entry.audio_weight if entry.audio_weight is not None else inp.get("audio_weight")

    rkey = request_key(
        kind=entry.kind, style_key=entry.style_key, theme="" if entry.kind == "cover" else title,
        work_id=entry.work_id, instrument=entry.instrument, lyrics=lyrics,
        tts_provider=entry.tts_provider, voice=entry.voice, gen_provider=GEN_PROVIDER, model=model,
        duration_s=entry.duration_s or (api_tracks[0].duration_s if 0 in api_tracks else 0.0),
        source_audio_sha=src_sha,
    )
    report: List[Dict[str, object]] = []
    ledger_id = task_id or entry.ledger_id
    new_spend = lib.ledger.record_spend(
        ledger_id, credits, kind=entry.kind, request_key=rkey, model=model,
        gen_provider=GEN_PROVIDER, ts=done_at or None,
    )
    for variant in (0, 1):
        mp3 = src / f"{entry.prefix}_{variant}.mp3"
        if not mp3.exists():
            report.append({"file": mp3.name, "status": "missing"})
            continue
        api = api_tracks.get(variant)
        duration = api.duration_s if api else entry.duration_s
        res = lib.save_track(
            mp3.read_bytes(),
            prompt=style_tags, title=title, lyrics=lyrics, model=model,
            duration_ms=int(duration * 1000), is_instrumental=instrumental,
            genre=entry.style_key, kind=entry.kind, style_key=entry.style_key,
            request_key=rkey, pool_key=pool_key(entry.kind, entry.style_key, instrumental),
            gen_provider=GEN_PROVIDER, task_id=task_id, variant=variant, credits=credits,
            tts_provider=entry.tts_provider, voice=entry.voice, work_id=entry.work_id,
            instrument=entry.instrument, source_audio_sha=src_sha,
            meta_source=entry.meta_source, created_at=done_at or None,
            notes=f"audio_weight={audio_weight}" if audio_weight is not None else "",
            dedupe_sha=True,
        )
        status = "error" if not res.get("success") else ("exists" if res.get("duplicate") else "imported")
        report.append({
            "file": mp3.name, "status": status, "id": res.get("id"),
            "task_id": task_id, "variant": variant, "ledger_new": new_spend,
            **({"error": res["error"]} if status == "error" else {}),
        })
    return report


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", required=True, type=Path, help="каталог с mp3 и JSON проб")
    ap.add_argument("--library", default=None, help="корень библиотеки (по умолчанию $MUSIC_LIBRARY_PATH или /data/music_library)")
    ap.add_argument("--dry-run", action="store_true", help="только показать, что будет импортировано")
    args = ap.parse_args(argv)

    if args.dry_run:
        for e in FIRST_ENTRIES:
            for v in (0, 1):
                f = args.src / f"{e.prefix}_{v}.mp3"
                print(f"{'ok     ' if f.exists() else 'MISSING'} {f.name}  kind={e.kind} style={e.style_key}")
        return 0

    lib = GeneratedMusicLibrary(args.library)
    failed = 0
    for entry in FIRST_ENTRIES:
        for row in import_entry(entry, args.src, lib):
            print(json.dumps(row, ensure_ascii=False))
            failed += row["status"] in ("missing", "error")
    print(f"library: {lib.count} трек(ов) в {lib.root_dir}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
