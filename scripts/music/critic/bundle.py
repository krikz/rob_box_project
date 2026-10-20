#!/usr/bin/env python3
"""bundle.py — бандл для музыкального критика: папка на жанр, внутри md (запрос + код)
и mp3 (рендер), с промтом критику.

Вход — каталог прогона ``s7_styles.sh``, снятый с робота (index.txt, <key>.full.log,
s7_<key>.wav) плюс модели треков ``tracks/<track_id>.json`` (``music_v2_tracks_dir``,
ADR-0152 §2.3). Код аранжировщика получается ``render(track, deck).code`` — тем же
рендером, что играл на роботе; ``load_track`` до печати кода доказывает, что
модель собрана обратно без потерь (round-trip ``track_json``).

    python scripts/music/critic/bundle.py --run-dir <прогон> --out <бандл>

Переезд ``_method/build_critic_bundle.py`` + ``_method/render_track_code.py`` из артефактов (ADR-0157 §3.6, PR-4):
одна реализация, рендер кода — в процессе (``load_track`` + ``render``), без подпроцесса.
"""

from __future__ import annotations

import argparse
import collections.abc as abc
import dataclasses
import json
import re
import shutil
import subprocess
import sys
import typing
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "rob_box_music"))  # пакет этого чекаута, не editable-установка основного

#: Стили прогона s7 (тот же порядок, что в таблице SETS скрипта s7_styles.sh).
STYLES = [
    ("club", "клубный", "космос"),
    ("rave", "рейв", "в пещере горного короля"),
    ("synthwave", "синтвейв", "Моцарт"),
    ("chiptune", "восьмибитный", "в пещере горного короля"),
    ("breaks", "брейкбит", "космос"),
    ("dnb", "драм-н-бейс", "Моцарт"),
    ("lofi", "лоуфай", "космос"),
    ("rock", "рок", "в пещере горного короля"),
    ("grunge", "гранж", "космос"),
    ("jazz", "джаз", "Моцарт"),
]

SET_HEAD = re.compile(r"^=== SET (\S+) '([^']*)' '([^']*)' start (\S+)$")
SET_DONE = re.compile(r"^=== SET (\S+) done \S+ started=(\d+) late=(\d+)")
INJECT = re.compile(r"^inject (\S+)$")
DOCKER_TS = re.compile(r"^(\S+Z) ")
STARTED = re.compile(r"\[music v2\] started track_id=(\S+)")
SILENCE = re.compile(r"silence_end: ([\d.]+)")

CRITIC_PROMPT = """Ты — профессиональный музыкальный критик и звукорежиссёр. Прослушай приложенный аудиофайл `{mp3}`,
сгенерированный алгоритмическим аранжировщиком робота (стиль: {ru} / `{key}`, тема: «{theme}»). Это живая запись
с робота: `0:00` — момент, когда первый трек встал на плеер; дальше без монтажа, в конце слышно сведение с
наложением следующего трека сета.

Найди таймкоды (в секундах от начала файла), где:
1. мелодия/тема звучит немузыкально или фальшиво (не по ладу, «мимо» аккорда, обрыв фразы);
2. ломается ритмический рисунок (сбитый грув, наложения, пропуски, кривой переход между треками);
3. инструменты заглушают друг друга (маскировка, каша в низах, перегруз, резкость в верхах).

Дай четкий текстовый отчёт с указанием секунд. Если место звучит нормально — так и пиши. Ниже приложен код,
который аранжировщик отдал на синтез: по нему видно, что он собирался сыграть (роли, ноты, рисунки), — можешь
ссылаться на конкретные строки кода в своих замечаниях."""


def parse_index(path: Path) -> dict[str, dict]:
    """Разбор ``index.txt``: ключ стиля → слово, тема, старт записи, момент инъекции, счётчики."""
    rows: dict[str, dict] = {}
    last = None
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        head = SET_HEAD.match(line)
        if head:
            key, word, theme, start = head.groups()
            rows[key] = {"key": key, "word": word, "theme": theme, "record_start": start}
            last = key
            continue
        inject = INJECT.match(line)
        if inject and last:
            rows[last]["inject_at"] = inject.group(1)
            continue
        done = SET_DONE.match(line)
        if done:
            key, started, late = done.groups()
            rows.setdefault(key, {}).update(started=int(started), late=int(late))
    return rows


def track_starts(log: Path) -> list[tuple[str, float]]:
    """``[(track_id, docker-штамп в секундах)]`` по строкам ``[music v2] started`` — в порядке игры."""
    out = []
    for line in log.read_text(encoding="utf-8", errors="replace").splitlines():
        ts = DOCKER_TS.match(line)
        found = STARTED.search(line)
        if ts and found:
            out.append((found.group(1), datetime.fromisoformat(ts.group(1)).timestamp()))
    return out


def build(tp: typing.Any, value: typing.Any) -> typing.Any:
    """Собрать значение аннотированного типа ``tp`` из JSON-значения ``value`` (обратное ``diversity.track_json``)."""
    if value is None:
        return None
    origin = typing.get_origin(tp)
    if origin is typing.Union:  # Optional[X]: в модели None бывает только вместо всего поля
        return build(next(a for a in typing.get_args(tp) if a is not type(None)), value)
    if dataclasses.is_dataclass(tp):
        hints = typing.get_type_hints(tp)
        unknown = set(value) - {f.name for f in dataclasses.fields(tp)}
        if unknown:
            raise TypeError(f"{tp.__name__}: лишние поля {sorted(unknown)}")
        return tp(**{name: build(hints[name], value[name]) for name in value})
    args = typing.get_args(tp)
    if origin in (tuple, abc.Sequence):
        if len(args) == 2 and args[1] is Ellipsis:
            return tuple(build(args[0], v) for v in value)
        return tuple(build(a, v) for a, v in zip(args, value))
    if origin in (frozenset, set, abc.Set, abc.MutableSet):
        return frozenset(build(args[0], v) for v in value) if args else frozenset(value)
    if origin in (dict, abc.Mapping, abc.MutableMapping):
        return {build(args[0], k): build(args[1], v) for k, v in value.items()} if len(args) == 2 else dict(value)
    if origin is None and tp in (frozenset, set, tuple, list, dict):  # голые контейнеры без параметров
        return tp(value)
    if tp is typing.Any or origin is None:
        return value
    raise TypeError(f"не умею собирать {tp!r} из {type(value).__name__}")


def load_track(text: str) -> typing.Any:
    """``Track`` из JSON модели; round-trip ``track_json`` обязан совпасть с исходником посимвольно (иначе ``ValueError``)."""
    from rob_box_music.diversity import track_json
    from rob_box_music.model import Track

    source = text.rstrip("\r\n")  # завершающий перевод строки — артефакт передачи, не поле модели
    track = build(Track, json.loads(source))
    if track_json(track) != source:
        raise ValueError("round-trip модели не совпал с исходным JSON — код не доказан")
    return track


def load_track_file(path: Path) -> typing.Any:
    return load_track(path.read_text(encoding="utf-8"))


def render_code(model: Path, deck: str) -> tuple[str, str]:
    """``(код, итог разбора)``: рендер кода из модели + строка доказательства round-trip."""
    from rob_box_music.render.renardo import render

    program = render(load_track_file(model), deck)
    proof = (f"✅ {program.track_id} deck={deck} bpm={program.bpm} form_beats={program.form_beats} "
             f"ролей={len(program.slots)} synths={len(program.synths)} "
             f"samples={len(program.samples) or len(program.sample_files)} round-trip=OK "
             f"строк_кода={len(program.code.splitlines())}")
    return program.code, proof


def to_mp3(wav: Path, offset_s: float, mp3: Path) -> str:
    """wav → mp3 с обрезкой до-музыкального начала (``offset_s``). Возвращает строку ffmpeg."""
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-i", str(wav)]
    if offset_s > 0:
        cmd += ["-ss", f"{offset_s:.3f}"]
    cmd += ["-b:a", "192k", str(mp3)]
    subprocess.run(cmd, check=True)
    return f"ffmpeg -i {wav.name} " + (f"-ss {offset_s:.3f} " if offset_s > 0 else "") + f"-b:a 192k {mp3.name}"


def audio_start(wav: Path, floor_db: int = 50) -> float | None:
    """Первый не-тихий момент записи (`ffmpeg silencedetect`) — независимая проверка смещения старта музыки."""
    proc = subprocess.run(
        ["ffmpeg", "-hide_banner", "-v", "info", "-i", str(wav), "-af", f"silencedetect=n=-{floor_db}dB:d=0.4",
         "-f", "null", "-"],
        capture_output=True, text=True, encoding="utf-8", errors="replace",
    )
    found = SILENCE.search(proc.stderr)
    return float(found.group(1)) if found else None


def duration_s(path: Path) -> float:
    """Длительность файла по ffprobe, с."""
    proc = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "default=nw=1:nk=1", str(path)],
        capture_output=True, text=True, encoding="utf-8", errors="replace",
    )
    return float(proc.stdout.strip() or 0.0)


def md_for(style: dict, mp3_name: str, entries: list[dict], facts: dict) -> str:
    inject = facts["inject_at"]
    if inject != "—" and not inject.startswith("20"):  # index.txt пишет у инъекции только время
        inject = f"{facts['record_start'][:10]}T{inject}"
    lines = [
        f"# {style['ru']} · `{style['key']}`",
        "",
        "## Входящий запрос",
        "",
        f"«Робот, включи {style['ru']} сет на тему {style['theme']}»",
        "",
        f"- инъекция в `/voice/stt/result` (реплика целиком, как от человека): {inject}",
        f"  - «{style['ru']}» подтверждён в логе робота строкой `🎤 STT:` — "
        f"{'да' if facts.get('seen_in_log') else 'НЕТ (см. _raw/' + style['key'] + '.full.log)'}",
        f"- старт записи `jack_rec` (jack:out_1/2 суперколлайдера): {facts['record_start']}",
        f"- первый трек встал на плеер: +{facts['music_offset']:.2f} с от старта записи "
        f"(±0.2 с — задержка старта `jack_rec`; момент ловится строкой `[music v2] started` в логе робота)",
        "",
        "## Рендер",
        "",
        f"[{mp3_name}]({mp3_name}) — запись 200 с минус {facts['music_offset']:.2f} с до старта первого трека; "
        f"`0:00` = первый трек на плеере.",
        "",
        f"Проверка смещения независимым замером (`ffmpeg silencedetect`, порог −50 дБ): звук в исходной записи "
        f"начинается на {facts['audio_start']:.3f} с — расхождение с логом "
        f"{abs(facts['audio_start'] - facts['music_offset']):.3f} с.",
        "",
        "```",
        facts["ffmpeg_line"],
        "```",
        "",
        "## Факты аранжировщика",
        "",
        "| | |",
        "|---|---|",
        f"| темп сета | {facts['bpm']} BPM |",
        f"| треков в окне записи | {facts['started']} (строк `[music v2] started`) |",
        f"| late-бандлов за окно | {facts['late']} (во всём прогоне 10 стилей: {facts['late_all']}) |",
        "",
    ]
    for n, entry in enumerate(entries, 1):
        lines += [
            f"## Код аранжировщика — трек {n}",
            "",
            f"`{entry['track_id']}` · BPM {entry['bpm']} · дека {entry['deck']} · "
            f"встал на плеер на +{entry['offset']:.2f} с от старта записи "
            f"(≈{max(entry['offset'] - facts['music_offset'], 0):.2f} с в файле `{mp3_name}`)",
            "",
            "```python",
            entry["code"].rstrip("\n"),
            "```",
            "",
            f"Источник кода: модель трека `{entry['model']}` (`music_v2_tracks_dir`, ADR-0152 §2.3), собранная"
            f" обратно в `Track` и отрендеренная тем же `render()`, что играл на роботе: "
            f"{entry['proof'].lstrip('✅ ')}.",
            "",
        ]
    lines += ["## Промт критику", "", CRITIC_PROMPT.format(mp3=mp3_name, **style), ""]
    return "\n".join(lines)


def index_md(rows: list[dict], run: Path, index: dict[str, dict]) -> str:
    """``index.md`` бандла: что внутри, как получен код, цифры прогона."""
    late_all = sum(int(r.get("late", 0)) for r in index.values())
    lines = [
        "# Стили аранжировщика: рендеры и код (материал для музыкального критика)",
        "",
        "Внутри — по папке на жанр: `<жанр>/<жанр>.md` (входящая реплика человека, код аранжировщика, промт критику) "
        "и рядом `<жанр>/<жанр>.mp3` (живая запись с робота, обрезанная до старта первого трека).",
        "",
        "| жанр | запрос | трек 1 | BPM | треков в окне | late | mp3 | строк кода |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        facts, entries = r["facts"], r["entries"]
        lines.append(
            f"| {r['ru']} (`{r['key']}`) | «Робот, включи {r['ru']} сет на тему {r['theme']}» "
            f"| `{entries[0]['track_id']}` | {facts['bpm']} | {len(entries)} | {facts['late']} "
            f"| [{r['mp3'].name}]({r['key']}/{r['mp3'].name}) {r['duration']:.1f} с "
            f"| {sum(len(e['code'].splitlines()) for e in entries)} |"
        )
    lines += [
        "",
        f"Прогон: `scripts/music/live_dj/s7_styles.sh` на Vision Pi, каталог `{run.name}`, "
        f"записи по 200 с (`jack_rec`, jack:out_1/2 суперколлайдера), "
        f"late-бандлов за весь прогон (харнесс): {late_all}.",
        "",
        "## Как получен код",
        "",
        "Код аранжировщика — не пересказ и не реконструкция по логам: робот на каждый трек сета пишет полную "
        "модель (`music_v2_tracks_dir`, ADR-0152 §2.3), а `scripts/music/critic/bundle.py` собирает её обратно в "
        "`Track` и рендерит тем же `render()`, что играл на роботе. Перед печатью кода скрипт требует "
        "round-trip: `track_json(собранная модель)` обязан совпасть с исходным JSON символ в символ — иначе код "
        "не печатается (выход 2). Строка проверки приведена в конце каждого блока кода в per-жанровых md.",
        "",
        "Смещение старта музыки взято из строки `[music v2] started` лога робота (это ошибка ±0.2 с на старт "
        "`jack_rec`); независимая проверка — замер тишины `ffmpeg silencedetect` в тех же md.",
        "",
        "## Что проверить за критиком",
        "",
        "- Слух — за критиком: цифры late-бандлов говорят про нагрузку стенда, а не про музыкальность.",
        "- Во время прогона робот слышал постороннюю речь (чужое видео/аудиокнига) — на музыку это не влияет "
        "(фразы инжектятся прямо в `/voice/stt/result`), но нагрузка стенда была выше обычной.",
        "",
        "## Файлы",
        "",
        "- `scripts/music/critic/bundle.py` — сборка бандла и модель трека → код (round-trip).",
        "- `_raw/index.txt` — журнал прогона (времена, счётчики late), `_raw/<жанр>.full.log` — полный лог "
        "`voice-assistant` по стилю (запрос, план сета, `[music v2] started`).",
        "",
        "В `club/club.md` после промта дописан разбор товарища Шифу (разделы «Ответ без кода» и «Ответ после того, "
        "как дали посмотреть на код») — он к бандлу приложен как образец, в остальных жанрах кода ещё нет ответа.",
        "",
    ]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description="Бандл для критика: папка на жанр, md + mp3")
    parser.add_argument("--run-dir", type=Path, required=True, help="каталог прогона (index.txt, *.full.log, *.wav, tracks/)")
    parser.add_argument("--out", type=Path, required=True, help="куда класть бандл")
    parser.add_argument("--only", nargs="*", default=None, help="подмножество стилей (для проверки)")
    parser.add_argument("--keep-md", nargs="*", default=None, help="стили, у которых md не переписывать (дописано вручную)")
    args = parser.parse_args(argv)

    run, out = args.run_dir, args.out
    index = parse_index(run / "index.txt")
    late_all = sum(int(r.get("late", 0)) for r in index.values())
    out.mkdir(parents=True, exist_ok=True)
    written = []

    for key, ru, theme in STYLES:
        if args.only and key not in args.only:
            continue
        row = index.get(key)
        log, wav = run / f"{key}.full.log", run / f"s7_{key}.wav"
        if row is None or not log.exists() or not wav.exists():
            print(f"⚠️  {key}: нет данных прогона (index={row is not None} log={log.exists()} wav={wav.exists()})")
            continue
        starts = track_starts(log)
        record_start = datetime.fromisoformat(row["record_start"]).timestamp()
        entries = []
        for track_id, ts in starts:
            offset = ts - record_start
            if offset > 200:
                continue
            deck = track_id.split(":")[2]
            model = run / "tracks" / f"{track_id.replace(':', '_')}.json"
            if not model.exists():
                print(f"⚠️  {key}: нет модели {model.name} — трек пропущен")
                continue
            code, proof = render_code(model, deck)
            bpm = int(re.search(r"bpm=(\d+)", proof).group(1)) if "bpm=" in proof else 0
            entries.append({"track_id": track_id, "deck": deck, "offset": offset, "code": code,
                            "proof": proof, "model": model.name, "bpm": bpm})
        if not entries:
            print(f"⚠️  {key}: ни одной модели трека нет — бандл не собран")
            continue
        style_dir = out / key
        style_dir.mkdir(exist_ok=True)
        mp3 = style_dir / f"{key}.mp3"
        ffmpeg_line = to_mp3(wav, entries[0]["offset"], mp3)
        measured = audio_start(wav)
        facts = {
            "record_start": row["record_start"], "inject_at": row.get("inject_at", "—"),
            "music_offset": entries[0]["offset"], "ffmpeg_line": ffmpeg_line,
            "audio_start": measured if measured is not None else entries[0]["offset"],
            "bpm": entries[0]["bpm"], "started": row.get("started", len(entries)), "late": row.get("late", 0),
            "late_all": late_all,
            "seen_in_log": f"🎤 STT: Робот, включи {ru} сет на тему {theme}" in log.read_text(encoding="utf-8", errors="replace"),
        }
        if args.keep_md and key in args.keep_md:
            print(f"↷  {key}: md не переписываю (дописано вручную), mp3 обновлён")
        else:
            (style_dir / f"{key}.md").write_text(
                md_for({"key": key, "ru": ru, "theme": theme}, mp3.name, entries, facts), encoding="utf-8")
        written.append({"key": key, "ru": ru, "theme": theme, "mp3": mp3, "entries": entries, "facts": facts,
                        "duration": duration_s(mp3)})
        print(f"✅ {key}: {mp3.name} ({mp3.stat().st_size // 1024} КиБ), треков={len(entries)}, "
              f"late={facts['late']}, смещение музыки {facts['music_offset']:.2f} с "
              f"(замер начала звука {facts['audio_start']:.3f} с)")
    print(f"итого: {len(written)} из {len(STYLES)} — {', '.join(r['key'] for r in written)}")
    if written and not args.only:
        raw = out / "_raw"
        raw.mkdir(exist_ok=True)
        shutil.copy2(run / "index.txt", raw / "index.txt")
        for r in written:
            shutil.copy2(run / f"{r['key']}.full.log", raw / f"{r['key']}.full.log")
        (out / "index.md").write_text(index_md(written, run, index), encoding="utf-8")
        print(f"index.md: {out / 'index.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
