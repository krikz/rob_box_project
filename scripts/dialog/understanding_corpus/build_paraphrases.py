#!/usr/bin/env python3
"""Корпус замера понимания, часть 2: перефразы шагов марафона (карточка #4163, этап 2).

    python scripts/dialog/understanding_corpus/build_paraphrases.py --raw <каталог вне репо>   # собрать (DeepSeek)
    python scripts/dialog/understanding_corpus/build_paraphrases.py --check                    # сверить файл, без сети
    python scripts/dialog/understanding_corpus/build_paraphrases.py --sample 20                # случайные пары

По 2-3 перефразы на каждый оцениваемый текстовый шаг части 1: синонимы, порядок слов, разговорное, падежи. Пишет их
модель (DeepSeek), потом каждая проходит автопроверку ``alternatives.check_paraphrase``: вейк в начале тот же (без вейка -
без вейка), имена людей и треков, числа, цитаты в «…» и якоря ожиданий на месте, не копия исходника. Не прошла -
модель получает причину и пишет замену (до ``--rounds`` кругов). Ожидание шага наследуется без изменений (в файле его
нет). Исключены шаги, где перефразировать нечего: голое вейк-слово «Робот.» (подхват бэклога).

``--check`` повторяет автопроверку по замороженному файлу и сверяет хеш; ``--sample N`` печатает N случайных пар
«исходник → перефраза» (зерно 4163) для ручной выборки. Сырые ответы модели - в ``--raw`` (в репозиторий не кладутся).
Ключ ``DEEPSEEK_API_KEY`` - из ``scripts/dialogue/.env`` или окружения, нигде не печатается.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import random
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))

import alternatives as alt  # noqa: E402
import build_marathon  # noqa: E402

PART = 2
OUT = alt.part_path(PART)
WANT, MIN_KEEP = 3, 2
BATCH = 10
MODEL = "deepseek-v4-flash"
TEMPERATURE = 1.0
SEED = 4163
ENV_FILES = (build_marathon.dm.REPO / "scripts" / "dialogue" / ".env", build_marathon.dm.REPO / ".env")
EXCLUDE_BARE_WAKE = "голое вейк-слово («Робот.»): перефразировать нечего, шаг проверяет подхват бэклога"
#: Ручная проверка воркера (28.10): перефразы, прошедшие автопроверку, но меняющие смысл просьбы. ``(id, текст)`` →
#: причина; при сборке они отбрасываются и модель пишет замену (``--top-up``).
MANUAL_REJECT: Dict[Tuple[str, str], str] = {
    ("5/n505_vibe_dark", "Робот, атмосферу сделай помрачнее, в духе киберпанка как в киберпанке."):
        "сбой модели: «в духе киберпанка как в киберпанке» - повтор",
    ("6/n607_vibe_chill", "Робот, сделай потише и поспокойнее, Валера ведь уснёт за рулём."):
        "«потише» - просьба о громкости, а в исходнике - о характере музыки («помягче»)",
    ("6/n610_dj_state_honest", "Робот, у тебя диджей-режим теперь выключен или не включён?"):
        "вопрос с готовым ответом: «выключен или не включён» - оба варианта значат «выключен»",
    ("6/n610_dj_state_honest", "Робот, диджей-режим в данный момент выключен али не включён?"):
        "вопрос с готовым ответом: «выключен али не включён» - оба варианта значат «выключен»",
    ("10/n1009_memory_context_after_reset", "Робот, вспомни, какой чай я пью."):
        "пропала прямая просьба «поищи в памяти», на которой стоит ожидание memory_search",
}

SYSTEM = """Ты собираешь тестовый корпус для голосового робота в мастерской. Тебе дают реплики людей - так, как их
произносят вслух. Для каждой реплики напиши перефразы: как ДРУГОЙ человек сказал бы ТО ЖЕ САМОЕ своими словами.
Меняй синонимы, порядок слов, разговорность, падежи и конструкции; каждая перефраза заметно отличается от исходника и от
других перефраз. Смысл просьбы не меняй, ничего не добавляй и не убирай.

Жёсткие правила:
1. Обращение (поле wake). Если wake не пусто - перефраза начинается ровно с этого слова и запятой (например
   «Робот, ...»), и больше нигде в перефразе этого слова нет. Если wake пусто - в перефразе нет ни «робот», ни «робокс»,
   ни «роман», ни «рома», ни «робо», ни других обращений к роботу.
2. Имена людей, голосов, команд и героев песен (поле names) остаются; падеж можно менять.
3. Числа (поле numbers) остаются теми же - словом или цифрой; новых чисел не добавляй.
4. Текст в кавычках «...» остаётся дословно, в тех же кавычках.
5. Поле keep - группы вида «а|б|в»: в перефразе должно быть слово, которое начинается так же, как один из вариантов
   каждой группы.
6. Поле avoid - группы вида «а|б|в»: слов, которые начинаются так же, как варианты этих групп, в перефразе быть НЕ
   должно (это подсказка ответа). Не добавляй и других подробностей, которых нет в исходнике.
7. Если человек прямо просит действие («поищи в памяти», «найди в интернете», «запомни», «включи», «выключи»), прямая
   просьба остаётся просьбой того же действия - не заменяй её на «вспомни», «скажи» и т. п.
8. Это устная речь: без смайликов, без пояснений, одна реплика - одна строка.

Ответ - JSON: {"items": [{"id": "<id>", "paraphrases": ["...", "..."]}]} - ровно столько перефраз, сколько просят (n)."""


# ── Модель ────────────────────────────────────────────────────────────────────────────────────────────────────────


def client() -> Any:
    """Клиент DeepSeek (OpenAI-совместимый); ключ - только из окружения."""
    import os  # noqa: PLC0415

    from env_files import load_env_files  # noqa: PLC0415
    from openai import OpenAI  # noqa: PLC0415

    load_env_files(ENV_FILES)
    key = os.environ.get("DEEPSEEK_API_KEY", "")
    if not key:
        raise SystemExit("нет DEEPSEEK_API_KEY (scripts/dialogue/.env или окружение)")
    return OpenAI(api_key=key, base_url="https://api.deepseek.com", timeout=120)


class Asker:
    """Запрос JSON к модели с учётом вызовов и токенов; каждый ответ пишется сырым в ``raw`` (вне репозитория)."""

    def __init__(self, raw: Optional[Path], system: str, temperature: float) -> None:
        self._client = client()
        self._raw, self._system, self._temperature = raw, system, temperature
        self.calls = self.input_tokens = self.output_tokens = 0

    def ask(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        user = json.dumps(payload, ensure_ascii=False)
        for attempt in range(3):
            try:
                resp = self._client.chat.completions.create(
                    model=MODEL, temperature=self._temperature, response_format={"type": "json_object"},
                    messages=[{"role": "system", "content": self._system}, {"role": "user", "content": user}],
                    extra_body={"thinking": {"type": "disabled"}})
                break
            except Exception as exc:  # noqa: BLE001 — сеть/лимит: три попытки, потом громко
                if attempt == 2:
                    raise
                print(f"   повтор запроса: {type(exc).__name__}", flush=True)
                time.sleep(3)
        self.calls += 1
        self.input_tokens += resp.usage.prompt_tokens if resp.usage else 0
        self.output_tokens += resp.usage.completion_tokens if resp.usage else 0
        text = resp.choices[0].message.content or "{}"
        if self._raw is not None:
            self._raw.mkdir(parents=True, exist_ok=True)
            (self._raw / f"call_{self.calls:03d}.json").write_text(
                json.dumps({"request": payload, "response": text}, ensure_ascii=False, indent=1), encoding="utf-8")
        return json.loads(text)

    def usage(self) -> Dict[str, int]:
        return {"calls": self.calls, "input_tokens": self.input_tokens, "output_tokens": self.output_tokens}


def prompt_sha(system: str) -> str:
    return hashlib.sha256(system.encode("utf-8")).hexdigest()


# ── Шаги ──────────────────────────────────────────────────────────────────────────────────────────────────────────


def load_base() -> Dict[str, Any]:
    base = json.loads(build_marathon.CORPUS.read_text(encoding="utf-8"))
    if build_marathon.digest(base["acts"]) != base["corpus_sha256"]:
        raise SystemExit("marathon.json правили руками - сначала build_marathon.py")
    return base


def wake_table() -> List[str]:
    from dry_stand import WAKE_WORDS_YAML  # noqa: PLC0415
    from rob_box_dialog.address import wake_words  # noqa: PLC0415

    return list(wake_words(str(WAKE_WORDS_YAML))[0])


def plan(base: Dict[str, Any], wakes: Sequence[str]) -> Tuple[List[Dict[str, Any]], List[Dict[str, str]]]:
    """Оцениваемые шаги с ограничениями для модели и исключённые шаги с причиной."""
    groups, leaks = alt.anchors(base), alt.leak_groups(base)
    todo, excluded = [], []
    for entry in (s for a in base["acts"] for s in a["steps"] if s["scored"]):
        text = entry["step"]["text"]
        if not alt.norm(text).replace(alt.leading_wake(text, wakes), "", 1).strip():
            excluded.append({"id": entry["id"], "text": text, "reason": EXCLUDE_BARE_WAKE})
            continue
        words = re.findall(r"[\w-]+", text)
        todo.append({"id": entry["id"], "text": text, "wake": alt.leading_wake(text, wakes).capitalize(),
                     "names": [w for w in words if alt.names_in(w)], "numbers": [w for w in words if alt.numbers_in(w)],
                     "keep": groups[entry["id"]], "avoid": leaks[entry["id"]]})
    return todo, excluded


def problems_of(item: Dict[str, Any], cand: str, wakes: Sequence[str]) -> List[str]:
    """Автопроверка (:func:`alternatives.check_paraphrase`) и ручной отказ (:data:`MANUAL_REJECT`)."""
    problems = alt.check_paraphrase(item["text"], cand, wakes, item["keep"], item["avoid"])
    manual = MANUAL_REJECT.get((item["id"], cand))
    return problems + ([f"ручная проверка: {manual}"] if manual else [])


def accept(item: Dict[str, Any], candidates: Sequence[str], kept: List[str], wakes: Sequence[str],
           rejected: List[Dict[str, Any]]) -> None:
    """Перефразы, прошедшие проверку, - в ``kept`` (без повторов), остальные - в ``rejected`` с причиной."""
    for cand in candidates:
        cand = str(cand).strip()
        problems = problems_of(item, cand, wakes)
        if not problems and alt.norm(cand) in {alt.norm(k) for k in kept}:
            problems = ["повтор другой перефразы"]
        if problems:
            rejected.append({"id": item["id"], "text": cand, "problems": problems})
        elif len(kept) < WANT:
            kept.append(cand)


def generate(todo: Sequence[Dict[str, Any]], asker: Asker, wakes: Sequence[str], rounds: int,
             start: Optional[Dict[str, List[str]]] = None) -> Tuple[Dict[str, List[str]], List[Dict[str, Any]]]:
    """Добрать до :data:`WANT` перефраз на шаг; ``start`` - уже принятые (``--top-up``), они снова проходят проверку."""
    kept: Dict[str, List[str]] = {item["id"]: [] for item in todo}
    rejected: List[Dict[str, Any]] = []
    for item in todo:
        accept(item, (start or {}).get(item["id"], []), kept[item["id"]], wakes, rejected)
    for round_no in range(1, rounds + 1):
        need = [item for item in todo if len(kept[item["id"]]) < WANT]
        if not need:
            break
        print(f"круг {round_no}: шагов без полного набора {len(need)}", flush=True)
        for offset in range(0, len(need), BATCH):
            ask_batch(need[offset:offset + BATCH], kept, rejected, asker, wakes)
    return kept, rejected


def ask_batch(batch: Sequence[Dict[str, Any]], kept: Dict[str, List[str]], rejected: List[Dict[str, Any]],
              asker: Asker, wakes: Sequence[str]) -> None:
    """Один запрос к модели на пачку шагов: сколько ещё нужно, что уже принято, что отвергнуто и почему."""
    payload = {"items": [{**item, "n": WANT - len(kept[item["id"]]), "already": kept[item["id"]],
                          "rejected": [r for r in rejected if r["id"] == item["id"]][-3:]} for item in batch]}
    answer = asker.ask(payload)
    by_id = {str(row.get("id")): row.get("paraphrases") or [] for row in answer.get("items") or []}
    for item in batch:
        accept(item, by_id.get(item["id"], []), kept[item["id"]], wakes, rejected)


# ── Файл ──────────────────────────────────────────────────────────────────────────────────────────────────────────


def past_runs(previous: Optional[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """История сборок прошлого файла; у первой сборки (до ``--top-up``) истории нет - её строка из полей ``method``."""
    method = (previous or {}).get("method") or {}
    if not previous or method.get("history"):
        return list(method.get("history") or [])
    return [{"generated_utc": method.get("generated_utc"), "prompt_sha256": method.get("prompt_sha256"),
             "rounds": method.get("rounds"), "kept_from_previous": 0,
             "rejected": f"{method.get('rejected_by_checks')} (тексты не сохранялись)",
             **{k: method.get(k, 0) for k in ("calls", "input_tokens", "output_tokens")}}]


def build(raw: Optional[Path], rounds: int, previous: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Собрать часть; ``previous`` (``--top-up``) - прошлый файл: его перефразы снова проверяются, модель дописывает
    только недостающие, история сборок - в ``method.history``."""
    base, wakes = load_base(), wake_table()
    todo, excluded = plan(base, wakes)
    asker = Asker(raw, SYSTEM, TEMPERATURE)
    start = {sid: [r["text"] for r in rows] for sid, rows in (previous or {}).get("alternatives", {}).items()}
    kept, rejected = generate(todo, asker, wakes, rounds, start)
    texts = {t["id"]: t["text"] for t in todo}
    excluded += [{"id": i, "text": texts[i], "reason": f"после {rounds} кругов автопроверку не прошла ни одна перефраза"}
                 for i, rows in kept.items() if not rows]
    run = {"generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S"), "prompt_sha256": prompt_sha(SYSTEM),
           "rounds": rounds, "kept_from_previous": sum(len(v) for v in start.values()),
           "rejected": [{k: r[k] for k in ("id", "text", "problems")} for r in rejected], **asker.usage()}
    return part_file(base, kept, excluded, [*past_runs(previous), run])


def part_file(base: Dict[str, Any], kept: Dict[str, List[str]], excluded: List[Dict[str, Any]],
              history: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Содержимое ``paraphrases.json``: альтернативы, исключённые шаги, счётчики и история сборок."""
    alternatives = {i: [{"text": t} for t in rows] for i, rows in kept.items() if rows}
    sizes = [len(r) for r in alternatives.values()]
    return {"name": "understanding_paraphrases_part2", "schema": alt.SCHEMA, "part": PART, "base": alt.base_ref(base),
            "generator": "scripts/dialog/understanding_corpus/build_paraphrases.py",
            "method": {"model": MODEL, "temperature": TEMPERATURE, "manual_reject": len(MANUAL_REJECT),
                       **{k: sum(h[k] for h in history) for k in ("calls", "input_tokens", "output_tokens")},
                       "history": history},
            "counts": {"steps": len(alternatives), "alternatives": sum(sizes), "excluded": len(excluded),
                       **{f"steps_with_{n}": sizes.count(n) for n in (3, 2, 1)},
                       "steps_below_min": sum(s < MIN_KEEP for s in sizes)},
            "excluded": excluded, "alternatives": alternatives}


def check(path: Path) -> int:
    """Хеш и автопроверка каждой перефразы замороженного файла; код 1 - что-то не так."""
    base, wakes = load_base(), wake_table()
    data = alt.load_part(PART, base, path)
    items = {item["id"]: item for item in plan(base, wakes)[0]}
    bad = [(sid, row["text"], problems_of(items[sid], row["text"], wakes)) if sid in items
           else (sid, row["text"], ["шаг исключён из части 2"]) for sid, rows in data["alternatives"].items()
           for row in rows]
    bad = [b for b in bad if b[2]]
    for sid, text, problems in bad:
        print(f"НЕ ПРОШЛА {sid}: {text!r}: {problems}")
    print(f"часть {PART}: {data['counts']}, sha256 {data['corpus_sha256'][:12]}, не прошли автопроверку: {len(bad)}")
    return 1 if bad else 0


def drop_failing(data: Dict[str, Any], base: Dict[str, Any]) -> Dict[str, Any]:
    """Убрать из замороженной части перефразы, которые перестали проходить автопроверку, БЕЗ новых текстов и без сети (#4196:
    ожидания шага n306 выросли на «не узнал», и перефраза «…а кого так и не узнал?» подсказывает ответ). Остаётся подмножество
    измеренных перефраз; убранные - в ``method.dropped_rows`` (текст и причина). Шаг, у которого осталось меньше
    :data:`MIN_KEEP`, - ``SystemExit``: недостающее дописывает ``--top-up`` с моделью, это решает человек, а не молчаливая потеря."""
    wakes = wake_table()
    items = {item["id"]: item for item in plan(base, wakes)[0]}
    keep: Dict[str, List[Dict[str, Any]]] = {}
    dropped: List[Dict[str, Any]] = []
    for sid, rows in data["alternatives"].items():
        good = [r for r in rows if sid in items and not problems_of(items[sid], r["text"], wakes)]
        dropped += [{"id": sid, "text": r["text"], "problems": problems_of(items[sid], r["text"], wakes) if sid in items
                     else ["шаг исключён из части 2"]} for r in rows if r not in good]
        if len(good) < MIN_KEEP:
            raise SystemExit(f"{sid}: после снятия непрошедших осталось {len(good)} перефраз (< {MIN_KEEP}) - нужен --top-up")
        keep[sid] = good
    method = {**data["method"], "dropped_rows": [*data["method"].get("dropped_rows", []), *dropped]}
    counts = alt.recount(data["counts"], keep, data["excluded"])
    counts.update({f"steps_with_{n}": sum(len(r) == n for r in keep.values()) for n in (3, 2, 1)})
    counts["steps_below_min"] = sum(len(r) < MIN_KEEP for r in keep.values())
    return {**data, "method": method, "counts": counts, "alternatives": keep}


def sample(path: Path, n: int) -> List[Tuple[str, str, str]]:
    """N случайных пар «исходник → перефраза» (зерно :data:`SEED`) для ручной проверки."""
    base = load_base()
    data = alt.load_part(PART, base, path)
    index = alt.index_of(base)
    pairs = [(sid, index[sid]["step"]["text"], row["text"]) for sid, rows in data["alternatives"].items()
             for row in rows]
    return random.Random(SEED).sample(pairs, min(n, len(pairs)))


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="сверить замороженный файл, без сети")
    ap.add_argument("--sample", type=int, default=0, help="напечатать N случайных пар исходник → перефраза")
    ap.add_argument("--raw", default="", help="каталог сырых ответов модели (вне репозитория)")
    ap.add_argument("--rounds", type=int, default=4, help="кругов замены не прошедших автопроверку")
    ap.add_argument("--top-up", action="store_true", help="взять перефразы из файла, перепроверить и дописать "
                                                          "недостающие (после правки MANUAL_REJECT)")
    ap.add_argument("--repoint-from", default="", help="старый marathon.json: перевесить файл на текущую часть 1 без "
                                                       "новых текстов (alternatives.repoint)")
    ap.add_argument("--drop-failing", action="store_true", help="убрать перефразы, переставшие проходить автопроверку "
                                                                "(без сети и новых текстов)")
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args(argv)
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    if args.drop_failing:
        base = load_base()
        out = Path(args.out)
        data = drop_failing(alt.load_part(PART, base, out), base)
        before = json.loads(out.read_text(encoding="utf-8"))["corpus_sha256"]
        sha = alt.write_part(out, data)
        print(f"часть {PART}: убрано непрошедших {len(data['method']['dropped_rows'])}, {data['counts']}, sha256 {sha[:12]} "
              f"(было {before[:12]})")
        return 0
    if args.repoint_from:
        alt.repoint_file(PART, Path(args.out), Path(args.repoint_from), load_base())
        return 0
    if args.check:
        return check(Path(args.out))
    if args.sample:
        print("| # | шаг | исходник | перефраза |\n|---|---|---|---|")
        for n, (sid, src, para) in enumerate(sample(Path(args.out), args.sample), 1):
            print(f"| {n} | {sid} | {src} | {para} |")
        return 0
    previous = json.loads(Path(args.out).read_text(encoding="utf-8")) if args.top_up else None
    data = build(Path(args.raw) if args.raw else None, args.rounds, previous)
    sha = alt.write_part(Path(args.out), data)
    print(f"записан {args.out}: {data['counts']} sha256={sha[:12]}; модель {data['method']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
