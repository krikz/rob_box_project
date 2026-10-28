#!/usr/bin/env python3
"""Корпус замера понимания, часть 4: английские фразы марафона (карточка #4163, этап 2).

    python scripts/dialog/understanding_corpus/build_english.py --raw <каталог вне репо>   # собрать (DeepSeek)
    python scripts/dialog/understanding_corpus/build_english.py --check                    # сверить файл, без сети
    python scripts/dialog/understanding_corpus/build_english.py --list                     # выбор шагов и причины

Берутся оцениваемые шаги, чьё ожидание держится на тулах и строках лога, а не на русских словах в ответе робота
(:func:`choose`): ждут тул (``expected_tool_calls``), строку лога (``patterns`` кроме общих «речь началась» / «ход
завершён») или бэклог (``expect: backlog``); нет ``expected_keywords`` и запретов на слова ответа (``must_not_say``
кроме общих фраз отказа и фраз кода). Ожидание шага то же; русские фразы кода (отказы, «Включаю музыку.») робот говорит
по-русски на любом языке реплики, поэтому их запреты остаются честными.

Вейк. В таблице вейков робота (``docker/sound/config/wake_words.yaml``) английского «Robot» нет: с ним ни один шаг не
адресован ни в одном варианте, и часть меряла бы адресацию, а не понимание. Взято имя робота латиницей - ``Robbox``
(в таблице есть). Перевод - модель (DeepSeek), проверка - ``alternatives.check_translation``: вейк, имена латиницей,
числа, нет кириллицы. Сырые ответы - в ``--raw`` (вне репозитория).
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))

import alternatives as alt  # noqa: E402
import build_paraphrases as bp  # noqa: E402

PART = 4
OUT = alt.part_path(PART)
TEMPERATURE = 0.3
#: Общие паттерны: «речь началась», «ход завершён» - на тулах и смысле не держатся.
GENERIC_PATTERNS = frozenset({"SpeechEvent started speech_id", "📊 \\[turn\\] .*stage=done status=finished"})
#: Запреты ответа, которые держатся на русских словах содержания (а не на фразах кода): шаг не берём.
CONTENT_FORBIDDEN = {"10/n1007_no_leak": "must_not_say - русские слова прошлых тем («по именам», «космос», «китайск»)"}
#: Шаги, выбранные правилом, но не переводимые по смыслу.
NOT_TRANSLATABLE = {
    "1/n106_alt_wake_roboks": "шаг про второе русское имя-вейк «Робокс»",
    "8/n805b_unsilence": "ожидание - строка Tier-1 «unsilence», её в обоих вариантах даёт только код по русскому «говори»; "
                         "провал оставляет робота в тишине до конца марафона и валит шаги акта 9 этой части"}

SYSTEM = f"""You translate utterances of people in a workshop, spoken aloud to a voice robot, from Russian into
natural spoken English - the way a native speaker would say the same thing. Keep the meaning and the register, add and
drop nothing.

Hard rules:
1. Address. If the Russian starts with an address to the robot («Робот», «Робокс», «Робота» ...), the English starts
   with exactly "{alt.EN_WAKE}, " ("{alt.EN_WAKE}." if that is the whole utterance); every other address to the robot
   inside the utterance is "{alt.EN_WAKE}" too. If there is no address, do not add one and do not use the words robot,
   robbox or rob box.
2. Names in Latin letters: Саша - Sasha, Борис - Boris, Гриша - Grisha (тётя Гриша - Aunt Grisha), Валера - Valera,
   Алёна - Alyona, Джейн - Jane, Спартак - Spartak (football club), енот - raccoon.
3. Numbers stay the same numbers (words or digits); add no other numbers.
4. No Cyrillic in the answer; translate quotes «...» into English inside double quotes.

Answer JSON: {{"items": [{{"id": "<id>", "en": "..."}}]}}"""


def holds_on(step: Dict[str, Any]) -> List[str]:
    """На чём держится ожидание шага, кроме слов ответа: тулы, строки лога (кроме общих), бэклог."""
    acc = step.get("acceptance") or {}
    tools = [t for t in acc.get("expected_tool_calls") or [] if t]
    patterns = [p for p in step.get("patterns") or [] if p not in GENERIC_PATTERNS]
    return [*([f"тулы {tools}"] if tools else []), *([f"паттерны лога ({len(patterns)})"] if patterns else []),
            *(["бэклог"] if step.get("expect") == "backlog" else [])]


def refusal(entry: Dict[str, Any]) -> str:
    """Почему шаг не идёт в часть 4 («» - идёт)."""
    if (entry["step"].get("acceptance") or {}).get("expected_keywords"):
        return "ожидание держится на русских словах ответа (expected_keywords)"
    sid = entry["id"]
    if sid in CONTENT_FORBIDDEN or sid in NOT_TRANSLATABLE:
        return CONTENT_FORBIDDEN.get(sid) or NOT_TRANSLATABLE[sid]
    if not holds_on(entry["step"]):
        return "ожидание - только «ход завершён и робот что-то сказал»: на тулах и логе не держится"
    return ""


def choose(base: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], List[Dict[str, str]]]:
    """Шаги части 4 и отказы с причиной (по всем оцениваемым шагам части 1)."""
    generic = set(alt.generic_groups(base))
    chosen, refused = [], []
    for entry in (s for a in base["acts"] for s in a["steps"] if s["scored"]):
        why = refusal(entry)
        if why:
            refused.append({"id": entry["id"], "reason": why})
            continue
        forbidden = [g for g in (entry["step"].get("acceptance") or {}).get("must_not_say") or [] if g not in generic]
        chosen.append({"id": entry["id"], "text": entry["step"]["text"], "why": ", ".join(holds_on(entry["step"])),
                       "code_phrases_forbidden": forbidden})
    return chosen, refused


def translate(todo: Sequence[Dict[str, Any]], asker: bp.Asker, wakes: Sequence[str], rounds: int,
              start: Optional[Dict[str, str]] = None) -> Tuple[Dict[str, str], List[Dict[str, Any]]]:
    """Перевести шаги; ``start`` - уже готовые (``--top-up``): берутся, если снова проходят проверку."""
    done = {t["id"]: start[t["id"]] for t in todo if t["id"] in (start or {})
            and not alt.check_translation(t["text"], start[t["id"]], wakes)}
    rejected: List[Dict[str, Any]] = []
    for round_no in range(1, rounds + 1):
        need = [t for t in todo if t["id"] not in done]
        if not need:
            break
        print(f"круг {round_no}: без перевода {len(need)}", flush=True)
        for offset in range(0, len(need), 15):
            translate_batch(need[offset:offset + 15], done, rejected, asker, wakes)
    return done, rejected


def translate_batch(batch: Sequence[Dict[str, Any]], done: Dict[str, str], rejected: List[Dict[str, Any]],
                    asker: bp.Asker, wakes: Sequence[str]) -> None:
    """Один запрос на пачку: прошедший проверку перевод - в ``done``, остальные - в ``rejected`` с причиной."""
    answer = asker.ask({"items": [{"id": t["id"], "ru": t["text"],
                                   "rejected": [r for r in rejected if r["id"] == t["id"]][-2:]} for t in batch]})
    got = {str(r.get("id")): str(r.get("en") or "").strip() for r in answer.get("items") or []}
    for item in batch:
        problems = alt.check_translation(item["text"], got.get(item["id"], ""), wakes)
        if problems:
            rejected.append({"id": item["id"], "en": got.get(item["id"], ""), "problems": problems})
        else:
            done[item["id"]] = got[item["id"]]


def build(raw: Optional[Path], rounds: int, previous: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    base, wakes = bp.load_base(), bp.wake_table()
    chosen, refused = choose(base)
    asker = bp.Asker(raw, SYSTEM, TEMPERATURE)
    start = {sid: rows[0]["text"] for sid, rows in (previous or {}).get("alternatives", {}).items()}
    done, rejected = translate(chosen, asker, wakes, rounds, start)
    past = (previous or {}).get("method") or {}
    usage = {k: asker.usage()[k] + int(past.get(k, 0)) for k in ("calls", "input_tokens", "output_tokens")}
    refused += [{"id": t["id"], "reason": f"перевод не прошёл проверку за {rounds} круга"} for t in chosen
                if t["id"] not in done]
    alternatives = {t["id"]: [{"text": done[t["id"]], "why": t["why"],
                               "code_phrases_forbidden": t["code_phrases_forbidden"]}] for t in chosen
                    if t["id"] in done}
    return {"name": "understanding_english_part4", "schema": alt.SCHEMA, "part": PART, "base": alt.base_ref(base),
            "generator": "scripts/dialog/understanding_corpus/build_english.py",
            "method": {"model": bp.MODEL, "temperature": TEMPERATURE, "prompt_sha256": bp.prompt_sha(SYSTEM),
                       "wake": alt.EN_WAKE, "generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S"),
                       "rounds": rounds, "top_up": previous is not None, "kept_from_previous": len(start),
                       "rejected": [*past.get("rejected", []), *rejected], **usage},
            "counts": {"steps": len(alternatives), "alternatives": len(alternatives), "excluded": len(refused)},
            "excluded": refused, "alternatives": alternatives}


def check(path: Path) -> int:
    base, wakes = bp.load_base(), bp.wake_table()
    data = alt.load_part(PART, base, path)
    index = alt.index_of(base)
    chosen = {t["id"] for t in choose(base)[0]}
    bad = [(sid, row["text"], alt.check_translation(index[sid]["step"]["text"], row["text"], wakes)
            + ([] if sid in chosen else ["шаг не проходит правило выбора части 4"]))
           for sid, rows in data["alternatives"].items() for row in rows]
    bad = [b for b in bad if b[2]]
    for sid, text, problems in bad:
        print(f"НЕ ПРОШЁЛ {sid}: {text!r}: {problems}")
    print(f"часть {PART}: {data['counts']}, sha256 {data['corpus_sha256'][:12]}, не прошли проверку: {len(bad)}")
    return 1 if bad else 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="сверить замороженный файл, без сети")
    ap.add_argument("--list", action="store_true", help="напечатать выбор шагов и причины, без сети")
    ap.add_argument("--raw", default="", help="каталог сырых ответов модели (вне репозитория)")
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--top-up", action="store_true", help="взять переводы из файла, перепроверить и дописать недостающие")
    ap.add_argument("--repoint-from", default="", help="старый marathon.json: перевесить файл на текущую часть 1 без "
                                                       "новых текстов (alternatives.repoint)")
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args(argv)
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    if args.repoint_from:
        alt.repoint_file(PART, Path(args.out), Path(args.repoint_from), bp.load_base())
        return 0
    if args.check:
        return check(Path(args.out))
    if args.list:
        chosen, refused = choose(bp.load_base())
        print(f"выбрано {len(chosen)}:")
        for t in chosen:
            print(f"  + {t['id']}: {t['why']}" + (f"; запреты-фразы кода: {t['code_phrases_forbidden']}"
                                                  if t["code_phrases_forbidden"] else ""))
        print(f"не выбрано {len(refused)}:")
        for r in refused:
            print(f"  - {r['id']}: {r['reason']}")
        return 0
    previous = json.loads(Path(args.out).read_text(encoding="utf-8")) if args.top_up else None
    data = build(Path(args.raw) if args.raw else None, args.rounds, previous)
    sha = alt.write_part(Path(args.out), data)
    print(f"записан {args.out}: {data['counts']} sha256={sha[:12]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
