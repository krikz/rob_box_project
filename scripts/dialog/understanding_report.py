"""Отчёт замера «кто понимает фразу» (карточка #4163): метрики по частям корпуса, правило решения Шифу, markdown.

Числа считает :mod:`understanding_metrics` по отчётам шагов стенда; здесь - сборка по частям и вариантам, правило
решения, заданное ДО прогона, и текст отчёта. Вердикта «принимаем» здесь нет: правило считается механически, решает Шифу.

Правило (карточка #4163, допуски Шифу 28.10): вариант V принимаем, если выполнено всё -
  * на фразах марафона (часть 1) V хуже A не больше чем на :data:`MAX_PASS_DROP_PP` п.п.;
  * на перефразах (часть 2) и искажениях STT (часть 3) V лучше A (доля PASS строго выше);
  * опасных ошибок у V не больше, чем у A (по всем частям прогона);
  * медиана хода V растёт над A не больше чем на X с (``--max-latency-growth-s``, по всем частям прогона).
Часть 4 (английский) в правиле не стоит - справочно.
"""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional, Sequence

import dry_marathon as dm
import understanding_metrics as um
import understanding_variants as uv
from alternatives import TITLES

#: Решение Шифу 28.10: на фразах марафона вариант хуже A не больше чем на 3 п.п.
MAX_PASS_DROP_PP = 3.0
#: Решение Шифу 28.10 (Telegram, 17:34 UTC): «до 3-х секунд» - медиана хода варианта растёт относительно A не больше X с.
MAX_LATENCY_GROWTH_S = 3.0
#: Допуск расхождения вердиктов A с живым марафоном (поручение координатора): больше - вывод по вариантам не подаётся.
MAX_LIVE_DISAGREE = 0.10
#: Части, где правило требует «вариант лучше A».
BETTER_PARTS = (2, 3)
JUDGED = (dm.PASS, dm.FAIL)


# ── Сборка чисел ──────────────────────────────────────────────────────────────────────────────────────────────────


def step_record(act: str, report: dm.StepReport, entry: Dict[str, Any]) -> Dict[str, Any]:
    return {"id": um.key(act, report), "text": report.text, "kind": report.kind, "scored": entry["scored"],
            "verdict": report.verdict, "first_verdict": report.first_verdict, "attempts": report.attempts,
            "fail": report.fail, "tools": report.tools, "said": report.said, "llm_calls": report.llm_calls,
            "turns": report.turns}


def jev_entry(runs: Sequence[Any], items: Sequence[um.Item], groups_file: str,
              price: Callable[[Dict[str, Any]], Dict[str, Any]]) -> Dict[str, Any]:
    """Строка бюджета Jev варианта C: вызовы, сбои, отказы по бюджету Jev, токены, цена, совпадение группы."""
    routers = [r.router for r in runs if r.router]
    jev = {k: sum(x[k] for x in routers) for k in ("calls", "errors", "input_tokens", "output_tokens")}
    refused = sum(1 for _, _, rep in items for t in rep.turns if (t.get("route") or {}).get("error") == "jev_budget")
    return {**jev, "budget_refusals": refused, "threshold": routers[0]["threshold"] if routers else None,
            "groups_file": groups_file, "price": price(jev), "route_agreement": um.route_agreement(items)}


def variant_entry(runs: Sequence[Any], index: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    return {"metrics": um.variant_metrics([r.reports for r in runs], index),
            "llm_calls_total": sum(r.llm_calls for r in runs),
            "elapsed_s": [round(r.elapsed_s, 1) for r in runs], "caps": [r.cap for r in runs],
            "runs_planned": len(runs), "runs_done": sum(not r.skipped for r in runs),
            "runs": [[step_record(a, rep, index[um.key(a, rep)]) for a, reps in r.reports.items() for rep in reps
                      if um.key(a, rep) in index] for r in runs]}


def part_entry(part: int, results: Sequence[Any], index: Dict[str, Dict[str, Any]], groups_file: str,
               price: Callable[[Dict[str, Any]], Dict[str, Any]], max_latency_growth_s: float) -> Dict[str, Any]:
    variants: Dict[str, Any] = {}
    for variant in dict.fromkeys(r.variant for r in results):
        mine = [r for r in results if r.variant == variant]
        variants[variant] = variant_entry(mine, index)
        if variant == uv.C:
            items = [i for r in mine for i in um.scored_items(r.reports, index)]
            variants[variant]["jev"] = jev_entry(mine, items, groups_file, price)
    return {"title": TITLES[part], "variants": variants, "paired": paired(variants),
            "decision_rule": decision_rule(variants, max_latency_growth_s, part)}


def paired(variants: Dict[str, Any]) -> Dict[str, Dict[str, int]]:
    """Парное сравнение с A по одинаковым (прогон, шаг), где оба вердикта PASS/FAIL: V выиграл / проиграл / вничью."""
    base = variants.get(uv.A)
    out: Dict[str, Dict[str, int]] = {}
    if base is None:
        return out
    for name, entry in variants.items():
        if name == uv.A:
            continue
        tally = {"won": 0, "lost": 0, "same": 0}
        for ref_run, my_run in zip(base["runs"], entry["runs"]):
            ref = {s["id"]: s["verdict"] for s in ref_run if s["scored"]}
            for step in my_run:
                theirs = ref.get(step["id"])
                if not step["scored"] or theirs not in JUDGED or step["verdict"] not in JUDGED:
                    continue
                tally["same" if theirs == step["verdict"] else "won" if step["verdict"] == dm.PASS else "lost"] += 1
        out[name] = tally
    return out


# ── Правило решения ───────────────────────────────────────────────────────────────────────────────────────────────


def _pp(a: Optional[float], b: Optional[float]) -> Optional[float]:
    return None if a is None or b is None else (a - b) * 100.0


def _growth_ms(mine: Optional[float], ref: Optional[float]) -> Optional[float]:
    """Рост медианы хода варианта над A, мс; медианы нет хоть у одного - не измерено (``None``), а не «0 мс»."""
    return None if mine is None or ref is None else mine - ref


def decision_rule(variants: Dict[str, Any], max_latency_growth_s: float = MAX_LATENCY_GROWTH_S,
                  part: int = 1) -> Dict[str, Any]:
    """Критерии правила для одной части, что вообще можно проверить на ней: часть 1 - потеря PASS ≤ 3 п.п.; части 2-3 -
    доля PASS строго выше A; часть 4 - разница справочно. Опасные и медиана хода - на каждой части. Медиана: рост над A
    не больше ``max_latency_growth_s`` - ``True``, больше - ``False``, медианы нет - ``None``."""
    base = variants.get(uv.A)
    out: Dict[str, Any] = {}
    for name in (uv.B, uv.BN, uv.C):
        if base is None or name not in variants:
            continue
        mine, ref = variants[name]["metrics"], base["metrics"]
        drop = _pp(ref["pass"]["share"], mine["pass"]["share"])
        d_lat = _growth_ms(mine["latency"]["median_ms"], ref["latency"]["median_ms"])
        rule = {"part": part, "pass_drop_pp": drop,
                "danger_A_B": [ref["danger"]["total"], mine["danger"]["total"]],
                "danger_ok": mine["danger"]["total"] <= ref["danger"]["total"],
                "median_latency_growth_ms": d_lat, "median_latency_limit_s": max_latency_growth_s,
                "median_latency_ok": None if d_lat is None else d_lat <= max_latency_growth_s * 1000.0}
        if part == 1:
            rule["pass_drop_ok"] = None if drop is None else drop <= MAX_PASS_DROP_PP
        elif part in BETTER_PARTS:
            rule["better_than_A"] = None if drop is None else drop < 0
        out[name] = rule
    return out


def overall_rule(parts: Dict[int, Any], pooled: Dict[str, Dict[str, Any]], max_latency_growth_s: float
                 ) -> Dict[str, Any]:
    """Правило целиком по частям прогона. ``pooled`` - метрики варианта по оцениваемым шагам ВСЕХ частей (опасные и
    медиана хода). Нет части, нужной правилу, - условие «не измерено» (``None``), и всё правило не выполнено."""
    out: Dict[str, Any] = {}
    for name in pooled:
        if name == uv.A:
            continue
        rule1 = parts.get(1, {}).get("decision_rule", {}).get(name, {})
        better = {p: parts.get(p, {}).get("decision_rule", {}).get(name, {}).get("better_than_A") for p in BETTER_PARTS}
        ref, mine = pooled[uv.A], pooled[name]
        d_lat = _growth_ms(mine["latency"]["median_ms"], ref["latency"]["median_ms"])
        checks = {"part1_pass_drop_ok": rule1.get("pass_drop_ok"),
                  **{f"part{p}_better": better[p] for p in BETTER_PARTS},
                  "danger_ok": mine["danger"]["total"] <= ref["danger"]["total"],
                  "median_latency_ok": None if d_lat is None else d_lat <= max_latency_growth_s * 1000.0}
        out[name] = {**checks, "danger_A_V": [ref["danger"]["total"], mine["danger"]["total"]],
                     "median_latency_growth_ms": d_lat, "all_met": all(v is True for v in checks.values())}
    return out


def pooled_metrics(results: Sequence[Any], indexes: Dict[int, Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    """Опасные и время хода варианта по оцениваемым шагам всех частей прогона вместе."""
    out: Dict[str, Dict[str, Any]] = {}
    for variant in dict.fromkeys(r.variant for r in results):
        items = [i for r in results if r.variant == variant for i in um.scored_items(r.reports, indexes[r.part])]
        out[variant] = {"danger": um.danger(items), "latency": um.latency(items)}
    return out


def build_report(results: Sequence[Any], indexes: Dict[int, Dict[str, Any]], meta: Dict[str, Any],
                 live: Optional[Dict[str, Any]], groups_file: str,
                 price: Callable[[Dict[str, Any]], Dict[str, Any]]) -> Dict[str, Any]:
    growth = meta["max_latency_growth_s"]
    parts = {p: part_entry(p, [r for r in results if r.part == p], indexes[p], groups_file, price, growth)
             for p in sorted(dict.fromkeys(r.part for r in results))}
    pooled = pooled_metrics(results, indexes)
    report = {"meta": meta, "parts": {str(p): v for p, v in parts.items()},
              "overall_rule": overall_rule(parts, pooled, growth),
              "pooled": {v: {"danger_total": m["danger"]["total"], "median_ms": m["latency"]["median_ms"]}
                         for v, m in pooled.items()},
              "skipped": [{"variant": r.variant, "part": r.part, "run": r.run} for r in results if r.skipped]}
    if live is not None:
        report["live"] = live
    return report


# ── Markdown ──────────────────────────────────────────────────────────────────────────────────────────────────────


def pct(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value * 100:.1f} %"


def num(value: Optional[float], digits: int = 0) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def signed_ms(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value:+.0f} мс"


def _row(label: str, variants: Dict[str, Any], cell: Callable[[Dict[str, Any]], str]) -> str:
    return f"| {label} | " + " | ".join(cell(v) for v in variants.values()) + " |"


def _stability(m: Dict[str, Any]) -> str:
    st = m["stability"]
    return pct(st["same_verdict_share"]) + f" ({st['steps']} шагов)" if st["same_verdict_share"] is not None \
        else st["note"]


def metrics_table(variants: Dict[str, Any]) -> List[str]:
    heads = list(variants)
    lines = ["| метрика | " + " | ".join(heads) + " |", "|---|" + "---|" * len(heads)]
    m = lambda v: v["metrics"]  # noqa: E731
    lines += [
        _row("прогонов сделано / задумано", variants, lambda v: f"{v['runs_done']} / {v['runs_planned']}"),
        _row("1. оцениваемых шагов (PASS / FAIL / не оценено / NOT_RUN)", variants, lambda v: "{} / {} / {} / {} / {}"
             .format(m(v)["pass"]["steps"], m(v)["pass"]["PASS"], m(v)["pass"]["FAIL"], m(v)["pass"]["NOT_EVALUATED"],
                     m(v)["pass"]["NOT_RUN"])),
        _row("1. доля PASS (из PASS+FAIL)", variants, lambda v: pct(m(v)["pass"]["share"])),
        _row("1. доля PASS с первой попытки", variants, lambda v: pct(m(v)["pass"]["first_attempt_share"])),
        _row("2. опасных ошибок всего", variants, lambda v: str(m(v)["danger"]["total"])),
    ]
    for kind, label in (("false_stop", "ложный стоп"), ("motion_instead_of_look", "движение вместо взгляда"),
                        ("delete_without_confirm", "удаление без подтверждения"),
                        ("claimed_without_tool", "«сделал» без исполненного тула")):
        lines.append(_row(f"2.  - {label}", variants, lambda v, k=kind: str(m(v)["danger"][k]["count"]) + (
            "" if m(v)["danger"][k]["measurable"] else " (в корпусе не измеряется)")))
    lines += [
        _row("3. размер среза (медиана / p90 / макс), тулов", variants, lambda v: "{} / {} / {}".format(
            num(m(v)["slice"]["size"]["median"]), num(m(v)["slice"]["size"]["p90"]), num(m(v)["slice"]["size"]["max"]))),
        _row("3. ожидаемый тул: вызван / нет в срезе / мост добавил / в срезе, но выбран другой", variants,
             lambda v: "{called} / {missing} / {rescued} / {wrong_choice}".format(**m(v)["slice"]["expected_tool_checks"])),
        _row("4. время хода, мс (медиана / p90)", variants,
             lambda v: f"{num(m(v)['latency']['median_ms'])} / {num(m(v)['latency']['p90_ms'])}"),
        _row("4. доля ходов без LLM", variants, lambda v: pct(m(v)["latency"]["without_llm_share"])),
        _row("5. вызовов LLM на ход", variants, lambda v: num(m(v)["llm"]["calls_per_turn"], 2)),
        _row("5. токенов на ход с LLM", variants, lambda v: num(m(v)["llm"]["tokens_per_llm_turn"])
             if m(v)["llm"]["tokens_reported"] else "n/a (провайдер не прислал usage)"),
        _row("6. устойчивость (тот же вердикт во всех прогонах)", variants, lambda v: _stability(m(v))),
        _row("вызовов LLM за прогоны, всего", variants, lambda v: str(v["llm_calls_total"])),
    ]
    return lines


def act_table(variants: Dict[str, Any]) -> List[str]:
    acts = list(dict.fromkeys(a for v in variants.values() for a in v["metrics"]["by_act"]))
    heads = list(variants)
    lines = ["| акт | " + " | ".join(f"{h}: PASS/FAIL" for h in heads) + " |", "|---|" + "---|" * len(heads)]
    for act in acts:
        cells = []
        for v in variants.values():
            row = v["metrics"]["by_act"].get(act)
            cells.append(f"{row['PASS']}/{row['FAIL']}" if row else "-")
        lines.append(f"| {act} | " + " | ".join(cells) + " |")
    return lines


def disagreements(variants: Dict[str, Any]) -> List[str]:
    """Шаги, где вердикты вариантов разошлись (по первому прогону): что разное - то и надо читать глазами."""
    base = list(variants)[0]
    lines: List[str] = []
    first = {name: {s["id"]: s for s in (v["runs"][0] if v["runs"] else []) if s["scored"]}
             for name, v in variants.items()}
    for item_id, step in first[base].items():
        verdicts = {name: first[name].get(item_id, {}).get("verdict", "-") for name in variants}
        if len(set(verdicts.values())) > 1:
            why = "; ".join(f"{n}: {first[n][item_id]['fail'][0][:90]}" for n in variants
                            if item_id in first[n] and first[n][item_id]["fail"])
            lines.append(f"| {item_id} | {step['text'][:70]} | " + " | ".join(verdicts.values()) + f" | {why} |")
    head = ["| шаг | текст | " + " | ".join(variants) + " | чем не прошёл |",
            "|---|---|" + "---|" * len(variants) + "---|"]
    return head + lines if lines else ["(вердикты вариантов совпали на всех шагах)"]


def danger_lines(variants: Dict[str, Any]) -> List[str]:
    out = []
    for name, v in variants.items():
        for kind, found in v["metrics"]["danger"].items():
            if isinstance(found, dict) and found["count"]:
                out.append(f"- {name} / {kind}: {', '.join(found['steps'])}")
    return out or ["(нет)"]


def summary_table(report: Dict[str, Any]) -> List[str]:
    """Главная таблица: A против остальных вариантов по частям."""
    lines = ["| часть | вариант | шагов (PASS/FAIL) | доля PASS | Δ к A, п.п. | опасных | медиана хода, мс | "
             "устойчивость | пары с A: выиграл / проиграл / вничью |", "|---|---|---|---|---|---|---|---|---|"]
    for key, part in report["parts"].items():
        base = part["variants"].get(uv.A)
        for name, v in part["variants"].items():
            m = v["metrics"]
            delta = "-" if name == uv.A or base is None else num(_pp(m["pass"]["share"],
                                                                     base["metrics"]["pass"]["share"]), 1)
            pair = part["paired"].get(name)
            lines.append(f"| {key}. {part['title']} | {name} | {m['pass']['PASS']}/{m['pass']['FAIL']} | "
                         f"{pct(m['pass']['share'])} | {delta} | {m['danger']['total']} | "
                         f"{num(m['latency']['median_ms'])} | {_stability(m)} | "
                         f"{'-' if pair is None else '{won} / {lost} / {same}'.format(**pair)} |")
    return lines


def _pass_clause(rule: Dict[str, Any]) -> str:
    """Условие правила о доле PASS для части: допуск потери (часть 1), «лучше A» (части 2-3), справочно (часть 4)."""
    gain = None if rule["pass_drop_pp"] is None else 0.0 - rule["pass_drop_pp"]
    if "pass_drop_ok" in rule:
        return f"потеря PASS {num(rule['pass_drop_pp'], 1)} п.п. (допуск {MAX_PASS_DROP_PP:g}: {rule['pass_drop_ok']})"
    if "better_than_A" in rule:
        return f"PASS выше A на {num(gain, 1)} п.п. (лучше A: {rule['better_than_A']})"
    return f"PASS выше A на {num(gain, 1)} п.п. (справочно, в правиле нет)"


def rule_lines(report: Dict[str, Any]) -> List[str]:
    meta = report["meta"]
    out = [f"Допуски (решения Шифу 28.10): потеря доли PASS против A на фразах марафона - не больше "
           f"{MAX_PASS_DROP_PP:g} п.п.; на перефразах и искажениях STT - строго выше A; опасных не больше; рост "
           f"медианы хода против A - не больше {meta['max_latency_growth_s']:g} с.", ""]
    for key, part in report["parts"].items():
        for name, rule in part["decision_rule"].items():
            out.append(f"- часть {key} ({part['title']}), {name} против A: {_pass_clause(rule)}; опасных A/{name}: "
                       f"{rule['danger_A_B'][0]}/{rule['danger_A_B'][1]} ({rule['danger_ok']}); медиана хода "
                       f"{signed_ms(rule['median_latency_growth_ms'])} (допуск {rule['median_latency_limit_s']:g} с: "
                       f"{rule['median_latency_ok']}).")
    for name, rule in report.get("overall_rule", {}).items():
        out += ["", f"**Правило целиком, {name} против A (все части прогона):** часть 1 в допуске: "
                f"{rule['part1_pass_drop_ok']}; перефразы лучше A: {rule['part2_better']}; искажения STT лучше A: "
                f"{rule['part3_better']}; опасных A/{name}: {rule['danger_A_V'][0]}/{rule['danger_A_V'][1]} "
                f"({rule['danger_ok']}); медиана хода {signed_ms(rule['median_latency_growth_ms'])} "
                f"({rule['median_latency_ok']}). **Все условия выполнены: {rule['all_met']}** (решает Шифу)."]
    return out


def part_section(key: str, part: Dict[str, Any]) -> List[str]:
    v = part["variants"]
    return [f"## Часть {key} - {part['title']}", "", *metrics_table(v), "", "### По актам", "", *act_table(v), "",
            "### Где вердикты вариантов разошлись (прогон 1)", "", *disagreements(v), "",
            "### Опасные ошибки по шагам", "", *danger_lines(v), ""]


def jev_lines(report: Dict[str, Any]) -> List[str]:
    out, total = [], {"calls": 0, "errors": 0, "budget_refusals": 0, "input_tokens": 0, "output_tokens": 0, "usd": 0.0}
    for key, part in report["parts"].items():
        for name, v in part["variants"].items():
            if "jev" in v:
                j = v["jev"]
                out.append(f"- часть {key}, вариант {name}: вызовов Jev {j['calls']}, сбоев {j['errors']}, отказов по "
                           f"бюджету Jev {j['budget_refusals']}, токены вход/выход {j['input_tokens']}/"
                           f"{j['output_tokens']}, порог {j['threshold']}, файл групп `{j['groups_file']}`; цена "
                           f"{j['price']}; выбор Jev = группа ожидаемого тула: {j['route_agreement']}.")
                for k in ("calls", "errors", "budget_refusals", "input_tokens", "output_tokens"):
                    total[k] += j[k]
                total["usd"] += j["price"].get("usd") or 0.0
    if out:
        out.insert(0, f"- Jev всего: вызовов {total['calls']}, сбоев {total['errors']}, отказов по бюджету Jev "
                      f"{total['budget_refusals']}, токены вход/выход {total['input_tokens']}/{total['output_tokens']}, "
                      f"цена ${total['usd']:.4f} (по PRICE_PER_MTOK jev_eval.py, НЕ СВЕРЕНО с тарифом).")
    return out


def render(report: Dict[str, Any]) -> str:
    meta = report["meta"]
    stage = 1 if [str(p) for p in meta["parts"]] == ["1"] else 2
    out = [f"# Замер понимания фразы, этап {stage} ({meta['generated_utc']} UTC)", "",
           f"Пометка: **{meta['note']}**. Провайдер: {meta['providers']}, варианты {meta['variants']}, прогонов "
           f"{meta['runs']}, бюджет LLM {meta['budget']}, бюджет Jev {meta['jev_budget']}.", ""]
    if meta["fake"]:
        out += [f"**Фейковый провайдер (`{meta['fake']}`): проверка обвязки, цифры не характеризуют ни код, ни модель.**",
                ""]
    out += ["Корпус:", ""] + [f"- часть {c['part']} ({c['title']}): `{c['name']}` sha256 `{c['sha256'][:12]}`, "
                              f"{c['counts']}" for c in meta["corpus"]]
    out += ["", meta["rotation"], "", "## Главная таблица: варианты по частям", "", *summary_table(report), ""]
    if report["skipped"]:
        out += ["Не запущено (бюджет LLM кончился до прогона): " + ", ".join(
            f"{s['variant']} часть {s['part']} прогон {s['run']}" for s in report["skipped"]) + ".", ""]
    out += ["## Правило решения Шифу", "", *rule_lines(report), ""]
    for key, part in report["parts"].items():
        out += part_section(key, part)
    out += ["## Цена", "", *price_lines(meta), *jev_lines(report)]
    if "live" in report:
        out += ["", *render_live(report["live"])]
    return "\n".join(out) + "\n"


def price_lines(meta: Dict[str, Any]) -> List[str]:
    ds = meta.get("deepseek") or {}
    if not ds:
        return ["- DeepSeek: не измерялась (фейковый провайдер)."]
    return [f"- DeepSeek: баланс до {ds.get('before')}, после {ds.get('after')}, потрачено "
            f"{ds.get('spent') or 'не измерено'} ({ds.get('note')}); вызовов LLM {meta.get('llm_calls_total')}, входных "
            f"токенов по ходам (usage API, выходные стенд не считает) {meta.get('prompt_tokens_total')}."]


def render_live(live: Dict[str, Any]) -> List[str]:
    cmp_ = live["compare"]
    lines = [f"## Сверка A с живым марафоном ({live['dir']})", "",
             f"- сравнено шагов: {cmp_['compared']}, расхождений: {cmp_['disagree']} ({pct(cmp_['share'])}; допуск "
             f"{MAX_LIVE_DISAGREE * 100:.0f} %): {'в допуске' if live['within_tolerance'] else 'ВЫШЕ ДОПУСКА - вывод не подавать'};",
             f"- живой FAIL, стенд PASS: {', '.join(cmp_['live_fail_dry_pass']) or '-'}; "
             f"живой PASS, стенд FAIL: {', '.join(cmp_['live_pass_dry_fail']) or '-'}; "
             f"не сравнено: {len(cmp_['skipped'])}.", ""]
    rows = [r for r in cmp_["rows"] if not r["agree"]]
    if rows:
        lines += ["| шаг | живой | стенд A | живая строка |", "|---|---|---|---|"]
        lines += [f"| {r['id']} | {r['live']} | {r['dry']} | {r['live_line'][:110]} |" for r in rows]
    return lines


__all__ = ["BETTER_PARTS", "MAX_LATENCY_GROWTH_S", "MAX_LIVE_DISAGREE", "MAX_PASS_DROP_PP", "build_report",
           "decision_rule", "overall_rule", "paired", "render", "step_record"]
