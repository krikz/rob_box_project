#!/usr/bin/env python3
"""Факты для карты удаления v1 (`docs/dialog/pr15_removal_map.md`, PR-15 #3711): AST-граф импортов и размеры.

Скрипт читает дерево git-ссылки (`git ls-tree` + `git cat-file`), а не рабочую копию: результат не зависит от
мусора в чекауте (каталоги «C:», `__pycache__`), и его можно повторить на любой ссылке.

Методика (карта §2):
* граф — AST всех отслеживаемых `*.py` (весь репозиторий: `src`, `scripts`, `tools`, `tests`, `docker`, ...);
  ребро «файл → модуль» строится по `import a.b` и `from a import b` (в т. ч. относительным и ленивым внутри функций);
  `from pkg import Name`, где `pkg/__init__.py` реэкспортирует `Name` из подмодуля, даёт ребро к подмодулю;
* «читатель» модуля — не-тестовый файл с ребром к нему; `__init__.py` пакета (реэкспорт) читателем не считается,
  но печатается отдельно (его правят при удалении); тест — путь с каталогом `test`/`tests`, `test_*.py`,
  `conftest.py`;
* «замкнутое v1-множество» — неподвижная точка: начиная с корня (`dialogue_node.py`), добавляем не-тестовый
  модуль из `src`, у которого есть хотя бы один читатель и **все** читатели (из `src` и вне его — `scripts/`,
  `tools/`) уже в множестве;
* размер — `wc -l`: число символов `\\n` в blob (то же, что `git show <ref>:<path> | wc -l`).

Запуск из корня репозитория:
    python docs/dialog/pr15_map_facts.py --ref origin/develop --old 8178e2809
"""

from __future__ import annotations

import argparse
import ast
import subprocess
import sys
from collections import defaultdict
from pathlib import PurePosixPath

V = "src/rob_box_voice/rob_box_voice/"
ROOT = V + "dialogue_node.py"
# D6: мёртвые модули вне карточки (решение — удалить в 15a с grep-доказательством).
D6 = [V + "core/llm_config.py", V + "core/voice_memory_init.py", V + "utils/github_summary.py"]
# Гард CI на ключи счётчика пропусков v1 (G-Lint Code.yml): уходит вместе с dialogue_node (15p).
GUARD = ["scripts/lint/dialogue_skip_reasons.py", V + "core/skip_reason_checker.py"]
# Модули карты, которые сокращаются или остаются (печатаются в таблице размеров).
EXTRA = [V + p for p in (
    "core/conversation_history.py", "core/speech_compat.py", "core/speak_helpers.py", "core/dialogue_guards.py",
    "core/dialogue_helpers.py", "core/dialogue_text.py", "core/greeting_window.py", "core/turn_context.py",
    "core/llm_skip_reasons.py", "core/skip_reason_checker.py", "core/llm_config.py", "core/voice_memory_init.py",
    "utils/github_summary.py", "dialog_node.py", "core/dialog_v2_ports.py", "core/dialog_v2_parity.py",
    "core/plan_host.py")] + ["src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py"]
V2_PREFIXES = ("rob_box_dialog.", "rob_box_voice.dialog_node", "rob_box_voice.core.dialog_v2_")
# Части `dialogue_node.py` по методам (карта §6). «a..b» — все методы класса подряд от a до b (порядок в файле).
PARTS = {
    "15b": ["_on_startup_greeting*", "_subscribe_vision_events", "_on_vision_event", "_cancel_greeting_timer"],
    "15c": ["_on_stt", "_finish_stt_admission", "_resume_stt_after_media", "_dispatch_cleaned"],
    "15d": ["_on_vad", "vad_callback", "_handle_silence", "_on_command_feedback", "_on_tts_finished",
            "_on_tts_provider_state", "_on_odom_snapshot", "_publish_state", "_map_emotion_to_animation",
            "_is_llm_unavailable_error", "_generate_fallback_response", "_detect_pitch_intent",
            "_on_mcp_tools_update", "_on_perception_update"],
    "15f": ["_on_speaker.._current_acquaintance",
            "_resolve_speaker_for_utterance.._publish_confirmed_identity_growth"],
    "15g": ["_configure_face_voice_hint.._publish_speaker_observation"],
    "15h": ["_voice_turn_text", "_release_turn_speech", "_music_reply_context", "_publish_turn_text",
            "_publish_response", "_publish_response_batch", "_speak_direct",
            "fn:_clean_name", "fn:identity_question", "fn:classify_identity_confirmation"],
    "15i": ["_music_playing_now.._execute_media_tool_result", "_on_music_state", "_on_sound_state"],
    "15j": ["_on_dialogue_control.._publish_control_ack", "_session_epoch_gate.._finalize_turn_dsm",
            "_reset_dialogue_session.._on_inactivity_check"],
    "15k": ["_handle_llm_error", "_invoke_llm_with_telemetry", "_prepare_user_input_context", "_handle_result"],
    "15l": ["_dispatch_turn", "_build_dynamic_system_context", "_handle_speaker_turn", "_run_turn"],
    "15n": ["_parse_provider_chain.._music_state_mem", "parameters_callback"],
    "15o": ["_declare_params", "shutdown_asyncio_loop", "destroy_node", "fn:main"],
    "15p": ["__init__"],
}


def git(*args: str) -> bytes:
    return subprocess.run(["git", *args], check=True, capture_output=True).stdout


def cat_batch(ref: str, names: list[str]) -> dict[str, bytes]:
    req = "".join(f"{ref}:{n}\n" for n in names).encode("utf-8")
    out = subprocess.run(["git", "cat-file", "--batch"], input=req, check=True, capture_output=True).stdout
    blobs, pos = {}, 0
    for name in names:
        header_end = out.index(b"\n", pos)
        parts = out[pos:header_end].split()
        if parts[-1] == b"missing":
            pos = header_end + 1
            continue
        size = int(parts[2])
        blobs[name] = out[header_end + 1: header_end + 1 + size]
        pos = header_end + 1 + size + 1
    return blobs


def tree(ref: str) -> dict[str, bytes]:
    names = [n for n in git("ls-tree", "-r", "--name-only", ref).decode("utf-8").splitlines() if n.endswith(".py")]
    return cat_batch(ref, names)


def wc_l(blob: bytes) -> int:
    return blob.count(b"\n")


def is_test(path: str) -> bool:
    p = PurePosixPath(path)
    return bool({"test", "tests"} & set(p.parts[:-1])) or p.name.startswith("test_") or p.name == "conftest.py"


def module_names(files: set[str]) -> dict[str, str]:
    """Файл → dotted-имя: подъём по каталогам с `__init__.py`."""
    dirs_with_init = {str(PurePosixPath(f).parent) for f in files if f.endswith("/__init__.py")}
    names = {}
    for f in files:
        p = PurePosixPath(f)
        parts = [p.stem] if p.name != "__init__.py" else []
        d = p.parent
        while str(d) in dirs_with_init:
            parts.insert(0, d.name)
            d = d.parent
        names[f] = ".".join(parts)
    return names


def priority(f: str) -> tuple:
    return (not f.startswith("src/"), is_test(f), f)


class Graph:
    def __init__(self, blobs: dict[str, bytes]):
        self.blobs = blobs
        self.files = set(blobs)
        self.name_of = module_names(self.files)
        self.file_of: dict[str, str] = {}
        for f in sorted(self.files, key=priority):
            self.file_of.setdefault(self.name_of[f], f)
        self.trees = {}
        for f, b in blobs.items():
            try:
                self.trees[f] = ast.parse(b.decode("utf-8", "replace"), filename=f)
            except (SyntaxError, ValueError):  # py2-скрипты, файлы с NUL — без рёбер
                self.trees[f] = None
        self.edges: dict[str, set[str]] = defaultdict(set)  # importer -> targets
        for f in self.files:
            for target in self._targets(f):
                if target != f:
                    self.edges[f].add(target)
        self.readers: dict[str, set[str]] = defaultdict(set)
        for f, ts in self.edges.items():
            for t in ts:
                self.readers[t].add(f)

    def _package(self, f: str) -> str:
        name = self.name_of[f]
        return name if f.endswith("/__init__.py") else name.rpartition(".")[0]

    def _resolve_name(self, mod: str, attr: str, depth: int = 0) -> str | None:
        """`from mod import attr` → файл: подмодуль, реэкспорт из `__init__` (до 3 уровней) или сам `mod`."""
        if f"{mod}.{attr}" in self.file_of:
            return self.file_of[f"{mod}.{attr}"]
        target = self.file_of.get(mod)
        if target and target.endswith("/__init__.py") and depth < 3 and self.trees.get(target):
            for node in ast.walk(self.trees[target]):
                if isinstance(node, ast.ImportFrom) and any((a.asname or a.name) == attr for a in node.names):
                    src = self._abs(target, node)
                    if src:
                        real = next(a.name for a in node.names if (a.asname or a.name) == attr)
                        return self._resolve_name(src, real, depth + 1) or self.file_of.get(src)
        return target

    def _abs(self, f: str, node: ast.ImportFrom) -> str | None:
        if not node.level:
            return node.module
        base = self._package(f).split(".") if self._package(f) else []
        if node.level > 1:
            base = base[: len(base) - (node.level - 1)]
        return ".".join(base + ([node.module] if node.module else []))

    def _targets(self, f: str):
        t = self.trees.get(f)
        if t is None:
            return
        for node in ast.walk(t):
            if isinstance(node, ast.Import):
                for a in node.names:
                    if a.name in self.file_of:
                        yield self.file_of[a.name]
            elif isinstance(node, ast.ImportFrom):
                mod = self._abs(f, node)
                if not mod:
                    continue
                for a in node.names:
                    r = self._resolve_name(mod, a.name) if a.name != "*" else self.file_of.get(mod)
                    if r:
                        yield r

    def nontest_readers(self, f: str) -> tuple[set[str], set[str]]:
        """(читатели без `__init__`-реэкспорта, `__init__`-реэкспортёры)."""
        rs = {r for r in self.readers.get(f, ()) if not is_test(r)}
        inits = {r for r in rs if r.endswith("/__init__.py")}
        return rs - inits, inits

    def test_readers(self, f: str) -> set[str]:
        return {r for r in self.readers.get(f, ()) if is_test(r)}

    def closure(self, roots: set[str]) -> set[str]:
        s = set(roots)
        changed = True
        while changed:
            changed = False
            for f in sorted(self.files):
                if f in s or not f.startswith("src/") or is_test(f) or f.endswith("/__init__.py"):
                    continue
                rs, _ = self.nontest_readers(f)
                if rs and rs <= s:
                    s.add(f)
                    changed = True
        return s


def short(f: str) -> str:
    return f.replace(V, "").replace("src/rob_box_", "rob_box_")


def methods(blob: bytes):
    """Методы класса DialogueNode и функции модуля: (имя, начало с декораторами, конец)."""
    t = ast.parse(blob.decode("utf-8"))
    cls, funcs = [], []
    for node in t.body:
        if isinstance(node, ast.ClassDef) and node.name == "DialogueNode":
            for m in node.body:
                if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    start = min([m.lineno] + [d.lineno for d in m.decorator_list])
                    cls.append((m.name, start, m.end_lineno))
            klass = (node.lineno, node.end_lineno)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            start = min([node.lineno] + [d.lineno for d in node.decorator_list])
            funcs.append(("fn:" + node.name, start, node.end_lineno))
    return cls, funcs, klass


def part_sizes(blob: bytes):
    cls, funcs, klass = methods(blob)
    order = [m[0] for m in cls]
    by = {m[0]: m for m in cls + funcs}
    explicit = {s for specs in PARTS.values() for s in specs if ".." not in s and not s.endswith("*")}
    explicit |= {n for specs in PARTS.values() for s in specs if s.endswith("*") for n in order if n.startswith(s[:-1])}
    ranged = {}  # широкий диапазон (15n) не забирает методы, попавшие в более узкие диапазоны других частей
    for part, specs in PARTS.items():
        for spec in specs:
            if ".." in spec:
                a, b = spec.split("..")
                ranged[spec] = order[order.index(a): order.index(b) + 1]
    taken, rows = {}, []
    for part, specs in PARTS.items():
        names = []
        for spec in specs:
            if ".." in spec:  # диапазон берёт только методы, не названные явно и не вошедшие в более узкий диапазон
                narrower = {n for s2, ns in ranged.items() if s2 != spec and len(ns) < len(ranged[spec]) for n in ns}
                names += [n for n in ranged[spec] if n not in explicit and n not in narrower]
            elif spec.endswith("*"):
                names += [n for n in order if n.startswith(spec[:-1])]
            else:
                names.append(spec)
        lines = 0
        for n in names:
            if n in taken:
                raise SystemExit(f"{n}: в двух частях ({taken[n]}, {part})")
            taken[n] = part
            _, s, e = by[n]
            lines += e - s + 1
        rows.append((part, len(names), lines))
    free = [m for m in cls + funcs if m[0] not in taken]
    return rows, free, cls, funcs, klass, taken


def root_usage(g: Graph, closed: set[str]) -> dict[str, dict[str, set[str]]]:
    """Модуль множества → {часть: методы dialogue_node.py, где встречаются импортированные из него имена}.

    Учитываются имена из `from m import a` и `import m` верхнего уровня файла; `self.<атрибут>`, созданный в
    `__init__`, не отслеживается (см. «Что не проверено»)."""
    blob = g.blobs[ROOT]
    tree_ = g.trees[ROOT]
    name_src: dict[str, str] = {}
    for node in ast.walk(tree_):
        if isinstance(node, ast.ImportFrom):
            mod = g._abs(ROOT, node)
            for a in node.names:
                r = g._resolve_name(mod, a.name) if mod else None
                if r:
                    name_src[a.asname or a.name] = r
        elif isinstance(node, ast.Import):
            for a in node.names:
                if a.name in g.file_of:
                    name_src[(a.asname or a.name).split(".")[0]] = g.file_of[a.name]
    _, _, cls, funcs, _, taken = part_sizes(blob)
    spans = [(n, s, e) for n, s, e in cls + funcs]
    usage: dict[str, dict[str, set[str]]] = defaultdict(lambda: defaultdict(set))
    for node in ast.walk(tree_):
        if isinstance(node, ast.Name) and node.id in name_src:
            line = node.lineno
            owner = next((n for n, s, e in spans if s <= line <= e), "<модуль>")
            usage[name_src[node.id]][taken.get(owner, "шапка")].add(owner)
    return usage


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", default="origin/develop")
    ap.add_argument("--old", default="8178e2809")
    ap.add_argument("--methods", action="store_true", help="только список методов dialogue_node.py с частями")
    args = ap.parse_args()
    if args.methods:
        sys.stdout.reconfigure(encoding="utf-8")
        _, _, cls, funcs, _, taken = part_sizes(cat_batch(args.ref, [ROOT])[ROOT])
        for n, s, e in cls + funcs:
            print(f"{s:5d}-{e:5d} {e - s + 1:4d} {taken.get(n, '—'):4s} {n}")
        return
    sys.stdout.reconfigure(encoding="utf-8")  # Windows: консоль cp1252
    sha = git("rev-parse", "--short=9", args.ref).decode().strip()
    old_sha = git("rev-parse", "--short=9", args.old).decode().strip()
    blobs = tree(args.ref)
    g = Graph(blobs)
    n_edges = sum(len(v) for v in g.edges.values())
    src_files = [f for f in g.files if f.startswith("src/")]
    print(f"## ref {args.ref} = {sha}; old {args.old} = {old_sha}")
    print(f"py-файлов: всего {len(g.files)}, в src {len(src_files)}; рёбер (файл→файл) всего {n_edges}, "
          f"из src {sum(len(v) for k, v in g.edges.items() if k.startswith('src/'))}")

    closed = g.closure({ROOT})
    print(f"\n### Замкнутое v1-множество от {short(ROOT)}: {len(closed)} файлов, "
          f"{sum(wc_l(blobs[f]) for f in closed)} строк (wc -l)")
    for f in sorted(closed, key=short):
        rs, inits = g.nontest_readers(f)
        ts = g.test_readers(f)
        ext = sorted(r for r in rs if not r.startswith("src/"))
        print(f"  {wc_l(blobs[f]):5d}  {short(f):45s} читатели {len(rs)}: {', '.join(sorted(map(short, rs)))}"
              f"{' | вне src: ' + ', '.join(ext) if ext else ''}"
              f"{' | реэкспорт: ' + ', '.join(map(short, inits)) if inits else ''}"
              f" | тестов {len(ts)} ({sum(wc_l(blobs[t]) for t in ts)} строк)")

    wider = g.closure({ROOT, *D6, *GUARD})
    extra = sorted(wider - closed - set(D6) - set(GUARD), key=short)
    print(f"\n### Сироты после удаления множества + D6 + гарда skip-reasons: {len(extra)}")
    for f in extra:
        print(f"  {wc_l(blobs[f]):5d}  {short(f)}")

    print("\n### Модули карты вне множества: размер, читатели (не-тест), тесты")
    for f in EXTRA:
        if f not in blobs:
            print(f"  нет файла: {f}")
            continue
        rs, inits = g.nontest_readers(f)
        ts = g.test_readers(f)
        print(f"  {wc_l(blobs[f]):5d}  {short(f):45s} в множестве={'да' if f in closed else 'нет'}; "
              f"читатели {len(rs)}: {', '.join(sorted(map(short, rs)))}"
              f"{' | реэкспорт: ' + ', '.join(map(short, inits)) if inits else ''}"
              f" | тестов {len(ts)} ({sum(wc_l(blobs[t]) for t in ts)} строк)")

    print("\n### Мёртвые модули rob_box_voice (нет не-тестовых читателей ни в src, ни вне; не точка входа)")
    setup = git("show", f"{args.ref}:src/rob_box_voice/setup.py").decode("utf-8")
    for f in sorted(g.files):
        if not f.startswith(V) or is_test(f) or f.endswith("/__init__.py") or f in closed:
            continue
        rs, inits = g.nontest_readers(f)
        mod = g.name_of[f]
        if not rs and f"{mod}:" not in setup:
            print(f"  {wc_l(blobs[f]):5d}  {short(f)}  (тестов {len(g.test_readers(f))}"
                  f"{'; реэкспорт ' + ', '.join(map(short, inits)) if inits else ''})")

    v1_tests = defaultdict(set)
    for f in closed | set(D6) | set(GUARD):
        for t in g.test_readers(f):
            v1_tests[t].add(f)
    mixed = sorted(t for t in v1_tests
                   if any(g.name_of.get(x, "").startswith(V2_PREFIXES) for x in g.edges.get(t, ())))
    print(f"\n### Тесты, импортирующие v1-множество (+D6, гард): {len(v1_tests)} файлов, "
          f"{sum(wc_l(blobs[t]) for t in v1_tests)} строк; из них смешанных (импортируют и v2: "
          f"{', '.join(V2_PREFIXES)}): {len(mixed)}")
    for t in mixed:
        print(f"  {wc_l(blobs[t]):5d}  {t}")
    dsm = "src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py"
    dsm_tests = g.test_readers(dsm)
    print(f"Тесты DSM (AST, вкл. реэкспорт через rob_box_harness.core): {len(dsm_tests)} файлов, "
          f"{sum(wc_l(blobs[t]) for t in dsm_tests)} строк; пересечение с тестами множества: "
          f"{len(dsm_tests & set(v1_tests))}")
    rs, inits = g.nontest_readers(dsm)
    print(f"Читатели DSM (не-тест): {', '.join(sorted(map(short, rs)))} | реэкспорт: {', '.join(map(short, inits))}")
    issue_tests = [f for f in g.files if PurePosixPath(f).name.startswith("test_issue_")]
    voice_t = [f for f in issue_tests if f.startswith("src/rob_box_voice/test/")]
    print(f"test_issue_*: всего {len(issue_tests)}; rob_box_voice/test {len(voice_t)} (из них импортируют v1-множество "
          f"{sum(f in v1_tests for f in voice_t)}, v1 или DSM {sum(f in v1_tests or f in dsm_tests for f in voice_t)}); "
          f"другие пакеты src {sum(f.startswith('src/') and f not in voice_t for f in issue_tests)}; "
          f"tests/ {sum(f.startswith('tests/') for f in issue_tests)}")

    print(f"\n### Размеры wc -l: {old_sha} → {sha} (blob равен = файл не менялся)")
    old_blobs = cat_batch(args.old, sorted(closed | set(EXTRA)))
    for f in sorted(closed | set(EXTRA), key=short):
        new_b, old_b = blobs.get(f), old_blobs.get(f)
        same = "=" if new_b == old_b else "изм."
        o = wc_l(old_b) if old_b is not None else "-"
        n = wc_l(new_b) if new_b is not None else "-"
        print(f"  {str(o):>5} → {str(n):>5}  {same:4s}  {short(f)}")
    old_g = Graph(tree(args.old))
    old_closed = old_g.closure({ROOT})
    print(f"Граф на {old_sha} той же методикой: py-файлов {len(old_g.files)}, в src "
          f"{sum(1 for f in old_g.files if f.startswith('src/'))}; рёбер {sum(len(v) for v in old_g.edges.values())}, "
          f"из src {sum(len(v) for k, v in old_g.edges.items() if k.startswith('src/'))}")
    print(f"Множество на {old_sha}: {len(old_closed)} файлов, "
          f"{sum(wc_l(old_blobs.get(f) or git('show', f'{args.old}:{f}')) for f in old_closed)} строк; "
          f"ушли: {', '.join(sorted(map(short, old_closed - closed)))}; пришли: "
          f"{', '.join(sorted(map(short, closed - old_closed))) or '—'}")

    rows, free, cls, funcs, klass, _ = part_sizes(blobs[ROOT])
    in_methods = sum(e - s + 1 for _, s, e in cls)
    print(f"\n### dialogue_node.py по AST: {len(cls)} методов DialogueNode, {in_methods} строк в методах; "
          f"класс {klass[0]}–{klass[1]}; функций модуля {len(funcs)}; файл {wc_l(blobs[ROOT])} строк")
    for part, n, lines in rows:
        print(f"  {part}: {n:3d} методов/функций, {lines:4d} строк")
    print(f"  не распределены по частям ({len(free)}): "
          + ", ".join(f"{n} {e - s + 1}" for n, s, e in free))
    print(f"  вне методов и функций: {wc_l(blobs[ROOT]) - in_methods - sum(e - s + 1 for _, s, e in funcs)} строк")

    print("\n### Где dialogue_node.py использует имена из модулей v1 (часть: методы); импорт в шапке не считается")
    usage = root_usage(g, closed)
    shared = ["core/greeting_window.py", "core/turn_context.py", "core/speak_helpers.py", "core/dialogue_guards.py",
              "core/dialogue_helpers.py", "core/llm_skip_reasons.py"]
    watch = sorted(closed - {ROOT}, key=short) + [V + p for p in shared] + [
        "src/rob_box_harness/rob_box_harness/core/dialogue_state_machine.py"]
    for f in watch:
        parts = usage.get(f, {})
        body = "; ".join(f"{p}: {', '.join(sorted(ms))}" for p, ms in sorted(parts.items()))
        print(f"  {short(f)} ← {body or 'только импорт / не используется в dialogue_node'}")

    hc = "src/rob_box_harness/rob_box_harness/core/"
    agent_roots = {ROOT, *D6, *GUARD, "scripts/dialogue/chat.py", "scripts/dialogue/local_tools.py",
                   hc + "agent_core.py", hc + "dialogue_state_machine.py"}
    base = g.closure({ROOT, *D6, *GUARD})
    extra = sorted(g.closure(agent_roots) - base - agent_roots, key=short)
    print("\n### Цикл хода v1 в rob_box_harness: что осиротеет, если удалить ещё scripts/dialogue/{chat,local_tools}.py, "
          "agent_core.py (класс AgentCore) и DSM")
    for f in extra:
        rs, _ = g.nontest_readers(f)
        print(f"  {wc_l(blobs[f]):5d}  {short(f)} ← {', '.join(sorted(map(short, rs)))}; тестов {len(g.test_readers(f))}")
    print(f"  итого сирот: {len(extra)} файлов, {sum(wc_l(blobs[f]) for f in extra)} строк")
    for f in ("agent_core.py", "assembly.py", "tool_loop/command_turn.py", "tool_loop/text_classify.py",
              "acceptance.py", "confirmation_policy.py"):
        rs, _ = g.nontest_readers(hc + f)
        ts = g.test_readers(hc + f)
        print(f"  {wc_l(blobs[hc + f]):5d}  core/{f} — читатели: {', '.join(sorted(map(short, rs)))}; "
              f"тестов {len(ts)} ({sum(wc_l(blobs[t]) for t in ts)} строк)")
    t = ast.parse(blobs[hc + "agent_core.py"].decode("utf-8"))
    print("  agent_core.py верхний уровень: " + "; ".join(
        f"{n.name} {n.lineno}–{n.end_lineno} ({n.end_lineno - n.lineno + 1})" for n in t.body
        if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.end_lineno - n.lineno + 1 >= 20))

    print("\n### Определения верхнего уровня (начало–конец, строк)")
    for f in (V + "core/stt_admission.py", V + "core/turn_context.py", V + "core/speak_helpers.py",
              V + "core/dialogue_helpers.py", V + "core/dialogue_guards.py"):
        t = ast.parse(blobs[f].decode("utf-8"))
        defs = [(getattr(n, "name", type(n).__name__), min([n.lineno] + [d.lineno for d in
                 getattr(n, "decorator_list", [])]), n.end_lineno)
                for n in t.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))]
        print(f"  {short(f)} ({wc_l(blobs[f])}): " + "; ".join(f"{n} {s}–{e} ({e - s + 1})" for n, s, e in defs))


if __name__ == "__main__":
    main()
