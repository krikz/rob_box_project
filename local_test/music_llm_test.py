#!/usr/bin/env python3
"""
music_llm_test.py — Концепт-тест: LLM управляет музыкой через MusicManager

Архитектура:
  SuperCollider (natively on Windows, port 57110)
      ↑ OSC UDP
  renardo_lib  ← MusicManager.execute_code()
      ↑ python
  @function_tool wrappers  ← openai-agents SDK
      ↑ tool calls
  DeepSeek LLM
      ↑ user input
  interactive REPL  ← тебя

══════════════════════════════════════════════
Требования:
  pip install openai-agents renardo renardo-lib

  SC нативно на Windows:
    scide  → Server → Start Server (порт 57110)

Использование:
  set DEEPSEEK_API_KEY=sk-...
  python local_test/music_llm_test.py

  # Без SC/renardo — только смотреть что LLM вызывает (dry-run):
  python local_test/music_llm_test.py --dry-run
══════════════════════════════════════════════
"""

import sys
import os
import asyncio
import argparse
import json
import textwrap
from pathlib import Path

# ── Добавить mcp_tools в path (не нужно pip install) ──────────────────────
ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "src" / "rob_box_mcp_tools"))

# ── Отключить трейсинг openai-agents (не нужен, используем DeepSeek) ────────
os.environ.setdefault("OPENAI_API_KEY", "not-used-tracing-disabled")

# ── DeepSeek ────────────────────────────────────────────────────────────────
DEEPSEEK_API_KEY = 'sk-16b012d9d92f4aba89d07a9e8f9b6cba'
DEEPSEEK_BASE_URL = "https://api.deepseek.com/v1"
MODEL_NAME = "deepseek-chat"  # или deepseek-reasoner

# ─────────────────────────────────────────────────────────────────────────────
# DryRunMusicManager — логирует вызовы вместо реального выполнения
# ─────────────────────────────────────────────────────────────────────────────

class DryRunMusicManager:
    """Мок MusicManager: не требует SC/renardo, только логирует вызовы."""

    VIBE_PRESETS = {
        "chill":     {"scale": "major",    "bpm": 85,  "root": "C"},
        "energetic": {"scale": "minor",    "bpm": 140, "root": "A"},
        "ambient":   {"scale": "dorian",   "bpm": 70,  "root": "D"},
        "jazz":      {"scale": "lydian",   "bpm": 120, "root": "F"},
        "dark":      {"scale": "phrygian", "bpm": 100, "root": "E"},
    }

    def __init__(self) -> None:
        self._pattern_history: dict = {}
        self._active_patterns: set = set()
        self._current_preset: str | None = None
        print("[DryRun] MusicManager инициализирован (SC/renardo не требуется)")

    def execute_code(self, code: str, pattern_name: str | None = None) -> dict:
        print(f"\n[DryRun] 🎵 EXECUTE CODE:")
        for line in code.splitlines():
            print(f"  >>> {line}")
        if pattern_name:
            self._pattern_history[pattern_name] = code
            self._active_patterns.add(pattern_name)
            print(f"[DryRun] Паттерн '{pattern_name}' сохранён в историю")
        return {"success": True, "message": f"[DryRun] Код бы выполнился: {code[:60]}..."}

    def stop_pattern(self, pattern_name: str) -> dict:
        print(f"[DryRun] ⏹  STOP pattern='{pattern_name}'")
        self._active_patterns.discard(pattern_name)
        return {"success": True, "message": f"[DryRun] Паттерн '{pattern_name}' остановлен"}

    def stop_all(self) -> dict:
        print("[DryRun] ⏹  STOP ALL (Clock.clear())")
        self._active_patterns.clear()
        return {"success": True, "message": "[DryRun] Вся музыка остановлена"}

    def set_vibe_preset(self, preset_name: str) -> dict:
        if preset_name not in self.VIBE_PRESETS:
            return {"success": False, "error": f"Пресет '{preset_name}' не найден"}
        preset = self.VIBE_PRESETS[preset_name]
        self._current_preset = preset_name
        print(f"[DryRun] 🎼 SET VIBE PRESET '{preset_name}': {preset}")
        return {
            "success": True,
            "message": f"[DryRun] Пресет '{preset_name}' применён: {preset}",
            "preset": preset,
        }

    def get_state(self) -> dict:
        return {
            "renardo_available": False,
            "supercollider_running": False,
            "current_preset": self._current_preset,
            "pattern_history": dict(self._pattern_history),
            "active_patterns": list(self._active_patterns),
            "dry_run": True,
        }


# ─────────────────────────────────────────────────────────────────────────────
# LiveMusicManager — реальный менеджер без зависимостей от ROS2/MCPTool
# ─────────────────────────────────────────────────────────────────────────────

import re as _re
_BLOCKED_TOKENS = _re.compile(
    r"\b(import|os|sys|subprocess|shutil|socket|requests|urllib|http|"
    r"open|exec|eval|compile|globals|locals|vars|getattr|setattr|delattr)\b"
)

class LiveMusicManager:
    """Standalone MusicManager для локального теста (без ROS2/MCPTool)."""

    # Root ноты как числа полутонов (C=0, D=2, E=4, F=5, G=7, A=9, B=11)
    VIBE_PRESETS = {
        "chill":     {"scale": "major",    "bpm": 85,  "root": 0},
        "energetic": {"scale": "minor",    "bpm": 140, "root": 9},
        "ambient":   {"scale": "dorian",   "bpm": 70,  "root": 2},
        "jazz":      {"scale": "lydian",   "bpm": 120, "root": 5},
        "dark":      {"scale": "phrygian", "bpm": 100, "root": 4},
    }

    def __init__(self) -> None:
        self._pattern_history: dict = {}
        self._active_patterns: set = set()
        self._current_preset: str | None = None
        self._ctx: dict = {}
        self._renardo_available = False
        self._init_renardo()

    def _init_renardo(self) -> None:
        try:
            import renardo_lib.runtime as _rt
            import time
            import os

            self._ctx = vars(_rt).copy()
            self._renardo_available = True
            print("✅ renardo_lib.runtime загружен")

            # Подключаем к SC (scsynth:57110 + sclang:57120)
            S = self._ctx.get("Server")
            if S:
                try:
                    S.init_connection()
                    print("✅ Server.init_connection() OK")
                except Exception as e:
                    print(f"⚠️  init_connection: {e}")

            # Загружаем все SynthDefs в sclang (он их компилирует и отправляет в scsynth)
            if S:
                base = os.path.join(
                    os.path.dirname(_rt.__file__),
                    "SynthDefManagement", "sclang_code"
                )
                # Директории с .scd файлами синтезаторов
                for sub in ("scsynth", "sceffects", "scenvelopes"):
                    sub_path = os.path.join(base, sub)
                    if not os.path.isdir(sub_path):
                        continue
                    for fname in os.listdir(sub_path):
                        if fname.endswith(".scd"):
                            full = os.path.join(sub_path, fname)
                            try:
                                S.loadSynthDef(full)
                            except Exception:
                                pass
                # Базовые файлы
                for fname in ("Buffers.scd", "OscFunc.scd", "Record.scd"):
                    full = os.path.join(base, fname)
                    if os.path.isfile(full):
                        try:
                            S.loadSynthDef(full)
                        except Exception:
                            pass

                time.sleep(2)  # Ждём пока sclang скомпилирует синтдефы
                print("✅ SynthDefs загружены в SC")

        except Exception as e:
            print(f"⚠️  renardo_lib недоступен: {e}")

    def _check_sc(self) -> bool:
        import socket
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.settimeout(1.0)
                s.sendto(b"/status\x00,\x00\x00\x00", ("127.0.0.1", 57110))
                return len(s.recvfrom(512)[0]) > 0
        except OSError:
            return False

    def execute_code(self, code: str, pattern_name: str | None = None) -> dict:
        if _BLOCKED_TOKENS.search(code):
            return {"success": False, "error": "Запрещённые конструкции в коде"}
        if not self._check_sc():
            return {"success": False, "error": "SuperCollider не запущен (127.0.0.1:57110)"}
        if not self._renardo_available:
            return {"success": False, "error": "renardo_lib недоступен"}
        try:
            exec(code, self._ctx)  # noqa: S102
        except Exception as exc:
            return {"success": False, "error": f"Ошибка выполнения: {exc}"}
        if pattern_name:
            self._pattern_history[pattern_name] = code
            self._active_patterns.add(pattern_name)
        return {"success": True, "message": "Код выполнен"}

    def stop_pattern(self, pattern_name: str) -> dict:
        if self._renardo_available and self._check_sc():
            try:
                exec(f"{pattern_name}.stop()", self._ctx)  # noqa: S102
            except Exception as exc:
                return {"success": False, "error": str(exc)}
        self._active_patterns.discard(pattern_name)
        return {"success": True, "message": f"Паттерн '{pattern_name}' остановлен"}

    def stop_all(self) -> dict:
        if self._renardo_available and self._check_sc():
            try:
                exec("Clock.clear()", self._ctx)  # noqa: S102
            except Exception as exc:
                return {"success": False, "error": str(exc)}
        self._active_patterns.clear()
        return {"success": True, "message": "Вся музыка остановлена"}

    def set_vibe_preset(self, preset_name: str) -> dict:
        if preset_name not in self.VIBE_PRESETS:
            return {"success": False, "error": f"Пресет '{preset_name}' не найден"}
        p = self.VIBE_PRESETS[preset_name]
        self._current_preset = preset_name
        if self._renardo_available:
            try:
                code = f"Clock.bpm = {p['bpm']}\nScale.default = Scale.{p['scale']}\nRoot.default = {p['root']}"
                exec(code, self._ctx)  # noqa: S102
            except Exception as exc:
                print(f"⚠️  Пресет применён частично: {exc}")
        return {"success": True, "message": f"Пресет '{preset_name}': {p}", "preset": p}

    def get_state(self) -> dict:
        return {
            "renardo_available": self._renardo_available,
            "supercollider_running": self._check_sc(),
            "current_preset": self._current_preset,
            "pattern_history": dict(self._pattern_history),
            "active_patterns": list(self._active_patterns),
        }


# ─────────────────────────────────────────────────────────────────────────────
# Создание MusicManager (реальный или dry-run)
# ─────────────────────────────────────────────────────────────────────────────

def create_manager(dry_run: bool):
    if dry_run:
        return DryRunMusicManager()

    mgr = LiveMusicManager()
    state = mgr.get_state()

    if not state["supercollider_running"]:
        print("⚠️  SuperCollider не запущен на localhost:57110!")
        print("   Запусти scide → Server → Boot Server")
        ans = input("Продолжить всё равно? [y/N]: ").strip().lower()
        if ans != "y":
            sys.exit(1)

    if not state["renardo_available"]:
        print("⚠️  renardo_lib не инициализировался. Запускаюсь в dry-run режиме.")
        return DryRunMusicManager()

    print(f"✅ LiveMusicManager готов: SC={state['supercollider_running']}, renardo={state['renardo_available']}")
    return mgr


# ─────────────────────────────────────────────────────────────────────────────
# Системный промпт
# ─────────────────────────────────────────────────────────────────────────────

# Загрузить документацию Renardo из файла
_REF_PATH = Path(__file__).parent / "RENARDO_REFERENCE.md"
_RENARDO_REF = _REF_PATH.read_text(encoding="utf-8") if _REF_PATH.exists() else ""

SYSTEM_PROMPT = textwrap.dedent(f"""
    Ты — Роб, робот-музыкант. Ты умеешь играть живую музыку в реальном времени
    с помощью Renardo (FoxDot-совместимый язык генеративной музыки).

    Твои музыкальные инструменты:
    - search_samples(query, pack, case): ВСЕГДА вызывай перед созданием музыки!
      Ищет семплы по ключевому слову в имени файла (имя файла = характер звука).
      query="kick" → все кик-барабаны. query="snare" → снейры. query="*" → обзор букв.
    - execute_music_code(code): выполнить Renardo-код (создать/обновить паттерн)
    - stop_music(pattern_name): остановить паттерн или всю музыку
    - set_vibe_preset(vibe): быстро задать вайб (chill/energetic/ambient/jazz/dark)
    - get_music_state(): узнать текущее состояние
    - generate_tts_sample: НЕДОСТУПЕН — не использовать!
      Для голосовых/вокальных звуков ищи через search_samples:
      search_samples("vocal") / search_samples("voice") / search_samples("scream") / search_samples("choir")
      При крике/страхе: search_samples("scream") или search_samples("dist", pack="1_pitchglitch_samples")

    СТРАТЕГИЯ ВЫБОРА СЕМПЛОВ:
    1. Вызови search_samples("kick") → получаешь letter + sample_index + готовый play_code
    2. Используй разные слова: "snare", "hat", "bass", "synth", "dist", "glitch", "pad"
    3. Для обзора всех букв: search_samples("*") → компактный список без лишних деталей
    4. pack="1_pitchglitch_samples" — больше разнообразных звуков чем стандартный пак
    5. Заглавная буква в play() → используй case="upper" в search_samples

    ═══════════════════════════════════════════
    ПОЛНАЯ ДОКУМЕНТАЦИЯ RENARDO:
    ═══════════════════════════════════════════
    {_RENARDO_REF}
    ═══════════════════════════════════════════

    ⚠️ КРИТИЧНО — ОШИБКИ КОТОРЫЕ НЕЛЬЗЯ ДЕЛАТЬ НИКОГДА:

    🚫 ПРАВИЛО №0 — play() ПРИНИМАЕТ ТОЛЬКО ОДНУ БУКВУ + sample=N:
    play() НЕ ЗНАЕТ никаких названий типа "industrial_1", "kick_hard", "bass_drum", "metal"!
    ❌ ЗАПРЕЩЕНО: play("industrial_1"), play("kick"), play("factory"), play("horror")
    ✅ ПРАВИЛЬНО: play("x", sample=3)  ← буква из search_samples() + индекс файла
    АЛГОРИТМ: 1) search_samples("industrial") → получаешь letter="b", sample_index=2
               2) d1 >> play("b", sample=2)  ← используй ЭТО
    Перед ЛЮБЫМ play() → сначала search_samples() чтобы узнать реальную букву!

    🚫 ПРАВИЛО №1 — ЗАПРЕТ ПСЕВДОНИМОВ PLAYER (NameError: name '...' is not defined):
    Разрешённые имена player: d1-d9 (drums), p1-p9 (synth), s1-s9 (sample), l1-l9 (loop)
    НЕЛЬЗЯ создавать переменные-псевдонимы для players — они НЕ работают в Clock callbacks!
    ❌ ЗАПРЕЩЕНО: rooster1=d1, guardian_angels=p1, modern_effects=p2, old_phone_sounds=d2,
                  main_theme=p3, angel_choir=p2 — ВСЁ ЧТО НЕ d1..d9/p1..p9/s1..s9/l1..l9!
    ✅ ПРАВИЛЬНО: используй d1, p1, p2 НАПРЯМУЮ — везде, включая callbacks:
      d1 >> play("x-o-"); def fn(): d1.amp=0; Clock.future(8, fn)  # ← d1 везде!
    ✅ ПРАВИЛЬНО: если callback ссылается на player — весь код в ОДНОМ execute_music_code

    - НИКОГДА не пиши `Clock.bpm.set(N)` — bpm это int, у него нет .set()!
    - НИКОГДА не пиши `lambda: Clock.bpm = N` — в lambda нельзя присваивать!
    - Для смены BPM в Clock.future() используй ТОЛЬКО:
        a) `Clock.future(N, lambda: setattr(Clock, 'bpm', 170))`  ← правильно
        b) `def fn(): Clock.bpm = 170\nClock.future(N, fn)`        ← правильно
    - То же самое для Scale/Root в lambda: `lambda: setattr(Scale, 'default', 'minor')`
    - НИКОГДА не передавай строки-ноты в degree: `degree=['E','C','G']` → ошибка!
      degree принимает ТОЛЬКО целые числа (позиции в гамме: 0,1,2,...)
    - Для произвольных мелодий (RTTTL, Nokia, конкретные ноты) используй `midinote`:
      `p1 >> pluck(midinote=[64,62,60,62,64,64,64], dur=[0.5,0.5,0.5,0.5,0.5,0.5,1])`
      MIDI = 12*(octave+1)+semitone; C=0,D=2,E=4,F=5,G=7,A=9,B=11; #нота=+1
      C4=48,E4=52,G4=55,C5=60,E5=64,G5=67 | C1=24,E1=28,G1=31,B1=35,C2=36
    - НИКОГДА не вызывай `.loop()` без аргумента: `P[...].loop()` → TypeError!
      Используй: `.loop(n)` где n — количество повторений, например `.loop(4)`
      ИЛИ просто не используй .loop() — паттерны в renardo зациклены по умолчанию
    - НИКОГДА не передавай одиночное число в var() первым аргументом:
      ❌ var(0, 4), var(0.5, 8), var(4, 2) → 'float' object is not iterable!
      ✅ ВСЕГДА список: var([0,4,5,3], 4), var([0.5,1.0], 8), var([4,5], 2)
      ✅ Исключение: dur= и sus= принимают одиночные числа, но var() — нет

    ПАЛИТРА СИНТЕЗАТОРОВ — используй РАЗНЫЕ инструменты для разных слоёв:
    🥁 Ударные/Lo:   d1-d9 >> play(...)   — всегда через play() с семплами
    🎸 Бас:          bass, wobblebass, dub, fuzz, dirt  (oct=2..3, degree=[0,-2,0])
    🎹 Мелодия/Lead: pluck, blip, arpy, pianovel, epiano, rhpiano, karp, sitar, marimba
       ⚠️ НЕТ "piano" — используй pianovel / epiano / rhpiano!
    🔔 Атмосфера:    pads, ambi, space, faim, bell, gong  (amp=0.3..0.6, sus=4)
    ⚡ Агрессия/Глитч: rave, donk, varsaw, pulse, quin, feel

    Примеры правильного слоения:
      d1 >> play("x-o-")                              # ударные
      p1 >> bass([0,-2,0,-2], oct=2, dur=1)           # бас (НЕ pluck!)
      p2 >> arpy([0,2,4,2], oct=5, dur=0.5)           # мелодия (НЕ pluck!)
      p3 >> pads([0], dur=4, amp=0.3, sus=6)          # атмосфера (НЕ pluck!)

    ⚠️ ПРАВИЛО РАЗНООБРАЗИЯ — ОБЯЗАТЕЛЬНО:
    - НИКОГДА не используй pluck для всех паттернов! pluck = только 1 паттерн максимум
    - Каждый трек = минимум 3 слоя разными инструментами
    - Бас → bass/wobblebass/dub, НЕ pluck
    - Атмосфера → pads/ambi/space, НЕ pluck
    - Перкуссия всегда через d1-d9 >> play()

    🎤 ПРАВИЛО ВОКАЛА — если хочешь слышимый вокал:
    - amp МИНИМУМ 0.5, иначе вокал утонет в миксе (НЕ 0.2-0.3!)
    - Короткие вокальные семплы (Eyh, DeDuDip, YeahHi): dur=0.5..2, amp=0.6
      Пример: d4 >> play("c", sample=1, dur=1, amp=0.6, room=0.3)
    - Длинные хор/сустейн (Choir, ChoirWahhh, Laaaoooaaa): dur=4..8, amp=0.55
      Пример: d4 >> play("c", sample=11, dur=4, amp=0.55, room=0.5, mix=0.4)
    - Когда вокал в миксе — снизь pads до amp=0.2 чтобы не перекрывали
    - Используй КОНКРЕТНЫЙ sample=N (из search_samples), НЕ PRand для вокала —
      PRand даст непредсказуемую длину и вокал будет обрезаться или не в такт
    - ⚠️ НИКОГДА .fadein(N) для вокала! fadein(16) при 80 BPM = 12 секунд тишины —
      вокал просто не слышен. Вокал добавляй БЕЗ fadein, или максимум fadein(4)
    - ⚠️ dur= должен совпадать с длиной семпла! Короткий семпл (1-2 сек) с dur=8
      играет раз в 6 секунд — большинство времени тишина. Для коротких семплов
      используй dur=1..2, для длинных (Laaaoooaaa, Choir) — dur=4..6

    Стратегия:
    1. Сначала вызови set_vibe_preset если пользователь описывает настроение
    2. Затем execute_music_code — создавай минимум 3 паттерна: drums + bass + melody (+atmosphere)
    3. Используй реальные renardo-конструкции: var(), linvar(), PDur(), PRand(), Group(),
       .every(), .follow(), .fadein()/.fadeout(), .eclipse() — делай КРУТУЮ музыку!
    4. Для голосовых эффектов и криков — search_samples("scream") / search_samples("vocal") / search_samples("voice")
    5. Отвечай кратко — просто играй, не объясняй каждую строчку кода

    ВАЖНО: Всегда вызывай инструменты немедленно. Никогда не имитируй — всегда реально играй.
""")


# ─────────────────────────────────────────────────────────────────────────────
# Вывод того что нагенерила нейронка за один ход
# ─────────────────────────────────────────────────────────────────────────────

def _print_turn_log(log: list) -> None:
    """Печатает компактную сводку tool-вызовов, собранных за один ход."""
    if not log:
        return
    W = 52
    print()
    print("  ┌─ 🔧 Инструменты нейронки " + "─" * (W - 26))
    for entry in log:
        name = entry["name"]
        ok   = entry.get("ok", True)
        mark = "✓" if ok else "✗"

        if name == "execute_music_code":
            code = entry.get("code", "")
            pat  = entry.get("pattern_name") or ""
            tag  = f" [{pat}]" if pat else ""
            print(f"  │ {mark} execute_music_code{tag}")
            for line in code.splitlines():
                print(f"  │     {line}")
            if not ok:
                print(f"  │   ⚠ {entry.get('error', '')}")

        elif name == "search_samples":
            query = entry.get("query", "")
            pack  = entry.get("pack", "")
            case  = entry.get("case", "")
            found = entry.get("found", "?")
            extra = f", pack={pack}" if pack and pack != "0_foxdot_default" else ""
            extra += f", case={case}" if case and case != "lower" else ""
            print(f"  │ 🔍 search_samples('{query}'{extra}) → {found} результатов")
            for r in entry.get("results", [])[:5]:
                print(f"  │      {r['play_code']}  ← {r['filename']}")
            if entry.get("found", 0) > 5:
                print(f"  │      ... ещё {entry['found'] - 5} результатов")

        elif name == "set_vibe_preset":
            preset = entry.get("preset_name", "")
            info   = entry.get("preset", {})
            details = f"  bpm={info.get('bpm')} scale={info.get('scale')}" if info else ""
            print(f"  │ {mark} 🎼 set_vibe_preset('{preset}'){details}")

        elif name == "stop_music":
            pat = entry.get("pattern_name") or "all"
            print(f"  │ {mark} ⏹  stop_music('{pat}')")

        elif name == "get_music_state":
            print(f"  │ {mark} 📊 get_music_state()")

        else:
            print(f"  │ {mark} ⚙  {name}({entry})")

    print("  └" + "─" * W)


# ─────────────────────────────────────────────────────────────────────────────
# Основной REPL
# ─────────────────────────────────────────────────────────────────────────────

async def run_repl(manager, dry_run: bool):
    if not DEEPSEEK_API_KEY:
        print("❌ Установи переменную окружения DEEPSEEK_API_KEY=sk-...")
        sys.exit(1)

    try:
        from agents import Agent, Runner, function_tool, set_tracing_disabled
        from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel
        from openai import AsyncOpenAI
    except ImportError:
        print("❌ openai-agents не установлен!")
        print("   pip install openai-agents")
        sys.exit(1)

    set_tracing_disabled(True)

    # ── Лог вызовов инструментов за один ход ──────────────────────────────
    turn_log: list[dict] = []

    # ── Определяем function tools ──────────────────────────────────────────

    @function_tool
    def search_samples(
        query: str,
        pack: str = "0_foxdot_default",
        case: str = "lower",
    ) -> str:
        """Поиск семплов по ключевому слову. Используй ПЕРЕД созданием музыки!

        Имена файлов описывают характер звука: "Kick", "Snare", "HiHat", "Bass", "Synth", "Dist" и т.д.

        Args:
            query: Ключевое слово для поиска в имени файла.
                   Примеры: "kick", "snare", "hat", "bass", "synth", "dist", "glitch",
                            "loop", "vocal", "guitar", "bell", "drone", "clap", "pad".
                   Используй "*" для компактного обзора всех букв (только количества).
            pack: "0_foxdot_default" (стандартный) или "1_pitchglitch_samples" (больше звуков).
            case: "lower" = строчная буква в play(), "upper" = заглавная.
        Returns:
            JSON: letter, sample_index, filename, готовый play_code для каждого совпадения.
        """
        samples_roots = [
            Path.home() / "AppData" / "Roaming" / "renardo" / "samples",
            Path.home() / ".renardo" / "samples",
        ]
        samples_root = next((p for p in samples_roots if p.exists()), None)
        if not samples_root:
            return json.dumps({"error": "Папка с семплами не найдена"})

        pack_path = samples_root / pack
        if not pack_path.exists():
            available = [d.name for d in samples_root.iterdir() if d.is_dir()]
            return json.dumps({"error": f"Пакет '{pack}' не найден", "available_packs": available})

        exts = {".wav", ".aif", ".aiff", ".mp3"}

        # query="*" → компактный обзор: только буквы и количества
        if query.strip() == "*":
            _log_entry: dict = {"name": "search_samples", "query": query, "pack": pack, "case": case}
            overview = {}
            for folder in sorted(pack_path.iterdir()):
                if not folder.is_dir() or folder.name.startswith("."):
                    continue
                sub = folder / case
                if not sub.exists():
                    sub = folder
                count = sum(1 for f in sub.iterdir() if f.is_file() and f.suffix.lower() in exts)
                if count:
                    overview[folder.name] = count
            result_json = json.dumps(
                {"pack": pack, "case": case, "letters": overview,
                 "hint": 'Ищи по слову: search_samples("kick") или search_samples("synth", pack="1_pitchglitch_samples")'},
                ensure_ascii=False, indent=2,
            )
            _log_entry["found"] = f"обзор {len(overview)} букв"
            _log_entry["results"] = []
            turn_log.append(_log_entry)
            return result_json

        # Поиск по ключевому слову
        _log_entry = {"name": "search_samples", "query": query, "pack": pack, "case": case}
        q = query.lower().strip()
        results = []
        for folder in sorted(pack_path.iterdir()):
            if not folder.is_dir() or folder.name.startswith("."):
                continue
            sub = folder / case
            if not sub.exists():
                sub = folder
            files = sorted([f for f in sub.iterdir() if f.is_file() and f.suffix.lower() in exts])
            for idx, f in enumerate(files):
                if q in f.name.lower():
                    play_letter = folder.name.upper() if case == "upper" else folder.name
                    results.append({
                        "letter": play_letter,
                        "sample_index": idx,
                        "filename": f.name,
                        "play_code": f'd1 >> play("{play_letter}", sample={idx})',
                    })
            if len(results) >= 30:
                break  # cap at 30 results

        if not results:
            _log_entry["found"] = 0
            _log_entry["results"] = []
            turn_log.append(_log_entry)
            return json.dumps(
                {"query": query, "pack": pack, "found": 0,
                 "hint": 'Попробуй: "kick", "snare", "hat", "bass", "synth", "dist", "loop", "*" (обзор)'},
                ensure_ascii=False,
            )

        _log_entry["found"] = len(results)
        _log_entry["results"] = results
        turn_log.append(_log_entry)
        return json.dumps(
            {"query": query, "pack": pack, "case": case, "found": len(results), "results": results},
            ensure_ascii=False,
            indent=2,
        )

    @function_tool
    def execute_music_code(code: str, pattern_name: str | None = None) -> str:
        """Выполнить Renardo-код для создания или изменения музыкального паттерна.
        
        Args:
            code: Строка Renardo-кода, например: 'p1 >> pluck([0, 2, 4], dur=0.5)'
            pattern_name: Имя паттерна для истории (p1, bass, drums и т.д.)
        """
        result = manager.execute_code(code, pattern_name)
        ok = result.get("success", False)
        entry = {"name": "execute_music_code", "code": code, "pattern_name": pattern_name,
                 "ok": ok, "error": result.get("error", "")}
        turn_log.append(entry)
        return json.dumps(result, ensure_ascii=False)

    @function_tool
    def stop_music(pattern_name: str | None = None) -> str:
        """Остановить паттерн по имени или всю музыку.
        
        Args:
            pattern_name: Имя паттерна ('p1', 'bass') или 'all' / пусто для остановки всего
        """
        if not pattern_name or pattern_name.strip().lower() == "all":
            result = manager.stop_all()
        else:
            result = manager.stop_pattern(pattern_name)
        turn_log.append({"name": "stop_music", "pattern_name": pattern_name,
                         "ok": result.get("success", False)})
        return json.dumps(result, ensure_ascii=False)

    @function_tool
    def set_vibe_preset(preset_name: str) -> str:
        """Применить вайб-пресет: chill, energetic, ambient, jazz, dark.
        
        Args:
            preset_name: Одно из: chill, energetic, ambient, jazz, dark
        """
        result = manager.set_vibe_preset(preset_name)
        entry = {"name": "set_vibe_preset", "preset_name": preset_name,
                 "ok": result.get("success", False),
                 "preset": result.get("preset", {})}
        turn_log.append(entry)
        return json.dumps(result, ensure_ascii=False)

    @function_tool
    def get_music_state() -> str:
        """Получить текущее состояние: SC доступность, активные паттерны, история."""
        state = manager.get_state()
        turn_log.append({"name": "get_music_state", "ok": True})
        parts = [
            f"SC запущен: {state['supercollider_running']}",
            f"renardo: {state['renardo_available']}",
            f"Пресет: {state['current_preset'] or 'нет'}",
            f"Активные паттерны: {', '.join(state['active_patterns']) or 'нет'}",
        ]
        return "\n".join(parts)

    @function_tool
    async def generate_tts_sample(text: str, letter: str = "v") -> str:
        """Сгенерировать голосовой семпл из текста (TTS) и добавить в renardo-семплы.

        Args:
            text: Текст для озвучки, например 'кек', 'огонь', 'привет роботы'
            letter: Буква-идентификатор папки в renardo (по умолчанию 'v' = voice)
        Returns:
            JSON с путём к файлу и полем play_code для воспроизведения
        """
        try:
            import edge_tts  # noqa: PLC0415
        except ImportError:
            return json.dumps({"success": False, "error": "pip install edge-tts"})

        try:
            import shutil
            import subprocess
            import tempfile

            letter = letter.lower()[0]
            samples_base = (
                Path.home() / "AppData" / "Roaming" / "renardo" / "samples" / "0_foxdot_default"
            )
            sample_dir = samples_base / letter.upper()
            sample_dir.mkdir(parents=True, exist_ok=True)

            existing = sorted(sample_dir.glob(f"{letter}*.wav"))
            idx = len(existing)
            out_path = sample_dir / f"{letter}{idx}.wav"

            # edge-tts генерит MP3 → конвертируем в WAV (SC не читает MP3)
            with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as tmp:
                tmp_mp3 = tmp.name
            try:
                tts = edge_tts.Communicate(text, voice="ru-RU-DmitryNeural")
                await tts.save(tmp_mp3)

                # Ищем ffmpeg
                ffmpeg_exe = shutil.which("ffmpeg") or (
                    Path(r"C:\Users\krikz\AppData\Local\Microsoft\WinGet\Packages")
                    / "Gyan.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe"
                    / "ffmpeg-8.0.1-full_build" / "bin" / "ffmpeg.exe"
                )
                # loudnorm: нормализация до -12 LUFS (громко и чётко слышно)
                # ar 44100 / pcm_s16le: стандарт WAV для SuperCollider
                subprocess.run(
                    [
                        str(ffmpeg_exe), "-y", "-i", tmp_mp3,
                        "-filter:a", "loudnorm=I=-12:LRA=7:TP=-2",
                        "-ar", "44100",
                        "-acodec", "pcm_s16le",
                        str(out_path),
                    ],
                    capture_output=True,
                    check=True,
                )
            finally:
                Path(tmp_mp3).unlink(missing_ok=True)

            play_code = f'p_voice >> play("{letter}", sample={idx}, amp=1.5)'
            if not dry_run:
                # Перезагружаем семплы в renardo чтобы новый файл был виден
                try:
                    ctx = manager._ctx if hasattr(manager, "_ctx") else {}
                    Samples = ctx.get("Samples")
                    if Samples and hasattr(Samples, "reload"):
                        Samples.reload()
                except Exception:
                    pass  # не критично — семпл всё равно будет виден после рестарта

            print(f"\n[TTS] 🎤 Семпл '{text}' → {out_path.name} (play: {play_code})")
            return json.dumps(
                {
                    "success": True,
                    "file": str(out_path),
                    "index": idx,
                    "play_code": play_code,
                    "message": f"Семпл '{text}' сохранён как {out_path.name}. Воспроизведи: {play_code}",
                },
                ensure_ascii=False,
            )
        except Exception as e:
            return json.dumps({"success": False, "error": str(e)}, ensure_ascii=False)

    # ── Создаём агента ─────────────────────────────────────────────────────

    client = AsyncOpenAI(
        api_key=DEEPSEEK_API_KEY,
        base_url=DEEPSEEK_BASE_URL,
    )
    model = OpenAIChatCompletionsModel(
        model=MODEL_NAME,
        openai_client=client,
    )

    agent = Agent(
        name="RobMusicAgent",
        instructions=SYSTEM_PROMPT,
        tools=[search_samples, execute_music_code, stop_music, set_vibe_preset, get_music_state],
        model=model,
    )

    # ── История разговора ──────────────────────────────────────────────────
    history = []

    mode_label = " [DRY RUN]" if dry_run else ""
    print(f"\n{'='*60}")
    print(f"🎵 Музыкальный LLM-тест{mode_label}")
    print(f"   Модель: {MODEL_NAME}")
    print(f"   Режим: {'dry-run (код только логируется)' if dry_run else 'live (SC на localhost:57110)'}")
    print(f"   Команды: 'quit' | 'stop' (вся музыка) | 'state' (статус)")
    print(f"{'='*60}\n")
    print("Примеры запросов:")
    print("  → сыграй что-нибудь джазовое")
    print("  → добавь барабаны")
    print("  → сделай бодрее")
    print("  → страшная атмосфера заброшенной фермы")
    print("  → тёмный эмбиент с глитчем")
    print("  → весёлый 8-бит")
    print("  → стоп")
    print()

    while True:
        try:
            user_input = input("Ты: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n\nВыход")
            if not dry_run:
                manager.stop_all()
            break

        if not user_input:
            continue

        if user_input.lower() in ("quit", "exit", "выход"):
            if not dry_run:
                manager.stop_all()
            break

        if user_input.lower() in ("stop", "стоп", "тихо"):
            result = manager.stop_all()
            print(f"⏹  {result['message']}")
            continue

        if user_input.lower() in ("state", "статус", "что играет"):
            state = manager.get_state()
            print(f"SC: {state['supercollider_running']} | renardo: {state['renardo_available']}")
            print(f"Пресет: {state['current_preset'] or 'нет'}")
            print(f"Активные: {', '.join(state['active_patterns']) or 'нет'}")
            continue

        # Добавить в историю
        history.append({"role": "user", "content": user_input})

        print("🤖 Роб думает...")
        try:
            turn_log.clear()
            result = await Runner.run(agent, input=history, max_turns=30)
            response_text = result.final_output or ""

            # Обновить историю для следующего хода
            history = result.to_input_list()

            # ── Показать что нагенерила нейронка ──────────────────────────
            _print_turn_log(turn_log)

            print(f"Роб: {response_text}")

        except Exception as e:
            print(f"❌ Ошибка LLM: {e}")
            # Убрать последний user message если ошибка
            if history and history[-1]["role"] == "user":
                history.pop()


# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Music LLM Concept Test")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Не подключаться к SC/renardo — только логировать вызовы инструментов",
    )
    parser.add_argument(
        "--model",
        default="deepseek-chat",
        help="Модель DeepSeek (deepseek-chat | deepseek-reasoner)",
    )
    args = parser.parse_args()

    global MODEL_NAME
    MODEL_NAME = args.model

    manager = create_manager(dry_run=args.dry_run)
    asyncio.run(run_repl(manager, dry_run=args.dry_run))


if __name__ == "__main__":
    main()
