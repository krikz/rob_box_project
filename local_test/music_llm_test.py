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

            # ── Monkey-patch: заставляем getBufferFromSymbol реально использовать spack ──
            # По умолчанию renardo игнорирует spack и всегда играет из 0_foxdot_default.
            # Этот патч позволяет play("c", spack=1) выбирать 1_pitchglitch_samples.
            try:
                from renardo_gatherer.collections import SAMPLES_DIR_PATH, sample_path_from_symbol as _spts
                from renardo_lib.SynthDefManagement.BufferManagement import BufferManager, nil as _nil

                _spack_dirs = sorted([d for d in SAMPLES_DIR_PATH.iterdir() if d.is_dir()])

                def _patched_getBufferFromSymbol(self, symbol, spack, index=0):
                    if symbol.isspace():
                        return _nil
                    # spack int → индекс в отсортированном списке паков
                    if isinstance(spack, int) and 0 <= spack < len(_spack_dirs):
                        spack_path = _spack_dirs[spack]
                    else:
                        spack_path = _spack_dirs[0]  # default: 0_foxdot_default
                    sample_path = _spts(symbol, spack_path=spack_path)
                    if sample_path is None:
                        return _nil
                    sample_path = self._findSample(sample_path, index)
                    if sample_path is None:
                        return _nil
                    return self._allocateAndLoad(sample_path)

                BufferManager.getBufferFromSymbol = _patched_getBufferFromSymbol
                print(f"✅ Monkey-patch spack: паки = {[d.name for d in _spack_dirs]}")
            except Exception as e:
                print(f"⚠️  Monkey-patch spack не применён: {e}")

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

    ⚠️ КРИТИЧНО — ИСПОЛЬЗУЙ БУКВУ ТОЧНО ИЗ play_code РЕЗУЛЬТАТА search_samples:
    Если search_samples("snare") вернул:  d1 >> play("i", sample=2)
    → в паттерне используй "i":  d2 >> play("..i...i.", sample=2)
    ❌ ЗАПРЕЩЕНО подставлять "o" вместо "i" потому что "знаешь" что o=hihat!
       В renardo буква определяет ПАПКУ с семплами — только из search_samples!
    ⚠️ РЕГИСТР БУКВЫ: search_samples("kick") без case="upper" вернёт строчную "x" (папка lower/)
       Для заглавной "X" (папка upper/) используй: search_samples("kick", case="upper")
       Строчная "x" и заглавная "X" = РАЗНЫЕ ПАПКИ = разные звуки!
       ✅ Рекомендуется для кика: search_samples("kick", case="upper") → play("X.X.X.X.", ...)

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
      C4=60,D4=62,E4=64,F4=65,G4=67,A4=69,B4=71,C5=72 | C3=48,C2=36,C1=24
      ❌ C4=48 — НЕВЕРНО! C4 в MIDI = 60 (средняя октава)
    - НИКОГДА не используй `sinvar()` — такой функции в renardo НЕТ!
      ❌ pan=sinvar([-0.5, 0.5], 8)  ← NameError: name 'sinvar' is not defined!
      ✅ pan=var([-0.5, 0.5], 8) или pan=linvar(-0.5, 0.5, 16) — это СУЩЕСТВУЮЩИЕ функции
    - НИКОГДА не вызывай `.loop()` без аргумента: `P[...].loop()` → TypeError!
      Используй: `.loop(n)` где n — количество повторений, например `.loop(4)`
      ИЛИ просто не используй .loop() — паттерны в renardo зациклены по умолчанию
    - НИКОГДА не передавай одиночное число в var() первым аргументом:
      ❌ var(0, 4), var(0.5, 8), var(4, 2) → 'float' object is not iterable!
      ✅ ВСЕГДА список: var([0,4,5,3], 4), var([0.5,1.0], 8), var([4,5], 2)
      ✅ Исключение: dur= и sus= принимают одиночные числа, но var() — нет

    ПАЛИТРА СИНТЕЗАТОРОВ — используй РАЗНЫЕ инструменты для разных слоёв:
    🥁 Ударные/Lo:   d1-d9 >> play(...)   — всегда через play() с семплами
    🎸 Бас:          bass, wobblebass, dub, fuzz, dirt, subbass, moogbass  (oct=2..3)
    🎹 Мелодия/Lead: blip, arpy, pianovel, epiano, rhpiano, karp, sitar, marimba, keys, cs80lead
       ⚠️ НЕТ "piano" — используй pianovel / epiano / rhpiano!
       ⚠️ pluck = гитарный щипок, НЕ духовые/оркестр! Для трубы/флейты — blip/flute/brass
    🎺 Оркестр/Духовые: brass, blip, flute, soprano, eoboe, organ, organ2
    🎻 Струнные/Пэды: strings, pads, ambi, space, faim, sinepad, mhpad, ecello, viola
    🔔 Атмосфера/Bells: bell, gong, kalimba, marimba, steeldrum, tubularbell  (amp=0.3..0.6)
    ⚡ Агрессия/Глитч: rave, donk, varsaw, pulse, quin, feel, tb303, hoover

    Примеры правильного слоения:
      d1 >> play("x-o-")                              # ударные
      p1 >> bass([0,-2,0,-2], oct=2, dur=1)           # бас (НЕ pluck!)
      p2 >> brass([0,2,4,2], oct=4, dur=0.5)          # духовые мелодия
      p3 >> strings([0], dur=4, amp=0.3, sus=6)       # струнные пэд

    ⚠️ ПРАВИЛО РАЗНООБРАЗИЯ — ОБЯЗАТЕЛЬНО:
    - НИКОГДА не используй pluck для всех паттернов! pluck = только 1 паттерн максимум
    - pluck = гитарный звук, НЕ использовать для оркестра/марша/духовых!
    - Каждый трек = минимум 3 слоя РАЗНЫМИ инструментами из РАЗНЫХ категорий
    - Бас → bass/wobblebass/dub, НЕ pluck
    - Атмосфера → pads/ambi/strings/space, НЕ pluck
    - Перкуссия всегда через d1-d9 >> play()

    ⚠️ ПРАВИЛО УЗНАВАЕМЫХ МЕЛОДИЙ — при запросе конкретной песни/марша:
    - Root.default КРИТИЧЕН! «Имперский марш» = Root.default = "G", Scale.default = "minor"
    - Интервалы определяют узнаваемость: повторяй 3 ноты + прыжок характерный
    - Не выдумывай интервалы — если не уверен в мелодии, сделай атмосферную версию
    - Для маршей: барабаны dur=0.5 с паттерном "X.X.X.X." или "X.oX.o", BPM 80-120

    🔇 ПРАВИЛО ПРОСТОТЫ ДЛЯ ИЗВЕСТНЫХ МЕЛОДИЙ:
    ❌ НЕ делай 5-6 паттернов (p1-p5, d1-d4) — они перекрывают мелодию!
    ✅ МАКСИМУМ 3 слоя: мелодия + лёгкий бас + барабаны
    ✅ Иерархия громкости ОБЯЗАТЕЛЬНА:
       - Мелодия/brass: amp=0.8-1.0 (самая громкая!)
       - Бас аккомпанемент: amp=0.2-0.3 (тихий фон)
       - Кик барабан: amp=0.3-0.4 (семплы ГРОМЧЕ синтезаторов!)
       - Снейр/хэт: amp=0.2-0.3
    ⚠️ ВАЖНО: play()-семплы звучат в 2-3× громче synth той же amp=!
       amp=0.7 для барабана = заглушает мелодию! Используй amp=0.3-0.4 для ударных!
    ❌ ЗАПРЕЩЕНО: p2 >> bass(...) — synth `bass` = "пердящая" бас-гитара с distortion!
       ✅ Для низкого баса: p2 >> brass(midinote=[43,43], dur=4, oct=3, amp=0.25) или пропусти
    ❌ НЕ добавляй `shape=` к brass/strings — вызывает искажение звука
    ❌ НЕ добавляй blip/gong/extra layers когда есть главная мелодия brass
    ❌ ЗАПРЕЩЕНО: play("..o...o.") для снейра — "o" в renardo НЕ снейр! "o" = другой семпл-папки!
       В renardo буква "o" ≠ open hihat из классических драм-машин — это ДРУГОЙ звук!
       ✅ Снейр = буква "i" (всегда! из search_samples("snare") → play("i", ...))
    ✅ Правильный минимальный Имперский марш (главная тема A+B, BPM=96 из MIDI):
       Clock.bpm = 96
       p1 >> brass(midinote=[67,67,67, 63,70,67, 63,70,67,      # A: DA DA DA DUM-da-DUM
                              74,74,74, 75,70,66, 63,70,67],     # B: DA DA DA DUM-da-DUM (выше)
                   dur=[1.5,0.5,1, 1,0.5,1, 1,0.5,2,
                        1.5,0.5,1, 1,0.5,1, 1,0.5,2],
                   amp=0.9, sus=1.5, room=0.3)
       d1 >> play("X.X.X.X.", sample=1, amp=0.35, room=0.1)  # кик X = из search_samples kick
       d2 >> play("..i...i.", sample=3, amp=0.25, room=0.1)  # снейр i = из search_samples snare
    ⚠️ УДАРНЫЕ ГРОМКОСТНЫЕ НОРМЫ — семплы в SC изначально ГРОМЧЕ синтезаторов той же amp!
       Если слышны в основном барабаны — значит их amp завышен!
       ✅ Кик: amp=0.3..0.4 (НЕ 0.7-0.8!)
       ✅ Снейр: amp=0.2..0.3 (НЕ 0.5-0.6!)
       ✅ Хэт: amp=0.15..0.25
       ✅ Мелодия/brass: amp=0.8..1.0 — должна доминировать!

    🎼 ИМПЕРСКИЙ МАРШ — точные MIDI ноты (ALL exact, no scale approximations):
    ⚠️ НЕ используй scale degrees для Имперского марша — мелодия содержит хроматику (Gb, Ab)!
    ⚠️ ОШИБКА: [0,0,0, 5,-5,0] — неверно! degree 5 в G minor это Eb ВЫШЕ G (большая секста),
       а мелодия идёт ВНИЗ на малую терцию G→Eb и Bb это ВЫШЕ G, не ниже.
    ✅ ПРАВИЛЬНО — используй midinote= (Root/Scale не влияют на midinote!):
       Фраза A: G4  G4  G4  | Eb4  Bb4  G4  | Eb4  Bb4  G4
                67  67  67     63   70   67    63   70   67
                dur=[1.5,0.5,1, 1,0.5,1, 1,0.5,2]
       Фраза B: D5   D5   D5  | Eb5  Bb4  Gb4 | Eb4  Bb4  G4
                74   74   74    75   70   66    63   70   67
                dur=[1.5,0.5,1, 1,0.5,1, 1,0.5,2]
       Фраза C (проигрыш — бегущий хроматический пассаж):
                G5  G4  G4  G5  F#5  F5  E5  Eb5  E5  Ab4  C#5  C5  B4  Bb4  A4  Bb4
                79  67  67  79   78  77  76   75  76   68   73  72  71   70  69   70
                dur=[0.5,0.25,0.25, 0.75,0.5,0.25,0.25,0.25,0.25, 0.25,0.75,0.5,0.25,0.25,0.25,0.25]
       Фраза D (финал проигрыша — выход на тему):
                Eb4  Gb4  Eb4  Gb4  Bb4  G4  Eb4  Bb4  D5
                 63   66   63   66   70  67   63   70   74
                dur=[0.5,0.5, 0.5,0.25,0.75, 0.5,0.25,0.25,2]
    ✅ Правильный код (A+B = тема, 3 слоя максимум):
       Clock.bpm = 96
       Root.default = "G"
       Scale.default = "minor"
       p1 >> brass(
           midinote=[67,67,67, 63,70,67, 63,70,67, 74,74,74, 75,70,66, 63,70,67],
           dur=[1.5,0.5,1, 1,0.5,1, 1,0.5,2, 1.5,0.5,1, 1,0.5,1, 1,0.5,2],
           amp=0.9, sus=1.5, room=0.3
       )
       d1 >> play("X.X.X.X.", sample=1, amp=0.35, room=0.1)  # кик
       d2 >> play("..i...i.", sample=3, amp=0.25, room=0.1)  # снейр
    ❌ ЗАПРЕЩЕНО для Имперского марша:
       ❌ p2 >> bass(...) — bass синтезатор = "пердящая" бас-гитара с distortion, РАЗРУШАЕТ марш!
       ❌ p3 >> strings(degree=[...]) — лишний слой, заглушает brass мелодию!
       ❌ p4 >> brass(degree=[...], oct=5) — фанфары поверх главной темы = каша!
       ❌ shape= — вызывает искажение звука у brass/strings!
       ❌ .every(8, "stutter", 2) — случайная трансформация ломает маршевый ритм!
    ✅ ТОЛЬКО: p1(brass мелодия) + d1(кик) + d2(снейр) = 3 слоя, больше НЕ НУЖНО!
    ⚠️ НЕ добавляй oct= когда используешь midinote= — midinote уже абсолютный!
    ⚠️ Root.default и Scale.default ВСЕГДА устанавливай В НАЧАЛЕ кода, перед паттернами!
    Справка MIDI: G4=67, Eb4=63, Bb4=70, D5=74, Eb5=75, Gb4=66, G5=79, F#5=78, Ab4=68, Db5=73

    🎄 В ЛЕСУ РОДИЛАСЬ ЁЛОЧКА — точные MIDI ноты (F major, 3/4 вальс, BPM=72):
    ⚠️ Тональность F major (НЕ C major!) Мелодия начинается на A4(69), НЕ G4!
    Фраза 1 «В лесу родилась ёлочка, зимой и летом стройная»:
       A4  A4  G4  A4  F4  C4  C4  C4  A4  A4  Bb4  G4  C5
       69  69  67  69  65  60  60  60  69  69   70   67  72
    Фраза 2 «Она была пушистая, стройная была»:
       C5  F4  F4  Bb4 Bb4  A4  G4  F4  F4  A4  A4  G4  A4  F4
       72  65  65   70  70  69  67  65  65  69  69  67  69  65
    ✅ Правильный код (BPM=72, dur=0.5 — все ноты восьмые):
       Clock.bpm = 72
       p1 >> bell(
           midinote=[69,69,67,69,65, 60,60,60, 69,69,70,67,72,
                     72,65,65,70,70, 69,67,65,65, 69,69,67,69,65],
           dur=0.5,
           amp=0.85, sus=0.6, room=0.3
       )
       p2 >> pads([(0,4,7)], dur=3, oct=4, amp=0.2, room=0.5)  # лёгкая гармония
       d1 >> play("X..", sample=0, amp=0.3)                    # вальс: кик на 1-ю долю
    ❌ ЗАПРЕЩЕНО для ёлочки/вальса: p3 >> arpy(...), p4 >> bass(...) — НАРУШАЕТ правило ≤3 слоёв!
       ❌ bass синтезатор = "пердящая" бас-гитара с distortion — звучит ужасно в вальсе!
       ❌ arpy = лишний слой поверх bell, заглушает мелодию!
       ✅ ТОЛЬКО: p1(bell мелодия) + p2(pads гармония) + d1(барабаны) = 3 слоя максимум
    ❌ ЗАПРЕЩЕНО: delay= для разбивки мелодии на части!
       p1>>bell([фраза1], delay=0); p2>>bell([фраза2], delay=9) — АНТИПАТТЕРН!
       После первого прохода p1 начнётся с нуля, p2 со смещением — ВСЁ РАЗЪЕДЕТСЯ!
    ✅ ВСЕ НОТЫ мелодии = ОДИН паттерн: p1>>bell(midinote=[нота1,нота2,...все_ноты_подряд...])
    ⚠️ НЕ добавляй oct= — midinote уже содержит абсолютные ноты!
    ⚠️ Используй bell (НЕ bells — такого синтезатора НЕ существует!)

    � ТАБЛИЦА НОТ → SCALE DEGREES (С-major/minor, Root.default="C"):
    ⚠️ Scale.default = "major" (дефолт) — 7 ступеней диатонической гаммы:
      До=C=0, Ре=D=1, МИ=E=2, ФА=F=3, СОЛЬ=G=4, ЛЯ=A=5, СИ=B=6
      Следующая октава: До2=C2=7, Ре2=8 ...
    ⚠️ ОШИБКА: [4,5] в C major = G,A (Соль,Ля) — НЕ E,F!
      МИ+ФА = [2,3], ЛЯ+СИ = [5,6], ДО+РЕ = [0,1]
    - Если нужны конкретные ноты вне зависимости от Scale → используй midinote=:
      C4=60, D4=62, E4=64, F4=65, G4=67, A4=69, B4=71, C5=72
      C#/Db=61, D#/Eb=63, F#/Gb=66, G#/Ab=68, A#/Bb=70
    - Для хроматических мелодий (все 12 полутонов): Scale.default = Scale.chromatic
      тогда [0,1,2,3,4,5,6,7,8,9,10,11] = C,C#,D,D#,E,F,F#,G,G#,A,A#,B

    �🎤 ПРАВИЛО ВОКАЛА — если хочешь слышимый вокал:
    ❌❌❌ НИКОГДА НЕ УГАДЫВАЙ SAMPLE INDEX! ❌❌❌
    → ВСЕГДА сначала вызови search_samples("vocal", pack="1_pitchglitch_samples") —
      без этого ты НЕ знаешь что лежит под каким номером! Не пиши sample=9 и
      в комментарии "Laaaoooaaa" — это ложь, ты не знаешь что там.

    ⚠️ ОБЯЗАТЕЛЬНО ИСПОЛЬЗУЙ spack= В play():
      spack=0 → 0_foxdot_default (ударные/перкуссия для букв a-z)
      spack=1 → 1_pitchglitch_samples (вокал, хор, этнические звуки)
      search_samples() вернёт готовый play_code С ПРАВИЛЬНЫМ spack= — просто скопируй!

    - amp МИНИМУМ 0.5, иначе вокал утонет в миксе (НЕ 0.2-0.3!)
    - Короткие семплы (dur файла ~0.5-2 сек) → dur=1, sus=1, amp=0.6
      Пример: d4 >> play("c", sample=1, spack=1, dur=1, sus=1, amp=0.6, room=0.3)
    - Длинные хор/сустейн (dur файла ~4-8 сек) → dur=6, sus=6, amp=0.55
      Пример: d4 >> play("c", sample=11, spack=1, dur=6, sus=6, amp=0.55, room=0.5)
    - Когда вокал в миксе — снизь pads до amp=0.2 чтобы не перекрывали
    - Используй КОНКРЕТНЫЙ sample=N (из search_samples), НЕ PRand для вокала —
      PRand даст непредсказуемую длину и вокал будет обрезаться или не в такт
    ❌ ЗАПРЕЩЕНО: d1 >> play("c", ...).fadein(16)  ← вокал не слышен 12 секунд!
    ❌ ЗАПРЕЩЕНО: d1 >> play("c", ...).fadein(24)  ← вокал не слышен 18 секунд!
    ✅ ПРАВИЛЬНО:  d1 >> play("c", sample=N, spack=1, dur=2, sus=2, amp=0.6, room=0.4)
    ✅ МОЖНО:      d1 >> play("c", sample=N, spack=1).fadein(4)   ← максимум 4 такта
    - ❌❌❌ ОБЯЗАТЕЛЬНО ДОБАВЛЯЙ sus= ДЛЯ ВОКАЛА! ❌❌❌
      Без sus= дефолт = sus=1 (0.75 сек при 80 BPM) — длинный семпл ОБРЕЗАЕТСЯ!
      sus= должен СОВПАДАТЬ с dur=:
        Короткий вокал: dur=2, sus=2, amp=0.6
        Длинный хор:    dur=8, sus=8, amp=0.55
      ✅ d1 >> play("c", sample=N, spack=1, dur=8, sus=8, amp=0.55, room=0.4)  ← ПРАВИЛЬНО
      ❌ d1 >> play("c", sample=N, spack=1, dur=8,        amp=0.55, room=0.4)  ← ОБРЕЖЕТ ДО 1 ТАКТА!
      ❌ d1 >> play("c", sample=N,          dur=8, sus=8, amp=0.55, room=0.4)  ← CONGA, НЕ ВОКАЛ!
    - ⚠️ dur= должен совпадать с длиной семпла! Короткий семпл (1-2 сек) с dur=8
      играет раз в 6 секунд — большинство времени тишина. Для коротких семплов
      используй dur=1..2, для длинных (Laaaoooaaa, Choir) — dur=4..6

    📱 ПРАВИЛО NOKIA/RTTTL НОТАЦИИ — когда пользователь даёт нотацию вида "4E1 8.C1 16G1":
    - Длительности: `1`=целая(dur=4), `2`=половинная(dur=2), `4`=четверть(dur=1),
      `8`=восьмая(dur=0.5), `16`=шестнадцатая(dur=0.25), `32`=32я(dur=0.125)
    - Точка после длительности: dur × 1.5: `8.`=0.75, `4.`=1.5, `2.`=3
    - Пауза `8-` или `8r` → добавляй `rest(0.5)` в список нот, или 0 в midinote
    - ⚠️ ОКТАВЫ Nokia: "1"="2" в Nokia ≠ MIDI октаве! Nokia "1" = MIDI октава 4..5 (concert pitch)!
      MIDI для пьес в регистре E1..B2: прибавь 48 (4 октавы) → MIDI 28→76 (E5), 35→83 (B5)
      Т.е. все midinote значения из Nokia нотации нужно поднять на +48 для звучания в нормальном диапазоне
    - ⚠️ pluck = гитара, НЕ подходит для Nokia мелодий / духовых!
      ✅ Для Nokia/RTTTL используй: blip (синтетично как телефон), brass (оркестрально), organ (органно)
    - ⚠️❌ НИКОГДА НЕ ИСПОЛЬЗУЙ oct= вместе с midinote= — они КОНФЛИКТУЮТ!
      oct= ИГНОРИРУЕТ midinote= и ломает высоту звука!
      ❌ p1 >> blip(midinote=[67,64,65], oct=5, ...) ← oct= ЛОМАЕТ ВЫСОТУ!
      ✅ p1 >> blip(midinote=[67,64,65], ...)         ← только midinote=, без oct=
      Если звучит слишком низко — прибавь 12 к каждому midinote (G4=67 → G5=79).
    - ⚠️ НИКОГДА не выдумывай название песни по нотам! Если узнаёшь — назови; если нет — просто сыграй.
      Нотация `E E E C↑ G↓ E C↑ G↓ E B B B C↑ G↓ Eb C↑ G↓ E` = ИМПЕРСКИЙ МАРШ (Star Wars)!
    - Пример правильной конвертации Nokia → renardo:
      `4E1 4E1 4E1 8.C1 16G1 4E1 8.C1 16G1 2E1` →
      midinote=[76,76,76, 72, 79, 76, 72, 79, 76], dur=[1, 1, 1, 0.75, 0.25, 1, 0.75, 0.25, 2]
      p1 >> brass(midinote=[76,76,76,72,79,76,72,79,76], dur=[1,1,1,0.75,0.25,1,0.75,0.25,2], amp=0.9)

    Стратегия:
    1. Сначала вызови set_vibe_preset если пользователь описывает настроение
    2. Затем execute_music_code — создавай минимум 3 паттерна: drums + bass + melody (+atmosphere)
    3. Используй реальные renardo-конструкции: var(), linvar(), PDur(), PRand(), Group(),
       .every(), .follow(), .fadeout(), .eclipse() — делай КРУТУЮ музыку!
       ⚠️ var(values, durations) — КОЛИЧЕСТВО элементов в обоих списках ДОЛЖНО СОВПАДАТЬ!
          ❌ var([60, 62, 64], [4, 2]) — 3 значения, 2 длительности → IndexError краш!
          ✅ var([60, 62, 64], [4, 2, 2]) — 3 значения, 3 длительности → OK
          ✅ var([60, 64], [4, 4]) — 2 и 2 → OK
       ⚠️ .fadein() — ТОЛЬКО для p1-p9 (синтезаторов), НИКОГДА для d1-d9 (семплов/вокала)!
       ⚠️ .fadein(N) — МАКСИМУМ 4 такта! fadein(8) = 24 секунды тишины — пользователь ждёт!
          ❌ ЗАПРЕЩЕНО: .fadein(8), .fadein(12), .fadein(16) — слишком долго!
          ✅ ПРАВИЛЬНО: .fadein(2) или .fadein(4) — или вообще без fadein!
    4. Для голосовых семплов — СНАЧАЛА вызови search_samples("vocal", pack="1_pitchglitch_samples"),
       затем используй play_code из результата — он уже содержит правильный spack=1!
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
            found_int = entry.get("found") if isinstance(entry.get("found"), int) else 0
            if found_int > 5:
                print(f"  │      ... ещё {found_int - 5} результатов")

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

        # Номер пака (spack=N в play()) — 0-based индекс в отсортированном списке паков.
        # После monkey-patch play("c", sample=N, spack=1) корректно играет из 1_pitchglitch_samples.
        all_packs = sorted([d.name for d in samples_root.iterdir() if d.is_dir()])
        spack_num = all_packs.index(pack) if pack in all_packs else 0
        spack_suffix = f", spack={spack_num}" if spack_num != 0 else ""

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
                        "spack": spack_num,
                        "filename": f.name,
                        "play_code": f'd1 >> play("{play_letter}", sample={idx}{spack_suffix})',
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
