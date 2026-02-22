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
    - execute_music_code(code): выполнить Renardo-код (создать/обновить паттерн)
    - stop_music(pattern_name): остановить паттерн или всю музыку
    - set_vibe_preset(vibe): быстро задать вайб (chill/energetic/ambient/jazz/dark)
    - get_music_state(): узнать текущее состояние
    - generate_tts_sample(text, letter): сгенерить слово/фразу через TTS.
      Возвращает play_code — его сразу передай в execute_music_code.

    Пример TTS: generate_tts_sample("кек", "v") → play_code: 'p_voice >> play("v", sample=0, amp=1.5)'

    ═══════════════════════════════════════════
    ПОЛНАЯ ДОКУМЕНТАЦИЯ RENARDO:
    ═══════════════════════════════════════════
    {_RENARDO_REF}
    ═══════════════════════════════════════════

    ⚠️ КРИТИЧНО — ОШИБКА КОТОРУЮ НЕЛЬЗЯ ДЕЛАТЬ НИКОГДА:
    - НИКОГДА не пиши `Clock.bpm.set(N)` — bpm это int, у него нет .set()!
    - НИКОГДА не пиши `lambda: Clock.bpm = N` — в lambda нельзя присваивать!
    - Для смены BPM в Clock.future() используй ТОЛЬКО:
        a) `Clock.future(N, lambda: setattr(Clock, 'bpm', 170))`  ← правильно
        b) `def fn(): Clock.bpm = 170\nClock.future(N, fn)`        ← правильно
    - То же самое для Scale/Root в lambda: `lambda: setattr(Scale, 'default', 'minor')`

    Стратегия:
    1. Сначала вызови set_vibe_preset если пользователь описывает настроение
    2. Затем execute_music_code — создавай несколько паттернов сразу (бас + ритм + мелодия)
    3. Используй реальные renardo-конструкции: var(), linvar(), PDur(), PRand(), Group(),
       .every(), .follow(), .fadein()/.fadeout(), .eclipse() — делай КРУТУЮ музыку!
    4. Если просят озвучить слово/фразу — используй generate_tts_sample
    5. Отвечай кратко — просто играй, не объясняй каждую строчку кода

    ВАЖНО: Всегда вызывай инструменты немедленно. Никогда не имитируй — всегда реально играй.
""")


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

    # ── Определяем function tools ──────────────────────────────────────────

    @function_tool
    def execute_music_code(code: str, pattern_name: str | None = None) -> str:
        """Выполнить Renardo-код для создания или изменения музыкального паттерна.
        
        Args:
            code: Строка Renardo-кода, например: 'p1 >> pluck([0, 2, 4], dur=0.5)'
            pattern_name: Имя паттерна для истории (p1, bass, drums и т.д.)
        """
        result = manager.execute_code(code, pattern_name)
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
        return json.dumps(result, ensure_ascii=False)

    @function_tool
    def set_vibe_preset(preset_name: str) -> str:
        """Применить вайб-пресет: chill, energetic, ambient, jazz, dark.
        
        Args:
            preset_name: Одно из: chill, energetic, ambient, jazz, dark
        """
        result = manager.set_vibe_preset(preset_name)
        return json.dumps(result, ensure_ascii=False)

    @function_tool
    def get_music_state() -> str:
        """Получить текущее состояние: SC доступность, активные паттерны, история."""
        state = manager.get_state()
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
        tools=[execute_music_code, stop_music, set_vibe_preset, get_music_state, generate_tts_sample],
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
    print("  → скажи 'кек' и сыграй это в ритм")
    print("  → озвучь слово 'огонь' и зациклируй")
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
            result = await Runner.run(agent, input=history, max_turns=30)
            response_text = result.final_output or ""
            print(f"Роб: {response_text}")

            # Обновить историю для следующего хода
            history = result.to_input_list()

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
