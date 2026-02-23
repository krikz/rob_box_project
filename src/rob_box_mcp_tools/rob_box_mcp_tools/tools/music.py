#!/usr/bin/env python3
"""
music.py - Инструменты для управления музыкой в реальном времени через Renardo

Модуль предоставляет:
- MusicManager: Базовый класс управления Renardo (история паттернов, SC-проверка, пресеты, фильтрация кода)
- ExecuteMusicCodeTool: Выполнить Renardo-код в безопасном контексте
- StopMusicTool: Остановить паттерны или всю музыку
- SetVibePresetTool: Применить вайб-пресет (скейл, темп, тоника)
- GetMusicStateTool: Получить текущее состояние музыки и историю паттернов
"""

import re
import socket
from typing import Any, Dict, List, Optional, Tuple

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType

# ---------------------------------------------------------------------------
# Safety filter — compiled once at import time
# ---------------------------------------------------------------------------

_BLOCKED_TOKENS = re.compile(
    r"\b("
    r"import|os|sys|subprocess|shutil|socket|requests|urllib|http|ftplib|"
    r"importlib|builtins|__import__|__builtins__|__class__|__subclasses__|"
    r"open|exec|eval|compile|globals|locals|vars|getattr|setattr|delattr"
    r")\b"
)


# ---------------------------------------------------------------------------
# MusicManager
# ---------------------------------------------------------------------------


class MusicManager:
    """Управляет интеграцией с Renardo для LLM-контроля музыки в реальном времени.

    Возможности:
    - Безопасное выполнение Renardo-кода (execute_code)
    - История паттернов с возможностью мутации и остановки по имени
    - Проверка доступности SuperCollider перед воспроизведением
    - Вайб-пресеты для быстрой настройки скейла / BPM / тоники
    - Фильтрация опасных системных команд в пользовательском коде
    """

    #: Доступные вайб-пресеты: имя -> {scale, bpm, root}
    #: root — целое число полутонов от C (C=0, D=2, E=4, F=5, G=7, A=9, B=11)
    VIBE_PRESETS: Dict[str, Dict[str, Any]] = {
        "chill":     {"scale": "major",    "bpm": 85,  "root": 0},   # C
        "energetic": {"scale": "minor",    "bpm": 140, "root": 9},   # A
        "ambient":   {"scale": "dorian",   "bpm": 70,  "root": 2},   # D
        "jazz":      {"scale": "lydian",   "bpm": 120, "root": 5},   # F
        "dark":      {"scale": "phrygian", "bpm": 100, "root": 4},   # E
    }

    SC_HOST: str = "127.0.0.1"
    SC_PORT: int = 57110

    def __init__(self) -> None:
        #: pattern_name -> последний выполненный код
        self._pattern_history: Dict[str, str] = {}
        #: множество имён активных паттернов
        self._active_patterns: set = set()
        #: имя текущего пресета
        self._current_preset: Optional[str] = None
        #: контекст выполнения для renardo
        self._renardo_context: Dict[str, Any] = {}
        #: True если renardo доступен, False/None иначе
        self._renardo_available: Optional[bool] = None
        self._initialize_renardo()

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _initialize_renardo(self) -> None:
        """Попытка инициализировать Renardo-контекст и загрузить SynthDef-ы в SC.

        Pipeline:
        1. Создаём директории семплов (иначе renardo_lib.runtime падает при импорте).
        2. Импортируем renardo_lib.runtime.
        3. Подключаемся к scsynth через Server.init_connection().
        4. Создаём Group 1 в scsynth через raw OSC (иначе /s_new падает).
        5. Загружаем все SynthDef-ы: sdef.add() → write(.scd) + load() →
           OSC /foxdot → sclang компилирует .scd → /d_recv → scsynth.
        6. Ждём 5 секунд пока sclang скомпилирует все 188 SynthDef-ов.

        NOTE: SynthDefs — это plain dict, НЕ объект с методом .reload()!
        Правильный способ: for sdef in SynthDefs.values(): sdef.add()
        """
        import struct as _struct
        import time as _time

        def _send_osc_raw(address: str, *args) -> None:
            """Отправить raw OSC сообщение на scsynth (UDP 57110)."""
            msg = bytearray()
            addr_bytes = address.encode() + b"\x00"
            while len(addr_bytes) % 4:
                addr_bytes += b"\x00"
            types = b"," + b"".join(b"i" if isinstance(a, int) else b"f" for a in args) + b"\x00"
            while len(types) % 4:
                types += b"\x00"
            msg.extend(addr_bytes)
            msg.extend(types)
            for a in args:
                if isinstance(a, int):
                    msg.extend(_struct.pack(">i", a))
                elif isinstance(a, float):
                    msg.extend(_struct.pack(">f", a))
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as _s:
                _s.sendto(bytes(msg), (self.SC_HOST, self.SC_PORT))

        try:
            # renardo_lib.runtime при импорте пытается листить директории сэмплов.
            # Если 0_foxdot_default не установлен — падает FileNotFoundError.
            # Создаём пустую структуру директорий заранее, чтобы импорт проходил.
            import pathlib

            samples_base = pathlib.Path.home() / ".config" / "renardo" / "samples" / "0_foxdot_default"
            _SAMPLE_SUBDIRS = ["_", "_loop_"] + list("abcdefghijklmnopqrstuvwxyz")
            for subdir in _SAMPLE_SUBDIRS:
                (samples_base / subdir).mkdir(parents=True, exist_ok=True)

            # renardo_lib само по себе пустое; нужен renardo_lib.runtime
            import renardo_lib.runtime as _rt

            # Подключаемся к scsynth (Server.booted = True после этого)
            if not _rt.Server.booted:
                _rt.Server.init_connection()

            # Создаём Group 1 в scsynth — renardo отправляет все ноты в эту группу.
            # Без неё scsynth возвращает "Group 1 not found" на каждый /s_new.
            _send_osc_raw("/g_new", 1, 0, 0)

            # Загружаем все SynthDef-ы через sclang.
            # SynthDefs — это plain Python dict, НЕ объект с .reload()!
            # sdef.add() = write(.scd файл на диск) + load() (отправляет путь
            # через OSC /foxdot → sclang → компилирует → /d_recv → scsynth)
            for sdef in _rt.SynthDefs.values():
                sdef.add()

            # Ждём компиляции всех 188 SynthDef-ов через sclang.
            # Без паузы renardo сразу пытается играть, scsynth отвечает "not found".
            _time.sleep(5)

            self._renardo_context = vars(_rt).copy()
            self._renardo_available = True
        except (ImportError, Exception):
            self._renardo_available = False
            self._renardo_context = {}

    # ------------------------------------------------------------------
    # SuperCollider check
    # ------------------------------------------------------------------

    def _check_supercollider(self) -> bool:
        """Проверить, запущен ли SuperCollider, отправив OSC /status по UDP.

        scsynth слушает на UDP-порту SC_PORT. Отправляем минимальный OSC
        /status запрос и ждём ответа. TCP-проверка не подходит — scsynth
        по умолчанию принимает только UDP.

        Returns:
            True если scsynth отвечает на SC_PORT.
        """
        # Минимальный OSC /status: "/status\0" (8 байт) + ",\0\0\0" (4 байта)
        osc_status = b"/status\x00,\x00\x00\x00"
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.settimeout(1.0)
                sock.sendto(osc_status, (self.SC_HOST, self.SC_PORT))
                data, _ = sock.recvfrom(512)
                return len(data) > 0
        except OSError:
            return False

    # ------------------------------------------------------------------
    # Code safety filter
    # ------------------------------------------------------------------

    def _filter_code(self, code: str) -> Tuple[bool, str]:
        """Проверить код на наличие опасных конструкций.

        Args:
            code: Строка кода для проверки.

        Returns:
            (is_safe, error_message) — (True, "") если код безопасен.
        """
        match = _BLOCKED_TOKENS.search(code)
        if match:
            return False, f"Запрещённый токен в коде: '{match.group()}'"
        return True, ""

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def execute_code(self, code: str, pattern_name: Optional[str] = None) -> Dict[str, Any]:
        """Безопасно выполнить Renardo-код.

        Перед выполнением проверяется:
        1. Фильтр опасных конструкций.
        2. Доступность SuperCollider.
        3. Доступность библиотеки Renardo.

        Args:
            code: Строка Python/Renardo-кода.
            pattern_name: Имя паттерна для хранения в истории (опционально).

        Returns:
            dict с ключами ``success``, ``message`` (или ``error``), ``code``.
        """
        is_safe, filter_error = self._filter_code(code)
        if not is_safe:
            return {"success": False, "error": filter_error}

        if not self._check_supercollider():
            return {
                "success": False,
                "error": "SuperCollider не запущен. Запустите SuperCollider перед воспроизведением музыки.",
            }

        if not self._renardo_available:
            return {
                "success": False,
                "error": "Renardo недоступен. Установите пакет renardo_lib.",
            }

        try:
            exec(code, self._renardo_context)  # noqa: S102
        except Exception as exc:
            return {"success": False, "error": f"Ошибка выполнения: {exc}"}

        if pattern_name:
            self._pattern_history[pattern_name] = code
            self._active_patterns.add(pattern_name)

        return {"success": True, "message": "Код выполнен успешно", "code": code}

    def stop_pattern(self, pattern_name: str) -> Dict[str, Any]:
        """Остановить именованный паттерн.

        Не требует наличия паттерна в истории — LLM может вызвать stop для
        любого player (d1, p1, ...) даже если execute_code не сохранял по имени.

        Args:
            pattern_name: Имя паттерна/плеера (d1, p1, bass и т.д.).

        Returns:
            dict с ключами ``success`` и ``message`` (или ``error``).
        """
        stop_code = f"{pattern_name}.stop()"

        if self._renardo_available and self._check_supercollider():
            try:
                exec(stop_code, self._renardo_context)  # noqa: S102
            except Exception as exc:
                return {"success": False, "error": f"Ошибка остановки паттерна: {exc}"}

        self._active_patterns.discard(pattern_name)
        return {"success": True, "message": f"Паттерн '{pattern_name}' остановлен"}

    def stop_all(self) -> Dict[str, Any]:
        """Остановить всю музыку через Clock.clear().

        Returns:
            dict с ключами ``success`` и ``message`` (или ``error``).
        """
        if self._renardo_available and self._check_supercollider():
            try:
                exec("Clock.clear()", self._renardo_context)  # noqa: S102
            except Exception as exc:
                return {"success": False, "error": f"Ошибка остановки: {exc}"}

        self._active_patterns.clear()
        return {"success": True, "message": "Вся музыка остановлена"}

    def set_vibe_preset(self, preset_name: str) -> Dict[str, Any]:
        """Применить вайб-пресет (скейл, BPM, тоника).

        Если Renardo/SC недоступны, пресет запоминается для отложенного применения.

        Args:
            preset_name: Ключ из VIBE_PRESETS.

        Returns:
            dict с ключами ``success``, ``message``, ``preset`` (или ``error``).
        """
        if preset_name not in self.VIBE_PRESETS:
            available = ", ".join(self.VIBE_PRESETS)
            return {
                "success": False,
                "error": f"Пресет '{preset_name}' не найден. Доступные: {available}",
            }

        preset = self.VIBE_PRESETS[preset_name]
        self._current_preset = preset_name

        if self._renardo_available and self._check_supercollider():
            # Root.default принимает целое число (полутонов от C) или строку "C"
            # Root.C и подобные атрибуты НЕ существуют в Renardo!
            preset_code = (
                f"Clock.bpm = {preset['bpm']}\n"
                f"Scale.default = Scale.{preset['scale']}\n"
                f"Root.default = {preset['root']}"
            )
            try:
                exec(preset_code, self._renardo_context)  # noqa: S102
            except Exception as exc:
                return {"success": False, "error": f"Ошибка применения пресета: {exc}"}

        return {
            "success": True,
            "message": f"Пресет '{preset_name}' применён: scale={preset['scale']}, bpm={preset['bpm']}, root={preset['root']}",
            "preset": preset,
        }

    def get_state(self) -> Dict[str, Any]:
        """Вернуть текущее состояние музыкального менеджера.

        Returns:
            dict с полями renardo_available, supercollider_running,
            current_preset, pattern_history, active_patterns.
        """
        return {
            "renardo_available": self._renardo_available,
            "supercollider_running": self._check_supercollider(),
            "current_preset": self._current_preset,
            "pattern_history": dict(self._pattern_history),
            "active_patterns": list(self._active_patterns),
        }


# ---------------------------------------------------------------------------
# MCPTool wrappers
# ---------------------------------------------------------------------------


class ExecuteMusicCodeTool(MCPTool):
    """Выполнить Renardo/FoxDot-код для создания или изменения музыкального паттерна."""

    def __init__(self, node, manager: MusicManager) -> None:
        super().__init__(node)
        self._manager = manager

    @property
    def name(self) -> str:
        return "execute_music_code"

    @property
    def description(self) -> str:
        return (
            "Выполнить Renardo-код для создания или изменения музыкального паттерна в реальном времени. "
            "Код выполняется в контексте Renardo (FoxDot-совместимый синтаксис). "
            "Пример: 'p1 >> pluck([0, 2, 4], dur=0.5, amp=0.8)'. "
            "Перед выполнением проверяется доступность SuperCollider. "
            "Опасные системные команды автоматически блокируются. "
            "Укажи pattern_name чтобы паттерн можно было остановить или изменить позже."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="code",
                type="string",
                description="Строка Python/Renardo-кода для выполнения. Например: 'p1 >> pluck([0, 2, 4])'",
                required=True,
            ),
            MCPToolParameter(
                name="pattern_name",
                type="string",
                description=(
                    "Имя паттерна для хранения в истории (например: 'p1', 'bass', 'drums'). "
                    "Используется для последующей мутации или остановки паттерна."
                ),
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, code: str, pattern_name: Optional[str] = None) -> MCPToolResult:
        """Выполнить Renardo-код."""
        self.log_info(f"Выполнение музыкального кода: {code[:80]}...")
        result = self._manager.execute_code(code, pattern_name)
        if result["success"]:
            return MCPToolResult(success=True, data=result, message=result["message"])
        return MCPToolResult(success=False, error=result["error"])


class StopMusicTool(MCPTool):
    """Остановить музыкальный паттерн по имени или всю музыку сразу."""

    def __init__(self, node, manager: MusicManager) -> None:
        super().__init__(node)
        self._manager = manager

    @property
    def name(self) -> str:
        return "stop_music"

    @property
    def description(self) -> str:
        return (
            "Остановить музыкальный паттерн по имени или всю музыку. "
            "Если указан pattern_name — остановится только этот паттерн. "
            "Если pattern_name не указан или равен 'all' — остановится вся музыка (Clock.clear())."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="pattern_name",
                type="string",
                description=(
                    "Имя паттерна для остановки (например: 'p1', 'bass'). "
                    "Передай 'all' или оставь пустым для остановки всей музыки."
                ),
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, pattern_name: Optional[str] = None) -> MCPToolResult:
        """Остановить паттерн или всю музыку."""
        if not pattern_name or pattern_name.strip().lower() == "all":
            self.log_info("Остановка всей музыки")
            result = self._manager.stop_all()
        else:
            self.log_info(f"Остановка паттерна: {pattern_name}")
            result = self._manager.stop_pattern(pattern_name)

        if result["success"]:
            return MCPToolResult(success=True, data=result, message=result["message"])
        return MCPToolResult(success=False, error=result["error"])


class SetVibePresetTool(MCPTool):
    """Применить вайб-пресет для быстрой настройки скейла, BPM и тоники."""

    PRESET_NAMES: List[str] = list(MusicManager.VIBE_PRESETS.keys())

    def __init__(self, node, manager: MusicManager) -> None:
        super().__init__(node)
        self._manager = manager

    @property
    def name(self) -> str:
        return "set_vibe_preset"

    @property
    def description(self) -> str:
        presets_desc = ", ".join(
            f"{name} (scale={p['scale']}, bpm={p['bpm']})" for name, p in MusicManager.VIBE_PRESETS.items()
        )
        return (
            "Применить вайб-пресет для быстрой настройки музыкального контекста. "
            "Устанавливает скейл, BPM и тонику в Renardo одной командой. "
            f"Доступные пресеты: {presets_desc}. "
            "Устанавливает: Clock.bpm, Scale.default, Root.default (целое число полутонов от C)."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="preset_name",
                type="string",
                description="Имя пресета для применения",
                required=True,
                enum=self.PRESET_NAMES,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, preset_name: str) -> MCPToolResult:
        """Применить вайб-пресет."""
        self.log_info(f"Применение пресета: {preset_name}")
        result = self._manager.set_vibe_preset(preset_name)
        if result["success"]:
            return MCPToolResult(success=True, data=result, message=result["message"])
        return MCPToolResult(success=False, error=result["error"])


class GetMusicStateTool(MCPTool):
    """Получить текущее состояние музыки: доступность Renardo/SC, активные паттерны, история."""

    def __init__(self, node, manager: MusicManager) -> None:
        super().__init__(node)
        self._manager = manager

    @property
    def name(self) -> str:
        return "get_music_state"

    @property
    def description(self) -> str:
        return (
            "Получить текущее состояние музыкального менеджера: "
            "доступность Renardo и SuperCollider, список активных паттернов, "
            "историю кода паттернов и применённый пресет."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def read_only(self) -> bool:
        return True

    @property
    def destructive(self) -> bool:
        return False

    def execute(self) -> MCPToolResult:
        """Вернуть текущее состояние музыки."""
        state = self._manager.get_state()
        parts = [
            f"Renardo: {'доступен' if state['renardo_available'] else 'недоступен'}",
            f"SuperCollider: {'запущен' if state['supercollider_running'] else 'не запущен'}",
            f"Пресет: {state['current_preset'] or 'не задан'}",
            f"Активные паттерны: {', '.join(state['active_patterns']) or 'нет'}",
            f"История паттернов: {', '.join(state['pattern_history'].keys()) or 'нет'}",
        ]
        return MCPToolResult(
            success=True,
            data=state,
            message="\n".join(parts),
        )
