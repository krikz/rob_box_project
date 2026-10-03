"""test_music_pattern_runtime.py — unit-тесты для core/music_pattern_runtime.

Тесты фабрикуют ``MusicManager`` через ``__new__`` (без ``__init__`` —
как в ``test/test_tools/test_music.py::_make_manager``, иначе пришлось
бы поднимать ROS/SCLang/сэмплы), инициализируют минимальный набор полей
и передают в ``MusicPatternRuntime``. Так обходим
``MusicManager.__init__`` (который конструирует sclang-канал, OSC,
SynthDef-цикл — это всё уже протестировано в ``test_music.py``).

Каждый тест пишет поведенческий контракт соответствующего метода —
тот же, что держали xfail-стабы: для ``execute_code`` это «код уходит
в Renardo-context», «небезопасный код отклоняется ДО exec», «fallback
на недоступном SC возвращает honest error без тишины» и т.д. Тесты
наследуют сьют test_tools/test_music.py по стилю: Mock'и для сервисов
(_check_supercollider, is_music_stack_healthy, _renardo_context
``Samples`` и т.д.), но сам runtime — реальный.

Сьют остаётся локальным (``pytest test/test_core/...``) и не зависит от
``rclpy`` (как ``test_arrangement_presets``).
"""

from __future__ import annotations

import re
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import pytest

# Mock ROS 2 — ``core.music_pattern_runtime`` не тянет rclpy, но
# ``tools.music`` импортируется через ``rob_box_mcp_tools.tools.__init__``
# (``from .navigation import *`` → ``nav2_msgs``, ``from .voice import *``,
# ``from .music import *``, etc.) — на тестах весь ROS-стек и
# rob_box-specific зависимости мокаются. Стратегия: сначала загрузить
# РЕАЛЬНЫЙ ``rob_box_mcp_tools.tools.music`` (он лежит на
# ``sys.path`` через conftest, никаких ROS-зависимостей на верхнем
# уровне не требует), а потом подменить ``rob_box_mcp_tools.tools`` и
# остальные модули на stub'ы, чтобы дальнейшие ``from .navigation
# import *`` etc. не падали. Тонкий момент: реальный ``tools.music``
# при первом импорте загрузит ВСЕ модули пакета ``tools`` (через
# ``__init__.py`` ``from .animation import *`` и т.д.) — поэтому сначала
# мокаем ROS-зависимости, а только потом импортируем music.
import sys as _sys
import types as _types

# Чистый ROS 2 / rob_box-misc — заглушки. Делаем это ДО любых импортов
# модулей ``rob_box_mcp_tools.tools.*`` (они через __init__ тянут
# geometry_msgs, nav2_msgs, builtin_interfaces и пр.). Используем
# ModuleType + sys.modules[parent][name] = child, чтобы
# ``from action_msgs.srv import X`` находил ``srv`` как атрибут
# пакета, а не пытался импортировать его как модуль.
def _install_stub_pkg(name: str) -> None:
    """Поставить пустой пакет ``name`` в sys.modules (с ``__path__``)."""
    if name in _sys.modules:
        return
    _mod = _types.ModuleType(name)
    _mod.__path__ = []  # type: ignore[attr-defined]
    _sys.modules[name] = _mod


def _install_stub_module(name: str) -> None:
    """Поставить пустой модуль ``name`` в sys.modules (без ``__path__``).

    Используем ``MagicMock()`` вместо ``ModuleType``, чтобы
    ``from rclpy.action import ActionClient`` отдавало ``ActionClient``
    как атрибут (а не бросало ``ImportError: cannot import name``).
    """
    if name in _sys.modules:
        return
    _sys.modules[name] = MagicMock()


# Пакеты (с __path__) — для ``from X.something import ...``,
# где ``X`` — настоящий пакет.
for _pkg_name in [
    "rclpy", "std_msgs", "geometry_msgs", "nav_msgs", "nav2_msgs",
    "sensor_msgs", "builtin_interfaces", "unique_identifier",
    "action_msgs", "tf2_ros",
]:
    _install_stub_pkg(_pkg_name)
    # Подпакеты — в виде MagicMock-модулей (атрибуты = любые имена).
    for _sub in ("msg", "srv", "action", "node", "qos", "lifecycle",
                 "publisher", "subscription"):
        _install_stub_module(f"{_pkg_name}.{_sub}")

# tf2_geometry_msgs — отдельный модуль.
_install_stub_module("tf2_geometry_msgs")

# Импортируем ``tools.music`` напрямую — обход ``rob_box_mcp_tools.tools``
# как пакета. conftest.py добавляет ``src/rob_box_mcp_tools`` в sys.path,
# и реальный пакет ``rob_box_mcp_tools`` живёт в нём.
import importlib as _importlib
_real_music = _importlib.import_module(
    "rob_box_mcp_tools.tools.music"
)
MusicManager = _real_music.MusicManager
# Теперь делаем ``rob_box_mcp_tools.tools`` пустым пакетом, чтобы любые
# последующие ``from rob_box_mcp_tools.tools.X import ...`` (например,
# внутри ``core.music_pattern_runtime`` — он ссылается на
# ``MusicManager`` через ``self._mgr``, но не импортирует tools) шли
# мимо. Если в будущем runtime начнёт импортировать другие tools —
# добавим их сюда.
_pkg = _types.ModuleType("rob_box_mcp_tools.tools")
_pkg.__path__ = []
_sys.modules["rob_box_mcp_tools.tools"] = _pkg
for _name in ["music"]:
    setattr(_pkg, _name, _real_music)
    _sys.modules[f"rob_box_mcp_tools.tools.{_name}"] = _real_music


from rob_box_mcp_tools.core.music_pattern_runtime import MusicPatternRuntime


# ---------------------------------------------------------------------------
# Helpers — фабрика "голого" MusicManager (без __init__, без Renardo)
# ---------------------------------------------------------------------------


def _make_manager(
    *, sc_running: bool = False, renardo_available: bool = False,
) -> MusicManager:
    """Создать ``MusicManager`` через ``__new__`` с минимальным набором полей.

    Тесты ``test_music.py`` делают то же самое в ``_make_manager()`` —
    обход ``__init__`` через ``__new__`` плюс ручная инициализация полей.
    Так можно тестировать публичный API без подъёма Renardo/SC/SCLang.
    """
    mgr = MusicManager.__new__(MusicManager)
    # Минимум полей, к которым обращается runtime. Имена повторяют
    # ``MusicManager.__init__`` defaults — расхождение = тест-баг.
    mgr._max_amp = 0.85
    mgr._master_gain = 0.5
    mgr._master_gain_applied = False
    mgr._pattern_history = {}
    mgr._active_patterns = set()
    # ``_renardo_context`` должен содержать players (``d1..d9``, ``p1..p9``
    # и т.д.) — иначе ``exec("d1 >> play('x')", mgr._renardo_context)``
    # падает с ``NameError``. В проде эти имена появляются при
    # ``Clock.clear()`` / ``_prepare_renardo_namespace()`` /
    # ``_renardo_imports.py``; на тестах — кладём MagicMock'и, чтобы exec
    # прошёл и проверил логику маршрутизации. Плюс — ``play``/``sample``,
    # которые Renardo прибивает в globals перед exec'ом нашего кода.
    _PLAYERS = [
        f"{p}{i}" for p in ("d", "p", "s", "l") for i in range(1, 10)
    ]
    mgr._renardo_context = {name: MagicMock() for name in _PLAYERS}
    mgr._renardo_context["play"] = MagicMock()
    mgr._renardo_context["sample"] = MagicMock()
    # ``Clock`` в тестах — MagicMock с пустым ``bpm=None`` (тогда
    # ``renardo_bpm`` отдаёт 120.0, см. ``getattr(clock, "bpm", 120) or
    # 120``). ``Clock.clear()`` через MagicMock — no-op с возвратом
    # MagicMock(), что устраивает ``_stop_all_teardown``.
    _clock_mock = MagicMock()
    type(_clock_mock).bpm = None  # type: ignore[attr-defined]
    mgr._renardo_context["Clock"] = _clock_mock
    mgr._renardo_available = renardo_available
    mgr._renardo_last_error = None
    # Music-stack health defaults (issue G-MUSIC).
    mgr._music_stack_status = SimpleNamespace(
        is_healthy=True, oscdef_registered=True, missing_synths=(), fatal_errors=(),
    )
    mgr._require_healthy = True
    # Music session lifecycle defaults (issue #935).
    mgr._auto_stop_ttl_seconds = 300
    mgr._music_session_active_since = None
    mgr._last_music_activity_at = None
    mgr._last_stop_at = None
    mgr._auto_stop_count = 0
    # Issue #990 / #1812 / #2461 deadlines defaults.
    mgr._music_deadline_at = None
    mgr._music_deadline_segments = None
    mgr._music_form_deadline_at = None
    mgr._music_form_cycle_ends_at = None
    mgr._dj_mode_enabled = False
    # ADR-0141 (issue #3133) — track-id machinery.
    mgr._track_seq = 0
    mgr._track_id_prefix = "test"
    mgr.current_track_id = None
    mgr.last_finished_track_id = None
    # Issue #3113 — current track name; set/cleared by ``clear_form_deadline``
    # wrapper.
    mgr.current_track_name = None
    # Константы, которые ``_execute_apply_segments_safety_net`` и
    # ``renardo_bpm`` читают с ``self._mgr`` — без них Runtime упадёт
    # на ``AttributeError``. Значения повторяют ``MusicManager.__init__``
    # defaults (issue #949, #990, BEATS_PER_BAR=4 — стандартный 4/4).
    mgr.MAX_SEGMENTS = 64
    mgr.BEATS_PER_BAR = 4
    mgr.DEPRECATED_DURATION_SEC_CLAMP = 60.0
    mgr.bpm = 120
    # Thread-safety lock (ADR-0141, _state_lock) — fake lock не блокирует.
    import threading
    mgr._state_lock = threading.RLock()
    # Сервисы, которые runtime дёргает через ``self._mgr.<method>()``.
    mgr._check_supercollider = Mock(return_value=sc_running)
    mgr._ensure_renardo_available = Mock(return_value=renardo_available)
    mgr.known_synth_names = Mock(return_value=None)
    mgr.set_master_gain = Mock()
    mgr.is_music_stack_healthy = Mock(return_value=True)
    mgr.music_stack_unavailable_error = Mock(
        return_value={"success": False, "error": "music unavailable"},
    )
    # Phase 3 / ADR-0149 — ``_prepare_renardo_namespace`` и
    # ``_schedule_transition_cleanup`` живут в ``MusicManager`` до Phase 6
    # shim-removal. Заглушки no-op'ы.
    mgr._prepare_renardo_namespace = Mock()
    mgr._schedule_transition_cleanup = Mock()
    mgr._ramp_down_group = Mock()
    mgr._stamp_new_track = Mock()
    mgr._end_music_session = Mock()
    mgr._start_new_track_id = Mock()
    # Runtime — навешиваем в конце, после того как все сервисы инициализированы.
    mgr._runtime = MusicPatternRuntime(mgr)
    return mgr


@pytest.fixture
def manager() -> MusicManager:
    """Фикстура: ``MusicManager`` без SC/Renardo, без active session."""
    return _make_manager(sc_running=False, renardo_available=False)


@pytest.fixture
def manager_running() -> MusicManager:
    """Фикстура: ``MusicManager`` с «поднятыми» SC и Renardo."""
    return _make_manager(sc_running=True, renardo_available=True)


# ---------------------------------------------------------------------------
# execute_code — 3 контракта: routing / validation / fallback
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestExecuteCodeRoutesToRenardo:
    """``execute_code`` маршрутизирует Renardo-код в ``_renardo_context``."""

    def test_simple_code_executes(self, manager_running: MusicManager) -> None:
        result = manager_running._runtime.execute_code("d1 >> play('x')")
        # Успех: success=True, code отражён, activity stamp выставлен.
        assert result["success"] is True, f"execute_code returned {result!r}"
        assert "Код выполнен успешно" in result["message"]
        # pattern_name не задан — поэтому _pattern_history не вырос.
        assert manager_running._pattern_history == {}
        # Сессионный учёт (issue #935) — _stamp_new_track делегирован в
        # _mgr._stamp_new_track.
        manager_running._stamp_new_track.assert_called_once()

    def test_pattern_name_added_to_history(self, manager_running: MusicManager) -> None:
        manager_running._runtime.execute_code(
            "d1 >> play('x')", pattern_name="drums",
        )
        assert "drums" in manager_running._pattern_history
        assert "drums" in manager_running._active_patterns

    def test_segments_safety_net_arms_deadline(
        self, manager_running: MusicManager,
    ) -> None:
        manager_running._runtime.execute_code(
            "d1 >> play('x')", segments=8,
        )
        # __total_segments = 8, deadline взведён.
        assert manager_running._renardo_context["__total_segments"] == 8
        assert manager_running._renardo_context["__total_beats"] == 32  # 8 * 4
        assert manager_running._music_deadline_at is not None

    def test_duration_sec_clamps_and_does_not_arm(
        self, manager_running: MusicManager,
    ) -> None:
        # DEPRECATED-путь (issue #949 → #990): кладёт __total_beats
        # через clamp ≥ 60s, но НЕ взводит _music_deadline_at.
        before = manager_running._music_deadline_at
        manager_running._runtime.execute_code(
            "d1 >> play('x')", duration_sec=6.0,  # короче DEPRECATED-clamp
        )
        assert manager_running._renardo_context["__total_beats"] > 0
        # Дедлайн НЕ взведён (issue #990, deprecated-путь не стопит).
        assert manager_running._music_deadline_at == before


@pytest.mark.unit
class TestExecuteCodeValidatesInput:
    """``execute_code`` отклоняет небезопасный / пустой / битый код ДО exec."""

    def test_security_filter_blocks_dangerous_code(
        self, manager_running: MusicManager,
    ) -> None:
        # ``import os`` ловится security-фильтром renardo_sanitizer.
        result = manager_running._runtime.execute_code("import os; os.system('id')")
        assert result["success"] is False
        assert "error" in result
        # exec НЕ вызван — _renardo_context не получил имени os.
        assert "os" not in manager_running._renardo_context

    def test_empty_code_still_executes(
        self, manager_running: MusicManager,
    ) -> None:
        # Пустой код проходит sanitizer (нет security-триггера), exec
        # на пустой строке — no-op.
        result = manager_running._runtime.execute_code("")
        assert result["success"] is True


@pytest.mark.unit
class TestExecuteCodeFallbackPath:
    """``execute_code`` возвращает honest error при недоступном SC/Renardo."""

    def test_sc_down_returns_clear_error(self, manager: MusicManager) -> None:
        # manager (без sc_running) → _check_supercollider == False.
        result = manager._runtime.execute_code("d1 >> play('x')")
        assert result["success"] is False
        assert "SuperCollider" in result["error"]
        # exec НЕ вызван, pattern_history не вырос.
        assert manager._pattern_history == {}

    def test_renardo_unavailable_returns_clear_error(self, manager: MusicManager) -> None:
        # SC «работает», но Renardo недоступен.
        manager._check_supercollider.return_value = True
        manager._ensure_renardo_available.return_value = False
        result = manager._runtime.execute_code("d1 >> play('x')")
        assert result["success"] is False
        assert "Renardo" in result["error"]

    def test_degraded_stack_returns_unavailable_error(
        self, manager_running: MusicManager,
    ) -> None:
        # Music stack degraded → ранний return через
        # ``music_stack_unavailable_error()`` (issue G-MUSIC).
        manager_running._require_healthy = True
        manager_running.is_music_stack_healthy.return_value = False
        result = manager_running._runtime.execute_code("d1 >> play('x')")
        assert result["success"] is False


# ---------------------------------------------------------------------------
# stop_pattern / stop_all
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestStopPattern:
    """``stop_pattern`` валидирует имя + ``call_player_stop``."""

    def test_invalid_name_rejected_without_call(
        self, manager_running: MusicManager,
    ) -> None:
        # Не-идентификатор (точка) → regex-отказ, без обращения к Renardo.
        with patch.object(
            manager_running._runtime, "call_player_stop"
        ) as csp:
            result = manager_running._runtime.stop_pattern("p1.stop()")
        assert result["success"] is False
        assert "Недопустимое имя" in result["error"]
        csp.assert_not_called()

    def test_unknown_name_rejected_with_list(
        self, manager_running: MusicManager,
    ) -> None:
        # Корректный идентификатор, но не в whitelist.
        manager_running._active_patterns = {"drums", "bass"}
        with patch.object(
            manager_running._runtime, "call_player_stop"
        ) as csp:
            result = manager_running._runtime.stop_pattern("unknown")
        assert result["success"] is False
        assert "Неизвестный паттерн" in result["error"]
        assert "drums" in result["error"]  # список активных
        csp.assert_not_called()

    def test_known_pattern_calls_stop_and_drops(
        self, manager_running: MusicManager,
    ) -> None:
        manager_running._active_patterns = {"drums"}
        with patch.object(
            manager_running._runtime, "call_player_stop"
        ) as csp:
            result = manager_running._runtime.stop_pattern("drums")
        assert result["success"] is True
        csp.assert_called_once_with("drums")
        assert "drums" not in manager_running._active_patterns

    def test_builtin_player_allowed(self, manager_running: MusicManager) -> None:
        # d1-d9 / p1-p9 / s1-s9 / l1-l9 — builtin'ы Renardo, всегда
        # разрешены по whitelist.
        with patch.object(
            manager_running._runtime, "call_player_stop"
        ) as csp:
            result = manager_running._runtime.stop_pattern("p1")
        assert result["success"] is True
        csp.assert_called_once_with("p1")


@pytest.mark.unit
class TestStopAllIdleAndActive:
    """``stop_all`` ведёт себя одинаково для пустого и активного состояния."""

    def test_idle_manager_no_error(self, manager: MusicManager) -> None:
        result = manager._runtime.stop_all()
        assert result["success"] is True
        assert "остановлена" in result["message"].lower() or "ok" in result["message"].lower()

    def test_active_manager_stops_and_resets(
        self, manager_running: MusicManager,
    ) -> None:
        manager_running._active_patterns = {"drums", "bass"}
        result = manager_running._runtime.stop_all()
        assert result["success"] is True
        # _end_music_session делегирован (issue #3133).
        manager_running._end_music_session.assert_called_once()
        # _ramp_down_group делегирован (anti-click ramp).
        manager_running._ramp_down_group.assert_called_once_with(1)

    def test_degraded_stack_returns_error_but_still_resets(
        self, manager_running: MusicManager,
    ) -> None:
        manager_running._require_healthy = True
        manager_running.is_music_stack_healthy.return_value = False
        result = manager_running._runtime.stop_all()
        assert result["success"] is False
        assert "degraded" in result["error"].lower() or "недоступна" in result["error"].lower()
        # Сессия всё равно сброшена (issue #935 safety-net).
        manager_running._end_music_session.assert_called_once()


# ---------------------------------------------------------------------------
# Helpers (call_player_stop / prewarm_sample_buffers / resolve_pattern_name)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestCallPlayerStop:
    """``call_player_stop`` достаёт плеер по имени и зовёт ``.stop()``."""

    def test_known_player_stopped(self, manager: MusicManager) -> None:
        fake_player = Mock()
        manager._renardo_context["p1"] = fake_player
        manager._runtime.call_player_stop("p1")
        fake_player.stop.assert_called_once()

    def test_unknown_player_is_noop(self, manager: MusicManager) -> None:
        # Не raise — best-effort остановка, не валидация.
        manager._runtime.call_player_stop("unknown")


@pytest.mark.unit
class TestPrewarmSampleBuffers:
    """``prewarm_sample_buffers`` грузит буферы ДО exec (live 13.08, issue #1815)."""

    def test_no_samples_noop(self, manager: MusicManager) -> None:
        # Samples отсутствует в контексте — no-op, без raise.
        manager._runtime.prewarm_sample_buffers("d1 >> play('x-o-')")

    def test_hyphen_is_a_sound_not_rest(
        self, manager: MusicManager, monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        # Issue #1815: «-» ЗВУЧАЩИЙ хэт (hyphen-каталог), не пауза.
        # Фильтрация символов происходит в ``renardo_adapter.load_sample_buffers``
        # (общий с владельцем плеера v2). Мы проверяем, что runtime
        # делегирует в этот адаптер, и адаптер получает строку со всеми
        # символами (фильтр — внутри адаптера).
        called_with: list = []
        fake_samples = Mock()
        # Подменяем адаптер, чтобы перехватить аргументы.
        from rob_box_mcp_tools.core import music_pattern_runtime as mpr

        def fake_load(samples, symbols):
            called_with.append(symbols)

        monkeypatch.setattr(mpr.renardo_adapter, "load_sample_buffers", fake_load)
        manager._renardo_context["Samples"] = fake_samples
        manager._runtime.prewarm_sample_buffers("d1 >> play('x-o-')")
        assert called_with == ["x-o-"]


@pytest.mark.unit
class TestResolvePatternName:
    """``resolve_pattern_name`` whitelist для ``stop_pattern`` (issue G-MUSIC)."""

    def test_builtin_p1_allowed(self, manager: MusicManager) -> None:
        ok, err = manager._runtime.resolve_pattern_name("p1")
        assert ok is True
        assert err == ""

    def test_attack_string_rejected(self, manager: MusicManager) -> None:
        # Бывший RCE: ``p1.stop(); __import__('os')...`` — не-идентификатор.
        ok, err = manager._runtime.resolve_pattern_name("p1.stop(); __import__('os')")
        assert ok is False
        assert "Недопустимое имя" in err

    def test_active_pattern_allowed(self, manager: MusicManager) -> None:
        manager._active_patterns = {"bass"}
        ok, _ = manager._runtime.resolve_pattern_name("bass")
        assert ok is True

    def test_unknown_rejected(self, manager: MusicManager) -> None:
        ok, err = manager._runtime.resolve_pattern_name("foo")
        assert ok is False
        assert "Неизвестный паттерн" in err


# ---------------------------------------------------------------------------
# BPM / schedule_stop / form-deadlines
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestRenardoBpm:
    """``renardo_bpm`` — текущий BPM Renardo с дефолтом 120."""

    def test_default_when_no_clock(self, manager: MusicManager) -> None:
        assert manager._runtime.renardo_bpm() == 120.0

    def test_returns_clock_bpm(self, manager: MusicManager) -> None:
        manager._renardo_context["Clock"] = SimpleNamespace(bpm=83)
        assert manager._runtime.renardo_bpm() == 83.0

    def test_zero_bpm_falls_back_to_120(self, manager: MusicManager) -> None:
        # Битый Clock.bpm=0 — положительный fallback.
        manager._renardo_context["Clock"] = SimpleNamespace(bpm=0)
        assert manager._runtime.renardo_bpm() == 120.0


@pytest.mark.unit
class TestScheduleStop:
    """``schedule_stop`` ставит deadline по issue #990 (segments backstop)."""

    def test_minimum_floor_60s(self, manager: MusicManager) -> None:
        # 8 тактов @ 90bpm = 21.3s музыки, но минимум 60s
        # (live 30.08 vision-pi 12:30: «сыграй короткий бит» → 8 сегментов
        # убивало трек на 20с).
        before = time.monotonic()
        manager._runtime.schedule_stop(segments=8, bpm=90.0)
        deadline = manager._music_deadline_at
        assert deadline is not None
        assert deadline - before >= 60.0  # MIN_SEGMENTS_DEADLINE_SECONDS
        assert manager._music_deadline_segments == 8

    def test_safety_factor_applies(self, manager: MusicManager) -> None:
        # 64 такта @ 120bpm = 128s; * 2.0 safety = 256s — выше минимума.
        before = time.monotonic()
        manager._runtime.schedule_stop(segments=64, bpm=120.0)
        deadline = manager._music_deadline_at
        assert deadline is not None
        # BEATS_PER_BAR=4, BAR=4*60/120=2s, 64 bars = 128s, *2 = 256s.
        assert deadline - before >= 250.0


@pytest.mark.unit
class TestSetFormDeadline:
    """``set_form_deadline`` взводит момент конца одной формы (issue #1812)."""

    def test_deadline_set_in_future(self, manager: MusicManager) -> None:
        before = time.monotonic()
        manager._runtime.set_form_deadline(120.0)
        assert manager._music_form_deadline_at is not None
        assert manager._music_form_deadline_at >= before + 120.0

    def test_negative_duration_clamps_to_zero(self, manager: MusicManager) -> None:
        before = time.monotonic()
        manager._runtime.set_form_deadline(-50.0)
        # max(0.0, -50.0) = 0.0 → deadline ≈ now.
        assert manager._music_form_deadline_at is not None
        assert manager._music_form_deadline_at >= before


@pytest.mark.unit
class TestSetFormCycleEnd:
    """``set_form_cycle_end`` взводится на КАЖДЫЙ compose_music (issue #2461)."""

    def test_cycle_end_set_independently(self, manager: MusicManager) -> None:
        before = time.monotonic()
        manager._runtime.set_form_cycle_end(45.0)
        assert manager._music_form_cycle_ends_at is not None
        assert manager._music_form_cycle_ends_at >= before + 45.0
        # НЕ путается с form_deadline (другой канал, issue #2461).
        assert manager._music_form_deadline_at is None


@pytest.mark.unit
class TestClearFormDeadline:
    """``clear_form_deadline`` сбрасывает ОБА канала (#1812 + #2461)."""

    def test_clears_both(self, manager: MusicManager) -> None:
        manager._music_form_deadline_at = 100.0
        manager._music_form_cycle_ends_at = 200.0
        manager._runtime.clear_form_deadline()
        assert manager._music_form_deadline_at is None
        assert manager._music_form_cycle_ends_at is None

    def test_clear_when_already_none(self, manager: MusicManager) -> None:
        # Повторный вызов — no-op, без raise.
        manager._runtime.clear_form_deadline()
        assert manager._music_form_deadline_at is None
        assert manager._music_form_cycle_ends_at is None


# ---------------------------------------------------------------------------
# Watchdog
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestAutoStopIdleMusic:
    """``auto_stop_idle_music`` — watchdog (issue #935/990/1812/1000)."""

    def test_no_activity_is_noop(self, manager: MusicManager) -> None:
        result = manager._runtime.auto_stop_idle_music()
        assert result["stopped"] is False
        assert result["idle_seconds"] is None

    def test_below_ttl_is_noop(self, manager: MusicManager) -> None:
        manager._last_music_activity_at = time.monotonic() - 5.0
        result = manager._runtime.auto_stop_idle_music(ttl_seconds=300.0)
        assert result["stopped"] is False
        assert result["idle_seconds"] == pytest.approx(5.0, abs=0.01)

    def test_segments_deadline_stops(self, manager_running: MusicManager) -> None:
        manager_running._last_music_activity_at = time.monotonic() - 5.0
        manager_running._music_deadline_at = time.monotonic() - 1.0  # истёк
        manager_running._music_deadline_segments = 8
        with patch.object(manager_running._runtime, "stop_all") as stop_all:
            stop_all.return_value = {"success": True, "message": "ok"}
            result = manager_running._runtime.auto_stop_idle_music()
        assert result["stopped"] is True
        assert result["stop_reason"] == "segments_deadline"
        assert result["deadline_segments"] == 8
        assert manager_running._auto_stop_count == 1

    def test_dj_mode_ignores_segments_deadline(
        self, manager_running: MusicManager,
    ) -> None:
        # live 10:13 DJ: дедлайн игнорируется, DJ живёт по idle-TTL.
        manager_running._dj_mode_enabled = True
        manager_running._last_music_activity_at = time.monotonic() - 5.0
        manager_running._music_deadline_at = time.monotonic() - 1.0
        result = manager_running._runtime.auto_stop_idle_music()
        assert result["stopped"] is False
        # Дедлайн сброшен, чтобы следующий переход продлил сессию.
        assert manager_running._music_deadline_at is None

    def test_form_deadline_holds(self, manager_running: MusicManager) -> None:
        # Issue #1812: repeat=False форма ещё не доиграла → hold.
        manager_running._last_music_activity_at = time.monotonic() - 500.0
        manager_running._music_form_deadline_at = time.monotonic() + 100.0
        result = manager_running._runtime.auto_stop_idle_music(ttl_seconds=300.0)
        assert result["stopped"] is False
        assert result["held_reason"] == "form_not_finished"

    def test_idle_ttl_stops(self, manager_running: MusicManager) -> None:
        manager_running._last_music_activity_at = time.monotonic() - 500.0
        with patch.object(manager_running._runtime, "stop_all") as stop_all:
            stop_all.return_value = {"success": True, "message": "ok"}
            result = manager_running._runtime.auto_stop_idle_music(ttl_seconds=300.0)
        assert result["stopped"] is True
        assert result["stop_reason"] == "idle_ttl"


@pytest.mark.unit
class TestStopMusicOnSessionEnd:
    """``stop_music_on_session_end`` — DIALOGUE_END hook (issue #935)."""

    def test_when_no_session(self, manager: MusicManager) -> None:
        # No music session open — вызов безопасный, stop_all всё равно
        # дёргается (идемпотентно).
        with patch.object(manager._runtime, "stop_all") as stop_all:
            stop_all.return_value = {"success": True, "message": "ok"}
            result = manager._runtime.stop_music_on_session_end()
        assert result["was_active"] is False
        assert result["stopped_patterns"] == []
        stop_all.assert_called_once()

    def test_when_session_active(self, manager_running: MusicManager) -> None:
        manager_running._music_session_active_since = time.monotonic() - 60.0
        manager_running._active_patterns = {"drums", "bass"}
        with patch.object(manager_running._runtime, "stop_all") as stop_all:
            stop_all.return_value = {"success": True, "message": "ok"}
            result = manager_running._runtime.stop_music_on_session_end()
        assert result["was_active"] is True
        assert set(result["stopped_patterns"]) == {"drums", "bass"}
        assert "stop_music сработал" in result["message"]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
