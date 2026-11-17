"""Тесты heartbeat живости vision-нод (issue #2703, #2704).

Регресс-контракт (issue #2703): здоровая нода в пустой сцене (infer
выполнился успешно, 0 детекций выше confidence_threshold) обязана
оставаться healthy. Старый healthcheck (`ros2 topic echo --once`)
красил контейнер unhealthy в этой ситуации — heartbeat теперь
обновляется по факту успешного `infer()`, а не по факту публикации
`VisionEvent`.

Два блока:
  A. ``FileHeartbeat`` (pure, без rclpy) — запись/чтение/протухание,
     через инжектируемый ``time_fn`` (никаких реальных ``time.sleep``).
  B. ``VisionHailoNode._tick`` wiring — heartbeat обновляется ПОСЛЕ
     успешного infer(), НЕ обновляется при исключении, требует rclpy
     для импорта модуля ноды (тот же паттерн, что
     ``test_vision_hailo_poll_gaze.py``: конструируем ноду через
     ``__new__``, минуя ``__init__``).

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_vision_heartbeat.py -v -o addopts=""
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest


# `parents[2]` = .../src/rob_box_perception → содержит пакет
# rob_box_perception/ (тот же приём, что в test_vision_hailo_phase15.py).
_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

heartbeat_mod = importlib.import_module('rob_box_perception.utils.heartbeat')


class _FakeClock:
    """Детерминированный инжектируемый time_fn — без реальных sleep()."""

    def __init__(self, start: float = 1_000_000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, delta: float) -> None:
        self.now += delta


# ============================================================================
# A. FileHeartbeat — pure unit tests
# ============================================================================


def test_default_heartbeat_path_matches_node_names():
    """Дефолтные пути совпадают с тем, что читают healthcheck-скрипты."""
    assert (
        heartbeat_mod.default_heartbeat_path('vision_hailo')
        == '/tmp/vision_hailo_heartbeat'
    )
    assert (
        heartbeat_mod.default_heartbeat_path('vision_face')
        == '/tmp/vision_face_heartbeat'
    )


def test_age_sec_none_when_file_missing(tmp_path):
    hb = heartbeat_mod.FileHeartbeat(str(tmp_path / 'hb'))
    assert hb.age_sec() is None


def test_beat_creates_fresh_file(tmp_path):
    clock = _FakeClock()
    hb = heartbeat_mod.FileHeartbeat(str(tmp_path / 'hb'), time_fn=clock)
    hb.beat()
    age = hb.age_sec()
    assert age is not None
    assert age == pytest.approx(0.0, abs=1e-6)


def test_heartbeat_becomes_stale_without_real_sleep(tmp_path):
    """Протухание heartbeat симулируется продвижением fake-clock, не sleep().

    Это и есть тест-сценарий из issue #2703 acceptance: "infer не
    выполняется дольше порога -> heartbeat протух".
    """
    clock = _FakeClock()
    hb = heartbeat_mod.FileHeartbeat(str(tmp_path / 'hb'), time_fn=clock)
    hb.beat()
    assert hb.age_sec() == pytest.approx(0.0, abs=1e-6)

    stale_threshold = 30.0
    clock.advance(stale_threshold + 5.0)
    age = hb.age_sec()
    assert age is not None
    assert age > stale_threshold, (
        f'heartbeat должен был протухнуть: age={age}s, порог={stale_threshold}s'
    )


def test_beat_repeated_refreshes_age(tmp_path):
    """Повторные beat() (аналог тика _tick) держат heartbeat свежим."""
    clock = _FakeClock()
    hb = heartbeat_mod.FileHeartbeat(str(tmp_path / 'hb'), time_fn=clock)

    for _ in range(5):
        clock.advance(0.5)
        hb.beat()
        assert hb.age_sec() == pytest.approx(0.0, abs=1e-6)


def test_age_sec_none_on_garbage_content(tmp_path):
    """Файл существует, но содержимое не парсится -> None (не raise)."""
    path = tmp_path / 'hb'
    path.write_text('not-a-timestamp', encoding='utf-8')
    hb = heartbeat_mod.FileHeartbeat(str(path))
    assert hb.age_sec() is None


# ============================================================================
# B. VisionHailoNode._tick wiring (требует rclpy для импорта модуля)
# ============================================================================

pytest.importorskip('rclpy')

from rob_box_perception.vision_hailo_node import VisionHailoNode  # noqa: E402


class _FakeLogger:
    def error(self, *_a, **_kw) -> None:
        pass

    def warning(self, *_a, **_kw) -> None:
        pass

    def info(self, *_a, **_kw) -> None:
        pass


class _FakePublisher:
    def __init__(self) -> None:
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class _FakeLoader:
    """Duck-typed HEFLoader: контролируемые события/исключения."""

    def __init__(self, events=None, raise_exc: Exception | None = None) -> None:
        self._events = events if events is not None else []
        self._raise_exc = raise_exc
        self.calls = 0

    def infer(self, frame_id, image):  # noqa: D401 — duck-typed HEFLoader API
        self.calls += 1
        if self._raise_exc is not None:
            raise self._raise_exc
        return list(self._events)


def _make_bare_node(
    loader: _FakeLoader,
    heartbeat_path: str,
    clock: _FakeClock,
    *,
    publish_when_no_input: bool = True,
    has_received_frame: bool = True,
    is_real_mode: bool = False,
) -> VisionHailoNode:
    """Собрать VisionHailoNode в обход __init__ (паттерн test_vision_hailo_poll_gaze.py)."""
    node = VisionHailoNode.__new__(VisionHailoNode)
    node._publisher = _FakePublisher()
    node.publish_when_no_input = publish_when_no_input
    node._has_received_frame = has_received_frame
    node._last_frame_id = 'cam' if has_received_frame else None
    node._is_real_mode = is_real_mode
    node._latest_frame = None
    node._loader = loader
    node.confidence_threshold = 0.5
    node._consecutive_failures = 0
    node._degraded_logged = False
    node._max_logged_failures = 3
    node.get_logger = lambda: _FakeLogger()  # type: ignore[method-assign]
    node.heartbeat_path = heartbeat_path
    node._heartbeat = heartbeat_mod.FileHeartbeat(heartbeat_path, time_fn=clock)
    return node


def test_tick_empty_scene_updates_heartbeat(tmp_path):
    """Ключевой регресс issue #2703: пустая сцена -> heartbeat свежий.

    infer() успешен, детекций 0 (пустая сцена) — это ЗДОРОВАЯ нода.
    Heartbeat обязан обновиться, иначе healthcheck снова красит
    контейнер unhealthy на пустой мастерской.
    """
    clock = _FakeClock()
    hb_path = str(tmp_path / 'vision_hailo_heartbeat')
    loader = _FakeLoader(events=[])  # infer успешен, raw_events == []
    node = _make_bare_node(loader, hb_path, clock, publish_when_no_input=True)

    assert node._heartbeat.age_sec() is None  # до тика heartbeat не было

    node._tick()

    assert loader.calls == 1
    age = node._heartbeat.age_sec()
    assert age is not None, 'heartbeat не был записан после успешного infer()'
    assert age == pytest.approx(0.0, abs=1e-6)


def test_tick_infer_failure_does_not_update_heartbeat(tmp_path):
    """infer() падает (degraded state) -> heartbeat НЕ обновляется."""
    clock = _FakeClock()
    hb_path = str(tmp_path / 'vision_hailo_heartbeat')
    loader = _FakeLoader(raise_exc=RuntimeError('HailoRT run() failed (fake)'))
    node = _make_bare_node(loader, hb_path, clock, publish_when_no_input=True)

    node._tick()

    assert loader.calls == 1
    assert node._heartbeat.age_sec() is None, (
        'heartbeat не должен обновляться, когда infer() бросил исключение '
        '(issue #2703 contract: heartbeat != "процесс жив", '
        'heartbeat == "последний infer успешен").'
    )


def test_tick_infer_stops_heartbeat_goes_stale(tmp_path):
    """Регресс-сценарий acceptance: _tick перестаёт выполняться -> heartbeat стареет.

    Симулируем через fake-clock (без реального sleep): один успешный тик,
    затем продвигаем время без новых тиков — возраст должен превысить
    порог healthcheck'а (30s, см. healthcheck_frame.sh HEARTBEAT_STALE_SEC).
    """
    clock = _FakeClock()
    hb_path = str(tmp_path / 'vision_hailo_heartbeat')
    loader = _FakeLoader(events=[])
    node = _make_bare_node(loader, hb_path, clock, publish_when_no_input=True)

    node._tick()
    assert node._heartbeat.age_sec() == pytest.approx(0.0, abs=1e-6)

    # _tick "перестал выполняться" — таймер завис / executor заблокирован.
    clock.advance(31.0)
    age = node._heartbeat.age_sec()
    assert age is not None and age > 30.0, (
        f'heartbeat обязан протухнуть после {31.0}s без тика, age={age}'
    )


def test_tick_no_frame_yet_with_publish_when_no_input_false_skips_infer(tmp_path):
    """Прод-конфиг (publish_when_no_input=False, ADR-0104 acceptance #5):

    Пока источник ни разу не отдал кадр, _tick вообще не должен вызывать
    infer() (ранний return) — значит, heartbeat остаётся пустым, и
    healthcheck честно красит контейнер, пока источник действительно
    недоступен (в отличие от stub / CI, где publish_when_no_input=True).
    """
    clock = _FakeClock()
    hb_path = str(tmp_path / 'vision_hailo_heartbeat')
    loader = _FakeLoader(events=[])
    node = _make_bare_node(
        loader,
        hb_path,
        clock,
        publish_when_no_input=False,
        has_received_frame=False,
    )

    node._tick()

    assert loader.calls == 0, 'infer() не должен звонить без кадра в prod-режиме'
    assert node._heartbeat.age_sec() is None


def test_tick_recovers_heartbeat_after_previous_failure(tmp_path):
    """После цепочки фейлов успешный infer() снова свежит heartbeat."""
    clock = _FakeClock()
    hb_path = str(tmp_path / 'vision_hailo_heartbeat')
    failing_loader = _FakeLoader(raise_exc=RuntimeError('boom'))
    node = _make_bare_node(failing_loader, hb_path, clock, publish_when_no_input=True)

    node._tick()
    assert node._heartbeat.age_sec() is None
    assert node._consecutive_failures == 1

    # Loader "восстановился" — следующий infer() успешен.
    node._loader = _FakeLoader(events=[])
    clock.advance(1.0)
    node._tick()

    assert node._consecutive_failures == 0
    age = node._heartbeat.age_sec()
    assert age is not None
    assert age == pytest.approx(0.0, abs=1e-6)
