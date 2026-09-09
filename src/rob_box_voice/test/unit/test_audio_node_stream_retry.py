"""
test_audio_node_stream_retry.py — guard-тест на живучесть захвата аудио.

История (04.09.2026, робот 10.1.1.21). После появления на USB-шине второй
аудио-карты (HD USB Camera потолочной камеры) ReSpeaker переставал
попадать в перечисление PortAudio к моменту старта audio_node: в логах
`❌ ReSpeaker аудио устройство не найдено!`, дальше `return` — и захват
не поднимался НИКОГДА, хотя через минуту то же устройство прекрасно
находилось (`hw:1,0`, 6ch).

Снаружи это выглядело как «робот перестал слышать», но не как отказ:
VAD и DoA читаются с ReSpeaker по USB HID отдельным каналом и работали,
поэтому в логи честно шло `🎙️ VAD: речь`, а каждая фраза отлетала как
`❌ Речь отклонена: 0.00с` — буфер захвата пуст, наполнять его некому.
Контейнер при этом оставался `healthy`.

Вторая мина того же места: открытие потока лежало ВНУТРИ `_apply_dsp`,
за двумя ранними `return` (`dsp_apply_on_start=False` и «USB HID не
подключен»). Выключение DSP-тюнинга параметром молча отключало микрофон.

Тест статический (AST) — как соседние audio_node-тесты, без ROS mocks:
  1. открытие потока живёт в `open_audio_stream`, а не в `_apply_dsp`;
  2. `initialize_hardware` зовёт его безусловно, вне if/else по USB HID;
  3. обе ветки отказа планируют ретрай, а не просто выходят.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
AUDIO_NODE_PY = REPO_ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "audio_node.py"


def _audio_node_class() -> ast.ClassDef:
    tree = ast.parse(AUDIO_NODE_PY.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "AudioNode":
            return node
    pytest.fail("класс AudioNode не найден в audio_node.py")


def _method(name: str) -> ast.FunctionDef:
    for item in _audio_node_class().body:
        if isinstance(item, ast.FunctionDef) and item.name == name:
            return item
    pytest.fail(f"метод AudioNode.{name} не найден")


def _calls(node: ast.AST) -> set[str]:
    """Имена вызовов внутри узла: и `foo()`, и `self.foo()`."""
    found: set[str] = set()
    for sub in ast.walk(node):
        if isinstance(sub, ast.Call):
            func = sub.func
            if isinstance(func, ast.Attribute):
                found.add(func.attr)
            elif isinstance(func, ast.Name):
                found.add(func.id)
    return found


def test_stream_open_lives_in_open_audio_stream() -> None:
    """Поток открывается в своём методе, а не внутри DSP-тюнинга."""
    assert "open" in _calls(_method("open_audio_stream")), (
        "open_audio_stream больше не открывает PyAudio-поток"
    )
    dsp_calls = _calls(_method("_apply_dsp"))
    assert "start_stream" not in dsp_calls, (
        "открытие захвата вернулось внутрь _apply_dsp — оно снова отключится "
        "при dsp_apply_on_start=False"
    )


def test_initialize_hardware_opens_stream_unconditionally() -> None:
    """open_audio_stream вызывается вне ветки «USB HID подключился».

    Внутри if/else он снова стал бы заложником VAD/DoA-канала.
    """
    init = _method("initialize_hardware")
    top_level = {
        name
        for stmt in init.body
        if not isinstance(stmt, (ast.If, ast.Try))
        for name in _calls(stmt)
    }
    assert "open_audio_stream" in top_level, (
        "initialize_hardware не зовёт open_audio_stream на верхнем уровне"
    )


def test_both_failure_paths_schedule_retry() -> None:
    """«Устройство не найдено» и «ошибка открытия» уходят в ретрай."""
    src = ast.get_source_segment(
        AUDIO_NODE_PY.read_text(encoding="utf-8"), _method("open_audio_stream")
    )
    assert src is not None
    assert src.count("_schedule_audio_retry()") >= 2, (
        "одна из веток отказа снова делает return без ретрая — захват "
        "не поднимется до перезапуска ноды"
    )
    assert "publish_state('error_no_device')" in src
    assert "publish_state('error_stream')" in src


def test_retry_reenumerates_portaudio() -> None:
    """На каждой попытке PyAudio пересоздаётся.

    PortAudio делает снимок списка устройств при инициализации: без
    terminate() ретрай вечно смотрел бы на старый снимок без ReSpeaker.
    """
    calls = _calls(_method("open_audio_stream"))
    assert "terminate" in calls, (
        "open_audio_stream не пересоздаёт PyAudio — ретрай будет видеть "
        "устаревшее перечисление устройств"
    )
