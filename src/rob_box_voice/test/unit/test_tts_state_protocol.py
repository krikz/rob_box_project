"""Словарь состояний ``/voice/tts/state`` — один протокол на четыре ноды.

``tts_node`` публикует состояния, а слушают их трое: ``stt_node``
(гейт эха), ``audio_node`` (VAD/grace) и ``animation_player_node``
(анимация рта). Каждый держит СВОЙ список строк, и списки разъехались:

* ``stt_node`` не знал ``"stopped"`` — а его публикует STOP/barge-in
  (``tts_node._handle_stop_command``). Флаг ``is_robot_speaking``
  оставался ``True``, и в hardware-AEC (это конфиг робота,
  ``docker/vision/config/voice_assistant/stt_node.yaml``) фразы короче
  0.8 с молча отбрасывались — «робот», «стоп», «да».
* ``tts_silero_warming`` не знал НИКТО. Его публикует путь, где чанк
  пропущен и НЕ будет озвучен; после него шёл ``return`` без
  терминального состояния. Рот продолжал артикулировать в тишине.

Ровно тот класс, что уже стоил issue #1252 / #1734 с ``wake_words``:
одна вокабуляра, размноженная по файлам.

Тесты читают ИСХОДНИКИ — ноды тянут rclpy, поднимать их незачем.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

_VOICE = Path(__file__).resolve().parents[2] / "rob_box_voice"
_TTS_NODE = _VOICE / "tts_node.py"
_STT_NODE = _VOICE / "stt_node.py"
_AUDIO_NODE = _VOICE / "audio_node.py"
# Аниматор живёт в другом ROS-пакете, но слушает ТОТ ЖЕ топик и держит
# СВОЮ копию словаря — именно он давал «рот говорит, робот молчит».
_ANIM_NODE = (
    Path(__file__).resolve().parents[3]
    / "rob_box_animations"
    / "scripts"
    / "animation_player_node.py"
)


def _published_states() -> set[str]:
    """Строковые литералы всех ``self.publish_state("…")`` в tts_node."""
    tree = ast.parse(_TTS_NODE.read_text(encoding="utf-8"))
    found: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not (isinstance(func, ast.Attribute) and func.attr == "publish_state"):
            continue
        if node.args and isinstance(node.args[0], ast.Constant):
            value = node.args[0].value
            if isinstance(value, str):
                found.add(value)
    return found


def _state_literals(path: Path, callback: str) -> set[str]:
    """Все строковые литералы в списках/кортежах внутри ``callback``."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == callback:
            out: set[str] = set()
            for sub in ast.walk(node):
                if isinstance(sub, (ast.List, ast.Tuple, ast.Set)):
                    for elt in sub.elts:
                        if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                            out.add(elt.value)
            return out
    raise AssertionError(f"{callback} не найден в {path.name}")


def test_tts_node_publishes_the_states_we_think_it_does() -> None:
    """Фиксируем вокабуляр продюсера — чтобы новое состояние заметили."""
    assert _published_states() == {
        "ready",
        "stopped",
        "synthesizing",
        "playing",
        "tts_silero_warming",
    }


@pytest.mark.parametrize(
    ("path", "callback"),
    [
        (_STT_NODE, "tts_state_callback"),
        (_AUDIO_NODE, "_on_tts_state"),
        (_ANIM_NODE, "tts_state_callback"),
    ],
)
def test_every_published_state_is_classified(path: Path, callback: str) -> None:
    """Ни одно состояние не должно проваливаться мимо обеих веток.

    Провалившееся состояние = «ничего не меняем», то есть подписчик
    залипает в предыдущем. Для ``stt_node`` это застрявший
    ``is_robot_speaking``, для ``audio_node`` — ``tts_active``.
    """
    known = _state_literals(path, callback)
    missing = _published_states() - known
    assert not missing, (
        f"{path.name}:{callback} не классифицирует состояния {sorted(missing)} — "
        "подписчик останется в предыдущем состоянии"
    )


def test_stopped_ends_speech_for_every_consumer() -> None:
    """``stopped`` (STOP/barge-in) обязан считаться концом речи везде."""
    for path, callback in (
        (_STT_NODE, "tts_state_callback"),
        (_AUDIO_NODE, "_on_tts_state"),
        (_ANIM_NODE, "tts_state_callback"),
    ):
        assert "stopped" in _state_literals(path, callback), (
            f"{path.name}:{callback} не знает 'stopped'; его публикует "
            "tts_node._handle_stop_command на каждый barge-in"
        )
