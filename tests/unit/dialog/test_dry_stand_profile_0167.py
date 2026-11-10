"""Сухой стенд робота берёт вейки из профиля робота и без него не стартует (ADR-0167 §2.4, шаг Ш1б).

Нейтральные умолчания ядра (вейк выключен, адресовано всё) стенду робота не годятся: он ловит ошибки адресации,
а на них каждая реплика марафона была бы «к роботу».
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

pytest.importorskip("openai", reason="сухому стенду нужен настоящий мост команд, а ему - клиент openai")

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "dialog"))

import dry_stand  # noqa: E402
from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog.profile import ProfileError, load_profile  # noqa: E402
from rob_box_llm.providers.fake import FakeLLMProvider  # noqa: E402
from rob_box_dialog_host.dialog_v2_ports import CallCounter  # noqa: E402


def _stand(tmp_path):
    return dry_stand.DryStand(FakeLLMProvider(), CallCounter(), "Ты робот.", tmp_path)


def test_stand_wake_is_the_robot_profile():
    assert dry_stand.ROBOT_PROFILE == REPO / "docker" / "sound" / "config" / "assistant.yaml"
    wake = dry_stand.robot_wake()
    assert wake == list(load_profile(dry_stand.ROBOT_PROFILE).wake_personality)
    personality, _ = kn.resolve_wake_word_namespaces(str(dry_stand.ROBOT_PROFILE.parent / "wake_words.yaml"))
    assert wake == personality and "робот" in wake


def test_stand_takes_its_wake_from_the_profile(tmp_path):
    assert _stand(tmp_path).wake == dry_stand.robot_wake()


def test_stand_without_robot_profile_does_not_start_with_neutral_wake(tmp_path, monkeypatch):
    monkeypatch.setattr(dry_stand, "ROBOT_PROFILE", tmp_path / "nope" / "assistant.yaml")
    with pytest.raises(ProfileError, match="файла нет"):
        _stand(tmp_path)
