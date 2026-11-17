"""Реплика робота для памяти контекста: SpeechRequest агента personality (dialog v2, issue #3911)."""

from types import SimpleNamespace

import pytest

from rob_box_perception.core.robot_speech import robot_said


def request(agent='personality', text='Привет', commit=True):
    return SimpleNamespace(agent=agent, text=text, commit=commit)


def test_robot_voice_request_is_said_text_without_speak_tags():
    assert robot_said(request(text='Привет, я Робби!')) == 'Привет, я Робби!'
    assert robot_said(request(text='Просто текст')) == 'Просто текст'


@pytest.mark.parametrize('agent', ['operator', 'telegram', 'system', ''])
def test_other_agents_are_not_the_robot_voice(agent):
    assert robot_said(request(agent=agent)) == ''


def test_cache_warmup_is_not_speech():
    assert robot_said(request(commit=False)) == ''


def test_empty_request_is_nothing():
    assert robot_said(request(text='  ')) == ''
