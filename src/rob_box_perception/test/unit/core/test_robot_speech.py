"""Реплика робота для памяти контекста: SpeechRequest (dialog v2) и JSON dialogue_node v1 (issue #3911)."""

from types import SimpleNamespace

import pytest

from rob_box_perception.core.robot_speech import robot_said


def request(agent='personality', text='Привет', commit=True):
    return SimpleNamespace(agent=agent, text=text, commit=commit)


def v1(data):
    return SimpleNamespace(data=data)


def test_robot_voice_request_is_said_text_without_speak_tags():
    assert robot_said(request(text='<speak>Привет, я Робби!</speak>')) == 'Привет, я Робби!'
    assert robot_said(request(text='Просто текст')) == 'Просто текст'


@pytest.mark.parametrize('agent', ['operator', 'telegram', 'system', ''])
def test_other_agents_are_not_the_robot_voice(agent):
    assert robot_said(request(agent=agent)) == ''


def test_cache_warmup_is_not_speech():
    assert robot_said(request(commit=False)) == ''


def test_empty_request_is_nothing():
    assert robot_said(request(text='  ')) == ''


def test_v1_json_with_ssml():
    assert robot_said(v1('{"ssml": "<speak>Привет!</speak>", "speech_id": "x"}')) == 'Привет!'


def test_v1_plain_text_and_json_without_ssml():
    assert robot_said(v1('  Привет  ')) == 'Привет'
    assert robot_said(v1('{"emotion": "happy"}')) == ''
