"""Tier-1 «говори нормально» (#4022): сброс скорости и тона кодом; соседние команды не затронуты."""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.agents import agent_spec, allows_tier1
from rob_box_dialog.grammar import Tier1Command, match
from rob_box_dialog.session import SceneFrame, SessionSnapshot


def _snap(**kw):
    return SessionSnapshot(agent="personality", session_id="t", epoch=0, started_at=0.0, **kw)


IDLE = _snap()
SILENCED = _snap(scenes=(SceneFrame("Silence", "user"),))
VOICES = [("Russian_ReliableMan", "Надёжный мужчина"), ("Russian_BrightHeroine", "Яркая героиня")]
NORMAL = Tier1Command(kn.PROSODY_NORMAL_KIND)


@pytest.mark.parametrize("text", [
    "Робот, говори нормально", "говори нормально", "Робот, говори с обычной скоростью", "говори как обычно",
    "Робот, говори нормальным темпом", "говори в обычном темпе", "говори с нормальной скоростью и тоном",
    "Робот, говори нормально, пожалуйста", "говори уже нормально", "робот, говори обычным тоном",
    "Робот, верни нормальную скорость речи", "Робот, читай как обычно", "говори со стандартной скоростью",
    "Говори нормально!",
])
def test_speak_normally_is_a_tier1_pace_reset(text):
    assert match(text, IDLE, VOICES) == NORMAL, text


@pytest.mark.parametrize("text", [
    "Робот, говори обычным голосом", "говори нормальным голосом", "верни обычный голос", "говори своим обычным голосом",
])
def test_ordinary_voice_is_still_the_voice_command_not_the_pace_reset(text):
    """«Обычным ГОЛОСОМ» (#3803) - голос робота, а не скорость: ``set_voice`` default, как и было."""
    assert match(text, IDLE, VOICES) == Tier1Command("set_voice", {"voice": kn.VOICE_DEFAULT_ARG}), text


@pytest.mark.parametrize("text", [
    "верни нормальную скорость",  # без речи: может быть скорость езды
    "поставь музыку в нормальном темпе", "играй как обычно", "расскажи как обычно", "нормально",
    "ты обычно говоришь быстро", "говори нормально и расскажи анекдот", "говори как пират",
    "говори нормально громче", "не говори нормально", "говори нормальным человеческим голосом",
    "говори", "говори быстрее", "говори медленнее", "расскажи про нормальную скорость",
])
def test_not_a_pace_reset(text):
    command = match(text, IDLE, VOICES)
    assert command is None or command.kind != kn.PROSODY_NORMAL_KIND, text


def test_speak_normally_is_not_a_volume_command():
    """«Нормально» в громкости значит «обычная громкость» - но только вместе со словом про громкость."""
    assert match("говори нормально", IDLE, VOICES).kind == kn.PROSODY_NORMAL_KIND
    assert match("громкость нормальная", IDLE, VOICES).kind == "volume"


def test_pace_reset_speaks_so_it_is_dropped_in_silence_and_belongs_to_the_voice_group():
    assert kn.TIER1_KINDS[kn.PROSODY_NORMAL_KIND] is True
    silenced = match("говори нормально", SILENCED, VOICES)  # «говори» в тишине - её снятие, а не сброс просодии
    assert silenced is None or silenced.kind != kn.PROSODY_NORMAL_KIND
    assert allows_tier1(agent_spec("personality"), kn.PROSODY_NORMAL_KIND)
    assert not allows_tier1(agent_spec("operator"), kn.PROSODY_NORMAL_KIND)
    assert not allows_tier1(agent_spec("telegram"), kn.PROSODY_NORMAL_KIND)
