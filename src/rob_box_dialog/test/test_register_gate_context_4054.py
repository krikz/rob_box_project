"""#4054 + #2925: контекст хода v2 для гейта ``register_speaker`` - кто говорит и как назвал себя.

Без этого гейт в v2 молчал (контекст нёс только ``utterance_id`` и ``heard_text``), и после #4054 «запомни, что Саша любит
чай» голосом Нины переименовало бы живой профиль «Нина» в «Саша». Здесь - решения кода, из которых dialog_node собирает
контекст (чистые, без ROS): ``speaker_adapter.confident_name`` (кем голос фразы УВЕРЕННО узнан) и
``speaker_adapter.named_self`` / ``words.self_intro_name`` (как человек назвал себя в реплике). Сборка контекста в ноде -
``rob_box_voice/test/unit/node/test_dialog_node.py``, гейт и переименование на проводе - ``rob_box_mcp_tools/test/
test_issue_4054_register_gate_v2.py``.
"""

import pytest

from rob_box_dialog.speaker_adapter import confident_name, named_self
from rob_box_dialog.words import introduces_self, self_intro_name

#: Результаты биометрии фразы (``rob_box_core.speaker_wire``), как их кладёт ``speaker_id_node``.
CONFIDENT = {"is_known": True, "speaker_id": "e35517b2", "name": "Нина", "confidence": 0.93}
TENTATIVE = {"is_known": True, "speaker_id": "e35517b2", "name": None, "tentative_name": "Нина",
             "tentative_kind": "single", "confidence": 0.77}
CONTESTED = {"is_known": True, "speaker_id": "e35517b2", "name": None, "tentative_name": "Нина",
             "tentative_kind": "contested", "confidence": 0.8}
NAMELESS = {"is_known": True, "speaker_id": "e35517b2", "name": None, "epithet": "Хмурый", "confidence": 0.9}
UNKNOWN = {"is_known": False}
INCONCLUSIVE = {"is_known": False, "inconclusive": True, "reason": "too_short"}


def test_voice_is_confident_only_when_biometry_named_it():
    """Имя в результате биометрии ставит владелец только при уверенном исходе (``classify_name_confidence`` =
    confident, порог ``name_confidence_band_high``) - та же полоса, что у переименования профиля в #4054."""
    assert confident_name(CONFIDENT) == "Нина"


@pytest.mark.parametrize("result", [TENTATIVE, CONTESTED, NAMELESS, UNKNOWN, INCONCLUSIVE, None, {}],
                         ids=["tentative", "contested", "nameless", "unknown", "inconclusive", "no-result", "empty"])
def test_voice_that_is_not_confident_is_nobody(result):
    """Гипотеза, спор, профиль без имени, неузнанный голос, фраза без оценки и «биометрии ещё нет» - не «уверенно»:
    гейт тогда молчит (как у v1), регистрацию разбирает ADR-0127."""
    assert confident_name(result) is None


def test_confident_name_without_a_profile_id_is_not_confident():
    """Битый результат (имя есть, профиля нет) - назвать некого: та же граница, что у ``voice_certainty``."""
    assert confident_name({"is_known": True, "name": "Нина"}) is None


@pytest.mark.parametrize("text, name", [
    ("запомни, что Саша любит чай", None),
    ("привет, я Тамара, мы с тобой ещё не знакомы, запомни мой голос", "тамара"),
    ("меня зовут Денчик", "денчик"),
    ("меня зовут Олег", "олег"),
    ("привет, я Аскар", "аскар"),
    ("зови меня Тамара", "Тамара"),
    ("запомни меня как Тамара", "Тамара"),
    ("меня зовут не Нина, а Тамара", "Тамара"),
    ("Запомни про меня: Дарья болит за Спартак", None),
    ("как меня зовут?", None),
    ("запомни мой голос", None),
    ("я думаю, что это Саша", None),
    ("расскажи сказку про Сашу", None),
], ids=lambda value: str(value)[:40])
def test_name_a_person_gave_himself_in_the_phrase(text, name):
    """Имя, которым человек назвал СЕБЯ, - из слов реплики (просьба звать иначе или представление); чужое имя в
    факте («Саша любит чай», «Дарья болит за Спартак») себе не даётся."""
    assert named_self(text) == name


@pytest.mark.parametrize("text", [
    "Робот, здравствуй, я Саша, чиню тут технику", "привет, я Борис", "робот это я, Валера", "меня зовут Денис",
    "привет я Аскар", "зови меня Шифу", "я думаю, что это правда", "я устал", "привет, я дома", "я Аскар",
    "робот, включи музыку", "как меня зовут",
])
def test_self_intro_name_is_exactly_what_introduces_self_finds(text):
    """Одна реализация: имя представления есть ровно там, где ``introduces_self`` говорит «представился» (ворота
    ``people`` в срезе тулов не поменялись)."""
    assert (self_intro_name(text) is not None) == introduces_self(text)
